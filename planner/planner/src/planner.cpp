#include "planner.hpp"

#include <functional>
#include <chrono>
#include <cmath>
#include <cassert>

#define __DEBUG

using std::placeholders::_1;

#ifdef __DEBUG
/**
 * @brief creates a dump to visualize the clipper enviroment on desmos
 */
void desmos_dump(auto clipper_env){
    for (const auto& s : clipper_env){
        std::cout << "polygon(";
        for (const auto t : s)
            std::cout << "(" << t.x << ", " << t.y << "), ";
        std::cout << "\b\b)\n";
    }
    std::cout << "---------------------------------------------\n";
}

/**
 * @brief creates a dump to visualize the shortest path on desmos
 */
void path_dump(auto path){
    for (uint32_t i = 0U; i < path.size() - 1; ++i){
        std::cout << "polygon((" << path[i].x() << ", " << path[i].y() << "), (";
        std::cout << path[i + 1].x() << ", " << path[i + 1].y() << "))\n";
    }
    std::cout << "---------------------------------------------\n";
}
#endif

/**
 * @brief branchless implementation of the sgn function
 * 
 * @note the concept is used to ensure that this works only on the expected types
 */
template <typename T>
requires requires { std::is_arithmetic_v<T>; }
inline constexpr T sgn(T x){
    return (x > 0) - (x < 0);
}

/**
 * @brief numerically stable implementation of the sinc function
 */
inline constexpr double sinc(double x){
    if (fabs(x) > 0.002)
        return sin(x) / x;
    return 1 - x * x / 6 * (1 - x * x / 20);
}

/**
 * @brief sample a point from a dubins arc
 */
inline Clipper2Lib::PointD circline(double x, double y, double th, double s, double k){
    double sin, cos;
    sincos(th + k * s / 2., &sin, &cos);
    return {
        x + s * sinc(k * s / 2.) * cos,
        y + s * sinc(k * s / 2.) * sin,
    };
}

/**
 * @brief sample a dubins arc to check intersections using clipper2
 */
Clipper2Lib::PathD sample_arc(dubins::d_arc arc){
    static constexpr uint_fast32_t n_samples = 30; // should always be good enough
    Clipper2Lib::PathD v{ n_samples };

    std::for_each(std::execution::par_unseq, v.begin(), v.end(), [=](auto& p){
        for (uint32_t i = 0; i < n_samples; ++i){
            double s = arc.L / n_samples * i;
            p = circline(arc.x0, arc.y0, arc.th0, s, arc.k);
        }
    });

    return v;
}

/**
 * @brief get w angle from a quaternion msg
 */
inline double get_yaw(const auto& orientation){
    tf2::Quaternion q;
    double garbage_r, garbage_p;
    double th;

    q.setX(orientation.x);
    q.setY(orientation.y);
    q.setZ(orientation.z);
    q.setW(orientation.w);
    // q.normalize(); // should already be normalized
    tf2::Matrix3x3(q).getRPY(garbage_r, garbage_p, th);
    return th;
}

/**
 * @brief constructor of the Planner class
 */
Planner::Planner() : Node("planner_node"){
    pub0_ = this->create_publisher<nav_msgs::msg::Path>("shelfino0/plan", 10);
    pub1_ = this->create_publisher<nav_msgs::msg::Path>("shelfino1/plan", 10);
    pub2_ = this->create_publisher<nav_msgs::msg::Path>("shelfino2/plan", 10);

    static constexpr rmw_qos_profile_t rmw_qos_profile_custom = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10,
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false
    };
    static const auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_group_;

    obstacles_sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "obstacles", custom_qos, std::bind(&Planner::obstacles_callback, this, _1), options);
    borders_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        "map_borders", custom_qos, std::bind(&Planner::borders_callback, this, _1), options);
    gate_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "gate_position", custom_qos, std::bind(&Planner::gate_callback, this, _1), options);
    sub0_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "shelfino0/amcl_pose", custom_qos, std::bind(&Planner::sub0_callback, this, _1), options);
    sub1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "shelfino1/amcl_pose", custom_qos, std::bind(&Planner::sub1_callback, this, _1), options);
    sub2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "shelfino2/amcl_pose", custom_qos, std::bind(&Planner::sub2_callback, this, _1), options);
}

/**
 * @brief callbacks to gather the necessary data 
 */
///@{
void Planner::obstacles_callback(obstacles_msgs::msg::ObstacleArrayMsg msg){
    obstacles_msg_ = std::move(msg);
    data_.count_down();
    obstacles_sub_.reset();
}

void Planner::borders_callback(geometry_msgs::msg::Polygon msg){
    borders_msg_ = std::move(msg);
    data_.count_down();
    borders_sub_.reset();
}

void Planner::gate_callback(geometry_msgs::msg::PoseArray msg){
    gate_msg_ = std::move(msg);
    data_.count_down();
    gate_sub_.reset();
}

void Planner::sub0_callback(geometry_msgs::msg::PoseWithCovarianceStamped msg){
    pos0_ = std::move(msg.pose.pose);
    data_.count_down();
    sub0_.reset();
}

void Planner::sub1_callback(geometry_msgs::msg::PoseWithCovarianceStamped msg){
    pos1_ = std::move(msg.pose.pose);
    data_.count_down();
    sub1_.reset();
}

void Planner::sub2_callback(geometry_msgs::msg::PoseWithCovarianceStamped msg){
    pos2_ = std::move(msg.pose.pose);
    data_.count_down();
    sub2_.reset();
}
///@}

/**
 * @brief check if a given dubins path is collision free using clipper, this method is very general as it
 * could potentially work with any type of trajectory. The idea is to approximate curves using line segments 
 * and the do clipping using the enviroment as a clip and the trajectory as an open subject
 * 
 * @todo test
 */
bool is_collision_free(const dubins::d_curve& c, const Clipper2Lib::PathsD& env){
    static constexpr double e = 1e-6;
    Clipper2Lib::PathsD polylines, garbage, intersections;
    if (c.a1.L > e) polylines.push_back(sample_arc(c.a1));
    if (c.a3.L > e) polylines.push_back(sample_arc(c.a3));
    if (c.a2.L > e) polylines.push_back(
        (c.a2.th0 - c.a2.thf < e ? Clipper2Lib::PathD({ {c.a2.x0, c.a2.y0}, {c.a2.xf, c.a2.yf} }) : sample_arc(c.a2))
    );

    Clipper2Lib::ClipperD cd;
    cd.AddClip({ env.begin() + 1, env.end() });
    cd.AddOpenSubject(polylines);
    cd.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::NonZero, garbage, intersections);
    if (intersections.size())
        return false;
    cd.Clear();
    cd.AddClip({ { env[0] } });
    cd.AddOpenSubject(polylines);
    cd.Execute(Clipper2Lib::ClipType::Difference, Clipper2Lib::FillRule::NonZero, garbage, intersections);
    return !intersections.size();
}

/**
 * @brief get a safe (collision free) dubins curve to go from a to b using clipper2
 * 
 * @param a starting point
 * @param th0 starting angle
 * @param b ending point
 * @param thf ending angle
 * @param Kmax maximum curvature
 * @return the safe dubins curve
 * 
 * @note to do this we check for collision every possible dubins path from a to b
 * @todo sample a new point using a decent euristic to find a good path like 
 * a -> intermediate_point -> b
 */
dubins::d_curve Planner::sample_curve(VisiLibity::Point a, double th0, VisiLibity::Point b, double thf, double Kmax){
    auto paths = dubins::d_paths(a.x(), a.y(), th0, b.x(), b.y(), thf, Kmax);
    std::cerr << paths.size() << '\n';

    for (const auto& c : paths)
        if (is_collision_free(c, min_env)) return c;

    // TODO: sample a good point to try and find a new feasible path
    assert(0 && "No feasible path found!");
}

/**
 * @brief generates the collision safe curve as described in the project report
 * 
 * @param a starting point
 * @param b intersection point of the two edges
 * @param c ending point of the third edge
 * @param r minmum curvature radius
 * @return the collision safe dubins curve
 */
dubins::d_curve Planner::get_safe_curve(VisiLibity::Point a, VisiLibity::Point b, VisiLibity::Point c, VisiLibity::Point& new_a, double r){
    dubins::d_curve curve;

    double vx0 = b.x() - a.x();
    double vy0 = b.y() - a.y();
    double th0 = atan2(vy0, vx0); // * sgn(vy0);

    double vxf = c.x() - b.x();
    double vyf = c.y() - b.y();
    double thf = atan2(vyf, vxf); // * sgn(vyf);

    /*
    norm0...
     */
    double normf = sqrt(vxf * vxf + vyf * vyf);
    double unitxf = vxf / normf;
    double unityf = vyf / normf;

    /*
     * |A·B| = |A| |B| cos(θ)
     * |A×B| = |A| |B| sin(θ)
     * with this we can easily get the angle between the two vectors
     * we add fabs to normalize in [0, pi)
     */
    double alpha = M_PI - atan2(fabs(vx0 * vyf - vy0 * vxf), vx0 * vxf + vy0 * vyf);
    double sina, cosa;
    sincos(alpha / 2., &sina, &cosa);
    double d = r * (cosa / sina);
    double xf = b.x() + d * unitxf;
    double yf = b.y() + d * unityf;
    new_a.set_x(xf);
    new_a.set_y(yf);

    // this is slower than actually creating the curve manually
    [[maybe_unused]] auto ok = dubins::d_shortest(a.x(), a.y(), th0, xf, yf, thf, 1. / r, curve);
#ifdef __DEBUG
    assert(ok != -1); // should be fine (by construction)
    assert(curve.a1.x0 - curve.a2.x0 < 1e-16 && curve.a1.y0 - curve.a2.y0 < 1e-16); // the epsilon is actually needed
#endif
    return curve;
}

/**
 * @brief create the dubins_path following the proposed strategies 
 * 
 * @todo angle case 3
 */
multi_dubins::path_t Planner::dubins_path(const VisiLibity::Polyline& path, double th0, double thf, double r){
    multi_dubins::path_t sol{ path.size() - 2 };

    switch (path.size()){
    case 2:
        return { sample_curve(path[0], th0, path[1], thf, 1. / min_r) };
    case 3:
        return {};
    case 4:
        return {};
    // default:
    }

    // general case
    VisiLibity::Point new_a = path[1];
    for (uint64_t i = 1; i < path.size() - 2; ++i){
        std::cout << "(" << new_a.x() << ", " << new_a.y() << "), ";
        std::cout << "(" << path[i + 1].x() << ", " << path[i + 1].y() << "), ";
        std::cout << "(" << path[i + 2].x() << ", " << path[i + 2].y() << ")\n";
        sol[i] = get_safe_curve(new_a, path[i + 1], path[i + 2], new_a, r);
    }

    /*
    // NOTE: only iff we use the alternative method
    // manually construct the last safe segment (always a straight line)
    const auto& tmp = path[path.size() - 2];
    const double th = sol[sol.size() - 3].a3.thf;
    double last_l = sqrt((new_a.x() - tmp.x()) * (new_a.x() - tmp.x()) + (new_a.y() - tmp.y()) * (new_a.y() - tmp.y()));
    sol[sol.size() - 2] = {
        {new_a.x(), new_a.y(), th, 1 / r, 0, new_a.x(), new_a.y(), th}, 
        {new_a.x(), new_a.y(), th, 1 / r, last_l, tmp.x(), tmp.y(), th}, 
        {tmp.x(), tmp.y(), th, 1 / r, 0, tmp.x(), tmp.y(), th}, 
        last_l
    };
    */
    // sol.front() = sample_curve();
    // sol.end() = sample_curve();

#ifdef __DEBUG
    std::cout << "------------------------------------------------------------------------------------------\n";
    for (auto i =  1UL; i < path.size() - 2; ++i){
        std::cout << sol[i].a1.x0 << " " << sol[i].a1.y0 << " " << sol[i].a1.th0 << " " << sol[i].a1.L << "\n";
        std::cout << sol[i].a2.x0 << " " << sol[i].a2.y0 << " " << sol[i].a2.th0 << " " << sol[i].a2.L << "\n";
        std::cout << sol[i].a3.x0 << " " << sol[i].a3.y0 << " " << sol[i].a3.th0 << " " << sol[i].a3.L << "\n";
        std::cout << sol[i].a3.xf << " " << sol[i].a3.yf << " " << sol[i].a3.thf << " " << "\n"; 
        std::cout << "------------------------------------------------------------------------------------------\n";
    }
#endif

    return sol;
}

/*
// TODO
VisiLibity::Polyline my_BAstar(VisiLibity::Environment env, VisiLibity::Visibility_Graph g,
    VisiLibity::Point start, VisiLibity::Point end, double e){
    VisiLibity::Polyline short_p;

    // trivial case
    VisiLibity::Visibility_Polygon start_vp(start, env, e);
    if (end.in(start_vp, e))
        return VisiLibity::Polyline({start, end});

    VisiLibity::Visibility_Polygon end_vp(end, env, e);
    std::vector
}
*/

/**
 * @brief function that generates the path planning for the 3 robots using, when possible, parallel execution
 * 
 * @note given that the number of obstacles and points can be rather small, using multithreading might
 * be slower than a sequential execution but it allows better scalability (and gains in perfomance when it matters)
 * @todo check if tinygeom2d is faster than VisiLibity
 */
void Planner::plan(){
    // chosen epsilon for enviroment check
    static constexpr double e = 0.001;
    // wait for the needed data
    data_.wait();

    // read the necessary data
    // read positions
    VisiLibity::Point robot0{pos0_.position.x, pos0_.position.y};
    VisiLibity::Point robot1{pos1_.position.x, pos1_.position.y};
    VisiLibity::Point robot2{pos2_.position.x, pos2_.position.y};
    VisiLibity::Point gate{gate_msg_.poses[0].position.x, gate_msg_.poses[0].position.y};
    [[maybe_unused]] double th0 = get_yaw(pos0_.orientation);
    [[maybe_unused]] double th1 = get_yaw(pos1_.orientation);
    [[maybe_unused]] double th2 = get_yaw(pos2_.orientation);
    [[maybe_unused]] double thg = get_yaw(gate_msg_.poses[0].orientation); // ???
    thg = M_PI / 2.;

    // generating Clipper data
    // Clipper vector of polygons to offset
    Clipper2Lib::PathD clipper_border_aux{borders_msg_.points.size()};
    Clipper2Lib::PathsD clipper_obs{obstacles_msg_.obstacles.size()};
    // insert the map borders 
    std::transform(borders_msg_.points.begin(), borders_msg_.points.end(), clipper_border_aux.begin(), [](const auto p){
        return Clipper2Lib::PointD(p.x, p.y);
    });
    Clipper2Lib::PathsD clipper_border{1};
    clipper_border[0] = std::move(clipper_border_aux);

    // insert (in a parallel way) the obstacles
    std::transform(std::execution::par_unseq, obstacles_msg_.obstacles.begin(), obstacles_msg_.obstacles.end(), 
        clipper_obs.begin(), [](const auto& ob){
        Clipper2Lib::PathD polygon(ob.polygon.points.size());
        for (auto i = 0UL; i < ob.polygon.points.size(); ++i)
            polygon[i] = Clipper2Lib::PointD(ob.polygon.points[i].x, ob.polygon.points[i].y);
        return polygon;
    });
    
#ifdef __DEBUG
    desmos_dump(clipper_obs);
#endif
    // get the offsetted enviroment + a minimum offset version of the obstacles used for collision detection when needed
    min_env = offsetting::offset_minimum(clipper_border, clipper_obs, hrobot_sz);
    auto aux = offsetting::offset_env(clipper_border, clipper_obs, hrobot_sz, min_r);
#ifdef __DEBUG
    desmos_dump(min_env);
    desmos_dump(aux);
#endif

    // generating VisiLibity enviroment
    std::vector<VisiLibity::Polygon> visilibity_env(aux.size());
    std::transform(std::execution::par_unseq, aux.begin(), aux.end(), visilibity_env.begin(), [](const auto& p){
        VisiLibity::Polygon ptmp;
        for (auto i = p.rbegin(); i != p.rend(); ++i)
            ptmp.push_back(VisiLibity::Point(i->x, i->y));
        return ptmp;
    });
    VisiLibity::Environment env(visilibity_env);
    assert(env.is_valid(e)); // if this fails it might be needed to reduce e

    // visibility graph
    VisiLibity::Visibility_Graph g(env, e);
    auto short_p0 = env.shortest_path(robot0, gate, g, e);
    auto short_p1 = env.shortest_path(robot1, gate, g, e);
    auto short_p2 = env.shortest_path(robot2, gate, g, e);
#ifdef __DEBUG
    path_dump(short_p0);
    path_dump(short_p1);
    path_dump(short_p2);
#endif

    // get the path made out of dubin curves
    multi_dubins::path_t p0;
    multi_dubins::path_t p1;
    multi_dubins::path_t p2;

    p0 = dubins_path(short_p0, th0, thg, min_r);
    std::cout << p0[0].a1.x0 << " " << p0[0].a1.y0 << '\n';
    std::cout << p0[0].a2.x0 << " " << p0[0].a2.y0 << '\n';
    std::cout << p0[0].a3.x0 << " " << p0[0].a3.y0 << '\n';
    std::cout << p0[0].a3.xf << " " << p0[0].a3.yf << '\n';
}
