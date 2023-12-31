#include "planner.hpp"

#include <functional>
#include <chrono>
#include <cmath>
#include <cassert>

using std::placeholders::_1;
namespace srv = std::ranges::views;
using sampling_t = std::tuple<Clipper2Lib::PathsD, std::vector<double>>;

#define __DEBUG
#ifdef __DEBUG

Clipper2Lib::PathD sample_arc(dubins::d_arc, uint_fast32_t);

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

/**
 * @brief creates a dump to visualize a dubins arc on desmos
 */
void arc_dump(auto path){
    for (uint32_t i = 0U; i < path.size() - 1; ++i){
        std::cout << "polygon((" << path[i].x << ", " << path[i].y << "), (";
        std::cout << path[i + 1].x << ", " << path[i + 1].y << "))\n";
    }
    std::cout << "---------------------------------------------\n";
}


/**
 * @brief creates a dump to visualize the dubins shortest path on desmos
 */
void dubins_dump(const auto& dpath){
    for (const auto& c : dpath){
        arc_dump(sample_arc(c.a1, 4));
        arc_dump(sample_arc(c.a2, 4));
        arc_dump(sample_arc(c.a3, 4));
    }
    std::cout << "---------------------------------------------\n";
}

#else

#define desmos_dump(x)
#define path_dump(x)
#define dubins_dump(x)

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
 * 
 * @note yes, this is duplicated code
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
 * @brief sample a point from a line (segment)
 */
inline Clipper2Lib::PointD lineline(double x0, double y0, double xf, double yf, double s){
    return { 
        x0 + s * (xf - x0), 
        y0 + s * (yf - y0),
    };
}

/**
 * @brief get yaw angle from a quaternion msg
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
 * @brief creates msg to send on topic /shelfino#/plan
 * 
 * @param fpath the path to send on the topic
 * @return the created msg
 * 
 * @todo debug
 */
nav_msgs::msg::Path Planner::getPathMsg(const Clipper2Lib::PathsD& fpath){

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";
    std::vector<geometry_msgs::msg::PoseStamped> posesTemp{ fpath[0].size() + fpath[1].size() + fpath[2].size() };

    assert(0 && "TODO");

    return path;
}

/**
 * @brief sample a dubins arc
 * 
 * @param arc the dubins arc to sample
 * @param n_samples fast number of samples needed. 
 *      "How can it be this fast!?" 
 *      "It just is."
 */
Clipper2Lib::PathD sample_arc(dubins::d_arc arc, uint_fast32_t n_samples = 30){
    Clipper2Lib::PathD v{ n_samples + 1 };

    // &p - &v[0] is a nice hack to get the element idx without race conditions
    std::for_each(std::execution::par_unseq, v.begin(), v.end(), [&](auto& p){
        double s = arc.L / n_samples * (&p - &v[0]);
        p = circline(arc.x0, arc.y0, arc.th0, s, arc.k);
    });

    return v;
}

/**
 * @brief sample a line (segment)
 * 
 * @param line the line to sample
 * @param n_samples number of samples needed. 
 */
Clipper2Lib::PathD sample_line(dubins::d_arc line, uint32_t n_samples = 30){
    Clipper2Lib::PathD v{ n_samples + 1 };

    // &p - &v[0] is a nice hack to get the element idx without race conditions
    std::for_each(std::execution::par_unseq, v.begin(), v.end(), [&](auto& p){
        double s = 1. / n_samples * (&p - &v[0]);
        p = lineline(line.x0, line.y0, line.xf, line.yf, s);
    });

    return v;
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
 * @brief callbacks to gather the necessary data, a latch is used to sync with the planning phase
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
 * @brief samples the path in segments that are long l
 * 
 * @param p path to sample
 * @param l length of the segments to sample (except for the last one of each arc due to remainder)
 * @param s_curves number of starting curves before safe path
 * @param e_curves number of ending curves after safe path
 * @return the sampled points, and the length of the path from the start to that point
 * 
 * @note s_curves and e_curves are here in case we find a good sampling way for intermediate points
 * in the first and last trait 
 * @todo better handling of ls
 */
sampling_t sample_path(const multi_dubins::path_t& p, double l, uint64_t s_curves = 1, uint64_t e_curves = 1){
    Clipper2Lib::PathsD out{ 3 };
    std::vector<double> ls;
    double curr_l = 0;

    // lambdas are a nice way to avoid duplicated code
    auto aux_sample_arc = [&](const auto& arc, size_t sz, int32_t i){
        for (size_t j = 1; j <= sz; ++j){
            double s = arc.L / sz * j;
            out[i].push_back(circline(arc.x0, arc.y0, arc.th0, s, arc.k));
            ls.push_back(curr_l + l * j);
        } 
    };

    auto aux_sample_line = [&](const auto& line, size_t sz, int32_t i){
        for (size_t j = 1; j <= sz; ++j){
            double s = 1. / sz * j;
            out[i].push_back(lineline(line.x0, line.y0, line.xf, line.yf, s));
            ls.push_back(curr_l + l * j);
        } 
    };

    out.back().push_back({ p.front().a1.x0, p.front().a1.y0 });
    ls.push_back(curr_l);

    for (size_t i = 0; i < s_curves; ++i){
        aux_sample_arc(p[i].a1, p[i].a1.L / l, 0);
        curr_l += p[i].a1.L;
        p[i].a2.th0 == p[i].a2.thf ? aux_sample_line(p[i].a2, p[i].a2.L / l, 0) : aux_sample_arc(p[i].a2, p[i].a2.L / l, 0);
        curr_l += p[i].a2.L;
        aux_sample_arc(p[i].a3, p[i].a3.L / l, 0);
        curr_l += p[i].a3.L;
    }

    for (size_t i = s_curves; i < p.size() - e_curves; ++i){
        aux_sample_arc(p[i].a1, p[i].a1.L / l, 1);
        curr_l += p[i].a1.L;
        p[i].a2.th0 == p[i].a2.thf ? aux_sample_line(p[i].a2, p[i].a2.L / l, 1) : aux_sample_arc(p[i].a2, p[i].a2.L / l, 1);
        curr_l += p[i].a2.L;
        aux_sample_arc(p[i].a3, p[i].a3.L / l, 1);
        curr_l += p[i].a3.L;
    }

    
    for (size_t i = p.size() - e_curves; i < p.size(); ++i){
        aux_sample_arc(p[i].a1, p[i].a1.L / l, 2);
        curr_l += p[i].a1.L;
        p[i].a2.th0 == p[i].a2.thf ? aux_sample_line(p[i].a2, p[i].a2.L / l, 2) : aux_sample_arc(p[i].a2, p[i].a2.L / l, 2);
        curr_l += p[i].a2.L;
        aux_sample_arc(p[i].a3, p[i].a3.L / l, 2);
        curr_l += p[i].a3.L;
    }

    return { out, ls };
}

/**
 * @brief check if a given dubins path is collision free using clipper, this method is very general as it
 * could potentially work with any type of trajectory. The idea is to approximate curves using line segments 
 * and the do clipping using the enviroment as a clip and the trajectory as an open subject
 * 
 * @note under the assumption of rectangles as borders we could use RectClip which is more efficient
 * @todo test and, if we find a good sampling heuristic, try to clip multiple curves at once for optimization
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
    cd.AddClip(Clipper2Lib::PathsD{ env.begin() + 1, env.end() });
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
 * @brief get a safe (collision free) dubins curve to go from a to b using clipper2, if not found
 * just shutdown, error handling is pretty useless at this point (there's no path anyway)
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

    for (const auto& c : paths)
        if (is_collision_free(c, min_env_)) return c;

    // TODO: sample a good point to try and find a new feasible path
    RCLCPP_ERROR(this->get_logger(), "Could not find a feasible path using dubins curves to go from (%lf, %lf) to (%lf, %lf)!\n",
        a.x(), a.y(), b.x(), b.y());
    rclcpp::shutdown();
    exit(1);
}

/**
 * @brief generates the collision safe curve as described in the project report
 * 
 * @param a starting point
 * @param b intersection point of the two edges
 * @param c ending point of the third edge
 * @param [out] new_a ending point of dubins curve
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

    double normf = sqrt(vxf * vxf + vyf * vyf);
    double unitxf = vxf / normf;
    double unityf = vyf / normf;

    /*
     * |A·B| = |A| |B| cos(θ)
     * |A×B| = |A| |B| sin(θ)
     * with this we can easily get the angle between the two vectors
     * we add fabs to normalize in [0, pi)
     * M_PI - angle cause _/ -> /_ 
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
    assert(curve.a1.x0 - curve.a2.x0 < 1e-12 && curve.a1.y0 - curve.a2.y0 < 1e-12); // the epsilon is actually needed
#endif
    return curve;
}

/**
 * @brief create the dubins_path following the proposed strategies 
 * 
 * @todo angle case 3
 */
multi_dubins::path_t Planner::dubins_path(const VisiLibity::Polyline& path, double th0, double thf, double r){
    double th;
    switch (path.size()){
    case 2:
        return { sample_curve(path[0], th0, path[1], thf, 1. / min_r) };
    case 3:
        // note th as intermediate angle might not always be the best but it looks like 
        // a decent choice for O(1) time algorithm
        th = atan2(path[2].y() - path[1].y(), path[2].x() - path[1].x());
        return { 
            sample_curve(path[0], th0, path[1], th, 1. / min_r),
            sample_curve(path[1], th, path[2], thf, 1. / min_r)
        };
    case 4:
        th = atan2(path[2].y() - path[1].y(), path[2].x() - path[1].x());
        dubins::d_curve safe_curve;
        dubins::d_shortest(path[1].x(), path[1].y(), th, path[2].x(), path[2].y(), th, 1. / min_r, safe_curve);
        return {
            sample_curve(path[0], th0, path[1], th, 1. / min_r),
            safe_curve, // always a straight line, in line with our intermediate curves strategy
            sample_curve(path[2], th, path[3], thf, 1. / min_r)
        };
    // default:
    }

    // general case
    multi_dubins::path_t sol{ path.size() - 1 };
    VisiLibity::Point new_a = path[1];
    for (uint64_t i = 1; i < path.size() - 2; ++i)
        sol[i] = get_safe_curve(new_a, path[i + 1], path[i + 2], new_a, r);

    /*
    // NOTE: The alternative method
    // manually construct the last safe segment as a safe line instead that as a line + arc
    // it's non-trivial to choose which one is best
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
    sol.front() = sample_curve(path[0], th0, path[1], sol[1].a1.th0, 1. / min_r);
    sol.back() = sample_curve(new_a, sol[sol.size() - 2].a3.thf, path[path.size() - 1], thf, 1. / min_r);

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
    double th0 = get_yaw(pos0_.orientation);
    double th1 = get_yaw(pos1_.orientation);
    double th2 = get_yaw(pos2_.orientation);
    double thg = get_yaw(gate_msg_.poses[0].orientation); 
    // thg *= -1; // IMHO it's a bug that sometimes it gives the opposite angle (opposite to the wall)

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
        // Can't wait for c++26 to allow constexpr cmath functions (:
        constexpr double sec30 = 1.1547005383792515290182975610039149112952035025402537520372046529;
        constexpr double cos60 = .5;
        constexpr double sin60 = 0.8660254037844386467637231707529361834714026269051903140279034897;

        // circles are approximated to hexagons
        if (ob.radius){
            const double r = ob.radius;
            const double xc = ob.polygon.points[0].x;
            const double yc = ob.polygon.points[0].y;
            // closed solution for the hexagon
            return Clipper2Lib::PathD({
                {xc + r * sec30, yc},
                {xc + r * sec30 * cos60, yc - r * sec30 * sin60},
                {xc - r * sec30 * cos60, yc - r * sec30 * sin60},
                {xc - r * sec30, yc},
                {xc - r * sec30 * cos60, yc + r * sec30 * sin60},
                {xc + r * sec30 * cos60, yc + r * sec30 * sin60}
            });
        }

        Clipper2Lib::PathD polygon(ob.polygon.points.size());
        for (auto i = 0UL; i < ob.polygon.points.size(); ++i)
            polygon[i] = Clipper2Lib::PointD(ob.polygon.points[i].x, ob.polygon.points[i].y);
        return polygon;
    });
    
    desmos_dump(clipper_obs);

    // get the offsetted enviroment + a minimum offset version of the obstacles used for collision detection when needed
    min_env_ = offsetting::offset_minimum(clipper_border, clipper_obs, hrobot_sz);
    auto aux = offsetting::offset_env(clipper_border, clipper_obs, hrobot_sz, min_r);

    desmos_dump(min_env_);
    desmos_dump(aux);

    // generating VisiLibity enviroment
    std::vector<VisiLibity::Polygon> visilibity_env(aux.size());
    std::transform(std::execution::par_unseq, aux.begin(), aux.end(), visilibity_env.begin(), [](const auto& p){
        VisiLibity::Polygon ptmp;
        for (auto i = p.rbegin(); i != p.rend(); ++i)
            ptmp.push_back(VisiLibity::Point(i->x, i->y));
        return ptmp;
    });
    env_ = VisiLibity::Environment(visilibity_env);
    assert(env_.is_valid(e)); // if this fails it might be needed to reduce e (cause 0.001 is fairly big)

    // visibility graph
    VisiLibity::Visibility_Graph g(env_, e);
    // NOTE: it is assumed that the center of the robot is at least .5m from the obstacles for this to work properly
    // this should be a reasonable request as the robot ha a radius of ~.4m (.35 + .05 for safety) 
    // (it might still work even if this is not satisfied)
    /*
    // to check if it's really ok use (the gate should always be fine)
    robot0.in(env, e);
    robot1.in(env, e);
    robot2.in(env, e);
    */
    auto short_p0 = env_.shortest_path(robot0, gate, g, e);
    auto short_p1 = env_.shortest_path(robot1, gate, g, e);
    auto short_p2 = env_.shortest_path(robot2, gate, g, e);

    path_dump(short_p0);
    path_dump(short_p1);
    path_dump(short_p2);

    // get the path made out of dubin curves
    multi_dubins::path_t p0;
    multi_dubins::path_t p1;
    multi_dubins::path_t p2;
    /*
     * NOTE: this could easily be done by three separated threads but idk if the libraries are thread safe
     */
    p0 = dubins_path(short_p0, th0, thg, min_r);
    p1 = dubins_path(short_p1, th1, thg, min_r);
    p2 = dubins_path(short_p2, th2, thg, min_r);
    
    dubins_dump(p0);
    dubins_dump(p1);
    dubins_dump(p2);

    double l = .15; // good enough?
    // int32_t s0, e0, s1, e1, s2, e2;
    auto s_plus_l0 = sample_path(p0, l, 1, 1);
    auto s_plus_l1 = sample_path(p1, l, 1, 1);
    auto s_plus_l2 = sample_path(p2, l, 1, 1);
    // handy tuple unpack feature from c++17
    const auto& [samples0, ls0] = s_plus_l0;
    const auto& [samples1, ls1] = s_plus_l1;
    const auto& [samples2, ls2] = s_plus_l2;

#ifdef __DEBUG
    for (const auto& v : samples0)
        for (const auto& p : v)
            std::cout << "(" << p.x << ", " << p.y << ")\n";
    
    for (const auto l : ls0)
        std::cout << l << '\n';

    for (const auto& v : samples1)
        for (const auto& p : v)
            std::cout << "(" << p.x << ", " << p.y << ")\n";

    for (const auto l : ls1)
        std::cout << l << '\n';

    for (const auto& v : samples2)
        for (const auto& p : v)
            std::cout << "(" << p.x << ", " << p.y << ")\n";
    
    for (const auto l : ls2)
        std::cout << l << '\n';
#endif
}
