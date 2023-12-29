#include <functional>
#include <cassert>
#include "offsetting.hpp"
#include "execution"

using std::placeholders::_1;

/**
 * @brief  formula finds the distance between a circle and the intersection point of
 * two segments assuming the circle is tangent to both of them (distance + half of the robot size)
 * The real (intuitive) formula is off + r * excsc(alpha / 2) but excsc is not in cmath :(
 * 
 * @param off robot offset
 * @param r inverse of maximum curvature
 * @param alpha angle formed by the two segments
 * @return actually needed offset
 * 
 * @note in the end this was not used but it's still cool
inline constexpr double off_plus_excsc(double off, double r, double alpha){
    return off + r * (1. / sin(alpha / 2.) - 1.);
}
 */

/**
 * @brief adds the necessary offset to the given enviroment using Miter as a join type 
 * then clips the result to remove overlapping of objects
 * 
 * @param border map border
 * @param ob array of obstacles
 * @param robot_r radius of the robot (unused for now but might be useful someday)
 * @param min_r minimum curvature radius
 * @return the enviroment obtained by offsetting + clipping
 * 
 * @note using min_r with Miter might cause problems with angles which are less than 30 degrees (but in our use case 
 * this cannot happen) a nice workaround would be to approximate the object to it's oriented bounding box or convex hull
 */
Clipper2Lib::PathsD offsetting::offset_env(const Clipper2Lib::PathsD& border, const Clipper2Lib::PathsD& ob, [[maybe_unused]] double robot_r, double min_r){
    auto border_off = Clipper2Lib::InflatePaths(border, -min_r, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon);
    auto ob_off = Clipper2Lib::InflatePaths(ob, min_r, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon, 2.1);
    Clipper2Lib::ClipperD cd;
    Clipper2Lib::PathsD p;
    cd.AddClip(ob_off);
    cd.AddSubject(border_off);
    cd.Execute(Clipper2Lib::ClipType::Difference, Clipper2Lib::FillRule::NonZero, p);

    // we need to do this crap cause for some reason VisiLibity cannot stand obstacles sharing a vertex with the border!?!?
    cd.Clear();
    Clipper2Lib::PathsD border_aux = { p[0] };
    p.erase(p.begin());
    cd.AddClip(border_aux);
    cd.AddSubject(p);
    cd.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::NonZero, p);
    p.emplace(p.begin(), border_aux[0].rbegin(), border_aux[0].rend());

    return p;
}

/**
 * @brief specialization of offset_env where the returned enviroment is the minimum env to ensure safety
 */
Clipper2Lib::PathsD offsetting::offset_minimum(const Clipper2Lib::PathsD& border, const Clipper2Lib::PathsD& ob, double robot_r){
    return offset_env(border, ob, .0, robot_r);
}
