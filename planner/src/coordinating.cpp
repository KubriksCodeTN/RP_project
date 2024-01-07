#include <execution>
#include <ranges>
#include "coordinating.hpp"

using std::placeholders::_1;

/**
 * @brief Euclidean distance (squared)
 */
inline constexpr double d(const auto& p0, const auto& p1){
    return (p0.x - p1.x) * (p0.x - p1.x) + (p0.y - p1.y) * (p0.y - p1.y);
}

/**
 * @brief get possible collisions between two paths
 * 
 * @todo try use an execution policy to optimize (maybe ignore long list of collision?)
 */
std::vector<coordinating::collision_t> get_collisions(const coordinating::sampling_t& path1, const coordinating::sampling_t& path2, 
    double robot_r){
    // points, lengths 
    const auto& [ps1, ls1] = path1;
    const auto& [ps2, ls2] = path2;

    // NOPE! c++23 needed :(
    // auto it1 = std::views::zip(ps1, ls1);

    std::vector<coordinating::collision_t> out;

    for (size_t i = 0; i < ps1.size(); ++i)
        for (size_t j = 0; j < ps2.size(); ++j)
            if (d(ps1[i], ps2[j]) <= 4 * robot_r * robot_r)
                out.emplace_back(ps1[i], ps2[j], ls1[i], ls2[j]);

    return out;
}

/**
 * @brief check if the potential collision is an actual collision (ie the robots go there at the same time)
 * (equal velocities <-> equal length == equal time)
 */
inline constexpr bool same_time(const coordinating::collision_t& c, double off_t1, double off_t2, double robot_sz, double safety_off){
    const auto& t1 = get<2>(c);
    const auto& t2 = get<3>(c);
    return std::abs((t1 + off_t1) - (t2 + off_t2)) <= robot_sz + safety_off;
}

/**
 * @brief find the time that the robots need to wait in order to avoid collisions, sadly we have little room
 * for optimization at the moment cause we cannot assume to take the shortest path and we cannot stop once we start
 * 
 * @note this algorithm is generalizable to n robots so this could work even with more than 3 shelfinos
 */
coordinating::triplet_t coordinating::coordinate(const coordinating::sampling_t& path1, const coordinating::sampling_t& path2, 
    const coordinating::sampling_t& path3, double robot_r, double safety_off){
    // by default we should wait enough for a collision to be avoided for sure
    const double wait_time = 2 * robot_r + safety_off; 

    // Find potential collision points
    std::vector<coordinating::collision_t> collisions_1_2 = get_collisions(path1, path2, robot_r);
    std::vector<coordinating::collision_t> collisions_2_3 = get_collisions(path2, path3, robot_r);
    std::vector<coordinating::collision_t> collisions_1_3 = get_collisions(path1, path3, robot_r);

    /*
    for (const auto& c : collisions_1_2){
        const auto& [p1, p2, l1, l2] = c;
        std::cout << "(" << p1.x << ", " << p1.y << ")\n";
        std::cout << "(" << p2.x << ", " << p2.y << ")\n";
        std::cout << l1 << " " << l2 << '\n';
    }
    for (const auto& c : collisions_1_3){
        const auto& [p1, p2, l1, l2] = c;
        std::cout << "(" << p1.x << ", " << p1.y << ")\n";
        std::cout << "(" << p2.x << ", " << p2.y << ")\n";
        std::cout << l1 << " " << l2 << '\n';
    }
    for (const auto& c : collisions_2_3){
        const auto& [p1, p2, l1, l2] = c;
        std::cout << "(" << p1.x << ", " << p1.y << ")\n";
        std::cout << "(" << p2.x << ", " << p2.y << ")\n";
        std::cout << l1 << " " << l2 << '\n';
    }
    */

    double wait_t1, wait_t2, wait_t3;
    wait_t1 = wait_t2 = wait_t3 = 0;
    bool r1_r2_ok, r2_r3_ok, r1_r3_ok; 
    r1_r2_ok = r2_r3_ok = r1_r3_ok = false;

    const double l1 = get<1>(path1).back();
    const double l2 = get<1>(path2).back();
    const double l3 = get<1>(path3).back();

    while (4){
        while(!r1_r2_ok){
            if (std::any_of(std::execution::par_unseq, collisions_1_2.begin(), collisions_1_2.end(), [&](const auto& c){
                return same_time(c, wait_t1, wait_t2, 2 * robot_r, safety_off);
            })){
                if(l1 < l2){
                    wait_t2 += wait_time;
                    r2_r3_ok = false;
                }
                else{
                    wait_t1 += wait_time;
                    r1_r3_ok = false;
                }
            }
            else r1_r2_ok = true;
        }
        while(!r2_r3_ok){
            if (std::any_of(std::execution::par_unseq, collisions_2_3.begin(), collisions_2_3.end(), [&](const auto& c){
                return same_time(c, wait_t2, wait_t3, 2 * robot_r, safety_off);
            })){
                if(l2 < l3){
                    wait_t3 += wait_time;
                    r1_r3_ok = false;
                }
                else{
                    wait_t2 += wait_time;
                    r1_r2_ok = false;
                }
            }
            else r2_r3_ok = true;
        }
        while(!r1_r3_ok){
            if (std::any_of(std::execution::par_unseq, collisions_1_3.begin(), collisions_1_3.end(), [&](const auto& c){
                return same_time(c, wait_t1, wait_t3, 2 * robot_r, safety_off);
            })){
                if(l1 < l3){
                    wait_t3 += wait_time;
                    r2_r3_ok = false;
                }
                else{
                    wait_t1 += wait_time;
                    r1_r2_ok = false;
                }
            }
            else r1_r3_ok = true;
        }

        if (r1_r2_ok && r2_r3_ok && r1_r3_ok)
            break;
    }

    return { wait_t1 / .2, wait_t2 / .2, wait_t3 / .2 };
}