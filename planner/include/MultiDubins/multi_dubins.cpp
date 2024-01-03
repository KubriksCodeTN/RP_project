#include "multi_dubins.hpp"
#include "BS_thread_pool_light.hpp"

using namespace multi_dubins;
using namespace dubins;

/**
 * @brief calculates the multipoint dubins shortest path using interpolation of dubins curves and using
 * dynamic programming to find optimal angles in the middle points of the path. 
 * 
 * @param v array of points to interpolate
 * @param th0 starting angle at point v[0]
 * @param thf arriving angle for point v[-1]
 * @param Kmax maximum curvature
 * @param k number of angle intervals per cycle
 * @param m  number of refinement cycles
 * @return the optimal path
 * 
 * @note for more info see this paper: "An Iterative Dynamic Programming Approach to the Multipoint Markov-Dubins Problem"
 * @todo __maybe__ remove the goto
 */
path_t multi_dubins::dp_dubins(const vector<point_t>& v, double th0, double thf, double Kmax, uint32_t k, uint32_t m){
    static BS::thread_pool_light pool(std::min(v.size() - 1, std::thread::hardware_concurrency() - 1UL));
    vector<d_curve> ret(v.size() - 1);

    double oldh = 2 * M_PI;
    double h = oldh / k;

    vector<double> ths(v.size(), 2 * M_PI);
    vector<int32_t> paths((v.size() - 1) * k);
    vector<double> lengths((v.size() - 1) * k);
    vector<d_curve> matrix((v.size() - 1) * k * k);
    
    /*
     * matrix[i][j][l] <--> matrix[(i * k + j) * k + l]
     */

    ++m;
    while (4){
        --m;
        auto matrix_loop = [&](const int32_t a, const int32_t b){
            for (int32_t i = a; i < b; ++i){
                for (uint32_t j = 0; j < k; ++j){
                    for (uint32_t l = 0; l < k; ++l){
                        d_shortest(get<0>(v[i]), get<1>(v[i]), ths[i] - oldh + h * j, 
                                   get<0>(v[i + 1]), get<1>(v[i + 1]), ths[i + 1] - oldh + h * l, Kmax, matrix[(i * k + j) * k + l]);
                    }
                }
            }
        };
        pool.push_loop(1, v.size() - 2, matrix_loop);
        for (uint32_t i = 0; i < k; ++i){
            d_shortest(get<0>(v[v.size() - 2]), get<1>(v[v.size() - 2]), ths[v.size() - 2] - oldh + h * i, 
                       get<0>(v.back()), get<1>(v.back()), thf, Kmax, matrix[((v.size() - 2) * k + i) * k]);
            lengths[(v.size() - 2) * k + i] = matrix[((v.size() - 2) * k + i) * k].L;
        }
        pool.wait_for_tasks();

        for (auto i = v.size() - 3; i > 0; --i){
            for (uint32_t j = 0; j < k; ++j){
                lengths[i * k + j] = matrix[(i * k + j) * k].L + lengths[(i + 1) * k];
                paths[i * k + j] = 0;
                for (uint32_t l = 1; l < k; ++l){
                    if (matrix[(i * k + j) * k + l].L + lengths[(i + 1) * k + l] < lengths[i * k + j]){
                        lengths[i * k + j] = matrix[(i * k + j) * k + l].L + lengths[(i + 1) * k + l];
                        paths[i * k + j] = l;
                    }
                }
            }
        }

        lengths[0] = std::numeric_limits<double>::max();
        for (uint32_t i = 0; i < k; ++i){
            d_shortest(get<0>(v[0]), get<1>(v[0]), th0, 
                       get<0>(v[1]), get<1>(v[1]), ths[1] - oldh + h * i, Kmax, matrix[1]);
            if (matrix[1].L + lengths[1 * k + i] < lengths[0]){
                matrix[0] = matrix[1];
                lengths[0] = matrix[1].L + lengths[1 * k + i];
                paths[0] = i;
            }
        }

        int32_t idx = paths[0];

        if (m)
            goto again;
        ret[0] = matrix[0];
        for (auto i = 1UL; i < v.size() - 1; ++i){
            ret[i] = matrix[(i * k + idx) * k + paths[i * k + idx]];
            idx = paths[i * k + idx];
        }
        return ret;

    again:
        for (auto i = 1UL; i < v.size() - 1; ++i){
            ths[i] = ths[i] - oldh + idx * h; 
            idx = paths[i * k + idx];
        }
        oldh = h * 1.5;
        h = 2 * oldh / k; // * 2 is important
    }
}
