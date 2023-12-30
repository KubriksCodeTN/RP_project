#include <vector>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <inttypes.h>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>

using std::vector;
using edge_t = std::tuple<int32_t, double>;
using graph_t = vector<vector<edge_t>>;
using path_t = std::deque<int32_t>;
using point_t = std::tuple<double, double>;
using map_t = std::unordered_map<int32_t, point_t>; 
using heap_t = std::priority_queue<std::tuple<double, int32_t>, std::vector<std::tuple<double, int32_t>>, std::greater<std::tuple<double, int32_t>>>;

/**
 * @note this is a very bad way of doing this
 */
struct graph{
    graph_t g;
    map_t map;
};

/**
 * @brief The so called "average function" by Goldberg.
 */
std::tuple<double, double> h(const graph& g, int32_t s, int32_t e, int32_t v){
    const auto& [xs, ys] = g.map.at(s);
    const auto& [xe, ye] = g.map.at(e);
    const auto& [xv, yv] = g.map.at(v);

    double pi_f = sqrt((xv - xe) * (xv - xe) + (yv - ye) * (yv - ye));
    double pi_r = sqrt((xs - xv) * (xs - xv) + (ys - yv) * (ys - yv));
    double pf = (pi_f - pi_r) * .5;
    double pr = -pf;
    return { pf, pr };
}

/**
 * @brief this is a possible implementation of bidirectional A* as explained here:
 * https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf
 * 
 * @note in the end this was not used but it's still cool
 */
double BAstar(const graph& g, int32_t s, int32_t e, path_t& p){
    constexpr double inf = std::numeric_limits<double>::infinity();

    heap_t pq_s, pq_e;
    
    std::unordered_set<int32_t> cs, ce;
    vector<bool> vs(g.g.size(), 1), ve(g.g.size(), 1);
    vector<double> gs(g.g.size(), inf), ge(g.g.size(), inf);
    vector<double> fs(g.g.size(), inf), fe(g.g.size(), inf);
    vector<int32_t> f1(g.g.size()), f2(g.g.size());

    f1[s] = -1;
    f2[e] = -1;

    auto hf = std::get<0>(h(g, s, e, s));
    gs[s] = 0;
    fs[s] = hf;
    pq_s.emplace(fs[s], s);

    auto hrr = std::get<1>(h(g, s, e, e));
    ge[e] = 0;
    fe[e] = hrr;
    pq_e.emplace(fe[e], e);
    int32_t u;

    while (pq_s.size() ||  pq_e.size()){
        while (4){
            u = std::get<1>(pq_s.top());
            pq_s.pop();
            if (vs[u]){
                vs[u] = 0;
                break;
            }
        }
        for (const auto [v, w] : g.g[u]){
            if (cs.contains(v))
                continue;
            auto tmpg = gs[u] + w;
            if (gs[v] > tmpg){
                auto hh = std::get<0>(h(g, s, e, v));
                gs[v] = tmpg;
                fs[v] = tmpg + hh;
                f1[v] = u;
                pq_s.emplace(fs[v], v);
            }
        }
        if (ce.contains(u))
            break;
        cs.emplace(u);

        while (4){
            u = std::get<1>(pq_e.top());
            pq_e.pop();
            if (ve[u]){
                ve[u] = 0;
                break;
            }
        }
        for (const auto [v, w] : g.g[u]){
            if (ce.contains(v))
                continue;
            auto tmpg = ge[u] + w;
            if (ge[v] > tmpg){
                auto hh = std::get<1>(h(g, s, e, v));
                ge[v] = tmpg;
                fe[v] = tmpg + hh;
                f2[v] = u;
                pq_e.emplace(fe[v], v);
            }
        }
        if (cs.contains(u))
            break;
        ce.emplace(u);
    }

    double mu = inf;
    int32_t mid;
    cs.merge(ce);
    for (const auto u : cs){
        if (gs[u] + ge[u] < mu){
            mu = gs[u] + ge[u];
            mid = u;
        }
    }

    u = mid;
    while (f1[u] != -1){
        p.insert(p.begin(), u);
        u = f1[u];
    }
    p.insert(p.begin(), s);
    u = mid;
    auto aux = f2[u];
    while (aux != -1){
        p.insert(p.end(), aux);
        u = aux;
        aux = f2[u];
    }

    return mu;
}
