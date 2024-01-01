// WARNING non-standard pragma
#pragma once

#include "clipper.h"


namespace coordinating{

    using sampling_t = std::tuple<Clipper2Lib::PathD, std::vector<double>>;
    using collision_t = std::tuple<Clipper2Lib::PointD, Clipper2Lib::PointD, double, double>;
    using triplet_t = std::tuple<double, double, double>;

    triplet_t coordinate(const sampling_t&, const sampling_t&, const sampling_t&, double, double);
};