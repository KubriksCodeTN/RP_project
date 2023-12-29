// WARNING: non-standard pragma
#pragma once

#include <cmath>
#include <tuple>
#include "clipper.h"

namespace offsetting{

    Clipper2Lib::PathsD offset_env(const Clipper2Lib::PathsD&, const Clipper2Lib::PathsD&, double, double);
    Clipper2Lib::PathsD offset_minimum(const Clipper2Lib::PathsD&, const Clipper2Lib::PathsD&, double);

};
