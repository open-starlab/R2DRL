#include "goalie_dash_compat.h"
#include <cassert>
#include <iostream>
#include <limits>

void check(double requested, double lo, double hi, double min_angle, double max_angle,
           double power, double direction)
{
    const auto c = goalie_dash_compat::resolve(requested, lo, hi, min_angle, max_angle);
    assert(c.power == power && c.direction == direction);
}

int main()
{
    check(-100, 0, 100, -180, 180, 100, 180);
    check(-30, 0, 100, -180, 180, 30, 180);
    check(-150, 0, 100, -180, 180, 100, 180);
    check(-100, -100, 100, -180, 180, -100, 0);
    check(-100, -20, 100, -180, 180, -20, 0);
    check(100, 0, 100, -180, 180, 100, 0);
    check(150, 0, 100, -180, 180, 100, 0);
    check(0, 0, 100, -180, 180, 0, 0);
    check(-100, 0, 100, -180, 90, 100, -180);
    check(-100, 0, 100, -90, 90, 0, 0);
    check(-1, 0, 100, -180, 180, 1, 180);
    check(-0.0, 0, 100, -180, 180, 0, 0);
    check(std::numeric_limits<double>::quiet_NaN(), 0, 100, -180, 180, 0, 0);
    check(std::numeric_limits<double>::infinity(), 0, 100, -180, 180, 0, 0);
    std::cout << "PASS: 14 goalkeeper dash compatibility cases\n";
}
