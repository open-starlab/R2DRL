// Goalkeeper-only adaptation to servers that forbid negative dash power.
#ifndef HELIOS_GOALIE_DASH_COMPAT_H
#define HELIOS_GOALIE_DASH_COMPAT_H

#include <algorithm>
#include <cmath>

namespace goalie_dash_compat {

struct Command {
    double power;
    double direction;
};

inline bool supports_directional_reverse(double min_angle, double max_angle)
{
    return max_angle >= 180.0 || min_angle <= -180.0;
}

inline Command resolve(double requested_power, double min_power, double max_power,
                       double min_angle, double max_angle)
{
    if (!std::isfinite(requested_power)) return {0.0, 0.0};
    if (requested_power < 0.0 && min_power >= 0.0
        && supports_directional_reverse(min_angle, max_angle))
    {
        // Keep server back_dash_rate and positive-power stamina accounting.
        return {std::min(max_power, std::max(min_power, -requested_power)),
                max_angle >= 180.0 ? 180.0 : -180.0};
    }
    // Preserve negative-power semantics on older servers that support them.
    return {std::min(max_power, std::max(min_power, requested_power)), 0.0};
}

} // namespace goalie_dash_compat
#endif
