#include "vex.h"

#include "odometry.h"

#include <cmath>


namespace Odom {

  // PI is a POSIX extension, not standard C++ � it happens to exist in the
  // VEX toolchain's math.h but isn't guaranteed anywhere else.
  constexpr double PI = 3.14159265358979323846;

  const double WHEEL_DIAMETER   = 3.0;
  const double TICKS_PER_REV    = 360.0;
  const double TRACK_WIDTH      = 11.5;
  const double BACK_WHEEL_OFFSET = 5.5;

  namespace {

    double g_x = 0, g_y = 0, g_theta = 0;   // theta in RADIANS
    double g_lastLeft = 0, g_lastRight = 0, g_lastBack = 0;

    double toInches(double ticks) {
      return (ticks / TICKS_PER_REV) * WHEEL_DIAMETER * PI;
    }

  }  // namespace

  void reset() {
    g_x = g_y = g_theta = 0;
    leftEncoder.resetPosition();
    rightEncoder.resetPosition();
    backEncoder.resetPosition();
    g_lastLeft = g_lastRight = g_lastBack = 0;
  }

  void update() {
    const double left  = leftEncoder.position(degrees);
    const double right = rightEncoder.position(degrees);
    const double back  = backEncoder.position(degrees);

    const double dLeft  = toInches(left - g_lastLeft);
    const double dRight = toInches(right - g_lastRight);
    const double dBack  = toInches(back - g_lastBack);

    g_lastLeft = left;
    g_lastRight = right;
    g_lastBack = back;

    // Radians: a difference in arc length over the track width is already an
    // angle. The old code computed this exact quantity and then wrapped it as
    // though it were degrees (`if (deltaTheta > 180)`), a branch that could
    // never fire — a single 20 ms tick never turns the robot 180 radians — and
    // would have corrupted the pose if it had.
    const double dTheta = (dRight - dLeft) / TRACK_WIDTH;

    const double dForward = (dLeft + dRight) / 2.0;

    // Remove the sideways travel the rear wheel sees purely because it sits off
    // the tracking centre and swings when the robot rotates.
    const double dLateral = dBack - (dTheta * BACK_WHEEL_OFFSET);

    // Rotate the local step into field coordinates about the midpoint heading,
    // which is a better approximation over a finite tick than either endpoint.
    const double mid = g_theta + dTheta / 2.0;

    g_x += dForward * std::cos(mid) - dLateral * std::sin(mid);
    g_y += dForward * std::sin(mid) + dLateral * std::cos(mid);
    g_theta += dTheta;

    while (g_theta > PI)  g_theta -= 2 * PI;
    while (g_theta <= -PI) g_theta += 2 * PI;
  }

  double x() { return g_x; }
  double y() { return g_y; }
  double headingDeg() { return g_theta * 180.0 / PI; }

}  // namespace Odom
