// odometry.h — three-wheel absolute position tracking.
//
// Two parallel tracking wheels give forward travel and heading change; the
// perpendicular rear wheel captures sideways displacement, which a two-encoder
// differential model silently misses. On a VEX field that sideways term is not
// academic — it is what a defensive push does to you.

#pragma once

namespace Odom {

  // Tracking geometry, in inches. Measure these on the actual robot.
  extern const double WHEEL_DIAMETER;
  extern const double TICKS_PER_REV;
  extern const double TRACK_WIDTH;        // between the two parallel wheels
  extern const double BACK_WHEEL_OFFSET;  // rear wheel to the tracking centre

  void reset();

  // Integrates the encoder deltas since the last call. Cheap; call it often —
  // the pose is only as fresh as the last call, and every motion primitive
  // calls it on completion.
  void update();

  double x();          // inches
  double y();          // inches
  double headingDeg(); // degrees, wrapped to (-180, 180]

}
