// autons.h — autonomous routines.
//
// Each routine is a named function so the brain-screen selector can pick one at
// match time. Distances are in wheel turns and headings in degrees; both are
// re-tuned per field, which is exactly why they live here and not scattered
// through main.cpp.

#pragma once

namespace Auton {

  enum Routine {
    NONE = 0,
    NEAR_SIDE,
    FAR_SIDE,
    SKILLS,
    ROUTINE_COUNT
  };

  const char* name(Routine r);

  // Runs the selected routine. Safe to call with NONE.
  void run(Routine r);

  // Brain-screen selector. Call from pre_auton; returns the current choice.
  Routine selected();
  void cycleSelection();
  void drawSelector();

}
