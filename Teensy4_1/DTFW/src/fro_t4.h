#ifndef FRO_T4_H
#define FRO_T4_H
#include <math.h>
// Teensy 4.1 / Arduino C++
// Queue-size-1 C∞ feedrate smoother (no blending, no growing queue).
// Hard-clamped output/input to [0,1]. Works with floats.
// Call tick() at a fixed rate (dt hardcoded below).
class FRO_T4 {
public:
	const float eps = 1e-6f;

  // Hardcoded tuning, no-arg ctor
  FRO_T4() {
    dt   = 0.001f;   // 1 kHz tick
    T01  = 0.1f;   // seconds to move by 1.0 (0->1)
    Tmin = dt;       // minimum move duration
    reset(0.0f);
  }

  // Reset state (y0 in [0,1])
  inline void reset(float y0_in) {
    y = clip01(y0_in);

    active = false;
    pending_valid = false;
    last_valid = false;

    y0 = y;
    tgt = y;
    tau = 0.0f;
    Tcur = Tmin;

    last_r = y;
  }

  // Push new reference sample r (float). Clamped to [0,1].
  inline void setFeedrate(float r_in) {
    float r = clip01(r_in);

    if (!last_valid) {
      last_valid = true;
      last_r = r;
      y = r;
      y0 = r;
      tgt = r;
      tau = 0.0f;
      Tcur = Tmin;
      active = false;
      pending_valid = false;
      return;
    }

    // Float compare with a tiny deadband to avoid re-triggering on noise
	if (fabsf(r - last_r) <= eps) return;
	last_r = r;
    // For PRBS-ish values near 0/1, exact compare is fine.
    //if (r == last_r) return;
    //last_r = r;

    if (active) {
      // queue size 1: latest wins
      pending = r;
      pending_valid = true;
    } else {
      startMove(r);
    }
  }

  // Advance by one tick (dt). Must be called at fixed rate matching dt.
  inline void tick() {
    if (!active) return;

    tau += dt;
    if (tau > Tcur) tau = Tcur;

    // Evaluate C∞ move (no "snap overwrite")
    float s = (Tcur > 0.0f) ? (tau / Tcur) : 1.0f;
    float w = S(s);
    y = clip01(y0 + (tgt - y0) * w);

    // Finish?
    if (tau >= Tcur) {
      active = false;

      // If pending exists, start it now (latest wins)
      if (pending_valid) {
        float nxt = pending;
        pending_valid = false;
        if (nxt != tgt) startMove(nxt);
      }
    }
  }

  inline float getFeedrate() const { return y; }

private:
  // tuning
  float dt, T01, Tmin;

  // current output + move state
  float y, y0, tgt, tau, Tcur;
  bool  active;

  // queue size 1
  float pending;
  bool  pending_valid;

  // last input (to avoid re-triggering)
  float last_r;
  bool  last_valid;

  inline float clip01(float v) const {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
  }

  inline float T_for(float dy) const {
    float T = fabsf(dy) * T01;
    return (T < Tmin) ? Tmin : T;
  }

  inline void startMove(float new_tgt) {
    new_tgt = clip01(new_tgt);
    float dy = new_tgt - y;

    if (dy == 0.0f) {
      active = false;
      y0 = y;
      tgt = new_tgt;
      tau = 0.0f;
      Tcur = Tmin;
      return;
    }

    active = true;
    y0 = y;
    tgt = new_tgt;
    tau = 0.0f;
    Tcur = T_for(dy);
  }

  // C∞ step S(s): 0->1 on [0,1]
  inline float S(float s) const {
    if (s <= 0.0f) return 0.0f;
    if (s >= 1.0f) return 1.0f;

    // bump(s)=exp(-1/s), bump(1-s)=exp(-1/(1-s))
    // small epsilon to avoid exp(-inf) and divide weirdness
    float sp = (s < eps) ? eps : s;
    float sm = (1.0f - s < eps) ? eps : (1.0f - s);

    float a = expf(-1.0f / sp);
    float b = expf(-1.0f / sm);
    float den = a + b;
    return (den > 0.0f) ? (a / den) : s;
  }
};
#endif // FRO_T4_H
