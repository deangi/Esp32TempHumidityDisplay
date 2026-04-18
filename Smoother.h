#pragma once
#include <Arduino.h>

// Abstract base: subclasses precompute coefficients in prepare(), then answer
// point queries in evaluate().  x is a fractional sample index in [0, n-1].
class Smoother {
public:
  virtual void  prepare(const float* buf, int n) = 0;
  virtual float evaluate(float x) const          = 0;
  virtual ~Smoother() {}
};

// ── Centered moving average ───────────────────────────────────────────────────
// Window narrows at the edges so end-points are never padded with zeros.
class MovingAverageSmoother : public Smoother {
public:
  explicit MovingAverageSmoother(int window = 5);
  void  prepare(const float* buf, int n) override;
  float evaluate(float x) const override;
private:
  int          _win;
  const float* _buf;
  int          _n;
};

// ── Natural cubic spline ──────────────────────────────────────────────────────
// Subsamples the input to at most maxKnots evenly-spaced knots before fitting,
// so the spline smooths across noise rather than interpolating through it.
// evaluate() maps from the original sample-index space back to knot space,
// enabling pixel-resolution curve drawing at any sample density.
class CubicSplineSmoother : public Smoother {
public:
  explicit CubicSplineSmoother(int maxKnots = 80);
  ~CubicSplineSmoother() override;
  void  prepare(const float* buf, int n) override;
  float evaluate(float x) const override;
private:
  int          _maxKnots;
  const float* _y;     // original buffer (not owned)
  float*       _ySub;  // subsampled knot values (owned)
  float*       _M;     // second derivatives at each knot (owned)
  int          _n;     // original sample count
  int          _k;     // actual knot count used
};
