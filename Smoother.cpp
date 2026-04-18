#include "Smoother.h"

// ── MovingAverageSmoother ─────────────────────────────────────────────────────

MovingAverageSmoother::MovingAverageSmoother(int window)
  : _win(window), _buf(nullptr), _n(0) {}

void MovingAverageSmoother::prepare(const float* buf, int n) {
  _buf = buf;
  _n   = n;
}

float MovingAverageSmoother::evaluate(float x) const {
  int i    = constrain((int)roundf(x), 0, _n - 1);
  int half = _win / 2;
  int lo   = max(0,      i - half);
  int hi   = min(_n - 1, i + half);
  float sum = 0;
  for (int j = lo; j <= hi; j++) sum += _buf[j];
  return sum / (hi - lo + 1);
}

// ── CubicSplineSmoother ───────────────────────────────────────────────────────

CubicSplineSmoother::CubicSplineSmoother(int maxKnots)
  : _maxKnots(maxKnots), _y(nullptr), _ySub(nullptr), _M(nullptr), _n(0), _k(0) {}

CubicSplineSmoother::~CubicSplineSmoother() {
  delete[] _ySub;
  delete[] _M;
}

// Fit a natural cubic spline through at most _maxKnots evenly-spaced knots
// subsampled from buf[0..n-1].  When n <= _maxKnots all points are used and
// the spline interpolates exactly; otherwise the reduced knot set smooths out
// high-frequency noise.
//
// Second derivatives M[i] satisfy (natural BCs: M[0]=M[k-1]=0):
//   M[i-1] + 4*M[i] + M[i+1] = 6*(y[i-1] - 2*y[i] + y[i+1])  i=1..k-2
// Solved in O(k) with the Thomas tridiagonal algorithm.
void CubicSplineSmoother::prepare(const float* buf, int n) {
  _y = buf;
  _n = n;

  int k = (_maxKnots > 0 && _maxKnots < n) ? _maxKnots : n;
  _k = k;

  // Build subsampled knot values
  delete[] _ySub;
  _ySub = new float[k];
  if (k == n) {
    memcpy(_ySub, buf, n * sizeof(float));
  } else {
    for (int i = 0; i < k; i++) {
      int idx = (int)roundf((float)i * (float)(n - 1) / (float)(k - 1));
      _ySub[i] = buf[constrain(idx, 0, n - 1)];
    }
  }

  // Fit spline through _ySub
  delete[] _M;
  _M = new float[k]();   // zero-initialised: M[0]=M[k-1]=0 (natural BCs)
  if (k < 3) return;

  int    m = k - 2;
  float* c = new float[m];   // normalised upper diagonal
  float* d = new float[m];   // normalised RHS

  c[0] = 0.25f;
  d[0] = 1.5f * (_ySub[0] - 2.0f*_ySub[1] + _ySub[2]);

  for (int j = 1; j < m; j++) {
    float denom = 4.0f - c[j - 1];
    c[j] = 1.0f / denom;
    d[j] = (6.0f * (_ySub[j] - 2.0f*_ySub[j + 1] + _ySub[j + 2]) - d[j - 1]) / denom;
  }

  _M[m] = d[m - 1];
  for (int j = m - 2; j >= 0; j--)
    _M[j + 1] = d[j] - c[j] * _M[j + 2];

  delete[] c;
  delete[] d;
}

// Evaluate the spline at fractional sample index x in [0, _n-1].
// x is first remapped to knot-index space [0, _k-1], then the appropriate
// piecewise cubic is evaluated:
//   S(t) = M[i]*(1-t)^3/6 + M[i+1]*t^3/6
//         + (y[i]   - M[i]  /6)*(1-t)
//         + (y[i+1] - M[i+1]/6)*t        where t = kx - i
float CubicSplineSmoother::evaluate(float x) const {
  if (_k < 2) return (_k == 1) ? _ySub[0] : 0.0f;

  float kx = (_n > 1) ? x * (float)(_k - 1) / (float)(_n - 1) : 0.0f;
  kx = constrain(kx, 0.0f, (float)(_k - 1));

  int   i = (int)kx;
  if (i >= _k - 1) i = _k - 2;
  float t = kx - (float)i;
  float u = 1.0f - t;

  return  _M[i]           * u*u*u / 6.0f
        + _M[i + 1]       * t*t*t / 6.0f
        + (_ySub[i]     - _M[i]     / 6.0f) * u
        + (_ySub[i + 1] - _M[i + 1] / 6.0f) * t;
}
