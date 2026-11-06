// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OptimizationHelpers.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// PSO+Brent and PSO+Powell optimization wrappers for precise bounding box computation (OCCT's namespace
/// GeomBndLib_OptimizationHelpers).
/// </summary>
/// <remarks>
/// The function objects are private readonly structs implementing the MathUtils functor interfaces, which the JIT
/// specialises (decision 51). Where OCCT's <c>Value</c> returns false without writing its result, the port writes 0
/// (decision 32); the solvers discard it. OCCT's template <c>AdjustExtrT</c>, which only chooses between the 3D and
/// 2D functions, is the pair of overloads of the same name.
/// </remarks>
internal static class OptimizationHelpers
{
    // ============================================================
    // Internal function objects for optimization
    // ============================================================

    /// <summary>1D function object for coordinate minimization/maximization along a 3D curve.</summary>
    private readonly struct CurveCoordFunc(CurveAdaptorBase theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : IScalarFunction
    {
        private readonly CurveAdaptorBase _curve = theCurve;
        private readonly double _uMin = theUMin;
        private readonly double _uMax = theUMax;
        private readonly int _coordIdx = theCoordIdx;
        private readonly double _sign = theSign;

        public bool Value(double x, out double f)
        {
            if (x < _uMin || x > _uMax)
            {
                f = 0.0;
                return false;
            }
            Point3D p = _curve.Value(x);
            f = _sign * p.Coord(_coordIdx);
            return true;
        }
    }

    /// <summary>Multi-variable function for PSO on a 3D curve.</summary>
    private readonly struct CurveCoordMVarFunc(CurveAdaptorBase theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : IVectorFunction
    {
        private readonly CurveAdaptorBase _curve = theCurve;
        private readonly double _uMin = theUMin;
        private readonly double _uMax = theUMax;
        private readonly int _coordIdx = theCoordIdx;
        private readonly double _sign = theSign;

        public bool Value(math_Vector x, out double f)
        {
            if (x[1] < _uMin || x[1] > _uMax)
            {
                f = 0.0;
                return false;
            }
            Point3D p = _curve.Value(x[1]);
            f = _sign * p.Coord(_coordIdx);
            return true;
        }

        public int VariableCount => 1;
    }

    /// <summary>1D function object for coordinate minimization/maximization along a 2D curve.</summary>
    private readonly struct Curve2dCoordFunc(Curve2DAdaptorBase theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : IScalarFunction
    {
        private readonly Curve2DAdaptorBase _curve = theCurve;
        private readonly double _uMin = theUMin;
        private readonly double _uMax = theUMax;
        private readonly int _coordIdx = theCoordIdx;
        private readonly double _sign = theSign;

        public bool Value(double x, out double f)
        {
            if (x < _uMin || x > _uMax)
            {
                f = 0.0;
                return false;
            }
            Point2D p = _curve.Value(x);
            f = _sign * p.Coord(_coordIdx);
            return true;
        }
    }

    /// <summary>Multi-variable function for PSO on a 2D curve.</summary>
    private readonly struct Curve2dCoordMVarFunc(Curve2DAdaptorBase theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : IVectorFunction
    {
        private readonly Curve2DAdaptorBase _curve = theCurve;
        private readonly double _uMin = theUMin;
        private readonly double _uMax = theUMax;
        private readonly int _coordIdx = theCoordIdx;
        private readonly double _sign = theSign;

        public bool Value(math_Vector x, out double f)
        {
            if (x[1] < _uMin || x[1] > _uMax)
            {
                f = 0.0;
                return false;
            }
            Point2D p = _curve.Value(x[1]);
            f = _sign * p.Coord(_coordIdx);
            return true;
        }

        public int VariableCount => 1;
    }

    /// <summary>Multi-variable function for PSO+Powell on a surface.</summary>
    private readonly struct SurfCoordFunc : IVectorFunction
    {
        private readonly SurfaceAdaptorBase _surf;
        private readonly double _uMin;
        private readonly double _uMax;
        private readonly double _vMin;
        private readonly double _vMax;
        private readonly int _coordIdx;
        private readonly double _sign;
        private readonly double _penalty;

        public SurfCoordFunc(SurfaceAdaptorBase surf,
                             double uMin,
                             double uMax,
                             double vMin,
                             double vMax,
                             int coordIdx,
                             double sign)
        {
            _surf = surf;
            _uMin = uMin;
            _uMax = uMax;
            _vMin = vMin;
            _vMax = vMax;
            _coordIdx = coordIdx;
            _sign = sign;
            _penalty = 0.0;

            math_Vector x = new(1, 2);
            x[1] = uMin;
            x[2] = (vMin + vMax) / 2.0;
            Value(x, out double f1);
            x[1] = uMax;
            Value(x, out double f2);
            double du = Math.Abs((f2 - f1) / (uMax - uMin));
            x[1] = (uMin + uMax) / 2.0;
            x[2] = vMin;
            Value(x, out f1);
            x[2] = vMax;
            Value(x, out f2);
            double dv = Math.Abs((f2 - f1) / (vMax - vMin));
            double penalty = 10.0 * Max(du, dv);
            _penalty = Max(penalty, 1.0);
        }

        public bool Value(math_Vector x, out double f)
        {
            if (x[1] >= _uMin && x[1] <= _uMax && x[2] >= _vMin && x[2] <= _vMax)
            {
                Point3D p = _surf.Value(x[1], x[2]);
                f = _sign * p.Coord(_coordIdx);
            }
            else
            {
                double uPen = 0.0, vPen = 0.0, u0, v0;
                if (x[1] < _uMin)
                {
                    uPen = _penalty * (_uMin - x[1]);
                    u0 = _uMin;
                }
                else if (x[1] > _uMax)
                {
                    uPen = _penalty * (x[1] - _uMax);
                    u0 = _uMax;
                }
                else
                {
                    u0 = x[1];
                }
                if (x[2] < _vMin)
                {
                    vPen = _penalty * (_vMin - x[2]);
                    v0 = _vMin;
                }
                else if (x[2] > _vMax)
                {
                    vPen = _penalty * (x[2] - _vMax);
                    v0 = _vMax;
                }
                else
                {
                    v0 = x[2];
                }
                Point3D p = _surf.Value(u0, v0);
                f = _sign * p.Coord(_coordIdx) + uPen + vPen;
            }
            return true;
        }

        public int VariableCount => 2;
    }

    // ============================================================
    // Public helper functions
    // ============================================================

    /// <summary>
    /// Adjust coordinate extremum using PSO + Brent for 1D parameter curves: returns the adjusted extremum of
    /// coordinate theCoordIdx (1=X, 2=Y, 3=Z) on [theUMin, theUMax], starting from theExtr0; a minimum if theIsMin.
    /// </summary>
    internal static double AdjustExtrCurve(CurveAdaptorBase curve,
                                           double uMin,
                                           double uMax,
                                           double extr0,
                                           int coordIdx,
                                           double tol,
                                           bool isMin)
    {
        double sign = isMin ? 1.0 : -1.0;
        double extr = sign * extr0;

        double uTol = Max(curve.Resolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        double du = (curve.LastParameter() - curve.FirstParameter());

        double reltol = uTol / Max(Math.Abs(uMin), Math.Abs(uMax));
        Config brentCfg = new();
        brentCfg.XTolerance = reltol;
        brentCfg.FTolerance = uTol;
        brentCfg.MaxIterations = 100;

        if (uMax - uMin < 0.01 * du)
        {
            CurveCoordFunc func0 = new(curve, uMin, uMax, coordIdx, sign);
            ScalarResult optLoc0 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(func0, uMin, uMax, brentCfg);
            if (optLoc0.Status == Status.OK || optLoc0.Status == Status.MaxIterations)
            {
                if (optLoc0.Value.HasValue)
                {
                    extr = optLoc0.Value.Value;
                }
                return sign * extr;
            }
        }

        int nbParticles = Max(8, RealToInt(32 * (uMax - uMin) / du));
        math_Vector lowBorder = new(1, 1);
        math_Vector uppBorder = new(1, 1);
        lowBorder[1] = uMin;
        uppBorder[1] = uMax;

        CurveCoordMVarFunc func = new(curve, uMin, uMax, coordIdx, sign);
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig psoCfg = new(nbParticles);
        VectorResult psoRes = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(func, lowBorder, uppBorder, psoCfg);

        math_Vector t = new(1, 1);
        if (psoRes.Status == Status.OK || psoRes.Status == Status.MaxIterations)
        {
            if (psoRes.Solution is not null)
            {
                t[1] = psoRes.Solution[1];
            }
            else
            {
                t[1] = (uMin + uMax) / 2.0;
            }
            if (psoRes.Value.HasValue)
            {
                extr = psoRes.Value.Value;
            }
        }
        else
        {
            t[1] = (uMin + uMax) / 2.0;
        }

        CurveCoordFunc func1 = new(curve, uMin, uMax, coordIdx, sign);
        double step = Min(0.1 * du, (uMax - uMin) / (nbParticles + 1));
        ScalarResult optLoc = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(func1,
                                              Max(t[1] - step, uMin),
                                              Min(t[1] + step, uMax),
                                              brentCfg);

        if (optLoc.Status == Status.OK || optLoc.Status == Status.MaxIterations)
        {
            if (optLoc.Value.HasValue)
            {
                extr = optLoc.Value.Value;
                // Only blend it with PSO result if both were valid
                if (psoRes.Value.HasValue && psoRes.Solution is not null
                    && t[1] == psoRes.Solution[1])
                {
                    extr = Min(extr, psoRes.Value.Value);
                }
                return sign * extr;
            }
        }

        return sign * extr;
    }

    /// <summary>Adjust coordinate extremum using PSO + Brent for 1D parameter 2D curves (1=X, 2=Y).</summary>
    internal static double AdjustExtrCurve2d(Curve2DAdaptorBase curve,
                                             double uMin,
                                             double uMax,
                                             double extr0,
                                             int coordIdx,
                                             double tol,
                                             bool isMin)
    {
        double sign = isMin ? 1.0 : -1.0;
        double extr = sign * extr0;

        double uTol = Max(curve.Resolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        double du = (curve.LastParameter() - curve.FirstParameter());

        double reltol = uTol / Max(Math.Abs(uMin), Math.Abs(uMax));

        Config brentCfg = new();
        brentCfg.XTolerance = reltol;
        brentCfg.FTolerance = uTol;
        brentCfg.MaxIterations = 100;

        if (uMax - uMin < 0.01 * du)
        {
            Curve2dCoordFunc func0 = new(curve, uMin, uMax, coordIdx, sign);
            ScalarResult optLoc0 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(func0, uMin, uMax, brentCfg);
            if (optLoc0.Status == Status.OK || optLoc0.Status == Status.MaxIterations)
            {
                if (optLoc0.Value.HasValue)
                {
                    extr = optLoc0.Value.Value;
                }
                return sign * extr;
            }
        }

        int nbParticles = Max(8, RealToInt(32 * (uMax - uMin) / du));
        math_Vector lowBorder = new(1, 1);
        math_Vector uppBorder = new(1, 1);
        lowBorder[1] = uMin;
        uppBorder[1] = uMax;

        Curve2dCoordMVarFunc func = new(curve, uMin, uMax, coordIdx, sign);
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig psoCfg = new(nbParticles);
        VectorResult psoRes = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(func, lowBorder, uppBorder, psoCfg);

        math_Vector t = new(1, 1);
        if (psoRes.Status == Status.OK || psoRes.Status == Status.MaxIterations)
        {
            if (psoRes.Solution is not null)
            {
                t[1] = psoRes.Solution[1];
            }
            else
            {
                t[1] = (uMin + uMax) / 2.0;
            }
            if (psoRes.Value.HasValue)
            {
                extr = psoRes.Value.Value;
            }
        }
        else
        {
            t[1] = (uMin + uMax) / 2.0;
        }

        double step = Min(0.1 * du, (uMax - uMin) / (nbParticles + 1));
        Curve2dCoordFunc func1 = new(curve, uMin, uMax, coordIdx, sign);
        ScalarResult optLoc = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(func1,
                                              Max(t[1] - step, uMin),
                                              Min(t[1] + step, uMax),
                                              brentCfg);

        if (optLoc.Status == Status.OK || optLoc.Status == Status.MaxIterations)
        {
            if (optLoc.Value.HasValue)
            {
                extr = optLoc.Value.Value;
                if (psoRes.Value.HasValue && psoRes.Solution is not null
                    && t[1] == psoRes.Solution[1])
                {
                    extr = Min(extr, psoRes.Value.Value);
                }
                return sign * extr;
            }
        }

        return sign * extr;
    }

    /// <summary>
    /// Adjust coordinate extremum using PSO + Powell for 2D parameter surfaces, on [theUMin, theUMax] x
    /// [theVMin, theVMax].
    /// </summary>
    internal static double AdjustExtrSurf(SurfaceAdaptorBase surf,
                                          double uMin,
                                          double uMax,
                                          double vMin,
                                          double vMax,
                                          double extr0,
                                          int coordIdx,
                                          double tol,
                                          bool isMin,
                                          global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSwarm? keptSwarm = null)
    {
        double sign = isMin ? 1.0 : -1.0;
        double extr = sign * extr0;
        double relTol = 2.0 * tol;
        if (Math.Abs(extr) > tol)
        {
            relTol /= Math.Abs(extr);
        }
        double du = (surf.LastUParameter() - surf.FirstUParameter());
        double dv = (surf.LastVParameter() - surf.FirstVParameter());

        math_Vector lowBorder = new(1, 2);
        math_Vector uppBorder = new(1, 2);
        lowBorder[1] = uMin;
        uppBorder[1] = uMax;
        lowBorder[2] = vMin;
        uppBorder[2] = vMax;

        int nbU = Max(8, RealToInt(32 * (uMax - uMin) / du));
        int nbV = Max(8, RealToInt(32 * (vMax - vMin) / dv));
        int nbParticles = nbU * nbV;

        SurfCoordFunc func = new(surf, uMin, uMax, vMin, vMax, coordIdx, sign);
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig psoCfg = new(nbParticles);
        VectorResult psoRes = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(func, lowBorder, uppBorder, psoCfg, keptSwarm);

        math_Vector t = new(1, 2);
        if (psoRes.Status == Status.OK || psoRes.Status == Status.MaxIterations)
        {
            if (psoRes.Solution is not null)
            {
                t = new math_Vector(psoRes.Solution);
            }
            else
            {
                t[1] = (uMin + uMax) / 2.0;
                t[2] = (vMin + vMax) / 2.0;
            }
            if (psoRes.Value.HasValue)
            {
                extr = psoRes.Value.Value;
            }
        }
        else
        {
            t[1] = (uMin + uMax) / 2.0;
            t[2] = (vMin + vMax) / 2.0;
        }

        Matrix dir = new(1, 2, 1, 2, 0.0);
        dir[1, 1] = 1.0;
        dir[2, 2] = 1.0;

        Config powellCfg = new();
        powellCfg.XTolerance = relTol;
        powellCfg.FTolerance = tol;
        powellCfg.MaxIterations = 200;

        VectorResult powell = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PowellWithDirections(func, t, dir, powellCfg);

        if (powell.Status == Status.OK || powell.Status == Status.MaxIterations)
        {
            if (powell.Value.HasValue)
            {
                extr = Min(extr, powell.Value.Value);
            }
        }

        return sign * extr;
    }

    /// <summary>OCCT's <c>AdjustExtrT</c> over a 3D curve.</summary>
    internal static double AdjustExtrT(CurveAdaptorBase curve, double uMin, double uMax, double extr0,
                                       int coordIdx, double tol, bool isMin)
        => AdjustExtrCurve(curve, uMin, uMax, extr0, coordIdx, tol, isMin);

    /// <summary>OCCT's <c>AdjustExtrT</c> over a 2D curve.</summary>
    internal static double AdjustExtrT(Curve2DAdaptorBase curve, double uMin, double uMax, double extr0,
                                       int coordIdx, double tol, bool isMin)
        => AdjustExtrCurve2d(curve, uMin, uMax, extr0, coordIdx, tol, isMin);
}
