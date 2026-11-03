// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherCurve2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherCurve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a general 2D curve through its adaptor: sampling with a deflection estimate for
/// <c>Box</c>, sampling + PSO/Brent numerical optimization for <c>BoxOptimal</c>.
/// </summary>
/// <remarks>
/// Keeps a reference to the adaptor, as OCCT does. Unlike the 3D GeomBndLib_OtherCurve, OCCT's file has its own
/// optimizer (<c>AdjustExtr2dCurve</c>) over the legacy math_BrentMinimum and math_PSO, ported here as it is.
/// </remarks>
public sealed class OtherCurve2dBounds : Curve2dBounds.ICurve2dEvaluator
{
    private readonly Curve2DAdaptorBase _curve;

    public OtherCurve2dBounds(Curve2DAdaptorBase curve)
    {
        _curve = curve;
    }

    Curve2D? Curve2dBounds.ICurve2dEvaluator.CurveGeometry() => null;

    /// <summary>Fill 2D box with sampled points and return maximum deflection.</summary>
    private static double FillBox2d(ref Box2d box, Curve2DAdaptorBase curve, double first, double last, int n)
    {
        Point2D p1, p2, p3;
        curve.D0(first, out p1);
        box.Add(p1);
        double p = first, dp = last - first, tol = 0.0;
        if (Math.Abs(dp) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            dp /= 2 * n;
            for (int i = 1; i <= n; i++)
            {
                p += dp;
                curve.D0(p, out p2);
                box.Add(p2);
                p += dp;
                curve.D0(p, out p3);
                box.Add(p3);
                Point2D pc = new((p1.XY() + p3.XY()) / 2.0);
                tol = Max(tol, pc.Distance(p2));
                p1 = p3;
            }
        }
        else
        {
            curve.D0(first, out p1);
            box.Add(p1);
            curve.D0(last, out p3);
            box.Add(p3);
        }
        return tol;
    }

    /// <summary>1D function object for coordinate minimization/maximization along a 2D curve.</summary>
    private sealed class Curv2dCoordFunc(Curve2DAdaptorBase theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : Function
    {
        private readonly Curve2DAdaptorBase _curve = theCurve;
        private readonly double _uMin = theUMin;
        private readonly double _uMax = theUMax;
        private readonly int _coordIdx = theCoordIdx;
        private readonly double _sign = theSign;

        public override bool Value(double x, out double f)
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
    private sealed class Curv2dCoordMVarFunc(Curve2DAdaptorBase theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : MultipleVarFunction
    {
        private readonly Curve2DAdaptorBase _curve = theCurve;
        private readonly double _uMin = theUMin;
        private readonly double _uMax = theUMax;
        private readonly int _coordIdx = theCoordIdx;
        private readonly double _sign = theSign;

        public override bool Value(math_Vector x, out double f)
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

        public override int VariableCount => 1;
    }

    /// <summary>Adjust coordinate extremum using PSO + Brent for 2D curve.</summary>
    private static double AdjustExtr2dCurve(Curve2DAdaptorBase curve,
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
        if (uMax - uMin < 0.01 * du)
        {
            BrentMinimum optLoc0 = new(reltol, 100, uTol);
            Curv2dCoordFunc func0 = new(curve, uMin, uMax, coordIdx, sign);
            optLoc0.Perform(func0, uMin, (uMin + uMax) / 2.0, uMax);
            if (optLoc0.IsDone)
            {
                extr = optLoc0.Minimum();
                return sign * extr;
            }
        }

        int nbParticles = Max(8, RealToInt(32 * (uMax - uMin) / du));
        double maxStep = (uMax - uMin) / (nbParticles + 1);
        math_Vector t = new(1, 1);
        math_Vector lowBorder = new(1, 1);
        math_Vector uppBorder = new(1, 1);
        math_Vector steps = new(1, 1);
        lowBorder[1] = uMin;
        uppBorder[1] = uMax;
        steps[1] = Min(0.1 * du, maxStep);

        Curv2dCoordMVarFunc func = new(curve, uMin, uMax, coordIdx, sign);
        Pso finder = new(func, lowBorder, uppBorder, steps, nbParticles);
        finder.Perform(steps, out extr, t);

        BrentMinimum optLoc = new(reltol, 100, uTol);
        Curv2dCoordFunc func1 = new(curve, uMin, uMax, coordIdx, sign);
        optLoc.Perform(func1, Max(t[1] - steps[1], uMin), t[1], Min(t[1] + steps[1], uMax));

        if (optLoc.IsDone)
        {
            extr = optLoc.Minimum();
            return sign * extr;
        }

        return sign * extr;
    }

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double tol) => Box(_curve.FirstParameter(), _curve.LastParameter(), tol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double u1, double u2, double theTol)
    {
        Box2d box = new();
        const double weakness = 1.5;
        const int n = 33;
        Box2d b1 = new();
        double tol = FillBox2d(ref b1, _curve, u1, u2, n);
        b1.Enlarge(weakness * tol);
        b1.Get(out double x, out double y, out double X, out double Y);
        box.Update(x, y, X, Y);
        box.Enlarge(theTol);
        return box;
    }

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Box2d BoxOptimal(double tol) => BoxOptimal(_curve.FirstParameter(), _curve.LastParameter(), tol);

    /// <summary>Compute precise bounding box using PSO + Brent optimization.</summary>
    public Box2d BoxOptimal(double u1, double u2, double tol)
    {
        Box2d box = new();
        Curve2DAdaptorBase c = _curve;
        int nu = SamplingHelpers.ComputeNbSamples2d(c, u1, u2);

        Span<double> coordMin = [RealLast(), RealLast()];
        Span<double> coordMax = [-RealLast(), -RealLast()];
        Span<double> deflMax = [-RealLast(), -RealLast()];

        Point2D p;
        double du = (u2 - u1) / (nu - 1), du2 = du / 2.0;
        NCollection_Array1<Xy> pnts = new(1, nu);
        double u;
        for (int i = 1, j = 0; i <= nu; i++, j++)
        {
            u = u1 + j * du;
            c.D0(u, out p);
            pnts[i] = p.XY();
            for (int k = 0; k < 2; ++k)
            {
                if (coordMin[k] > p.Coord(k + 1))
                {
                    coordMin[k] = p.Coord(k + 1);
                }
                if (coordMax[k] < p.Coord(k + 1))
                {
                    coordMax[k] = p.Coord(k + 1);
                }
            }
            if (i > 1)
            {
                Xy pm = 0.5 * (pnts[i - 1] + pnts[i]);
                c.D0(u - du2, out p);
                Xy aD = (p.XY() - pm);
                for (int k = 0; k < 2; ++k)
                {
                    if (coordMin[k] > p.Coord(k + 1))
                    {
                        coordMin[k] = p.Coord(k + 1);
                    }
                    if (coordMax[k] < p.Coord(k + 1))
                    {
                        coordMax[k] = p.Coord(k + 1);
                    }
                    double d = Math.Abs(aD.Coord(k + 1));
                    if (deflMax[k] < d)
                    {
                        deflMax[k] = d;
                    }
                }
            }
        }

        double eps = Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion());
        for (int k = 0; k < 2; ++k)
        {
            double d = deflMax[k];
            if (d <= eps)
            {
                continue;
            }
            double cMin = coordMin[k];
            double cMax = coordMax[k];
            for (int i = 1; i <= nu; ++i)
            {
                if (pnts[i].Coord(k + 1) - cMin < d)
                {
                    double umin = u1 + Max(0, i - 2) * du;
                    double umax = u1 + Min(nu - 1, i) * du;
                    double cmin = AdjustExtr2dCurve(c, umin, umax, cMin, k + 1, eps, true);
                    if (cmin < cMin)
                    {
                        cMin = cmin;
                    }
                }
                else if (cMax - pnts[i].Coord(k + 1) < d)
                {
                    double umin = u1 + Max(0, i - 2) * du;
                    double umax = u1 + Min(nu - 1, i) * du;
                    double cmax = AdjustExtr2dCurve(c, umin, umax, cMax, k + 1, eps, false);
                    if (cmax > cMax)
                    {
                        cMax = cmax;
                    }
                }
            }
            coordMin[k] = cMin;
            coordMax[k] = cMax;
        }

        box.Add(new Point2D(coordMin[0], coordMin[1]));
        box.Add(new Point2D(coordMax[0], coordMax[1]));
        box.Enlarge(eps);
        return box;
    }
}
