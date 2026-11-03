// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a general 3D curve through its adaptor: sampling with a deflection estimate for
/// <c>Box</c>, sampling + PSO/Brent numerical optimization for <c>BoxOptimal</c>.
/// </summary>
/// <remarks>Keeps a reference to the adaptor, as OCCT does (its <c>std::reference_wrapper</c>).</remarks>
public sealed class OtherCurveBounds : CurveBounds.ICurveEvaluator
{
    private readonly CurveAdaptorBase _curve;

    public OtherCurveBounds(CurveAdaptorBase curve)
    {
        _curve = curve;
    }

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => null;

    /// <summary>Fill box with sampled points and return maximum deflection.</summary>
    private static double FillBox(ref Box box, CurveAdaptorBase curve, double first, double last, int n)
    {
        Point3D p1, p2, p3;
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
                Point3D pc = new((p1.XYZ() + p3.XYZ()) / 2.0);
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

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double tol) => Box(_curve.FirstParameter(), _curve.LastParameter(), tol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double theTol)
    {
        Box box = new();
        const double weakness = 1.5;
        const int n = 33;
        Box b1 = new();
        double tol = FillBox(ref b1, _curve, u1, u2, n);
        b1.Enlarge(weakness * tol);
        b1.Get(out double x, out double y, out double z, out double X, out double Y, out double Z);
        box.Update(x, y, z, X, Y, Z);
        box.Enlarge(theTol);
        return box;
    }

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Box BoxOptimal(double tol) => BoxOptimal(_curve.FirstParameter(), _curve.LastParameter(), tol);

    /// <summary>Compute precise bounding box using PSO + Brent optimization.</summary>
    public Box BoxOptimal(double u1, double u2, double tol)
    {
        Box box = new();
        CurveAdaptorBase c = _curve;
        int nu = SamplingHelpers.ComputeNbSamples(c, u1, u2);

        Span<double> coordMin = [RealLast(), RealLast(), RealLast()];
        Span<double> coordMax = [-RealLast(), -RealLast(), -RealLast()];
        Span<double> deflMax = [-RealLast(), -RealLast(), -RealLast()];

        Point3D p;
        double du = (u2 - u1) / (nu - 1), du2 = du / 2.0;
        NCollection_Array1<Xyz> pnts = new(1, nu);
        double u;
        for (int i = 1, j = 0; i <= nu; i++, j++)
        {
            u = u1 + j * du;
            c.D0(u, out p);
            pnts[i] = p.XYZ();
            for (int k = 0; k < 3; ++k)
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
                Xyz pm = 0.5 * (pnts[i - 1] + pnts[i]);
                c.D0(u - du2, out p);
                Xyz aD = (p.XYZ() - pm);
                for (int k = 0; k < 3; ++k)
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
        for (int k = 0; k < 3; ++k)
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
                    double cmin = OptimizationHelpers.AdjustExtrCurve(c, umin, umax, cMin, k + 1, eps, true);
                    if (cmin < cMin)
                    {
                        cMin = cmin;
                    }
                }
                else if (cMax - pnts[i].Coord(k + 1) < d)
                {
                    double umin = u1 + Max(0, i - 2) * du;
                    double umax = u1 + Min(nu - 1, i) * du;
                    double cmax = OptimizationHelpers.AdjustExtrCurve(c, umin, umax, cMax, k + 1, eps, false);
                    if (cmax > cMax)
                    {
                        cMax = cmax;
                    }
                }
            }
            coordMin[k] = cMin;
            coordMax[k] = cMax;
        }

        box.Add(new Point3D(coordMin[0], coordMin[1], coordMin[2]));
        box.Add(new Point3D(coordMax[0], coordMax[1], coordMax[2]));
        box.Enlarge(eps);
        return box;
    }
}
