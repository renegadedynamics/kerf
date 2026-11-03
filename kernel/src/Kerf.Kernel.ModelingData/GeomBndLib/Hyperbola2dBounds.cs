// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 2D hyperbola (Geom2d_Hyperbola): the arc ends and the per-coordinate extrema, opened
/// for infinite parameters. The static method accepting gp_Hypr2d can be used directly.
/// </summary>
internal sealed class Hyperbola2dBounds : Curve2dBounds.ICurve2dEvaluator
{
    private readonly HyperbolaCurve2D _geom;

    public Hyperbola2dBounds(HyperbolaCurve2D hyperbola)
    {
        _geom = hyperbola;
    }

    public HyperbolaCurve2D Geometry() => _geom;

    Curve2D? Curve2dBounds.ICurve2dEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double u1, double u2, double tol) => Box(_geom.Hyperbola, u1, u2, tol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box2d BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for finite 2D hyperbola arc, adding per-coordinate extrema.</summary>
    internal static void ComputeHyperbola2dBox(in Hyperbola2D hypr, double t1, double t2, ref Box2d box)
    {
        Point2D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(t1, hypr);
        Point2D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(t2, hypr);
        box.Add(p1);
        box.Add(p2);

        if (t1 * t2 < 0.0)
        {
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, hypr));
        }

        Frame2D pos = hypr.Axis;
        Xy xDir = pos.XDirection.XY();
        Xy yDir = pos.YDirection.XY();
        double rmaj = hypr.MajorRadius;
        double rmin = hypr.MinorRadius;
        double eps = Epsilon(1.0);

        for (int i = 1; i <= 2; ++i)
        {
            double a = rmin * yDir.Coord(i);
            double b = rmaj * xDir.Coord(i);

            double abp = Math.Abs(a + b);
            double bam = Math.Abs(b - a);

            if (abp < eps || bam < eps)
            {
                continue;
            }

            double cf = bam / abp;
            double t3 = 0.5 * Math.Log(cf);

            if (t3 < t1 || t3 > t2)
            {
                continue;
            }
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(t3, hypr));
        }
    }

    /// <summary>Compute bounding box for a 2D hyperbola arc [theU1, theU2] defined by gp_Hypr2d.</summary>
    public static Box2d Box(in Hyperbola2D hypr, double u1, double u2, double tol)
    {
        Box2d box = new();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                throw new FailureException("GeomBndLib_Hyperbola2d::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                box.OpenXmax();
                box.OpenYmax();
            }
            else
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, hypr));
            }
            box.OpenXmin();
            box.OpenYmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                box.OpenXmin();
                box.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                throw new FailureException("GeomBndLib_Hyperbola2d::Box - bad parameter");
            }
            else
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, hypr));
            }
            box.OpenXmax();
            box.OpenYmax();
        }
        else
        {
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u1, hypr));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                box.OpenXmin();
                box.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                box.OpenXmax();
                box.OpenYmax();
            }
            else
            {
                ComputeHyperbola2dBox(hypr, u1, u2, ref box);
            }
        }
        box.Enlarge(tol);
        return box;
    }
}
