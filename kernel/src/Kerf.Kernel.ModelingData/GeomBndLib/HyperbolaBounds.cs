// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D hyperbola (Geom_Hyperbola): the arc ends and the per-coordinate extrema, opened
/// for infinite parameters. The static method accepting gp_Hypr can be used directly.
/// </summary>
public sealed class HyperbolaBounds : CurveBounds.ICurveEvaluator
{
    private readonly HyperbolaCurve3D _geom;

    public HyperbolaBounds(HyperbolaCurve3D hyperbola)
    {
        _geom = hyperbola;
    }

    public HyperbolaCurve3D Geometry() => _geom;

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol) => Box(_geom.Hyperbola, u1, u2, tol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for finite hyperbola arc, adding per-coordinate extrema.</summary>
    private static void ComputeHyperbolaBox(in Hyperbola3D hypr, double t1, double t2, ref Box box)
    {
        Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(t1, hypr);
        Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(t2, hypr);
        box.Add(p1);
        box.Add(p2);

        if (t1 * t2 < 0.0)
        {
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, hypr));
        }

        Frame3D pos = hypr.Position;
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        double rmaj = hypr.MajorRadius;
        double rmin = hypr.MinorRadius;
        double eps = Epsilon(1.0);

        for (int i = 1; i <= 3; ++i)
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

    /// <summary>Compute bounding box for a hyperbola arc [theU1, theU2] defined by gp_Hypr.</summary>
    public static Box Box(in Hyperbola3D hypr, double u1, double u2, double tol)
    {
        Box box = new();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                throw new FailureException("GeomBndLib_Hyperbola::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                box.OpenXmax();
                box.OpenYmax();
                box.OpenZmax();
            }
            else
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, hypr));
            }
            box.OpenXmin();
            box.OpenYmin();
            box.OpenZmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                box.OpenXmin();
                box.OpenYmin();
                box.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                throw new FailureException("GeomBndLib_Hyperbola::Box - bad parameter");
            }
            else
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, hypr));
            }
            box.OpenXmax();
            box.OpenYmax();
            box.OpenZmax();
        }
        else
        {
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u1, hypr));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                box.OpenXmin();
                box.OpenYmin();
                box.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                box.OpenXmax();
                box.OpenYmax();
                box.OpenZmax();
            }
            else
            {
                ComputeHyperbolaBox(hypr, u1, u2, ref box);
            }
        }
        box.Enlarge(tol);
        return box;
    }
}
