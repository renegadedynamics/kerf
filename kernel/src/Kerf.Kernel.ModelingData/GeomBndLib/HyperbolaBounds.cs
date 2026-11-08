// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D hyperbola (Geom_Hyperbola): the arc ends and the per-coordinate extrema, opened
/// for infinite parameters. The static method accepting gp_Hypr can be used directly.
/// </summary>
internal sealed class HyperbolaBounds : CurveBounds.ICurveEvaluator
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
    // Kerf: the points of the half-line from the finite end where a coordinate is extreme (decision 1964)
    private static void AddHalfLineExtrema(in Hyperbola3D hypr, double finite, int towards, ref Box box)
    {
        Xyz xDir = hypr.Position.XDirection.XYZ(), yDir = hypr.Position.YDirection.XYZ();
        for (int i = 1; i <= 3; ++i)
        {
            double a = hypr.MinorRadius * yDir.Coord(i), b = hypr.MajorRadius * xDir.Coord(i);
            double abp = Math.Abs(a + b), bam = Math.Abs(b - a);
            if (abp < Epsilon(1.0) || bam < Epsilon(1.0))
            {
                continue;
            }
            double t3 = 0.5 * Math.Log(bam / abp);
            if (towards < 0 ? t3 < finite : t3 > finite)
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(t3, hypr));
            }
        }
    }

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
        // Kerf: one end infinite. OCCT opened the three minima or maxima by the end's sign, but the point goes along
        // R.X + s.r.Y (cosh and sinh both grow as e^|U| / 2), so the box missed the branch on the sides it goes
        // towards. Each axis opens that way; where the two cancel, the coordinate falls back towards the centre's, which
        // bounds it with the finite end (decision 1964).
        if (InfiniteHelpers.HalfInfinite(u1, u2, out double finite, out int towards))
        {
            Point3D end = global::Kerf.Engine.Geometry.ElCLib.Value(finite, hypr);
            box.Add(end);
            AddHalfLineExtrema(hypr, finite, towards, ref box);
            Xyz along = (hypr.MajorRadius * hypr.Position.XDirection.XYZ()) + (towards * hypr.MinorRadius * hypr.Position.YDirection.XYZ());
            Xyz back = hypr.Location.XYZ() - end.XYZ();
            InfiniteHelpers.OpenAlong(new Xyz(along.X() != 0.0 ? along.X() : back.X(), along.Y() != 0.0 ? along.Y() : back.Y(),
                                              along.Z() != 0.0 ? along.Z() : back.Z()), ref box);
            box.Enlarge(tol);
            return box;
        }
        // both ends infinite: the whole curve, open on every side, as OCCT has it; the same end twice is no arc
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1))
        {
            if ((u1 < 0.0) == (u2 < 0.0))
            {
                throw new FailureException("GeomBndLib_Hyperbola::Box - bad parameter");
            }
            box.OpenXmin();
            box.OpenYmin();
            box.OpenZmin();
            box.OpenXmax();
            box.OpenYmax();
            box.OpenZmax();
        }
        else
        {
            ComputeHyperbolaBox(hypr, u1, u2, ref box);
        }
        box.Enlarge(tol);
        return box;
    }
}
