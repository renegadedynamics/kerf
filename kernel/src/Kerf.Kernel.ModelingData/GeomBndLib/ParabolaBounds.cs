// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D parabola (Geom_Parabola): the arc ends and the vertex, opened for infinite
/// parameters. The static method accepting gp_Parab can be used directly.
/// </summary>
internal sealed class ParabolaBounds : CurveBounds.ICurveEvaluator
{
    private readonly ParabolaCurve3D _geom;

    public ParabolaBounds(ParabolaCurve3D parabola)
    {
        _geom = parabola;
    }

    public ParabolaCurve3D Geometry() => _geom;

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol) => Box(_geom.Parabola, u1, u2, tol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for a parabola arc [theU1, theU2] defined by gp_Parab.</summary>
    public static Box Box(in Parabola3D parab, double u1, double u2, double tol)
    {
        Box box = new();
        // Kerf: one end infinite. OCCT opened the three minima or maxima by the end's sign, but the point goes to
        // +infinity along the axis X whichever way (its U^2 term), and along +-Y only where X has no part: the box
        // missed the curve on those sides. Each axis opens towards where the point goes, and its other side is
        // bounded by the finite end and the coordinate's extremum (decision 23104).
        if (InfiniteHelpers.HalfInfinite(u1, u2, out double finite, out int towards))
        {
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(finite, parab));
            AddCoordinateExtrema(parab, towards < 0 ? double.NegativeInfinity : finite, towards < 0 ? finite : double.PositiveInfinity, ref box);
            Xyz xDir = parab.Position.XDirection.XYZ(), yDir = parab.Position.YDirection.XYZ();
            InfiniteHelpers.OpenAlong(new Xyz(xDir.X() != 0.0 ? xDir.X() : towards * yDir.X(), xDir.Y() != 0.0 ? xDir.Y() : towards * yDir.Y(),
                                              xDir.Z() != 0.0 ? xDir.Z() : towards * yDir.Z()), ref box);
            box.Enlarge(tol);
            return box;
        }
        // both ends infinite: the whole curve, open on every side, as OCCT has it; the same end twice is no arc
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1))
        {
            if ((u1 < 0.0) == (u2 < 0.0))
            {
                throw new FailureException("GeomBndLib_Parabola::Box - bad parameter");
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
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u1, parab));
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, parab));
            AddCoordinateExtrema(parab, u1, u2, ref box);
        }
        box.Enlarge(tol);
        return box;
    }

    /// <summary>
    /// Adds the points of the arc [U1, U2] where a coordinate is extreme: x_k(U) = O_k + U^2/(4F).X_k + U.Y_k is
    /// extreme at U = -2F.Y_k/X_k. OCCT added only the vertex (U = 0), which misses them when the parabola is turned
    /// in its plane (occt-issues.md; decision 1429).
    /// </summary>
    internal static void AddCoordinateExtrema(in Parabola3D parab, double u1, double u2, ref Box box)
    {
        double focal = parab.Focal;
        if (Math.Abs(focal) <= Constants.Resolution())
        {
            return; // a line: its ends bound it
        }
        Xyz xDir = parab.Position.XDirection.XYZ();
        Xyz yDir = parab.Position.YDirection.XYZ();
        for (int k = 1; k <= 3; k++)
        {
            double xk = xDir.Coord(k);
            if (Math.Abs(xk) <= Constants.Resolution())
            {
                continue; // linear in U: extreme at the ends
            }
            double u = -2.0 * focal * yDir.Coord(k) / xk;
            if (u > u1 && u < u2)
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u, parab));
            }
        }
    }
}
