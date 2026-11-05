// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 2D parabola (Geom2d_Parabola): the arc ends and the vertex, opened for infinite
/// parameters. The static method accepting gp_Parab2d can be used directly.
/// </summary>
internal sealed class Parabola2dBounds : Curve2dBounds.ICurve2dEvaluator
{
    private readonly ParabolaCurve2D _geom;

    public Parabola2dBounds(ParabolaCurve2D parabola)
    {
        _geom = parabola;
    }

    public ParabolaCurve2D Geometry() => _geom;

    Curve2D? Curve2dBounds.ICurve2dEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double u1, double u2, double tol) => Box(_geom.Parabola, u1, u2, tol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box2d BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for a 2D parabola arc [theU1, theU2] defined by gp_Parab2d.</summary>
    public static Box2d Box(in Parabola2D parab, double u1, double u2, double tol)
    {
        Box2d box = new();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                throw new FailureException("GeomBndLib_Parabola2d::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                box.OpenXmax();
                box.OpenYmax();
            }
            else
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, parab));
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
                throw new FailureException("GeomBndLib_Parabola2d::Box - bad parameter");
            }
            else
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, parab));
            }
            box.OpenXmax();
            box.OpenYmax();
        }
        else
        {
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u1, parab));
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
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, parab));
                AddCoordinateExtrema(parab, u1, u2, ref box);
            }
        }
        box.Enlarge(tol);
        return box;
    }

    /// <summary>
    /// Adds the points of the arc [U1, U2] where a coordinate is extreme: x_k(U) = O_k + U^2/(4F).X_k + U.Y_k is
    /// extreme at U = -2F.Y_k/X_k. OCCT added only the vertex (U = 0), which misses them when the parabola is turned
    /// in its plane (occt-issues.md; decision 1429).
    /// </summary>
    internal static void AddCoordinateExtrema(in Parabola2D parab, double u1, double u2, ref Box2d box)
    {
        double focal = parab.Focal;
        if (Math.Abs(focal) <= Constants.Resolution())
        {
            return; // a line: its ends bound it
        }
        Xy xDir = parab.Axis.XDirection.XY();
        Xy yDir = parab.Axis.YDirection.XY();
        for (int k = 1; k <= 2; k++)
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
