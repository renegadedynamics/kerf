// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D parabola (Geom_Parabola): the arc ends and the vertex, opened for infinite
/// parameters. The static method accepting gp_Parab can be used directly.
/// </summary>
public sealed class ParabolaBounds : CurveBounds.ICurveEvaluator
{
    private readonly ParabolaCurve3D _geom;

    public ParabolaBounds(ParabolaCurve3D parabola)
    {
        _geom = parabola;
    }

    public ParabolaCurve3D Geometry() => _geom;

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol) => Box(_geom.Parab(), u1, u2, tol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double tol) => Box(_geom.FirstParameter(), _geom.LastParameter(), tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for a parabola arc [theU1, theU2] defined by gp_Parab.</summary>
    public static Box Box(in Parabola3D parab, double u1, double u2, double tol)
    {
        Box box = new();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                throw new FailureException("GeomBndLib_Parabola::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                box.OpenXmax();
                box.OpenYmax();
                box.OpenZmax();
            }
            else
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, parab));
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
                throw new FailureException("GeomBndLib_Parabola::Box - bad parameter");
            }
            else
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, parab));
            }
            box.OpenXmax();
            box.OpenYmax();
            box.OpenZmax();
        }
        else
        {
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u1, parab));
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
                // OCCT adds only the ends and the vertex: the coordinate extrema of a parabola turned in its plane
                // (at U = -2F.Yk/Xk) are missed, so the box may not contain the arc (occt-issues.md).
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, parab));
                if (u1 * u2 < 0)
                {
                    box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, parab));
                }
            }
        }
        box.Enlarge(tol);
        return box;
    }
}
