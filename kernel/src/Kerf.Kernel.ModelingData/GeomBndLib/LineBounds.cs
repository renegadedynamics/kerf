// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Line.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D line (Geom_Line). Handles infinite parameters by opening the box in appropriate
/// directions. The static method accepting gp_Lin can be used directly without constructing a Geom_Line.
/// </summary>
internal sealed class LineBounds : CurveBounds.ICurveEvaluator
{
    private readonly LineCurve3D _geom;

    public LineBounds(LineCurve3D line)
    {
        _geom = line;
    }

    public LineCurve3D Geometry() => _geom;

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for line segment [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol) => Box(_geom.Line, u1, u2, tol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for a line segment [theU1, theU2] defined by gp_Lin.</summary>
    public static Box Box(in Line3D lin, double u1, double u2, double tol)
    {
        Box box = new();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                throw new FailureException("GeomBndLib_Line::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                InfiniteHelpers.OpenMinMax(lin.Direction, ref box);
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, lin));
            }
            else
            {
                InfiniteHelpers.OpenMin(lin.Direction, ref box);
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, lin));
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                InfiniteHelpers.OpenMinMax(lin.Direction, ref box);
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, lin));
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                throw new FailureException("GeomBndLib_Line::Box - bad parameter");
            }
            else
            {
                InfiniteHelpers.OpenMax(lin.Direction, ref box);
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, lin));
            }
        }
        else
        {
            box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u1, lin));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(u2))
            {
                InfiniteHelpers.OpenMin(lin.Direction, ref box);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(u2))
            {
                InfiniteHelpers.OpenMax(lin.Direction, ref box);
            }
            else
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.Value(u2, lin));
            }
        }
        box.Enlarge(tol);
        return box;
    }
}
