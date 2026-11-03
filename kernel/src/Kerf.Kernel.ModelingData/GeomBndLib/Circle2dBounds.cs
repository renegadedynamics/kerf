// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Circle2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Circle2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 2D circle (Geom2d_Circle), with analytical per-coordinate extrema. The static methods
/// accepting gp_Circ2d can be used directly without constructing a Geom2d_Circle.
/// </summary>
public sealed class Circle2dBounds : Curve2dBounds.ICurve2dEvaluator
{
    private readonly CircleCurve2D _geom;

    public Circle2dBounds(CircleCurve2D circle)
    {
        _geom = circle;
    }

    public CircleCurve2D Geometry() => _geom;

    Curve2D? Curve2dBounds.ICurve2dEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for full circle.</summary>
    public Box2d Box(double tol) => Box(_geom.Circle, tol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double u1, double u2, double tol) => Box(_geom.Circle, u1, u2, tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box2d BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for a full circle defined by gp_Circ2d.</summary>
    public static Box2d Box(in Circle2D circ, double tol)
    {
        Box2d box = new();
        double r = circ.Radius;
        Xy o = circ.Location.XY();
        Xy xd = circ.XAxis.Direction.XY();
        Xy yd = circ.YAxis.Direction.XY();

        // For a full circle, compute per-coordinate analytical extrema.
        Span<double> min = stackalloc double[2];
        Span<double> max = stackalloc double[2];
        for (int k = 1; k <= 2; ++k)
        {
            double xk = xd.Coord(k);
            double yk = yd.Coord(k);
            double amp = Math.Sqrt(r * r * xk * xk + r * r * yk * yk);
            min[k - 1] = o.Coord(k) - amp;
            max[k - 1] = o.Coord(k) + amp;
        }
        box.Update(min[0], min[1], max[0], max[1]);
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute bounding box for a circle arc [theU1, theU2] defined by gp_Circ2d.</summary>
    public static Box2d Box(in Circle2D circ, double u1, double u2, double tol)
    {
        Box2d box = new();
        double r = circ.Radius;
        Xy o = circ.Location.XY();
        Xy xd = circ.XAxis.Direction.XY();
        Xy yd = circ.YAxis.Direction.XY();

        if (u2 - u1 >= 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return Box(circ, tol);
        }

        // Add arc endpoints (OCCT's lambda aEval).
        Point2D aEval(double t) => new(o + r * Math.Cos(t) * xd + r * Math.Sin(t) * yd);

        box.Add(aEval(u1));
        box.Add(aEval(u2));

        // For each coordinate, check if the extremal parameter lies within the arc.
        for (int k = 1; k <= 2; ++k)
        {
            double xk = xd.Coord(k);
            double yk = yd.Coord(k);

            double tExtrMin;
            if (Math.Abs(xk) > Constants.Resolution())
            {
                tExtrMin = Math.Atan(yk / xk);
                tExtrMin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tExtrMin, 0.0, 2.0 * M_PI);
            }
            else
            {
                tExtrMin = M_PI / 2.0;
            }
            double tExtrMax = tExtrMin <= M_PI ? tExtrMin + M_PI : tExtrMin - M_PI;

            double tk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tExtrMin, u1, u1 + 2.0 * M_PI);
            if (tk >= u1 && tk <= u2)
            {
                box.Add(aEval(tExtrMin));
            }
            tk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tExtrMax, u1, u1 + 2.0 * M_PI);
            if (tk >= u1 && tk <= u2)
            {
                box.Add(aEval(tExtrMax));
            }
        }
        box.Enlarge(tol);
        return box;
    }
}
