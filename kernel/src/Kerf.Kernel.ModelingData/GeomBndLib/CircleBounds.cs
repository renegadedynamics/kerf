// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Circle.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Circle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D circle (Geom_Circle), with analytical per-coordinate extrema. The static methods
/// accepting gp_Circ can be used directly without constructing a Geom_Circle.
/// </summary>
internal sealed class CircleBounds : CurveBounds.ICurveEvaluator
{
    private readonly CircleCurve3D _geom;

    public CircleBounds(CircleCurve3D circle)
    {
        _geom = circle;
    }

    public CircleCurve3D Geometry() => _geom;

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for full circle.</summary>
    public Box Box(double tol) => Box(_geom.Circle, tol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol) => Box(_geom.Circle, u1, u2, tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for a full circle defined by gp_Circ.</summary>
    public static Box Box(in Circle3D circ, double tol)
    {
        Box box = new();
        double r = circ.Radius;
        Xyz o = circ.Location.XYZ();
        Xyz xd = circ.XAxis.Direction.XYZ();
        Xyz yd = circ.YAxis.Direction.XYZ();

        // For a full circle, compute per-coordinate analytical extrema.
        Span<double> min = stackalloc double[3];
        Span<double> max = stackalloc double[3];
        for (int k = 1; k <= 3; ++k)
        {
            double xk = xd.Coord(k);
            double yk = yd.Coord(k);
            double amp = Math.Sqrt(r * r * xk * xk + r * r * yk * yk);
            min[k - 1] = o.Coord(k) - amp;
            max[k - 1] = o.Coord(k) + amp;
        }
        box.Update(min[0], min[1], min[2], max[0], max[1], max[2]);
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute bounding box for a circle arc [theU1, theU2] defined by gp_Circ.</summary>
    public static Box Box(in Circle3D circ, double u1, double u2, double tol)
    {
        Box box = new();
        double r = circ.Radius;
        Xyz o = circ.Location.XYZ();
        Xyz xd = circ.XAxis.Direction.XYZ();
        Xyz yd = circ.YAxis.Direction.XYZ();

        double period = 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (u2 - u1 >= period)
        {
            return Box(circ, tol);
        }

        double aU1 = u1, aU2 = u2;
        double aTol = Epsilon(1.0);
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, aTol, ref aU1, ref aU2);

        // Add arc endpoints.
        Frame3D pos = circ.Position;
        box.Add(global::Kerf.Engine.Geometry.ElCLib.CircleValue(aU1, pos, r));
        box.Add(global::Kerf.Engine.Geometry.ElCLib.CircleValue(aU2, pos, r));

        // For each coordinate, check if the extremal parameter lies within the arc.
        for (int k = 1; k <= 3; ++k)
        {
            double xk = xd.Coord(k);
            double yk = yd.Coord(k);

            // Extremal parameter for min: tan(t) = (R * Yk) / (R * Xk)
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

            double valMin = r * Math.Cos(tExtrMin) * xk + r * Math.Sin(tExtrMin) * yk + o.Coord(k);
            double valMax = r * Math.Cos(tExtrMax) * xk + r * Math.Sin(tExtrMax) * yk + o.Coord(k);
            if (valMin > valMax)
            {
                (valMin, valMax) = (valMax, valMin);
                (tExtrMin, tExtrMax) = (tExtrMax, tExtrMin);
            }

            double tk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tExtrMin, aU1, aU1 + 2.0 * M_PI);
            if (tk >= aU1 && tk <= aU2)
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.CircleValue(tExtrMin, pos, r));
            }
            tk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tExtrMax, aU1, aU1 + 2.0 * M_PI);
            if (tk >= aU1 && tk <= aU2)
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.CircleValue(tExtrMax, pos, r));
            }
        }
        box.Enlarge(tol);
        return box;
    }
}
