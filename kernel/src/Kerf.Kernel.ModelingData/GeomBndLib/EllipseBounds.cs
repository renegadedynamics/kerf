// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Ellipse.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D ellipse (Geom_Ellipse), with analytical per-coordinate extrema. The static methods
/// accepting gp_Elips can be used directly without constructing a Geom_Ellipse.
/// </summary>
internal sealed class EllipseBounds : CurveBounds.ICurveEvaluator
{
    private readonly EllipseCurve3D _geom;

    public EllipseBounds(EllipseCurve3D ellipse)
    {
        _geom = ellipse;
    }

    public EllipseCurve3D Geometry() => _geom;

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for full ellipse.</summary>
    public Box Box(double tol) => Box(_geom.Ellipse, tol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol) => Box(_geom.Ellipse, u1, u2, tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for a full ellipse defined by gp_Elips.</summary>
    public static Box Box(in Ellipse3D elips, double tol)
    {
        Box box = new();
        double majR = elips.MajorRadius;
        double minR = elips.MinorRadius;
        Xyz o = elips.Location.XYZ();
        Xyz xd = elips.XAxis.Direction.XYZ();
        Xyz yd = elips.YAxis.Direction.XYZ();

        // For a full ellipse, compute per-coordinate analytical extrema.
        Span<double> min = stackalloc double[3];
        Span<double> max = stackalloc double[3];
        for (int k = 1; k <= 3; ++k)
        {
            double xk = xd.Coord(k);
            double yk = yd.Coord(k);
            double amp = Math.Sqrt(majR * majR * xk * xk + minR * minR * yk * yk);
            min[k - 1] = o.Coord(k) - amp;
            max[k - 1] = o.Coord(k) + amp;
        }
        box.Update(min[0], min[1], min[2], max[0], max[1], max[2]);
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute bounding box for an ellipse arc [theU1, theU2] defined by gp_Elips.</summary>
    public static Box Box(in Ellipse3D elips, double u1, double u2, double tol)
    {
        Box box = new();
        double majR = elips.MajorRadius;
        double minR = elips.MinorRadius;
        Xyz o = elips.Location.XYZ();
        Xyz xd = elips.XAxis.Direction.XYZ();
        Xyz yd = elips.YAxis.Direction.XYZ();

        double period = 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (u2 - u1 >= period)
        {
            return Box(elips, tol);
        }

        double aU1 = u1, aU2 = u2;
        double aTol = Epsilon(1.0);
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, aTol, ref aU1, ref aU2);

        // Add arc endpoints.
        Frame3D pos = elips.Position;
        box.Add(global::Kerf.Engine.Geometry.ElCLib.EllipseValue(aU1, pos, majR, minR));
        box.Add(global::Kerf.Engine.Geometry.ElCLib.EllipseValue(aU2, pos, majR, minR));

        // For each coordinate, check if the extremal parameter lies within the arc.
        for (int k = 1; k <= 3; ++k)
        {
            double xk = xd.Coord(k);
            double yk = yd.Coord(k);

            double tExtrMin;
            if (Math.Abs(xk) > Constants.Resolution())
            {
                tExtrMin = Math.Atan((minR * yk) / (majR * xk));
                tExtrMin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tExtrMin, 0.0, 2.0 * M_PI);
            }
            else
            {
                tExtrMin = M_PI / 2.0;
            }
            double tExtrMax = tExtrMin <= M_PI ? tExtrMin + M_PI : tExtrMin - M_PI;

            double valMin = majR * Math.Cos(tExtrMin) * xk + minR * Math.Sin(tExtrMin) * yk + o.Coord(k);
            double valMax = majR * Math.Cos(tExtrMax) * xk + minR * Math.Sin(tExtrMax) * yk + o.Coord(k);
            if (valMin > valMax)
            {
                (tExtrMin, tExtrMax) = (tExtrMax, tExtrMin);
            }

            double tk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tExtrMin, aU1, aU1 + 2.0 * M_PI);
            if (tk >= aU1 && tk <= aU2)
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.EllipseValue(tExtrMin, pos, majR, minR));
            }
            tk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tExtrMax, aU1, aU1 + 2.0 * M_PI);
            if (tk >= aU1 && tk <= aU2)
            {
                box.Add(global::Kerf.Engine.Geometry.ElCLib.EllipseValue(tExtrMax, pos, majR, minR));
            }
        }
        box.Enlarge(tol);
        return box;
    }
}
