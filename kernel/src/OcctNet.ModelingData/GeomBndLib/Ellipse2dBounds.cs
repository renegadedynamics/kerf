// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Ellipse2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Ellipse2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 2D ellipse (Geom2d_Ellipse), with analytical per-coordinate extrema. The static
/// methods accepting gp_Elips2d can be used directly without constructing a Geom2d_Ellipse.
/// </summary>
public sealed class Ellipse2dBounds : Curve2dBounds.ICurve2dEvaluator
{
    private readonly EllipseCurve2D _geom;

    public Ellipse2dBounds(EllipseCurve2D ellipse)
    {
        _geom = ellipse;
    }

    public EllipseCurve2D Geometry() => _geom;

    Curve2D? Curve2dBounds.ICurve2dEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for full ellipse.</summary>
    public Box2d Box(double tol) => Box(_geom.Elips2d(), tol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double u1, double u2, double tol) => Box(_geom.Elips2d(), u1, u2, tol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box2d BoxOptimal(double u1, double u2, double tol) => Box(u1, u2, tol);

    /// <summary>Compute bounding box for a full ellipse defined by gp_Elips2d.</summary>
    public static Box2d Box(in Ellipse2D elips, double tol)
    {
        Box2d box = new();
        double majR = elips.MajorRadius();
        double minR = elips.MinorRadius();
        Xy o = elips.Location().XY();
        Xy xd = elips.XAxis().Direction().XY();
        Xy yd = elips.YAxis().Direction().XY();

        Span<double> min = stackalloc double[2];
        Span<double> max = stackalloc double[2];
        for (int k = 1; k <= 2; ++k)
        {
            double xk = xd.Coord(k);
            double yk = yd.Coord(k);
            double amp = Math.Sqrt(majR * majR * xk * xk + minR * minR * yk * yk);
            min[k - 1] = o.Coord(k) - amp;
            max[k - 1] = o.Coord(k) + amp;
        }
        box.Update(min[0], min[1], max[0], max[1]);
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute bounding box for an ellipse arc [theU1, theU2] defined by gp_Elips2d.</summary>
    public static Box2d Box(in Ellipse2D elips, double u1, double u2, double tol)
    {
        Box2d box = new();
        double majR = elips.MajorRadius();
        double minR = elips.MinorRadius();
        Xy o = elips.Location().XY();
        Xy xd = elips.XAxis().Direction().XY();
        Xy yd = elips.YAxis().Direction().XY();

        if (u2 - u1 >= 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return Box(elips, tol);
        }

        // Add arc endpoints (OCCT's lambda aEval).
        Point2D aEval(double t) => new(o + majR * Math.Cos(t) * xd + minR * Math.Sin(t) * yd);

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
                tExtrMin = Math.Atan((minR * yk) / (majR * xk));
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
