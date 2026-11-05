// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Circ2d: from an axis and a radius, concentric to a circle through a point or at a signed
/// offset, through three points, from a center and a radius, or from a center and a point.
/// </summary>
internal sealed class MakeCirc2d : Root
{
    private Circle2D _theCirc2d = new();

    /// <summary>
    /// A circle through three points, oriented so that they come in order. If all three are coincident, the result is
    /// a circle of zero radius at P1.
    /// </summary>
    public MakeCirc2d(in Point2D p1, in Point2D p2, in Point2D p3)
    {
        Direction2D dirx = new(Direction2D.D.X);
        // Three cases:
        // 1) All three points are coincident: result is a circle centered at P1 with zero radius.
        // 2) Two of the three points are coincident: create the perpendicular bisector of the two
        //    distinct points and the line through them. The center is their intersection.
        // 3) All three points are distinct: create the perpendicular bisector of P1P2 and
        //    the perpendicular bisector of P1P3. The center is their intersection and
        //    the radius is the distance from that center to any of the three points.
        double dist1 = p1.DistanceTo(p2);
        double dist2 = p1.DistanceTo(p3);
        double dist3 = p2.DistanceTo(p3);
        if ((dist1 < Constants.Resolution()) && (dist2 < Constants.Resolution()) && (dist3 < Constants.Resolution()))
        {
            _theCirc2d = new Circle2D(new Axis2D(p1, dirx), 0.0);
            _theError = Done;
        }
        else
        {
            Line2D l1 = new();
            Line2D l2 = new();
            double x1, y1, x2, y2, x3, y3;
            p1.Coord(out x1, out y1);
            p2.Coord(out x2, out y2);
            p3.Coord(out x3, out y3);
            if (dist1 > Constants.Resolution())
            {
                l1 = new Line2D(new Point2D((p1.XY() + p2.XY()) / 2.0), new Direction2D(p1.Y - p2.Y, p2.X - p1.X));
            }
            if (dist2 > Constants.Resolution())
            {
                l2 = new Line2D(new Point2D((p1.XY() + p3.XY()) / 2.0), new Direction2D(p1.Y - p3.Y, p3.X - p1.X));
            }
            if (dist2 <= Constants.Resolution())
            {
                l2 = new Line2D(p1, new Direction2D(p1.Y - p2.Y, p2.X - p1.X));
            }
            else if (dist1 <= Constants.Resolution())
            {
                l1 = new Line2D(p1, new Direction2D(p1.Y - p3.Y, p3.X - p1.X));
            }
            else if (dist3 <= Constants.Resolution())
            {
                l2 = new Line2D(p1, new Direction2D(p1.Y - p2.Y, p2.X - p1.X));
            }
            AnalyticIntersection intp = new(l1, l2);
            if (intp.IsDone)
            {
                if (!intp.IsEmpty)
                {
                    Point2D pInt = intp.Point(1).Value();
                    dist1 = p1.DistanceTo(pInt);
                    dist2 = p2.DistanceTo(pInt);
                    dist3 = p3.DistanceTo(pInt);
                    double xc, yc;
                    pInt.Coord(out xc, out yc);
                    Direction2D d1 = new(x1 - xc, y1 - yc);
                    Direction2D d2 = new(xc - x3, yc - y3);
                    _theCirc2d = new Circle2D(new Frame2D(pInt, d1, d2), (dist1 + dist2 + dist3) / 3.0);
                    double alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(_theCirc2d, p1);
                    double alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(_theCirc2d, p2);
                    double alpha3 = global::Kerf.Engine.Geometry.ElCLib.Parameter(_theCirc2d, p3);
                    if ((alpha1 > alpha2) || (alpha2 > alpha3))
                    {
                        _theCirc2d = _theCirc2d.Reversed();
                    }
                    _theError = Done;
                }
                else
                {
                    // OCCT leaves TheError unset here (collinear points: parallel bisectors), so its status is
                    // undefined; the port reports the failure rather than a .NET default of gce_Done.
                    _theError = IntersectionError;
                }
            }
            else
            {
                _theError = IntersectionError;
            }
        }
    }

    /// <summary>
    /// A circle from its X axis (whose location is the center), a radius and a sense of parametrization; fails with
    /// gce_NegativeRadius if Radius is negative.
    /// </summary>
    public MakeCirc2d(in Axis2D xAxis, double radius, bool sense = true)
    {
        if (radius >= 0.0)
        {
            _theCirc2d = new Circle2D(xAxis, radius, sense);
            _theError = Done;
        }
        else
        {
            _theError = NegativeRadius;
        }
    }

    /// <summary>A circle from its axis system and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public MakeCirc2d(in Frame2D axis, double radius)
    {
        if (radius >= 0.0)
        {
            _theCirc2d = new Circle2D(axis, radius);
            _theError = Done;
        }
        else
        {
            _theError = NegativeRadius;
        }
    }

    /// <summary>A circle from its center and radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public MakeCirc2d(in Point2D center, double radius, bool sense = true)
    {
        if (radius >= 0.0)
        {
            _theCirc2d = new Circle2D(new Axis2D(center, new Direction2D(Direction2D.D.X)), radius, sense);
            _theError = Done;
        }
        else
        {
            _theError = NegativeRadius;
        }
    }

    /// <summary>A circle from its center and a point on it.</summary>
    public MakeCirc2d(in Point2D center, in Point2D point, bool sense = true)
    {
        _theCirc2d = new Circle2D(new Axis2D(center, new Direction2D(Direction2D.D.X)), point.DistanceTo(center), sense);
        _theError = Done;
    }

    /// <summary>A circle concentric to Circ through Point.</summary>
    public MakeCirc2d(in Circle2D circ, in Point2D point)
    {
        _theCirc2d = new Circle2D(circ.Axis, point.DistanceTo(circ.Location));
        _theError = Done;
    }

    /// <summary>A circle concentric to Circ, of radius Abs(Circ.Radius() + Dist1).</summary>
    public MakeCirc2d(in Circle2D circ, double dist1)
    {
        _theCirc2d = new Circle2D(circ.Axis, Math.Abs(circ.Radius + dist1));
        _theError = Done;
    }

    /// <summary>The constructed circle.</summary>
    /// <exception cref="NotDoneException">No circle was constructed.</exception>
    public Circle2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeCirc2d::Value() - no result");
        return _theCirc2d;
    }

    /// <summary>Alias for Value().</summary>
    public Circle2D Operator() => Value();

    /// <summary>The constructed circle (OCCT's implicit conversion operator).</summary>
    public static explicit operator Circle2D(MakeCirc2d maker) => maker.Operator();
}
