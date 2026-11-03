// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Circ2d: from an axis and a radius, concentric to a circle through a point or at a signed
/// offset, through three points, from a center and a radius, or from a center and a point.
/// </summary>
public sealed class gce_MakeCirc2d : gce_Root
{
    private Circle2D TheCirc2d = new();

    /// <summary>
    /// A circle through three points, oriented so that they come in order. If all three are coincident, the result is
    /// a circle of zero radius at P1.
    /// </summary>
    public gce_MakeCirc2d(in Point2D P1, in Point2D P2, in Point2D P3)
    {
        Direction2D dirx = new(Direction2D.D.X);
        // Three cases:
        // 1) All three points are coincident: result is a circle centered at P1 with zero radius.
        // 2) Two of the three points are coincident: create the perpendicular bisector of the two
        //    distinct points and the line through them. The center is their intersection.
        // 3) All three points are distinct: create the perpendicular bisector of P1P2 and
        //    the perpendicular bisector of P1P3. The center is their intersection and
        //    the radius is the distance from that center to any of the three points.
        double dist1 = P1.Distance(P2);
        double dist2 = P1.Distance(P3);
        double dist3 = P2.Distance(P3);
        if ((dist1 < Constants.Resolution()) && (dist2 < Constants.Resolution()) && (dist3 < Constants.Resolution()))
        {
            TheCirc2d = new Circle2D(new Axis2D(P1, dirx), 0.0);
            TheError = gce_Done;
        }
        else
        {
            Line2D L1 = new();
            Line2D L2 = new();
            double x1, y1, x2, y2, x3, y3;
            P1.Coord(out x1, out y1);
            P2.Coord(out x2, out y2);
            P3.Coord(out x3, out y3);
            if (dist1 > Constants.Resolution())
            {
                L1 = new Line2D(new Point2D((P1.XY() + P2.XY()) / 2.0), new Direction2D(P1.Y() - P2.Y(), P2.X() - P1.X()));
            }
            if (dist2 > Constants.Resolution())
            {
                L2 = new Line2D(new Point2D((P1.XY() + P3.XY()) / 2.0), new Direction2D(P1.Y() - P3.Y(), P3.X() - P1.X()));
            }
            if (dist2 <= Constants.Resolution())
            {
                L2 = new Line2D(P1, new Direction2D(P1.Y() - P2.Y(), P2.X() - P1.X()));
            }
            else if (dist1 <= Constants.Resolution())
            {
                L1 = new Line2D(P1, new Direction2D(P1.Y() - P3.Y(), P3.X() - P1.X()));
            }
            else if (dist3 <= Constants.Resolution())
            {
                L2 = new Line2D(P1, new Direction2D(P1.Y() - P2.Y(), P2.X() - P1.X()));
            }
            IntAna2d_AnaIntersection Intp = new(L1, L2);
            if (Intp.IsDone())
            {
                if (!Intp.IsEmpty())
                {
                    Point2D pInt = Intp.Point(1).Value();
                    dist1 = P1.Distance(pInt);
                    dist2 = P2.Distance(pInt);
                    dist3 = P3.Distance(pInt);
                    double xc, yc;
                    pInt.Coord(out xc, out yc);
                    Direction2D d1 = new(x1 - xc, y1 - yc);
                    Direction2D d2 = new(xc - x3, yc - y3);
                    TheCirc2d = new Circle2D(new Frame2D(pInt, d1, d2), (dist1 + dist2 + dist3) / 3.0);
                    double Alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(TheCirc2d, P1);
                    double Alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(TheCirc2d, P2);
                    double Alpha3 = global::Kerf.Engine.Geometry.ElCLib.Parameter(TheCirc2d, P3);
                    if ((Alpha1 > Alpha2) || (Alpha2 > Alpha3))
                    {
                        TheCirc2d.Reverse();
                    }
                    TheError = gce_Done;
                }
                else
                {
                    // OCCT leaves TheError unset here (collinear points: parallel bisectors), so its status is
                    // undefined; the port reports the failure rather than a .NET default of gce_Done.
                    TheError = gce_IntersectionError;
                }
            }
            else
            {
                TheError = gce_IntersectionError;
            }
        }
    }

    /// <summary>
    /// A circle from its X axis (whose location is the center), a radius and a sense of parametrization; fails with
    /// gce_NegativeRadius if Radius is negative.
    /// </summary>
    public gce_MakeCirc2d(in Axis2D XAxis, double Radius, bool Sense = true)
    {
        if (Radius >= 0.0)
        {
            TheCirc2d = new Circle2D(XAxis, Radius, Sense);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_NegativeRadius;
        }
    }

    /// <summary>A circle from its axis system and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCirc2d(in Frame2D Axis, double Radius)
    {
        if (Radius >= 0.0)
        {
            TheCirc2d = new Circle2D(Axis, Radius);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_NegativeRadius;
        }
    }

    /// <summary>A circle from its center and radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCirc2d(in Point2D Center, double Radius, bool Sense = true)
    {
        if (Radius >= 0.0)
        {
            TheCirc2d = new Circle2D(new Axis2D(Center, new Direction2D(Direction2D.D.X)), Radius, Sense);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_NegativeRadius;
        }
    }

    /// <summary>A circle from its center and a point on it.</summary>
    public gce_MakeCirc2d(in Point2D Center, in Point2D Point, bool Sense = true)
    {
        TheCirc2d = new Circle2D(new Axis2D(Center, new Direction2D(Direction2D.D.X)), Point.Distance(Center), Sense);
        TheError = gce_Done;
    }

    /// <summary>A circle concentric to Circ through Point.</summary>
    public gce_MakeCirc2d(in Circle2D Circ, in Point2D Point)
    {
        TheCirc2d = new Circle2D(Circ.Axis(), Point.Distance(Circ.Location()));
        TheError = gce_Done;
    }

    /// <summary>A circle concentric to Circ, of radius Abs(Circ.Radius() + Dist1).</summary>
    public gce_MakeCirc2d(in Circle2D Circ, double Dist1)
    {
        TheCirc2d = new Circle2D(Circ.Axis(), Math.Abs(Circ.Radius() + Dist1));
        TheError = gce_Done;
    }

    /// <summary>The constructed circle.</summary>
    /// <exception cref="NotDoneException">No circle was constructed.</exception>
    public Circle2D Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeCirc2d::Value() - no result");
        return TheCirc2d;
    }

    /// <summary>Alias for Value().</summary>
    public Circle2D Operator() => Value();

    /// <summary>The constructed circle (OCCT's implicit conversion operator).</summary>
    public static explicit operator Circle2D(gce_MakeCirc2d theMaker) => theMaker.Operator();
}
