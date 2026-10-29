// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Circ2d: from an axis and a radius, concentric to a circle through a point or at a signed
/// offset, through three points, from a center and a radius, or from a center and a point.
/// </summary>
public sealed class gce_MakeCirc2d : gce_Root
{
    private gp_Circ2d TheCirc2d = new();

    /// <summary>
    /// A circle through three points, oriented so that they come in order. If all three are coincident, the result is
    /// a circle of zero radius at P1.
    /// </summary>
    public gce_MakeCirc2d(in gp_Pnt2d P1, in gp_Pnt2d P2, in gp_Pnt2d P3)
    {
        gp_Dir2d dirx = new(gp_Dir2d.D.X);
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
        if ((dist1 < gp.Resolution()) && (dist2 < gp.Resolution()) && (dist3 < gp.Resolution()))
        {
            TheCirc2d = new gp_Circ2d(new gp_Ax2d(P1, dirx), 0.0);
            TheError = gce_Done;
        }
        else
        {
            gp_Lin2d L1 = new();
            gp_Lin2d L2 = new();
            double x1, y1, x2, y2, x3, y3;
            P1.Coord(out x1, out y1);
            P2.Coord(out x2, out y2);
            P3.Coord(out x3, out y3);
            if (dist1 > gp.Resolution())
            {
                L1 = new gp_Lin2d(new gp_Pnt2d((P1.XY() + P2.XY()) / 2.0), new gp_Dir2d(P1.Y() - P2.Y(), P2.X() - P1.X()));
            }
            if (dist2 > gp.Resolution())
            {
                L2 = new gp_Lin2d(new gp_Pnt2d((P1.XY() + P3.XY()) / 2.0), new gp_Dir2d(P1.Y() - P3.Y(), P3.X() - P1.X()));
            }
            if (dist2 <= gp.Resolution())
            {
                L2 = new gp_Lin2d(P1, new gp_Dir2d(P1.Y() - P2.Y(), P2.X() - P1.X()));
            }
            else if (dist1 <= gp.Resolution())
            {
                L1 = new gp_Lin2d(P1, new gp_Dir2d(P1.Y() - P3.Y(), P3.X() - P1.X()));
            }
            else if (dist3 <= gp.Resolution())
            {
                L2 = new gp_Lin2d(P1, new gp_Dir2d(P1.Y() - P2.Y(), P2.X() - P1.X()));
            }
            IntAna2d_AnaIntersection Intp = new(L1, L2);
            if (Intp.IsDone())
            {
                if (!Intp.IsEmpty())
                {
                    gp_Pnt2d pInt = Intp.Point(1).Value();
                    dist1 = P1.Distance(pInt);
                    dist2 = P2.Distance(pInt);
                    dist3 = P3.Distance(pInt);
                    double xc, yc;
                    pInt.Coord(out xc, out yc);
                    gp_Dir2d d1 = new(x1 - xc, y1 - yc);
                    gp_Dir2d d2 = new(xc - x3, yc - y3);
                    TheCirc2d = new gp_Circ2d(new gp_Ax22d(pInt, d1, d2), (dist1 + dist2 + dist3) / 3.0);
                    double Alpha1 = ElCLib.Parameter(TheCirc2d, P1);
                    double Alpha2 = ElCLib.Parameter(TheCirc2d, P2);
                    double Alpha3 = ElCLib.Parameter(TheCirc2d, P3);
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
    public gce_MakeCirc2d(in gp_Ax2d XAxis, double Radius, bool Sense = true)
    {
        if (Radius >= 0.0)
        {
            TheCirc2d = new gp_Circ2d(XAxis, Radius, Sense);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_NegativeRadius;
        }
    }

    /// <summary>A circle from its axis system and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCirc2d(in gp_Ax22d Axis, double Radius)
    {
        if (Radius >= 0.0)
        {
            TheCirc2d = new gp_Circ2d(Axis, Radius);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_NegativeRadius;
        }
    }

    /// <summary>A circle from its center and radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCirc2d(in gp_Pnt2d Center, double Radius, bool Sense = true)
    {
        if (Radius >= 0.0)
        {
            TheCirc2d = new gp_Circ2d(new gp_Ax2d(Center, new gp_Dir2d(gp_Dir2d.D.X)), Radius, Sense);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_NegativeRadius;
        }
    }

    /// <summary>A circle from its center and a point on it.</summary>
    public gce_MakeCirc2d(in gp_Pnt2d Center, in gp_Pnt2d Point, bool Sense = true)
    {
        TheCirc2d = new gp_Circ2d(new gp_Ax2d(Center, new gp_Dir2d(gp_Dir2d.D.X)), Point.Distance(Center), Sense);
        TheError = gce_Done;
    }

    /// <summary>A circle concentric to Circ through Point.</summary>
    public gce_MakeCirc2d(in gp_Circ2d Circ, in gp_Pnt2d Point)
    {
        TheCirc2d = new gp_Circ2d(Circ.Axis(), Point.Distance(Circ.Location()));
        TheError = gce_Done;
    }

    /// <summary>A circle concentric to Circ, of radius Abs(Circ.Radius() + Dist1).</summary>
    public gce_MakeCirc2d(in gp_Circ2d Circ, double Dist1)
    {
        TheCirc2d = new gp_Circ2d(Circ.Axis(), Math.Abs(Circ.Radius() + Dist1));
        TheError = gce_Done;
    }

    /// <summary>The constructed circle.</summary>
    /// <exception cref="StdFail_NotDone">No circle was constructed.</exception>
    public gp_Circ2d Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeCirc2d::Value() - no result");
        return TheCirc2d;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Circ2d Operator() => Value();

    /// <summary>The constructed circle (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Circ2d(gce_MakeCirc2d theMaker) => theMaker.Operator();
}
