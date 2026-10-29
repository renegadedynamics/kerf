// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Circ: from an axis system and a radius, coaxial to a circle through a point or at a signed
/// offset, through three points, from a center and a normal or plane, from a center and a point on the axis, or from
/// an axis and a radius.
/// </summary>
public sealed class gce_MakeCirc : gce_Root
{
    private gp_Circ TheCirc = new();

    /// <summary>
    /// A circle through three points. If all three are coincident, the result is a circle of zero radius at P1; if
    /// two are, the construction fails with gce_ConfusedPoints; collinear points give gce_ColinearPoints.
    /// </summary>
    public gce_MakeCirc(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3)
    {
        // Three cases:
        // 1) All three points are coincident: result is a circle centered at P1 with zero radius.
        // 2) Two of the three points are coincident: no solution (error: ConfusedPoints).
        // 3) All three points are distinct: create the perpendicular bisector of P1P2 and
        //    the perpendicular bisector of P2P3. The center is their intersection and
        //    the radius is the distance from that center to any of the three points.
        double dist1, dist2, dist3, aResolution;
        //
        aResolution = gp.Resolution();
        //
        dist1 = P1.Distance(P2);
        dist2 = P1.Distance(P3);
        dist3 = P2.Distance(P3);
        //
        if ((dist1 < aResolution) && (dist2 < aResolution) && (dist3 < aResolution))
        {
            gp_Dir Dirx = new(gp_Dir.D.X);
            gp_Dir Dirz = new(gp_Dir.D.Z);
            TheCirc = new gp_Circ(new gp_Ax2(P1, Dirx, Dirz), 0.0);
            return; // OCCT: TheError is left unset (gce_Done in the port)
        }
        if (dist1 < aResolution || dist2 < aResolution)
        {
            TheError = gce_ConfusedPoints;
            return;
        }
        //
        double x1, y1, z1, x2, y2, z2, x3, y3, z3;
        //
        P1.Coord(out x1, out y1, out z1);
        P2.Coord(out x2, out y2, out z2);
        P3.Coord(out x3, out y3, out z3);
        gp_Dir Dir1 = new(x2 - x1, y2 - y1, z2 - z1);
        gp_Vec VDir2 = new(x3 - x2, y3 - y2, z3 - z2);
        //
        gp_Ax1 anAx1 = new(P1, Dir1);
        gp_Lin aL12 = new(anAx1);
        if (aL12.Distance(P3) < aResolution)
        {
            TheError = gce_ColinearPoints;
            return;
        }
        //
        gp_Vec VDir1 = new(Dir1);
        gp_Vec VDir3 = VDir1.Crossed(VDir2);
        if (VDir3.SquareMagnitude() < aResolution)
        {
            TheError = gce_ColinearPoints;
            return;
        }
        //
        gp_Dir Dir3 = new(VDir3);
        gp_Dir dir = Dir1.Crossed(Dir3);
        gp_Lin L1 = new(new gp_Pnt((P1.XYZ() + P2.XYZ()) / 2.0), dir);
        dir = new gp_Dir(VDir2.Crossed(new gp_Vec(Dir3)));
        gp_Lin L2 = new(new gp_Pnt((P3.XYZ() + P2.XYZ()) / 2.0), dir);
        double Tol = Precision.PConfusion();
        Extrema_ExtElC distmin = new(L1, L2, Tol);
        if (!distmin.IsDone())
        {
            TheError = gce_IntersectionError;
        }
        else
        {
            int nbext;
            //
            //
            if (distmin.IsParallel())
            {
                TheError = gce_IntersectionError;
                return;
            }
            nbext = distmin.NbExt();
            //
            //
            if (nbext == 0)
            {
                TheError = gce_IntersectionError;
            }
            else
            {
                double TheDist = RealLast();
                gp_Pnt pInt = new(), pon1, pon2;
                int i = 1;
                Extrema_POnCurv Pon1, Pon2;
                while (i <= nbext)
                {
                    if (distmin.SquareDistance(i) < TheDist)
                    {
                        TheDist = distmin.SquareDistance(i);
                        distmin.Points(i, out Pon1, out Pon2);
                        pon1 = Pon1.Value();
                        pon2 = Pon2.Value();
                        pInt = new gp_Pnt((pon1.XYZ() + pon2.XYZ()) / 2.0);
                    }
                    i++;
                }
                // modified by NIZNHY-PKV Thu Mar  3 11:30:34 2005f
                // Dir2.Cross(Dir1);
                // modified by NIZNHY-PKV Thu Mar  3 11:30:37 2005t
                dist1 = P1.Distance(pInt);
                dist2 = P2.Distance(pInt);
                dist3 = P3.Distance(pInt);
                pInt.Coord(out x3, out y3, out z3);
                if (dist1 < aResolution)
                {
                    gp_Dir Dirx = new(gp_Dir.D.X);
                    gp_Dir Dirz = new(gp_Dir.D.Z);
                    TheCirc = new gp_Circ(new gp_Ax2(pInt, Dirx, Dirz), 0.0);
                    return; // OCCT: TheError is left unset (gce_Done in the port)
                }
                Dir1 = new gp_Dir(x1 - x3, y1 - y3, z1 - z3);
                // modified by NIZNHY-PKV Thu Mar  3 11:31:11 2005f
                // Dir2 = gp_Dir(x2-x3,y2-y3,z2-z3);
                // modified by NIZNHY-PKV Thu Mar  3 11:31:13 2005t
                //
                TheCirc = new gp_Circ(new gp_Ax2(pInt, new gp_Dir(VDir3), Dir1), (dist1 + dist2 + dist3) / 3.0);
                TheError = gce_Done;
            }
        }
    }

    /// <summary>A circle from an axis system and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCirc(in gp_Ax2 A2, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheError = gce_Done;
            TheCirc = new gp_Circ(A2, Radius);
        }
    }

    /// <summary>A circle from its center, the plane it lies in and its radius.</summary>
    public gce_MakeCirc(in gp_Pnt Center, in gp_Pln Plane, double Radius)
    {
        gce_MakeCirc C = new(Center, Plane.Position().Direction(), Radius);
        // OCCT calls C.Value(), whose check its release build (No_Exception) compiles out: a negative radius gives
        // the status gce_NegativeRadius there, and the port's always-on check would throw instead.
        TheCirc = C.TheCirc;
        TheError = C.Status();
    }

    /// <summary>A circle from its center, the normal to its plane and its radius.</summary>
    public gce_MakeCirc(in gp_Pnt Center, in gp_Dir Norm, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            double A = Norm.X();
            double B = Norm.Y();
            double C = Norm.Z();
            double Aabs = Math.Abs(A);
            double Babs = Math.Abs(B);
            double Cabs = Math.Abs(C);
            gp_Ax2 Pos;
            // To determine the X axis:
            // the dot product Vx.Norm = 0.
            // find the max(A,B,C) for the division.
            // one of the vector coordinates is zero.
            if (Babs <= Aabs && Babs <= Cabs)
            {
                if (Aabs > Cabs)
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(-C, 0.0, A));
                }
                else
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(C, 0.0, -A));
                }
            }
            else if (Aabs <= Babs && Aabs <= Cabs)
            {
                if (Babs > Cabs)
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(0.0, -C, B));
                }
                else
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(0.0, C, -B));
                }
            }
            else
            {
                if (Aabs > Babs)
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(-B, A, 0.0));
                }
                else
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(B, -A, 0.0));
                }
            }
            TheCirc = new gp_Circ(Pos, Radius);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// A circle from its center, a point on its axis and its radius; fails with gce_NullAxis if the points coincide.
    /// </summary>
    public gce_MakeCirc(in gp_Pnt Center, in gp_Pnt Ptaxis, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            if (Center.Distance(Ptaxis) <= gp.Resolution())
            {
                TheError = gce_NullAxis;
            }
            else
            {
                double A = Ptaxis.X() - Center.X();
                double B = Ptaxis.Y() - Center.Y();
                double C = Ptaxis.Z() - Center.Z();
                double Aabs = Math.Abs(A);
                double Babs = Math.Abs(B);
                double Cabs = Math.Abs(C);
                gp_Ax2 Pos;
                // To determine the X axis:
                // the dot product Vx.Norm = 0.
                // find the max(A,B,C) for the division.
                // one of the vector coordinates is zero.
                gp_Dir Norm = new gce_MakeDir(Center, Ptaxis).Value();
                if (Babs <= Aabs && Babs <= Cabs)
                {
                    if (Aabs > Cabs)
                    {
                        Pos = new gp_Ax2(Center, Norm, new gp_Dir(-C, 0.0, A));
                    }
                    else
                    {
                        Pos = new gp_Ax2(Center, Norm, new gp_Dir(C, 0.0, -A));
                    }
                }
                else if (Aabs <= Babs && Aabs <= Cabs)
                {
                    if (Babs > Cabs)
                    {
                        Pos = new gp_Ax2(Center, Norm, new gp_Dir(0.0, -C, B));
                    }
                    else
                    {
                        Pos = new gp_Ax2(Center, Norm, new gp_Dir(0.0, C, -B));
                    }
                }
                else
                {
                    if (Aabs > Babs)
                    {
                        Pos = new gp_Ax2(Center, Norm, new gp_Dir(-B, A, 0.0));
                    }
                    else
                    {
                        Pos = new gp_Ax2(Center, Norm, new gp_Dir(B, -A, 0.0));
                    }
                }
                TheCirc = new gp_Circ(Pos, Radius);
                TheError = gce_Done;
            }
        }
    }

    /// <summary>A circle from its axis and its radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCirc(in gp_Ax1 Axis, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            gp_Dir Norm = Axis.Direction();
            gp_Pnt Center = Axis.Location();
            double A = Norm.X();
            double B = Norm.Y();
            double C = Norm.Z();
            double Aabs = Math.Abs(A);
            double Babs = Math.Abs(B);
            double Cabs = Math.Abs(C);
            gp_Ax2 Pos;
            // To determine the X axis:
            // the dot product Vx.Norm = 0.
            // find the max(A,B,C) for the division.
            // one of the vector coordinates is zero.
            if (Babs <= Aabs && Babs <= Cabs)
            {
                if (Aabs > Cabs)
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(-C, 0.0, A));
                }
                else
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(C, 0.0, -A));
                }
            }
            else if (Aabs <= Babs && Aabs <= Cabs)
            {
                if (Babs > Cabs)
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(0.0, -C, B));
                }
                else
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(0.0, C, -B));
                }
            }
            else
            {
                if (Aabs > Babs)
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(-B, A, 0.0));
                }
                else
                {
                    Pos = new gp_Ax2(Center, Norm, new gp_Dir(B, -A, 0.0));
                }
            }
            TheCirc = new gp_Circ(Pos, Radius);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// A circle coaxial to Circ at the signed distance Dist (enclosing it when positive); fails with
    /// gce_NegativeRadius if the radius would be negative.
    /// </summary>
    public gce_MakeCirc(in gp_Circ Circ, double Dist)
    {
        double Rad = Circ.Radius() + Dist;
        if (Rad < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheCirc = new gp_Circ(Circ.Position(), Rad);
            TheError = gce_Done;
        }
    }

    /// <summary>A circle coaxial to Circ through the point P.</summary>
    public gce_MakeCirc(in gp_Circ Circ, in gp_Pnt P)
    {
        double Rad = new gp_Lin(Circ.Axis()).Distance(P);
        TheCirc = new gp_Circ(Circ.Position(), Rad);
        TheError = gce_Done;
    }

    /// <summary>The constructed circle.</summary>
    /// <exception cref="StdFail_NotDone">No circle was constructed.</exception>
    public gp_Circ Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeCirc::Value() - no result");
        return TheCirc;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Circ Operator() => Value();

    /// <summary>The constructed circle (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Circ(gce_MakeCirc theMaker) => theMaker.Operator();
}
