// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Circ: from an axis system and a radius, coaxial to a circle through a point or at a signed
/// offset, through three points, from a center and a normal or plane, from a center and a point on the axis, or from
/// an axis and a radius.
/// </summary>
public sealed class gce_MakeCirc : gce_Root
{
    private Circle3D TheCirc = new();

    /// <summary>
    /// A circle through three points. If all three are coincident, the result is a circle of zero radius at P1; if
    /// two are, the construction fails with gce_ConfusedPoints; collinear points give gce_ColinearPoints.
    /// </summary>
    public gce_MakeCirc(in Point3D P1, in Point3D P2, in Point3D P3)
    {
        // Three cases:
        // 1) All three points are coincident: result is a circle centered at P1 with zero radius.
        // 2) Two of the three points are coincident: no solution (error: ConfusedPoints).
        // 3) All three points are distinct: create the perpendicular bisector of P1P2 and
        //    the perpendicular bisector of P2P3. The center is their intersection and
        //    the radius is the distance from that center to any of the three points.
        double dist1, dist2, dist3, aResolution;
        //
        aResolution = Constants.Resolution();
        //
        dist1 = P1.Distance(P2);
        dist2 = P1.Distance(P3);
        dist3 = P2.Distance(P3);
        //
        if ((dist1 < aResolution) && (dist2 < aResolution) && (dist3 < aResolution))
        {
            Direction3D Dirx = new(Direction3D.D.X);
            Direction3D Dirz = new(Direction3D.D.Z);
            TheCirc = new Circle3D(new Frame3D(P1, Dirx, Dirz), 0.0);
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
        Direction3D Dir1 = new(x2 - x1, y2 - y1, z2 - z1);
        Vector3D VDir2 = new(x3 - x2, y3 - y2, z3 - z2);
        //
        Axis3D anAx1 = new(P1, Dir1);
        Line3D aL12 = new(anAx1);
        if (aL12.Distance(P3) < aResolution)
        {
            TheError = gce_ColinearPoints;
            return;
        }
        //
        Vector3D VDir1 = new(Dir1);
        Vector3D VDir3 = VDir1.Crossed(VDir2);
        if (VDir3.SquareMagnitude() < aResolution)
        {
            TheError = gce_ColinearPoints;
            return;
        }
        //
        Direction3D Dir3 = new(VDir3);
        Direction3D dir = Dir1.Crossed(Dir3);
        Line3D L1 = new(new Point3D((P1.XYZ() + P2.XYZ()) / 2.0), dir);
        dir = new Direction3D(VDir2.Crossed(new Vector3D(Dir3)));
        Line3D L2 = new(new Point3D((P3.XYZ() + P2.XYZ()) / 2.0), dir);
        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
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
                Point3D pInt = new(), pon1, pon2;
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
                        pInt = new Point3D((pon1.XYZ() + pon2.XYZ()) / 2.0);
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
                    Direction3D Dirx = new(Direction3D.D.X);
                    Direction3D Dirz = new(Direction3D.D.Z);
                    TheCirc = new Circle3D(new Frame3D(pInt, Dirx, Dirz), 0.0);
                    return; // OCCT: TheError is left unset (gce_Done in the port)
                }
                Dir1 = new Direction3D(x1 - x3, y1 - y3, z1 - z3);
                // modified by NIZNHY-PKV Thu Mar  3 11:31:11 2005f
                // Dir2 = gp_Dir(x2-x3,y2-y3,z2-z3);
                // modified by NIZNHY-PKV Thu Mar  3 11:31:13 2005t
                //
                TheCirc = new Circle3D(new Frame3D(pInt, new Direction3D(VDir3), Dir1), (dist1 + dist2 + dist3) / 3.0);
                TheError = gce_Done;
            }
        }
    }

    /// <summary>A circle from an axis system and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCirc(in Frame3D A2, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheError = gce_Done;
            TheCirc = new Circle3D(A2, Radius);
        }
    }

    /// <summary>A circle from its center, the plane it lies in and its radius.</summary>
    public gce_MakeCirc(in Point3D Center, in Plane3D Plane, double Radius)
    {
        gce_MakeCirc C = new(Center, Plane.Position().Direction(), Radius);
        // OCCT calls C.Value(), whose check its release build (No_Exception) compiles out: a negative radius gives
        // the status gce_NegativeRadius there, and the port's always-on check would throw instead.
        TheCirc = C.TheCirc;
        TheError = C.Status();
    }

    /// <summary>A circle from its center, the normal to its plane and its radius.</summary>
    public gce_MakeCirc(in Point3D Center, in Direction3D Norm, double Radius)
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
            Frame3D Pos;
            // To determine the X axis:
            // the dot product Vx.Norm = 0.
            // find the max(A,B,C) for the division.
            // one of the vector coordinates is zero.
            if (Babs <= Aabs && Babs <= Cabs)
            {
                if (Aabs > Cabs)
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(-C, 0.0, A));
                }
                else
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(C, 0.0, -A));
                }
            }
            else if (Aabs <= Babs && Aabs <= Cabs)
            {
                if (Babs > Cabs)
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(0.0, -C, B));
                }
                else
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(0.0, C, -B));
                }
            }
            else
            {
                if (Aabs > Babs)
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(-B, A, 0.0));
                }
                else
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(B, -A, 0.0));
                }
            }
            TheCirc = new Circle3D(Pos, Radius);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// A circle from its center, a point on its axis and its radius; fails with gce_NullAxis if the points coincide.
    /// </summary>
    public gce_MakeCirc(in Point3D Center, in Point3D Ptaxis, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            if (Center.Distance(Ptaxis) <= Constants.Resolution())
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
                Frame3D Pos;
                // To determine the X axis:
                // the dot product Vx.Norm = 0.
                // find the max(A,B,C) for the division.
                // one of the vector coordinates is zero.
                Direction3D Norm = new gce_MakeDir(Center, Ptaxis).Value();
                if (Babs <= Aabs && Babs <= Cabs)
                {
                    if (Aabs > Cabs)
                    {
                        Pos = new Frame3D(Center, Norm, new Direction3D(-C, 0.0, A));
                    }
                    else
                    {
                        Pos = new Frame3D(Center, Norm, new Direction3D(C, 0.0, -A));
                    }
                }
                else if (Aabs <= Babs && Aabs <= Cabs)
                {
                    if (Babs > Cabs)
                    {
                        Pos = new Frame3D(Center, Norm, new Direction3D(0.0, -C, B));
                    }
                    else
                    {
                        Pos = new Frame3D(Center, Norm, new Direction3D(0.0, C, -B));
                    }
                }
                else
                {
                    if (Aabs > Babs)
                    {
                        Pos = new Frame3D(Center, Norm, new Direction3D(-B, A, 0.0));
                    }
                    else
                    {
                        Pos = new Frame3D(Center, Norm, new Direction3D(B, -A, 0.0));
                    }
                }
                TheCirc = new Circle3D(Pos, Radius);
                TheError = gce_Done;
            }
        }
    }

    /// <summary>A circle from its axis and its radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public gce_MakeCirc(in Axis3D Axis, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            Direction3D Norm = Axis.Direction();
            Point3D Center = Axis.Location();
            double A = Norm.X();
            double B = Norm.Y();
            double C = Norm.Z();
            double Aabs = Math.Abs(A);
            double Babs = Math.Abs(B);
            double Cabs = Math.Abs(C);
            Frame3D Pos;
            // To determine the X axis:
            // the dot product Vx.Norm = 0.
            // find the max(A,B,C) for the division.
            // one of the vector coordinates is zero.
            if (Babs <= Aabs && Babs <= Cabs)
            {
                if (Aabs > Cabs)
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(-C, 0.0, A));
                }
                else
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(C, 0.0, -A));
                }
            }
            else if (Aabs <= Babs && Aabs <= Cabs)
            {
                if (Babs > Cabs)
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(0.0, -C, B));
                }
                else
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(0.0, C, -B));
                }
            }
            else
            {
                if (Aabs > Babs)
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(-B, A, 0.0));
                }
                else
                {
                    Pos = new Frame3D(Center, Norm, new Direction3D(B, -A, 0.0));
                }
            }
            TheCirc = new Circle3D(Pos, Radius);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// A circle coaxial to Circ at the signed distance Dist (enclosing it when positive); fails with
    /// gce_NegativeRadius if the radius would be negative.
    /// </summary>
    public gce_MakeCirc(in Circle3D Circ, double Dist)
    {
        double Rad = Circ.Radius() + Dist;
        if (Rad < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheCirc = new Circle3D(Circ.Position(), Rad);
            TheError = gce_Done;
        }
    }

    /// <summary>A circle coaxial to Circ through the point P.</summary>
    public gce_MakeCirc(in Circle3D Circ, in Point3D P)
    {
        double Rad = new Line3D(Circ.Axis()).Distance(P);
        TheCirc = new Circle3D(Circ.Position(), Rad);
        TheError = gce_Done;
    }

    /// <summary>The constructed circle.</summary>
    /// <exception cref="NotDoneException">No circle was constructed.</exception>
    public Circle3D Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeCirc::Value() - no result");
        return TheCirc;
    }

    /// <summary>Alias for Value().</summary>
    public Circle3D Operator() => Value();

    /// <summary>The constructed circle (OCCT's implicit conversion operator).</summary>
    public static explicit operator Circle3D(gce_MakeCirc theMaker) => theMaker.Operator();
}
