// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ContAna.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ContAna.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ContAna.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The contours of the quadrics in closed form: a circle on a sphere, two lines on a cylinder, two (or four, for a
/// draft) lines through the apex of a cone; for a view direction, a draft angle, or an eye.
/// </summary>
/// <remarks>A working object, as OCCT's: <c>Perform</c> sets the result, the queries read it.</remarks>
public sealed class Contap_ContAna
{
    private const double Tolpetit = 1.0e-8;

    private bool done;
    private int nbSol;
    private CurveKind typL;
    private Point3D pt1 = new();
    private Point3D pt2 = new();
    private Point3D pt3 = new();
    private Point3D pt4 = new();
    private Direction3D dir1 = new();
    private Direction3D dir2 = new();
    private Direction3D dir3 = new();
    private Direction3D dir4 = new();
    private double prm;

    public Contap_ContAna()
    {
        done = false;
        nbSol = 0;
        typL = OtherCurve;
        prm = 0.0;
    }

    /// <summary>The silhouette of a sphere for the direction <paramref name="D"/>.</summary>
    public void Perform(in Sphere S, in Direction3D D)
    {
        done = false;
        typL = global::Kerf.Geometry.CurveKind.Circle;
        pt1 = S.Location();
        dir1 = D;
        if (Math.Abs(D.Dot(S.XAxis().Direction())) < 0.9999999999999)
        {
            dir2 = D.Crossed(S.XAxis().Direction());
        }
        else
        {
            dir2 = D.Crossed(S.YAxis().Direction());
        }
        prm = S.Radius();
        nbSol = 1;
        done = true;
    }

    /// <summary>The draft contour of angle <paramref name="Angle"/> of a sphere for the direction <paramref name="D"/>.</summary>
    public void Perform(in Sphere S, in Direction3D D, double Angle)
    {
        done = false;
        typL = global::Kerf.Geometry.CurveKind.Circle;

        dir1 = D;
        if (Math.Abs(D.Dot(S.XAxis().Direction())) < 0.9999999999999)
        {
            dir2 = D.Crossed(S.XAxis().Direction());
        }
        else
        {
            dir2 = D.Crossed(S.YAxis().Direction());
        }
        double alpha = (S.Direct() ? Angle : -Angle);
        pt1.SetXYZ(S.Location().XYZ() - S.Radius() * Math.Sin(alpha) * D.XYZ());
        prm = S.Radius() * Math.Cos(alpha);
        nbSol = 1;
        done = true;
    }

    /// <summary>The silhouette of a sphere for the eye <paramref name="Eye"/>.</summary>
    public void Perform(in Sphere S, in Point3D Eye)
    {
        done = false;

        double radius = S.Radius();
        double dist = Eye.Distance(S.Location());
        if (dist <= radius)
        {
            nbSol = 0;
        }
        else
        {
            prm = radius * Math.Sqrt(1.0 - radius * radius / (dist * dist));
            if (prm < Tolpetit)
            {
                nbSol = 0;
            }
            else
            {
                Xyz locxyz = S.Location().XYZ();
                dir1.SetXYZ(Eye.XYZ() - locxyz);
                pt1.SetXYZ(locxyz + (radius * radius / dist) * dir1.XYZ());
                if (Math.Abs(dir1.Dot(S.XAxis().Direction())) < 0.9999999999999)
                {
                    dir2 = dir1.Crossed(S.XAxis().Direction());
                }
                else
                {
                    dir2 = dir1.Crossed(S.YAxis().Direction());
                }
                nbSol = 1;
                typL = global::Kerf.Geometry.CurveKind.Circle;
            }
        }
        done = true;
    }

    /// <summary>The silhouette of a cylinder for the direction <paramref name="D"/>.</summary>
    public void Perform(in Cylinder C, in Direction3D D)
    {
        done = false;

        Xyz normale = C.Position().Direction().XYZ();
        normale.Cross(D.XYZ());
        if (normale.Modulus() <= 1e-15)
        {
            nbSol = 0;
        }
        else
        {
            normale.Normalize();
            typL = global::Kerf.Geometry.CurveKind.Line;
            dir1 = C.Position().Direction();
            dir2 = dir1;
            pt1.SetXYZ(C.Location().XYZ() + C.Radius() * normale);
            pt2.SetXYZ(C.Location().XYZ() - C.Radius() * normale);
            nbSol = 2;
        }

        done = true;
    }

    /// <summary>The draft contour of angle <paramref name="Angle"/> of a cylinder for the direction <paramref name="D"/>.</summary>
    public void Perform(in Cylinder C, in Direction3D D, double Angle)
    {
        done = false;

        double Coefcos = D.Dot(C.Position().XDirection());
        double Coefsin = D.Dot(C.Position().YDirection());
        double Coefcst = Math.Cos(M_PI * 0.5 + Angle);

        double norm1 = Coefcos * Coefcos + Coefsin * Coefsin;
        double norm2 = Math.Sqrt(norm1);

        if (Math.Abs(Coefcst) < norm2)
        {
            typL = global::Kerf.Geometry.CurveKind.Line;
            nbSol = 2;
            dir1 = dir2 = C.Position().Direction();

            if (!C.Direct())
            { // The normal is inverted.
                Coefcos = -Coefcos;
                Coefsin = -Coefsin;
            }

            // Necessary to solve Coefcos*cos(t) + Coefsin*sin(t) = Coefcst
            // and the origins of solution are in the reference of the
            // cylinder in (R*cost0, R*sint0,0) and (R*cost1,R*sint1,0)
            // By setting cos(phi) = Coefcos/std::sqrt(Coefcos**2 + Coefsin**2) and
            //           sin(phi) = Coefsin/std::sqrt(Coefcos**2 + Coefsin**2)
            // and by using trigonometric relations the values of cosinus
            // and sinus to the solutions are obtained.

            prm = Math.Sqrt(norm1 - Coefcst * Coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (Coefcos * Coefcst - Coefsin * prm) / norm1;
            cost1 = (Coefcos * Coefcst + Coefsin * prm) / norm1;

            sint0 = (Coefcos * prm + Coefsin * Coefcst) / norm1;
            sint1 = (-Coefcos * prm + Coefsin * Coefcst) / norm1;

            Xyz Xdir = C.Position().XDirection().XYZ();
            Xyz Ydir = C.Position().YDirection().XYZ();
            Xyz dirxyz = new();

            dirxyz.SetLinearForm(cost0, Xdir, sint0, Ydir);
            dirxyz.Multiply(C.Radius());
            pt1.SetXYZ(C.Location().XYZ().Added(dirxyz));

            dirxyz.SetLinearForm(cost1, Xdir, sint1, Ydir);
            dirxyz.Multiply(C.Radius());
            pt2.SetXYZ(C.Location().XYZ().Added(dirxyz));
        }
        else
        {
            nbSol = 0;
        }

        done = true;
    }

    /// <summary>The silhouette of a cylinder for the eye <paramref name="Eye"/>.</summary>
    public void Perform(in Cylinder C, in Point3D Eye)
    {
        done = false;

        double radius = C.Radius();
        Line3D theaxis = new(C.Axis());
        double dist = theaxis.Distance(Eye);
        if (dist <= radius)
        {
            nbSol = 0;
        }
        else
        {
            typL = global::Kerf.Geometry.CurveKind.Line;
            prm = radius * Math.Sqrt(1.0 - radius * radius / (dist * dist));
            dir1 = C.Axis().Direction();
            dir2 = dir1;
            Xyz axeye = theaxis.Normal(Eye).Direction().XYZ(); // orientate the axis to the outside
            Xyz normale = (theaxis.Direction().Crossed(new Direction3D(axeye))).XYZ();
            //      normale.Normalize();
            pt1.SetXYZ(C.Location().XYZ() + (radius * radius / dist) * axeye);
            pt2.SetXYZ(pt1.XYZ() - prm * normale);
            pt1.SetXYZ(pt1.XYZ() + prm * normale);
            nbSol = 2;
        }
        done = true;
    }

    /// <summary>The silhouette of a cone for the direction <paramref name="D"/>.</summary>
    public void Perform(in Cone C, in Direction3D D)
    {
        done = false;

        double Tgtalpha = Math.Tan(C.SemiAngle());

        double Coefcos = D.Dot(C.Position().XDirection());
        double Coefsin = D.Dot(C.Position().YDirection());
        double Coefcst = D.Dot(C.Axis().Direction()) * Tgtalpha;

        double norm1 = Coefcos * Coefcos + Coefsin * Coefsin;
        double norm2 = Math.Sqrt(norm1);

        if (Math.Abs(Coefcst) < norm2)
        {
            typL = global::Kerf.Geometry.CurveKind.Line;
            nbSol = 2;
            pt1 = C.Apex();
            pt2 = pt1;
            // Necessary to solve Coefcos*cos(t) + Coefsin*sin(t) = Coefcst
            // and director vectors of solutions are
            // cos(t0) * XDirection + sin(t0) * YDirection + ZDirection/Tgtalpha

            prm = Math.Sqrt(norm1 - Coefcst * Coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (Coefcos * Coefcst - Coefsin * prm) / norm1;
            cost1 = (Coefcos * Coefcst + Coefsin * prm) / norm1;

            sint0 = (Coefcos * prm + Coefsin * Coefcst) / norm1;
            sint1 = (-Coefcos * prm + Coefsin * Coefcst) / norm1;

            Xyz Xdir = C.Position().XDirection().XYZ();
            Xyz Ydir = C.Position().YDirection().XYZ();
            Xyz Zdir = C.Axis().Direction().XYZ();
            Xyz dirxyz = new();
            dirxyz.SetLinearForm(cost0, Xdir, sint0, Ydir, 1.0 / Tgtalpha, Zdir);
            dir1.SetXYZ(dirxyz);
            pt1.SetXYZ(pt1.XYZ() + dirxyz);
            dirxyz.SetLinearForm(cost1, Xdir, sint1, Ydir, 1.0 / Tgtalpha, Zdir);
            dir2.SetXYZ(dirxyz);
            pt2.SetXYZ(pt2.XYZ() + dirxyz);
        }
        else
        {
            nbSol = 0;
        }
        done = true;
    }

    /// <summary>The draft contours of angle <paramref name="Angle"/> of a cone for the direction <paramref name="D"/>.</summary>
    public void Perform(in Cone C, in Direction3D D, double Angle)
    {
        done = false;
        nbSol = 0;

        double Ang = C.SemiAngle();
        double Cosa = Math.Cos(Ang);
        double Sina = Math.Sin(Ang);

        double Coefcos = D.Dot(C.Position().XDirection());
        double Coefsin = D.Dot(C.Position().YDirection());

        double Coefcst1 = Math.Cos(M_PI * 0.5 + Angle);

        double norm1 = Coefcos * Coefcos + Coefsin * Coefsin;
        double norm2 = Math.Sqrt(norm1);

        double Coefnz = D.Dot(C.Axis().Direction()) * Sina;
        double Coefcst = (Coefcst1 + Coefnz) / Cosa;

        if (Math.Abs(Coefcst) < norm2)
        {
            typL = global::Kerf.Geometry.CurveKind.Line;
            nbSol += 2;
            pt1 = C.Apex();
            pt2 = pt1;

            prm = Math.Sqrt(norm1 - Coefcst * Coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (Coefcos * Coefcst - Coefsin * prm) / norm1;
            cost1 = (Coefcos * Coefcst + Coefsin * prm) / norm1;

            sint0 = (Coefcos * prm + Coefsin * Coefcst) / norm1;
            sint1 = (-Coefcos * prm + Coefsin * Coefcst) / norm1;

            Xyz Xdir = C.Position().XDirection().XYZ();
            Xyz Ydir = C.Position().YDirection().XYZ();
            Xyz Zdir = C.Axis().Direction().XYZ();
            if (!C.Direct())
            {
                Zdir.Reverse();
            }
            Xyz dirxyz = new();
            dirxyz.SetLinearForm(cost0, Xdir, sint0, Ydir, Cosa / Sina, Zdir);
            dir1.SetXYZ(dirxyz);
            pt1.SetXYZ(pt1.XYZ() + dirxyz);
            dirxyz.SetLinearForm(cost1, Xdir, sint1, Ydir, Cosa / Sina, Zdir);
            dir2.SetXYZ(dirxyz);
            pt2.SetXYZ(pt2.XYZ() + dirxyz);
        }

        Coefcst = (Coefcst1 - Coefnz) / Cosa;

        if (Math.Abs(Coefcst) < norm2)
        {
            typL = global::Kerf.Geometry.CurveKind.Line;
            nbSol += 2;
            pt3 = C.Apex();
            pt4 = pt3;

            prm = Math.Sqrt(norm1 - Coefcst * Coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (Coefcos * Coefcst - Coefsin * prm) / norm1;
            cost1 = (Coefcos * Coefcst + Coefsin * prm) / norm1;

            sint0 = (Coefcos * prm + Coefsin * Coefcst) / norm1;
            sint1 = (-Coefcos * prm + Coefsin * Coefcst) / norm1;

            Xyz Xdir = C.Position().XDirection().XYZ();
            Xyz Ydir = C.Position().YDirection().XYZ();
            Xyz Zdir = C.Axis().Direction().XYZ();
            if (!C.Direct())
            {
                Zdir.Reverse();
            }
            Xyz dirxyz = new();
            dirxyz.SetLinearForm(cost0, Xdir, sint0, Ydir, -Cosa / Sina, Zdir);
            dir3.SetXYZ(dirxyz);
            pt3.SetXYZ(pt3.XYZ() + dirxyz);
            dirxyz.SetLinearForm(cost1, Xdir, sint1, Ydir, -Cosa / Sina, Zdir);
            dir4.SetXYZ(dirxyz);
            pt4.SetXYZ(pt4.XYZ() + dirxyz);
            if (nbSol == 2)
            {
                pt1 = pt3;
                pt2 = pt4;
                dir1 = dir3;
                dir2 = dir4;
            }
        }

        done = true;
    }

    /// <summary>The silhouette of a cone for the eye <paramref name="Eye"/>.</summary>
    public void Perform(in Cone C, in Point3D Eye)
    {
        done = false;

        double Tgtalpha = Math.Tan(C.SemiAngle());

        Xyz apexeye = Eye.XYZ();
        apexeye.Subtract(C.Apex().XYZ());

        double Coefcos = apexeye.Dot(C.Position().XDirection().XYZ());
        double Coefsin = apexeye.Dot(C.Position().YDirection().XYZ());
        double Coefcst = apexeye.Dot(C.Axis().Direction().XYZ()) * Tgtalpha;

        double norm1 = Coefcos * Coefcos + Coefsin * Coefsin;
        double norm2 = Math.Sqrt(Coefcos * Coefcos + Coefsin * Coefsin);

        if (Math.Abs(Coefcst) < norm2)
        {
            typL = global::Kerf.Geometry.CurveKind.Line;
            nbSol = 2;
            pt1 = C.Apex();
            pt2 = pt1;

            prm = Math.Sqrt(norm1 - Coefcst * Coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (Coefcos * Coefcst - Coefsin * prm) / norm1;
            cost1 = (Coefcos * Coefcst + Coefsin * prm) / norm1;

            sint0 = (Coefcos * prm + Coefsin * Coefcst) / norm1;
            sint1 = (-Coefcos * prm + Coefsin * Coefcst) / norm1;

            Xyz Xdir = C.Position().XDirection().XYZ();
            Xyz Ydir = C.Position().YDirection().XYZ();
            Xyz Zdir = C.Axis().Direction().XYZ();
            Xyz dirxyz = new();
            dirxyz.SetLinearForm(cost0, Xdir, sint0, Ydir, 1.0 / Tgtalpha, Zdir);
            dir1.SetXYZ(dirxyz);
            pt1.SetXYZ(pt1.XYZ() + dirxyz);
            dirxyz.SetLinearForm(cost1, Xdir, sint1, Ydir, 1.0 / Tgtalpha, Zdir);
            dir2.SetXYZ(dirxyz);
            pt2.SetXYZ(pt2.XYZ() + dirxyz);
        }
        else
        {
            nbSol = 0;
        }
        done = true;
    }

    public bool IsDone() => done;

    /// <summary>The number of contours; throws StdFail_NotDone before a computation.</summary>
    public int NbContours()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return nbSol;
    }

    /// <summary>GeomAbs_Line or GeomAbs_Circle; throws StdFail_NotDone or Standard_DomainError (no contour).</summary>
    public CurveKind TypeContour()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        if (nbSol == 0)
        {
            throw new DomainErrorException();
        }
        return typL;
    }

    /// <summary>The circle; throws StdFail_NotDone or Standard_DomainError.</summary>
    public Circle3D Circle()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        if (typL != global::Kerf.Geometry.CurveKind.Circle || nbSol == 0)
        {
            throw new DomainErrorException();
        }
        return new Circle3D(new Frame3D(pt1, dir1, dir2), prm);
    }

    /// <summary>The line <paramref name="Index"/>; throws StdFail_NotDone, Standard_DomainError or Standard_OutOfRange.</summary>
    public Line3D Line(int Index)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        if (typL != global::Kerf.Geometry.CurveKind.Line || nbSol == 0)
        {
            throw new DomainErrorException();
        }
        if (Index <= 0 || Index > nbSol)
        {
            throw new OutOfRangeException();
        }
        switch (Index)
        {
            case 1:
                return new Line3D(pt1, dir1);
            case 2:
                return new Line3D(pt2, dir2);
            case 3:
                return new Line3D(pt3, dir3);
            case 4:
                return new Line3D(pt4, dir4);
        }
        throw new OutOfRangeException("Program error in Contap_ContAna");
    }
}
