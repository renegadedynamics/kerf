// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanRad
{
    // circulare tangent to a circle a point and a given radius
    //=============================================================
    //========================================================================
    // Initialize WellDone to false.                                       +
    // Return circle C1.                                             +
    // Leave with error if the construction is impossible.     +
    // Create parallel to C1 in the proper direction.                           +
    // Create circle with center in Point1 of radius Radius.                   +
    // Intersect the parallel and the circle.                              +
    //                              ==> The center point of the solution.   +
    // Create the solution that will be added to already found solutions.         +
    // Fill the fields.                                                +
    //========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified circle and passing through a point.</summary>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public GccAna_Circ2d2TanRad(in GccEnt_QualifiedCirc Qualified1, in gp_Pnt2d Point2, double Radius, double Tolerance)
    {
        qualifier1 = new(1, 4);
        qualifier2 = new(1, 4);
        TheSame1 = new(1, 4);
        TheSame2 = new(1, 4);
        cirsol = new(1, 4);
        pnttg1sol = new(1, 4);
        pnttg2sol = new(1, 4);
        par1sol = new(1, 4);
        par2sol = new(1, 4);
        pararg1 = new(1, 4);
        pararg2 = new(1, 4);


        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        double   Tol = Math.Abs(Tolerance);
        NbrSol       = 0;
        WellDone     = false;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        int i;
        for (i = 1; i <= 4; i++)
        {
            TheSame1[i] = 0;
            TheSame2[i] = 0;
        }
        double                        deport = 0.0;
        int                           signe  = 0;
        int                           nbsol  = 0;
        gp_Circ2d                     C1     = Qualified1.Qualified();
        NCollection_Array1<gp_Circ2d> C = new(1, 4);
        double                        R1       = C1.Radius();
        double                        distance = (C1.Location()).Distance(Point2);
        double                        dispc1   = C1.Distance(Point2);
        gp_Dir2d dir1 = new(Point2.XY() - (C1.Location().XY()));
        gp_Pnt2d center1 = C1.Location();
        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
        }
        else
        {
            if (dispc1 - Radius * 2.0 > Tol)
            {
                WellDone = true;
            }
            else if (Qualified1.IsEnclosed())
            {
                //  =================================
                if ((distance - R1 > Tol) || (Radius - R1 > Tol))
                {
                    WellDone = true;
                }
                else
                {
                    if (Math.Abs(distance - R1) < Tol)
                    {
                        nbsol  = -1;
                        deport = R1 - Radius;
                        signe  = 1;
                    }
                    else
                    {
                        C[1]  = new gp_Circ2d(C1.XAxis(), Math.Abs(Radius - R1));
                        C[2]  = new gp_Circ2d(new gp_Ax2d(Point2, dirx), Radius);
                        nbsol = 1;
                    }
                }
            }
            else if (Qualified1.IsEnclosing())
            {
                //  ==================================
                if ((Tol < R1 - distance) || (Tol < R1 - Radius))
                {
                    WellDone = true;
                }
                else
                {
                    if (Math.Abs(distance - R1) < Tol)
                    {
                        nbsol  = -1;
                        deport = R1 - Radius;
                        signe  = 1;
                    }
                    else
                    {
                        C[1]  = new gp_Circ2d(C1.XAxis(), Math.Abs(Radius - R1));
                        C[2]  = new gp_Circ2d(new gp_Ax2d(Point2, dirx), Radius);
                        nbsol = 1;
                    }
                }
            }
            else if (Qualified1.IsOutside())
            {
                //  ================================
                if (Tol < R1 - distance)
                {
                    WellDone = true;
                }
                else if ((Math.Abs(distance - R1) < Tol) || (Math.Abs(dispc1 - Radius * 2.0) < Tol))
                {
                    nbsol  = -1;
                    deport = R1 + Radius;
                    signe  = -1;
                }
                else
                {
                    C[1]  = new gp_Circ2d(C1.XAxis(), Radius + R1);
                    C[2]  = new gp_Circ2d(new gp_Ax2d(Point2, dirx), Radius);
                    nbsol = 1;
                }
            }
            else if (Qualified1.IsUnqualified())
            {
                //  ====================================
                if (Math.Abs(dispc1 - Radius * 2.0) < Tol)
                {
                    WellDone = true;
                    gp_Pnt2d Center = new(center1.XY() + (distance - Radius) * dir1.XY());
                    cirsol[1] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                    //      ==================================================
                    if (Math.Abs(Center.Distance(center1) - R1) < Tol)
                    {
                        qualifier1[1] = GccEnt_enclosed;
                    }
                    else
                    {
                        qualifier1[1] = GccEnt_outside;
                    }
                    qualifier2[1] = GccEnt_noqualifier;
                    pnttg1sol[1]  = new gp_Pnt2d(Center.XY() - Radius * dir1.XY());
                    pnttg2sol[1]  = Point2;
                    WellDone      = true;
                    NbrSol        = 1;
                }
                else if ((Math.Abs(R1 - Radius) < Tol) && (Math.Abs(distance - R1) < Tol))
                {
                    cirsol[1] = C1;
                    //      =========================
                    qualifier1[1] = GccEnt_unqualified;
                    qualifier2[1] = GccEnt_noqualifier;
                    TheSame1[1]   = 1;
                    pnttg2sol[1]  = Point2;
                    WellDone      = true;
                    NbrSol        = 1;
                    C[1]          = new gp_Circ2d(C1.XAxis(), Radius + R1);
                    C[2]          = new gp_Circ2d(new gp_Ax2d(Point2, dirx), Radius);
                    nbsol         = 1;
                }
                else
                {
                    C[1]  = new gp_Circ2d(C1.XAxis(), Math.Abs(Radius - R1));
                    C[2]  = new gp_Circ2d(new gp_Ax2d(Point2, dirx), Radius);
                    C[3]  = new gp_Circ2d(C1.XAxis(), Radius + R1);
                    C[4]  = new gp_Circ2d(new gp_Ax2d(Point2, dirx), Radius);
                    nbsol = 2;
                }
            }
            if (nbsol > 0)
            {
                for (int j = 1; j <= nbsol; j++)
                {
                    IntAna2d_AnaIntersection Intp = new(C[2 * j - 1], C[2 * j]);
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            for (i = 1; i <= Intp.NbPoints(); i++)
                            {
                                NbrSol++;
                                gp_Pnt2d Center = Intp.Point(i).Value();
                                cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                                //            =======================================================
                                double distcc1 = center1.Distance(Center);
                                if (!Qualified1.IsUnqualified())
                                {
                                    qualifier1[NbrSol] = Qualified1.Qualifier();
                                }
                                else if (Math.Abs(distcc1 + Radius - R1) < Tol)
                                {
                                    qualifier1[NbrSol] = GccEnt_enclosed;
                                }
                                else if (Math.Abs(distcc1 - R1 - Radius) < Tol)
                                {
                                    qualifier1[NbrSol] = GccEnt_outside;
                                }
                                else
                                {
                                    qualifier1[NbrSol] = GccEnt_enclosing;
                                }
                                qualifier2[NbrSol] = GccEnt_noqualifier;
                                dir1               = new gp_Dir2d(Center.XY() - center1.XY());
                                if ((Center.Distance(center1) > C1.Radius())
                                        && (Radius < Center.Distance(center1) + C1.Radius()))
                                {
                                    pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() - Radius * dir1.XY());
                                }
                                else if ((Center.Distance(center1) < C1.Radius()) && (Radius < C1.Radius()))
                                {
                                    pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius * dir1.XY());
                                }
                                else
                                {
                                    pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() - Radius * dir1.XY());
                                }
                                pnttg2sol[NbrSol] = Point2;
                            }
                        }
                        WellDone = true;
                    }
                }
            }
            else if (nbsol < 0)
            {
                gp_Pnt2d Center = new(center1.XY() + deport * dir1.XY());
                cirsol[1] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                //    ==================================================
                qualifier1[1] = Qualified1.Qualifier();
                qualifier2[1] = GccEnt_noqualifier;
                if (Math.Abs(deport) <= Tol && Math.Abs(Radius - R1) <= Tol)
                {
                    TheSame1[1] = 1;
                }
                else
                {
                    pnttg1sol[1] = new gp_Pnt2d(Center.XY() + signe * Radius * dir1.XY());
                }
                pnttg2sol[1] = Point2;
                WellDone     = true;
                NbrSol       = 1;
            }
        }
        for (i = 1; i <= NbrSol; i++)
        {
            par1sol[i] = GccAna_ElCLib.Parameter(cirsol[i], pnttg1sol[i]);
            if (TheSame1[i] == 0)
            {
                pararg1[i] = GccAna_ElCLib.Parameter(C1, pnttg1sol[i]);
            }
            par2sol[i] = GccAna_ElCLib.Parameter(cirsol[i], pnttg2sol[i]);
            pararg2[i] = 0.0;
        }
    }
}
