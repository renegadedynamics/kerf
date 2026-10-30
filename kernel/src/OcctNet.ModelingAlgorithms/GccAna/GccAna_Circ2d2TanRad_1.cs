// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanRad
{
    // circular tangent to a circle, a line and a given radius
    //==============================================================
    //========================================================================
    // Initialize WellDone to false.                                         +
    // Return circle C1 and straight line L2.                                +
    // Leave with error if the construction is impossible.                   +
    // Create parallel to C1 in the proper direction.                        +
    // Create parallel to L2 in the proper direction.                        +
    // Intersect parallels ==> center point of the solution.                 +
    // Create the solution and add it to already found ones.                 +
    // Fill the fields.                                                      +
    //========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified circle and a qualified line.</summary>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public GccAna_Circ2d2TanRad(in GccEnt_QualifiedCirc Qualified1, in GccEnt_QualifiedLin Qualified2, double Radius, double Tolerance)
    {
        qualifier1 = new(1, 8);
        qualifier2 = new(1, 8);
        TheSame1 = new(1, 8);
        TheSame2 = new(1, 8);
        cirsol = new(1, 8);
        pnttg1sol = new(1, 8);
        pnttg2sol = new(1, 8);
        par1sol = new(1, 8);
        par2sol = new(1, 8);
        pararg1 = new(1, 8);
        pararg2 = new(1, 8);


        double   Tol = Math.Abs(Tolerance);
        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        NbrSol   = 0;
        WellDone = false;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        int i;
        for (i = 1; i <= 8; i++)
        {
            TheSame2[i] = 0;
        }
        int                        ncote1 = 0;
        int                        ncote2 = 0;
        int                        nbsol  = 0;
        double                     cote   = 0.0;
        double                     ccote  = 0.0;
        NCollection_Array1<double> cote1 = new(1, 3);
        NCollection_Array1<double> cote2 = new(1, 2);
        gp_Circ2d                  C1   = Qualified1.Qualified();
        gp_Lin2d                   L2   = Qualified2.Qualified();
        double                     xdir = (L2.Direction()).X();
        double                     ydir = (L2.Direction()).Y();
        gp_Dir2d normL2 = new(-ydir, xdir);
        double                     lxloc = (L2.Location()).X();
        double                     lyloc = (L2.Location()).Y();
        double                     cxloc = (C1.Location()).X();
        double                     cyloc = (C1.Location()).Y();
        double                     R1    = C1.Radius();
        gp_Pnt2d center1 = new(cxloc, cyloc);
        gp_Pnt2d origin2 = new(lxloc, lyloc);
        double                     dist     = Radius * 2.0 + R1;
        double                     distance = L2.Distance(center1);

        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
        }
        else
        {
            if (distance - dist > Tol)
            {
                WellDone = true;
            }
            else if (Qualified1.IsEnclosed())
            {
                //  =================================
                if (Qualified2.IsEnclosed())
                {
                    //    ============================
                    if (distance - R1 > Tol)
                    {
                        WellDone = true;
                    }
                    else if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                        && (Radius * 2.0 - R1 + distance > Tol))
                                      || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                              && (Radius * 2.0 - R1 - distance > Tol))
                                      || (Radius - R1 > Tol))
                    {
                        WellDone = true;
                    }
                    else
                    {
                        if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                  && (Radius * 2.0 > R1 - distance))
                                || (Math.Abs(distance - R1) < Tol)
                                || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                        && (Radius * 2.0 > (R1 + distance + Tol))))
                        {
                            cote  = 1.0;
                            nbsol = 3;
                            ccote = 1.0;
                        }
                        else
                        {
                            ncote1   = 1;
                            ncote2   = 1;
                            cote1[1] = R1 - Radius;
                            cote2[1] = 1.0;
                            nbsol    = 1;
                        }
                    }
                }
                else if (Qualified2.IsOutside())
                {
                    //    ================================
                    if (distance > R1 + Tol)
                    {
                        WellDone = true;
                    }
                    else if ((((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc)) > 0.0)
                                        && (Radius * 2.0 - R1 + distance > Tol))
                                      || (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc)) < 0.0)
                                              && (Radius * 2.0 - R1 - distance > Tol))
                                      || (Radius - R1 > Tol))
                    {
                        WellDone = true;
                    }
                    else
                    {
                        if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                  && (Radius * 2.0 > R1 - distance))
                                || (Math.Abs(R1 - Radius) < Tol)
                                || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                        && (Radius * 2.0 > (R1 + distance)))
                                || (Math.Abs(distance - R1) < Tol))
                        {
                            cote  = -1.0;
                            nbsol = 3;
                            ccote = 1.0;
                        }
                        else
                        {
                            ncote1   = 1;
                            ncote2   = 1;
                            cote1[1] = R1 - Radius;
                            cote2[1] = -1.0;
                            nbsol    = 1;
                        }
                    }
                }
                else if (Qualified2.IsUnqualified())
                {
                    //    ====================================
                    if ((distance - R1 > Tol) || (Radius * 2.0 - R1 - distance > Tol) || (Radius - R1 > Tol))
                    {
                        WellDone = true;
                    }
                    else if (distance > R1)
                    {
                        if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                        {
                            cote = 1.0;
                        }
                        else
                        {
                            cote  = -1.0;
                            nbsol = 3;
                        }
                        nbsol = 3;
                        ccote = 1;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 2;
                        cote1[1] = R1 - Radius;
                        cote2[1] = 1.0;
                        cote2[2] = -1.0;
                        nbsol    = 1;
                    }
                }
                if (nbsol == 3)
                {
                }
            }
            else if (Qualified1.IsEnclosing())
            {
                //  ==================================
                if (Qualified2.IsEnclosed())
                {
                    //    =================================
                    if ((distance < R1 - Tol) || (Radius < R1 - Tol) || (Radius * 2 < distance + R1 - Tol)
                            || (Tol < R1 - Radius) || (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0))
                    {
                        WellDone = true;
                    }
                    else if ((distance < R1) || (Radius < R1) || (Radius * 2.0 < distance + R1))
                    {
                        cote  = 1.0;
                        ccote = 1.0;
                        nbsol = 3;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = Radius - R1;
                        cote2[1] = 1.0;
                        nbsol    = 1;
                    }
                }
                else if (Qualified2.IsOutside())
                {
                    //    ================================
                    if ((Tol < R1 - distance) || (Tol < distance + R1 - Radius * 2.0) || (Tol < R1 - Radius)
                            || (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                    {
                        WellDone = true;
                    }
                    else if ((distance < R1) || (Radius * 2.0 < distance + R1) || (Radius < R1))
                    {
                        cote  = -1.0;
                        ccote = -1.0;
                        nbsol = 3;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = Radius - R1;
                        cote2[1] = -1.0;
                        nbsol    = 1;
                    }
                }
                else if (Qualified2.IsUnqualified())
                {
                    //    ====================================
                    if ((distance < R1 - Tol) || (Radius * 2.0 < distance + R1 - Tol) || (Radius < R1 - Tol))
                    {
                        WellDone = true;
                    }
                    else if ((distance < R1) || (Radius * 2.0 < distance + R1))
                    {
                        if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                        {
                            cote  = 1.0;
                            ccote = 1.0;
                            nbsol = 3;
                        }
                        else
                        {
                            ccote = -1.0;
                            cote  = -1.0;
                            nbsol = 3;
                        }
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 2;
                        cote1[1] = Radius - R1;
                        cote2[1] = 1.0;
                        cote2[2] = -1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsOutside()) && (Qualified2.IsEnclosed()))
            {
                //  ===============================================================
                if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > R1 + Tol))
                        || (distance > R1 + Radius * 2.0 + Tol))
                {
                    WellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > R1))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                      && (distance > R1 + 2.0 * Radius))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = Radius + R1;
                        cote2[1] = 1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsOutside()) && (Qualified2.IsOutside()))
            {
                //  ==============================================================
                if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > R1 + Tol))
                        || (distance > R1 + Radius * 2.0 + Tol))
                {
                    WellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > R1))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                      && (distance > R1 + 2.0 * Radius))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = Radius + R1;
                        cote2[1] = -1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsOutside()) && (Qualified2.IsUnqualified()))
            {
                //  ==================================================================
                if (distance > Radius * 2.0 + R1 + Tol)
                {
                    WellDone = true;
                }
                else if (distance > Radius * 2.0 + R1)
                {
                    if (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                    {
                        cote = 1.0;
                    }
                    else
                    {
                        cote = -1.0;
                    }
                    nbsol = 4;
                }
                else
                {
                    ncote1   = 1;
                    ncote2   = 2;
                    cote1[1] = Radius + R1;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                    nbsol    = 1;
                }
            }
            else if ((Qualified1.IsUnqualified()) && (Qualified2.IsEnclosed()))
            {
                //  ===================================================================
                if ((distance > R1 + Radius * 2.0 + Tol)
                        || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > R1 + Tol)))
                {
                    WellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > R1))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                      && (distance > R1 + 2.0 * Radius))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 2;
                        ncote2   = 1;
                        cote1[1] = Math.Abs(Radius - R1);
                        cote1[2] = Radius + R1;
                        cote2[1] = 1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsUnqualified()) && (Qualified2.IsOutside()))
            {
                //  ==================================================================
                if ((distance > R1 + Radius * 2.0 + Tol)
                        || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > R1 + Tol)))
                {
                    WellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > R1))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                      && (distance > R1 + 2.0 * Radius))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 2;
                        ncote2   = 1;
                        cote1[1] = Math.Abs(Radius - R1);
                        cote1[2] = Radius + R1;
                        cote2[1] = -1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsUnqualified()) && (Qualified2.IsUnqualified()))
            {
                //  ======================================================================
                if (distance > R1 + Radius * 2.0 + Tol)
                {
                    WellDone = true;
                }
                else if (distance > R1 + Radius * 2.0)
                {
                    if (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                    {
                        cote = -1.0;
                    }
                    else
                    {
                        cote = 1.0;
                    }
                    nbsol = 4;
                }
                else
                {
                    ncote1   = 2;
                    ncote2   = 2;
                    cote1[1] = Math.Abs(Radius - R1);
                    cote1[2] = Radius + R1;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                    nbsol    = 1;
                }
            }
            if (nbsol == 1)
            {
                for (int jcote1 = 1; jcote1 <= ncote1; jcote1++)
                {
                    for (int jcote2 = 1; jcote2 <= ncote2; jcote2++)
                    {
                        if (cote1[jcote1] < Tol)
                        {
                            continue;
                        }
                        gp_Circ2d cirint = new(new gp_Ax2d(center1, dirx), cote1[jcote1]);
                        gp_Lin2d linint = new(
                            new gp_Pnt2d(lxloc - cote2[jcote2] * ydir * Radius, lyloc + cote2[jcote2] * xdir * Radius),
                            L2.Direction());
                        IntAna2d_AnaIntersection Intp = new(linint, cirint);
                        if (Intp.IsDone())
                        {
                            if (!Intp.IsEmpty())
                            {
                                for (i = 1; i <= Intp.NbPoints(); i++)
                                {
                                    NbrSol++;
                                    gp_Pnt2d Center = Intp.Point(i).Value();
                                    gp_Ax2d axe = new(Center, dirx);
                                    cirsol[NbrSol] = new gp_Circ2d(axe, Radius);
    //              ======================================
                                    gp_Dir2d dc2 = new(origin2.XY() - Center.XY());
                                    double   distcc1 = Center.Distance(center1);
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
                                    if (!Qualified2.IsUnqualified())
                                    {
                                        qualifier2[NbrSol] = Qualified2.Qualifier();
                                    }
                                    else if (dc2.Dot(normL2) > 0.0)
                                    {
                                        qualifier2[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier2[NbrSol] = GccEnt_enclosed;
                                    }
                                    TheSame1[NbrSol] = 0;
                                    if ((Radius < R1) && Center.Distance(center1) <= R1)
                                    {
                                        pnttg1sol[NbrSol] =
                                            new gp_Pnt2d(Center.XY() + Radius * (new gp_Dir2d(Center.XY() - center1.XY()).XY()));
                                    }
                                    else
                                    {
                                        pnttg1sol[NbrSol] =
                                            new gp_Pnt2d(Center.XY() - Radius * (new gp_Dir2d(Center.XY() - center1.XY()).XY()));
                                    }
                                    pnttg2sol[NbrSol] =
                                        new gp_Pnt2d(Center.XY() + cote2[jcote2] * Radius * new gp_XY(ydir, -xdir));
                                }
                            }
                            WellDone = true;
                        }
                    }
                }
            }
            else if (nbsol == 2)
            {
                gp_Pnt2d Cen = new(center1.XY() + cote * (R1 + Radius) * new gp_XY(-ydir, xdir));
                gp_Ax2d axe = new(Cen, dirx);
                cirsol[1] = new gp_Circ2d(axe, Radius);
                //    =================================
                WellDone      = true;
                NbrSol        = 1;
                TheSame1[1]   = 0;
                qualifier1[1] = Qualified1.Qualifier();
                qualifier2[1] = Qualified2.Qualifier();
                pnttg1sol[1]  = new gp_Pnt2d(Cen.XY() + cote * Radius * new gp_XY(ydir, -xdir));
                pnttg2sol[1]  = new gp_Pnt2d(Cen.XY() + Radius * new gp_XY(ydir, -xdir));
            }
            else if (nbsol == 3)
            {
                WellDone = true;
                NbrSol   = 1;
                gp_Pnt2d Cen = new(center1.XY() + cote * (R1 - Radius) * new gp_XY(ydir, -xdir));
                gp_Ax2d axe = new(Cen, dirx);
                cirsol[1] = new gp_Circ2d(axe, Radius);
                //    =================================
                qualifier1[1] = Qualified1.Qualifier();
                qualifier2[1] = Qualified2.Qualifier();
                pnttg2sol[1]  = new gp_Pnt2d(Cen.XY() + cote * Radius * new gp_XY(ydir, -xdir));
                if (Math.Abs(R1 - Radius) > 0.0)
                {
                    pnttg1sol[1] = new gp_Pnt2d(Cen.XY() + ccote * Radius * new gp_XY(ydir, -xdir));
                }
                else
                {
                    TheSame1[1] = 1;
                }
            }
            else if (nbsol == 4)
            {
                gp_Pnt2d Cent = new(center1.XY() + cote * (R1 + Radius) * new gp_XY(-ydir, xdir));
                gp_Ax2d axe = new(Cent, dirx);
                cirsol[1] = new gp_Circ2d(axe, Radius);
                //    =================================
                qualifier1[1] = Qualified1.Qualifier();
                qualifier2[1] = Qualified2.Qualifier();
                WellDone      = true;
                NbrSol        = 1;
                TheSame1[1]   = 0;
                pnttg1sol[1]  = new gp_Pnt2d(Cent.XY() + cote * Radius * new gp_XY(ydir, -xdir));
                pnttg2sol[1]  = new gp_Pnt2d(Cent.XY() + cote * Radius * new gp_XY(-ydir, xdir));
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
            pararg2[i] = GccAna_ElCLib.Parameter(L2, pnttg2sol[i]);
        }
    }
}
