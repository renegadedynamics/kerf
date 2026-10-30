// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    //=========================================================================
    //   Creation of a circle tangent to two circles C1 and C2.               +
    //                        centered on a circle.                           +
    //  We start with distinguishing various boundary cases that will be      +
    //  processed separately.                                            +
    //  In the general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to C1 and C2 that give us all           +
    //  possible locations of centers of all circles tangent to C1 and C2. + We intersect these
    //  bissectrices with circle OnCirc which gives us   + points among which we choose the solutions. +
    //  The choice is made basing in Qualifiers of C1 and C2.  +
    //=========================================================================

    /// <summary>The circles tangent to two circles with their centre on a circle.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedCirc Qualified1,
                               in GccEnt_QualifiedCirc Qualified2,
                               in gp_Circ2d OnCirc,
                               double Tolerance)
        : this(8)
    {
        TheSame1.Init(0);
        TheSame2.Init(0);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double Tol = Math.Abs(Tolerance);
        gp_Circ2d C1 = Qualified1.Qualified();
        gp_Circ2d C2 = Qualified2.Qualified();
        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        NCollection_Array1<double> Radius = new(1, 2);
        NCollection_Array1<double> Rradius = new(1, 2);
        gp_Pnt2d center1 = C1.Location();
        gp_Pnt2d center2 = C2.Location();

        double R1 = C1.Radius();
        double R2 = C2.Radius();

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        int nbsol1 = 1;
        int nbsol2 = 0;
        double Ron = OnCirc.Radius();
        double distcco = OnCirc.Location().Distance(center1);
        gp_Dir2d dircc = new(OnCirc.Location().XY() - center1.XY());
        gp_Pnt2d pinterm = new(center1.XY() + (distcco - Ron) * dircc.XY());
        double distcc2 = pinterm.Distance(center2);
        double distcc1 = pinterm.Distance(center1);
        double d1 = Math.Abs(distcc2 - R2 - Math.Abs(distcc1 - R1));
        double d2 = Math.Abs(distcc2 + R2 - Math.Abs(distcc1 - R1));
        double d3 = Math.Abs(distcc2 - R2 - (distcc1 + R1));
        double d4 = Math.Abs(distcc2 + R2 - (distcc1 + R1));
        if (d1 > Tol || d2 > Tol || d3 > Tol || d4 > Tol)
        {
            pinterm = new gp_Pnt2d(center1.XY() + (distcco + Ron) * dircc.XY());
            distcc2 = pinterm.Distance(center2);
            distcc1 = pinterm.Distance(center1);
            d1 = Math.Abs(distcc2 - R2 - Math.Abs(distcc1 - R1));
            d2 = Math.Abs(distcc2 + R2 - Math.Abs(distcc1 - R1));
            d3 = Math.Abs(distcc2 - R2 - (distcc1 + R1));
            d4 = Math.Abs(distcc2 + R2 - (distcc1 + R1));
            if (d1 > Tol || d2 > Tol || d3 > Tol || d4 > Tol)
            {
                nbsol1 = 0;
            }
        }
        if (nbsol1 > 0)
        {
            if (Qualified1.IsEnclosed() || Qualified1.IsOutside())
            {
                nbsol1 = 1;
                Radius[1] = Math.Abs(distcc1 - R1);
            }
            else if (Qualified1.IsEnclosing())
            {
                nbsol1 = 1;
                Radius[1] = R1 + distcc1;
            }
            else if (Qualified1.IsUnqualified())
            {
                nbsol1 = 2;
                Radius[1] = Math.Abs(distcc1 - R1);
                Radius[2] = R1 + distcc1;
            }
            if (Qualified2.IsEnclosed() || Qualified2.IsOutside())
            {
                nbsol2 = 1;
                Rradius[1] = Math.Abs(distcc2 - R2);
            }
            else if (Qualified2.IsEnclosing())
            {
                nbsol2 = 1;
                Rradius[1] = R2 + distcc2;
            }
            else if (Qualified2.IsUnqualified())
            {
                nbsol2 = 2;
                Rradius[1] = Math.Abs(distcc2 - R2);
                Rradius[2] = R2 + distcc2;
            }
            for (int i = 1; i <= nbsol1; i++)
            {
                for (int j = 1; j <= nbsol2; j++)
                {
                    if (Math.Abs(Radius[i] - Rradius[j]) <= Tol)
                    {
                        WellDone = true;
                        NbrSol++;
                        cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(pinterm, dirx), Radius[i]);
                        //        ===========================================================
                        gp_Dir2d dc1 = new(center1.XY() - pinterm.XY());
                        gp_Dir2d dc2 = new(center2.XY() - pinterm.XY());
                        distcc1 = pinterm.Distance(center1);
                        distcc2 = pinterm.Distance(center2);
                        if (!Qualified1.IsUnqualified())
                        {
                            qualifier1[NbrSol] = Qualified1.Qualifier();
                        }
                        else if (Math.Abs(distcc1 + Radius[i] - R1) < Tol)
                        {
                            qualifier1[NbrSol] = GccEnt_enclosed;
                        }
                        else if (Math.Abs(distcc1 - R1 - Radius[i]) < Tol)
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
                        else if (Math.Abs(distcc2 + Radius[i] - R2) < Tol)
                        {
                            qualifier2[NbrSol] = GccEnt_enclosed;
                        }
                        else if (Math.Abs(distcc2 - R2 - Radius[i]) < Tol)
                        {
                            qualifier2[NbrSol] = GccEnt_outside;
                        }
                        else
                        {
                            qualifier2[NbrSol] = GccEnt_enclosing;
                        }
                        pnttg1sol[NbrSol] = new gp_Pnt2d(pinterm.XY() + Radius[i] * dc1.XY());
                        pnttg2sol[NbrSol] = new gp_Pnt2d(pinterm.XY() + Radius[i] * dc2.XY());
                        pntcen[NbrSol] = cirsol[NbrSol].Location();
                        par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                        pararg1[NbrSol] = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                        par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                        pararg2[NbrSol] = ElCLib.Parameter(C2, pnttg2sol[NbrSol]);
                        parcen3[NbrSol] = ElCLib.Parameter(OnCirc, pntcen[NbrSol]);
                    }
                }
            }
            if (WellDone)
            {
                return;
            }
        }

        //=========================================================================
        //   General case.                                                         +
        //=========================================================================

        GccAna_Circ2dBisec Bis = new(C1, C2);
        if (Bis.IsDone())
        {
            NCollection_Array1<double> Rbid = new(1, 2);
            NCollection_Array1<double> RBid = new(1, 2);
            int nbsolution = Bis.NbSolutions();
            for (int i = 1; i <= nbsolution; i++)
            {
                GccInt_Bisec Sol = Bis.ThisSolution(i);
                GccInt_IType typ = Sol.ArcType();
                IntAna2d_AnaIntersection Intp = new();
                if (typ == GccInt_Cir)
                {
                    Intp.Perform(OnCirc, Sol.Circle());
                }
                else if (typ == GccInt_Lin)
                {
                    Intp.Perform(Sol.Line(), OnCirc);
                }
                else if (typ == GccInt_Hpr)
                {
                    Intp.Perform(OnCirc, new IntAna2d_Conic(Sol.Hyperbola()));
                }
                else if (typ == GccInt_Ell)
                {
                    Intp.Perform(OnCirc, new IntAna2d_Conic(Sol.Ellipse()));
                }
                if (Intp.IsDone())
                {
                    if ((!Intp.IsEmpty()) && (!Intp.ParallelElements()) && (!Intp.IdenticalElements()))
                    {
                        for (int j = 1; j <= Intp.NbPoints(); j++)
                        {
                            gp_Pnt2d Center = Intp.Point(j).Value();
                            double dist1 = Center.Distance(center1);
                            double dist2 = Center.Distance(center2);
                            int nbsol = 0;
                            int nsol = 0;
                            int nnsol = 0;
                            R1 = C1.Radius();
                            R2 = C2.Radius();
                            if (Qualified1.IsEnclosed())
                            {
                                if (dist1 - R1 < Tol)
                                {
                                    nbsol = 1;
                                    Rbid[1] = Math.Abs(R1 - dist1);
                                }
                            }
                            else if (Qualified1.IsOutside())
                            {
                                if (R1 - dist1 < Tol)
                                {
                                    nbsol = 1;
                                    Rbid[1] = Math.Abs(dist1 - R1);
                                }
                            }
                            else if (Qualified1.IsEnclosing())
                            {
                                nbsol = 1;
                                Rbid[1] = dist1 + R1;
                            }
                            else if (Qualified1.IsUnqualified())
                            {
                                nbsol = 2;
                                Rbid[1] = dist1 + R1;
                                // OCCT: Rbid(1) is written twice and Rbid(2) never (it keeps the value of an earlier
                                // point, or the array's initial value: uninitialised in C++, 0 in the port)
                                Rbid[1] = Math.Abs(dist1 - R1);
                            }
                            if (Qualified2.IsEnclosed() && nbsol != 0)
                            {
                                if (dist2 - R2 < Tol)
                                {
                                    nsol = 1;
                                    RBid[1] = Math.Abs(R2 - dist2);
                                }
                            }
                            else if (Qualified2.IsOutside() && nbsol != 0)
                            {
                                if (R2 - dist2 < Tol)
                                {
                                    nsol = 1;
                                    RBid[1] = Math.Abs(R2 - dist2);
                                }
                            }
                            else if (Qualified2.IsEnclosing() && nbsol != 0)
                            {
                                nsol = 1;
                                RBid[1] = dist2 + R2;
                            }
                            else if (Qualified2.IsUnqualified() && nbsol != 0)
                            {
                                nsol = 2;
                                RBid[1] = dist2 + R2;
                                RBid[2] = Math.Abs(R2 - dist2);
                            }
                            for (int isol = 1; isol <= nbsol; isol++)
                            {
                                for (int jsol = 1; jsol <= nsol; jsol++)
                                {
                                    if (Math.Abs(Rbid[isol] - RBid[jsol]) <= Tol)
                                    {
                                        nnsol++;
                                        // OCCT: up to four matches for two elements (OCCT and the port throw past the end)
                                        Radius[nnsol] = (RBid[jsol] + Rbid[isol]) / 2.0;
                                    }
                                }
                            }
                            if (nnsol > 0)
                            {
                                for (int k = 1; k <= nnsol; k++)
                                {
                                    NbrSol++;
                                    cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius[k]);
                                    //              ==========================================================
                                    distcc1 = Center.Distance(center1);
                                    distcc2 = Center.Distance(center2);
                                    if (!Qualified1.IsUnqualified())
                                    {
                                        qualifier1[NbrSol] = Qualified1.Qualifier();
                                    }
                                    else if (Math.Abs(distcc1 + Radius[k] - R1) < Tol)
                                    {
                                        qualifier1[NbrSol] = GccEnt_enclosed;
                                    }
                                    else if (Math.Abs(distcc1 - R1 - Radius[k]) < Tol)
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
                                    else if (Math.Abs(distcc2 + Radius[k] - R2) < Tol)
                                    {
                                        qualifier2[NbrSol] = GccEnt_enclosed;
                                    }
                                    else if (Math.Abs(distcc2 - R2 - Radius[k]) < Tol)
                                    {
                                        qualifier2[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier2[NbrSol] = GccEnt_enclosing;
                                    }
                                    if (Center.Distance(center1) <= Tolerance
                                        && Math.Abs(Radius[k] - C1.Radius()) <= Tolerance)
                                    {
                                        TheSame1[NbrSol] = 1;
                                    }
                                    else
                                    {
                                        TheSame1[NbrSol] = 0;
                                        gp_Dir2d dc1 = new(center1.XY() - Center.XY());
                                        pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[k] * dc1.XY());
                                        par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                        pararg1[NbrSol] = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                    }
                                    if (Center.Distance(center2) <= Tolerance
                                        && Math.Abs(Radius[k] - C2.Radius()) <= Tolerance)
                                    {
                                        TheSame2[NbrSol] = 1;
                                    }
                                    else
                                    {
                                        TheSame2[NbrSol] = 0;
                                        gp_Dir2d dc2 = new(center2.XY() - Center.XY());
                                        pnttg2sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[k] * dc2.XY());
                                        par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                        pararg2[NbrSol] = ElCLib.Parameter(C2, pnttg2sol[NbrSol]);
                                    }
                                    pntcen[NbrSol] = Center;
                                    parcen3[NbrSol] = ElCLib.Parameter(OnCirc, pntcen[NbrSol]);
                                }
                            }
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }
}
