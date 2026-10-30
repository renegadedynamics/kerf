// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    //=========================================================================
    //   Creation of a circle tangent to Circle C1 and a straight line L2.    +
    //                        centered on a straight line.                    +
    //  We start by making difference between cases that we are going to      +
    //  process separately.                                                   +
    //  In general case:                                                      +
    //  ====================                                                  +
    //  We calculate bissectrices to C1 and L2 that give us                   +
    //  all possible locations of centers of all circles tangent to C1 and L2 +
    //  We intersect these bissectrices with straight line OnLine which gives +
    //  us points among which we'll choose the solutions.                     +
    //  The choices are made basing on Qualifiers of C1 and L2.               +
    //=========================================================================

    /// <summary>The circles tangent to a circle and a line with their centre on a line.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedCirc Qualified1,
                               in GccEnt_QualifiedLin Qualified2,
                               in gp_Lin2d OnLine,
                               double Tolerance)
        : this(4)
    {
        TheSame1.Init(0);
        TheSame2.Init(0);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double Tol = Math.Abs(Tolerance);
        double Radius = 0;
        bool ok = false;
        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        gp_Circ2d C1 = Qualified1.Qualified();
        gp_Lin2d L2 = Qualified2.Qualified();
        double R1 = C1.Radius();
        gp_Pnt2d center1 = C1.Location();
        gp_Pnt2d origin2 = L2.Location();
        gp_Dir2d dirL2 = L2.Direction();
        gp_Dir2d normL2 = new(-dirL2.Y(), dirL2.X());

        //=========================================================================
        //   Processing of limit cases.                                          +
        //=========================================================================

        double distcl = OnLine.Distance(center1);
        gp_Pnt2d pinterm = new(center1.XY() + distcl * new gp_XY(-OnLine.Direction().Y(), OnLine.Direction().X()));
        if (OnLine.Distance(pinterm) > Tolerance)
        {
            // OCCT: the same point again (the other side, center1 - distcl * normal, was meant)
            pinterm = new gp_Pnt2d(center1.XY() + distcl * new gp_XY(-OnLine.Direction().Y(), OnLine.Direction().X()));
        }
        double dist2 = L2.Distance(pinterm);
        if (Qualified1.IsEnclosed() || Qualified1.IsOutside())
        {
            if (Math.Abs(distcl - R1 - dist2) <= Tol)
            {
                ok = true;
            }
        }
        else if (Qualified1.IsEnclosing())
        {
            if (Math.Abs(dist2 - distcl - R1) <= Tol)
            {
                ok = true;
            }
        }
        else if (Qualified1.IsUnqualified())
        {
            ok = true;
        }
        else
        {
            throw new GccEnt_BadQualifier();
        }
        if (ok)
        {
            if (Qualified2.IsOutside())
            {
                gp_Pnt2d pbid = new(pinterm.XY() + dist2 * new gp_XY(-dirL2.Y(), dirL2.X()));
                if (L2.Distance(pbid) <= Tol)
                {
                    WellDone = true;
                }
            }
            else if (Qualified2.IsEnclosed())
            {
                gp_Pnt2d pbid = new(pinterm.XY() - dist2 * new gp_XY(-dirL2.Y(), dirL2.X()));
                if (L2.Distance(pbid) <= Tol)
                {
                    WellDone = true;
                }
            }
            else if (Qualified2.IsUnqualified())
            {
                WellDone = false;
            }
            else
            {
                throw new GccEnt_BadQualifier();
            }
        }
        if (WellDone)
        {
            NbrSol++;
            cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(pinterm, dirx), dist2);
            //   =======================================================
            gp_Dir2d dc1 = new(center1.XY() - pinterm.XY());
            gp_Dir2d dc2 = new(origin2.XY() - pinterm.XY());
            double distcc1 = pinterm.Distance(center1);
            if (!Qualified1.IsUnqualified())
            {
                qualifier1[NbrSol] = Qualified1.Qualifier();
            }
            else if (Math.Abs(distcc1 + dist2 - R1) < Tol)
            {
                qualifier1[NbrSol] = GccEnt_enclosed;
            }
            else if (Math.Abs(distcc1 - R1 - dist2) < Tol)
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

            double sign = dc2.Dot(new gp_Dir2d(-dirL2.Y(), dirL2.X()));
            dc2 = new gp_Dir2d(sign * new gp_XY(-dirL2.Y(), dirL2.X()));
            pnttg1sol[NbrSol] = new gp_Pnt2d(pinterm.XY() + dist2 * dc1.XY());
            pnttg2sol[NbrSol] = new gp_Pnt2d(pinterm.XY() + dist2 * dc2.XY());
            par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
            pararg1[NbrSol] = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
            par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
            pararg2[NbrSol] = ElCLib.Parameter(L2, pnttg2sol[NbrSol]);
            pntcen[NbrSol] = cirsol[NbrSol].Location();
            parcen3[NbrSol] = ElCLib.Parameter(OnLine, pntcen[NbrSol]);
            return;
        }

        //=========================================================================
        //   General case.                                                        +
        //=========================================================================

        GccAna_CircLin2dBisec Bis = new(C1, L2);
        if (Bis.IsDone())
        {
            int nbsolution = Bis.NbSolutions();
            for (int i = 1; i <= nbsolution; i++)
            {
                GccInt_Bisec Sol = Bis.ThisSolution(i);
                GccInt_IType type = Sol.ArcType();
                IntAna2d_AnaIntersection Intp = new();
                if (type == GccInt_Lin)
                {
                    Intp.Perform(OnLine, Sol.Line());
                }
                else if (type == GccInt_Par)
                {
                    Intp.Perform(OnLine, new IntAna2d_Conic(Sol.Parabola()));
                }
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int j = 1; j <= Intp.NbPoints(); j++)
                        {
                            gp_Pnt2d Center = Intp.Point(j).Value();
                            double dist1 = Center.Distance(center1);
                            dist2 = L2.Distance(Center);
                            //	     int nbsol = 1;
                            ok = false;
                            if (Qualified1.IsEnclosed())
                            {
                                if (dist1 - R1 < Tolerance)
                                {
                                    if (Math.Abs(Math.Abs(R1 - dist1) - dist2) < Tolerance)
                                    {
                                        ok = true;
                                    }
                                }
                            }
                            else if (Qualified1.IsOutside())
                            {
                                if (R1 - dist1 < Tolerance)
                                {
                                    if (Math.Abs(Math.Abs(R1 - dist1) - dist2) < Tolerance)
                                    {
                                        ok = true;
                                    }
                                }
                            }
                            else if (Qualified1.IsEnclosing() || Qualified1.IsUnqualified())
                            {
                                ok = true;
                            }
                            if (Qualified2.IsEnclosed() && ok)
                            {
                                if ((((origin2.X() - Center.X()) * (-dirL2.Y()))
                                     + ((origin2.Y() - Center.Y()) * (dirL2.X())))
                                    <= 0)
                                {
                                    ok = true;
                                    Radius = dist2;
                                }
                            }
                            else if (Qualified2.IsOutside() && ok)
                            {
                                if ((((origin2.X() - Center.X()) * (-dirL2.Y()))
                                     + ((origin2.Y() - Center.Y()) * (dirL2.X())))
                                    >= 0)
                                {
                                    ok = true;
                                    Radius = dist2;
                                }
                            }
                            else if (Qualified2.IsUnqualified() && ok)
                            {
                                ok = true;
                                Radius = dist2;
                            }
                            if (ok)
                            {
                                NbrSol++;
                                cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                                //             =======================================================
                                gp_Dir2d dc1 = new(center1.XY() - Center.XY());
                                gp_Dir2d dc2 = new(origin2.XY() - Center.XY());
                                double distcc1 = Center.Distance(center1);
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
                                if (Center.Distance(center1) <= Tolerance
                                    && Math.Abs(Radius - C1.Radius()) <= Tolerance)
                                {
                                    TheSame1[NbrSol] = 1;
                                }
                                else
                                {
                                    TheSame1[NbrSol] = 0;
                                    pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius * dc1.XY());
                                    par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                    pararg1[NbrSol] = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                }
                                TheSame2[NbrSol] = 0;
                                double sign = dc2.Dot(new gp_Dir2d(-dirL2.Y(), dirL2.X()));
                                dc2 = new gp_Dir2d(sign * new gp_XY(-dirL2.Y(), dirL2.X()));
                                pnttg2sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius * dc2.XY());
                                par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                pararg2[NbrSol] = ElCLib.Parameter(L2, pnttg2sol[NbrSol]);
                                pntcen[NbrSol] = Center;
                                parcen3[NbrSol] = ElCLib.Parameter(OnLine, pntcen[NbrSol]);
                            }
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }
}
