// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d3Tan
{
    //===========================================================================
    //   Creation of a circle tangent to a circle, a straight line and a point. +
    //===========================================================================
    public GccAna_Circ2d3Tan(in GccEnt_QualifiedCirc Qualified1,
        in GccEnt_QualifiedLin Qualified2,
        in gp_Pnt2d Point3,
        double Tolerance)
    {
        cirsol = new NCollection_Array1<gp_Circ2d>(1, 4);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, 4);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, 4);
        qualifier3 = new NCollection_Array1<GccEnt_Position>(1, 4);
        TheSame1 = new NCollection_Array1<int>(1, 4);
        TheSame2 = new NCollection_Array1<int>(1, 4);
        TheSame3 = new NCollection_Array1<int>(1, 4);
        pnttg1sol = new NCollection_Array1<gp_Pnt2d>(1, 4);
        pnttg2sol = new NCollection_Array1<gp_Pnt2d>(1, 4);
        pnttg3sol = new NCollection_Array1<gp_Pnt2d>(1, 4);
        par1sol = new NCollection_Array1<double>(1, 4);
        par2sol = new NCollection_Array1<double>(1, 4);
        par3sol = new NCollection_Array1<double>(1, 4);
        pararg1 = new NCollection_Array1<double>(1, 4);
        pararg2 = new NCollection_Array1<double>(1, 4);
        pararg3 = new NCollection_Array1<double>(1, 4);

        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        double Tol = Math.Abs(Tolerance);
        double MaxRad = 1e10, MinRad = 1e-6;
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        gp_Circ2d C1 = Qualified1.Qualified();
        gp_Lin2d L2 = Qualified2.Qualified();
        double R1 = C1.Radius();
        gp_Pnt2d center1 = C1.Location();
        gp_Pnt2d origin2 = L2.Location();
        gp_Dir2d dir2 = L2.Direction();
        gp_Dir2d normL2 = new(-dir2.Y(), dir2.X());

        NCollection_Array1<double> Radius = new(1, 2);
        GccAna_CircLin2dBisec Bis1 = new(C1, L2);
        GccAna_LinPnt2dBisec Bis2 = new(L2, Point3);
        if (Bis1.IsDone() && Bis2.IsDone())
        {
            int nbsolution1 = Bis1.NbSolutions();
            for (int i = 1; i <= nbsolution1; i++)
            {
                GccInt_Bisec Sol1 = Bis1.ThisSolution(i);
                GccInt_Bisec Sol2 = Bis2.ThisSolution();
                GccInt_IType typ1 = Sol1.ArcType();
                GccInt_IType typ2 = Sol2.ArcType();
                IntAna2d_AnaIntersection Intp = new();
                if (typ1 == GccInt_Lin)
                {
                    if (typ2 == GccInt_Lin)
                    {
                        Intp.Perform(Sol1.Line(), Sol2.Line());
                    }
                    else if (typ2 == GccInt_Par)
                    {
                        Intp.Perform(Sol1.Line(), new IntAna2d_Conic(Sol2.Parabola()));
                    }
                }
                else if (typ1 == GccInt_Par)
                {
                    if (typ2 == GccInt_Lin)
                    {
                        Intp.Perform(Sol2.Line(), new IntAna2d_Conic(Sol1.Parabola()));
                    }
                    else if (typ2 == GccInt_Par)
                    {
                        Intp.Perform(Sol1.Parabola(), new IntAna2d_Conic(Sol2.Parabola()));
                    }
                }
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int j = 1; j <= Intp.NbPoints(); j++)
                        {
                            gp_Pnt2d Center = Intp.Point(j).Value();
                            double dist1 = Center.Distance(C1.Location());
                            double dist2 = L2.Distance(Center);
                            double dist3 = Center.Distance(Point3);
                            int nbsol1 = 0;
                            int nbsol3 = 0;
                            bool ok = false;
                            if (Qualified1.IsEnclosed())
                            {
                                if (dist1 - R1 < Tolerance)
                                {
                                    Radius[1] = Math.Abs(R1 - dist1);
                                    nbsol1 = 1;
                                    ok = true;
                                }
                            }
                            else if (Qualified1.IsOutside())
                            {
                                if (R1 - dist1 < Tolerance)
                                {
                                    Radius[1] = Math.Abs(R1 - dist1);
                                    nbsol1 = 1;
                                    ok = true;
                                }
                            }
                            else if (Qualified1.IsEnclosing())
                            {
                                ok = true;
                                nbsol1 = 1;
                                Radius[1] = Math.Abs(R1 - dist1);
                            }
                            else if (Qualified1.IsUnqualified())
                            {
                                ok = true;
                                nbsol1 = 2;
                                Radius[1] = Math.Abs(R1 - dist1);
                                Radius[2] = R1 + dist1;
                            }
                            if (Qualified2.IsEnclosed() && ok)
                            {
                                if ((((L2.Location().X() - Center.X()) * (-L2.Direction().Y()))
                                     + ((L2.Location().Y() - Center.Y()) * (L2.Direction().X())))
                                    <= 0)
                                {
                                    for (int ii = 1; ii <= nbsol1; ii++)
                                    {
                                        if (Math.Abs(dist2 - Radius[ii]) < Tol)
                                        {
                                            ok = true;
                                            Radius[1] = Radius[ii];
                                        }
                                    }
                                }
                            }
                            else if (Qualified2.IsOutside() && ok)
                            {
                                if ((((L2.Location().X() - Center.X()) * (-L2.Direction().Y()))
                                     + ((L2.Location().Y() - Center.Y()) * (L2.Direction().X())))
                                    >= 0)
                                {
                                    for (int ii = 1; ii <= nbsol1; ii++)
                                    {
                                        if (Math.Abs(dist2 - Radius[ii]) < Tol)
                                        {
                                            ok = true;
                                            Radius[1] = Radius[ii];
                                        }
                                    }
                                }
                            }
                            else if (Qualified2.IsUnqualified() && ok)
                            {
                                for (int ii = 1; ii <= nbsol1; ii++)
                                {
                                    if (Math.Abs(dist2 - Radius[ii]) < Tol)
                                    {
                                        ok = true;
                                        Radius[1] = Radius[ii];
                                    }
                                }
                            }
                            if (Math.Abs(dist3 - Radius[1]) <= Tol && ok)
                            {
                                ok = true;
                                nbsol3 = 1;
                            }
                            if (ok)
                            {
                                for (int k = 1; k <= nbsol3; k++)
                                {
                                    if (NbrSol == 4)
                                    {
                                        break;
                                    }
                                    // pop : if the radius is too great - no creation
                                    if (Radius[k] > MaxRad)
                                    {
                                        break;
                                    }
                                    if (Math.Abs(Radius[k]) < MinRad)
                                    {
                                        break;
                                    }

                                    NbrSol++;
                                    cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius[k]);
                                    //               ==========================================================
                                    double distcc1 = Center.Distance(center1);
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
                                    gp_Dir2d dc2 = new(origin2.XY() - Center.XY());
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
                                    qualifier3[NbrSol] = GccEnt_noqualifier;
                                    if (Center.Distance(C1.Location()) <= Tolerance
                                        && Math.Abs(Radius[k] - R1) <= Tolerance)
                                    {
                                        TheSame1[NbrSol] = 1;
                                    }
                                    else
                                    {
                                        TheSame1[NbrSol] = 0;
                                        //  modified by NIZHNY-EAP Mon Nov  1 13:48:21 1999 ___BEGIN___
                                        //                   gp_Dir2d dc(C1.Location().XY()-Center.XY());
                                        // OCCT: the reversed direction puts the tangency point on the far side of the solution unless it is enclosed in C1
                                        gp_Dir2d dc1 = new(Center.XY() - C1.Location().XY());
                                        //  modified by NIZHNY-EAP Mon Nov  1 13:48:55 1999 ___END___
                                        pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[k] * dc1.XY());
                                        par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                        pararg1[NbrSol] = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                    }
                                    TheSame2[NbrSol] = 0;
                                    TheSame3[NbrSol] = 0;
                                    gp_Dir2d dc = new(L2.Location().XY() - Center.XY());
                                    double sign = dc.Dot(new gp_Dir2d(-L2.Direction().Y(), L2.Direction().X()));
                                    dc = new gp_Dir2d(sign * new gp_XY(-L2.Direction().Y(), L2.Direction().X()));
                                    pnttg2sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[k] * dc.XY());
                                    par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                    pararg2[NbrSol] = ElCLib.Parameter(L2, pnttg2sol[NbrSol]);
                                    pnttg3sol[NbrSol] = Point3;
                                    par3sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg3sol[NbrSol]);
                                    pararg3[NbrSol] = 0.0;
                                }
                            }
                        }
                    }
                    WellDone = true;
                }
                if (NbrSol == 4)
                {
                    break;
                }
            }
        }
    }
}
