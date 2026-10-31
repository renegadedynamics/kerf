// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_9.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    /// <summary>The circles tangent to two lines with their centre on a circle. The tolerance is not used.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedLin Qualified1,
                               in GccEnt_QualifiedLin Qualified2,
                               in gp_Circ2d OnCirc,
                               double Tolerance)
        : this(4)
    {
        _ = Tolerance; // OCCT: unnamed, not used
        TheSame1.Init(0);
        TheSame2.Init(0);
        WellDone = false;
        NbrSol = 0;

        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        gp_Lin2d L1 = Qualified1.Qualified();
        gp_Lin2d L2 = Qualified2.Qualified();
        GccAna_Lin2dBisec Bis = new(L1, L2);
        int i = 0, j = 0;
        int nbsol = 0;
        double sgn = 1.0;
        double s = 1.0;
        bool ok = false;
        gp_Dir2d D1 = L1.Direction();
        gp_Dir2d D2 = L2.Direction();
        gp_Dir2d Dnor1 = new(-D1.Y(), D1.X());
        gp_Dir2d Dnor2 = new(-D2.Y(), D2.X());
        gp_XY XYnor1 = new(-D1.Y(), D1.X());
        gp_XY XYnor2 = new(-D2.Y(), D2.X());
        gp_Pnt2d originL1 = L1.Location();
        gp_Pnt2d originL2 = L2.Location();
        gp_XY Dloc = originL1.XY() - originL2.XY();
        // OCCT: Angle is signed, so every pair of lines turning clockwise from D1 to D2 takes this "parallel" branch
        if (D1.Angle(D2) <= gp.Resolution())
        {
            if (Qualified1.IsEnclosed())
            {
                ok = Dloc.Dot(XYnor1) <= 0.0;
            }
            else if (Qualified1.IsOutside())
            {
                ok = Dloc.Dot(XYnor1) >= 0.0;
            }
            else
            {
                ok = true;
            }
            // OCCT: overwrites the first line's test instead of combining with it
            if (Qualified2.IsEnclosed())
            {
                ok = Dloc.Dot(XYnor2) >= 0.0;
            }
            else if (Qualified2.IsOutside())
            {
                ok = Dloc.Dot(XYnor2) <= 0.0;
            }
            else
            {
                ok = true;
            }
            if (ok)
            {
                IntAna2d_AnaIntersection Intp = new(Bis.ThisSolution(1), OnCirc);
                if (Intp.IsDone())
                {
                    WellDone = true;
                    if (!Intp.IsEmpty())
                    {
                        for (int l = 1; l <= Intp.NbPoints(); l++)
                        {
                            NbrSol++;
                            gp_Pnt2d pt = Intp.Point(l).Value();
                            gp_Ax2d axe = new(pt, dirx);
                            cirsol[NbrSol] = new gp_Circ2d(axe, L1.Distance(pt));
                            //          ===============================================
                            gp_Dir2d dc1 = new(originL1.XY() - pt.XY());
                            gp_Dir2d dc2 = new(originL2.XY() - pt.XY());
                            if (!Qualified1.IsUnqualified())
                            {
                                qualifier1[NbrSol] = Qualified1.Qualifier();
                            }
                            else if (dc1.Dot(Dnor1) > 0.0)
                            {
                                qualifier1[NbrSol] = GccEnt_outside;
                            }
                            else
                            {
                                qualifier1[NbrSol] = GccEnt_enclosed;
                            }
                            if (!Qualified2.IsUnqualified())
                            {
                                qualifier2[NbrSol] = Qualified2.Qualifier();
                            }
                            else if (dc2.Dot(Dnor2) > 0.0)
                            {
                                qualifier2[NbrSol] = GccEnt_outside;
                            }
                            else
                            {
                                qualifier2[NbrSol] = GccEnt_enclosed;
                            }
                        }
                    }
                }
            }
        }
        else if (Qualified1.IsEnclosed() && Qualified2.IsEnclosed())
        {
            //============================================================
            if (Bis.IsDone())
            {
                if (Bis.NbSolutions() == 2)
                {
                    nbsol = 1;
                    i = 2;
                    j = 1;
                    sgn = -1.0;
                }
            }
        }
        else if (Qualified1.IsEnclosed() && Qualified2.IsOutside())
        {
            //===========================================================
            if (Bis.IsDone())
            {
                if (Bis.NbSolutions() >= 1)
                {
                    nbsol = 1;
                    i = 1;
                    j = 1;
                    if (D1.Angle(D2) >= 0.0)
                    {
                        sgn = -1.0;
                    }
                }
            }
        }
        else if (Qualified1.IsOutside() && Qualified2.IsEnclosed())
        {
            //===========================================================
            if (Bis.IsDone())
            {
                if (Bis.NbSolutions() >= 1)
                {
                    nbsol = 1;
                    i = 1;
                    j = 1;
                    if (D1.Angle(D2) <= 0.0)
                    {
                        sgn = -1.0;
                    }
                }
            }
        }
        else if (Qualified1.IsOutside() && Qualified2.IsOutside())
        {
            //==========================================================
            if (Bis.IsDone())
            {
                if (Bis.NbSolutions() >= 1)
                {
                    nbsol = 1;
                    i = 2;
                    j = 1;
                }
            }
        }
        else if (Qualified1.IsUnqualified() && Qualified2.IsEnclosed())
        {
            //=============================================================
            if (Bis.IsDone())
            {
                nbsol = 2;
                if (Bis.NbSolutions() >= 1)
                {
                    i = 1;
                    j = 2;
                }
                if (D1.Angle(D2) >= 0.0)
                {
                    s = -1.0;
                }
                else
                {
                    sgn = -1.0;
                }
            }
        }
        else if (Qualified1.IsUnqualified() && Qualified2.IsOutside())
        {
            //==============================================================
            if (Bis.IsDone())
            {
                nbsol = 2;
                if (Bis.NbSolutions() >= 1)
                {
                    i = 1;
                    j = 2;
                }
                if (D1.Angle(D2) >= 0.0)
                {
                    s = -1.0;
                    sgn = -1.0;
                }
            }
        }
        else if (Qualified1.IsEnclosed() && Qualified2.IsUnqualified())
        {
            //===============================================================
            if (Bis.IsDone())
            {
                nbsol = 2;
                if (Bis.NbSolutions() >= 1)
                {
                    i = 1;
                    j = 2;
                }
                if (D1.Angle(D2) >= 0.0)
                {
                    sgn = -1.0;
                }
                else
                {
                    s = -1.0;
                }
            }
        }
        else if (Qualified1.IsOutside() && Qualified2.IsUnqualified())
        {
            //==============================================================
            if (Bis.IsDone())
            {
                nbsol = 2;
                if (Bis.NbSolutions() >= 1)
                {
                    i = 1;
                    j = 2;
                }
                if (D1.Angle(D2) <= 0.0)
                {
                    s = -1.0;
                    sgn = -1.0;
                }
            }
        }
        else if (Qualified1.IsUnqualified() && Qualified2.IsUnqualified())
        {
            //==================================================================
            nbsol = 4;
            i = 1;
            j = 2;
        }
        if (nbsol >= 1)
        {
            if (Bis.IsDone())
            {
                int kk = 0;
                for (int k = i; k <= i + j - 1; k++)
                {
                    kk++;
                    IntAna2d_AnaIntersection Intp = new(Bis.ThisSolution(k), OnCirc);
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            for (int l = 1; l <= Intp.NbPoints(); l++)
                            {
                                gp_Vec2d V = new(Intp.Point(l).Value(), Bis.ThisSolution(k).Location());
                                if ((kk == 1 && sgn * V.Dot(new gp_Vec2d(Bis.ThisSolution(k).Direction())) >= 0.0)
                                    || (kk == 2 && sgn * s * V.Dot(new gp_Vec2d(Bis.ThisSolution(k).Direction())) >= 0.0)
                                    || nbsol == 4)
                                {
                                    NbrSol++;
                                    // OCCT: the centre is the intersection point number i (the first bisector's
                                    // index), not l; native and the port throw Standard_OutOfRange when there is no
                                    // such point
                                    gp_Pnt2d pt = Intp.Point(i).Value();
                                    gp_Ax2d axe = new(pt, dirx);
                                    cirsol[NbrSol] = new gp_Circ2d(axe,
                                                                   //              ===============================
                                                                   L1.Distance(Intp.Point(l).Value()));
                                    //                                         ===================================
                                    gp_Dir2d dc1 = new(originL1.XY() - pt.XY());
                                    gp_Dir2d dc2 = new(originL2.XY() - pt.XY());
                                    if (!Qualified1.IsUnqualified())
                                    {
                                        qualifier1[NbrSol] = Qualified1.Qualifier();
                                    }
                                    else if (dc1.Dot(Dnor1) > 0.0)
                                    {
                                        qualifier1[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier1[NbrSol] = GccEnt_enclosed;
                                    }
                                    if (!Qualified2.IsUnqualified())
                                    {
                                        qualifier2[NbrSol] = Qualified2.Qualifier();
                                    }
                                    else if (dc2.Dot(Dnor2) > 0.0)
                                    {
                                        qualifier2[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier2[NbrSol] = GccEnt_enclosed;
                                    }
                                }
                            }
                        }
                        WellDone = true;
                    }
                }
            }
        }
        if (NbrSol > 0)
        {
            for (i = 1; i <= NbrSol; i++)
            {
                gp_Pnt2d pbid = cirsol[i].Location();
                double Radius = cirsol[i].Radius();
                gp_Dir2d dc2 = new(originL1.XY() - pbid.XY());
                double sign = dc2.Dot(new gp_Dir2d(-L1.Direction().Y(), L1.Direction().X()));
                dc2 = new gp_Dir2d(sign * new gp_XY(-L1.Direction().Y(), L1.Direction().X()));
                pnttg1sol[i] = new gp_Pnt2d(pbid.XY() + Radius * dc2.XY());
                dc2 = new gp_Dir2d(originL2.XY() - pbid.XY());
                sign = dc2.Dot(new gp_Dir2d(-L2.Direction().Y(), L2.Direction().X()));
                dc2 = new gp_Dir2d(sign * new gp_XY(-L2.Direction().Y(), L2.Direction().X()));
                pnttg2sol[i] = new gp_Pnt2d(pbid.XY() + Radius * dc2.XY());
                pntcen[i] = pbid;
                par1sol[i] = ElCLib.Parameter(cirsol[i], pnttg1sol[i]);
                pararg1[i] = ElCLib.Parameter(L1, pnttg1sol[i]);
                par2sol[i] = ElCLib.Parameter(cirsol[i], pnttg2sol[i]);
                pararg2[i] = ElCLib.Parameter(L2, pnttg2sol[i]);
                parcen3[i] = ElCLib.Parameter(OnCirc, pntcen[i]);
            }
        }
    }
}
