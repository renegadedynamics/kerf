// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_7.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    /// <summary>The circles tangent to a circle and a line with their centre on a circle.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedCirc Qualified1,
                               in GccEnt_QualifiedLin Qualified2,
                               in Circle2D OnCirc,
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
        double Radius = 0;
        Direction2D dirx = new(Direction2D.D.X);
        Circle2D C1 = Qualified1.Qualified();
        Line2D L2 = Qualified2.Qualified();
        double R1 = C1.Radius();
        Point2D center1 = C1.Location();
        Point2D origin2 = L2.Location();
        Direction2D dirL2 = L2.Direction();
        Direction2D normL2 = new(-dirL2.Y(), dirL2.X());

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        double Tol = Math.Abs(Tolerance);
        NCollection_Array1<double> Rradius = new(1, 2);
        int nbsol1 = 1;
        //  int nbsol2 = 0;
        double Ron = OnCirc.Radius();
        double distcco = OnCirc.Location().Distance(center1);
        Direction2D dircc = new(OnCirc.Location().XY() - center1.XY());
        Point2D pinterm = new(center1.XY() + (distcco - Ron) * dircc.XY());
        double distpl2 = L2.Distance(pinterm);
        double distcc1 = pinterm.Distance(center1);
        double d1 = Math.Abs(distpl2 - Math.Abs(distcc1 - R1));
        double d2 = Math.Abs(distpl2 - (distcc1 + R1));
        if (d1 > Tol || d2 > Tol)
        {
            // OCCT: the same point again (distcco + Ron was meant), and d1, d2 are not recomputed
            pinterm = new Point2D(center1.XY() + (distcco - Ron) * dircc.XY());
            if (d1 > Tol || d2 > Tol)
            {
                nbsol1 = 0;
            }
        }
        if (nbsol1 > 0)
        {
            if (Qualified1.IsEnclosed() || Qualified1.IsOutside())
            {
                nbsol1 = 1;
                Rradius[1] = Math.Abs(distcc1 - R1);
            }
            else if (Qualified1.IsEnclosing())
            {
                nbsol1 = 1;
                Rradius[1] = R1 + distcc1;
            }
            else if (Qualified1.IsUnqualified())
            {
                nbsol1 = 2;
                Rradius[1] = Math.Abs(distcc1 - R1);
                Rradius[2] = R1 + distcc1;
            }
            Direction2D dirbid = new(origin2.XY() - pinterm.XY());
            Direction2D normal = new(-dirL2.Y(), dirL2.X());
            // OCCT: tests Qualified1 (the circle's qualifier) against the line's side, with the same sign twice
            if (Qualified1.IsEnclosed() && dirbid.Dot(normal) < 0.0)
            {
                nbsol1 = 0;
            }
            else if (Qualified1.IsOutside() && dirbid.Dot(normal) < 0.0)
            {
                nbsol1 = 0;
            }
            for (int i = 1; i <= nbsol1; i++)
            {
                if (Math.Abs(Rradius[i] - distpl2) <= Tol)
                {
                    WellDone = true;
                    NbrSol++;
                    cirsol[NbrSol] = new Circle2D(new Axis2D(pinterm, dirx), Rradius[i]);
                    //      ===========================================================
                    Direction2D dc1 = new(center1.XY() - pinterm.XY());
                    Direction2D dc2 = new(origin2.XY() - pinterm.XY());
                    distcc1 = pinterm.Distance(center1);
                    if (!Qualified1.IsUnqualified())
                    {
                        qualifier1[NbrSol] = Qualified1.Qualifier();
                    }
                    else if (Math.Abs(distcc1 + Rradius[i] - R1) < Tol)
                    {
                        qualifier1[NbrSol] = GccEnt_enclosed;
                    }
                    else if (Math.Abs(distcc1 - R1 - Rradius[i]) < Tol)
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

                    double sign = dc2.Dot(new Direction2D(-dirL2.Y(), dirL2.X()));
                    dc2 = new Direction2D(sign * new Xy(-dirL2.Y(), dirL2.X()));
                    pnttg1sol[NbrSol] = new Point2D(pinterm.XY() + Rradius[i] * dc1.XY());
                    pnttg2sol[NbrSol] = new Point2D(pinterm.XY() + Rradius[i] * dc2.XY());
                    par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                    pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                    par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                    pararg2[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L2, pnttg2sol[NbrSol]);
                    // OCCT: pntcen(NbrSol) is never set here, so the parameter is that of the array's initial
                    // point (the origin)
                    parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen[NbrSol]);
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
                    Intp.Perform(Sol.Line(), OnCirc);
                }
                else if (type == GccInt_Par)
                {
                    Intp.Perform(OnCirc, new IntAna2d_Conic(Sol.Parabola()));
                }
                if (Intp.IsDone())
                {
                    if ((!Intp.IsEmpty()) && (!Intp.ParallelElements()) && (!Intp.IdenticalElements()))
                    {
                        for (int j = 1; j <= Intp.NbPoints(); j++)
                        {
                            Point2D Center = Intp.Point(j).Value();
                            double dist1 = Center.Distance(center1);
                            double dist2 = L2.Distance(Center);
                            //	    int nbsol = 1;
                            bool ok = false;
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
                                cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                                //            =======================================================
                                Direction2D dc2 = new(origin2.XY() - Center.XY());
                                distcc1 = Center.Distance(center1);
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
                                    Direction2D dc1 = new(center1.XY() - Center.XY());
                                    pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius * dc1.XY());
                                    par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                    pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                }
                                TheSame2[NbrSol] = 0;
                                double sign = dc2.Dot(new Direction2D(normL2.XY()));
                                dc2 = new Direction2D(sign * normL2.XY());
                                pnttg2sol[NbrSol] = new Point2D(Center.XY() + Radius * dc2.XY());
                                par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                pararg2[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L2, pnttg2sol[NbrSol]);
                                pntcen[NbrSol] = Center;
                                parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen[NbrSol]);
                            }
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }
}
