// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_8.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    /// <summary>The circles tangent to a circle and through a point with their centre on a circle.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedCirc Qualified1,
                               in Point2D Point2,
                               in Circle2D OnCirc,
                               double Tolerance)
        : this(4)
    {
        TheSame1.Init(0);
        TheSame2.Init(0);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double Tol = Math.Abs(Tolerance);
        NCollection_Array1<double> Radius = new(1, 2);
        Direction2D dirx = new(Direction2D.D.X);
        Circle2D C1 = Qualified1.Qualified();
        double R1 = C1.Radius();
        Point2D center1 = C1.Location();

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        int nbsol1 = 1;
        //  int nbsol2 = 0;
        double Ron = OnCirc.Radius();
        double distcco = OnCirc.Location().Distance(center1);
        Point2D pinterm = new();
        Direction2D dircc = new();
        bool SameCenter = false;
        if (!OnCirc.Location().IsEqual(center1, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            dircc = new Direction2D(OnCirc.Location().XY() - center1.XY());
            pinterm = new Point2D(center1.XY() + (distcco - Ron) * dircc.XY());
            double distcc2 = pinterm.Distance(Point2);
            double distcc1 = pinterm.Distance(center1);
            double d1 = Math.Abs(distcc2 - Math.Abs(distcc1 - R1));
            double d2 = Math.Abs(distcc2 - (distcc1 + R1));
            if (d1 > Tol || d2 > Tol)
            {
                if (!SameCenter)
                {
                    pinterm = new Point2D(center1.XY() + (distcco + Ron) * dircc.XY());
                }
                distcc2 = pinterm.Distance(Point2);
                distcc1 = pinterm.Distance(center1);
                d1 = Math.Abs(distcc2 - Math.Abs(distcc1 - R1));
                d2 = Math.Abs(distcc2 - (distcc1 + R1));
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
                for (int i = 1; i <= nbsol1; i++)
                {
                    WellDone = true;
                    if (i != 1 || !SameCenter)
                    {
                        NbrSol++;
                        cirsol[NbrSol] = new Circle2D(new Axis2D(pinterm, dirx), Radius[i]);
                        //    ===========================================================
                        Direction2D dc1 = new();
                        if (!SameCenter)
                        {
                            dc1 = new Direction2D(center1.XY() - pinterm.XY());
                        }
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
                        qualifier2[NbrSol] = GccEnt_noqualifier;
                        if (!SameCenter)
                        {
                            pnttg1sol[NbrSol] = new Point2D(pinterm.XY() + Radius[i] * dc1.XY());
                        }
                        else
                        {
                            pnttg1sol[NbrSol] = new Point2D(pinterm.XY());
                        }
                        pnttg2sol[NbrSol] = Point2;
                        pntcen[NbrSol] = cirsol[NbrSol].Location();
                        par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                        pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                        par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                        pararg2[NbrSol] = 0.0;
                        parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen[NbrSol]);
                    }
                }
                if (WellDone)
                {
                    return;
                }
            }
        }

        //=========================================================================
        //   General case.                                                         +
        //=========================================================================

        GccAna_CircPnt2dBisec Bis = new(C1, Point2);
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
                else if (type == GccInt_Cir)
                {
                    Intp.Perform(OnCirc, Sol.Circle());
                }
                else if (type == GccInt_Hpr)
                {
                    Intp.Perform(OnCirc, new IntAna2d_Conic(Sol.Hyperbola()));
                }
                else if (type == GccInt_Ell)
                {
                    Intp.Perform(OnCirc, new IntAna2d_Conic(Sol.Ellipse()));
                }
                if (Intp.IsDone())
                {
                    if ((!Intp.IsEmpty()) && (!Intp.ParallelElements()) && (!Intp.IdenticalElements()))
                    {
                        for (int j = 1; j <= Intp.NbPoints(); j++)
                        {
                            Point2D Center = Intp.Point(j).Value();
                            double dist1 = center1.Distance(Center);
                            int nbsol = 1;
                            bool ok = false;
                            // OCCT: the radius follows Qualified1 only, whichever branch of the bisector the centre is
                            // on, so with C1 qualified (and for both radii when it is not) a solution need not pass
                            // through Point2
                            if (Qualified1.IsEnclosed())
                            {
                                if (dist1 - C1.Radius() <= Tol)
                                {
                                    ok = true;
                                    Radius[1] = Math.Abs(C1.Radius() - dist1);
                                }
                            }
                            else if (Qualified1.IsOutside())
                            {
                                if (C1.Radius() - dist1 <= Tol)
                                {
                                    ok = true;
                                    Radius[1] = Math.Abs(C1.Radius() - dist1);
                                }
                            }
                            else if (Qualified1.IsEnclosing())
                            {
                                ok = true;
                                Radius[1] = C1.Radius() + dist1;
                            }
                            else if (Qualified1.IsUnqualified())
                            {
                                ok = true;
                                nbsol = 2;
                                Radius[1] = Math.Abs(C1.Radius() - dist1);
                                Radius[2] = C1.Radius() + dist1;
                            }
                            if (ok)
                            {
                                for (int k = 1; k <= nbsol; k++)
                                {
                                    // pop : protection against case center1 == Center
                                    if (center1.IsEqual(Center, global::Kerf.Engine.Foundation.Precision.Confusion()))
                                    {
                                        continue;
                                    }
                                    if (OnCirc.Distance(Center) > Tol)
                                    {
                                        continue;
                                    }
                                    if (NbrSol == 4)
                                    {
                                        break;
                                    }
                                    NbrSol++;
                                    cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius[k]);
                                    //              ==========================================================
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
                                    qualifier2[NbrSol] = GccEnt_noqualifier;
                                    if (distcc1 <= Tol && Math.Abs(Radius[k] - C1.Radius()) <= Tol)
                                    {
                                        TheSame1[NbrSol] = 1;
                                    }
                                    else
                                    {
                                        TheSame1[NbrSol] = 0;
                                        Direction2D dc1 = new(center1.XY() - Center.XY());
                                        pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius[k] * dc1.XY());
                                        par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                        pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                    }
                                    TheSame2[NbrSol] = 0;
                                    pntcen[NbrSol] = Center;
                                    pnttg2sol[NbrSol] = Point2;
                                    pararg2[NbrSol] = 0.0;
                                    par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                    parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen[NbrSol]);
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
