// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    //=========================================================================
    //  Circles tangent to circle C1, passing by point Point2 and centers     +
    //  on a straight line OnLine.                                            +
    //  We start by making difference with boundary cases that will be        +
    //  processed separately.                                                 +
    //  For the general case:                                                 +
    //  ====================                                                  +
    //  We calculate bissectrices to C1 and Point2 that give us all           +
    //  possible locations of centers of all circles                          +
    //  tangent to C1 and passing by Point2.                                  +
    //  We intersect these bissectrices with the straight line OnLine which   +
    //  gives us the points among which we'll choose the solutions.           +
    //  The choices are made using Qualifiers of C1 and C2.                   +
    //=========================================================================

    /// <summary>The circles tangent to a circle and through a point with their centre on a line.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedCirc Qualified1,
                               in Point2D Point2,
                               in Line2D OnLine,
                               double Tolerance)
        : this(4)
    {
        TheSame1.Init(0);
        TheSame2.Init(0);
        double Tol = Math.Abs(Tolerance);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        NCollection_Array1<double> Radius = new(1, 2);
        Direction2D dirx = new(Direction2D.D.X);
        Circle2D C1 = Qualified1.Qualified();
        double R1 = C1.Radius();
        Point2D center1 = C1.Location();

        //=========================================================================
        //   Processing of boundary cases.                                        +
        //=========================================================================

        double dp2l = OnLine.Distance(Point2);
        Direction2D donline = OnLine.Direction();
        Point2D pinterm = new(Point2.XY() + dp2l * new Xy(-donline.Y(), donline.X()));
        if (OnLine.Distance(pinterm) > Tol)
        {
            pinterm = new Point2D(Point2.XY() - dp2l * new Xy(-donline.Y(), donline.X()));
        }
        double dist = pinterm.Distance(center1);
        if (Qualified1.IsEnclosed() && Math.Abs(R1 - dist - dp2l) <= Tol)
        {
            WellDone = true;
        }
        else if (Qualified1.IsEnclosing() && Math.Abs(R1 + dist - dp2l) <= Tol)
        {
            WellDone = true;
        }
        else if (Qualified1.IsOutside() && Math.Abs(dist - dp2l) <= Tol)
        {
            WellDone = true;
        }
        else if (Qualified1.IsUnqualified()
                 && (Math.Abs(dist - dp2l) <= Tol || Math.Abs(R1 - dist - dp2l) <= Tol
                     || Math.Abs(R1 + dist - dp2l) <= Tol))
        {
            WellDone = true;
        }
        if (WellDone)
        {
            NbrSol++;
            cirsol[NbrSol] = new Circle2D(new Axis2D(pinterm, dirx), dp2l);
            //  ======================================================
            Direction2D dc1 = new(center1.XY() - pinterm.XY());
            double distcc1 = pinterm.Distance(center1);
            if (!Qualified1.IsUnqualified())
            {
                qualifier1[NbrSol] = Qualified1.Qualifier();
            }
            else if (Math.Abs(distcc1 + dp2l - R1) < Tol)
            {
                qualifier1[NbrSol] = GccEnt_enclosed;
            }
            else if (Math.Abs(distcc1 - R1 - dp2l) < Tol)
            {
                qualifier1[NbrSol] = GccEnt_outside;
            }
            else
            {
                qualifier1[NbrSol] = GccEnt_enclosing;
            }
            qualifier2[NbrSol] = GccEnt_noqualifier;
            pnttg1sol[NbrSol] = new Point2D(pinterm.XY() + dp2l * dc1.XY());
            par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
            pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
            pnttg2sol[NbrSol] = Point2;
            par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
            pntcen[NbrSol] = cirsol[NbrSol].Location();
            parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnLine, pntcen[NbrSol]);
            return;
        }

        //=========================================================================
        //   General case.                                                       +
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
                    Intp.Perform(OnLine, Sol.Line());
                }
                else if (type == GccInt_Cir)
                {
                    Intp.Perform(OnLine, Sol.Circle());
                }
                else if (type == GccInt_Ell)
                {
                    Intp.Perform(OnLine, new IntAna2d_Conic(Sol.Ellipse()));
                }
                else if (type == GccInt_Hpr)
                {
                    Intp.Perform(OnLine, new IntAna2d_Conic(Sol.Hyperbola()));
                }
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int j = 1; j <= Intp.NbPoints(); j++)
                        {
                            Point2D Center = Intp.Point(j).Value();
                            double dist1 = center1.Distance(Center);
                            int nbsol = 1;
                            bool ok = false;
                            // OCCT: with C1 qualified the radius follows the qualifier only, whichever branch of the
                            // bisector the centre is on, so a solution need not pass through Point2
                            if (Qualified1.IsEnclosed())
                            {
                                if (dist1 - C1.Radius() <= Tolerance)
                                {
                                    ok = true;
                                    Radius[1] = Math.Abs(C1.Radius() - dist1);
                                }
                            }
                            else if (Qualified1.IsOutside())
                            {
                                if (C1.Radius() - dist1 <= Tolerance)
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
                            /*	    else if (Qualified1.IsUnqualified() && ok) {
                                      ok = true;
                                      nbsol = 2;
                                      Radius(1) = std::abs(C1.Radius()-dist1);
                                      Radius(2) = C1.Radius()+dist1;
                                    }
                            */
                            else if (Qualified1.IsUnqualified())
                            {
                                double popradius = Center.Distance(Point2);
                                // OCCT: tests the difference as a boolean (non-zero), not against a tolerance
                                if (Math.Abs(popradius - dist1) != 0.0)
                                {
                                    ok = true;
                                    Radius[1] = popradius;
                                }
                            }

                            if (ok)
                            {
                                for (int k = 1; k <= nbsol; k++)
                                {
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
                                    if (Center.Distance(center1) <= Tolerance
                                        && Math.Abs(Radius[k] - C1.Radius()) <= Tolerance)
                                    {
                                        TheSame1[NbrSol] = 1;
                                    }
                                    else
                                    {
                                        TheSame1[NbrSol] = 0;
                                        Direction2D dc1 = new(center1.XY() - Center.XY());
                                        pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius[k] * dc1.XY());
                                        par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                        // OCCT: indexed by the bisector's number i instead of NbrSol
                                        pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                    }
                                    TheSame2[NbrSol] = 0;
                                    pnttg2sol[NbrSol] = Point2;
                                    par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                    pararg2[NbrSol] = 0.0;
                                    pntcen[NbrSol] = Center;
                                    parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnLine, pntcen[NbrSol]);
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
