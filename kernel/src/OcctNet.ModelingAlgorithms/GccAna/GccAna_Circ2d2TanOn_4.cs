// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    //=========================================================================
    //   Creation of a circle Tangent to : 1 straight line L1.                +
    //                        Passing by : 1 point Point2.                    +
    //                        Centered on  : 1 straight line OnLine.                  +
    //   with a Tolerance of precision  : Tolerance.                        +
    //                                                                        +
    //  We start by making difference with various boundary cases that will be +
    //  processed separately.                                            +
    //  For the general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to L1 and Point2 that give us       +
    //  all possible locations of centers of all circles        +
    //  tangent to L1 and passing through Point2.                                  +
    //  We intersect these bissectrices with straight line OnLine which gives us +
    //  the points among which we'll choose the solutions.   +
    //  The choices are made basing on Qualifieurs of L1.        +
    //=========================================================================

    /// <summary>The circles tangent to a line and through a point with their centre on a line.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedLin Qualified1,
                               in Point2D Point2,
                               in Line2D OnLine,
                               double Tolerance)
        : this(4)
    {
        TheSame1.Init(0);
        TheSame2.Init(0);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double Tol = Math.Abs(Tolerance);
        Direction2D dirx = new(Direction2D.D.X);
        Line2D L1 = Qualified1.Qualified();
        Point2D originL1 = L1.Location();
        Direction2D dirL1 = L1.Direction();
        Direction2D normal = new(-dirL1.Y(), dirL1.X());

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        if (dirL1.IsEqual(OnLine.Direction(), global::Kerf.Engine.Foundation.Precision.Confusion())
            && OnLine.Distance(originL1) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // POP : l2s 2 straight line are identic : no Sol
            NbrSol = 0;
            return;
        }

        double dp2l = OnLine.Distance(Point2);
        Direction2D donline = OnLine.Direction();
        Point2D pinterm = new(Point2.XY() + dp2l * new Xy(-donline.Y(), donline.X()));
        if (OnLine.Distance(pinterm) > Tol)
        {
            pinterm = new Point2D(Point2.XY() - dp2l * new Xy(-donline.Y(), donline.X()));
        }
        double dist = L1.Distance(pinterm);
        if (Math.Abs(dist - dp2l) <= Tol)
        {
            Direction2D dirbid = new(originL1.XY() - pinterm.XY());
            if (Qualified1.IsEnclosed() && dirbid.Dot(normal) < 0.0)
            {
                WellDone = true;
            }
            else if (Qualified1.IsOutside() && dirbid.Dot(normal) > 0.0)
            {
                WellDone = true;
            }
            else if (Qualified1.IsUnqualified())
            {
                WellDone = true;
            }
            if (WellDone)
            {
                NbrSol++;
                cirsol[NbrSol] = new Circle2D(new Axis2D(pinterm, dirx), dp2l);
                //    ======================================================
                qualifier2[NbrSol] = GccEnt_noqualifier;
                Direction2D dc2 = new(originL1.XY() - pinterm.XY());
                if (!Qualified1.IsUnqualified())
                {
                    qualifier1[NbrSol] = Qualified1.Qualifier();
                }
                else if (dc2.Dot(normal) > 0.0)
                {
                    qualifier1[NbrSol] = GccEnt_outside;
                }
                else
                {
                    qualifier1[NbrSol] = GccEnt_enclosed;
                }
                double sign = dc2.Dot(new Direction2D(-dirL1.Y(), dirL1.X()));
                dc2 = new Direction2D(sign * new Xy(-dirL1.Y(), dirL1.X()));
                pnttg1sol[NbrSol] = new Point2D(pinterm.XY() + dp2l * dc2.XY());
                pnttg2sol[NbrSol] = Point2;
                pntcen[NbrSol] = pinterm;
                par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                pararg2[NbrSol] = 0.0;
                parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnLine, pntcen[NbrSol]);
                return;
            }
        }

        //=========================================================================
        //   General case.                                                        +
        //=========================================================================

        GccAna_LinPnt2dBisec Bis = new(L1, Point2);
        if (Bis.IsDone())
        {
            GccInt_Bisec Sol = Bis.ThisSolution();
            GccInt_IType type = Sol.ArcType();
            AnalyticIntersection Intp = new();
            if (type == GccInt_Lin)
            {
                Intp.Perform(OnLine, Sol.Line());
            }
            if (type == GccInt_Par)
            {
                Intp.Perform(OnLine, new Conic(Sol.Parabola()));
            }
            if (Intp.IsDone)
            {
                if (!Intp.IsEmpty)
                {
                    for (int j = 1; j <= Intp.PointCount; j++)
                    {
                        Point2D Center = Intp.Point(j).Value();
                        double Radius = L1.Distance(Center);
                        //	  int nbsol = 1;
                        bool ok = false;
                        if (Qualified1.IsEnclosed())
                        {
                            if ((((originL1.X() - Center.X()) * (-dirL1.Y()))
                                 + ((originL1.Y() - Center.Y()) * (dirL1.X())))
                                <= 0)
                            {
                                ok = true;
                            }
                        }
                        else if (Qualified1.IsOutside())
                        {
                            if ((((originL1.X() - Center.X()) * (-dirL1.Y()))
                                 + ((originL1.Y() - Center.Y()) * (dirL1.X())))
                                >= 0)
                            {
                                ok = true;
                            }
                        }
                        else if (Qualified1.IsUnqualified())
                        {
                            ok = true;
                        }
                        if (ok)
                        {
                            NbrSol++;
                            cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                            //          =======================================================
                            qualifier2[NbrSol] = GccEnt_noqualifier;
                            Direction2D dc2 = new(originL1.XY() - Center.XY());
                            if (!Qualified1.IsUnqualified())
                            {
                                qualifier1[NbrSol] = Qualified1.Qualifier();
                            }
                            else if (dc2.Dot(normal) > 0.0)
                            {
                                qualifier1[NbrSol] = GccEnt_outside;
                            }
                            else
                            {
                                qualifier1[NbrSol] = GccEnt_enclosed;
                            }
                            TheSame1[NbrSol] = 0;
                            TheSame2[NbrSol] = 0;
                            Direction2D dc1 = new(originL1.XY() - Center.XY());
                            double sign = dc1.Dot(normal);
                            dc1 = new Direction2D(sign * (normal.XY()));
                            pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius * dc1.XY());
                            pnttg2sol[NbrSol] = Point2;
                            pntcen[NbrSol] = Center;
                            par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                            par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                            pararg2[NbrSol] = 0.0;
                            parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnLine, pntcen[NbrSol]);
                        }
                    }
                }
                WellDone = true;
            }
        }
    }
}
