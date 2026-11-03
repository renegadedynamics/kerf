// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_10.cxx
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
    //   Creation of a circle tangent to straight line L1 and a point Point2.   +
    //                        centered on a circle.                           +
    //  We start by making difference between boundary cases that will be    +
    //  processed separately.                                            +
    //  In general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to L1 and Point2 that give us all        +
    //  all possible locations of centers of all circles        +
    //  tangent to L1 and Point2.                                              +
    //  We intersect these bissectrices with circle OnCirc which gives us +
    //  points among which we'll choose the solutions.   +
    //  The choice is made using Qualifiers of L1.        +
    //=========================================================================

    /// <summary>The circles tangent to a line and through a point with their centre on a circle.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedLin Qualified1,
                               in Point2D Point2,
                               in Circle2D OnCirc,
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
        Direction2D normL1 = new(-dirL1.Y(), dirL1.X());

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        double Ron = OnCirc.Radius();
        double distpc = OnCirc.Location().Distance(Point2);
        Direction2D dir = new(OnCirc.Location().XY() - Point2.XY());
        Point2D pinterm = new(Point2.XY() + (distpc + Ron) * dir.XY());
        double dist1 = L1.Distance(pinterm);

        // OCCT: pinterm is distpc + Ron from Point2 but its distance to L1 is compared with distpc - Ron, so a circle
        // found here doesn't pass through Point2 unless OnCirc has radius 0
        if (Math.Abs(dist1 - distpc + Ron) <= Tol)
        {
            dir = new Direction2D(-dirL1.Y(), dirL1.X());
            Direction2D direc = new(originL1.XY() - pinterm.XY());
            if (Qualified1.IsOutside())
            {
                if (direc.Dot(dir) >= 0.0)
                {
                    WellDone = true;
                }
            }
            else if (Qualified1.IsEnclosed())
            {
                if (direc.Dot(dir) <= 0.0)
                {
                    WellDone = true;
                }
            }
            else
            {
                WellDone = true;
            }
            if (WellDone)
            {
                NbrSol++;
                cirsol[NbrSol] = new Circle2D(new Axis2D(pinterm, dirx), dist1);
                //    =======================================================
                Direction2D dc1 = new(originL1.XY() - pinterm.XY());
                double sign = dc1.Dot(normL1);
                if (!Qualified1.IsUnqualified())
                {
                    qualifier1[NbrSol] = Qualified1.Qualifier();
                }
                else if (dc1.Dot(normL1) > 0.0)
                {
                    qualifier1[NbrSol] = GccEnt_outside;
                }
                else
                {
                    qualifier1[NbrSol] = GccEnt_enclosed;
                }
                qualifier2[NbrSol] = GccEnt_noqualifier;
                dc1 = new Direction2D(sign * new Xy(-dirL1.Y(), dirL1.X()));
                pnttg1sol[NbrSol] = new Point2D(pinterm.XY() + dist1 * dc1.XY());
                par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                pntcen[NbrSol] = pinterm;
                parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen[NbrSol]);
                parcen3[NbrSol] = 0.0; // OCCT: overwrites the centre's parameter just computed
                pnttg2sol[NbrSol] = Point2;
                pararg2[NbrSol] = 0.0;
                par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                return;
            }
        }

        //=========================================================================
        //   General case.                                                         +
        //=========================================================================

        GccAna_LinPnt2dBisec Bis = new(L1, Point2);
        if (Bis.IsDone())
        {
            GccInt_Bisec Sol = Bis.ThisSolution();
            GccInt_IType type = Sol.ArcType();
            AnalyticIntersection Intp = new();
            if (type == GccInt_Lin)
            {
                Intp.Perform(Sol.Line(), OnCirc);
            }
            if (type == GccInt_Par)
            {
                Intp.Perform(OnCirc, new Conic(Sol.Parabola()));
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
                            TheSame1[NbrSol] = 0;
                            TheSame2[NbrSol] = 0;
                            Direction2D dc1 = new(originL1.XY() - Center.XY());
                            double sign = dc1.Dot(normL1);
                            if (!Qualified1.IsUnqualified())
                            {
                                qualifier1[NbrSol] = Qualified1.Qualifier();
                            }
                            else if (dc1.Dot(normL1) > 0.0)
                            {
                                qualifier1[NbrSol] = GccEnt_outside;
                            }
                            else
                            {
                                qualifier1[NbrSol] = GccEnt_enclosed;
                            }
                            qualifier2[NbrSol] = GccEnt_noqualifier;
                            dc1 = new Direction2D(sign * new Xy(-dirL1.Y(), dirL1.X()));
                            // OCCT: dist1 (the boundary case's distance) instead of Radius, so the tangency point is
                            // off the line unless the two happen to be equal
                            pnttg1sol[NbrSol] = new Point2D(Center.XY() + dist1 * dc1.XY());
                            par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                            pntcen[NbrSol] = Center;
                            parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen[NbrSol]);
                            pnttg2sol[NbrSol] = Point2;
                            pararg2[NbrSol] = 0.0;
                            par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                            qualifier2[NbrSol] = GccEnt_noqualifier;
                        }
                    }
                }
                WellDone = true;
            }
        }
    }
}
