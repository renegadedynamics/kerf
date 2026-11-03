// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_7.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d3Tan
{
    //=======================================================================
    //   Creation of a circle tangent to a circle and two points.           +
    //=======================================================================
    public GccAna_Circ2d3Tan(in GccEnt_QualifiedCirc Qualified1,
        in Point2D Point2,
        in Point2D Point3,
        double Tolerance)
    {
        cirsol = new NCollection_Array1<Circle2D>(1, 2);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, 2);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, 2);
        qualifier3 = new NCollection_Array1<GccEnt_Position>(1, 2);
        TheSame1 = new NCollection_Array1<int>(1, 2);
        TheSame2 = new NCollection_Array1<int>(1, 2);
        TheSame3 = new NCollection_Array1<int>(1, 2);
        pnttg1sol = new NCollection_Array1<Point2D>(1, 2);
        pnttg2sol = new NCollection_Array1<Point2D>(1, 2);
        pnttg3sol = new NCollection_Array1<Point2D>(1, 2);
        par1sol = new NCollection_Array1<double>(1, 2);
        par2sol = new NCollection_Array1<double>(1, 2);
        par3sol = new NCollection_Array1<double>(1, 2);
        pararg1 = new NCollection_Array1<double>(1, 2);
        pararg2 = new NCollection_Array1<double>(1, 2);
        pararg3 = new NCollection_Array1<double>(1, 2);

        Direction2D dirx = new(Direction2D.D.X);
        double Tol = Math.Abs(Tolerance);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D C1 = Qualified1.Qualified();
        double R1 = C1.Radius();
        Point2D center1 = C1.Location();
        NCollection_Array1<double> Radius = new(1, 2);

        if (Point2.IsEqual(Point3, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            WellDone = false;
            return;
        }

        GccAna_Pnt2dBisec Bis1 = new(Point2, Point3);
        GccAna_CircPnt2dBisec Bis2 = new(C1, Point2);

        if (Bis1.IsDone() && Bis2.IsDone())
        {
            int nbsolution2 = Bis2.NbSolutions();
            for (int i = 1; i <= nbsolution2; i++)
            {
                GccInt_Bisec Sol2 = Bis2.ThisSolution(i);
                GccInt_IType typ2 = Sol2.ArcType();
                Line2D Sol1 = Bis1.ThisSolution();
                AnalyticIntersection Intp = new();
                if (typ2 == GccInt_Cir)
                {
                    Intp.Perform(Sol1, Sol2.Circle());
                }
                else if (typ2 == GccInt_Lin)
                {
                    Intp.Perform(Sol1, Sol2.Line());
                }
                else if (typ2 == GccInt_Hpr)
                {
                    Intp.Perform(Sol1, new Conic(Sol2.Hyperbola()));
                }
                else if (typ2 == GccInt_Ell)
                {
                    Intp.Perform(Sol1, new Conic(Sol2.Ellipse()));
                }

                if (Intp.IsDone)
                {
                    if (!Intp.IsEmpty)
                    {
                        for (int j = 1; j <= Intp.PointCount; j++)
                        {
                            Point2D Center = Intp.Point(j).Value();
                            double dist1 = Center.Distance(center1);
                            double dist2 = Center.Distance(Point2);
                            double dist3 = Center.Distance(Point3);
                            int nbsol1 = 0;
                            //             int nbsol2 = 0;
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
                                Radius[1] = R1 + dist1;
                            }
                            else if (Qualified1.IsUnqualified())
                            {
                                ok = true;
                                nbsol1 = 2;
                                Radius[1] = Math.Abs(R1 - dist1);
                                Radius[2] = R1 + dist1;
                            }
                            if (ok)
                            {
                                ok = false;
                                for (int ii = 1; ii <= nbsol1; ii++)
                                {
                                    // pop                 if (Math.Abs(dist2-Radius[ii])<=Tol &&
                                    // Math.Abs(dist2-Radius[ii])<=Tol){
                                    if (Math.Abs(dist2 - Radius[ii]) <= Tol && Math.Abs(dist3 - Radius[ii]) <= Tol)
                                    {
                                        nbsol3 = ii;
                                        ok = true;
                                    }
                                }
                            }

                            if (ok)
                            {
                                //               for (int k = 1 ; k <= nbsol3 ; k++) {
                                if (NbrSol >= 2)
                                {
                                    break;
                                }
                                NbrSol++;
                                //                 cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center,dirx),Radius[k]);
                                cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius[nbsol3]);
                                //               ==========================================================
                                double distcc1 = Center.Distance(center1);
                                if (!Qualified1.IsUnqualified())
                                {
                                    qualifier1[NbrSol] = Qualified1.Qualifier();
                                }
                                else if (Math.Abs(distcc1 + Radius[nbsol3] - R1) < Tol)
                                {
                                    qualifier1[NbrSol] = GccEnt_enclosed;
                                }
                                else if (Math.Abs(distcc1 - R1 - Radius[nbsol3]) < Tol)
                                {
                                    qualifier1[NbrSol] = GccEnt_outside;
                                }
                                else
                                {
                                    qualifier1[NbrSol] = GccEnt_enclosing;
                                }
                                qualifier2[NbrSol] = GccEnt_noqualifier;
                                qualifier3[NbrSol] = GccEnt_noqualifier;
                                if (Center.Distance(center1) <= Tolerance
                                    && Math.Abs(Radius[nbsol3] - R1) <= Tolerance)
                                {
                                    TheSame1[NbrSol] = 1;
                                }
                                else
                                {
                                    TheSame1[NbrSol] = 0;
                                    Direction2D dc = new(center1.XY() - Center.XY());
                                    if (qualifier1[NbrSol] == GccEnt_enclosed)
                                    {
                                        // if tangent circle is inside the source circle, moving to edge of source circle
                                        dc.Reverse();
                                    }
                                    pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius[nbsol3] * dc.XY());
                                    par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                    pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                }

                                TheSame2[NbrSol] = 0;
                                pnttg2sol[NbrSol] = Point2;
                                par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                pararg2[NbrSol] = 0.0;
                                TheSame3[NbrSol] = 0;
                                pnttg3sol[NbrSol] = Point3;
                                par3sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg3sol[NbrSol]);
                                pararg3[NbrSol] = 0.0;
                                //}
                            }
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }
}
