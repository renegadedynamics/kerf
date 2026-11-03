// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_8.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d3Tan
{
    //=========================================================================
    //   Creation of a circle tangent to a straight line and two points.      +
    //=========================================================================
    public GccAna_Circ2d3Tan(in GccEnt_QualifiedLin Qualified1,
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

        WellDone = false;
        double Tol = Math.Abs(Tolerance);
        Direction2D dirx = new(Direction2D.D.X);
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Line2D L1 = Qualified1.Qualified();
        Point2D origin1 = L1.Location();
        Direction2D dir1 = L1.Direction();
        Direction2D normL1 = new(-dir1.Y(), dir1.X());

        if (Point2.IsEqual(Point3, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            WellDone = false;
            return;
        }

        GccAna_Pnt2dBisec Bis1 = new(Point2, Point3);
        GccAna_LinPnt2dBisec Bis2 = new(L1, Point2);
        if (Bis1.IsDone() && Bis2.IsDone())
        {
            Line2D linint1 = Bis1.ThisSolution();
            GccInt_Bisec Sol2 = Bis2.ThisSolution();
            GccInt_IType typ2 = Sol2.ArcType();
            AnalyticIntersection Intp = new();
            if (typ2 == GccInt_Lin)
            {
                Line2D linint2 = Sol2.Line();
                Intp.Perform(linint1, linint2);
            }
            else if (typ2 == GccInt_Par)
            {
                Intp.Perform(linint1, new Conic(Sol2.Parabola()));
            }
            if (Intp.IsDone)
            {
                if ((!Intp.IsEmpty) && (!Intp.ParallelElements()) && (!Intp.IdenticalElements()))
                {
                    for (int j = 1; j <= Intp.PointCount; j++)
                    {
                        Point2D Center = Intp.Point(j).Value();
                        double dist1 = L1.Distance(Center);
                        double dist2 = Center.Distance(Point2);

                        double Radius = 0;
                        int nbsol3 = 0;
                        bool ok = false;
                        double difference = (((origin1.X() - Center.X()) * (-dir1.Y()))
                                             + ((origin1.Y() - Center.Y()) * (dir1.X())));
                        if ((Qualified1.IsEnclosed() && difference <= 0)
                            || (Qualified1.IsOutside() && difference >= 0) || (Qualified1.IsUnqualified()))
                        {
                            ok = true;
                            Radius = dist1;
                        }
                        if (ok)
                        {
                            if (Math.Abs(dist2 - Radius) <= Tol)
                            {
                                nbsol3 = 1;
                            }
                            else
                            {
                                ok = false;
                            }
                        }
                        if (ok)
                        {
                            for (int k = 1; k <= nbsol3; k++)
                            {
                                NbrSol++;
                                cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                                //             =======================================================
                                Direction2D dc1 = new(origin1.XY() - Center.XY());
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
                                qualifier3[NbrSol] = GccEnt_noqualifier;
                                TheSame1[NbrSol] = 0;
                                Direction2D dc = new(origin1.XY() - Center.XY());
                                double sign = dc.Dot(new Direction2D(-dir1.Y(), dir1.X()));
                                dc = new Direction2D(sign * new Xy(-dir1.Y(), dir1.X()));
                                pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius * dc.XY());
                                par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                                TheSame2[NbrSol] = 0;
                                pnttg2sol[NbrSol] = Point2;
                                par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                pararg2[NbrSol] = 0.0;
                                TheSame3[NbrSol] = 0;
                                pnttg3sol[NbrSol] = Point3;
                                par3sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg3sol[NbrSol]);
                                pararg3[NbrSol] = 0.0;
                            }
                        }
                    }
                }
                WellDone = true;
            }
        }
    }
}
