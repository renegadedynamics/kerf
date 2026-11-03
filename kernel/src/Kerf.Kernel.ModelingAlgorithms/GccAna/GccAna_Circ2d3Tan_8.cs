// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_8.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2d3Tan
{
    //=========================================================================
    //   Creation of a circle tangent to a straight line and two points.      +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedLin qualified1,
        in Point2D point2,
        in Point2D point3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 2);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 2);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 2);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 2);
        _theSame1 = new NCollection_Array1<int>(1, 2);
        _theSame2 = new NCollection_Array1<int>(1, 2);
        _theSame3 = new NCollection_Array1<int>(1, 2);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 2);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 2);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 2);
        _par1sol = new NCollection_Array1<double>(1, 2);
        _par2sol = new NCollection_Array1<double>(1, 2);
        _par3sol = new NCollection_Array1<double>(1, 2);
        _pararg1 = new NCollection_Array1<double>(1, 2);
        _pararg2 = new NCollection_Array1<double>(1, 2);
        _pararg3 = new NCollection_Array1<double>(1, 2);

        _wellDone = false;
        double tol = Math.Abs(tolerance);
        Direction2D dirx = new(Direction2D.D.X);
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Line2D l1 = qualified1.Qualified();
        Point2D origin1 = l1.Location;
        Direction2D dir1 = l1.Direction;
        Direction2D normL1 = new(-dir1.Y, dir1.X);

        if (point2.IsEqual(point3, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            _wellDone = false;
            return;
        }

        Pnt2dBisec bis1 = new(point2, point3);
        LinPnt2dBisec bis2 = new(l1, point2);
        if (bis1.IsDone && bis2.IsDone)
        {
            Line2D linint1 = bis1.ThisSolution();
            ConstraintBisec sol2 = bis2.ThisSolution();
            ConstraintCurveType typ2 = sol2.ArcType();
            AnalyticIntersection intp = new();
            if (typ2 == ConstraintCurveType.Lin)
            {
                Line2D linint2 = sol2.Line();
                intp.Perform(linint1, linint2);
            }
            else if (typ2 == ConstraintCurveType.Par)
            {
                intp.Perform(linint1, new Conic(sol2.Parabola()));
            }
            if (intp.IsDone)
            {
                if ((!intp.IsEmpty) && (!intp.ParallelElements()) && (!intp.IdenticalElements()))
                {
                    for (int j = 1; j <= intp.PointCount; j++)
                    {
                        Point2D center = intp.Point(j).Value();
                        double dist1 = l1.DistanceTo(center);
                        double dist2 = center.DistanceTo(point2);

                        double radius = 0;
                        int nbsol3 = 0;
                        bool ok = false;
                        double difference = (((origin1.X - center.X) * (-dir1.Y))
                                             + ((origin1.Y - center.Y) * (dir1.X)));
                        if ((qualified1.IsEnclosed && difference <= 0)
                            || (qualified1.IsOutside && difference >= 0) || (qualified1.IsUnqualified))
                        {
                            ok = true;
                            radius = dist1;
                        }
                        if (ok)
                        {
                            if (Math.Abs(dist2 - radius) <= tol)
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
                                _nbrSol++;
                                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                //             =======================================================
                                Direction2D dc1 = new(origin1.XY() - center.XY());
                                if (!qualified1.IsUnqualified)
                                {
                                    _qualifier1[_nbrSol] = qualified1.Qualifier();
                                }
                                else if (dc1.Dot(normL1) > 0.0)
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                }
                                _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                                _qualifier3[_nbrSol] = Qualifier.Noqualifier;
                                _theSame1[_nbrSol] = 0;
                                Direction2D dc = new(origin1.XY() - center.XY());
                                double sign = dc.Dot(new Direction2D(-dir1.Y, dir1.X));
                                dc = new Direction2D(sign * new Xy(-dir1.Y, dir1.X));
                                _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc.XY());
                                _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                                _theSame2[_nbrSol] = 0;
                                _pnttg2sol[_nbrSol] = point2;
                                _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                _pararg2[_nbrSol] = 0.0;
                                _theSame3[_nbrSol] = 0;
                                _pnttg3sol[_nbrSol] = point3;
                                _par3sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg3sol[_nbrSol]);
                                _pararg3[_nbrSol] = 0.0;
                            }
                        }
                    }
                }
                _wellDone = true;
            }
        }
    }
}
