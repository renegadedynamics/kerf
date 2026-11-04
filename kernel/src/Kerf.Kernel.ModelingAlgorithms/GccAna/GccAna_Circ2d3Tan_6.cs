// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d3Tan
{
    //=========================================================================
    //   Creation of a circle tangent to two straight lines and a point.      +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedLin qualified1,
        in QualifiedLin qualified2,
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

        Direction2D dirx = new(Direction2D.D.X);
        _wellDone = false;
        double tol = Math.Abs(tolerance);
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        _pnttg3sol.Init(point3);

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Line2D l1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        Point2D origin1 = l1.Location;
        Direction2D dir1 = l1.Direction;
        Direction2D normL1 = new(-dir1.Y, dir1.X);
        Point2D origin2 = l2.Location;
        Direction2D dir2 = l2.Direction;
        Direction2D normL2 = new(-dir2.Y, dir2.X);

        Lin2dBisec bis1 = new(l1, l2);
        LinPnt2dBisec bis2 = new(l1, point3);
        if (bis1.IsDone && bis2.IsDone)
        {
            int nbsolution1 = bis1.SolutionCount;
            ConstraintBisec sol2 = bis2.ThisSolution();
            for (int i = 1; i <= nbsolution1; i++)
            {
                Line2D sol1 = bis1.ThisSolution(i);
                ConstraintCurveType typ2 = sol2.ArcType();
                AnalyticIntersection intp = new();
                if (typ2 == ConstraintCurveType.Lin)
                {
                    intp.Perform(sol1, sol2.Line());
                }
                else if (typ2 == ConstraintCurveType.Par)
                {
                    intp.Perform(sol1, new Conic(sol2.Parabola()));
                }
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double dist1 = l1.DistanceTo(center);
                            double dist2 = l2.DistanceTo(center);
                            double dist3 = center.DistanceTo(point3);
                            double radius = 0;
                            int nbsol3 = 0;
                            bool ok = false;
                            if (qualified1.IsEnclosed)
                            {
                                if ((((origin1.X - center.X) * (-dir1.Y))
                                     + ((origin1.Y - center.Y) * (dir1.X)))
                                    <= 0)
                                {
                                    ok = true;
                                    radius = dist1;
                                }
                            }
                            else if (qualified1.IsOutside)
                            {
                                if ((((origin1.X - center.X) * (-dir1.Y))
                                     + ((origin1.Y - center.Y) * (dir1.X)))
                                    >= 0)
                                {
                                    ok = true;
                                    radius = dist1;
                                }
                            }
                            else if (qualified1.IsUnqualified)
                            {
                                ok = true;
                                radius = dist1;
                            }
                            if (qualified2.IsEnclosed)
                            {
                                if ((((origin2.X - center.X) * (-dir2.Y))
                                     + ((origin2.Y - center.Y) * (dir2.X)))
                                    <= 0)
                                {
                                    if (Math.Abs(dist2 - radius) < tol)
                                    {
                                    }
                                    else
                                    {
                                        ok = false;
                                    }
                                }
                            }
                            else if (qualified2.IsOutside && ok)
                            {
                                if ((((origin2.X - center.X) * (-dir2.Y))
                                     + ((origin2.Y - center.Y) * (dir2.X)))
                                    >= 0)
                                {
                                    if (Math.Abs(dist2 - radius) < tol)
                                    {
                                    }
                                    else
                                    {
                                        ok = false;
                                    }
                                }
                            }
                            else if (qualified2.IsUnqualified && ok)
                            {
                                if (Math.Abs(dist2 - radius) < tol)
                                {
                                }
                                else
                                {
                                    ok = false;
                                }
                            }
                            if (ok)
                            {
                                if (Math.Abs(dist3 - radius) < tol)
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
                                    //               =======================================================
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
                                    Direction2D dc2 = new(origin2.XY() - center.XY());
                                    if (!qualified2.IsUnqualified)
                                    {
                                        _qualifier2[_nbrSol] = qualified2.Qualifier();
                                    }
                                    else if (dc2.Dot(normL2) > 0.0)
                                    {
                                        _qualifier2[_nbrSol] = Qualifier.Outside;
                                    }
                                    else
                                    {
                                        _qualifier2[_nbrSol] = Qualifier.Enclosed;
                                    }
                                    _qualifier3[_nbrSol] = Qualifier.Noqualifier;
                                    _theSame1[_nbrSol] = 0;
                                    Direction2D dc = new(origin1.XY() - center.XY());
                                    double sign = dc.Dot(new Direction2D(-dir1.Y, dir1.X));
                                    dc = new Direction2D(sign * new Xy(-dir1.Y, dir1.X));
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc.XY());
                                    _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                    _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                                    _theSame2[_nbrSol] = 0;
                                    dc = new Direction2D(origin2.XY() - center.XY());
                                    sign = dc.Dot(new Direction2D(-dir2.Y, dir2.X));
                                    dc = new Direction2D(sign * new Xy(-dir2.Y, dir2.X));
                                    _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius * dc.XY());
                                    _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                    _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
                                    _theSame3[_nbrSol] = 0;
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
}
