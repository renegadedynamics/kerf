// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_5.cxx
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
    //===========================================================================
    //   Creation of a circle tangent to a circle, a straight line and a point. +
    //===========================================================================
    public AnalyticCirc2d3Tan(in QualifiedCirc qualified1,
        in QualifiedLin qualified2,
        in Point2D point3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 4);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 4);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 4);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 4);
        _theSame1 = new NCollection_Array1<int>(1, 4);
        _theSame2 = new NCollection_Array1<int>(1, 4);
        _theSame3 = new NCollection_Array1<int>(1, 4);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 4);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 4);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 4);
        _par1sol = new NCollection_Array1<double>(1, 4);
        _par2sol = new NCollection_Array1<double>(1, 4);
        _par3sol = new NCollection_Array1<double>(1, 4);
        _pararg1 = new NCollection_Array1<double>(1, 4);
        _pararg2 = new NCollection_Array1<double>(1, 4);
        _pararg3 = new NCollection_Array1<double>(1, 4);

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        double maxRad = 1e10, minRad = 1e-6;
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D c1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        double r1 = c1.Radius;
        Point2D center1 = c1.Location;
        Point2D origin2 = l2.Location;
        Direction2D dir2 = l2.Direction;
        Direction2D normL2 = new(-dir2.Y, dir2.X);

        NCollection_Array1<double> radius = new(1, 2);
        CircLin2dBisec bis1 = new(c1, l2);
        LinPnt2dBisec bis2 = new(l2, point3);
        if (bis1.IsDone && bis2.IsDone)
        {
            int nbsolution1 = bis1.SolutionCount;
            for (int i = 1; i <= nbsolution1; i++)
            {
                ConstraintBisec sol1 = bis1.ThisSolution(i);
                ConstraintBisec sol2 = bis2.ThisSolution();
                ConstraintCurveType typ1 = sol1.ArcType();
                ConstraintCurveType typ2 = sol2.ArcType();
                AnalyticIntersection intp = new();
                if (typ1 == ConstraintCurveType.Lin)
                {
                    if (typ2 == ConstraintCurveType.Lin)
                    {
                        intp.Perform(sol1.Line(), sol2.Line());
                    }
                    else if (typ2 == ConstraintCurveType.Par)
                    {
                        intp.Perform(sol1.Line(), new Conic(sol2.Parabola()));
                    }
                }
                else if (typ1 == ConstraintCurveType.Par)
                {
                    if (typ2 == ConstraintCurveType.Lin)
                    {
                        intp.Perform(sol2.Line(), new Conic(sol1.Parabola()));
                    }
                    else if (typ2 == ConstraintCurveType.Par)
                    {
                        intp.Perform(sol1.Parabola(), new Conic(sol2.Parabola()));
                    }
                }
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double dist1 = center.DistanceTo(c1.Location);
                            double dist2 = l2.DistanceTo(center);
                            double dist3 = center.DistanceTo(point3);
                            int nbsol1 = 0;
                            int nbsol3 = 0;
                            bool ok = false;
                            if (qualified1.IsEnclosed)
                            {
                                if (dist1 - r1 < tolerance)
                                {
                                    radius[1] = Math.Abs(r1 - dist1);
                                    nbsol1 = 1;
                                    ok = true;
                                }
                            }
                            else if (qualified1.IsOutside)
                            {
                                if (r1 - dist1 < tolerance)
                                {
                                    radius[1] = Math.Abs(r1 - dist1);
                                    nbsol1 = 1;
                                    ok = true;
                                }
                            }
                            else if (qualified1.IsEnclosing)
                            {
                                ok = true;
                                nbsol1 = 1;
                                radius[1] = Math.Abs(r1 - dist1);
                            }
                            else if (qualified1.IsUnqualified)
                            {
                                ok = true;
                                nbsol1 = 2;
                                radius[1] = Math.Abs(r1 - dist1);
                                radius[2] = r1 + dist1;
                            }
                            if (qualified2.IsEnclosed && ok)
                            {
                                if ((((l2.Location.X - center.X) * (-l2.Direction.Y))
                                     + ((l2.Location.Y - center.Y) * (l2.Direction.X)))
                                    <= 0)
                                {
                                    for (int ii = 1; ii <= nbsol1; ii++)
                                    {
                                        if (Math.Abs(dist2 - radius[ii]) < tol)
                                        {
                                            ok = true;
                                            radius[1] = radius[ii];
                                        }
                                    }
                                }
                            }
                            else if (qualified2.IsOutside && ok)
                            {
                                if ((((l2.Location.X - center.X) * (-l2.Direction.Y))
                                     + ((l2.Location.Y - center.Y) * (l2.Direction.X)))
                                    >= 0)
                                {
                                    for (int ii = 1; ii <= nbsol1; ii++)
                                    {
                                        if (Math.Abs(dist2 - radius[ii]) < tol)
                                        {
                                            ok = true;
                                            radius[1] = radius[ii];
                                        }
                                    }
                                }
                            }
                            else if (qualified2.IsUnqualified && ok)
                            {
                                for (int ii = 1; ii <= nbsol1; ii++)
                                {
                                    if (Math.Abs(dist2 - radius[ii]) < tol)
                                    {
                                        ok = true;
                                        radius[1] = radius[ii];
                                    }
                                }
                            }
                            if (Math.Abs(dist3 - radius[1]) <= tol && ok)
                            {
                                ok = true;
                                nbsol3 = 1;
                            }
                            if (ok)
                            {
                                for (int k = 1; k <= nbsol3; k++)
                                {
                                    if (_nbrSol == 4)
                                    {
                                        break;
                                    }
                                    // pop : if the radius is too great - no creation
                                    if (radius[k] > maxRad)
                                    {
                                        break;
                                    }
                                    if (Math.Abs(radius[k]) < minRad)
                                    {
                                        break;
                                    }

                                    _nbrSol++;
                                    _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius[k]);
                                    //               ==========================================================
                                    double distcc1 = center.DistanceTo(center1);
                                    if (!qualified1.IsUnqualified)
                                    {
                                        _qualifier1[_nbrSol] = qualified1.Qualifier();
                                    }
                                    else if (Math.Abs(distcc1 + radius[k] - r1) < tol)
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                    }
                                    else if (Math.Abs(distcc1 - r1 - radius[k]) < tol)
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Outside;
                                    }
                                    else
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Enclosing;
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
                                    if (center.DistanceTo(c1.Location) <= tolerance
                                        && Math.Abs(radius[k] - r1) <= tolerance)
                                    {
                                        _theSame1[_nbrSol] = 1;
                                    }
                                    else
                                    {
                                        _theSame1[_nbrSol] = 0;
                                        //  modified by NIZHNY-EAP Mon Nov  1 13:48:21 1999 ___BEGIN___
                                        //                   gp_Dir2d dc(C1.Location().XY()-Center.XY());
                                        // OCCT: the reversed direction puts the tangency point on the far side of the solution unless it is enclosed in C1
                                        Direction2D dc1 = new(center.XY() - c1.Location.XY());
                                        //  modified by NIZHNY-EAP Mon Nov  1 13:48:55 1999 ___END___
                                        _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[k] * dc1.XY());
                                        _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                        _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                    }
                                    _theSame2[_nbrSol] = 0;
                                    _theSame3[_nbrSol] = 0;
                                    Direction2D dc = new(l2.Location.XY() - center.XY());
                                    double sign = dc.Dot(new Direction2D(-l2.Direction.Y, l2.Direction.X));
                                    dc = new Direction2D(sign * new Xy(-l2.Direction.Y, l2.Direction.X));
                                    _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius[k] * dc.XY());
                                    _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                    _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
                                    _pnttg3sol[_nbrSol] = point3;
                                    _par3sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg3sol[_nbrSol]);
                                    _pararg3[_nbrSol] = 0.0;
                                }
                            }
                        }
                    }
                    _wellDone = true;
                }
                if (_nbrSol == 4)
                {
                    break;
                }
            }
        }
    }
}
