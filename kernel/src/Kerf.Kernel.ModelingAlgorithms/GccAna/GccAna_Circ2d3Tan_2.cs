// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_2.cxx
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
    //   Creation of a circle tangent to a circle and two straight lines.          +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedCirc qualified1,
        in QualifiedLin qualified2,
        in QualifiedLin qualified3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 8);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 8);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 8);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 8);
        _theSame1 = new NCollection_Array1<int>(1, 8);
        _theSame2 = new NCollection_Array1<int>(1, 8);
        _theSame3 = new NCollection_Array1<int>(1, 8);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 8);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 8);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 8);
        _par1sol = new NCollection_Array1<double>(1, 8);
        _par2sol = new NCollection_Array1<double>(1, 8);
        _par3sol = new NCollection_Array1<double>(1, 8);
        _pararg1 = new NCollection_Array1<double>(1, 8);
        _pararg2 = new NCollection_Array1<double>(1, 8);
        _pararg3 = new NCollection_Array1<double>(1, 8);

        _theSame1.Init(0);

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified)
            || !(qualified3.IsEnclosed || qualified3.IsOutside || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D c1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        Line2D l3 = qualified3.Qualified();
        double r1 = c1.Radius;
        Point2D center1 = c1.Location;
        Point2D origin2 = l2.Location;
        Direction2D dir2 = l2.Direction;
        Direction2D normL2 = new(-dir2.Y, dir2.X);
        Point2D origin3 = l3.Location;
        Direction2D dir3 = l3.Direction;
        Direction2D normL3 = new(-dir3.Y, dir3.X);

        NCollection_Array1<double> radius = new(1, 2);
        CircLin2dBisec bis1 = new(c1, l2);
        Lin2dBisec bis2 = new(l2, l3);
        if (bis1.IsDone && bis2.IsDone)
        {
            int nbsolution1 = bis1.SolutionCount;
            int nbsolution2 = bis2.SolutionCount;
            for (int i = 1; i <= nbsolution1; i++)
            {
                ConstraintBisec sol1 = bis1.ThisSolution(i);
                ConstraintCurveType typ1 = sol1.ArcType();
                AnalyticIntersection intp = new();
                for (int k = 1; k <= nbsolution2; k++)
                {
                    if (typ1 == ConstraintCurveType.Lin)
                    {
                        intp.Perform(sol1.Line(), bis2.ThisSolution(k));
                    }
                    else if (typ1 == ConstraintCurveType.Par)
                    {
                        intp.Perform(bis2.ThisSolution(k), new Conic(sol1.Parabola()));
                    }
                    if (intp.IsDone)
                    {
                        if ((!intp.IsEmpty) && (!intp.ParallelElements()) && (!intp.IdenticalElements()))
                        {
                            for (int j = 1; j <= intp.PointCount; j++)
                            {
                                Point2D center = intp.Point(j).Value();
                                double dist1 = center.DistanceTo(center1);
                                double dist2 = l2.DistanceTo(center);
                                double dist3 = l3.DistanceTo(center);
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
                                    if ((((origin2.X - center.X) * (-dir2.Y))
                                         + ((origin2.Y - center.Y) * (dir2.X)))
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
                                    if ((((origin2.X - center.X) * (-dir2.Y))
                                         + ((origin2.Y - center.Y) * (dir2.X)))
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
                                if (qualified3.IsEnclosed && ok)
                                {
                                    if ((((origin3.X - center.X) * (-dir3.Y))
                                         + ((origin3.Y - center.Y) * (dir3.X)))
                                        <= 0)
                                    {
                                        if (Math.Abs(dist3 - radius[1]) < tol)
                                        {
                                            ok = true;
                                            nbsol3 = 1;
                                        }
                                    }
                                }
                                else if (qualified3.IsOutside && ok)
                                {
                                    if ((((origin3.X - center.X) * (-dir3.Y))
                                         + ((origin3.Y - center.Y) * (dir3.X)))
                                        >= 0)
                                    {
                                        if (Math.Abs(dist3 - radius[1]) < tol)
                                        {
                                            ok = true;
                                            nbsol3 = 1;
                                        }
                                    }
                                }
                                else if (qualified3.IsUnqualified && ok)
                                {
                                    if (Math.Abs(dist3 - radius[1]) < tol)
                                    {
                                        ok = true;
                                        nbsol3 = 1;
                                    }
                                }
                                if (ok)
                                {
                                    for (int m = 1; m <= nbsol3; m++)
                                    {
                                        _nbrSol++;
                                        _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius[m]);
                                        //                 ==========================================================
                                        double distcc1 = center.DistanceTo(center1);
                                        if (!qualified1.IsUnqualified)
                                        {
                                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                                        }
                                        else if (Math.Abs(distcc1 + radius[m] - r1) < tol)
                                        {
                                            _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                        }
                                        else if (Math.Abs(distcc1 - r1 - radius[m]) < tol)
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
                                        Direction2D dc3 = new(origin3.XY() - center.XY());
                                        if (!qualified3.IsUnqualified)
                                        {
                                            _qualifier3[_nbrSol] = qualified3.Qualifier();
                                        }
                                        else if (dc3.Dot(normL3) > 0.0)
                                        {
                                            _qualifier3[_nbrSol] = Qualifier.Outside;
                                        }
                                        else
                                        {
                                            _qualifier3[_nbrSol] = Qualifier.Enclosed;
                                        }
                                        if (center.DistanceTo(center1) <= tolerance
                                            && Math.Abs(radius[m] - r1) <= tolerance)
                                        {
                                            _theSame1[_nbrSol] = 1;
                                        }
                                        else
                                        {
                                            _theSame1[_nbrSol] = 0;
                                            Direction2D dc1 = new(center1.XY() - center.XY());
                                            _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[m] * dc1.XY());
                                            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                        }
                                        _theSame2[_nbrSol] = 0;
                                        _theSame3[_nbrSol] = 0;
                                        Direction2D dc = new(origin2.XY() - center.XY());
                                        double sign = dc.Dot(new Direction2D(-dir2.Y, dir2.X));
                                        dc = new Direction2D(sign * new Xy(-dir2.Y, dir2.X));
                                        _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius[m] * dc.XY());
                                        _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                        _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
                                        dc = new Direction2D(origin3.XY() - center.XY());
                                        sign = dc.Dot(new Direction2D(-dir3.Y, dir3.X));
                                        dc = new Direction2D(sign * new Xy(-dir3.Y, dir3.X));
                                        _pnttg3sol[_nbrSol] = new Point2D(center.XY() + radius[m] * dc.XY());
                                        _par3sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg3sol[_nbrSol]);
                                        _pararg3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l3, _pnttg3sol[_nbrSol]);
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
}
