// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_1.cxx
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
    //   Creation of a circle tangent to two circles and a straight line.     +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedCirc qualified1,
        in QualifiedCirc qualified2,
        in QualifiedLin qualified3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 16);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 16);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 16);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 16);
        _theSame1 = new NCollection_Array1<int>(1, 16);
        _theSame2 = new NCollection_Array1<int>(1, 16);
        _theSame3 = new NCollection_Array1<int>(1, 16);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 16);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 16);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 16);
        _par1sol = new NCollection_Array1<double>(1, 16);
        _par2sol = new NCollection_Array1<double>(1, 16);
        _par3sol = new NCollection_Array1<double>(1, 16);
        _pararg1 = new NCollection_Array1<double>(1, 16);
        _pararg2 = new NCollection_Array1<double>(1, 16);
        _pararg3 = new NCollection_Array1<double>(1, 16);

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified)
            || !(qualified3.IsEnclosed || qualified3.IsOutside || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();
        Line2D l3 = qualified3.Qualified();
        double r1 = c1.Radius;
        double r2 = c2.Radius;
        Point2D center1 = c1.Location;
        Point2D center2 = c2.Location;
        _ = center2;

        Point2D origin3 = l3.Location;
        Direction2D dir3 = l3.Direction;
        Direction2D normL3 = new(-dir3.Y, dir3.X);

        NCollection_Array1<double> radius = new(1, 2);
        CircLin2dBisec bis1 = new(c1, l3);
        CircLin2dBisec bis2 = new(c2, l3);
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
                    ConstraintBisec sol2 = bis2.ThisSolution(k);
                    ConstraintCurveType typ2 = sol2.ArcType();
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
                                double rradius = 0;
                                Point2D center = intp.Point(j).Value();

                                // pop : if the coordinates are too great, no creation
                                if (center.X > 1e10 || center.Y > 1e10)
                                {
                                    break;
                                }

                                double dist1 = center.DistanceTo(c1.Location);
                                double dist2 = center.DistanceTo(c2.Location);
                                double dist3 = l3.DistanceTo(center);

                                // pop : if the coordinates are too great, no creation
                                if (dist3 > 1e10)
                                {
                                    break;
                                }

                                int nbsol1 = 0;
                                int nbsol2 = 0;
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
                                    if (dist2 - r2 < tolerance)
                                    {
                                        for (int ii = 1; ii <= nbsol1; ii++)
                                        {
                                            if (Math.Abs(radius[ii] - Math.Abs(r2 - dist2)) < tol)
                                            {
                                                radius[1] = Math.Abs(r2 - dist2);
                                                ok = true;
                                                nbsol2 = 1;
                                            }
                                        }
                                    }
                                }
                                else if (qualified2.IsOutside && ok)
                                {
                                    if (r2 - dist2 < tolerance)
                                    {
                                        for (int ii = 1; ii <= nbsol1; ii++)
                                        {
                                            if (Math.Abs(radius[ii] - Math.Abs(r2 - dist2)) < tol)
                                            {
                                                radius[1] = Math.Abs(r2 - dist2);
                                                ok = true;
                                                nbsol2 = 1;
                                            }
                                        }
                                    }
                                }
                                else if (qualified2.IsEnclosing && ok)
                                {
                                    for (int ii = 1; ii <= nbsol1; ii++)
                                    {
                                        if (Math.Abs(radius[ii] - r2 - dist2) < tol)
                                        {
                                            radius[1] = r2 + dist2;
                                            ok = true;
                                            nbsol2 = 1;
                                        }
                                    }
                                }
                                else if (qualified2.IsUnqualified && ok)
                                {
                                    for (int ii = 1; ii <= nbsol1; ii++)
                                    {
                                        if (Math.Abs(radius[ii] - Math.Abs(r2 - dist2)) < tol)
                                        {
                                            rradius = Math.Abs(r2 - dist2);
                                            ok = true;
                                            nbsol2++;
                                        }
                                        else if (Math.Abs(radius[ii] - r2 - dist2) < tol)
                                        {
                                            rradius = r2 + dist2;
                                            ok = true;
                                            nbsol2++;
                                        }
                                    }
                                    if (nbsol2 == 1)
                                    {
                                        radius[1] = rradius;
                                    }
                                    else if (nbsol2 == 2)
                                    {
                                        radius[1] = Math.Abs(r2 - dist2);
                                        radius[2] = r2 + dist2;
                                    }
                                }
                                if (qualified3.IsEnclosed && ok)
                                {
                                    if ((((l3.Location.X - center.X) * (-l3.Direction.Y))
                                         + ((l3.Location.Y - center.Y) * (l3.Direction.X)))
                                        <= 0)
                                    {
                                        ok = true;
                                        nbsol3 = 1;
                                    }
                                }
                                // OCCT: Qualified2 where the line's Qualified3 is meant
                                else if (qualified2.IsOutside && ok)
                                {
                                    if ((((l3.Location.X - center.X) * (-l3.Direction.Y))
                                         + ((l3.Location.Y - center.Y) * (l3.Direction.X)))
                                        >= 0)
                                    {
                                        ok = true;
                                        nbsol3 = 1;
                                    }
                                }
                                // OCCT: Qualified2 where the line's Qualified3 is meant
                                else if (qualified2.IsUnqualified && ok)
                                {
                                    ok = true;
                                    nbsol3 = 1;
                                }
                                // OCCT: ok is not reset when the second circle's or the line's check fails, and the distance to the line is never compared with the radius: solutions not tangent to all three arguments are kept
                                if (ok)
                                {
                                    for (int ind3 = 1; ind3 <= nbsol3; ind3++)
                                    {
                                        _nbrSol++;
                                        _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius[ind3]);
                                        //                ==========================================================
                                        double distcc1 = center.DistanceTo(center1);
                                        if (!qualified1.IsUnqualified)
                                        {
                                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                                        }
                                        else if (Math.Abs(distcc1 + radius[ind3] - r1) < tol)
                                        {
                                            _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                        }
                                        else if (Math.Abs(distcc1 - r1 - radius[ind3]) < tol)
                                        {
                                            _qualifier1[_nbrSol] = Qualifier.Outside;
                                        }
                                        else
                                        {
                                            _qualifier1[_nbrSol] = Qualifier.Enclosing;
                                        }
                                        // OCCT: measured from center1, not center2
                                        double distcc2 = center.DistanceTo(center1);
                                        if (!qualified2.IsUnqualified)
                                        {
                                            _qualifier2[_nbrSol] = qualified2.Qualifier();
                                        }
                                        else if (Math.Abs(distcc2 + radius[ind3] - r2) < tol)
                                        {
                                            _qualifier2[_nbrSol] = Qualifier.Enclosed;
                                        }
                                        else if (Math.Abs(distcc2 - r2 - radius[ind3]) < tol)
                                        {
                                            _qualifier2[_nbrSol] = Qualifier.Outside;
                                        }
                                        else
                                        {
                                            _qualifier2[_nbrSol] = Qualifier.Enclosing;
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
                                        if (center.DistanceTo(c1.Location) <= tolerance
                                            && Math.Abs(radius[ind3] - r1) <= tolerance)
                                        {
                                            _theSame1[_nbrSol] = 1;
                                        }
                                        else
                                        {
                                            _theSame1[_nbrSol] = 0;
                                            Direction2D dc1 = new(c1.Location.XY() - center.XY());
                                            _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[ind3] * dc1.XY());
                                            // POP for protection if cirsol[NbrSol].Location == pnttg1sol[NbrSol]
                                            if (_cirsol[_nbrSol].Location.IsEqual(_pnttg1sol[_nbrSol],
                                                                                  global::Kerf.Engine.Foundation.Precision.Confusion()))
                                            {
                                                _par1sol[_nbrSol] = 1;
                                            }
                                            else
                                            {
                                                _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                            }
                                            // POP for protection if C1.Location == pnttg1sol[NbrSol]
                                            if (c1.Location.IsEqual(_pnttg1sol[_nbrSol], global::Kerf.Engine.Foundation.Precision.Confusion()))
                                            {
                                                _pararg1[_nbrSol] = 1;
                                            }
                                            else
                                            {
                                                _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                            }
                                        }
                                        if (center.DistanceTo(c2.Location) <= tolerance
                                            && Math.Abs(radius[ind3] - r2) <= tolerance)
                                        {
                                            _theSame2[_nbrSol] = 1;
                                        }
                                        else
                                        {
                                            _theSame2[_nbrSol] = 0;
                                            Direction2D dc2 = new(c2.Location.XY() - center.XY());
                                            _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius[ind3] * dc2.XY());
                                            // POP for protection if cirsol[NbrSol].Location == pnttg1sol[NbrSol]
                                            if (_cirsol[_nbrSol].Location.IsEqual(_pnttg1sol[_nbrSol],
                                                                                  global::Kerf.Engine.Foundation.Precision.Confusion()))
                                            {
                                                // OCCT: tests pnttg1sol and writes par1sol in the second tangency's block
                                                _par1sol[_nbrSol] = 1;
                                            }
                                            else
                                            {
                                                _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                            }
                                            // POP for protection if C2.Location == pnttg2sol[NbrSol]
                                            if (c2.Location.IsEqual(_pnttg2sol[_nbrSol], global::Kerf.Engine.Foundation.Precision.Confusion()))
                                            {
                                                _pararg2[_nbrSol] = 1;
                                            }
                                            else
                                            {
                                                _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c2, _pnttg2sol[_nbrSol]);
                                            }
                                        }
                                        _theSame3[_nbrSol] = 0;
                                        Direction2D dc = new(l3.Location.XY() - center.XY());
                                        double sign = dc.Dot(new Direction2D(-l3.Direction.Y, l3.Direction.X));
                                        dc = new Direction2D(sign * new Xy(-l3.Direction.Y, l3.Direction.X));
                                        _pnttg3sol[_nbrSol] = new Point2D(center.XY() + radius[ind3] * dc.XY());
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
