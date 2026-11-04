// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d3Tan
{
    private const int MaxSol = 20;

    //=========================================================================
    //   Creation of a circle tangent to two circles and a point.           +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedCirc qualified1,
        in QualifiedCirc qualified2,
        in Point2D point3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, MaxSol);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, MaxSol);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, MaxSol);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, MaxSol);
        _theSame1 = new NCollection_Array1<int>(1, MaxSol);
        _theSame2 = new NCollection_Array1<int>(1, MaxSol);
        _theSame3 = new NCollection_Array1<int>(1, MaxSol);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, MaxSol);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, MaxSol);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, MaxSol);
        _par1sol = new NCollection_Array1<double>(1, MaxSol);
        _par2sol = new NCollection_Array1<double>(1, MaxSol);
        _par3sol = new NCollection_Array1<double>(1, MaxSol);
        _pararg1 = new NCollection_Array1<double>(1, MaxSol);
        _pararg2 = new NCollection_Array1<double>(1, MaxSol);
        _pararg3 = new NCollection_Array1<double>(1, MaxSol);

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();
        double r1 = c1.Radius;
        double r2 = c2.Radius;
        Point2D center1 = c1.Location;
        Point2D center2 = c2.Location;

        NCollection_Array1<double> radius = new(1, 2);
        Circ2dBisec bis1 = new(c1, c2);
        CircPnt2dBisec bis2 = new(c1, point3);
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
                    if (typ1 == ConstraintCurveType.Cir)
                    {
                        if (typ2 == ConstraintCurveType.Cir)
                        {
                            intp.Perform(sol1.Circle(), sol2.Circle());
                        }
                        else if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol2.Line(), sol1.Circle());
                        }
                        else if (typ2 == ConstraintCurveType.Hpr)
                        {
                            intp.Perform(sol1.Circle(), new Conic(sol2.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Ell)
                        {
                            intp.Perform(sol1.Circle(), new Conic(sol2.Ellipse()));
                        }
                    }
                    else if (typ1 == ConstraintCurveType.Ell)
                    {
                        if (typ2 == ConstraintCurveType.Cir)
                        {
                            intp.Perform(sol2.Circle(), new Conic(sol1.Ellipse()));
                        }
                        else if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol2.Line(), new Conic(sol1.Ellipse()));
                        }
                        else if (typ2 == ConstraintCurveType.Hpr)
                        {
                            intp.Perform(sol1.Ellipse(), new Conic(sol2.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Ell)
                        {
                            intp.Perform(sol1.Ellipse(), new Conic(sol2.Ellipse()));
                        }
                    }
                    else if (typ1 == ConstraintCurveType.Lin)
                    {
                        if (typ2 == ConstraintCurveType.Cir)
                        {
                            intp.Perform(sol1.Line(), sol2.Circle());
                        }
                        else if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol1.Line(), sol2.Line());
                        }
                        else if (typ2 == ConstraintCurveType.Hpr)
                        {
                            intp.Perform(sol1.Line(), new Conic(sol2.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Ell)
                        {
                            intp.Perform(sol1.Line(), new Conic(sol2.Ellipse()));
                        }
                    }
                    else if (typ1 == ConstraintCurveType.Hpr)
                    {
                        if (typ2 == ConstraintCurveType.Cir)
                        {
                            intp.Perform(sol2.Circle(), new Conic(sol1.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol2.Line(), new Conic(sol1.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Hpr)
                        {
                            intp.Perform(sol2.Hyperbola(), new Conic(sol1.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Ell)
                        {
                            intp.Perform(sol2.Ellipse(), new Conic(sol1.Hyperbola()));
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
                                double dist1 = center.DistanceTo(center1);
                                double dist2 = center.DistanceTo(center2);
                                double dist3 = center.DistanceTo(point3);
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
                                    radius[1] = r1 + dist1;
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
                                for (int ii = 1; ii <= nbsol2; ii++)
                                {
                                    if (Math.Abs(dist3 - radius[ii]) <= tol)
                                    {
                                        nbsol3++;
                                        ok = true;
                                    }
                                }
                                if (ok)
                                {
                                    for (int k1 = 1; k1 <= nbsol3; k1++)
                                    {
                                        _nbrSol++;
                                        _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius[k1]);
                                        //                 ==========================================================
                                        double distcc1 = center.DistanceTo(center1);
                                        if (!qualified1.IsUnqualified)
                                        {
                                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                                        }
                                        else if (Math.Abs(distcc1 + radius[k1] - r1) < tol)
                                        {
                                            _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                        }
                                        else if (Math.Abs(distcc1 - r1 - radius[k1]) < tol)
                                        {
                                            _qualifier1[_nbrSol] = Qualifier.Outside;
                                        }
                                        else
                                        {
                                            _qualifier1[_nbrSol] = Qualifier.Enclosing;
                                        }

                                        //                   double distcc2 = Center.Distance(center1);
                                        double distcc2 = center.DistanceTo(center2);
                                        if (!qualified2.IsUnqualified)
                                        {
                                            _qualifier2[_nbrSol] = qualified2.Qualifier();
                                        }
                                        else if (Math.Abs(distcc2 + radius[k1] - r2) < tol)
                                        {
                                            _qualifier2[_nbrSol] = Qualifier.Enclosed;
                                        }
                                        else if (Math.Abs(distcc2 - r2 - radius[k1]) < tol)
                                        {
                                            _qualifier2[_nbrSol] = Qualifier.Outside;
                                        }
                                        else
                                        {
                                            _qualifier2[_nbrSol] = Qualifier.Enclosing;
                                        }
                                        _qualifier3[_nbrSol] = Qualifier.Noqualifier;
                                        if (center.DistanceTo(center1) <= tolerance
                                            && Math.Abs(radius[k1] - r1) <= tolerance)
                                        {
                                            _theSame1[_nbrSol] = 1;
                                        }
                                        else
                                        {
                                            _theSame1[_nbrSol] = 0;
                                            Direction2D dc = new(center1.XY() - center.XY());
                                            if (_qualifier1[_nbrSol] == Qualifier.Enclosed)
                                            {
                                                // if tangent circle is inside the source circle, moving to edge of source circle
                                                dc = dc.Reversed();
                                            }
                                            _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[k1] * dc.XY());
                                            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                        }
                                        if (center.DistanceTo(center2) <= tolerance
                                            && Math.Abs(radius[k1] - r2) <= tolerance)
                                        {
                                            _theSame2[_nbrSol] = 1;
                                        }
                                        else
                                        {
                                            _theSame2[_nbrSol] = 0;
                                            Direction2D dc = new(center2.XY() - center.XY());
                                            // case of concentric circles :
                                            // 2nd tangency point is at the other side of the circle solution
                                            double alpha = 1.0;
                                            if (center1.DistanceTo(center2) <= tolerance)
                                            {
                                                alpha = -1;
                                            }
                                            _pnttg2sol[_nbrSol] = new Point2D(center.XY() + alpha * radius[k1] * dc.XY());
                                            _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                            _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c2, _pnttg2sol[_nbrSol]);
                                        }
                                        _theSame3[_nbrSol] = 0;
                                        _pnttg3sol[_nbrSol] = point3;
                                        _par3sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg3sol[_nbrSol]);
                                        _pararg3[_nbrSol] = 0.0;
                                        _wellDone = true;
                                        if (_nbrSol == MaxSol)
                                        {
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        _wellDone = true;
                        if (_nbrSol == MaxSol)
                        {
                            break;
                        }
                    }
                    if (_nbrSol == MaxSol)
                    {
                        break;
                    }
                }
                if (_nbrSol == MaxSol)
                {
                    break;
                }
            }
        }

        // Debug to create the point on the solution circles.

        int kk;
        for (kk = 1; kk <= _nbrSol; kk++)
        {
            Circle2D cc = _cirsol[kk];
            double nr = cc.Location.DistanceTo(point3);
            if (Math.Abs(nr - cc.Radius) > tol)
            {
                _cirsol[kk] = _cirsol[kk].WithRadius(nr);
            }
        }

        // Debug to eliminate multiple solution.
        // this happens in case of intersection line hyperbola.
        double tol2 = tol * tol;
        for (kk = 1; kk < _nbrSol; kk++)
        {
            Point2D pk = _cirsol[kk].Location;
            for (int ll = kk + 1; ll <= _nbrSol; ll++)
            {
                Point2D pl = _cirsol[ll].Location;
                if (pk.SquareDistanceTo(pl) < tol2)
                {
                    for (int mm = ll + 1; mm <= _nbrSol; mm++)
                    {
                        _cirsol[mm - 1] = _cirsol[mm];
                        // OCCT: TheSame1..3 are not shifted with the other arrays
                        _pnttg1sol[mm - 1] = _pnttg1sol[mm];
                        _pnttg2sol[mm - 1] = _pnttg2sol[mm];
                        _pnttg3sol[mm - 1] = _pnttg3sol[mm];
                        _par1sol[mm - 1] = _par1sol[mm];
                        _par2sol[mm - 1] = _par2sol[mm];
                        _par3sol[mm - 1] = _par3sol[mm];
                        _pararg1[mm - 1] = _pararg1[mm];
                        _pararg2[mm - 1] = _pararg2[mm];
                        _pararg3[mm - 1] = _pararg3[mm];
                        _qualifier1[mm - 1] = _qualifier1[mm];
                        _qualifier2[mm - 1] = _qualifier2[mm];
                        _qualifier3[mm - 1] = _qualifier3[mm];
                    }
                    _nbrSol--;
                }
            }
        }
    }
}
