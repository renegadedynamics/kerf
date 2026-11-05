// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_7.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d3Tan
{
    //=======================================================================
    //   Creation of a circle tangent to a circle and two points.           +
    //=======================================================================
    public AnalyticCirc2d3Tan(in QualifiedCirc qualified1,
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

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D c1 = qualified1.Qualified();
        double r1 = c1.Radius;
        Point2D center1 = c1.Location;
        NCollection_Array1<double> radius = new(1, 2);

        if (point2.IsEqual(point3, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            _wellDone = false;
            return;
        }

        Pnt2dBisec bis1 = new(point2, point3);
        CircPnt2dBisec bis2 = new(c1, point2);

        if (bis1.IsDone && bis2.IsDone)
        {
            int nbsolution2 = bis2.SolutionCount;
            for (int i = 1; i <= nbsolution2; i++)
            {
                ConstraintBisec sol2 = bis2.ThisSolution(i);
                ConstraintCurveType typ2 = sol2.ArcType();
                Line2D sol1 = bis1.ThisSolution();
                AnalyticIntersection intp = new();
                if (typ2 == ConstraintCurveType.Cir)
                {
                    intp.Perform(sol1, sol2.Circle());
                }
                else if (typ2 == ConstraintCurveType.Lin)
                {
                    intp.Perform(sol1, sol2.Line());
                }
                else if (typ2 == ConstraintCurveType.Hpr)
                {
                    intp.Perform(sol1, new Conic(sol2.Hyperbola()));
                }
                else if (typ2 == ConstraintCurveType.Ell)
                {
                    intp.Perform(sol1, new Conic(sol2.Ellipse()));
                }

                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double dist1 = center.DistanceTo(center1);
                            double dist2 = center.DistanceTo(point2);
                            double dist3 = center.DistanceTo(point3);
                            int nbsol1 = 0;
                            //             int nbsol2 = 0;
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
                            if (ok)
                            {
                                ok = false;
                                for (int ii = 1; ii <= nbsol1; ii++)
                                {
                                    // pop                 if (Math.Abs(dist2-Radius[ii])<=Tol &&
                                    // Math.Abs(dist2-Radius[ii])<=Tol){
                                    if (Math.Abs(dist2 - radius[ii]) <= tol && Math.Abs(dist3 - radius[ii]) <= tol)
                                    {
                                        nbsol3 = ii;
                                        ok = true;
                                    }
                                }
                            }

                            if (ok)
                            {
                                //               for (int k = 1 ; k <= nbsol3 ; k++) {
                                if (_nbrSol >= 2)
                                {
                                    break;
                                }
                                _nbrSol++;
                                //                 cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center,dirx),Radius[k]);
                                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius[nbsol3]);
                                //               ==========================================================
                                double distcc1 = center.DistanceTo(center1);
                                if (!qualified1.IsUnqualified)
                                {
                                    _qualifier1[_nbrSol] = qualified1.Qualifier();
                                }
                                else if (Math.Abs(distcc1 + radius[nbsol3] - r1) < tol)
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                }
                                else if (Math.Abs(distcc1 - r1 - radius[nbsol3]) < tol)
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Enclosing;
                                }
                                _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                                _qualifier3[_nbrSol] = Qualifier.Noqualifier;
                                if (center.DistanceTo(center1) <= tolerance
                                    && Math.Abs(radius[nbsol3] - r1) <= tolerance)
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
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[nbsol3] * dc.XY());
                                    _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                    _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                }

                                _theSame2[_nbrSol] = 0;
                                _pnttg2sol[_nbrSol] = point2;
                                _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                _pararg2[_nbrSol] = 0.0;
                                _theSame3[_nbrSol] = 0;
                                _pnttg3sol[_nbrSol] = point3;
                                _par3sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg3sol[_nbrSol]);
                                _pararg3[_nbrSol] = 0.0;
                                //}
                            }
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }
}
