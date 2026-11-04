// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_8.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanOn
{
    /// <summary>The circles tangent to a circle and through a point with their centre on a circle.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in Point2D point2,
                               in Circle2D onCirc,
                               double tolerance)
        : this(4)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double tol = Math.Abs(tolerance);
        NCollection_Array1<double> radius = new(1, 2);
        Direction2D dirx = new(Direction2D.D.X);
        Circle2D c1 = qualified1.Qualified();
        double r1 = c1.Radius;
        Point2D center1 = c1.Location;

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        int nbsol1 = 1;
        //  int nbsol2 = 0;
        double ron = onCirc.Radius;
        double distcco = onCirc.Location.DistanceTo(center1);
        Point2D pinterm = new();
        Direction2D dircc = new();
        bool sameCenter = false;
        if (!onCirc.Location.IsEqual(center1, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            dircc = new Direction2D(onCirc.Location.XY() - center1.XY());
            pinterm = new Point2D(center1.XY() + (distcco - ron) * dircc.XY());
            double distcc2 = pinterm.DistanceTo(point2);
            double distcc1 = pinterm.DistanceTo(center1);
            double d1 = Math.Abs(distcc2 - Math.Abs(distcc1 - r1));
            double d2 = Math.Abs(distcc2 - (distcc1 + r1));
            if (d1 > tol || d2 > tol)
            {
                if (!sameCenter)
                {
                    pinterm = new Point2D(center1.XY() + (distcco + ron) * dircc.XY());
                }
                distcc2 = pinterm.DistanceTo(point2);
                distcc1 = pinterm.DistanceTo(center1);
                d1 = Math.Abs(distcc2 - Math.Abs(distcc1 - r1));
                d2 = Math.Abs(distcc2 - (distcc1 + r1));
                if (d1 > tol || d2 > tol)
                {
                    nbsol1 = 0;
                }
            }
            if (nbsol1 > 0)
            {
                if (qualified1.IsEnclosed || qualified1.IsOutside)
                {
                    nbsol1 = 1;
                    radius[1] = Math.Abs(distcc1 - r1);
                }
                else if (qualified1.IsEnclosing)
                {
                    nbsol1 = 1;
                    radius[1] = r1 + distcc1;
                }
                else if (qualified1.IsUnqualified)
                {
                    nbsol1 = 2;
                    radius[1] = Math.Abs(distcc1 - r1);
                    radius[2] = r1 + distcc1;
                }
                for (int i = 1; i <= nbsol1; i++)
                {
                    _wellDone = true;
                    if (i != 1 || !sameCenter)
                    {
                        _nbrSol++;
                        _cirsol[_nbrSol] = new Circle2D(new Axis2D(pinterm, dirx), radius[i]);
                        //    ===========================================================
                        Direction2D dc1 = new();
                        if (!sameCenter)
                        {
                            dc1 = new Direction2D(center1.XY() - pinterm.XY());
                        }
                        if (!qualified1.IsUnqualified)
                        {
                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                        }
                        else if (Math.Abs(distcc1 + radius[i] - r1) < tol)
                        {
                            _qualifier1[_nbrSol] = Qualifier.Enclosed;
                        }
                        else if (Math.Abs(distcc1 - r1 - radius[i]) < tol)
                        {
                            _qualifier1[_nbrSol] = Qualifier.Outside;
                        }
                        else
                        {
                            _qualifier1[_nbrSol] = Qualifier.Enclosing;
                        }
                        _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                        if (!sameCenter)
                        {
                            _pnttg1sol[_nbrSol] = new Point2D(pinterm.XY() + radius[i] * dc1.XY());
                        }
                        else
                        {
                            _pnttg1sol[_nbrSol] = new Point2D(pinterm.XY());
                        }
                        _pnttg2sol[_nbrSol] = point2;
                        _pntcen[_nbrSol] = _cirsol[_nbrSol].Location;
                        _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                        _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                        _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                        _pararg2[_nbrSol] = 0.0;
                        _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[_nbrSol]);
                    }
                }
                if (_wellDone)
                {
                    return;
                }
            }
        }

        //=========================================================================
        //   General case.                                                         +
        //=========================================================================

        CircPnt2dBisec bis = new(c1, point2);
        if (bis.IsDone)
        {
            int nbsolution = bis.SolutionCount;
            for (int i = 1; i <= nbsolution; i++)
            {
                ConstraintBisec sol = bis.ThisSolution(i);
                ConstraintCurveType type = sol.ArcType();
                AnalyticIntersection intp = new();
                if (type == ConstraintCurveType.Lin)
                {
                    intp.Perform(sol.Line(), onCirc);
                }
                else if (type == ConstraintCurveType.Cir)
                {
                    intp.Perform(onCirc, sol.Circle());
                }
                else if (type == ConstraintCurveType.Hpr)
                {
                    intp.Perform(onCirc, new Conic(sol.Hyperbola()));
                }
                else if (type == ConstraintCurveType.Ell)
                {
                    intp.Perform(onCirc, new Conic(sol.Ellipse()));
                }
                if (intp.IsDone)
                {
                    if ((!intp.IsEmpty) && (!intp.ParallelElements()) && (!intp.IdenticalElements()))
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double dist1 = center1.DistanceTo(center);
                            int nbsol = 1;
                            bool ok = false;
                            // OCCT: the radius follows Qualified1 only, whichever branch of the bisector the centre is
                            // on, so with C1 qualified (and for both radii when it is not) a solution need not pass
                            // through Point2
                            if (qualified1.IsEnclosed)
                            {
                                if (dist1 - c1.Radius <= tol)
                                {
                                    ok = true;
                                    radius[1] = Math.Abs(c1.Radius - dist1);
                                }
                            }
                            else if (qualified1.IsOutside)
                            {
                                if (c1.Radius - dist1 <= tol)
                                {
                                    ok = true;
                                    radius[1] = Math.Abs(c1.Radius - dist1);
                                }
                            }
                            else if (qualified1.IsEnclosing)
                            {
                                ok = true;
                                radius[1] = c1.Radius + dist1;
                            }
                            else if (qualified1.IsUnqualified)
                            {
                                ok = true;
                                nbsol = 2;
                                radius[1] = Math.Abs(c1.Radius - dist1);
                                radius[2] = c1.Radius + dist1;
                            }
                            if (ok)
                            {
                                for (int k = 1; k <= nbsol; k++)
                                {
                                    // pop : protection against case center1 == Center
                                    if (center1.IsEqual(center, global::Kerf.Engine.Foundation.Precision.Confusion()))
                                    {
                                        continue;
                                    }
                                    if (onCirc.DistanceTo(center) > tol)
                                    {
                                        continue;
                                    }
                                    if (_nbrSol == 4)
                                    {
                                        break;
                                    }
                                    _nbrSol++;
                                    _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius[k]);
                                    //              ==========================================================
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
                                    _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                                    if (distcc1 <= tol && Math.Abs(radius[k] - c1.Radius) <= tol)
                                    {
                                        _theSame1[_nbrSol] = 1;
                                    }
                                    else
                                    {
                                        _theSame1[_nbrSol] = 0;
                                        Direction2D dc1 = new(center1.XY() - center.XY());
                                        _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[k] * dc1.XY());
                                        _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                        _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                    }
                                    _theSame2[_nbrSol] = 0;
                                    _pntcen[_nbrSol] = center;
                                    _pnttg2sol[_nbrSol] = point2;
                                    _pararg2[_nbrSol] = 0.0;
                                    _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                    _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[_nbrSol]);
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
