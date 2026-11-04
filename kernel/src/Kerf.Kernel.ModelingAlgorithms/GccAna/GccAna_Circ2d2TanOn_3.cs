// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanOn
{
    //=========================================================================
    //  Circles tangent to circle C1, passing by point Point2 and centers     +
    //  on a straight line OnLine.                                            +
    //  We start by making difference with boundary cases that will be        +
    //  processed separately.                                                 +
    //  For the general case:                                                 +
    //  ====================                                                  +
    //  We calculate bissectrices to C1 and Point2 that give us all           +
    //  possible locations of centers of all circles                          +
    //  tangent to C1 and passing by Point2.                                  +
    //  We intersect these bissectrices with the straight line OnLine which   +
    //  gives us the points among which we'll choose the solutions.           +
    //  The choices are made using Qualifiers of C1 and C2.                   +
    //=========================================================================

    /// <summary>The circles tangent to a circle and through a point with their centre on a line.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in Point2D point2,
                               in Line2D onLine,
                               double tolerance)
        : this(4)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        NCollection_Array1<double> radius = new(1, 2);
        Direction2D dirx = new(Direction2D.D.X);
        Circle2D c1 = qualified1.Qualified();
        double r1 = c1.Radius;
        Point2D center1 = c1.Location;

        //=========================================================================
        //   Processing of boundary cases.                                        +
        //=========================================================================

        double dp2l = onLine.DistanceTo(point2);
        Direction2D donline = onLine.Direction;
        Point2D pinterm = new(point2.XY() + dp2l * new Xy(-donline.Y, donline.X));
        if (onLine.DistanceTo(pinterm) > tol)
        {
            pinterm = new Point2D(point2.XY() - dp2l * new Xy(-donline.Y, donline.X));
        }
        double dist = pinterm.DistanceTo(center1);
        if (qualified1.IsEnclosed && Math.Abs(r1 - dist - dp2l) <= tol)
        {
            _wellDone = true;
        }
        else if (qualified1.IsEnclosing && Math.Abs(r1 + dist - dp2l) <= tol)
        {
            _wellDone = true;
        }
        else if (qualified1.IsOutside && Math.Abs(dist - dp2l) <= tol)
        {
            _wellDone = true;
        }
        else if (qualified1.IsUnqualified
                 && (Math.Abs(dist - dp2l) <= tol || Math.Abs(r1 - dist - dp2l) <= tol
                     || Math.Abs(r1 + dist - dp2l) <= tol))
        {
            _wellDone = true;
        }
        if (_wellDone)
        {
            _nbrSol++;
            _cirsol[_nbrSol] = new Circle2D(new Axis2D(pinterm, dirx), dp2l);
            //  ======================================================
            Direction2D dc1 = new(center1.XY() - pinterm.XY());
            double distcc1 = pinterm.DistanceTo(center1);
            if (!qualified1.IsUnqualified)
            {
                _qualifier1[_nbrSol] = qualified1.Qualifier();
            }
            else if (Math.Abs(distcc1 + dp2l - r1) < tol)
            {
                _qualifier1[_nbrSol] = Qualifier.Enclosed;
            }
            else if (Math.Abs(distcc1 - r1 - dp2l) < tol)
            {
                _qualifier1[_nbrSol] = Qualifier.Outside;
            }
            else
            {
                _qualifier1[_nbrSol] = Qualifier.Enclosing;
            }
            _qualifier2[_nbrSol] = Qualifier.Noqualifier;
            _pnttg1sol[_nbrSol] = new Point2D(pinterm.XY() + dp2l * dc1.XY());
            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
            _pnttg2sol[_nbrSol] = point2;
            _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
            _pntcen[_nbrSol] = _cirsol[_nbrSol].Location;
            _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[_nbrSol]);
            return;
        }

        //=========================================================================
        //   General case.                                                       +
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
                    intp.Perform(onLine, sol.Line());
                }
                else if (type == ConstraintCurveType.Cir)
                {
                    intp.Perform(onLine, sol.Circle());
                }
                else if (type == ConstraintCurveType.Ell)
                {
                    intp.Perform(onLine, new Conic(sol.Ellipse()));
                }
                else if (type == ConstraintCurveType.Hpr)
                {
                    intp.Perform(onLine, new Conic(sol.Hyperbola()));
                }
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double dist1 = center1.DistanceTo(center);
                            int nbsol = 1;
                            bool ok = false;
                            // OCCT: with C1 qualified the radius follows the qualifier only, whichever branch of the
                            // bisector the centre is on, so a solution need not pass through Point2
                            if (qualified1.IsEnclosed)
                            {
                                if (dist1 - c1.Radius <= tolerance)
                                {
                                    ok = true;
                                    radius[1] = Math.Abs(c1.Radius - dist1);
                                }
                            }
                            else if (qualified1.IsOutside)
                            {
                                if (c1.Radius - dist1 <= tolerance)
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
                            /*	    else if (Qualified1.IsUnqualified() && ok) {
                                      ok = true;
                                      nbsol = 2;
                                      Radius(1) = std::abs(C1.Radius()-dist1);
                                      Radius(2) = C1.Radius()+dist1;
                                    }
                            */
                            else if (qualified1.IsUnqualified)
                            {
                                double popradius = center.DistanceTo(point2);
                                // OCCT: tests the difference as a boolean (non-zero), not against a tolerance
                                if (Math.Abs(popradius - dist1) != 0.0)
                                {
                                    ok = true;
                                    radius[1] = popradius;
                                }
                            }

                            if (ok)
                            {
                                for (int k = 1; k <= nbsol; k++)
                                {
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
                                    if (center.DistanceTo(center1) <= tolerance
                                        && Math.Abs(radius[k] - c1.Radius) <= tolerance)
                                    {
                                        _theSame1[_nbrSol] = 1;
                                    }
                                    else
                                    {
                                        _theSame1[_nbrSol] = 0;
                                        Direction2D dc1 = new(center1.XY() - center.XY());
                                        _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[k] * dc1.XY());
                                        _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                        // OCCT: indexed by the bisector's number i instead of NbrSol
                                        _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                    }
                                    _theSame2[_nbrSol] = 0;
                                    _pnttg2sol[_nbrSol] = point2;
                                    _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                    _pararg2[_nbrSol] = 0.0;
                                    _pntcen[_nbrSol] = center;
                                    _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[_nbrSol]);
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
