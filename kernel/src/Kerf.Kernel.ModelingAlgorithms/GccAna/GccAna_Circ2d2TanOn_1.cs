// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2d2TanOn
{
    //=========================================================================
    //   Creation of a circle tangent to Circle C1 and a straight line L2.    +
    //                        centered on a straight line.                    +
    //  We start by making difference between cases that we are going to      +
    //  process separately.                                                   +
    //  In general case:                                                      +
    //  ====================                                                  +
    //  We calculate bissectrices to C1 and L2 that give us                   +
    //  all possible locations of centers of all circles tangent to C1 and L2 +
    //  We intersect these bissectrices with straight line OnLine which gives +
    //  us points among which we'll choose the solutions.                     +
    //  The choices are made basing on Qualifiers of C1 and L2.               +
    //=========================================================================

    /// <summary>The circles tangent to a circle and a line with their centre on a line.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in QualifiedLin qualified2,
                               in Line2D onLine,
                               double tolerance)
        : this(4)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double tol = Math.Abs(tolerance);
        double radius = 0;
        bool ok = false;
        Direction2D dirx = new(Direction2D.D.X);
        Circle2D c1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        double r1 = c1.Radius();
        Point2D center1 = c1.Location();
        Point2D origin2 = l2.Location();
        Direction2D dirL2 = l2.Direction();
        Direction2D normL2 = new(-dirL2.Y(), dirL2.X());

        //=========================================================================
        //   Processing of limit cases.                                          +
        //=========================================================================

        double distcl = onLine.Distance(center1);
        Point2D pinterm = new(center1.XY() + distcl * new Xy(-onLine.Direction().Y(), onLine.Direction().X()));
        if (onLine.Distance(pinterm) > tolerance)
        {
            // OCCT: the same point again (the other side, center1 - distcl * normal, was meant)
            pinterm = new Point2D(center1.XY() + distcl * new Xy(-onLine.Direction().Y(), onLine.Direction().X()));
        }
        double dist2 = l2.Distance(pinterm);
        if (qualified1.IsEnclosed || qualified1.IsOutside)
        {
            if (Math.Abs(distcl - r1 - dist2) <= tol)
            {
                ok = true;
            }
        }
        else if (qualified1.IsEnclosing)
        {
            if (Math.Abs(dist2 - distcl - r1) <= tol)
            {
                ok = true;
            }
        }
        else if (qualified1.IsUnqualified)
        {
            ok = true;
        }
        else
        {
            throw new BadQualifierException();
        }
        if (ok)
        {
            if (qualified2.IsOutside)
            {
                Point2D pbid = new(pinterm.XY() + dist2 * new Xy(-dirL2.Y(), dirL2.X()));
                if (l2.Distance(pbid) <= tol)
                {
                    _wellDone = true;
                }
            }
            else if (qualified2.IsEnclosed)
            {
                Point2D pbid = new(pinterm.XY() - dist2 * new Xy(-dirL2.Y(), dirL2.X()));
                if (l2.Distance(pbid) <= tol)
                {
                    _wellDone = true;
                }
            }
            else if (qualified2.IsUnqualified)
            {
                _wellDone = false;
            }
            else
            {
                throw new BadQualifierException();
            }
        }
        if (_wellDone)
        {
            _nbrSol++;
            _cirsol[_nbrSol] = new Circle2D(new Axis2D(pinterm, dirx), dist2);
            //   =======================================================
            Direction2D dc1 = new(center1.XY() - pinterm.XY());
            Direction2D dc2 = new(origin2.XY() - pinterm.XY());
            double distcc1 = pinterm.Distance(center1);
            if (!qualified1.IsUnqualified)
            {
                _qualifier1[_nbrSol] = qualified1.Qualifier();
            }
            else if (Math.Abs(distcc1 + dist2 - r1) < tol)
            {
                _qualifier1[_nbrSol] = Qualifier.Enclosed;
            }
            else if (Math.Abs(distcc1 - r1 - dist2) < tol)
            {
                _qualifier1[_nbrSol] = Qualifier.Outside;
            }
            else
            {
                _qualifier1[_nbrSol] = Qualifier.Enclosing;
            }
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

            double sign = dc2.Dot(new Direction2D(-dirL2.Y(), dirL2.X()));
            dc2 = new Direction2D(sign * new Xy(-dirL2.Y(), dirL2.X()));
            _pnttg1sol[_nbrSol] = new Point2D(pinterm.XY() + dist2 * dc1.XY());
            _pnttg2sol[_nbrSol] = new Point2D(pinterm.XY() + dist2 * dc2.XY());
            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
            _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
            _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
            _pntcen[_nbrSol] = _cirsol[_nbrSol].Location();
            _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[_nbrSol]);
            return;
        }

        //=========================================================================
        //   General case.                                                        +
        //=========================================================================

        CircLin2dBisec bis = new(c1, l2);
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
                else if (type == ConstraintCurveType.Par)
                {
                    intp.Perform(onLine, new Conic(sol.Parabola()));
                }
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double dist1 = center.Distance(center1);
                            dist2 = l2.Distance(center);
                            //	     int nbsol = 1;
                            ok = false;
                            if (qualified1.IsEnclosed)
                            {
                                if (dist1 - r1 < tolerance)
                                {
                                    if (Math.Abs(Math.Abs(r1 - dist1) - dist2) < tolerance)
                                    {
                                        ok = true;
                                    }
                                }
                            }
                            else if (qualified1.IsOutside)
                            {
                                if (r1 - dist1 < tolerance)
                                {
                                    if (Math.Abs(Math.Abs(r1 - dist1) - dist2) < tolerance)
                                    {
                                        ok = true;
                                    }
                                }
                            }
                            else if (qualified1.IsEnclosing || qualified1.IsUnqualified)
                            {
                                ok = true;
                            }
                            if (qualified2.IsEnclosed && ok)
                            {
                                if ((((origin2.X() - center.X()) * (-dirL2.Y()))
                                     + ((origin2.Y() - center.Y()) * (dirL2.X())))
                                    <= 0)
                                {
                                    ok = true;
                                    radius = dist2;
                                }
                            }
                            else if (qualified2.IsOutside && ok)
                            {
                                if ((((origin2.X() - center.X()) * (-dirL2.Y()))
                                     + ((origin2.Y() - center.Y()) * (dirL2.X())))
                                    >= 0)
                                {
                                    ok = true;
                                    radius = dist2;
                                }
                            }
                            else if (qualified2.IsUnqualified && ok)
                            {
                                ok = true;
                                radius = dist2;
                            }
                            if (ok)
                            {
                                _nbrSol++;
                                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                //             =======================================================
                                Direction2D dc1 = new(center1.XY() - center.XY());
                                Direction2D dc2 = new(origin2.XY() - center.XY());
                                double distcc1 = center.Distance(center1);
                                if (!qualified1.IsUnqualified)
                                {
                                    _qualifier1[_nbrSol] = qualified1.Qualifier();
                                }
                                else if (Math.Abs(distcc1 + radius - r1) < tol)
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                }
                                else if (Math.Abs(distcc1 - r1 - radius) < tol)
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Enclosing;
                                }
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
                                if (center.Distance(center1) <= tolerance
                                    && Math.Abs(radius - c1.Radius()) <= tolerance)
                                {
                                    _theSame1[_nbrSol] = 1;
                                }
                                else
                                {
                                    _theSame1[_nbrSol] = 0;
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc1.XY());
                                    _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                    _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                }
                                _theSame2[_nbrSol] = 0;
                                double sign = dc2.Dot(new Direction2D(-dirL2.Y(), dirL2.X()));
                                dc2 = new Direction2D(sign * new Xy(-dirL2.Y(), dirL2.X()));
                                _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius * dc2.XY());
                                _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
                                _pntcen[_nbrSol] = center;
                                _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[_nbrSol]);
                            }
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }
}
