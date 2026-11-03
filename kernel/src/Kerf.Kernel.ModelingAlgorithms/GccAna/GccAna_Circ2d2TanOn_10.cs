// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_10.cxx
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
    //   Creation of a circle tangent to straight line L1 and a point Point2.   +
    //                        centered on a circle.                           +
    //  We start by making difference between boundary cases that will be    +
    //  processed separately.                                            +
    //  In general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to L1 and Point2 that give us all        +
    //  all possible locations of centers of all circles        +
    //  tangent to L1 and Point2.                                              +
    //  We intersect these bissectrices with circle OnCirc which gives us +
    //  points among which we'll choose the solutions.   +
    //  The choice is made using Qualifiers of L1.        +
    //=========================================================================

    /// <summary>The circles tangent to a line and through a point with their centre on a circle.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedLin qualified1,
                               in Point2D point2,
                               in Circle2D onCirc,
                               double tolerance)
        : this(4)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double tol = Math.Abs(tolerance);
        Direction2D dirx = new(Direction2D.D.X);
        Line2D l1 = qualified1.Qualified();
        Point2D originL1 = l1.Location();
        Direction2D dirL1 = l1.Direction();
        Direction2D normL1 = new(-dirL1.Y(), dirL1.X());

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        double ron = onCirc.Radius();
        double distpc = onCirc.Location().Distance(point2);
        Direction2D dir = new(onCirc.Location().XY() - point2.XY());
        Point2D pinterm = new(point2.XY() + (distpc + ron) * dir.XY());
        double dist1 = l1.Distance(pinterm);

        // OCCT: pinterm is distpc + Ron from Point2 but its distance to L1 is compared with distpc - Ron, so a circle
        // found here doesn't pass through Point2 unless OnCirc has radius 0
        if (Math.Abs(dist1 - distpc + ron) <= tol)
        {
            dir = new Direction2D(-dirL1.Y(), dirL1.X());
            Direction2D direc = new(originL1.XY() - pinterm.XY());
            if (qualified1.IsOutside)
            {
                if (direc.Dot(dir) >= 0.0)
                {
                    _wellDone = true;
                }
            }
            else if (qualified1.IsEnclosed)
            {
                if (direc.Dot(dir) <= 0.0)
                {
                    _wellDone = true;
                }
            }
            else
            {
                _wellDone = true;
            }
            if (_wellDone)
            {
                _nbrSol++;
                _cirsol[_nbrSol] = new Circle2D(new Axis2D(pinterm, dirx), dist1);
                //    =======================================================
                Direction2D dc1 = new(originL1.XY() - pinterm.XY());
                double sign = dc1.Dot(normL1);
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
                dc1 = new Direction2D(sign * new Xy(-dirL1.Y(), dirL1.X()));
                _pnttg1sol[_nbrSol] = new Point2D(pinterm.XY() + dist1 * dc1.XY());
                _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                _pntcen[_nbrSol] = pinterm;
                _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[_nbrSol]);
                _parcen3[_nbrSol] = 0.0; // OCCT: overwrites the centre's parameter just computed
                _pnttg2sol[_nbrSol] = point2;
                _pararg2[_nbrSol] = 0.0;
                _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                return;
            }
        }

        //=========================================================================
        //   General case.                                                         +
        //=========================================================================

        LinPnt2dBisec bis = new(l1, point2);
        if (bis.IsDone)
        {
            ConstraintBisec sol = bis.ThisSolution();
            ConstraintCurveType type = sol.ArcType();
            AnalyticIntersection intp = new();
            if (type == ConstraintCurveType.Lin)
            {
                intp.Perform(sol.Line(), onCirc);
            }
            if (type == ConstraintCurveType.Par)
            {
                intp.Perform(onCirc, new Conic(sol.Parabola()));
            }
            if (intp.IsDone)
            {
                if (!intp.IsEmpty)
                {
                    for (int j = 1; j <= intp.PointCount; j++)
                    {
                        Point2D center = intp.Point(j).Value();
                        double radius = l1.Distance(center);
                        //	  int nbsol = 1;
                        bool ok = false;
                        if (qualified1.IsEnclosed)
                        {
                            if ((((originL1.X() - center.X()) * (-dirL1.Y()))
                                 + ((originL1.Y() - center.Y()) * (dirL1.X())))
                                <= 0)
                            {
                                ok = true;
                            }
                        }
                        else if (qualified1.IsOutside)
                        {
                            if ((((originL1.X() - center.X()) * (-dirL1.Y()))
                                 + ((originL1.Y() - center.Y()) * (dirL1.X())))
                                >= 0)
                            {
                                ok = true;
                            }
                        }
                        else if (qualified1.IsUnqualified)
                        {
                            ok = true;
                        }
                        if (ok)
                        {
                            _nbrSol++;
                            _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                            //          =======================================================
                            _theSame1[_nbrSol] = 0;
                            _theSame2[_nbrSol] = 0;
                            Direction2D dc1 = new(originL1.XY() - center.XY());
                            double sign = dc1.Dot(normL1);
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
                            dc1 = new Direction2D(sign * new Xy(-dirL1.Y(), dirL1.X()));
                            // OCCT: dist1 (the boundary case's distance) instead of Radius, so the tangency point is
                            // off the line unless the two happen to be equal
                            _pnttg1sol[_nbrSol] = new Point2D(center.XY() + dist1 * dc1.XY());
                            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                            _pntcen[_nbrSol] = center;
                            _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[_nbrSol]);
                            _pnttg2sol[_nbrSol] = point2;
                            _pararg2[_nbrSol] = 0.0;
                            _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                            _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                        }
                    }
                }
                _wellDone = true;
            }
        }
    }
}
