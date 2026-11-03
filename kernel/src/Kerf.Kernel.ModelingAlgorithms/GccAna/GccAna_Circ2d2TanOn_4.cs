// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_4.cxx
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
    //   Creation of a circle Tangent to : 1 straight line L1.                +
    //                        Passing by : 1 point Point2.                    +
    //                        Centered on  : 1 straight line OnLine.                  +
    //   with a Tolerance of precision  : Tolerance.                        +
    //                                                                        +
    //  We start by making difference with various boundary cases that will be +
    //  processed separately.                                            +
    //  For the general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to L1 and Point2 that give us       +
    //  all possible locations of centers of all circles        +
    //  tangent to L1 and passing through Point2.                                  +
    //  We intersect these bissectrices with straight line OnLine which gives us +
    //  the points among which we'll choose the solutions.   +
    //  The choices are made basing on Qualifieurs of L1.        +
    //=========================================================================

    /// <summary>The circles tangent to a line and through a point with their centre on a line.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedLin qualified1,
                               in Point2D point2,
                               in Line2D onLine,
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
        Point2D originL1 = l1.Location;
        Direction2D dirL1 = l1.Direction;
        Direction2D normal = new(-dirL1.Y, dirL1.X);

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        if (dirL1.IsEqual(onLine.Direction, global::Kerf.Engine.Foundation.Precision.Confusion())
            && onLine.DistanceTo(originL1) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // POP : l2s 2 straight line are identic : no Sol
            _nbrSol = 0;
            return;
        }

        double dp2l = onLine.DistanceTo(point2);
        Direction2D donline = onLine.Direction;
        Point2D pinterm = new(point2.XY() + dp2l * new Xy(-donline.Y, donline.X));
        if (onLine.DistanceTo(pinterm) > tol)
        {
            pinterm = new Point2D(point2.XY() - dp2l * new Xy(-donline.Y, donline.X));
        }
        double dist = l1.DistanceTo(pinterm);
        if (Math.Abs(dist - dp2l) <= tol)
        {
            Direction2D dirbid = new(originL1.XY() - pinterm.XY());
            if (qualified1.IsEnclosed && dirbid.Dot(normal) < 0.0)
            {
                _wellDone = true;
            }
            else if (qualified1.IsOutside && dirbid.Dot(normal) > 0.0)
            {
                _wellDone = true;
            }
            else if (qualified1.IsUnqualified)
            {
                _wellDone = true;
            }
            if (_wellDone)
            {
                _nbrSol++;
                _cirsol[_nbrSol] = new Circle2D(new Axis2D(pinterm, dirx), dp2l);
                //    ======================================================
                _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                Direction2D dc2 = new(originL1.XY() - pinterm.XY());
                if (!qualified1.IsUnqualified)
                {
                    _qualifier1[_nbrSol] = qualified1.Qualifier();
                }
                else if (dc2.Dot(normal) > 0.0)
                {
                    _qualifier1[_nbrSol] = Qualifier.Outside;
                }
                else
                {
                    _qualifier1[_nbrSol] = Qualifier.Enclosed;
                }
                double sign = dc2.Dot(new Direction2D(-dirL1.Y, dirL1.X));
                dc2 = new Direction2D(sign * new Xy(-dirL1.Y, dirL1.X));
                _pnttg1sol[_nbrSol] = new Point2D(pinterm.XY() + dp2l * dc2.XY());
                _pnttg2sol[_nbrSol] = point2;
                _pntcen[_nbrSol] = pinterm;
                _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                _pararg2[_nbrSol] = 0.0;
                _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[_nbrSol]);
                return;
            }
        }

        //=========================================================================
        //   General case.                                                        +
        //=========================================================================

        LinPnt2dBisec bis = new(l1, point2);
        if (bis.IsDone)
        {
            ConstraintBisec sol = bis.ThisSolution();
            ConstraintCurveType type = sol.ArcType();
            AnalyticIntersection intp = new();
            if (type == ConstraintCurveType.Lin)
            {
                intp.Perform(onLine, sol.Line());
            }
            if (type == ConstraintCurveType.Par)
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
                        double radius = l1.DistanceTo(center);
                        //	  int nbsol = 1;
                        bool ok = false;
                        if (qualified1.IsEnclosed)
                        {
                            if ((((originL1.X - center.X) * (-dirL1.Y))
                                 + ((originL1.Y - center.Y) * (dirL1.X)))
                                <= 0)
                            {
                                ok = true;
                            }
                        }
                        else if (qualified1.IsOutside)
                        {
                            if ((((originL1.X - center.X) * (-dirL1.Y))
                                 + ((originL1.Y - center.Y) * (dirL1.X)))
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
                            _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                            Direction2D dc2 = new(originL1.XY() - center.XY());
                            if (!qualified1.IsUnqualified)
                            {
                                _qualifier1[_nbrSol] = qualified1.Qualifier();
                            }
                            else if (dc2.Dot(normal) > 0.0)
                            {
                                _qualifier1[_nbrSol] = Qualifier.Outside;
                            }
                            else
                            {
                                _qualifier1[_nbrSol] = Qualifier.Enclosed;
                            }
                            _theSame1[_nbrSol] = 0;
                            _theSame2[_nbrSol] = 0;
                            Direction2D dc1 = new(originL1.XY() - center.XY());
                            double sign = dc1.Dot(normal);
                            dc1 = new Direction2D(sign * (normal.XY()));
                            _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc1.XY());
                            _pnttg2sol[_nbrSol] = point2;
                            _pntcen[_nbrSol] = center;
                            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                            _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                            _pararg2[_nbrSol] = 0.0;
                            _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[_nbrSol]);
                        }
                    }
                }
                _wellDone = true;
            }
        }
    }
}
