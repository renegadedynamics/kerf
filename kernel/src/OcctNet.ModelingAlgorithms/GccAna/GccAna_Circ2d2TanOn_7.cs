// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_7.cxx
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
    /// <summary>The circles tangent to a circle and a line with their centre on a circle.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in QualifiedLin qualified2,
                               in Circle2D onCirc,
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
        double radius = 0;
        Direction2D dirx = new(Direction2D.D.X);
        Circle2D c1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        double r1 = c1.Radius();
        Point2D center1 = c1.Location();
        Point2D origin2 = l2.Location();
        Direction2D dirL2 = l2.Direction();
        Direction2D normL2 = new(-dirL2.Y(), dirL2.X());

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        double tol = Math.Abs(tolerance);
        NCollection_Array1<double> rradius = new(1, 2);
        int nbsol1 = 1;
        //  int nbsol2 = 0;
        double ron = onCirc.Radius();
        double distcco = onCirc.Location().Distance(center1);
        Direction2D dircc = new(onCirc.Location().XY() - center1.XY());
        Point2D pinterm = new(center1.XY() + (distcco - ron) * dircc.XY());
        double distpl2 = l2.Distance(pinterm);
        double distcc1 = pinterm.Distance(center1);
        double d1 = Math.Abs(distpl2 - Math.Abs(distcc1 - r1));
        double d2 = Math.Abs(distpl2 - (distcc1 + r1));
        if (d1 > tol || d2 > tol)
        {
            // OCCT: the same point again (distcco + Ron was meant), and d1, d2 are not recomputed
            pinterm = new Point2D(center1.XY() + (distcco - ron) * dircc.XY());
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
                rradius[1] = Math.Abs(distcc1 - r1);
            }
            else if (qualified1.IsEnclosing)
            {
                nbsol1 = 1;
                rradius[1] = r1 + distcc1;
            }
            else if (qualified1.IsUnqualified)
            {
                nbsol1 = 2;
                rradius[1] = Math.Abs(distcc1 - r1);
                rradius[2] = r1 + distcc1;
            }
            Direction2D dirbid = new(origin2.XY() - pinterm.XY());
            Direction2D normal = new(-dirL2.Y(), dirL2.X());
            // OCCT: tests Qualified1 (the circle's qualifier) against the line's side, with the same sign twice
            if (qualified1.IsEnclosed && dirbid.Dot(normal) < 0.0)
            {
                nbsol1 = 0;
            }
            else if (qualified1.IsOutside && dirbid.Dot(normal) < 0.0)
            {
                nbsol1 = 0;
            }
            for (int i = 1; i <= nbsol1; i++)
            {
                if (Math.Abs(rradius[i] - distpl2) <= tol)
                {
                    _wellDone = true;
                    _nbrSol++;
                    _cirsol[_nbrSol] = new Circle2D(new Axis2D(pinterm, dirx), rradius[i]);
                    //      ===========================================================
                    Direction2D dc1 = new(center1.XY() - pinterm.XY());
                    Direction2D dc2 = new(origin2.XY() - pinterm.XY());
                    distcc1 = pinterm.Distance(center1);
                    if (!qualified1.IsUnqualified)
                    {
                        _qualifier1[_nbrSol] = qualified1.Qualifier();
                    }
                    else if (Math.Abs(distcc1 + rradius[i] - r1) < tol)
                    {
                        _qualifier1[_nbrSol] = Qualifier.Enclosed;
                    }
                    else if (Math.Abs(distcc1 - r1 - rradius[i]) < tol)
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
                    _pnttg1sol[_nbrSol] = new Point2D(pinterm.XY() + rradius[i] * dc1.XY());
                    _pnttg2sol[_nbrSol] = new Point2D(pinterm.XY() + rradius[i] * dc2.XY());
                    _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                    _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                    _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                    _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
                    // OCCT: pntcen(NbrSol) is never set here, so the parameter is that of the array's initial
                    // point (the origin)
                    _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[_nbrSol]);
                }
            }
            if (_wellDone)
            {
                return;
            }
        }

        //=========================================================================
        //   General case.                                                         +
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
                    intp.Perform(sol.Line(), onCirc);
                }
                else if (type == ConstraintCurveType.Par)
                {
                    intp.Perform(onCirc, new Conic(sol.Parabola()));
                }
                if (intp.IsDone)
                {
                    if ((!intp.IsEmpty) && (!intp.ParallelElements()) && (!intp.IdenticalElements()))
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double dist1 = center.Distance(center1);
                            double dist2 = l2.Distance(center);
                            //	    int nbsol = 1;
                            bool ok = false;
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
                                //            =======================================================
                                Direction2D dc2 = new(origin2.XY() - center.XY());
                                distcc1 = center.Distance(center1);
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
                                    Direction2D dc1 = new(center1.XY() - center.XY());
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc1.XY());
                                    _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                    _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                }
                                _theSame2[_nbrSol] = 0;
                                double sign = dc2.Dot(new Direction2D(normL2.XY()));
                                dc2 = new Direction2D(sign * normL2.XY());
                                _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius * dc2.XY());
                                _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
                                _pntcen[_nbrSol] = center;
                                _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[_nbrSol]);
                            }
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }
}
