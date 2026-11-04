// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanOn
{
    /// <summary>The circles tangent to two lines with their centre on a line.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedLin qualified1,
                               in QualifiedLin qualified2,
                               in Line2D onLine,
                               double tolerance)
        : this(2)
    {
        // initialisations
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;

        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);

        // calculation of bisectrices of L1 and L2
        Line2D l1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        Point2D originL1 = l1.Location;
        Point2D originL2 = l2.Location;
        Lin2dBisec bis = new(l1, l2);

        if (bis.IsDone)
        {
            if (bis.SolutionCount == 1 || bis.SolutionCount == 2)
            {
                // if 1 bisectrice, L1 and L2 are parallel
                // if 2 bisectrices, L1 and L2 are intersected

                for (int k = 1; k <= bis.SolutionCount; k++)
                {
                    AnalyticIntersection intp = new(bis.ThisSolution(k), onLine);
                    if (intp.IsDone)
                    {
                        _wellDone = true;
                        // for degenerated cases, no acceptable solution
                        // (OnLine and bisectrice strictly parallel or not)
                        if (!intp.IdenticalElements() && !intp.ParallelElements() && !intp.IsEmpty)
                        {
                            // at maximum 1 point of intersection !
                            for (int l = 1; l <= intp.PointCount; l++)
                            {
                                Point2D pt = intp.Point(l).Value();
                                Axis2D axe = new(pt, dirx);
                                double radius = l1.DistanceTo(pt);
                                if (!l1.Contains(pt, tol) && radius < 1.0 / tol && _nbrSol < 2)
                                {
                                    // acceptable solution : the radius is correct
                                    _nbrSol++;
                                    _cirsol[_nbrSol] = new Circle2D(axe, radius);
                                }
                            }
                        }
                    }
                }
            }
        }

        // parce following the qualifiers NbrSol acceptable solutions

        for (int i = 1; i <= _nbrSol; i++)
        {
            Point2D pbid = _cirsol[i].Location;
            double radius = _cirsol[i].Radius;
            bool ok = false;

            // solution Outside or Enclosed / L1
            Direction2D dc1 = new(originL1.XY() - pbid.XY());
            double sign1 = dc1.Dot(new Direction2D(-l1.Direction.Y, l1.Direction.X));
            if (sign1 > 0.0)
            {
                ok = (qualified1.IsUnqualified || qualified1.IsOutside);
            }
            else
            {
                ok = (qualified1.IsUnqualified || qualified1.IsEnclosed);
            }

            // solution Outside or Enclosed / L2
            Direction2D dc2 = new(originL2.XY() - pbid.XY());
            double sign2 = dc2.Dot(new Direction2D(-l2.Direction.Y, l2.Direction.X));
            if (sign2 > 0.0)
            {
                ok = ok && (qualified2.IsUnqualified || qualified2.IsOutside);
            }
            else
            {
                ok = ok && (qualified2.IsUnqualified || qualified2.IsEnclosed);
            }

            if (ok)
            {
                // solution to be preserved
                dc1 = new Direction2D(sign1 * new Xy(-l1.Direction.Y, l1.Direction.X));
                _pnttg1sol[i] = new Point2D(pbid.XY() + radius * dc1.XY());
                if (sign1 > 0.0)
                {
                    _qualifier1[i] = Qualifier.Outside;
                }
                else
                {
                    _qualifier1[i] = Qualifier.Enclosed;
                }
                dc2 = new Direction2D(sign2 * new Xy(-l2.Direction.Y, l2.Direction.X));
                _pnttg2sol[i] = new Point2D(pbid.XY() + radius * dc2.XY());
                if (sign2 > 0.0)
                {
                    _qualifier2[i] = Qualifier.Outside;
                }
                else
                {
                    _qualifier2[i] = Qualifier.Enclosed;
                }
                _pntcen[i] = pbid;
                _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg1sol[i]);
                _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[i]);
                _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg2sol[i]);
                _pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[i]);
                _parcen3[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[i]);
            }
            else
            {
                // solution to be rejected
                if (i == _nbrSol)
                {
                    _nbrSol--;
                }
                else
                {
                    for (int k = i + 1; k <= _nbrSol; k++)
                    {
                        _cirsol[k - 1] = _cirsol[k];
                    }
                    _nbrSol--;
                    i--;
                }
            }
        }
    }
}
