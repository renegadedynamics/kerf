// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_11.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2d2TanOn
{
    //=========================================================================
    //   Creation of a circle passing by  : 2 points Point1 and Point2.        +
    //                        Center on   : 1 circle OnCirc.                  +
    //   with a Tolerance of precision    : Tolerance.                        +
    //                                                                        +
    //   Create L1 the straight line of points equidistant to Point1 and Point2.     +
    //   Create then solutions cirsol so that :                      +
    //   cirsol is the set of circles with center in one of inter-    +
    //   sections of L1 with OnCirc and the radius the distance between Point1 and   +
    //   the point calculated below.                                          +
    //=========================================================================

    /// <summary>The circles through two points with their centre on a circle.</summary>
    public AnalyticCirc2d2TanOn(in Point2D point1,
                               in Point2D point2,
                               in Circle2D onCirc,
                               double tolerance)
        : this(2)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        double dist = point1.DistanceTo(point2);
        double dp1cen = point1.DistanceTo(onCirc.Location);
        double dp2cen = point2.DistanceTo(onCirc.Location);
        double r = onCirc.Radius;
        Circle2D c1 = onCirc;
        if (dist < tol || Math.Abs(dp1cen + 2 * r - dp2cen) < tol
            || Math.Abs(dp2cen + 2 * r - dp1cen) < tol)
        {
            _wellDone = true;
            return;
        }
        Line2D l1 = new(new Point2D((point1.XY() + point2.XY()) / 2.0),
                          new Direction2D(point1.Y - point2.Y, point2.X - point1.X));
        // OCCT: never true, since the same conditions returned above (and the inner branches test the same one)
        if (Math.Abs(dp1cen + 2 * r - dp2cen) < tol || Math.Abs(dp2cen + 2 * r - dp1cen) < tol)
        {
            if (Math.Abs(dp1cen + 2 * r - dp2cen) < tol)
            {
                c1 = new Circle2D(new Axis2D(onCirc.Location, dirx),
                                   onCirc.Radius + Math.Abs(dp2cen - dp1cen - 2.0 * r));
            }
            else if (Math.Abs(dp1cen + 2 * r - dp2cen) < tol)
            {
                c1 = new Circle2D(new Axis2D(onCirc.Location, dirx),
                                   onCirc.Radius + Math.Abs(dp2cen - dp1cen - 2.0 * r));
            }
        }
        AnalyticIntersection intp = new(l1, c1);
        if (intp.IsDone)
        {
            if (!intp.IsEmpty)
            {
                for (int i = 1; i <= intp.PointCount; i++)
                {
                    _nbrSol++;
                    Axis2D axe = new(intp.Point(i).Value(), dirx);
                    _cirsol[_nbrSol] = new Circle2D(axe, point1.DistanceTo(intp.Point(i).Value()));
                    //      ======================================================================
                    _qualifier1[_nbrSol] = Qualifier.Noqualifier;
                    _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                    _pnttg1sol[_nbrSol] = point1;
                    _pararg1[_nbrSol] = 0.0;
                    _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                    _pnttg2sol[_nbrSol] = point2;
                    _pararg2[_nbrSol] = 0.0;
                    _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                    _pntcen[_nbrSol] = _cirsol[_nbrSol].Location;
                    _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[_nbrSol]);
                }
            }
            _wellDone = true;
        }
    }
}
