// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_5.cxx
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
    //   Creation of a circle passing by : 2 points Point1 and Point2.        +
    //                        Centered on  : 1 straight line OnLine.                  +
    //   with a Tolerance of precision  : Tolerance.                        +
    //                                                                        +
    //   Create L1 straight line of points equidistant from Point1 and Point2.     +
    //   Then create solutions cirsol such as :                        +
    //   cirsol is all circle with center at the intersections of  +
    //   L1 with OnLine and the radius the distance between Point1 and the point   +
    //   calculated above.                                                   +
    //=========================================================================

    /// <summary>The circles through two points with their centre on a line.</summary>
    public AnalyticCirc2d2TanOn(in Point2D point1,
                               in Point2D point2,
                               in Line2D onLine,
                               double tolerance)
        : this(2)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;

        Direction2D dirx = new(Direction2D.D.X);
        double dist = point1.Distance(point2);
        if (dist < Math.Abs(tolerance))
        {
            _wellDone = true;
        }
        else
        {
            Line2D l1 = new(new Point2D((point1.XY() + point2.XY()) / 2.0),
                              new Direction2D(point1.Y() - point2.Y(), point2.X() - point1.X()));
            AnalyticIntersection intp = new(l1, onLine);
            if (intp.IsDone)
            {
                if (!intp.IsEmpty)
                {
                    for (int i = 1; i <= intp.PointCount; i++)
                    {
                        _nbrSol++;
                        Axis2D axe = new(intp.Point(i).Value(), dirx);
                        _cirsol[_nbrSol] = new Circle2D(axe, point1.Distance(intp.Point(i).Value()));
                        //        ====================================================================
                        _qualifier1[_nbrSol] = Qualifier.Noqualifier;
                        _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                        _pnttg1sol[_nbrSol] = point1;
                        _pnttg2sol[_nbrSol] = point2;
                        _pntcen[_nbrSol] = _cirsol[_nbrSol].Location();
                        _pararg1[_nbrSol] = 0.0;
                        _pararg2[_nbrSol] = 0.0;
                        _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                        _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                        _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[_nbrSol]);
                    }
                }
                _wellDone = true;
            }
        }
    }
}
