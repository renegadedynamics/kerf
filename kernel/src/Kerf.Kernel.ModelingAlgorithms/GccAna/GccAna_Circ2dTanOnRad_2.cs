// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2dTanOnRad
{
    //=========================================================================
    //    typedef of handled objects :                                      +
    //=========================================================================

    //=========================================================================
    //   Circle tangent to a point      Point1.                               +
    //          center on straight line OnLine.                               +
    //          radius                  Radius.                               +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Eliminate cases not being the solution.                     +
    //  Solve the equation of second degree showing that the found center point +
    //  (xc,yc) is at distance Radius from point Point1 and on the straight line OnLine. +
    //  The solutions are represented by circles :                     +
    //                   - of center Pntcen(xc,yc)                            +
    //                   - of radius Radius.                                   +
    //=========================================================================

    /// <summary>Circles of radius Radius passing through a point, centred on a line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in Point2D point1, in Line2D onLine, double radius, double tolerance)
    {
        _cirsol = new(1, 2);
        _qualifier1 = new(1, 2);
        _theSame1 = new(1, 2);
        _pnttg1sol = new(1, 2);
        _pntcen3 = new(1, 2);
        _par1sol = new(1, 2);
        _pararg1 = new(1, 2);
        _parcen3 = new(1, 2);


        Direction2D dirx = new(Direction2D.D.X);
        double   tol  = Math.Abs(tolerance);
        _wellDone      = false;
        _nbrSol        = 0;
        double dp1lin = onLine.DistanceTo(point1);

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            // OCCT: WellDone stays false when the point is on OnLine (dp1lin < Tol), which still finds 2 solutions
            if (dp1lin > radius + tol)
            {
                _wellDone = true;
            }
            double xc;
            double yc;
            double x1    = point1.X;
            double y1    = point1.Y;
            double xbid  = 0;
            double xdir  = (onLine.Direction).X;
            double ydir  = (onLine.Direction).Y;
            double lxloc = (onLine.Location).X;
            double lyloc = (onLine.Location).Y;
            if (Math.Abs(dp1lin - radius) < tol)
            {
                _wellDone = true;
                _nbrSol   = 1;
                if (-ydir * (x1 - lxloc) + xdir * (y1 - lyloc) < 0.0)
                {
                    Axis2D axe = new(new Point2D(x1 - ydir * dp1lin, y1 + xdir * dp1lin), dirx);
                    _cirsol[_nbrSol] = new Circle2D(axe, radius);
                    //       ======================================
                    _qualifier1[_nbrSol] = Qualifier.Noqualifier;
                }
                else
                {
                    Axis2D axe = new(new Point2D(x1 + ydir * dp1lin, y1 - xdir * dp1lin), dirx);
                    _cirsol[_nbrSol] = new Circle2D(axe, radius);
                    //       ======================================
                    _qualifier1[_nbrSol] = Qualifier.Noqualifier;
                }
                _theSame1[_nbrSol]  = 0;
                _pnttg1sol[_nbrSol] = point1;
                _pntcen3[_nbrSol]   = _cirsol[_nbrSol].Location;
                _pararg1[_nbrSol]   = 0.0;
                _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                _parcen3[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen3[_nbrSol]);
            }
            else if (dp1lin < tol)
            {
                // OCCT: qualifier1 is not set on this path (uninitialised in OCCT, GccEnt_unqualified here)
                _pntcen3[1] = new Point2D(point1.X + radius * xdir, point1.Y + radius * ydir);
                _pntcen3[2] = new Point2D(point1.X - radius * xdir, point1.Y - radius * ydir);
                _pntcen3[1] = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen3[1]), onLine);
                _pntcen3[2] = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen3[2]), onLine);
                Axis2D axe = new(_pntcen3[1], onLine.Direction);
                _cirsol[1]    = new Circle2D(axe, radius);
                axe          = new Axis2D(_pntcen3[2], onLine.Direction);
                _cirsol[2]    = new Circle2D(axe, radius);
                _theSame1[1]  = 0;
                _pnttg1sol[1] = point1;
                _pararg1[1]   = 0.0;
                _par1sol[1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[1], _pnttg1sol[1]);
                _parcen3[1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen3[1]);
                _theSame1[2]  = 0;
                _pnttg1sol[2] = point1;
                _pararg1[2]   = 0.0;
                _par1sol[2]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[2], _pnttg1sol[2]);
                _parcen3[2]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen3[2]);
                _nbrSol       = 2;
            }
            else
            {
                double a, b, c;
                onLine.Coefficients(out a, out b, out c);
                double d = a;
                if (a == 0.0)
                {
                    a    = b;
                    b    = d;
                    xbid = x1;
                    x1   = y1;
                    y1   = xbid;
                }
                if (a != 0.0)
                {
                    DirectPolynomialRoots sol = new((b * b + a * a) / (a * a),
                                        2.0 * (b * c / (a * a) + (b / a) * x1 - y1),
                                        x1 * x1 + y1 * y1 + c * c / (a * a) - radius * radius + 2.0 * c * x1 / a);
                    if (sol.IsDone)
                    {
                        for (int i = 1; i <= sol.SolutionCount; i++)
                        {
                            if (d != 0.0)
                            {
                                yc = sol.Value(i);
                                xc = -(b / a) * yc - c / a;
                            }
                            else
                            {
                                xc = sol.Value(i);
                                yc = -(b / a) * xc - c / a;
                            }
                            _nbrSol++;
                            Point2D center = new(xc, yc);
                            _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =======================================================
                            _qualifier1[_nbrSol] = Qualifier.Noqualifier;
                            _theSame1[_nbrSol]   = 0;
                            _pnttg1sol[_nbrSol]  = point1;
                            _pntcen3[_nbrSol]    = _cirsol[_nbrSol].Location;
                            _pararg1[_nbrSol]    = 0.0;
                            _par1sol[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _parcen3[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen3[_nbrSol]);
                        }
                        _wellDone = true;
                    }
                }
            }
        }
    }
}
