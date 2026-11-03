// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2d2TanRad
{
    //   Circle passing by two points of given radius.
    //   ==============================================
    //==============================================================================
    //   Solution of the equation of second degree showing that the center of the  +
    //   circle is equidistant from two points.                                    +
    //==============================================================================
    /// <summary>Circles of radius Radius passing through two points.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2d2TanRad(in Point2D point1, in Point2D point2, double radius, double tolerance)
    {
        _qualifier1 = new(1, 2);
        _qualifier2 = new(1, 2);
        _theSame1 = new(1, 2);
        _theSame2 = new(1, 2);
        _cirsol = new(1, 2);
        _pnttg1sol = new(1, 2);
        _pnttg2sol = new(1, 2);
        _par1sol = new(1, 2);
        _par2sol = new(1, 2);
        _pararg1 = new(1, 2);
        _pararg2 = new(1, 2);


        Direction2D dirx = new(Direction2D.D.X);
        double   tol = Math.Abs(tolerance);
        _nbrSol       = 0;
        _wellDone     = false;
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (point1.Distance(point2) - radius * 2.0 > tol)
            {
                _wellDone = true;
            }
            else
            {
                if (point1.Distance(point2) - radius * 2.0 > 0.0)
                {
                    _wellDone = true;
                    _nbrSol   = 1;
                    Axis2D axe = new(new Point2D(0.5 * (point1.XY() + point2.XY())), dirx);
                    _cirsol[1] = new Circle2D(axe, radius);
                    //      =================================
                    _qualifier1[1] = Qualifier.Noqualifier;
                    _qualifier2[1] = Qualifier.Noqualifier;
                    _theSame1[1]   = 0;
                    _theSame2[1]   = 0;
                    _pnttg1sol[1]  = point1;
                    _pnttg2sol[1]  = point2;
                }
                else
                {
                    Circle2D c1 = new(new Axis2D(point1, dirx), radius);
                    Circle2D c2 = new(new Axis2D(point2, dirx), radius);
                    AnalyticIntersection intp = new(c1, c2);
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (int i = 1; i <= intp.PointCount; i++)
                            {
                                _nbrSol++;
                                Point2D center = intp.Point(i).Value();
                                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                //            =======================================================
                                _qualifier1[_nbrSol] = Qualifier.Noqualifier;
                                _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                                _theSame1[_nbrSol]   = 0;
                                _theSame2[_nbrSol]   = 0;
                                _pnttg1sol[_nbrSol]  = point1;
                                _pnttg2sol[_nbrSol]  = point2;
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
        }
        for (int i = 1; i <= _nbrSol; i++)
        {
            _pararg1[i] = 0.0;
            _pararg2[i] = 0.0;
            _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg1sol[i]);
            _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg2sol[i]);
        }
    }
}
