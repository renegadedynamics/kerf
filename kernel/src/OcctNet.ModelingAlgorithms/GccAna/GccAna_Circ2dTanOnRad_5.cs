// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to a point   Point1.                                  +
    //          center on circle     OnCirc.                                  +
    //          radius               Radius.                                  +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Eliminate cases not being the solution.                               +
    //  Create the circle with center in Point1 of radius Radius.             +
    //  Intersect this circle with OnCirc and obtain the center points        +
    //  of found solutions.                                                   +
    //  Create solutions cirsol.                                              +
    //=========================================================================
    /// <summary>Circles of radius Radius passing through a point, centred on a circle.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in Point2D point1, in Circle2D onCirc, double radius, double tolerance)
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
        double   tol   = Math.Abs(tolerance);
        _wellDone       = false;
        _nbrSol         = 0;
        double roncirc = onCirc.Radius();
        double dist1   = point1.Distance(onCirc.Location()) - roncirc;
        double dist2   = point1.Distance(onCirc.Location()) + roncirc;

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else if ((dist1 - radius > tol) || (tol < radius - dist2))
        {
            _wellDone = true;
        }
        else
        {
            int signe = 0;
            if (Math.Abs(dist1 - radius) < tol)
            {
                signe = 1;
            }
            else if (Math.Abs(dist2 - radius) < tol)
            {
                signe = -1;
            }
            if (signe != 0)
            {
                Axis2D axe = new(new Point2D(onCirc.Location().XY()
                                                          - roncirc
                                                                  * new Direction2D(onCirc.Location().X() - signe * point1.X(), // OCCT: Location - signe * Point1 is a sum when signe is -1
                                                                                        onCirc.Location().Y() - signe * point1.Y())
                                                                          .XY()),
                                        dirx);
                _cirsol[1] = new Circle2D(axe, radius);
                //      ================================
                _qualifier1[1] = Qualifier.Noqualifier;
                _theSame1[1]   = 0;
                _pnttg1sol[1]  = point1;
                _pntcen3[1]    = _cirsol[1].Location();
                _pararg1[1]    = 0.0;
                _par1sol[1]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[1], _pnttg1sol[1]);
                _parcen3[1]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen3[1]);
                _wellDone      = true;
                _nbrSol        = 1;
            }
            else
            {
                AnalyticIntersection intp = new(onCirc, new Circle2D(new Axis2D(point1, dirx), radius));
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int i = 1; i <= intp.PointCount; i++)
                        {
                            _nbrSol++;
                            Point2D center = intp.Point(i).Value();
                            _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =======================================================
                            _qualifier1[1] = Qualifier.Noqualifier; // OCCT: index 1, not NbrSol, here and below
                            _theSame1[1]   = 0;
                            _pnttg1sol[1]  = point1;
                            _pntcen3[1]    = _cirsol[1].Location();
                            _par1sol[1]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[1], _pnttg1sol[1]);
                            _parcen3[1]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen3[1]);
                            _pararg1[1]    = 0.0;
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }
}
