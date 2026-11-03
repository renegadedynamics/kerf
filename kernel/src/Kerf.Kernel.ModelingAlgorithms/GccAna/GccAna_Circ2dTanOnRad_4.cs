// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to straight line Qualified1 (L1).                     +
    //          center on  circle OnCirc.                                     +
    //          with radius       Radius.                                     +
    //                                                                        +
    //  Initialize table of solutions cirsol and all fields.                  +
    //  Eliminate cases not being the solution.                               +
    //  Create parallel line(s) to L1 in the required direction(s).           +
    //  Intersect parallel line(s) with OnCirc and obtain                     +
    //  center points of found solutions.                                     +
    //  Create solutions cirsol.                                              +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified line, centred on a circle.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in QualifiedLin qualified1, in Circle2D onCirc, double radius, double tolerance)
    {
        _cirsol = new(1, 4);
        _qualifier1 = new(1, 4);
        _theSame1 = new(1, 4);
        _pnttg1sol = new(1, 4);
        _pntcen3 = new(1, 4);
        _par1sol = new(1, 4);
        _pararg1 = new(1, 4);
        _parcen3 = new(1, 4);


        _theSame1.Init(0);
        Direction2D dirx = new(Direction2D.D.X);
        double   tol = Math.Abs(tolerance);
        _wellDone     = false;
        _nbrSol       = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //  Initialisation of various variables.                                 +
        //=========================================================================

        int      nbsol = 0;
        int      sign  = 0;
        Line2D l1    = qualified1.Qualified();
        Point2D origin1 = l1.Location;
        Direction2D dir1 = l1.Direction;
        Direction2D normL1 = new(-dir1.Y, dir1.X);
        double   dist1 = l1.DistanceTo(onCirc.Location) - onCirc.Radius;
        double   dist2 = l1.DistanceTo(onCirc.Location) + onCirc.Radius;

        //=========================================================================
        //  Processing.                                                           +
        //=========================================================================

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            l1 = qualified1.Qualified();
            if ((dist1 - radius > tol) || (tol < radius - dist2))
            {
                _wellDone = true;
            }
            else
            {

                // to modify later

                if (dist1 - radius > 0.0)
                {
                    dist1 = radius;
                }
                else if (dist2 - radius < 0.0)
                {
                    dist2 = radius;
                }

                if (qualified1.IsEnclosed)
                {
                    //     ============================
                    nbsol = 1;
                    sign  = -1;
                }
                else if (qualified1.IsOutside)
                {
                    //     ================================
                    nbsol = 1;
                    sign  = 1;
                }
                else
                {
                    //     ====
                    nbsol = 2;
                    sign  = 1;
                }
                for (int j = 1; j <= nbsol; j++)
                {
                    sign = -sign;
                    Line2D l = new(
                        new Point2D(origin1.X - sign * radius * dir1.Y, origin1.Y + sign * radius * dir1.X),
                        dir1);
                    AnalyticIntersection intp = new(l, onCirc);
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (int i = 1; i <= intp.PointCount; i++)
                            {
                                _nbrSol++;
                                Point2D center = intp.Point(i).Value();
                                Axis2D axe = new(center, dirx);
                                _cirsol[_nbrSol] = new Circle2D(axe, radius);
                                //             ======================================
                                Direction2D dc1 = new(origin1.XY() - center.XY());
                                sign = (int)dc1.Dot(normL1); // OCCT: truncates the dot product (0 unless it is exactly 1 or -1) and clobbers the loop's side
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
                                _pntcen3[_nbrSol]   = _cirsol[_nbrSol].Location;
                                _pnttg1sol[_nbrSol] = new Point2D(
                                    _pntcen3[_nbrSol].XY() + new Xy(sign * radius * dir1.Y, -sign * radius * dir1.X));
                                _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                                _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen3[_nbrSol]);
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
        }
    }
}
