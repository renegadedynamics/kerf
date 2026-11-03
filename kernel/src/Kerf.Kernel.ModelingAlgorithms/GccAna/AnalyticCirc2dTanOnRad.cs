// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// 2D circles of a given radius tangent to a qualified circle or line, or passing through a point, with their centre
/// on a line or a circle.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 4 solutions.</remarks>
public sealed partial class AnalyticCirc2dTanOnRad
{
    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pntcen3;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _parcen3;

    // PRO12736 : bug quand OnLine // Ox, JCT 20/03/98
    //========================================================================
    //       circular tangent to element of type :      - Circle.            +
    //                                                  - Line.             +
    //                                                  - Point.             +
    //              center on second element of type :  - Circle.  +
    //                                                  - Line.   +
    //              of given radius : Radius.                             +
    //========================================================================



    //=========================================================================
    //  Circle tangent  :  to circle Qualified1 (C1).                         +
    //         center   :  on straight line OnLine.                           +
    //         of radius :  Radius.                                           +
    //                                                                        +
    //  Initialise the table of solutions cirsol and all fields.              +
    //  Eliminate depending on the qualifier the cases not being solutions.   +
    //  Solve the equation of the second degree indicating that the found center +
    //  point (xc,yc) is at a distance Radius from circle C1 and              +
    //                     on straight line OnLine.                           +
    //  The solutions aret represented by circles :                           +
    //                   - with center Pntcen(xc,yc)                          +
    //                   - with radius Radius.                                +
    //=========================================================================

    /// <summary>Circles of radius Radius tangent to a qualified circle, centred on a line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in QualifiedCirc qualified1, in Line2D onLine, double radius, double tolerance)
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
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        NCollection_Array1<double> coef = new(1, 2);
        Circle2D                  c1 = qualified1.Qualified();

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            int      nbsol = 0;
            int      signe = 0;
            Point2D center = new();
            double   xc;
            double   yc;
            double   r1    = c1.Radius;
            double   dist  = onLine.DistanceTo(c1.Location);
            double   xdir  = (onLine.Direction).X;
            double   ydir  = (onLine.Direction).Y;
            double   lxloc = (onLine.Location).X;
            double   lyloc = (onLine.Location).Y;
            Point2D center1 = c1.Location;
            double   x1 = center1.X;
            double   y1 = center1.Y;
            if (qualified1.IsEnclosed)
            {
                //   ============================
                if (tol < radius - r1 + dist)
                {
                    _wellDone = true;
                }
                else
                {
                    if (Math.Abs(radius - r1 + dist) < tol)
                    {
                        _wellDone = true;
                        _nbrSol   = 1;
                        if (-ydir * (x1 - lxloc) + xdir * (y1 - lyloc) < 0.0)
                        {
                            center = new Point2D(x1 - ydir * dist, y1 + xdir * dist);
                        }
                        else
                        {
                            center = new Point2D(x1 + ydir * dist, y1 - xdir * dist);
                        }
                        signe = 1;
                    }
                    else
                    {
                        coef[1] = (r1 - radius) * (r1 - radius);
                        nbsol   = 1;
                    }
                }
            }
            else if (qualified1.IsEnclosing)
            {
                //   ==================================
                if (r1 + dist - radius > tol)
                {
                    _wellDone = true;
                }
                else
                {
                    if (r1 + dist - radius > 0.0)
                    {
                        _wellDone = true;
                        _nbrSol   = 1;
                        if (-ydir * (x1 - lxloc) + xdir * (y1 - lyloc) < 0.0)
                        {
                            center = new Point2D(x1 - ydir * dist, y1 + xdir * dist);
                        }
                        else
                        {
                            center = new Point2D(x1 + ydir * dist, y1 - xdir * dist);
                        }
                        signe = -1;
                    }
                    else
                    {
                        coef[1] = (radius - r1) * (radius - r1);
                        nbsol   = 1;
                    }
                }
            }
            else
            {
                //   ====
                if (dist - r1 - radius > tol)
                {
                    _wellDone = false;
                }
                else
                {
                    if (Math.Abs(dist - r1 - radius) < tol)
                    {
                        _wellDone = true;
                        _nbrSol   = 1;
                        if (-ydir * (x1 - lxloc) + xdir * (y1 - lyloc) < 0.0)
                        {
                            center = new Point2D(x1 - ydir * dist, y1 + xdir * dist);
                        }
                        else
                        {
                            center = new Point2D(x1 + ydir * dist, y1 - xdir * dist);
                        }
                        signe = -1;
                    }
                    else
                    {
                        if (qualified1.IsOutside)
                        {
                            //         ===========================
                            coef[1] = (radius + r1) * (radius + r1);
                            nbsol   = 1;
                        }
                        else
                        {
                            //         ====
                            coef[1] = (radius - r1) * (radius - r1);
                            coef[2] = (radius + r1) * (radius + r1);
                            nbsol   = 2;
                        }
                    }
                }
            }
            if (signe != 0)
            {
                _cirsol[1] = new Circle2D(new Axis2D(center, dirx), radius);
                //     ==================================================
                double distcc1 = center.DistanceTo(center1);
                if (!qualified1.IsUnqualified)
                {
                    _qualifier1[1] = qualified1.Qualifier();
                }
                else if (Math.Abs(distcc1 + radius - r1) < tol)
                {
                    _qualifier1[1] = Qualifier.Enclosed;
                }
                else if (Math.Abs(distcc1 - r1 - radius) < tol)
                {
                    _qualifier1[1] = Qualifier.Outside;
                }
                else
                {
                    _qualifier1[1] = Qualifier.Enclosing;
                }
                if (Math.Abs(radius - r1) <= tol)
                {
                    _theSame1[1] = 1;
                }
                else
                {
                    Direction2D dir1cen = new(center.X - x1, center.Y - y1);
                    _pnttg1sol[1] = new Point2D(center.XY() + signe * radius * dir1cen.XY());
                    _par1sol[1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[1], _pnttg1sol[1]);
                    _pararg1[1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[1]);
                }
                _pntcen3[1] = _cirsol[_nbrSol].Location;
                _parcen3[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen3[1]);
            }
            else if (nbsol > 0)
            {
                for (int j = 1; j <= nbsol; j++)
                {
                    double a, b, c;
                    onLine.Coefficients(out a, out b, out c);
                    double d = a;
                    double x0, y0;
                    if (Math.Abs(d) <= tol)
                    {
                        a  = b;
                        b  = d;
                        x0 = y1;
                        y0 = x1;
                    }
                    else
                    {
                        x0 = x1;
                        y0 = y1;
                    }
                    DirectPolynomialRoots sol = new((b * b + a * a) / (a * a),
                                        2.0 * (b * c / (a * a) + (b / a) * x0 - y0),
                                        x0 * x0 + y0 * y0 + c * c / (a * a) - coef[j] + 2.0 * c * x0 / a);
                    if (sol.IsDone)
                    {
                        for (int i = 1; i <= sol.SolutionCount; i++)
                        {

                            if (Math.Abs(d) > tol)
                            {
                                yc = sol.Value(i);
                                xc = -(b / a) * yc - c / a;
                            }
                            else
                            {
                                xc = sol.Value(i);
                                yc = -(b / a) * xc - c / a;
                            }
                            center = new Point2D(xc, yc);
                            if (onLine.DistanceTo(center) > tol)
                            {
                                continue;
                            }
                            _nbrSol++;
                            _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =======================================================
                            double distcc1 = center.DistanceTo(center1);
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
                            Direction2D dir1cen = new(center.X - x1, center.Y - y1);
                            // OCCT: the signs are swapped: dir1cen points from C1 to the centre, so these are the points
                            // of the solution opposite its tangency point
                            if ((radius > r1) || (center.DistanceTo(center1) > r1))
                            {
                                _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dir1cen.XY());
                            }
                            else
                            {
                                _pnttg1sol[_nbrSol] = new Point2D(center.XY() - radius * dir1cen.XY());
                            }
                            _pntcen3[_nbrSol] = _cirsol[_nbrSol].Location;
                            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                            _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen3[_nbrSol]);
                        }
                        _wellDone = true;
                    }
                }
            }
        }
    }

    /// <summary>True if the construction succeeded (even with no solution).</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of solutions.</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }

    /// <summary>The solution number Index.</summary>
    public Circle2D ThisSolution(int index)
    {
        if (index > _nbrSol || index <= 0)
        {
            throw new OutOfRangeException();
        }
        return _cirsol[index];
    }

    /// <summary>The qualifiers of the arguments for the solution number Index: those given, or those computed for unqualified arguments (GccEnt_noqualifier for a point).</summary>
    public void WhichQualifier(int index, out Qualifier qualif1)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            qualif1 = _qualifier1[index];
        }
    }

    /// <summary>The tangency point of the solution number Index with the first argument: its parameter on the solution, its parameter on the argument, and the point.</summary>
    public void Tangency1(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            parSol = _par1sol[index];
            parArg = _pararg1[index];
            pntSol = _pnttg1sol[index];
        }
    }

    /// <summary>The centre of the solution number Index on the On-curve: its parameter there, and the point.</summary>
    public void CenterOn3(int index, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            parArg = _parcen3[index];
            pntSol = _pnttg1sol[index]; // OCCT: the tangency point, not the centre pntcen3
        }
    }

    /// <summary>True if the solution number Index is the first argument.</summary>
    public bool IsTheSame1(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame1[index] == 0)
        {
            return false;
        }

        return true;
    }
}
