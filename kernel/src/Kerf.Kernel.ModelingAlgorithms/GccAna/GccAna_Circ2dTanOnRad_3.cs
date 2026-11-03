// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_3.cxx
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
    //   Circle tangent to a circle   Qualified1 (C1).                        +
    //          center on a circle  OnCirc.                                   +
    //          of radius              Radius.                                +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Eliminate cases not being the solution.                     +
    //  Create parallel(s) to C1 in the required direction(s).       +
    //  Intersect parallel(s) with OnCirc and obtain the   +
    //  center point(s) of found solution(s).       +
    //  Create solution(s) cirsol.                               +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified circle, centred on a circle.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in QualifiedCirc qualified1, in Circle2D onCirc, double radius, double tolerance)
    {
        _cirsol = new(1, 4);
        // OCCT: qualifier1(NbrSol) is written with NbrSol = 0 when sign != 0 (below), outside the array (1, 4): OCCT's
        // range check throws Standard_OutOfRange there, and so does the port
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
        Span<int> signe = stackalloc int[5];
        signe[0] = 0;
        signe[1] = 0;
        signe[2] = 0;
        signe[3] = 0;
        signe[4] = 0;
        Span<double> disparal = stackalloc double[2];
        int    nparal = 0;
        int    sign   = 0;
        _wellDone      = false;
        _nbrSol        = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D                    c1 = qualified1.Qualified();
        NCollection_Array1<Point2D> center = new(1, 4);
        NCollection_Array1<Direction2D> dir1on = new(1, 4);

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            double   r1  = c1.Radius;
            double   r2  = onCirc.Radius;
            double   c1x = c1.Location.X;
            double   c1y = c1.Location.Y;
            Point2D center1 = new(c1x, c1y);
            double   dist = onCirc.Location.DistanceTo(center1);
            double   onx  = onCirc.Location.X;
            double   ony  = onCirc.Location.Y;
            if (qualified1.IsEnclosed)
            {
                //   ============================
                if ((radius - r1 > tol) || (r1 - dist - r2 - radius > tol))
                {
                    _wellDone = true;
                }
                else
                {
                    if (Math.Abs(radius - r1) < tol)
                    {
                        if (onCirc.DistanceTo(center1) < tol)
                        {
                            _nbrSol         = 1;
                            _cirsol[_nbrSol] = c1;
                            //           ==============
                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                            _theSame1[_nbrSol]   = 1;
                            _pntcen3[_nbrSol]    = center1;
                            _parcen3[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen3[_nbrSol]);
                            _wellDone           = true;
                        }
                        else
                        {
                            _wellDone = false;
                        }
                    }
                    else if (Math.Abs(r1 - dist - r2 - radius) <= tol)
                    {
                        dir1on[1] = new Direction2D(c1x - onx, c1y - ony);
                        sign      = -1;
                    }
                    else
                    {
                        nparal      = 1;
                        disparal[0] = Math.Abs(r1 - radius);
                    }
                }
            }
            else if (qualified1.IsEnclosing)
            {
                //   ==================================
                if ((tol < r1 - radius) || (tol < r1 + dist - r2 - radius) || (radius - r1 - dist - r2 > tol))
                {
                    _wellDone = true;
                }
                else
                {
                    if (Math.Abs(radius - r1) < tol)
                    {
                        if (onCirc.DistanceTo(center1) < tol)
                        {
                            _nbrSol         = 1;
                            _cirsol[_nbrSol] = c1;
                            //           ==============
                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                            _theSame1[_nbrSol]   = 1;
                            _pntcen3[_nbrSol]    = center1;
                            _parcen3[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen3[_nbrSol]);
                            _wellDone           = true;
                        }
                        else
                        {
                            _wellDone = true;
                        }
                    }
                    else if ((Math.Abs(r1 + dist - r2 - radius) < tol)
                                      || (Math.Abs(radius - r1 - dist - r2) < tol))
                    {
                        dir1on[1] = new Direction2D(c1x - onx, c1y - ony);
                        if (Math.Abs(r1 + dist - r2 - radius) < tol)
                        {
                            sign = 1;
                        }
                        else
                        {
                            sign = -1;
                        }
                    }
                    else
                    {
                        nparal      = 1;
                        disparal[0] = Math.Abs(r1 - radius);
                    }
                }
            }
            else if (qualified1.IsOutside)
            {
                //   ================================
                if ((dist - r2 - r1 - radius > tol) || (radius - dist - r2 + r1 > tol))
                {
                    _wellDone = true;
                }
                else
                {
                    dir1on[1] = new Direction2D(c1x - onx, c1y - ony);
                    if (Math.Abs(r1 - dist - r2 + radius) < tol)
                    {
                        sign = -1;
                    }
                    else if (Math.Abs(dist - r1 - r2 - radius) < tol)
                    {
                        sign = 1;
                    }
                    else
                    {
                        nparal      = 1;
                        disparal[0] = Math.Abs(r1 + radius);
                    }
                }
            }
            else
            {
                if ((dist - r2 - r1 - radius > tol) || (radius - dist - r1 - r2 > tol))
                {
                    _wellDone = true;
                }
                else if ((r1 - dist - r2 > tol) && (tol < r1 - r2 - dist - radius))
                {
                    _wellDone = true;
                }
                else
                {
                    dir1on[1] = new Direction2D(c1x - onx, c1y - ony);
                    if (Math.Abs(dist - r1 - r2 - radius) < tol)
                    {
                        sign = 1;
                    }
                    else if (dist + r1 + r2 - radius < 0.0)
                    {
                        sign = -1;
                    }
                    else if ((r1 - dist + r2 > 0.0) && (r1 - r2 - dist - radius > 0.0))
                    {
                        sign = -1;
                    }
                    else
                    {
                        nparal      = 2;
                        disparal[0] = Math.Abs(r1 + radius);
                        disparal[1] = Math.Abs(r1 - radius);
                    }
                }
            }
            for (int jj = 0; jj < nparal; jj++)
            {
                AnalyticIntersection intp = new(onCirc, new Circle2D(new Axis2D(center1, dirx), disparal[jj]));
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int i = 1; i <= intp.PointCount; i++)
                        {
                            _nbrSol++;
                            center[_nbrSol] = intp.Point(i).Value();
                            _cirsol[_nbrSol] = new Circle2D(new Axis2D(center[_nbrSol], dirx), radius);
                            //           =============================================================
                            double distcc1 = center[_nbrSol].DistanceTo(center1);
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
                            dir1on[_nbrSol] = new Direction2D(c1x - center[_nbrSol].X, c1y - center[_nbrSol].Y);
                            if ((r1 > radius) && (center[_nbrSol].DistanceTo(center1) <= r1))
                            {
                                signe[_nbrSol - 1] = -1;
                            }
                            else
                            {
                                signe[_nbrSol - 1] = 1;
                            }
                        }
                    }
                    _wellDone = true;
                }
            }
            if (sign != 0)
            {
                center[1] = new Point2D(onCirc.Location.XY() + sign * r2 * dir1on[1].XY());
                _cirsol[1] = new Circle2D(new Axis2D(center[1], dirx), radius);
                //     =====================================================
                double distcc1 = center[1].DistanceTo(center1);
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
                _nbrSol            = 1;
                signe[_nbrSol - 1] = 1;
                _wellDone          = true;
            }
            int ii = 0;
            while (signe[ii] != 0)
            {
                _pnttg1sol[ii + 1] = new Point2D(center[ii + 1].XY() + signe[ii] * radius * dir1on[ii + 1].XY());
                _pntcen3[ii + 1]   = _cirsol[ii + 1].Location;
                _pararg1[ii + 1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[ii + 1]);
                _par1sol[ii + 1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[ii + 1], _pnttg1sol[ii + 1]);
                _parcen3[ii + 1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen3[ii + 1]);
                ii++;
            }
        }
    }
}
