// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanRad
{
    // circular tangent to a circle, a line and a given radius
    //==============================================================
    //========================================================================
    // Initialize WellDone to false.                                         +
    // Return circle C1 and straight line L2.                                +
    // Leave with error if the construction is impossible.                   +
    // Create parallel to C1 in the proper direction.                        +
    // Create parallel to L2 in the proper direction.                        +
    // Intersect parallels ==> center point of the solution.                 +
    // Create the solution and add it to already found ones.                 +
    // Fill the fields.                                                      +
    //========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified circle and a qualified line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2d2TanRad(in QualifiedCirc qualified1, in QualifiedLin qualified2, double radius, double tolerance)
    {
        _qualifier1 = new(1, 8);
        _qualifier2 = new(1, 8);
        _theSame1 = new(1, 8);
        _theSame2 = new(1, 8);
        _cirsol = new(1, 8);
        _pnttg1sol = new(1, 8);
        _pnttg2sol = new(1, 8);
        _par1sol = new(1, 8);
        _par2sol = new(1, 8);
        _pararg1 = new(1, 8);
        _pararg2 = new(1, 8);


        double   tol = Math.Abs(tolerance);
        Direction2D dirx = new(Direction2D.D.X);
        _nbrSol   = 0;
        _wellDone = false;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        int i;
        for (i = 1; i <= 8; i++)
        {
            _theSame2[i] = 0;
        }
        int                        ncote1 = 0;
        int                        ncote2 = 0;
        int                        nbsol  = 0;
        double                     cote   = 0.0;
        double                     ccote  = 0.0;
        NCollection_Array1<double> cote1 = new(1, 3);
        NCollection_Array1<double> cote2 = new(1, 2);
        Circle2D                  c1   = qualified1.Qualified();
        Line2D                   l2   = qualified2.Qualified();
        double                     xdir = (l2.Direction).X;
        double                     ydir = (l2.Direction).Y;
        Direction2D normL2 = new(-ydir, xdir);
        double                     lxloc = (l2.Location).X;
        double                     lyloc = (l2.Location).Y;
        double                     cxloc = (c1.Location).X;
        double                     cyloc = (c1.Location).Y;
        double                     r1    = c1.Radius;
        Point2D center1 = new(cxloc, cyloc);
        Point2D origin2 = new(lxloc, lyloc);
        double                     dist     = radius * 2.0 + r1;
        double                     distance = l2.DistanceTo(center1);

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (distance - dist > tol)
            {
                _wellDone = true;
            }
            else if (qualified1.IsEnclosed)
            {
                //  =================================
                if (qualified2.IsEnclosed)
                {
                    //    ============================
                    if (distance - r1 > tol)
                    {
                        _wellDone = true;
                    }
                    else if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                        && (radius * 2.0 - r1 + distance > tol))
                                      || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                              && (radius * 2.0 - r1 - distance > tol))
                                      || (radius - r1 > tol))
                    {
                        _wellDone = true;
                    }
                    else
                    {
                        if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                  && (radius * 2.0 > r1 - distance))
                                || (Math.Abs(distance - r1) < tol)
                                || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                        && (radius * 2.0 > (r1 + distance + tol))))
                        {
                            cote  = 1.0;
                            nbsol = 3;
                            ccote = 1.0;
                        }
                        else
                        {
                            ncote1   = 1;
                            ncote2   = 1;
                            cote1[1] = r1 - radius;
                            cote2[1] = 1.0;
                            nbsol    = 1;
                        }
                    }
                }
                else if (qualified2.IsOutside)
                {
                    //    ================================
                    if (distance > r1 + tol)
                    {
                        _wellDone = true;
                    }
                    else if ((((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc)) > 0.0)
                                        && (radius * 2.0 - r1 + distance > tol))
                                      || (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc)) < 0.0)
                                              && (radius * 2.0 - r1 - distance > tol))
                                      || (radius - r1 > tol))
                    {
                        _wellDone = true;
                    }
                    else
                    {
                        if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                  && (radius * 2.0 > r1 - distance))
                                || (Math.Abs(r1 - radius) < tol)
                                || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                        && (radius * 2.0 > (r1 + distance)))
                                || (Math.Abs(distance - r1) < tol))
                        {
                            cote  = -1.0;
                            nbsol = 3;
                            ccote = 1.0;
                        }
                        else
                        {
                            ncote1   = 1;
                            ncote2   = 1;
                            cote1[1] = r1 - radius;
                            cote2[1] = -1.0;
                            nbsol    = 1;
                        }
                    }
                }
                else if (qualified2.IsUnqualified)
                {
                    //    ====================================
                    if ((distance - r1 > tol) || (radius * 2.0 - r1 - distance > tol) || (radius - r1 > tol))
                    {
                        _wellDone = true;
                    }
                    else if (distance > r1)
                    {
                        if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                        {
                            cote = 1.0;
                        }
                        else
                        {
                            cote  = -1.0;
                            nbsol = 3;
                        }
                        nbsol = 3;
                        ccote = 1;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 2;
                        cote1[1] = r1 - radius;
                        cote2[1] = 1.0;
                        cote2[2] = -1.0;
                        nbsol    = 1;
                    }
                }
                if (nbsol == 3)
                {
                }
            }
            else if (qualified1.IsEnclosing)
            {
                //  ==================================
                if (qualified2.IsEnclosed)
                {
                    //    =================================
                    if ((distance < r1 - tol) || (radius < r1 - tol) || (radius * 2 < distance + r1 - tol)
                            || (tol < r1 - radius) || (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0))
                    {
                        _wellDone = true;
                    }
                    else if ((distance < r1) || (radius < r1) || (radius * 2.0 < distance + r1))
                    {
                        cote  = 1.0;
                        ccote = 1.0;
                        nbsol = 3;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = radius - r1;
                        cote2[1] = 1.0;
                        nbsol    = 1;
                    }
                }
                else if (qualified2.IsOutside)
                {
                    //    ================================
                    if ((tol < r1 - distance) || (tol < distance + r1 - radius * 2.0) || (tol < r1 - radius)
                            || (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                    {
                        _wellDone = true;
                    }
                    else if ((distance < r1) || (radius * 2.0 < distance + r1) || (radius < r1))
                    {
                        cote  = -1.0;
                        ccote = -1.0;
                        nbsol = 3;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = radius - r1;
                        cote2[1] = -1.0;
                        nbsol    = 1;
                    }
                }
                else if (qualified2.IsUnqualified)
                {
                    //    ====================================
                    if ((distance < r1 - tol) || (radius * 2.0 < distance + r1 - tol) || (radius < r1 - tol))
                    {
                        _wellDone = true;
                    }
                    else if ((distance < r1) || (radius * 2.0 < distance + r1))
                    {
                        if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                        {
                            cote  = 1.0;
                            ccote = 1.0;
                            nbsol = 3;
                        }
                        else
                        {
                            ccote = -1.0;
                            cote  = -1.0;
                            nbsol = 3;
                        }
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 2;
                        cote1[1] = radius - r1;
                        cote2[1] = 1.0;
                        cote2[2] = -1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((qualified1.IsOutside) && (qualified2.IsEnclosed))
            {
                //  ===============================================================
                if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > r1 + tol))
                        || (distance > r1 + radius * 2.0 + tol))
                {
                    _wellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > r1))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                      && (distance > r1 + 2.0 * radius))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = radius + r1;
                        cote2[1] = 1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((qualified1.IsOutside) && (qualified2.IsOutside))
            {
                //  ==============================================================
                if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > r1 + tol))
                        || (distance > r1 + radius * 2.0 + tol))
                {
                    _wellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > r1))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                      && (distance > r1 + 2.0 * radius))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = radius + r1;
                        cote2[1] = -1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((qualified1.IsOutside) && (qualified2.IsUnqualified))
            {
                //  ==================================================================
                if (distance > radius * 2.0 + r1 + tol)
                {
                    _wellDone = true;
                }
                else if (distance > radius * 2.0 + r1)
                {
                    if (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                    {
                        cote = 1.0;
                    }
                    else
                    {
                        cote = -1.0;
                    }
                    nbsol = 4;
                }
                else
                {
                    ncote1   = 1;
                    ncote2   = 2;
                    cote1[1] = radius + r1;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                    nbsol    = 1;
                }
            }
            else if ((qualified1.IsUnqualified) && (qualified2.IsEnclosed))
            {
                //  ===================================================================
                if ((distance > r1 + radius * 2.0 + tol)
                        || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > r1 + tol)))
                {
                    _wellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > r1))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                      && (distance > r1 + 2.0 * radius))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 2;
                        ncote2   = 1;
                        cote1[1] = Math.Abs(radius - r1);
                        cote1[2] = radius + r1;
                        cote2[1] = 1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((qualified1.IsUnqualified) && (qualified2.IsOutside))
            {
                //  ==================================================================
                if ((distance > r1 + radius * 2.0 + tol)
                        || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > r1 + tol)))
                {
                    _wellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > r1))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                      && (distance > r1 + 2.0 * radius))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 2;
                        ncote2   = 1;
                        cote1[1] = Math.Abs(radius - r1);
                        cote1[2] = radius + r1;
                        cote2[1] = -1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((qualified1.IsUnqualified) && (qualified2.IsUnqualified))
            {
                //  ======================================================================
                if (distance > r1 + radius * 2.0 + tol)
                {
                    _wellDone = true;
                }
                else if (distance > r1 + radius * 2.0)
                {
                    if (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                    {
                        cote = -1.0;
                    }
                    else
                    {
                        cote = 1.0;
                    }
                    nbsol = 4;
                }
                else
                {
                    ncote1   = 2;
                    ncote2   = 2;
                    cote1[1] = Math.Abs(radius - r1);
                    cote1[2] = radius + r1;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                    nbsol    = 1;
                }
            }
            if (nbsol == 1)
            {
                for (int jcote1 = 1; jcote1 <= ncote1; jcote1++)
                {
                    for (int jcote2 = 1; jcote2 <= ncote2; jcote2++)
                    {
                        if (cote1[jcote1] < tol)
                        {
                            continue;
                        }
                        Circle2D cirint = new(new Axis2D(center1, dirx), cote1[jcote1]);
                        Line2D linint = new(
                            new Point2D(lxloc - cote2[jcote2] * ydir * radius, lyloc + cote2[jcote2] * xdir * radius),
                            l2.Direction);
                        AnalyticIntersection intp = new(linint, cirint);
                        if (intp.IsDone)
                        {
                            if (!intp.IsEmpty)
                            {
                                for (i = 1; i <= intp.PointCount; i++)
                                {
                                    _nbrSol++;
                                    Point2D center = intp.Point(i).Value();
                                    Axis2D axe = new(center, dirx);
                                    _cirsol[_nbrSol] = new Circle2D(axe, radius);
    //              ======================================
                                    Direction2D dc2 = new(origin2.XY() - center.XY());
                                    double   distcc1 = center.DistanceTo(center1);
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
                                    _theSame1[_nbrSol] = 0;
                                    if ((radius < r1) && center.DistanceTo(center1) <= r1)
                                    {
                                        _pnttg1sol[_nbrSol] =
                                            new Point2D(center.XY() + radius * (new Direction2D(center.XY() - center1.XY()).XY()));
                                    }
                                    else
                                    {
                                        _pnttg1sol[_nbrSol] =
                                            new Point2D(center.XY() - radius * (new Direction2D(center.XY() - center1.XY()).XY()));
                                    }
                                    _pnttg2sol[_nbrSol] =
                                        new Point2D(center.XY() + cote2[jcote2] * radius * new Xy(ydir, -xdir));
                                }
                            }
                            _wellDone = true;
                        }
                    }
                }
            }
            else if (nbsol == 2)
            {
                Point2D cen = new(center1.XY() + cote * (r1 + radius) * new Xy(-ydir, xdir));
                Axis2D axe = new(cen, dirx);
                _cirsol[1] = new Circle2D(axe, radius);
                //    =================================
                _wellDone      = true;
                _nbrSol        = 1;
                _theSame1[1]   = 0;
                _qualifier1[1] = qualified1.Qualifier();
                _qualifier2[1] = qualified2.Qualifier();
                _pnttg1sol[1]  = new Point2D(cen.XY() + cote * radius * new Xy(ydir, -xdir));
                _pnttg2sol[1]  = new Point2D(cen.XY() + radius * new Xy(ydir, -xdir));
            }
            else if (nbsol == 3)
            {
                _wellDone = true;
                _nbrSol   = 1;
                Point2D cen = new(center1.XY() + cote * (r1 - radius) * new Xy(ydir, -xdir));
                Axis2D axe = new(cen, dirx);
                _cirsol[1] = new Circle2D(axe, radius);
                //    =================================
                _qualifier1[1] = qualified1.Qualifier();
                _qualifier2[1] = qualified2.Qualifier();
                _pnttg2sol[1]  = new Point2D(cen.XY() + cote * radius * new Xy(ydir, -xdir));
                if (Math.Abs(r1 - radius) > 0.0)
                {
                    _pnttg1sol[1] = new Point2D(cen.XY() + ccote * radius * new Xy(ydir, -xdir));
                }
                else
                {
                    _theSame1[1] = 1;
                }
            }
            else if (nbsol == 4)
            {
                Point2D cent = new(center1.XY() + cote * (r1 + radius) * new Xy(-ydir, xdir));
                Axis2D axe = new(cent, dirx);
                _cirsol[1] = new Circle2D(axe, radius);
                //    =================================
                _qualifier1[1] = qualified1.Qualifier();
                _qualifier2[1] = qualified2.Qualifier();
                _wellDone      = true;
                _nbrSol        = 1;
                _theSame1[1]   = 0;
                _pnttg1sol[1]  = new Point2D(cent.XY() + cote * radius * new Xy(ydir, -xdir));
                _pnttg2sol[1]  = new Point2D(cent.XY() + cote * radius * new Xy(-ydir, xdir));
            }
        }
        for (i = 1; i <= _nbrSol; i++)
        {
            _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg1sol[i]);
            if (_theSame1[i] == 0)
            {
                _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[i]);
            }
            _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg2sol[i]);
            _pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[i]);
        }
    }
}
