// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanRad
{
    // Modified:	Thu Jun 18 15:45:00 1998
    //		PRO10310 : cas ou le point est sur la droite


    // circular tangent to a line and a point and a given radius
    //=============================================================
    //========================================================================
    // Initialize WellDone to false.                                         +
    // Return line L1.                                                       +
    // Leave with error if the construction is impossible.                   +
    // Create parallel to L1 in the proper direction.                        +
    // Create the circle with center at Point1 of radius Radius.             +
    // Intersect the parallel and the circle.                                +
    //                              ==> The center point of the  solution.   +
    // Create the solution to be added to already found solutions.           +
    // Fill the fields.                                                      +
    //========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified line and passing through a point.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2d2TanRad(in QualifiedLin qualified1, in Point2D point2, double radius, double tolerance)
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
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        int                        nbsol  = 0;
        int                        nbcote = 0;
        NCollection_Array1<double> cote = new(1, 2);
        Line2D                   l1     = qualified1.Qualified();
        double                     displ1 = l1.DistanceTo(point2);
        double                     xdir   = (l1.Direction).X;
        double                     ydir   = (l1.Direction).Y;
        double                     lxloc  = (l1.Location).X;
        double                     lyloc  = (l1.Location).Y;
        Point2D origin1 = new(lxloc, lyloc);
        Direction2D normL1 = new(-ydir, xdir);
        double                     cxloc = point2.X;
        double                     cyloc = point2.Y;

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }

        else
        {
            if (displ1 - radius * 2.0 > tol)
            {
                _wellDone = true;
            }
            else if (qualified1.IsEnclosed)
            {
                //  =================================
                if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0))
                {
                    _wellDone = true;
                }
                else
                {
                    if (displ1 - radius * 2.0 > 0.0)
                    {
                        nbsol   = 2;
                        _nbrSol  = 1;
                        nbcote  = 1;
                        cote[1] = 1.0;
                    }
                    else
                    {
                        nbsol   = 1;
                        nbcote  = 1;
                        cote[1] = 1.0;
                    }
                }
            }
            else if (qualified1.IsOutside)
            {
                //  ================================
                if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                {
                    _wellDone = true;
                }
                else
                {
                    if (displ1 - radius * 2.0 > 0.0)
                    {
                        nbsol   = 2;
                        nbcote  = 1;
                        cote[1] = -1.0;
                    }
                    else
                    {
                        nbsol   = 1;
                        nbcote  = 1;
                        cote[1] = -1.0;
                    }
                }
            }
            else if (qualified1.IsUnqualified)
            {
                //  ====================================
                if (displ1 - radius * 2.0 > 0.0)
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                    {
                        nbsol   = 2;
                        nbcote  = 1;
                        cote[1] = 1.0;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0))
                    {
                        nbsol   = 2;
                        nbcote  = 1;
                        cote[1] = -1.0;
                    }
                }
                else
                {
                    nbsol   = 1;
                    nbcote  = 2;
                    cote[1] = 1.0;
                    cote[2] = -1.0;
                }
            }

            if (nbsol == 1)
            {
                if (displ1 < 1.0e-10)
                {
                    // particular case when Point2 is on the line
                    // construct two solutions directly
                    for (int jcote = 1; jcote <= nbcote; jcote++)
                    {
                        _nbrSol++;
                        Point2D center = new(cxloc - cote[jcote] * ydir * radius, cyloc + cote[jcote] * xdir * radius);
                        _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                        //        =======================================================
                        _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                        if (!qualified1.IsUnqualified)
                        {
                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                        }
                        else if (cote[jcote] > 0.0)
                        {
                            _qualifier1[_nbrSol] = Qualifier.Outside;
                        }
                        else
                        {
                            _qualifier1[_nbrSol] = Qualifier.Enclosed;
                        }
                        _theSame1[_nbrSol]  = 0;
                        _theSame2[_nbrSol]  = 0;
                        _pnttg1sol[_nbrSol] = point2;
                        _pnttg2sol[_nbrSol] = point2;
                    }
                    _wellDone = true;
                }
                else
                {
                    Circle2D cirint = new(new Axis2D(point2, dirx), radius);
                    for (int jcote = 1; jcote <= nbcote; jcote++)
                    {
                        Line2D linint = new(
                            new Point2D(lxloc - cote[jcote] * ydir * radius, lyloc + cote[jcote] * xdir * radius),
                            l1.Direction);
                        AnalyticIntersection intp = new(linint, cirint);
                        if (intp.IsDone)
                        {
                            if (!intp.IsEmpty)
                            {
                                for (int i = 1; i <= intp.PointCount && _nbrSol < 2; i++)
                                {
                                    _nbrSol++;
                                    Point2D center = intp.Point(i).Value();
                                    _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                    //            =======================================================
                                    Direction2D dc1 = new(origin1.XY() - center.XY());
                                    _qualifier2[_nbrSol] = Qualifier.Noqualifier;
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
                                    _theSame1[_nbrSol] = 0;
                                    _theSame2[_nbrSol] = 0;
                                    _pnttg1sol[_nbrSol] =
                                        new Point2D(center.XY() + cote[jcote] * radius * new Xy(ydir, -xdir));
                                    _pnttg2sol[_nbrSol] = point2;
                                }
                            }
                            _wellDone = true;
                        }
                    }
                }
            }

            else if (nbsol == 2)
            {
                Point2D center = new(point2.XY() + cote[1] * radius * new Xy(-ydir, xdir));
                _wellDone  = true;
                _nbrSol    = 1;
                _cirsol[1] = new Circle2D(new Axis2D(center, dirx), radius);
                //    ==================================================
                _qualifier2[1] = Qualifier.Noqualifier;
                _theSame1[1]   = 0;
                _theSame2[1]   = 0;
                _pnttg1sol[1]  = new Point2D(center.XY() + cote[1] * radius * new Xy(ydir, -xdir));
                _pnttg2sol[1]  = point2;
            }
        }

        for (int i = 1; i <= _nbrSol; i++)
        {
            _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg1sol[i]);
            _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[i]);
            _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg2sol[i]);
            _pararg2[i] = 0.0;
        }
    }
}
