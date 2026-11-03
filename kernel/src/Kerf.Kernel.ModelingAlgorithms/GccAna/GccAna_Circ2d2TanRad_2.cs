// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_2.cxx
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
    // circulare tangent to a circle a point and a given radius
    //=============================================================
    //========================================================================
    // Initialize WellDone to false.                                       +
    // Return circle C1.                                             +
    // Leave with error if the construction is impossible.     +
    // Create parallel to C1 in the proper direction.                           +
    // Create circle with center in Point1 of radius Radius.                   +
    // Intersect the parallel and the circle.                              +
    //                              ==> The center point of the solution.   +
    // Create the solution that will be added to already found solutions.         +
    // Fill the fields.                                                +
    //========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified circle and passing through a point.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2d2TanRad(in QualifiedCirc qualified1, in Point2D point2, double radius, double tolerance)
    {
        _qualifier1 = new(1, 4);
        _qualifier2 = new(1, 4);
        _theSame1 = new(1, 4);
        _theSame2 = new(1, 4);
        _cirsol = new(1, 4);
        _pnttg1sol = new(1, 4);
        _pnttg2sol = new(1, 4);
        _par1sol = new(1, 4);
        _par2sol = new(1, 4);
        _pararg1 = new(1, 4);
        _pararg2 = new(1, 4);


        Direction2D dirx = new(Direction2D.D.X);
        double   tol = Math.Abs(tolerance);
        _nbrSol       = 0;
        _wellDone     = false;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        int i;
        for (i = 1; i <= 4; i++)
        {
            _theSame1[i] = 0;
            _theSame2[i] = 0;
        }
        double                        deport = 0.0;
        int                           signe  = 0;
        int                           nbsol  = 0;
        Circle2D                     c1     = qualified1.Qualified();
        NCollection_Array1<Circle2D> c = new(1, 4);
        double                        r1       = c1.Radius;
        double                        distance = (c1.Location).DistanceTo(point2);
        double                        dispc1   = c1.DistanceTo(point2);
        Direction2D dir1 = new(point2.XY() - (c1.Location.XY()));
        Point2D center1 = c1.Location;
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (dispc1 - radius * 2.0 > tol)
            {
                _wellDone = true;
            }
            else if (qualified1.IsEnclosed)
            {
                //  =================================
                if ((distance - r1 > tol) || (radius - r1 > tol))
                {
                    _wellDone = true;
                }
                else
                {
                    if (Math.Abs(distance - r1) < tol)
                    {
                        nbsol  = -1;
                        deport = r1 - radius;
                        signe  = 1;
                    }
                    else
                    {
                        c[1]  = new Circle2D(c1.XAxis, Math.Abs(radius - r1));
                        c[2]  = new Circle2D(new Axis2D(point2, dirx), radius);
                        nbsol = 1;
                    }
                }
            }
            else if (qualified1.IsEnclosing)
            {
                //  ==================================
                if ((tol < r1 - distance) || (tol < r1 - radius))
                {
                    _wellDone = true;
                }
                else
                {
                    if (Math.Abs(distance - r1) < tol)
                    {
                        nbsol  = -1;
                        deport = r1 - radius;
                        signe  = 1;
                    }
                    else
                    {
                        c[1]  = new Circle2D(c1.XAxis, Math.Abs(radius - r1));
                        c[2]  = new Circle2D(new Axis2D(point2, dirx), radius);
                        nbsol = 1;
                    }
                }
            }
            else if (qualified1.IsOutside)
            {
                //  ================================
                if (tol < r1 - distance)
                {
                    _wellDone = true;
                }
                else if ((Math.Abs(distance - r1) < tol) || (Math.Abs(dispc1 - radius * 2.0) < tol))
                {
                    nbsol  = -1;
                    deport = r1 + radius;
                    signe  = -1;
                }
                else
                {
                    c[1]  = new Circle2D(c1.XAxis, radius + r1);
                    c[2]  = new Circle2D(new Axis2D(point2, dirx), radius);
                    nbsol = 1;
                }
            }
            else if (qualified1.IsUnqualified)
            {
                //  ====================================
                if (Math.Abs(dispc1 - radius * 2.0) < tol)
                {
                    _wellDone = true;
                    Point2D center = new(center1.XY() + (distance - radius) * dir1.XY());
                    _cirsol[1] = new Circle2D(new Axis2D(center, dirx), radius);
                    //      ==================================================
                    if (Math.Abs(center.DistanceTo(center1) - r1) < tol)
                    {
                        _qualifier1[1] = Qualifier.Enclosed;
                    }
                    else
                    {
                        _qualifier1[1] = Qualifier.Outside;
                    }
                    _qualifier2[1] = Qualifier.Noqualifier;
                    _pnttg1sol[1]  = new Point2D(center.XY() - radius * dir1.XY());
                    _pnttg2sol[1]  = point2;
                    _wellDone      = true;
                    _nbrSol        = 1;
                }
                else if ((Math.Abs(r1 - radius) < tol) && (Math.Abs(distance - r1) < tol))
                {
                    _cirsol[1] = c1;
                    //      =========================
                    _qualifier1[1] = Qualifier.Unqualified;
                    _qualifier2[1] = Qualifier.Noqualifier;
                    _theSame1[1]   = 1;
                    _pnttg2sol[1]  = point2;
                    _wellDone      = true;
                    _nbrSol        = 1;
                    c[1]          = new Circle2D(c1.XAxis, radius + r1);
                    c[2]          = new Circle2D(new Axis2D(point2, dirx), radius);
                    nbsol         = 1;
                }
                else
                {
                    c[1]  = new Circle2D(c1.XAxis, Math.Abs(radius - r1));
                    c[2]  = new Circle2D(new Axis2D(point2, dirx), radius);
                    c[3]  = new Circle2D(c1.XAxis, radius + r1);
                    c[4]  = new Circle2D(new Axis2D(point2, dirx), radius);
                    nbsol = 2;
                }
            }
            if (nbsol > 0)
            {
                for (int j = 1; j <= nbsol; j++)
                {
                    AnalyticIntersection intp = new(c[2 * j - 1], c[2 * j]);
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (i = 1; i <= intp.PointCount; i++)
                            {
                                _nbrSol++;
                                Point2D center = intp.Point(i).Value();
                                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                //            =======================================================
                                double distcc1 = center1.DistanceTo(center);
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
                                _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                                dir1               = new Direction2D(center.XY() - center1.XY());
                                if ((center.DistanceTo(center1) > c1.Radius)
                                        && (radius < center.DistanceTo(center1) + c1.Radius))
                                {
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() - radius * dir1.XY());
                                }
                                else if ((center.DistanceTo(center1) < c1.Radius) && (radius < c1.Radius))
                                {
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dir1.XY());
                                }
                                else
                                {
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() - radius * dir1.XY());
                                }
                                _pnttg2sol[_nbrSol] = point2;
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
            else if (nbsol < 0)
            {
                Point2D center = new(center1.XY() + deport * dir1.XY());
                _cirsol[1] = new Circle2D(new Axis2D(center, dirx), radius);
                //    ==================================================
                _qualifier1[1] = qualified1.Qualifier();
                _qualifier2[1] = Qualifier.Noqualifier;
                if (Math.Abs(deport) <= tol && Math.Abs(radius - r1) <= tol)
                {
                    _theSame1[1] = 1;
                }
                else
                {
                    _pnttg1sol[1] = new Point2D(center.XY() + signe * radius * dir1.XY());
                }
                _pnttg2sol[1] = point2;
                _wellDone     = true;
                _nbrSol       = 1;
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
            _pararg2[i] = 0.0;
        }
    }
}
