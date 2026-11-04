// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// 2D circles of a given radius tangent to two arguments, each a qualified circle, a qualified line or a point
/// (passed through). Each solution comes with the qualifiers of its arguments (computed for unqualified ones) and its
/// tangency points.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). Up to 8 solutions (two circles), 4 (a circle and a
/// line, two lines) or 2 (with a point).
/// </remarks>
internal sealed partial class AnalyticCirc2d2TanRad
{
    private bool _wellDone;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Qualifier> _qualifier2;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<int> _theSame2;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pnttg2sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;

    // circular tangent to two cercles and given radius
    //====================================================
    //==================================================================
    // Initialize WellDone to false.                                   +
    // Return circle C1 and circle C2.                                 +
    // Leave with error if the construction is impossible.             +
    // Distinguish boundary cases to process them separately.          +
    // Create parallel to C1 in the proper direction.                  +
    // Create parallel to C2 in the proper direction.                  +
    // Intersect parallels ==> center point of the solution.           +
    // Create the solution to be added to already found solutions.     +
    // Fill the fields.                                                +
    //==================================================================
    /// <summary>Circles of radius Radius tangent to two qualified circles.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2d2TanRad(in QualifiedCirc qualified1, in QualifiedCirc qualified2, double radius, double tolerance)
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
        _wellDone = false;
        _nbrSol   = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        bool                          invers = false;
        Circle2D                     c1     = qualified1.Qualified();
        Circle2D                     c2     = qualified2.Qualified();
        double                        r1     = c1.Radius;
        double                        r2     = c2.Radius;
        double                        rbid   = 0.0;
        int                           signe  = 0;
        double                        r3;
        double                        dist;
        NCollection_Array1<Circle2D> c = new(1, 8);
        c[1]           = c1;
        c[2]           = c2;
        int      nbsol = 0;
        Point2D center1 = c1.Location;
        Point2D center2 = c2.Location;
        Point2D center3 = new();
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else if (c[2].Location.IsEqual(c[1].Location, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            _wellDone = true;
        }
        else
        {
            Direction2D dir1 = new(c[2].Location.XY() - c[1].Location.XY());
            dist = (center2).DistanceTo(center1);
            if ((qualified1.IsEnclosed) && qualified2.IsEnclosed)
            {
                //   =========================================================
                if (radius * 2.0 - Math.Abs(r1 + r2 - dist) > tol)
                {
                    _wellDone = true;
                }
                else
                {
                    if ((dist - r1 - r2 > tol) || (tol < Max(r1, r2) - dist - Min(r1, r2)))
                    {
                        _wellDone = true;
                    }
                    else if (Math.Abs(dist - r1 - r2) <= tol
                                      || Math.Abs(Max(r1, r2) - dist - Min(r1, r2)) <= tol)
                    {
                        if (Math.Abs(dist - r1 - r2) <= tol)
                        {
                            rbid  = r2 + (dist - r1 - r2) / 2.0;
                            signe = -1;
                        }
                        else if (Math.Abs(Max(r1, r2) - dist - Min(r1, r2)) <= tol)
                        {
                            if (r1 > r2)
                            {
                                r3 = r2;
                            }
                            else
                            {
                                c2 = c1; // OCCT: overwrites C2, so pararg2 below is a parameter on C1
                                r3 = r1;
                            }
                            rbid  = -r3 + (Min(r1, r2) + dist - Max(r1, r2)) / 2.0;
                            signe = 1;
                        }
                        Axis2D axe = new(new Point2D(center2.XY() - rbid * dir1.XY()), dirx);
                        _cirsol[1] = new Circle2D(axe, radius);
                        //         =================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                        _pnttg1sol[1]  = new Point2D(center2.XY() + signe * r2 * dir1.XY());
                        _pnttg2sol[1]  = new Point2D(center1.XY() + r1 * dir1.XY());
                        _theSame1[1]   = 0;
                        _theSame2[1]   = 0;
                        _wellDone      = true;
                        _nbrSol        = 1;
                    }
                    else
                    {
                        c[1]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r1));
                        c[2]  = new Circle2D(new Axis2D(c[2].Location, dirx), Math.Abs(radius - r2));
                        nbsol = 1;
                    }
                }
            }
            else if (((qualified1.IsEnclosed) && qualified2.IsEnclosing) ||
                              //   ===================================================================
                              ((qualified1.IsEnclosing) && qualified2.IsEnclosed))
            {
                //            ========================================================
                if (qualified1.IsEnclosing)
                {
                    r3      = r1;
                    c[1]    = c2;
                    r1      = r2;
                    c[2]    = c1;
                    r2      = r3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1 = dir1.Reversed();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if ((r2 - radius > tol) || (tol < radius - r1) || (tol > r1 - dist - r2)
                        || ((tol < radius * 2 - r1 - dist - r2) || (tol < r1 + r2 - dist - radius * 2.0)))
                {
                    _wellDone = true;
                }
                else if ((Math.Abs(r2 - radius) <= tol) || (Math.Abs(r1 - radius) <= tol))
                {
                    if (Math.Abs(r2 - radius) <= tol)
                    {
                        c[2]         = c2;
                        r3           = r2;
                        _theSame2[1]  = 1;
                        _theSame1[1]  = 0;
                        _pnttg1sol[1] = new Point2D(c[2].Location.XY() + r3 * dir1.XY());
                    }
                    else if (Math.Abs(radius - r1) <= tol)
                    {
                        c[2]         = c1;
                        r3           = r1;
                        _theSame1[1]  = 1;
                        _theSame2[1]  = 0;
                        _pnttg2sol[1] = new Point2D(c[2].Location.XY() + r3 * dir1.XY());
                    }
                    _wellDone = true;
                    _nbrSol   = 1;
                    Axis2D axe = new(c[2].Location, dirx);
                    _cirsol[1] = new Circle2D(axe, radius);
                    //       =================================
                    _qualifier1[1] = qualified1.Qualifier();
                    _qualifier2[1] = qualified2.Qualifier();
                }
                else
                {
                    if ((Math.Abs(r2 + dist - r1) <= tol) || (Math.Abs(radius * 2.0 - r1 - dist - r2) < tol))
                    {
                        if (Math.Abs(r2 + dist - r1) <= tol)
                        {
                            signe = 1;
                        }
                        else if (Math.Abs(radius * 2.0 - r1 - dist - r2) < tol)
                        {
                            signe = -1;
                        }
                        _wellDone = true;
                        _nbrSol   = 1;
                        Axis2D axe = new(new Point2D(center1.XY() + signe * (r1 - radius) * dir1.XY()), dirx);
                        _cirsol[1] = new Circle2D(axe, radius);
                        //         =================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                        _pnttg1sol[1]  = new Point2D(center1.XY() + signe * r1 * dir1.XY());
                        _pnttg2sol[1]  = new Point2D(center2.XY() + r2 * dir1.XY());
                        _theSame1[1]   = 0;
                        _theSame2[1]   = 0;
                    }
                    else
                    {
                        c[1]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r1));
                        c[2]  = new Circle2D(new Axis2D(c[2].Location, dirx), Math.Abs(radius - r2));
                        nbsol = 1;
                    }
                }
            }
            else if (((qualified1.IsEnclosed) && (qualified2.IsOutside)) ||
                              //   ===================================================================
                              ((qualified1.IsOutside) && (qualified2.IsEnclosed)))
            {
                //            ========================================================
                if (qualified1.IsOutside)
                {
                    c[2]    = c1;
                    c[1]    = c2;
                    r3      = r1;
                    r1      = r2;
                    r2      = r3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1 = dir1.Reversed();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if ((radius - r1 > tol) || (dist - r2 - r1 > tol)
                        || ((r2 - dist - r1 + radius * 2 > tol) || (r1 - r2 - dist - radius * 2 > tol)))
                {
                    _wellDone = true;
                }
                else
                {
                    if (((radius - r1 > 0.0) && (Math.Abs(dist - r1 - r2) <= tol))
                            || (Math.Abs(r1 - r2 + dist - radius * 2.0) <= tol)
                            || (Math.Abs(r1 - r2 - dist - radius * 2.0) <= tol)
                            || (Math.Abs(dist - r1 - r2) <= tol))
                    {
                        if (Math.Abs(r1 - r2 + dist - radius * 2.0) <= tol)
                        {
                            signe = -1;
                        }
                        else
                        {
                            signe = 1;
                            if ((radius - r1 > 0.0) && (Math.Abs(dist - r1 - r2) <= tol))
                            {
                                r2 = r1;
                            }
                            else if (Math.Abs(dist - r1 - r2) <= tol)
                            {
                                r2 = r1;
                                if (Math.Abs(r1 - radius) <= tol)
                                {
                                    _theSame1[1] = 1;
                                }
                            }
                        }
                        _wellDone = true;
                        _nbrSol   = 1;
                        Axis2D axe = new(new Point2D(center1.XY() + signe * (r1 - radius) * dir1.XY()), dirx);
                        _cirsol[1] = new Circle2D(axe, radius);
                        //         =================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                        _pnttg1sol[1]  = new Point2D(center1.XY() + signe * r1 * dir1.XY());
                        _pnttg2sol[1]  = new Point2D(center2.XY() + signe * r2 * dir1.XY());
                        _theSame1[1]   = 0;
                        _theSame2[1]   = 0;
                    }
                    else
                    {
                        c[1]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r1));
                        c[2]  = new Circle2D(new Axis2D(c[2].Location, dirx), radius + r2);
                        nbsol = 1;
                    }
                }
            }
            else if (((qualified1.IsEnclosed) && (qualified2.IsUnqualified)) ||
                              //   =======================================================================
                              ((qualified1.IsUnqualified) && (qualified2.IsEnclosed)))
            {
                //            ============================================================
                if (qualified1.IsUnqualified)
                {
                    c[1]    = c2;
                    c[2]    = c1;
                    r3      = r1;
                    r1      = r2;
                    r2      = r3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1 = dir1.Reversed();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if ((radius - r1 > tol) || (dist - r2 - r1 > tol))
                {
                    _wellDone = true;
                }
                else
                {
                    if ((Math.Abs(dist - r2 - r1) <= tol) || (Math.Abs(radius - r1) <= tol))
                    {
                        _wellDone = true;
                        _nbrSol   = 1;
                        Axis2D ax = new(new Point2D(center1.XY() + (r1 - radius) * dir1.XY()), dirx);
                        _cirsol[1] = new Circle2D(ax, radius);
                        //         ================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                        _pnttg2sol[1]  = new Point2D(center1.XY() + (dist - r2) * dir1.XY());
                        _theSame2[1]   = 0;
                        if (Math.Abs(radius - r1) > 0.0)
                        {
                            _theSame1[1] = 1;
                        }
                        else
                        {
                            _theSame1[1]  = 0;
                            _pnttg1sol[1] = new Point2D(center1.XY() + r1 * dir1.XY());
                        }
                    }
                    else
                    {
                        c[3]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r1));
                        c[4]  = new Circle2D(new Axis2D(c[2].Location, dirx), radius + r2);
                        c[1]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r1));
                        c[2]  = new Circle2D(new Axis2D(c[2].Location, dirx), Math.Abs(radius - r2));
                        nbsol = 2;
                    }
                }
            }
            else if ((qualified1.IsEnclosing) && (qualified2.IsEnclosing))
            {
                //   ==================================================================
                if ((tol < Max(r1, r2) - radius) || (tol < Max(r1, r2) - dist - Min(r1, r2))
                        || (dist + r1 + r2 - radius * 2.0 > tol))
                {
                    _wellDone = true;
                }
                else
                {
                    if ((Math.Abs(dist + Min(r1, r2) - Max(r1, r2)) <= tol)
                            || (Math.Abs(r1 + r2 + dist - 2.0 * radius) <= tol))
                    {
                        if (Math.Abs(dist + Min(r1, r2) - Max(r1, r2)) <= tol)
                        {
                            signe = 1;
                        }
                        else
                        {
                            signe = -1;
                        }
                        _wellDone = true;
                        _nbrSol   = 1;
                        Axis2D axe = new(new Point2D(center1.XY() + signe * (r1 - radius) * dir1.XY()), dirx);
                        _cirsol[1] = new Circle2D(axe, radius);
                        //         =================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                        _pnttg1sol[1]  = new Point2D(center1.XY() + r1 * dir1.XY());
                        _pnttg2sol[1]  = new Point2D(center2.XY() + signe * r2 * dir1.XY());
                        _theSame1[1]   = 0;
                        _theSame2[1]   = 0;
                    }
                    else if (Math.Abs(radius - Max(r1, r2)) <= tol)
                    {
                        _wellDone = true;
                        _nbrSol   = 1;
                        if (r1 > r2)
                        {
                            c[1]         = c1;
                            c[2]         = c2;
                            _theSame1[1]  = 1;
                            _theSame2[1]  = 0;
                            _pnttg2sol[1] = new Point2D(center1.XY() + (dist + r2) * dir1.XY());
                        }
                        else
                        {
                            c[1]         = c2;
                            c[2]         = c1;
                            _theSame1[1]  = 0;
                            _theSame2[1]  = 1;
                            _pnttg1sol[1] = new Point2D(center1.XY() - r1 * dir1.XY());
                        }
                        Axis2D axe = new(c[1].Location, dirx);
                        _cirsol[1] = new Circle2D(axe, radius);
                        //         =================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                    }
                    else
                    {
                        c[1]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r1));
                        c[2]  = new Circle2D(new Axis2D(c[2].Location, dirx), Math.Abs(radius - r2));
                        nbsol = 1;
                    }
                }
            }
            else if (((qualified1.IsEnclosing) && (qualified2.IsOutside)) ||
                              //   ====================================================================
                              ((qualified1.IsOutside) && (qualified2.IsEnclosing)))
            {
                //            =========================================================
                if (qualified1.IsOutside)
                {
                    c[1]    = c2;
                    c[2]    = c1;
                    r3      = r1;
                    r1      = r2;
                    r2      = r3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1 = dir1.Reversed();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if ((r1 - radius > tol) || (tol < r1 + r2 - dist) || (dist - r2 + r1 - radius * 2.0 > tol))
                {
                    _wellDone = true;
                }
                else if (((Math.Abs(r1 - radius) <= tol) || (Math.Abs(r1 + r2 - dist) <= tol))
                                  || (Math.Abs(dist - r2 + r1 - radius * 2.0) <= tol))
                {
                    _wellDone = true;
                    _nbrSol   = 1;
                    if ((Math.Abs(r1 - radius) <= tol) || (Math.Abs(r1 + r2 - dist) <= tol))
                    {
                        _theSame1[1] = 1;
                    }
                    else
                    {
                        _theSame1[1] = 0;
                    }
                    _theSame2[1] = 0;
                    Axis2D axe = new(new Point2D(center1.XY() + (r1 - radius) * dir1.XY()), dirx);
                    _cirsol[1] = new Circle2D(axe, radius);
                    //       =================================
                    _qualifier1[1] = qualified1.Qualifier();
                    _qualifier2[1] = qualified2.Qualifier();
                    _pnttg1sol[1]  = new Point2D(center1.XY() - r1 * dir1.XY());
                    _pnttg2sol[1]  = new Point2D(center1.XY() + (dist - r2) * dir1.XY());
                }
                else
                {
                    c[1]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r1));
                    c[2]  = new Circle2D(new Axis2D(c[2].Location, dirx), radius + r2);
                    nbsol = 1;
                }
            }
            else if (((qualified1.IsEnclosing) && (qualified2.IsUnqualified)) ||
                              //   ========================================================================
                              ((qualified1.IsUnqualified) && (qualified2.IsEnclosing)))
            {
                //            =============================================================
                if (qualified1.IsUnqualified)
                {
                    c[1]    = c2;
                    c[2]    = c1;
                    r3      = r1;
                    r1      = r2;
                    r2      = r3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    invers  = true;
                    dir1 = dir1.Reversed();
                }
                if ((tol < r1 - dist - r2) || (r1 - radius > tol))
                {
                    _wellDone = true;
                }
                // OCCT: "> 0.0" holds for every configuration not already rejected, so this branch always returns one
                // circle, centred at center1 + (Radius - R1) dir1, which is not tangent to C2 in general
                else if ((Math.Abs(r1 - radius) <= tol) || (Math.Abs(r1 - dist - r2) > 0.0))
                {
                    if (Math.Abs(r1 - radius) <= tol)
                    {
                        _theSame1[1] = 1;
                        if ((Math.Abs(radius - r2) <= tol) && (center1.DistanceTo(center2) <= tol))
                        {
                            _theSame2[1] = 1;
                        }
                    }
                    else if (Math.Abs(r1 - dist - r2) > 0.0)
                    {
                        _theSame1[1] = 0;
                        _theSame2[1] = 0;
                    }
                    _wellDone = true;
                    _nbrSol   = 1;
                    Axis2D axe = new(new Point2D(center1.XY() + (radius - r1) * dir1.XY()), dirx);
                    _cirsol[1] = new Circle2D(axe, radius);
                    //       =================================
                    _qualifier1[1] = qualified1.Qualifier();
                    _qualifier2[1] = qualified2.Qualifier();
                    _pnttg1sol[1]  = new Point2D(center1.XY() + r1 * dir1.XY());
                    _pnttg2sol[1]  = new Point2D(center1.XY() + (dist + r2) * dir1.XY());
                }
                else
                {
                    c[3]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r1));
                    c[4]  = new Circle2D(new Axis2D(c[2].Location, dirx), radius + r2);
                    c[1]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r1));
                    c[2]  = new Circle2D(new Axis2D(c[2].Location, dirx), Math.Abs(radius - r2));
                    nbsol = 2;
                }
            }
            else if ((qualified1.IsOutside) && (qualified2.IsOutside))
            {
                //   ==============================================================
                if (tol < Max(r1, r2) - dist - Min(r1, r2))
                {
                    _wellDone = true;
                }
                else if (dist - r1 - r2 - radius * 2.0 > tol)
                {
                    _wellDone = true;
                }
                else
                {
                    if (Math.Abs(dist + Min(r1, r2) - Max(r1, r2)) <= tol)
                    {
                        _wellDone = true;
                        _nbrSol   = 1;
                        if (r1 < r2)
                        {
                            signe = -1;
                        }
                        else
                        {
                            signe = 1;
                        }
                        Axis2D axe = new(new Point2D(center1.XY() + signe * (radius + r1) * dir1.XY()), dirx);
                        _cirsol[1] = new Circle2D(axe, radius);
                        //         =================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                        _pnttg1sol[1]  = new Point2D(center1.XY() + signe * r1 * dir1.XY());
                        _pnttg2sol[1]  = _pnttg1sol[1];
                        _theSame1[1]   = 0;
                        _theSame2[1]   = 0;
                    }
                    else if (Math.Abs(dist - r1 - r2 - radius * 2.0) <= tol)
                    {
                        _wellDone = true;
                        _nbrSol   = 1;
                        Axis2D ax = new(new Point2D(center1.XY() + (r1 + radius) * dir1.XY()), dirx);
                        _cirsol[1] = new Circle2D(ax, radius);
                        //         ================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                        _pnttg1sol[1]  = new Point2D(center1.XY() + r1 * dir1.XY());
                        _pnttg2sol[1]  = new Point2D(center2.XY() - r2 * dir1.XY());
                        _theSame1[1]   = 0;
                        _theSame2[1]   = 0;
                    }
                    else
                    {
                        c[1]  = new Circle2D(new Axis2D(c[1].Location, dirx), radius + r1);
                        c[2]  = new Circle2D(new Axis2D(c[2].Location, dirx), radius + r2);
                        nbsol = 1;
                    }
                }
            }
            else if (((qualified1.IsOutside) && (qualified2.IsUnqualified)) ||
                              //   ======================================================================
                              ((qualified1.IsUnqualified) && (qualified2.IsOutside)))
            {
                //            ===========================================================
                if (qualified1.IsUnqualified)
                {
                    c[1]    = c2;
                    r3      = r1;
                    r1      = r2;
                    c[2]    = c1;
                    r2      = r3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1 = dir1.Reversed();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if (tol < r1 - dist - r2)
                {
                    _wellDone = true;
                }
                else if ((tol < r2 - dist - r1) && (tol < radius * 2.0 - r2 - dist + r1))
                {
                    _wellDone = true;
                }
                else if ((dist - r1 - r2 > tol) && (tol < dist - r1 - r2 - radius * 2.0))
                {
                    _wellDone = true;
                }
                else
                {
                    if ((Math.Abs(r1 - r2 - dist) <= tol)
                            || ((Math.Abs(dist - r1 - r2) <= tol)
                                    && (Math.Abs(radius * 2.0 - dist + r1 + r2) <= tol))
                            || ((Math.Abs(dist + r1 - r2) <= tol)
                                    && (Math.Abs(r2 + dist - r1 - radius * 2.0) <= tol)))
                    {
                        _wellDone = true;
                        _nbrSol   = 1;
                        Axis2D axe = new(new Point2D(center1.XY() + (radius + r1) * dir1.XY()), dirx);
                        _cirsol[1] = new Circle2D(axe, radius);
                        //         =================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                        _pnttg1sol[1]  = new Point2D(center1.XY() + r1 * dir1.XY());
                        _pnttg2sol[1]  = new Point2D(center1.XY() + (dist + r2) * dir1.XY());
                        _theSame1[1]   = 0;
                        _theSame2[1]   = 0;
                    }
                    else
                    {
                        c[3]  = new Circle2D(new Axis2D(c[1].Location, dirx), radius + r1);
                        c[4]  = new Circle2D(new Axis2D(c[2].Location, dirx), radius + r2);
                        c[1]  = new Circle2D(new Axis2D(c[1].Location, dirx), radius + r1);
                        c[2]  = new Circle2D(new Axis2D(c[1].Location, dirx), Math.Abs(radius - r2)); // OCCT: centred on C(1), not C(2)
                        nbsol = 2;
                    }
                }
            }
            else if ((qualified1.IsUnqualified) && (qualified2.IsUnqualified))
            {
                //   ======================================================================
                if ((dist - r1 - r2 > tol) && (tol < (dist - r1 - r2 - radius * 2)))
                {
                    _wellDone = true;
                }
                else if ((Max(r1, r2) - dist - Min(r1, r2) > tol)
                                  && (((Max(r1, r2) - dist - Min(r1, r2)) - radius * 2.0 > tol)))
                {
                    _wellDone = true;
                }
                else
                {
                    double p3 = Max(r1, r2) - Min(r1, r2) - dist - radius * 2.0;
                    double p4 = dist - r1 - r2;
                    double p5 = radius * 2.0 - dist + r1 + r2;
                    if (p3 > 0.0)
                    {
                        dist = Max(r1, r2) - Min(r1, r2) - radius * 2.0;
                    }
                    else if (p4 > 0.0 && p5 < 0.0)
                    {
                        r1 = dist - r2 - radius * 2.0;
                    }
                    c[1]  = new Circle2D(new Axis2D(center1, dirx), Math.Abs(radius - r1));
                    c[2]  = new Circle2D(new Axis2D(center2, dirx), Math.Abs(radius - r2));
                    c[3]  = new Circle2D(new Axis2D(center1, dirx), Math.Abs(radius - r1));
                    c[4]  = new Circle2D(new Axis2D(center2, dirx), radius + r2);
                    c[5]  = new Circle2D(new Axis2D(center1, dirx), radius + r1);
                    c[6]  = new Circle2D(new Axis2D(center2, dirx), Math.Abs(radius - r2));
                    c[7]  = new Circle2D(new Axis2D(center1, dirx), radius + r1);
                    c[8]  = new Circle2D(new Axis2D(center2, dirx), radius + r2);
                    nbsol = 4;
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
                            for (int i = 1; i <= intp.PointCount; i++)
                            {
                                _nbrSol++;
                                Point2D center = intp.Point(i).Value();
                                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                //             =======================================================
                                dir1 = new Direction2D(center.XY() - center1.XY());
                                Direction2D dir2 = new(center.XY() - center2.XY());
                                double   distcc1 = center.DistanceTo(center1);
                                double   distcc2 = center.DistanceTo(center2);
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
                                else if (Math.Abs(distcc2 + radius - r2) < tol)
                                {
                                    _qualifier2[_nbrSol] = Qualifier.Enclosed;
                                }
                                else if (Math.Abs(distcc2 - r2 - radius) < tol)
                                {
                                    _qualifier2[_nbrSol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier2[_nbrSol] = Qualifier.Enclosing;
                                }
                                if ((center.DistanceTo(center1) > r1) && (radius < center.DistanceTo(center1) + r1))
                                {
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() - radius * dir1.XY());
                                }
                                else if ((center.DistanceTo(center1) < r1) && (radius < r1))
                                {
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dir1.XY());
                                }
                                else
                                {
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() - radius * dir1.XY());
                                }
                                if ((center.DistanceTo(center2) > r2) && (radius < center.DistanceTo(center2) + r2))
                                {
                                    _pnttg2sol[_nbrSol] = new Point2D(center.XY() - radius * dir2.XY());
                                }
                                else if ((center.DistanceTo(center2) < r2) && (radius < r2))
                                {
                                    _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius * dir2.XY());
                                }
                                else
                                {
                                    _pnttg2sol[_nbrSol] = new Point2D(center.XY() - radius * dir2.XY());
                                }
                                _theSame1[_nbrSol] = 0;
                                _theSame2[_nbrSol] = 0;
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
        }
        // swapping of resulting tangency points if necessary
        if (invers)
        {
            Point2D psav = new();
            for (int i = 1; i <= _nbrSol; i++)
            {
                psav         = _pnttg1sol[i];
                _pnttg1sol[i] = _pnttg2sol[i];
                _pnttg2sol[i] = psav;
            }
        }
        // calculation of parameters of tangency points
        for (int i = 1; i <= _nbrSol; i++)
        {
            _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg1sol[i]);
            if (_theSame1[i] == 0)
            {
                _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[i]);
            }
            _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg2sol[i]);
            if (_theSame2[i] == 0)
            {
                _pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c2, _pnttg2sol[i]);
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
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        return _cirsol[index];
    }

    /// <summary>The qualifiers of the arguments for the solution number Index: those given, or those computed for unqualified arguments (GccEnt_noqualifier for a point).</summary>
    public void WhichQualifier(int index, out Qualifier qualif1, out Qualifier qualif2)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        qualif1 = _qualifier1[index];
        qualif2 = _qualifier2[index];
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
            if (_theSame1[index] == 0)
            {
                parSol = _par1sol[index];
                parArg = _pararg1[index];
                pntSol = _pnttg1sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>The tangency point of the solution number Index with the second argument: its parameter on the solution, its parameter on the argument, and the point.</summary>
    public void Tangency2(int index, out double parSol, out double parArg, out Point2D pntSol)
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
            if (_theSame2[index] == 0)
            {
                parSol = _par2sol[index];
                parArg = _pararg2[index];
                pntSol = _pnttg2sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
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

    /// <summary>True if the solution number Index is the second argument.</summary>
    public bool IsTheSame2(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame2[index] == 0)
        {
            return false;
        }
        return true;
    }
}
