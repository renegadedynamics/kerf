// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2d2Tan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2d2Tan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// 2D lines tangent to two circles (up to four), tangent to a circle and through a point (up to two), or through two
/// points. A circle may be qualified enclosing, outside or unqualified (not enclosed).
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). As in OCCT, WhichQualifier returns the qualifiers
/// given (GccEnt_unqualified for an unqualified circle, not a computed one), and the second circle's qualifier is
/// mostly the first's.
/// </remarks>
internal sealed class AnalyticLin2d2Tan
{
    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Line2D> _linsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Qualifier> _qualifier2;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pnttg2sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;

    private AnalyticLin2d2Tan(int size)
    {
        _linsol = new NCollection_Array1<Line2D>(1, size);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, size);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, size);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, size);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, size);
        _par1sol = new NCollection_Array1<double>(1, size);
        _par2sol = new NCollection_Array1<double>(1, size);
        _pararg1 = new NCollection_Array1<double>(1, size);
        _pararg2 = new NCollection_Array1<double>(1, size);
    }

    //=========================================================================
    //   Straight line passing through two points.                                      +
    //   ===============================                                      +
    //=========================================================================

    /// <summary>The line through two points, if they are at least Tolerance apart (else not done).</summary>
    public AnalyticLin2d2Tan(in Point2D thePoint1, in Point2D thePoint2, double tolerance) : this(1)
    {
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        double dist = thePoint1.DistanceTo(thePoint2);
        _qualifier1[1] = Qualifier.Noqualifier;
        _qualifier2[1] = Qualifier.Noqualifier;
        if (dist >= tol)
        {
            Direction2D dir = new(thePoint2.X - thePoint1.X, thePoint2.Y - thePoint1.Y);
            _linsol[1] = new Line2D(thePoint1, dir);
            //   ===================================
            _wellDone = true;
            _nbrSol = 1;
            _pnttg1sol[1] = thePoint1;
            _pnttg2sol[1] = thePoint2;
            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[_nbrSol], _pnttg1sol[_nbrSol]);
            _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[_nbrSol], _pnttg2sol[_nbrSol]);
            _pararg1[1] = 0.0;
            _pararg2[1] = 0.0;
        }
    }

    //=========================================================================
    //   Straight line tangent to a circle passing by a point.                +
    //   =================================================                    +
    //   Basing on the qualifier attached to circle Qualified1 (C1) define    +
    //   the direction of the tangent to be calculated.                       +
    //   This tangent will have connection point P1 (point of tangency with the circle. +
    //   It has angle A (sinus R1/dist or -R1/dist) with straight line        +
    //   passing by the center of the circle and ThePoint.                    +
    //=========================================================================

    /// <summary>The lines tangent to the qualified circle and through ThePoint.</summary>
    /// <exception cref="BadQualifierException">The circle is qualified enclosed, or GccEnt_noqualifier.</exception>
    public AnalyticLin2d2Tan(in QualifiedCirc qualified1, in Point2D thePoint, double tolerance) : this(2)
    {
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D c1 = qualified1.Qualified();
        double r1 = c1.Radius;

        if (qualified1.IsEnclosed)
        {
            throw new BadQualifierException();
        }
        // ============================
        else if (tol < r1 - thePoint.DistanceTo(c1.Location))
        {
            _wellDone = true;
        }
        else if (Math.Abs(thePoint.DistanceTo(c1.Location) - r1) <= tol)
        {
            Direction2D dir = new(new Vector2D(c1.Location, thePoint));
            _linsol[1] = new Line2D(thePoint, new Direction2D(-dir.Y, dir.X));
            //   =====================================================================
            _qualifier1[1] = qualified1.Qualifier();
            _qualifier2[1] = Qualifier.Noqualifier;
            _wellDone = true;
            _nbrSol = 1;
            _pnttg1sol[1] = thePoint;
            _pnttg2sol[1] = thePoint;
        }
        else
        {
            double signe = 1;
            double dist = thePoint.DistanceTo(c1.Location);
            double d = dist - Math.Sqrt(dist * dist - r1 * r1);
            if (qualified1.IsEnclosing)
            {
                //   =============================
                signe = 1;
                _nbrSol = 1;
            }
            else if (qualified1.IsOutside)
            {
                signe = -1;
                _nbrSol = 1;
            }
            else if (qualified1.IsUnqualified)
            {
                //   ====================================
                signe = 1;
                _nbrSol = 2;
            }
            for (int i = 1; i <= _nbrSol; i++)
            {
                Point2D p1 = c1.Location.Rotated(thePoint, Math.Asin(signe * r1 / dist));
                Direction2D d1 = new(new Vector2D(p1, thePoint));
                p1 = new Point2D(p1.XY() + d * d1.XY());
                _linsol[i] = new Line2D(p1, new Direction2D(new Vector2D(p1, thePoint)));
                //     ========================================================
                _qualifier1[i] = qualified1.Qualifier();
                _qualifier2[i] = Qualifier.Noqualifier;
                _pnttg1sol[i] = p1;
                _pnttg2sol[i] = thePoint;
                signe = -signe;
            }
            _wellDone = true;
        }

        for (int i = 1; i <= _nbrSol; i++)
        {
            _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[i], _pnttg1sol[i]);
            _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[i], _pnttg2sol[i]);
            // a circle of radius 0 puts the tangency point at its centre
            _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[i]);
            _pararg2[i] = 0.0;
        }
    }

    //=========================================================================
    //   Straight line tangent to two circles.                                +
    //   ====================================                                 +
    //   In the boundary cas (two interior circles are tangent to each other) +
    //   take the straight line orthogonal to the straight line connecting    +
    //   two circles.                                                         +
    //   In other cases subject the center of C1 (Qualified1) or of           +
    //   C2 (Qualified2), provided that R1 is greater than R2, to a           +
    //   rotation of angle A with sinus(A) = (R1+R2)/dist or                  +
    //                            sinus(A) = (R1-R2)/dist or                  +
    //                            sinus(A) = (R2-R1)/dist or                  +
    //                            sinus(A) = (-R1-R2)/dist                    +
    //   The point found this way is P1 or P2.                                +
    //   The direction of the straight line to be calculated  should  pass by +
    //   the center of rotation (center of C1 or of C2) and P1 or P2.         +
    //   Then translate the straight line to make it tangent to C1.           +
    //=========================================================================

    /// <summary>The lines tangent to two qualified circles.</summary>
    /// <exception cref="BadQualifierException">A circle is qualified enclosed, or GccEnt_noqualifier.</exception>
    public AnalyticLin2d2Tan(in QualifiedCirc qualified1, in QualifiedCirc qualified2, double tolerance)
        : this(4)
    {
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();

        if (qualified1.IsEnclosed || qualified2.IsEnclosed)
        {
            // =======================================================
            throw new BadQualifierException();
        }
        else
        {
            double r1 = c1.Radius;
            double r2 = c2.Radius;
            Direction2D d1 = new();
            int signe = 1;
            double dist = c1.Location.DistanceTo(c2.Location);
            if (tol < Max(r1, r2) - dist - Min(r1, r2))
            {
                _wellDone = true;
            }
            else if (!qualified1.IsUnqualified || !qualified2.IsUnqualified)
            {
                //   ====================================================================
                if (qualified1.IsEnclosing && qualified2.IsEnclosing)
                {
                    //     =========================================================
                    if (Math.Abs(dist + Min(r1, r2) - Max(r1, r2)) <= tol && dist >= tol)
                    {
                        if (r1 < r2)
                        {
                            d1 = new Direction2D(new Vector2D(c2.Location, c1.Location));
                        }
                        else
                        {
                            d1 = new Direction2D(new Vector2D(c1.Location, c2.Location));
                        }
                        Point2D p1 = new(c1.Location.XY() + r1 * d1.XY());
                        _linsol[1] = new Line2D(p1, new Direction2D(-d1.Y, d1.X));
                        //         =================================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified2.Qualifier();
                        _pnttg1sol[1] = p1;
                        _pnttg2sol[1] = p1;
                        _wellDone = true;
                        _nbrSol = 1;
                    }
                    else
                    {
                        Point2D p1 = c2.Location.Rotated(c1.Location, Math.Asin((r1 - r2) / dist));
                        d1 = new Direction2D(new Vector2D(c1.Location, p1));
                        p1 = new Point2D((c1.Location.XY() + new Xy(r1 * d1.Y, -r1 * d1.X)));
                        _linsol[1] = new Line2D(p1, d1);
                        //         ===========================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        _pnttg1sol[1] = p1;
                        _pnttg2sol[1] = new Point2D(c2.Location.XY() + new Xy(r2 * d1.Y, -r2 * d1.X));
                        _wellDone = true;
                        _nbrSol = 1;
                    }
                }
                else if ((qualified1.IsEnclosing && qualified2.IsOutside) ||
                         //     ================================================================
                         (qualified2.IsEnclosing && qualified1.IsOutside))
                {
                    //              =====================================================
                    if (qualified1.IsEnclosing && qualified2.IsOutside)
                    {
                        signe = 1;
                    }
                    else
                    {
                        signe = -1;
                    }
                    if (r1 + r2 - dist > tol)
                    {
                        _wellDone = true;
                    }
                    else if (Math.Abs(dist - r1 - r2) < tol && dist > tol)
                    {
                        d1 = new Direction2D(new Vector2D(c1.Location, c2.Location));
                        Point2D p1 = new(c1.Location.XY() + r1 * d1.XY());
                        _linsol[1] = new Line2D(p1, new Direction2D(-d1.Y, d1.X));
                        //          =================================================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        _pnttg1sol[1] = p1;
                        _pnttg2sol[1] = p1;
                        _wellDone = true;
                        _nbrSol = 1;
                    }
                    else
                    {
                        Point2D p1 = c2.Location.Rotated(c1.Location, Math.Asin(signe * (r1 + r2) / dist));
                        d1 = new Direction2D(new Vector2D(c1.Location, p1));
                        p1 = new Point2D(c1.Location.XY() + signe * (new Xy(r1 * d1.Y, -r1 * d1.X)));
                        _linsol[1] = new Line2D(p1, d1);
                        //         ===========================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        _pnttg1sol[1] = p1;
                        _pnttg2sol[1] = new Point2D(c2.Location.XY() + signe * (new Xy(-r2 * d1.Y, r2 * d1.X)));
                        _wellDone = true;
                        _nbrSol = 1;
                    }
                }
                else if (qualified1.IsOutside && qualified2.IsOutside)
                {
                    //     =========================================================
                    if (Math.Abs(dist + Min(r1, r2) - Max(r1, r2)) < tol && dist > tol)
                    {
                        if (r1 < r2)
                        {
                            d1 = new Direction2D(new Vector2D(c2.Location, c1.Location));
                        }
                        else
                        {
                            d1 = new Direction2D(new Vector2D(c1.Location, c2.Location));
                        }
                        _linsol[1] = new Line2D(new Point2D(c1.Location.XY() + r1 * d1.XY()),
                                                 //         =============================================================
                                                 new Direction2D(d1.Y, -d1.X));
                        //                              =========================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        _pnttg1sol[1] = new Point2D(c1.Location.XY() + r1 * d1.XY());
                        _pnttg2sol[1] = _pnttg1sol[1];
                        _wellDone = true;
                        _nbrSol = 1;
                    }
                    else
                    {
                        Point2D p1 = c2.Location.Rotated(c1.Location, Math.Asin((r2 - r1) / dist));
                        d1 = new Direction2D(new Vector2D(c1.Location, p1));
                        p1 = new Point2D(c1.Location.XY() + new Xy(-r1 * d1.Y, r1 * d1.X));
                        _linsol[1] = new Line2D(p1, d1);
                        //         ===========================
                        _qualifier1[1] = qualified1.Qualifier();
                        _qualifier2[1] = qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        _pnttg1sol[1] = p1;
                        _pnttg2sol[1] = new Point2D(c2.Location.XY() + (new Xy(-r2 * d1.Y, r2 * d1.X)));
                        _wellDone = true;
                        _nbrSol = 1;
                    }
                }
                else
                {
                    if ((qualified1.IsUnqualified && qualified2.IsEnclosing) ||
                        //       ===============================================================
                        (qualified2.IsUnqualified && qualified1.IsEnclosing))
                    {
                        //           =========================================================
                        if (qualified2.IsUnqualified)
                        {
                            signe = 1;
                        }
                        else
                        {
                            signe = -1;
                        }
                        if (Math.Abs(dist + Min(r1, r2) - Max(r1, r2)) < tol && dist > tol)
                        {
                            if (r1 < r2)
                            {
                                d1 = new Direction2D(new Vector2D(c2.Location, c1.Location));
                            }
                            else
                            {
                                d1 = new Direction2D(new Vector2D(c1.Location, c2.Location));
                            }
                            _linsol[1] = new Line2D(new Point2D(c1.Location.XY() + r1 * d1.XY()),
                                                     //           =============================================================
                                                     new Direction2D(-d1.Y, d1.X));
                            //                                =========================
                            _qualifier1[1] = qualified1.Qualifier();
                            _qualifier2[1] = qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                            _pnttg1sol[1] = new Point2D(c1.Location.XY() + r1 * d1.XY());
                            _pnttg2sol[1] = _pnttg1sol[1];
                            _wellDone = true;
                            _nbrSol = 1;
                        }
                        else
                        {
                            Point2D p1 = c2.Location.Rotated(c1.Location, Math.Asin((r1 - r2) / dist));
                            d1 = new Direction2D(new Vector2D(c1.Location, p1));
                            p1 = new Point2D(c1.Location.XY() + new Xy(r1 * d1.Y, -r1 * d1.X));
                            _linsol[1] = new Line2D(p1, d1);
                            //           ===========================
                            _qualifier1[1] = qualified1.Qualifier();
                            _qualifier2[1] = qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                            _pnttg1sol[1] = p1;
                            _pnttg2sol[1] =
                                new Point2D(c2.Location.XY() + signe * (new Xy(r2 * d1.Y, -r2 * d1.X)));
                            _wellDone = true;
                            _nbrSol = 1;
                            if (Min(r1, r2) + Max(r1, r2) < dist)
                            {
                                Point2D p2 =
                                    c2.Location.Rotated(c1.Location, Math.Asin(signe * (r1 + r2) / dist));
                                Direction2D d2 = new(new Vector2D(c1.Location, p2));
                                p2 = new Point2D(c1.Location.XY() + signe * (new Xy(r1 * d2.Y, -r1 * d2.X)));
                                _linsol[2] = new Line2D(p2, d2);
                                //             ===========================
                                // OCCT: writes the first solution's qualifiers (the second's stay unset) and gives the
                                // second solution the first's tangency point P1
                                _qualifier1[1] = qualified1.Qualifier();
                                _qualifier2[1] = qualified1.Qualifier();
                                _pnttg1sol[2] = p1;
                                _pnttg2sol[2] = new Point2D(c2.Location.XY() + (new Xy(-r2 * d2.Y, r2 * d2.X)));
                                _nbrSol = 2;
                            }
                        }
                    }
                    else if ((qualified1.IsUnqualified && qualified2.IsOutside) ||
                             //       ==================================================================
                             (qualified2.IsUnqualified && qualified1.IsOutside))
                    {
                        //           =======================================================
                        if (qualified2.IsUnqualified)
                        {
                            signe = 1;
                        }
                        else
                        {
                            signe = -1;
                        }
                        if (Math.Abs(dist + Min(r1, r2) - Max(r1, r2)) <= tol && dist >= tol)
                        {
                            if (r1 < r2)
                            {
                                d1 = new Direction2D(new Vector2D(c2.Location, c1.Location));
                            }
                            else
                            {
                                d1 = new Direction2D(new Vector2D(c1.Location, c2.Location));
                            }
                            _linsol[1] = new Line2D(new Point2D(c1.Location.XY() + r1 * d1.XY()),
                                                     //           =============================================================
                                                     new Direction2D(d1.Y, -d1.X));
                            //                                =========================
                            _qualifier1[1] = qualified1.Qualifier();
                            _qualifier2[1] = qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                            _pnttg1sol[1] = new Point2D(c1.Location.XY() + r1 * d1.XY());
                            _pnttg2sol[1] = _pnttg1sol[1];
                            _wellDone = true;
                            _nbrSol = 1;
                        }
                        else
                        {
                            Point2D p1 = c2.Location.Rotated(c1.Location, Math.Asin(signe * (r2 - r1) / dist));
                            d1 = new Direction2D(new Vector2D(c1.Location, p1));
                            p1 = new Point2D(c1.Location.XY() + new Xy(-r1 * d1.Y, r1 * d1.X));
                            _linsol[1] = new Line2D(p1, d1);
                            //           ===========================
                            _qualifier1[1] = qualified1.Qualifier();
                            _qualifier2[1] = qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                            _pnttg1sol[1] = p1;
                            _pnttg2sol[1] =
                                new Point2D(c2.Location.XY() + signe * (new Xy(-r2 * d1.Y, r2 * d1.X)));
                            _wellDone = true;
                            _nbrSol = 1;
                            if (Min(r1, r2) + Max(r1, r2) < dist)
                            {
                                Point2D p2 =
                                    c2.Location.Rotated(c1.Location, Math.Asin(signe * (-r2 - r1) / dist));
                                Direction2D d2 = new(new Vector2D(c1.Location, p2));
                                p2 = new Point2D(c1.Location.XY() + signe * (new Xy(-r1 * d2.Y, r1 * d2.X)));
                                _linsol[2] = new Line2D(p2, d2);
                                //             ===========================
                                // OCCT: writes the first solution's qualifiers (the second's stay unset)
                                _qualifier1[1] = qualified1.Qualifier();
                                _qualifier2[1] = qualified1.Qualifier();
                                _pnttg1sol[2] = p2;
                                _pnttg2sol[2] =
                                    new Point2D(c2.Location.XY() + signe * (new Xy(r2 * d2.Y, -r2 * d2.X)));
                                _nbrSol = 2;
                            }
                        }
                    }
                }
            }
            else
            {
                for (int i = 1; i <= 2; i++)
                {
                    signe = -signe;
                    _nbrSol++;
                    Point2D p1 = c2.Location.Rotated(c1.Location, Math.Asin(signe * (r2 - r1) / dist));
                    d1 = new Direction2D(new Vector2D(c1.Location, p1));
                    p1 = new Point2D(c1.Location.XY() + signe * new Xy(-r1 * d1.Y, r1 * d1.X));
                    _linsol[_nbrSol] = new Line2D(p1, d1);
                    //       ===========================
                    _qualifier1[_nbrSol] = qualified1.Qualifier();
                    _qualifier2[_nbrSol] = qualified1.Qualifier();
                    _pnttg1sol[_nbrSol] = p1;
                    _pnttg2sol[_nbrSol] =
                        new Point2D(c2.Location.XY() + signe * (new Xy(-r2 * d1.Y, r2 * d1.X)));
                    _wellDone = true;
                    if (Min(r1, r2) + Max(r1, r2) < dist)
                    {
                        Point2D p2 = c2.Location.Rotated(c1.Location, Math.Asin(signe * (r2 + r1) / dist));
                        Direction2D d2 = new(new Vector2D(c1.Location, p2));
                        p2 = new Point2D(c1.Location.XY() + signe * (new Xy(r1 * d2.Y, -r1 * d2.X)));
                        _nbrSol++;
                        _linsol[_nbrSol] = new Line2D(p2, d2);
                        //         ================================
                        _qualifier1[_nbrSol] = qualified1.Qualifier();
                        _qualifier2[_nbrSol] = qualified1.Qualifier();
                        _pnttg1sol[_nbrSol] = p2;
                        _pnttg2sol[_nbrSol] =
                            new Point2D(c2.Location.XY() + signe * (new Xy(-r2 * d2.Y, r2 * d2.X)));
                    }
                }
            }
        }

        for (int i = 1; i <= _nbrSol; i++)
        {
            _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[i], _pnttg1sol[i]);
            _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[i], _pnttg2sol[i]);
            // circles of radius 0 put the tangency points at their centres
            _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[i]);
            _pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c2, _pnttg2sol[i]);
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of solutions (OCCT doesn't check IsDone here).</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }

    /// <summary>The solution number Index (OCCT doesn't check IsDone here).</summary>
    public Line2D ThisSolution(int index)
    {
        if (index > _nbrSol || index <= 0)
        {
            throw new OutOfRangeException();
        }
        return _linsol[index];
    }

    /// <summary>The qualifiers of the arguments for the solution number Index (see the remarks).</summary>
    public void WhichQualifier(int index, out Qualifier qualif1, out Qualifier qualif2)
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
            qualif2 = _qualifier2[index];
        }
    }

    /// <summary>The tangency point of the solution number Index with the first argument, and its parameters.</summary>
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

    /// <summary>The tangency point of the solution number Index with the second argument, and its parameters.</summary>
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
            parSol = _par2sol[index];
            parArg = _pararg2[index];
            pntSol = _pnttg2sol[index];
        }
    }
}
