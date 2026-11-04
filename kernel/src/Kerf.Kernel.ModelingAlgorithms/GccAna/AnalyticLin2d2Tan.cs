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
/// An algorithm object computed by its constructor (decision 117). WhichQualifier gives the side each circle is on
/// (Kerf decision 1544).
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
        if (!(qualified1.IsEnclosing || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        // The tangents from the point, each oriented from its tangency point to the point, kept on the side the
        // qualifier asks, with the side as the computed qualifier (OCCT gave an unqualified circle no computed
        // qualifier: Kerf decision 1544)
        Circle2D c1 = qualified1.Qualified();
        _wellDone = true;
        Span<Line2D> lines = stackalloc Line2D[2];
        Span<Point2D> touches = stackalloc Point2D[2];
        int nb = Tangents(c1, 1.0, new Circle2D(new Axis2D(thePoint, new Direction2D(Direction2D.D.X)), 0.0), 1.0, tol, lines, touches, touches);
        for (int i = 0; i < nb; i++)
        {
            Point2D p1 = touches[i];
            Line2D sol = p1.DistanceTo(thePoint) > tol ? new Line2D(p1, new Direction2D(thePoint.XY() - p1.XY())) : lines[i];
            Qualifier side = Side(sol, c1);
            if (!qualified1.IsUnqualified && qualified1.Qualifier() != side)
            {
                continue;
            }
            int n = ++_nbrSol;
            _linsol[n] = sol;
            _qualifier1[n] = side;
            _qualifier2[n] = Qualifier.Noqualifier;
            _pnttg1sol[n] = p1;
            _pnttg2sol[n] = thePoint;
            _par1sol[n] = global::Kerf.Engine.Geometry.ElCLib.Parameter(sol, p1);
            _par2sol[n] = global::Kerf.Engine.Geometry.ElCLib.Parameter(sol, thePoint);
            _pararg1[n] = ParameterOn(c1, p1);
            _pararg2[n] = 0.0;
        }
    }

    /// <summary>The side of a line a circle is on, as GccEnt names it: enclosing on its left, outside on its right.</summary>
    private static Qualifier Side(in Line2D sol, in Circle2D c)
        => (sol.Direction.XY() ^ (c.Location.XY() - sol.Location.XY())) >= 0.0 ? Qualifier.Enclosing : Qualifier.Outside;

    /// <summary>The parameter of a point on a circle (0 at the centre of a circle of radius 0).</summary>
    private static double ParameterOn(in Circle2D c, in Point2D p)
        => p.DistanceTo(c.Location) <= Constants.Resolution() ? 0.0 : global::Kerf.Engine.Geometry.ElCLib.Parameter(c, p);

    /// <summary>
    /// The oriented lines with circle 1 on the side s1 (+1 left, -1 right) and circle 2 on the side s2 at their radii:
    /// with n the line's left normal, n.(c2 - c1) = s2 r2 - s1 r1, solved for n's angle (one line where the two
    /// solutions meet within the tolerance). Writes the lines and their tangency points; returns how many.
    /// </summary>
    private static int Tangents(in Circle2D c1, double s1, in Circle2D c2, double s2, double tol,
                                Span<Line2D> lines, Span<Point2D> touch1, Span<Point2D> touch2)
    {
        Xy v = c2.Location.XY() - c1.Location.XY();
        double dist = v.Modulus();
        if (dist <= tol)
        {
            return 0;
        }
        double k = s2 * c2.Radius - s1 * c1.Radius;
        if (Math.Abs(k) > dist + tol)
        {
            return 0;
        }
        double phi = Math.Atan2(v.Y(), v.X());
        double alpha = Math.Acos(Math.Clamp(k / dist, -1.0, 1.0));
        int nb = dist - Math.Abs(k) <= tol ? 1 : 2;
        for (int i = 0; i < nb; i++)
        {
            double theta = i == 0 ? phi + alpha : phi - alpha;
            Xy n = new(Math.Cos(theta), Math.Sin(theta));
            Point2D p1 = new(c1.Location.XY() - (s1 * c1.Radius) * n);
            touch2[i] = new Point2D(c2.Location.XY() - (s2 * c2.Radius) * n);
            touch1[i] = p1;
            lines[i] = new Line2D(p1, new Direction2D(n.Y(), -n.X()));
        }
        return nb;
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
        if (!(qualified1.IsEnclosing || qualified1.IsOutside || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosing || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        // The common tangents, each oriented from its tangency point on C1 to that on C2, kept when the circles are on
        // the sides their qualifiers ask (so each pair of qualifiers picks one of the four tangents), with the sides as
        // the computed qualifiers (OCCT gave the second circle the first's qualifier, unqualified circles none, and
        // solution 2 of some branches solution 1's tangency point: Kerf decision 1544)
        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();
        _wellDone = true;
        // the outer tangents (both circles on one side) and the inner ones, taken in OCCT's order: an outer one, the
        // inner one on its other side, the other outer one, the other inner one
        Span<Line2D> lines = stackalloc Line2D[4];
        Span<Point2D> touch1 = stackalloc Point2D[4];
        Span<Point2D> touch2 = stackalloc Point2D[4];
        int nbOuter = Tangents(c1, 1.0, c2, 1.0, tol, lines[..2], touch1[..2], touch2[..2]);
        int nbInner = Tangents(c1, 1.0, c2, -1.0, tol, lines[2..], touch1[2..], touch2[2..]);
        Span<int> order = stackalloc int[4];
        int nbOrder = 0;
        for (int i = 0; i < 2; i++)
        {
            if (i < nbOuter)
            {
                order[nbOrder++] = i;
            }
            if (nbInner == 2 || (nbInner == 1 && i == 0))
            {
                order[nbOrder++] = 2 + (nbInner == 2 ? 1 - i : 0);
            }
        }
        Span<Line2D> candidates = stackalloc Line2D[2];
        foreach (int i in order[..nbOrder])
        {
            {
                Point2D p1 = touch1[i];
                Point2D p2 = touch2[i];
                int nc = 1;
                if (p1.DistanceTo(p2) > tol)
                {
                    candidates[0] = new Line2D(p1, new Direction2D(p2.XY() - p1.XY()));
                }
                else
                {
                    // the circles touch there: either sense
                    candidates[0] = lines[i];
                    candidates[1] = lines[i].Reversed();
                    nc = 2;
                }
                for (int j = 0; j < nc && _nbrSol < _linsol.Upper(); j++)
                {
                    Line2D sol = candidates[j];
                    Qualifier side1 = Side(sol, c1);
                    Qualifier side2 = Side(sol, c2);
                    if ((!qualified1.IsUnqualified && side1 != qualified1.Qualifier() && c1.Radius > tol)
                        || (!qualified2.IsUnqualified && side2 != qualified2.Qualifier() && c2.Radius > tol)
                        || IsSolution(sol, tol))
                    {
                        continue;
                    }
                    int n = ++_nbrSol;
                    _linsol[n] = sol;
                    _qualifier1[n] = c1.Radius > tol || qualified1.IsUnqualified ? side1 : qualified1.Qualifier();
                    _qualifier2[n] = c2.Radius > tol || qualified2.IsUnqualified ? side2 : qualified2.Qualifier();
                    _pnttg1sol[n] = p1;
                    _pnttg2sol[n] = p2;
                    _par1sol[n] = global::Kerf.Engine.Geometry.ElCLib.Parameter(sol, p1);
                    _par2sol[n] = global::Kerf.Engine.Geometry.ElCLib.Parameter(sol, p2);
                    _pararg1[n] = ParameterOn(c1, p1);
                    _pararg2[n] = ParameterOn(c2, p2);
                }
            }
        }
    }

    /// <summary>True if a line is a solution already (whatever its sense).</summary>
    private bool IsSolution(in Line2D sol, double tol)
    {
        for (int i = 1; i <= _nbrSol; i++)
        {
            if (Math.Abs(_linsol[i].Direction.XY() ^ sol.Direction.XY()) <= tol && _linsol[i].DistanceTo(sol.Location) <= tol)
            {
                return true;
            }
        }
        return false;
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
