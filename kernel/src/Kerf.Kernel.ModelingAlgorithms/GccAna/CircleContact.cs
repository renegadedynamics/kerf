// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// An argument a solution circle must touch: a qualified circle, a qualified line, or a point to pass through. It
/// decides from the geometry whether a candidate circle touches it as its qualifier asks, and where (Kerf decision
/// 17101: GccAna's solvers check their candidates and fill their tangency data through it, in place of OCCT's
/// per-case code, which got qualifiers, tangency points and filtering wrong).
/// </summary>
internal readonly struct CircleContact
{
    private enum Kind { Circle, Line, Point }

    private readonly Kind _kind;
    private readonly Circle2D _circle;
    private readonly Line2D _line;
    private readonly Point2D _point;
    private readonly Qualifier _qualifier;

    public CircleContact(in QualifiedCirc qualified)
    {
        _kind = Kind.Circle;
        _circle = qualified.Qualified();
        _qualifier = qualified.Qualifier();
    }

    public CircleContact(in QualifiedLin qualified)
    {
        _kind = Kind.Line;
        _line = qualified.Qualified();
        _qualifier = qualified.Qualifier();
    }

    public CircleContact(in Point2D point)
    {
        _kind = Kind.Point;
        _point = point;
        _qualifier = Qualifier.Noqualifier;
    }

    public bool IsPoint => _kind == Kind.Point;

    /// <summary>
    /// True if the circle touches the argument as its qualifier asks, within the tolerance: a circle from outside,
    /// enclosed in it or enclosing it; a line on its left (enclosed) or right (outside); a point on it. The qualifier
    /// the solution has is the one asked, or for an unqualified argument the one found (noqualifier for a point).
    /// </summary>
    public bool Touches(in Circle2D sol, double tolerance, out Qualifier qualifier)
    {
        qualifier = _qualifier;
        double r = sol.Radius;
        switch (_kind)
        {
            case Kind.Circle:
                {
                    double d = sol.Location.DistanceTo(_circle.Location);
                    double ra = _circle.Radius;
                    // the asked position first, then the nearest one for an unqualified circle
                    double outside = Math.Abs(d - r - ra);
                    double enclosed = Math.Abs(d + r - ra);
                    double enclosing = Math.Abs(d + ra - r);
                    switch (_qualifier)
                    {
                        case Qualifier.Outside:
                            return outside <= tolerance;
                        case Qualifier.Enclosed:
                            return enclosed <= tolerance;
                        case Qualifier.Enclosing:
                            return enclosing <= tolerance;
                    }
                    double best = Math.Min(outside, Math.Min(enclosed, enclosing));
                    qualifier = best == enclosed ? Qualifier.Enclosed : best == outside ? Qualifier.Outside : Qualifier.Enclosing;
                    return best <= tolerance;
                }
            case Kind.Line:
                {
                    double s = SignedDistance(sol.Location);
                    Qualifier side = s >= 0.0 ? Qualifier.Enclosed : Qualifier.Outside;
                    if (_qualifier == Qualifier.Unqualified)
                    {
                        qualifier = side;
                    }
                    return qualifier == side && Math.Abs(Math.Abs(s) - r) <= tolerance;
                }
            default:
                return Math.Abs(sol.Location.DistanceTo(_point) - r) <= tolerance;
        }
    }

    /// <summary>True if the circle is the argument circle itself.</summary>
    public bool IsSame(in Circle2D sol, double tolerance)
        => _kind == Kind.Circle && sol.Location.DistanceTo(_circle.Location) <= tolerance
           && Math.Abs(sol.Radius - _circle.Radius) <= tolerance;

    /// <summary>Where the circle touches the argument, given the qualifier it touches it with.</summary>
    public Point2D TouchPoint(in Circle2D sol, Qualifier qualifier)
    {
        Point2D c = sol.Location;
        switch (_kind)
        {
            case Kind.Circle:
                {
                    // along the line of centres: towards the argument's centre, away from it when enclosed
                    Xy u = _circle.Location.XY() - c.XY();
                    double d = u.Modulus();
                    if (d <= 0.0)
                    {
                        return ElCLib.Value(0.0, sol);
                    }
                    double r = qualifier == Qualifier.Enclosed ? -sol.Radius : sol.Radius;
                    return new Point2D(c.XY() + (r / d) * u);
                }
            case Kind.Line:
                {
                    Xy n = new(-_line.Direction.Y, _line.Direction.X);
                    return new Point2D(c.XY() - SignedDistance(c) * n);
                }
            default:
                return _point;
        }
    }

    /// <summary>
    /// The radii of the circles centred at a point that touch the argument: d + r and |d - r| for a circle whose centre
    /// is at d, the distance for a line or a point. Returns how many it wrote (positive radii only).
    /// </summary>
    public int Radii(in Point2D center, Span<double> radii)
    {
        int n = 0;
        switch (_kind)
        {
            case Kind.Circle:
                {
                    double d = center.DistanceTo(_circle.Location);
                    radii[n++] = d + _circle.Radius;
                    if (Math.Abs(d - _circle.Radius) > 0.0)
                    {
                        radii[n++] = Math.Abs(d - _circle.Radius);
                    }
                    break;
                }
            case Kind.Line:
                radii[n++] = Math.Abs(SignedDistance(center));
                break;
            default:
                radii[n++] = center.DistanceTo(_point);
                break;
        }
        return n;
    }

    /// <summary>
    /// Writes the tangency data of solution n with the argument (the point where it touches, its parameters on both)
    /// and returns 0; returns 1 and writes nothing if the solution is the argument itself.
    /// </summary>
    public int Touch(int n, in Circle2D sol, Qualifier qualifier, double tolerance,
                     Kerf.Engine.NCollection_Array1<Point2D> pnt, Kerf.Engine.NCollection_Array1<double> parSol, Kerf.Engine.NCollection_Array1<double> parArg)
    {
        if (IsSame(sol, tolerance))
        {
            return 1;
        }
        Point2D p = TouchPoint(sol, qualifier);
        pnt[n] = p;
        parSol[n] = ElCLib.Parameter(sol, p);
        parArg[n] = Parameter(p);
        return 0;
    }

    /// <summary>The parameter of a point on the argument (0 for a point, and at the centre of a circle of radius 0).</summary>
    public double Parameter(in Point2D p) => _kind switch
    {
        Kind.Circle => p.DistanceTo(_circle.Location) <= Constants.Resolution() ? 0.0 : ElCLib.Parameter(_circle, p),
        Kind.Line => ElCLib.Parameter(_line, p),
        _ => 0.0,
    };

    private double SignedDistance(in Point2D p) => _line.Direction.XY() ^ (p.XY() - _line.Location.XY());
}
