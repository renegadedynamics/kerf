// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Meshing.Tools;

namespace Kerf.Geometry;

/// <summary>
/// How a constrained circle or line lies against a curve it touches (OCCT's <c>GccEnt_Position</c>): for a circle
/// target, the solution encloses it, is enclosed by it or is outside it; for a line, it is on the side the line's
/// direction leaves on its right (outside) or left (enclosed).
/// </summary>
public enum Tangency
{
    /// <summary>Any position: every solution.</summary>
    Unqualified,

    /// <summary>The solution encloses the target (a circle; not a line).</summary>
    Enclosing,

    /// <summary>The solution is enclosed by the target (inside a circle, or on a line's left).</summary>
    Enclosed,

    /// <summary>The solution is outside the target (outside a circle, or on a line's right).</summary>
    Outside,
}

/// <summary>
/// What a constrained circle or line touches: a 2D curve with a <see cref="Tangency"/>, or a point it passes through.
/// </summary>
public readonly record struct TangentTarget2D
{
    private TangentTarget2D(Curve2D? curve, Point2D point, Tangency tangency, Interval? range = null)
    {
        Curve = curve;
        Point = point;
        Tangency = tangency;
        Range = range;
    }

    /// <summary>The curve touched, or null for a point.</summary>
    public Curve2D? Curve { get; }

    /// <summary>The point passed through, when <see cref="Curve"/> is null.</summary>
    public Point2D Point { get; }

    /// <summary>The position asked of the solution against the curve (<see cref="Tangency.Unqualified"/> for a point).</summary>
    public Tangency Tangency { get; }

    /// <summary>The stretch of the curve that counts, or null for the whole curve (its own range).</summary>
    public Interval? Range { get; }

    /// <summary>True for a point, false for a curve.</summary>
    public bool IsPoint => Curve is null;

    /// <summary>The curve, touched in the given position.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="curve"/> is null.</exception>
    public static TangentTarget2D Tangent(Curve2D curve, Tangency tangency = Tangency.Unqualified)
    {
        ArgumentNullException.ThrowIfNull(curve);
        if (!Enum.IsDefined(tangency))
        {
            throw new ArgumentOutOfRangeException(nameof(tangency), tangency, "An unknown tangency.");
        }

        return new TangentTarget2D(curve, default, tangency);
    }

    /// <summary>The stretch <paramref name="range"/> of the curve (an edge's range on its curve), touched in the given
    /// position: an iterative solve (a curve other than a line or a circle) is bounded by it and starts in its middle.
    /// Lines and circles are solved whole, so a caller that wants the stretch checks the contacts' parameters.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="curve"/> is null.</exception>
    /// <exception cref="ArgumentException"><paramref name="range"/> is empty or not a number.</exception>
    public static TangentTarget2D Tangent(Curve2D curve, Interval range, Tangency tangency = Tangency.Unqualified)
    {
        TangentTarget2D whole = Tangent(curve, tangency);
        if (!(range.Min < range.Max))
        {
            throw new ArgumentException("The range must run from a lower parameter to a higher one.", nameof(range));
        }

        return new TangentTarget2D(whole.Curve, default, tangency, range);
    }

    /// <summary>The point, passed through.</summary>
    public static TangentTarget2D Through(Point2D point) => new(null, point, Tangency.Unqualified);

    /// <summary>A curve touched in any position.</summary>
    public static implicit operator TangentTarget2D(Curve2D curve) => Tangent(curve);

    /// <summary>A point passed through.</summary>
    public static implicit operator TangentTarget2D(Point2D point) => Through(point);
}

/// <summary>Where a solution meets one of its targets.</summary>
/// <param name="Tangency">The solution's position against the target (<see cref="Tangency.Unqualified"/> for a point).</param>
/// <param name="Point">The point of contact (the point itself, for a point target).</param>
/// <param name="SolutionParameter">The contact's parameter on the solution.</param>
/// <param name="TargetParameter">The contact's parameter on the target curve (0 for a point).</param>
/// <param name="IsCoincident">True when the solution is the target itself (a circle target of the radius asked): there is no
/// single point of contact, and the other members are NaN.</param>
public readonly record struct TangentContact2D(Tangency Tangency, Point2D Point, double SolutionParameter, double TargetParameter,
                                               bool IsCoincident = false);

/// <summary>A constrained circle, with where it meets each target, in the order the targets were given.</summary>
public sealed record TangentCircle2D(Circle2D Circle, ImmutableArray<TangentContact2D> Contacts);

/// <summary>A constrained line, with where it meets each target, in the order the targets were given.</summary>
public sealed record TangentLine2D(Line2D Line, ImmutableArray<TangentContact2D> Contacts);

/// <summary>
/// Circles and lines constrained by 2D curves and points (docs/api/geometry.md; OCCT's <c>Geom2dGcc</c>, which solves
/// lines and circles exactly through <c>GccAna</c> and other curves iteratively): tangent to curves, through points, of
/// a radius, about a centre. Each returns every solution found, with its points of contact; none is an empty list, and
/// a solver that fails is a failed result (<c>geometry.tangents-failed</c>).
/// </summary>
public static class Tangents2D
{
    /// <summary>A solver that failed: <c>geometry.tangents-failed</c>.</summary>
    public static DiagnosticCode TangentsFailed { get; } = new("geometry.tangents-failed");

    /// <summary>The lines tangent to (or through) two targets, at least one of them a curve (OCCT's
    /// <c>Geom2dGcc_Lin2d2Tan</c>).</summary>
    /// <param name="first">The first target.</param>
    /// <param name="second">The second target.</param>
    /// <param name="angularTolerance">The angular tolerance of the solver.</param>
    /// <exception cref="ArgumentException">Two points (a line through two points isn't a tangent), or a line target
    /// qualified <see cref="Tangency.Enclosing"/>.</exception>
    public static Result<ImmutableArray<TangentLine2D>> Lines(TangentTarget2D first, TangentTarget2D second,
                                                             double angularTolerance = Tolerances.Angular)
    {
        Check(first, nameof(first));
        Check(second, nameof(second));
        if (first.IsPoint && second.IsPoint)
        {
            throw new ArgumentException("A tangent line needs a curve to touch, not two points.", nameof(second));
        }

        (TangentTarget2D[] ordered, int[] order) = CurvesFirst([first, second]);
        return Solve(() =>
        {
            Lin2d2Tan solver = ordered[1].IsPoint
                ? new Lin2d2Tan(Qualified(ordered[0]), ordered[1].Point, angularTolerance)
                : new Lin2d2Tan(Qualified(ordered[0]), Qualified(ordered[1]), angularTolerance);
            return Lines(solver.IsDone, solver.SolutionCount, solver.ThisSolution, order, i =>
            {
                solver.WhichQualifier(i, out Qualifier q1, out Qualifier q2);
                return [q1, q2];
            }, (i, k) => k == 0 ? Contact(solver.Tangency1, i) : Contact(solver.Tangency2, i), ordered);
        });
    }

    /// <summary>The circles of <paramref name="radius"/> tangent to (or through) two targets (OCCT's
    /// <c>Geom2dGcc_Circ2d2TanRad</c>).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="radius"/> isn't positive.</exception>
    /// <exception cref="ArgumentException">A line target qualified <see cref="Tangency.Enclosing"/>.</exception>
    public static Result<ImmutableArray<TangentCircle2D>> Circles(TangentTarget2D first, TangentTarget2D second, double radius,
                                                                 double tolerance = Tolerances.Confusion)
    {
        Check(first, nameof(first));
        Check(second, nameof(second));
        MakerOutcome.Positive(radius, nameof(radius));
        (TangentTarget2D[] ordered, int[] order) = CurvesFirst([first, second]);
        return Solve(() =>
        {
            Circ2d2TanRad solver = (ordered[0].IsPoint, ordered[1].IsPoint) switch
            {
                (false, false) => new Circ2d2TanRad(Qualified(ordered[0]), Qualified(ordered[1]), radius, tolerance),
                (false, true) => new Circ2d2TanRad(Qualified(ordered[0]), Point(ordered[1]), radius, tolerance),
                _ => new Circ2d2TanRad(Point(ordered[0]), Point(ordered[1]), radius, tolerance),
            };
            return Circles(solver.IsDone, solver.SolutionCount, solver.ThisSolution, order, i =>
            {
                solver.WhichQualifier(i, out Qualifier q1, out Qualifier q2);
                return [q1, q2];
            }, (i, k) => k == 0 ? Contact(solver.Tangency1, i, solver.IsTheSame1(i)) : Contact(solver.Tangency2, i, solver.IsTheSame2(i)),
            ordered);
        });
    }

    /// <summary>The circles tangent to (or through) three targets (OCCT's <c>Geom2dGcc_Circ2d3Tan</c>). Curves other
    /// than lines and circles are solved iteratively from the middle of their ranges.</summary>
    /// <exception cref="ArgumentException">A line target qualified <see cref="Tangency.Enclosing"/>.</exception>
    public static Result<ImmutableArray<TangentCircle2D>> Circles(TangentTarget2D first, TangentTarget2D second, TangentTarget2D third,
                                                                 double tolerance = Tolerances.Confusion)
    {
        Check(first, nameof(first));
        Check(second, nameof(second));
        Check(third, nameof(third));
        (TangentTarget2D[] o, int[] order) = CurvesFirst([first, second, third]);
        return Solve(() =>
        {
            Circ2d3Tan solver = (o[0].IsPoint, o[1].IsPoint, o[2].IsPoint) switch
            {
                (false, false, false) => new Circ2d3Tan(Qualified(o[0]), Qualified(o[1]), Qualified(o[2]), tolerance,
                                                        Middle(o[0]), Middle(o[1]), Middle(o[2])),
                (false, false, true) => new Circ2d3Tan(Qualified(o[0]), Qualified(o[1]), Point(o[2]), tolerance, Middle(o[0]), Middle(o[1])),
                (false, true, true) => new Circ2d3Tan(Qualified(o[0]), Point(o[1]), Point(o[2]), tolerance, Middle(o[0])),
                _ => new Circ2d3Tan(Point(o[0]), Point(o[1]), Point(o[2]), tolerance),
            };
            return Circles(solver.IsDone, solver.SolutionCount, solver.ThisSolution, order, i =>
            {
                solver.WhichQualifier(i, out Qualifier q1, out Qualifier q2, out Qualifier q3);
                return [q1, q2, q3];
            }, (i, k) => k switch
            {
                0 => Contact(solver.Tangency1, i, solver.IsTheSame1(i)),
                1 => Contact(solver.Tangency2, i, solver.IsTheSame2(i)),
                _ => Contact(solver.Tangency3, i, solver.IsTheSame3(i)),
            }, o);
        });
    }

    /// <summary>The circles centred on <paramref name="centre"/> tangent to a curve (OCCT's <c>Geom2dGcc_Circ2dTanCen</c>).</summary>
    /// <exception cref="ArgumentException"><paramref name="target"/> is a point, or a line qualified
    /// <see cref="Tangency.Enclosing"/>.</exception>
    public static Result<ImmutableArray<TangentCircle2D>> CirclesCentredAt(TangentTarget2D target, Point2D centre,
                                                                          double tolerance = Tolerances.Confusion)
    {
        Check(target, nameof(target));
        if (target.IsPoint)
        {
            throw new ArgumentException("A circle about a centre is tangent to a curve; a point gives one circle, made directly.", nameof(target));
        }

        TangentTarget2D[] ordered = [target];
        return Solve(() =>
        {
            var solver = new Circ2dTanCen(Qualified(target), new CartesianPoint2(centre), tolerance);
            return Circles(solver.IsDone, solver.SolutionCount, solver.ThisSolution, [0], i =>
            {
                solver.WhichQualifier(i, out Qualifier q1);
                return [q1];
            }, (i, _) => Contact(solver.Tangency1, i, solver.IsTheSame1(i)), ordered);
        });
    }

    /// <summary>The circles tangent to (or through) two targets with their centre on <paramref name="locus"/> (OCCT's
    /// <c>Geom2dGcc_Circ2d2TanOn</c>).</summary>
    /// <param name="first">The first target.</param>
    /// <param name="second">The second target.</param>
    /// <param name="locus">The curve the centres lie on.</param>
    /// <param name="locusRange">The stretch of <paramref name="locus"/> an iterative solve searches, or null for its own
    /// range (as <see cref="TangentTarget2D.Tangent(Curve2D, Interval, Tangency)"/>: a line or circle is solved whole).</param>
    /// <param name="tolerance">The solver's tolerance.</param>
    /// <exception cref="ArgumentNullException"><paramref name="locus"/> is null.</exception>
    /// <exception cref="ArgumentException">A line target qualified <see cref="Tangency.Enclosing"/>.</exception>
    public static Result<ImmutableArray<TangentCircle2D>> CirclesCentredOn(TangentTarget2D first, TangentTarget2D second, Curve2D locus,
                                                                          Interval? locusRange = null,
                                                                          double tolerance = Tolerances.Confusion)
    {
        Check(first, nameof(first));
        Check(second, nameof(second));
        TangentTarget2D on = locusRange is { } range ? TangentTarget2D.Tangent(locus, range) : TangentTarget2D.Tangent(locus);
        (TangentTarget2D[] o, int[] order) = CurvesFirst([first, second]);
        return Solve(() =>
        {
            CurveAdaptor2D onCurve = Adaptor(on);
            Circ2d2TanOn solver = (o[0].IsPoint, o[1].IsPoint) switch
            {
                (false, false) => new Circ2d2TanOn(Qualified(o[0]), Qualified(o[1]), onCurve, tolerance, Middle(o[0]), Middle(o[1]), Middle(on)),
                (false, true) => new Circ2d2TanOn(Qualified(o[0]), Point(o[1]), onCurve, tolerance, Middle(o[0]), Middle(on)),
                _ => new Circ2d2TanOn(Point(o[0]), Point(o[1]), onCurve, tolerance),
            };
            return Circles(solver.IsDone, solver.SolutionCount, solver.ThisSolution, order, i =>
            {
                solver.WhichQualifier(i, out Qualifier q1, out Qualifier q2);
                return [q1, q2];
            }, (i, k) => k == 0 ? Contact(solver.Tangency1, i, solver.IsTheSame1(i)) : Contact(solver.Tangency2, i, solver.IsTheSame2(i)),
            o);
        });
    }

    /// <summary>The circles of <paramref name="radius"/> tangent to (or through) a target with their centre on
    /// <paramref name="locus"/> (OCCT's <c>Geom2dGcc_Circ2dTanOnRad</c>).</summary>
    /// <param name="target">The target.</param>
    /// <param name="locus">The curve the centres lie on.</param>
    /// <param name="radius">The circles' radius.</param>
    /// <param name="locusRange">The stretch of <paramref name="locus"/> an iterative solve searches, or null for its own
    /// range (a line or circle is solved whole).</param>
    /// <param name="tolerance">The solver's tolerance.</param>
    /// <exception cref="ArgumentNullException"><paramref name="locus"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="radius"/> isn't positive.</exception>
    /// <exception cref="ArgumentException">A line target qualified <see cref="Tangency.Enclosing"/>.</exception>
    public static Result<ImmutableArray<TangentCircle2D>> CirclesCentredOn(TangentTarget2D target, Curve2D locus, double radius,
                                                                          Interval? locusRange = null,
                                                                          double tolerance = Tolerances.Confusion)
    {
        Check(target, nameof(target));
        TangentTarget2D on = locusRange is { } range ? TangentTarget2D.Tangent(locus, range) : TangentTarget2D.Tangent(locus);
        MakerOutcome.Positive(radius, nameof(radius));
        TangentTarget2D[] ordered = [target];
        return Solve(() =>
        {
            CurveAdaptor2D onCurve = Adaptor(on);
            Circ2dTanOnRad solver = target.IsPoint
                ? new Circ2dTanOnRad(Point(target), onCurve, radius, tolerance)
                : new Circ2dTanOnRad(Qualified(target), onCurve, radius, tolerance);
            return Circles(solver.IsDone, solver.SolutionCount, solver.ThisSolution, [0], i =>
            {
                solver.WhichQualifier(i, out Qualifier q1);
                return [q1];
            }, (i, _) => Contact(solver.Tangency1, i, solver.IsTheSame1(i)), ordered);
        });
    }

    /// <summary>
    /// The lines tangent to a curve at <paramref name="angle"/> (radians, anticlockwise) to <paramref name="reference"/>
    /// (OCCT's <c>Geom2dGcc_Lin2dTanObl</c>). Each line's first contact is its tangency; its second is where it crosses
    /// the reference, NaN throughout where it runs parallel to it.
    /// </summary>
    /// <param name="target">The curve touched.</param>
    /// <param name="reference">The line the angle is measured from.</param>
    /// <param name="angle">The angle, in radians.</param>
    /// <param name="angularTolerance">The angular tolerance of the solver.</param>
    /// <exception cref="ArgumentException"><paramref name="target"/> is a point, or a line qualified
    /// <see cref="Tangency.Enclosing"/>.</exception>
    public static Result<ImmutableArray<TangentLine2D>> LinesAtAngle(TangentTarget2D target, Line2D reference, double angle,
                                                                    double angularTolerance = Tolerances.Angular)
    {
        Check(target, nameof(target));
        if (target.IsPoint)
        {
            throw new ArgumentException("A line at an angle through a point is a single line, made directly; this needs a curve to touch.", nameof(target));
        }

        if (!double.IsFinite(angle))
        {
            throw new ArgumentOutOfRangeException(nameof(angle), angle, "The angle must be a finite number.");
        }

        return Solve(() =>
        {
            var solver = new Lin2dTanObl(Qualified(target), reference, angularTolerance, angle);
            if (!solver.IsDone)
            {
                return Result<ImmutableArray<TangentLine2D>>.Failure(Diagnostic.Error(TangentsFailed, "The constrained lines couldn't be computed."));
            }

            var lines = ImmutableArray.CreateBuilder<TangentLine2D>(solver.SolutionCount);
            for (int i = 1; i <= solver.SolutionCount; i++)
            {
                solver.WhichQualifier(i, out Qualifier qualifier);
                solver.Tangency1(i, out double parSol, out double parArg, out Point2D point);
                var tangency = new TangentContact2D(ToTangency(qualifier), point, parSol, parArg);
                TangentContact2D crossing;
                try
                {
                    solver.Intersection2(i, out double crossSol, out double crossArg, out Point2D crossPoint);
                    crossing = new TangentContact2D(Tangency.Unqualified, crossPoint, crossSol, crossArg);
                }
                catch (IsParallelException)
                {
                    crossing = new TangentContact2D(Tangency.Unqualified, new Point2D(double.NaN, double.NaN), double.NaN, double.NaN);
                }

                lines.Add(new TangentLine2D(solver.ThisSolution(i), [tangency, crossing]));
            }

            return lines.MoveToImmutable();
        });
    }

    // ------------------------------------------------------------------------------------------------ the plumbing

    private delegate void TangencyReader(int index, out double parSol, out double parArg, out Point2D point);

    private static void Check(in TangentTarget2D target, string name)
    {
        if (target is { IsPoint: true, Point: var p } && (double.IsNaN(p.X) || double.IsNaN(p.Y)))
        {
            throw new ArgumentException("The point isn't a number.", name);
        }

        if (target.Curve is LineCurve2D && target.Tangency == Tangency.Enclosing)
        {
            throw new ArgumentException("A line can't be enclosed: qualify it Enclosed, Outside or Unqualified.", name);
        }
    }

    // OCCT's solvers take their curves before their points
    private static (TangentTarget2D[] Ordered, int[] Order) CurvesFirst(TangentTarget2D[] targets)
    {
        int[] order = [.. Enumerable.Range(0, targets.Length).OrderBy(i => targets[i].IsPoint ? 1 : 0)];
        return ([.. order.Select(i => targets[i])], order);
    }

    private static CurveAdaptor2D Adaptor(in TangentTarget2D target) =>
        target.Range is { } range ? new CurveAdaptor2D(target.Curve!, range.Min, range.Max) : new CurveAdaptor2D(target.Curve!);

    private static QualifiedCurve Qualified(in TangentTarget2D target) =>
        new(Adaptor(target), target.Tangency switch
        {
            Tangency.Enclosing => Qualifier.Enclosing,
            Tangency.Enclosed => Qualifier.Enclosed,
            Tangency.Outside => Qualifier.Outside,
            _ => Qualifier.Unqualified,
        });

    private static CartesianPoint2 Point(in TangentTarget2D target) => new(target.Point);

    // The starting parameter of an iterative solve: the middle of a bounded curve, 0 on an unbounded one
    private static double Middle(in TangentTarget2D target)
    {
        (double first, double last) = target.Range is { } range ? (range.Min, range.Max) : (target.Curve!.FirstParameter, target.Curve.LastParameter);
        return double.IsInfinity(first) || double.IsInfinity(last) ? 0 : (last - first) / 2 + first;   // as build123d computes it, to the last bit
    }

    private static Tangency ToTangency(Qualifier qualifier) => qualifier switch
    {
        Qualifier.Enclosing => Tangency.Enclosing,
        Qualifier.Enclosed => Tangency.Enclosed,
        Qualifier.Outside => Tangency.Outside,
        _ => Tangency.Unqualified,
    };

    private static (double ParSol, double ParArg, Point2D Point)? Contact(TangencyReader read, int index, bool isSame = false)
    {
        if (isSame)
        {
            return null;
        }

        read(index, out double parSol, out double parArg, out Point2D point);
        return (parSol, parArg, point);
    }

    private static ImmutableArray<TangentContact2D> Contacts(int index, int[] order, Func<int, Qualifier[]> qualifiers,
                                                             Func<int, int, (double ParSol, double ParArg, Point2D Point)?> contact,
                                                             TangentTarget2D[] ordered)
    {
        Qualifier[] q = qualifiers(index);
        var contacts = new TangentContact2D[order.Length];
        for (int k = 0; k < order.Length; k++)
        {
            Tangency tangency = ordered[k].IsPoint ? Tangency.Unqualified : ToTangency(q[k]);
            contacts[order[k]] = contact(index, k) is { } c
                ? new TangentContact2D(tangency, c.Point, c.ParSol, ordered[k].IsPoint ? 0 : c.ParArg)
                : new TangentContact2D(tangency, new Point2D(double.NaN, double.NaN), double.NaN, double.NaN, IsCoincident: true);
        }

        return [.. contacts];
    }

    private static Result<ImmutableArray<TangentCircle2D>> Circles(bool isDone, int count, Func<int, Circle2D> solution, int[] order,
                                                                  Func<int, Qualifier[]> qualifiers,
                                                                  Func<int, int, (double, double, Point2D)?> contact,
                                                                  TangentTarget2D[] ordered)
    {
        if (!isDone)
        {
            return Result<ImmutableArray<TangentCircle2D>>.Failure(Diagnostic.Error(TangentsFailed, "The constrained circles couldn't be computed."));
        }

        var circles = ImmutableArray.CreateBuilder<TangentCircle2D>(count);
        for (int i = 1; i <= count; i++)
        {
            circles.Add(new TangentCircle2D(solution(i), Contacts(i, order, qualifiers, contact, ordered)));
        }

        return circles.MoveToImmutable();
    }

    private static Result<ImmutableArray<TangentLine2D>> Lines(bool isDone, int count, Func<int, Line2D> solution, int[] order,
                                                              Func<int, Qualifier[]> qualifiers,
                                                              Func<int, int, (double, double, Point2D)?> contact,
                                                              TangentTarget2D[] ordered)
    {
        if (!isDone)
        {
            return Result<ImmutableArray<TangentLine2D>>.Failure(Diagnostic.Error(TangentsFailed, "The constrained lines couldn't be computed."));
        }

        var lines = ImmutableArray.CreateBuilder<TangentLine2D>(count);
        for (int i = 1; i <= count; i++)
        {
            lines.Add(new TangentLine2D(solution(i), Contacts(i, order, qualifiers, contact, ordered)));
        }

        return lines.MoveToImmutable();
    }

    // The engine's failures (and its exceptions-as-statuses) as a failed result
    private static Result<T> Solve<T>(Func<Result<T>> run)
    {
        try
        {
            return run();
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            return Result<T>.Failure(Diagnostic.Error(TangentsFailed, FailureDiagnostics.ToDiagnostic(e).Message));
        }
    }
}
