// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using System.Globalization;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The laws a 2D constraint solver's solutions obey, checked from the geometry alone (Kerf unit 10K4, replacing
/// native OCCT's recordings as the reference): every solution is tangent to its arguments (passes through its
/// points), is centred on its locus and has the radius asked; each qualifier asked is honoured and each computed one
/// is the solution's; each tangency point is on the solution and on the argument where the two touch, and its
/// parameters give it; IsTheSame holds exactly when the solution is the argument circle.
/// </summary>
internal static class GccLaws
{
    /// <summary>An argument of a solver: a qualified circle, a qualified line or a point.</summary>
    internal abstract record Arg(Qualifier Q);

    internal sealed record CircArg(Circle2D C, Qualifier Q) : Arg(Q)
    {
        public override string ToString() => FormattableString.Invariant(
            $"circle ({C.Location.X:R}, {C.Location.Y:R}) r {C.Radius:R}{(C.IsDirect ? "" : " indirect")} {Q}");
    }

    internal sealed record LinArg(Line2D L, Qualifier Q) : Arg(Q)
    {
        public override string ToString() => FormattableString.Invariant(
            $"line ({L.Location.X:R}, {L.Location.Y:R}) d ({L.Direction.X:R}, {L.Direction.Y:R}) {Q}");
    }

    internal sealed record PntArg(Point2D P) : Arg(Qualifier.Unqualified)
    {
        public override string ToString() => FormattableString.Invariant($"point ({P.X:R}, {P.Y:R})");
    }

    /// <summary>A locus the centre is on.</summary>
    internal abstract record Locus;

    internal sealed record OnLine(Line2D L) : Locus;

    internal sealed record OnCircle(Circle2D C) : Locus;

    /// <summary>A circle solver's results, read through delegates (the solvers share no base).</summary>
    internal sealed class Circles
    {
        public required Func<bool> IsDone { get; init; }
        public required Func<int> Count { get; init; }
        public required Func<int, Circle2D> Solution { get; init; }
        /// <summary>The qualifiers of the arguments for a solution, or null when the solver has no WhichQualifier.</summary>
        public Func<int, Qualifier[]>? Qualifiers { get; init; }
        /// <summary>The k-th tangency (k from 1) of a solution.</summary>
        public required Func<int, int, (double ParSol, double ParArg, Point2D P)> Tangency { get; init; }
        public Func<int, int, bool>? IsTheSame { get; init; }
        public Func<int, (double Par, Point2D P)>? CenterOn { get; init; }
    }

    /// <summary>A line solver's results.</summary>
    internal sealed class Lines
    {
        public required Func<bool> IsDone { get; init; }
        public required Func<int> Count { get; init; }
        public required Func<int, Line2D> Solution { get; init; }
        public Func<int, Qualifier[]>? Qualifiers { get; init; }
        public required Func<int, int, (double ParSol, double ParArg, Point2D P)> Tangency { get; init; }
        /// <summary>The intersection with the second curve (a line or circle the solution is oblique or perpendicular to).</summary>
        public Func<int, (double ParSol, double ParArg, Point2D P)>? Intersection { get; init; }
    }

    /// <summary>The side of a line a circle argument is on, as a qualifier: enclosing on the left, outside on the right.</summary>
    public static Qualifier SideOf(in Line2D theSol, in Circle2D theC) => SignedDistance(theSol, theC.Location) >= 0.0 ? Qualifier.Enclosing : Qualifier.Outside;

    /// <summary>True if a line touches an argument (a circle, as its qualifier asks, or a point it passes through).</summary>
    public static bool Touches(in Line2D theSol, Arg theArg, double theTol) => theArg switch
    {
        CircArg c => Math.Abs(Math.Abs(SignedDistance(theSol, c.C.Location)) - c.C.Radius) <= theTol
                     && (c.Q == Qualifier.Unqualified || c.Q == SideOf(theSol, c.C) || c.C.Radius <= theTol),
        PntArg p => Math.Abs(SignedDistance(theSol, p.P)) <= theTol,
        _ => false,
    };

    /// <summary>
    /// The laws the line solver's results break: each solution touches its arguments (circles with the qualifier asked,
    /// WhichQualifier the side the circle is on; points on it), the tangency points where it touches with their
    /// parameters, the direction law, and the intersection with the second curve: on both, at their parameters, and for
    /// a circle the intersection nearest the first tangency point. <paramref name="theSecond"/> is null when the solver
    /// has no second curve; <paramref name="isParallel"/> says when a solution has no intersection with it (its
    /// Intersection2 must then throw).
    /// </summary>
    public static List<string> CheckLines(Func<Lines> theSolver, IReadOnlyList<Arg> theArgs, Func<Line2D, string?>? theDirectionLaw,
                                          object? theSecond, double theTol)
    {
        List<string> aBroken = [];
        Lines S;
        int aNb;
        try
        {
            S = theSolver();
            if (!S.IsDone())
            {
                aBroken.Add("not done");
                return aBroken;
            }
            aNb = S.Count();
        }
        catch (BadQualifierException) when (theArgs.Any(a => a.Q == Qualifier.Noqualifier))
        {
            // a qualifier no argument may have (OCCT didn't check it everywhere)
            return aBroken;
        }
        catch (Exception e)
        {
            aBroken.Add("throws " + Refusal.Unwrapped(e).GetType().Name);
            return aBroken;
        }
        for (int i = 1; i <= aNb; i++)
        {
            try
            {
                CheckLine(S, i, theArgs, theDirectionLaw, theSecond, theTol, aBroken);
            }
            catch (Exception e)
            {
                aBroken.Add($"solution {i}: throws {Refusal.Unwrapped(e).GetType().Name}");
            }
        }
        return aBroken;
    }

    private static void CheckLine(Lines S, int i, IReadOnlyList<Arg> theArgs, Func<Line2D, string?>? theDirectionLaw, object? theSecond,
                                  double theTol, List<string> theBroken)
    {
        Line2D aSol = S.Solution(i);
        string aWhat = FormattableString.Invariant($"solution {i} ({aSol.Location.X:R}, {aSol.Location.Y:R}) d ({aSol.Direction.X:R}, {aSol.Direction.Y:R})");
        if (theDirectionLaw?.Invoke(aSol) is string aLaw)
        {
            theBroken.Add($"{aWhat}: {aLaw}");
        }
        Qualifier[]? aQuals = S.Qualifiers?.Invoke(i);
        Point2D? aFirst = null;
        for (int k = 0; k < theArgs.Count; k++)
        {
            Arg anArg = theArgs[k];
            if (anArg is CircArg c0 && Math.Abs(Math.Abs(SignedDistance(aSol, c0.C.Location)) - c0.C.Radius) > theTol
                || anArg is PntArg p0 && Math.Abs(SignedDistance(aSol, p0.P)) > theTol)
            {
                theBroken.Add($"{aWhat}: not tangent to argument {k + 1}");
                continue;
            }
            if (anArg is CircArg c1)
            {
                Qualifier aSide = SideOf(aSol, c1.C);
                if (c1.Q != Qualifier.Unqualified && c1.Q != aSide && c1.C.Radius > theTol)
                {
                    theBroken.Add($"{aWhat}: qualifier {c1.Q} of argument {k + 1} not honoured ({aSide})");
                }
                if (aQuals is not null && aQuals[k] != aSide && c1.C.Radius > theTol)
                {
                    theBroken.Add($"{aWhat}: WhichQualifier {aQuals[k]} for argument {k + 1}, not {aSide}");
                }
            }
            (double aParSol, double aParArg, Point2D aP) = S.Tangency(i, k + 1);
            Point2D anExpected = anArg is CircArg c2
                ? new Point2D(c2.C.Location.XY() - SignedDistance(aSol, c2.C.Location) * new Xy(-aSol.Direction.Y, aSol.Direction.X))
                : ((PntArg)anArg).P;
            aFirst ??= anExpected;
            if (!Near(aP, anExpected, theTol))
            {
                theBroken.Add($"{aWhat}: tangency point {k + 1} not where it touches");
            }
            if (!Near(ElCLib.Value(aParSol, aSol), aP, theTol))
            {
                theBroken.Add($"{aWhat}: tangency {k + 1}'s parameter on the solution");
            }
            if (anArg is CircArg c3 && c3.C.Radius > theTol && !Near(ElCLib.Value(aParArg, c3.C), aP, theTol))
            {
                theBroken.Add($"{aWhat}: tangency {k + 1}'s parameter on the argument");
            }
        }
        if (theSecond is null || S.Intersection is null)
        {
            return;
        }
        bool isParallel = theSecond is Line2D sl && Math.Abs(sl.Direction.XY() ^ aSol.Direction.XY()) <= 1.0e-12;
        (double ParSol, double ParArg, Point2D P) anI;
        try
        {
            anI = S.Intersection(i);
        }
        catch (Exception e) when (e.GetType().Name == "IsParallelException")
        {
            if (!isParallel)
            {
                theBroken.Add($"{aWhat}: Intersection2 says parallel");
            }
            return;
        }
        if (isParallel)
        {
            theBroken.Add($"{aWhat}: Intersection2 of a parallel solution");
            return;
        }
        if (Math.Abs(SignedDistance(aSol, anI.P)) > theTol || !Near(ElCLib.Value(anI.ParSol, aSol), anI.P, theTol))
        {
            theBroken.Add($"{aWhat}: intersection point off the solution or its parameter");
        }
        switch (theSecond)
        {
            case Line2D l:
                if (Math.Abs(SignedDistance(l, anI.P)) > theTol || !Near(ElCLib.Value(anI.ParArg, l), anI.P, theTol))
                {
                    theBroken.Add($"{aWhat}: intersection point off the second line or its parameter");
                }
                break;
            case Circle2D c:
                {
                    if (Math.Abs(anI.P.DistanceTo(c.Location) - c.Radius) > theTol || !Near(ElCLib.Value(anI.ParArg, c), anI.P, theTol))
                    {
                        theBroken.Add($"{aWhat}: intersection point off the second circle or its parameter");
                    }
                    // the nearer of the two intersections to the first tangency point
                    double s = SignedDistance(aSol, c.Location);
                    Point2D aFoot = new(c.Location.XY() - s * new Xy(-aSol.Direction.Y, aSol.Direction.X));
                    double h = Math.Sqrt(Math.Max(0.0, c.Radius * c.Radius - s * s));
                    Point2D aA = new(aFoot.XY() + h * aSol.Direction.XY());
                    Point2D aB = new(aFoot.XY() - h * aSol.Direction.XY());
                    Point2D T = aFirst ?? aSol.Location;
                    double aNear = Math.Min(T.DistanceTo(aA), T.DistanceTo(aB));
                    if (T.DistanceTo(anI.P) > aNear + theTol)
                    {
                        theBroken.Add($"{aWhat}: intersection not the nearest");
                    }
                    break;
                }
        }
    }

    /// <summary>The qualifiers a circle has with respect to an argument it touches (several when it is the argument).</summary>
    public static List<Qualifier> Fits(in Circle2D theSol, Arg theArg, double theTol)
    {
        List<Qualifier> aFits = [];
        double R = theSol.Radius;
        switch (theArg)
        {
            case CircArg c:
                {
                    double d = theSol.Location.DistanceTo(c.C.Location);
                    double r = c.C.Radius;
                    if (Math.Abs(d - R - r) <= theTol)
                    {
                        aFits.Add(Qualifier.Outside);
                    }
                    if (Math.Abs(d + R - r) <= theTol)
                    {
                        aFits.Add(Qualifier.Enclosed);
                    }
                    if (Math.Abs(d + r - R) <= theTol)
                    {
                        aFits.Add(Qualifier.Enclosing);
                    }
                    break;
                }
            case LinArg l:
                {
                    double s = SignedDistance(l.L, theSol.Location);
                    if (Math.Abs(Math.Abs(s) - R) <= theTol)
                    {
                        aFits.Add(s >= 0.0 ? Qualifier.Enclosed : Qualifier.Outside);
                    }
                    break;
                }
            case PntArg p:
                if (Math.Abs(theSol.Location.DistanceTo(p.P) - R) <= theTol)
                {
                    aFits.Add(Qualifier.Unqualified);
                }
                break;
        }
        return aFits;
    }

    /// <summary>The distance of a point from a line, positive on its left (a line's enclosed side).</summary>
    public static double SignedDistance(in Line2D theL, in Point2D theP)
        => theL.Direction.XY() ^ (theP.XY() - theL.Location.XY());

    /// <summary>True if the circle is the argument circle itself.</summary>
    public static bool IsArgument(in Circle2D theSol, Arg theArg, double theTol)
        => theArg is CircArg c && theSol.Location.DistanceTo(c.C.Location) <= theTol
           && Math.Abs(theSol.Radius - c.C.Radius) <= theTol;

    /// <summary>True if a circle is a valid solution: tangent to every argument with its qualifier, on the locus, of the radius.</summary>
    public static bool IsSolution(in Circle2D theSol, IReadOnlyList<Arg> theArgs, Locus? theLocus, double? theRadius, double theTol)
    {
        foreach (Arg anArg in theArgs)
        {
            List<Qualifier> aFits = Fits(theSol, anArg, theTol);
            if (aFits.Count == 0 || (anArg.Q != Qualifier.Unqualified && !aFits.Contains(anArg.Q)))
            {
                return false;
            }
        }
        return (theLocus is null || OnLocus(theLocus, theSol.Location, theTol))
               && (theRadius is null || Math.Abs(theSol.Radius - theRadius.Value) <= theTol);
    }

    private static bool OnLocus(Locus theLocus, in Point2D theP, double theTol) => theLocus switch
    {
        OnLine l => Math.Abs(SignedDistance(l.L, theP)) <= theTol,
        OnCircle c => Math.Abs(theP.DistanceTo(c.C.Location) - c.C.Radius) <= theTol,
        _ => false,
    };

    /// <summary>Where a circle touches an argument it is tangent to with the given qualifier.</summary>
    private static Point2D TouchPoint(in Circle2D theSol, Arg theArg, Qualifier theFit)
    {
        Point2D C = theSol.Location;
        switch (theArg)
        {
            case CircArg c:
                {
                    Xy u = (c.C.Location.XY() - C.XY()) / C.DistanceTo(c.C.Location);
                    return new Point2D(C.XY() + (theFit == Qualifier.Enclosed ? -theSol.Radius : theSol.Radius) * u);
                }
            case LinArg l:
                {
                    Xy n = new(-l.L.Direction.Y, l.L.Direction.X);
                    return new Point2D(C.XY() - SignedDistance(l.L, C) * n);
                }
            default:
                return ((PntArg)theArg).P;
        }
    }

    /// <summary>
    /// The laws the circle solver's results break, as messages (none when every law holds). An exception from the
    /// solver is a broken law: the cases asked for have solutions or none, never an error.
    /// </summary>
    public static List<string> Check(Func<Circles> theSolver, IReadOnlyList<Arg> theArgs, Locus? theLocus, double? theRadius,
                                     double theTol)
    {
        List<string> aBroken = [];
        Circles S;
        int aNb;
        try
        {
            S = theSolver();
            if (!S.IsDone())
            {
                aBroken.Add("not done");
                return aBroken;
            }
            aNb = S.Count();
        }
        catch (BadQualifierException) when (theArgs.Any(a => a.Q == Qualifier.Noqualifier))
        {
            // a qualifier no argument may have (OCCT didn't check it everywhere)
            return aBroken;
        }
        catch (Exception e)
        {
            aBroken.Add("throws " + Refusal.Unwrapped(e).GetType().Name);
            return aBroken;
        }
        for (int i = 1; i <= aNb; i++)
        {
            try
            {
                CheckOne(S, i, theArgs, theLocus, theRadius, theTol, aBroken);
            }
            catch (Exception e)
            {
                aBroken.Add($"solution {i}: throws {Refusal.Unwrapped(e).GetType().Name}");
            }
        }
        return aBroken;
    }

    private static bool Near(in Point2D theA, in Point2D theB, double theTol) => theA.DistanceTo(theB) <= theTol;

    private static void CheckOne(Circles S, int i, IReadOnlyList<Arg> theArgs, Locus? theLocus, double? theRadius, double theTol,
                                 List<string> theBroken)
    {
        Circle2D aSol = S.Solution(i);
        string aWhat = FormattableString.Invariant($"solution {i} ({aSol.Location.X:R}, {aSol.Location.Y:R}) r {aSol.Radius:R}");
        if (theRadius is double R && Math.Abs(aSol.Radius - R) > theTol)
        {
            theBroken.Add($"{aWhat}: radius not {R}");
        }
        if (theLocus is not null)
        {
            if (!OnLocus(theLocus, aSol.Location, theTol))
            {
                theBroken.Add($"{aWhat}: centre off the locus");
            }
            if (S.CenterOn is not null)
            {
                (double aPar, Point2D aP) = S.CenterOn(i);
                if (!Near(aP, aSol.Location, theTol))
                {
                    theBroken.Add($"{aWhat}: CenterOn3's point is not the centre");
                }
                Point2D aAt = theLocus is OnLine l ? ElCLib.Value(aPar, l.L) : ElCLib.Value(aPar, ((OnCircle)theLocus).C);
                if (!Near(aAt, aSol.Location, theTol))
                {
                    theBroken.Add($"{aWhat}: CenterOn3's parameter is not the centre's");
                }
            }
        }
        Qualifier[]? aQuals = S.Qualifiers?.Invoke(i);
        for (int k = 0; k < theArgs.Count; k++)
        {
            Arg anArg = theArgs[k];
            List<Qualifier> aFits = Fits(aSol, anArg, theTol);
            if (aFits.Count == 0)
            {
                theBroken.Add($"{aWhat}: not tangent to argument {k + 1}");
                continue;
            }
            if (anArg.Q != Qualifier.Unqualified && !aFits.Contains(anArg.Q))
            {
                theBroken.Add($"{aWhat}: qualifier {anArg.Q} of argument {k + 1} not honoured ({string.Join('/', aFits)})");
            }
            // (a solution that is the argument circle may report it unqualified: it is both enclosed and enclosing)
            if (aQuals is not null && anArg is not PntArg && !aFits.Contains(aQuals[k])
                && !(IsArgument(aSol, anArg, theTol) && aQuals[k] == Qualifier.Unqualified))
            {
                theBroken.Add($"{aWhat}: WhichQualifier {aQuals[k]} for argument {k + 1}, not {string.Join('/', aFits)}");
            }
            bool isArg = IsArgument(aSol, anArg, theTol);
            bool isSame = S.IsTheSame?.Invoke(i, k + 1) ?? false;
            if (isSame != isArg && S.IsTheSame is not null)
            {
                theBroken.Add($"{aWhat}: IsTheSame{k + 1} is {isSame}");
            }
            if (isArg || isSame)
            {
                continue;
            }
            (double aParSol, double aParArg, Point2D aP) = S.Tangency(i, k + 1);
            Qualifier aFit = anArg.Q != Qualifier.Unqualified ? anArg.Q : aFits[0];
            if (!Near(aP, TouchPoint(aSol, anArg, aFit), theTol))
            {
                theBroken.Add($"{aWhat}: tangency point {k + 1} not where it touches");
            }
            if (!Near(ElCLib.Value(aParSol, aSol), aP, theTol))
            {
                theBroken.Add($"{aWhat}: tangency {k + 1}'s parameter on the solution");
            }
            Point2D? anOnArg = anArg switch
            {
                CircArg c => ElCLib.Value(aParArg, c.C),
                LinArg l => ElCLib.Value(aParArg, l.L),
                _ => null,
            };
            if (anOnArg is Point2D q && !Near(q, aP, theTol))
            {
                theBroken.Add($"{aWhat}: tangency {k + 1}'s parameter on the argument");
            }
        }
    }

    //=============================================================================================
    // Random arguments

    public static Circle2D RandomCircle(Random theR)
    {
        double a = theR.NextDouble() * 2 * Math.PI;
        return new Circle2D(new Axis2D(RandomPoint(theR), new Direction2D(Math.Cos(a), Math.Sin(a))),
                            0.5 + 3.5 * theR.NextDouble(), theR.Next(4) != 0);
    }

    public static Line2D RandomLine(Random theR)
    {
        double a = theR.NextDouble() * 2 * Math.PI;
        return new Line2D(RandomPoint(theR), new Direction2D(Math.Cos(a), Math.Sin(a)));
    }

    public static Point2D RandomPoint(Random theR) => new(-5 + 10 * theR.NextDouble(), -5 + 10 * theR.NextDouble());

    private static readonly Qualifier[] theCircleQualifiers =
        [Qualifier.Unqualified, Qualifier.Unqualified, Qualifier.Enclosing, Qualifier.Enclosed, Qualifier.Outside];

    private static readonly Qualifier[] theLineQualifiers = [Qualifier.Unqualified, Qualifier.Enclosed, Qualifier.Outside];

    public static CircArg RandomCircArg(Random theR, bool theQualified = true)
        => new(RandomCircle(theR), theQualified ? theCircleQualifiers[theR.Next(theCircleQualifiers.Length)] : Qualifier.Unqualified);

    public static LinArg RandomLinArg(Random theR, bool theQualified = true)
        => new(RandomLine(theR), theQualified ? theLineQualifiers[theR.Next(theLineQualifiers.Length)] : Qualifier.Unqualified);

    public static PntArg RandomPntArg(Random theR) => new(RandomPoint(theR));

    public static QualifiedCirc Q(CircArg theA) => new(theA.C, theA.Q);

    public static QualifiedLin Q(LinArg theA) => new(theA.L, theA.Q);

    /// <summary>The kinds of the broken laws, with their numbers stripped, counted: a survey's summary.</summary>
    public static void Tally(Dictionary<string, int> theTally, IEnumerable<string> theBroken)
    {
        foreach (string aLaw in theBroken.Select(b => System.Text.RegularExpressions.Regex.Replace(
                                                     System.Text.RegularExpressions.Regex.Replace(b, @"^solution \d+ \([^)]*\) r [^:]*: ", ""), "[0-9]", "#")).Distinct())
        {
            theTally[aLaw] = theTally.GetValueOrDefault(aLaw) + 1;
        }
    }

    public static string Describe(IReadOnlyList<Arg> theArgs, Locus? theLocus = null, double? theRadius = null)
        => string.Join("; ", theArgs) + (theLocus is null ? "" : "; on " + theLocus)
           + (theRadius is null ? "" : "; radius " + theRadius.Value.ToString("R", CultureInfo.InvariantCulture));
}
