// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using System.Globalization;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Plan 3.2's third level for the solvers Kerf corrected (unit 10K4, docs/occt-issues.md's triage batch K4): a case
/// whose results differ from native's recording is an intended difference when the port's results obey the laws of a
/// constrained circle and keep every solution of native's that obeys them. The laws, read from the port's tokens:
/// each solution has the radius asked and its centre on the curve asked (CenterOn3 giving it), and for each argument
/// either is the argument (IsTheSame) or touches it where Tangency says, the point on the solution and on the argument
/// at their parameters, the argument's tangent there perpendicular to the radius, and a line's or circle's qualifier
/// honoured. Native's solutions are judged by the distance from their centre to each argument.
/// </summary>
public sealed partial class Geom2dGccOracleTests
{
    /// <summary>An argument as the laws see it: a curve (a line's or circle's qualifier is judged) or a point.</summary>
    internal sealed record LawArg(Curve2D? Curve, Point2D? Point, Qualifier Q);

    internal sealed record Laws(LawArg[] Args, Curve2D? On, double? Radius, double Tolerance);

    private static LawArg L(Arg theA, Curve2D theP) => new(theP, null, theA.Q);

    private static LawArg L(double theX, double theY) => new(null, new Point2D(theX, theY), Qualifier.Unqualified);

    private static Dictionary<string, string> Tokens(Rec theR)
    {
        Dictionary<string, string> aMap = new(StringComparer.Ordinal);
        foreach (var (aName, aValue) in theR.T)
        {
            aMap[aName] = aValue;
        }
        return aMap;
    }

    private static double Num(Dictionary<string, string> theT, string theName)
    {
        string aV = theT[theName];
        return aV == "NaN" ? double.NaN : BitConverter.Int64BitsToDouble(long.Parse(aV, NumberStyles.HexNumber, CultureInfo.InvariantCulture));
    }

    private static Point2D Pnt(Dictionary<string, string> theT, string theName) => new(Num(theT, theName + ".x"), Num(theT, theName + ".y"));

    private static Circle2D Circle(Dictionary<string, string> theT, string theName)
        => new(new Frame2D(Pnt(theT, theName + ".loc"), new Direction2D(Num(theT, theName + ".xd.x"), Num(theT, theName + ".xd.y")),
                           new Direction2D(Num(theT, theName + ".yd.x"), Num(theT, theName + ".yd.y"))),
               Num(theT, theName + ".r"));

    /// <summary>The solutions' circles of a record.</summary>
    private static List<Circle2D> Circles(Rec theR)
    {
        Dictionary<string, string> aT = Tokens(theR);
        List<Circle2D> aList = [];
        if (!aT.TryGetValue("nb", out string? aNb))
        {
            return aList;
        }
        for (int i = 1; i <= int.Parse(aNb, CultureInfo.InvariantCulture); i++)
        {
            if (aT.ContainsKey($"s.{i}.r"))
            {
                aList.Add(Circle(aT, $"s.{i}"));
            }
        }
        return aList;
    }

    /// <summary>The line or circle a drawn analytic argument is, or null.</summary>
    private static object? Analytic(LawArg theA) => theA.Curve is null ? null : Analytic(theA.Curve);

    private static object? Analytic(Curve2D theC) => theC switch
    {
        LineCurve2D l => l.Line,
        CircleCurve2D c => c.Circle,
        TrimmedCurve2D t => Analytic(t.BasisCurve),
        _ => null,
    };

    /// <summary>True if a line's or circle's qualifier is honoured by the circle (other curves' aren't judged).</summary>
    private static bool Honours(in Circle2D theSol, LawArg theA, double theTol)
    {
        if (theA.Q == Qualifier.Unqualified)
        {
            return true;
        }
        switch (Analytic(theA))
        {
            case Line2D l:
                {
                    double s = l.Direction.XY() ^ (theSol.Location.XY() - l.Location.XY());
                    return theA.Q == (s >= 0 ? Qualifier.Enclosed : Qualifier.Outside);
                }
            case Circle2D c:
                {
                    double d = theSol.Location.DistanceTo(c.Location);
                    return theA.Q switch
                    {
                        Qualifier.Outside => Math.Abs(d - theSol.Radius - c.Radius) <= theTol,
                        Qualifier.Enclosed => Math.Abs(d + theSol.Radius - c.Radius) <= theTol,
                        _ => Math.Abs(d + c.Radius - theSol.Radius) <= theTol,
                    };
                }
        }
        return true;
    }

    /// <summary>The distances from a point to a curve where they are extremal.</summary>
    /// <remarks>A trimmed curve's basis is used: the solvers solve on the whole line or circle (GccAna), and the laws don't judge
    /// where on the curve a solution touches.</remarks>
    private static IEnumerable<double> Distances(Point2D theP, Curve2D theC)
    {
        while (theC is TrimmedCurve2D t)
        {
            theC = t.BasisCurve;
        }
        ProjectPointOnCurve2D aProj = new(theP, theC);
        return Enumerable.Range(1, aProj.NbPoints()).Select(aProj.Distance).ToList();
    }

    /// <summary>True if a circle is a solution by its distances to the arguments (native's solutions are judged so).</summary>
    private static bool IsSolution(in Circle2D theSol, Laws theLaws, double theTol)
    {
        double R = theSol.Radius;
        Point2D C = theSol.Location;
        if (theLaws.Radius is double aR && Math.Abs(R - aR) > theTol)
        {
            return false;
        }
        if (theLaws.On is not null && !Distances(C, theLaws.On).Any(d => d <= theTol))
        {
            return false;
        }
        foreach (LawArg anA in theLaws.Args)
        {
            bool isTangent = anA.Point is Point2D p ? Math.Abs(C.DistanceTo(p) - R) <= theTol
                                                    : Distances(C, anA.Curve!).Any(d => Math.Abs(d - R) <= theTol);
            if (!isTangent || !Honours(theSol, anA, theTol))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>The laws the port's record breaks.</summary>
    private static List<string> BrokenLaws(Rec thePort, Laws theLaws)
    {
        List<string> aBroken = [];
        double aTol = LawTolerance(theLaws.Tolerance);
        Dictionary<string, string> aT = Tokens(thePort);
        if (aT.TryGetValue("ctor", out string? aThrow))
        {
            aBroken.Add("the constructor " + aThrow);
            return aBroken;
        }
        if (aT["done"] != "1")
        {
            return aBroken;
        }
        int aNb = int.Parse(aT["nb"], CultureInfo.InvariantCulture);
        for (int i = 1; i <= aNb; i++)
        {
            if (aT.TryGetValue($"s.{i}", out string? aSolThrow))
            {
                aBroken.Add($"solution {i} {aSolThrow}");
                continue;
            }
            Circle2D aSol = Circle(aT, $"s.{i}");
            Point2D C = aSol.Location;
            double R = aSol.Radius;
            if (theLaws.Radius is double aR && Math.Abs(R - aR) > aTol)
            {
                aBroken.Add($"solution {i}: radius");
            }
            if (theLaws.On is not null)
            {
                // on the curve by its distance, or (where the projection misses, as at a B-spline's end) at CenterOn3's
                // parameter within the curve's range
                bool isOn = Distances(C, theLaws.On).Any(d => d <= aTol)
                            || (aT.ContainsKey($"c3.{i}.pararg") && Num(aT, $"c3.{i}.pararg") is double u
                                && u >= theLaws.On.FirstParameter - 1.0e-9 && u <= theLaws.On.LastParameter + 1.0e-9
                                && theLaws.On.Value(u).DistanceTo(C) <= aTol);
                if (!isOn)
                {
                    aBroken.Add($"solution {i}: centre off the curve");
                }
                if (aT.TryGetValue($"c3.{i}", out string? aC3Throw))
                {
                    aBroken.Add($"solution {i}: CenterOn3 {aC3Throw}");
                }
                else if (aT.ContainsKey($"c3.{i}.pnt.x")
                         && (Pnt(aT, $"c3.{i}.pnt").DistanceTo(C) > aTol
                             || theLaws.On.Value(Num(aT, $"c3.{i}.pararg")).DistanceTo(C) > aTol))
                {
                    aBroken.Add($"solution {i}: CenterOn3 isn't the centre");
                }
            }
            for (int k = 1; k <= theLaws.Args.Length; k++)
            {
                LawArg anA = theLaws.Args[k - 1];
                if (!Honours(aSol, anA, aTol))
                {
                    aBroken.Add($"solution {i}: qualifier {anA.Q} of argument {k}");
                }
                if (aT.GetValueOrDefault($"same{k}.{i}") == "1")
                {
                    if (Analytic(anA) is Circle2D c && (c.Location.DistanceTo(C) > aTol || Math.Abs(c.Radius - R) > aTol))
                    {
                        aBroken.Add($"solution {i}: IsTheSame{k} but not the argument");
                    }
                    continue;
                }
                if (aT.TryGetValue($"t{k}.{i}", out string? aTanThrow))
                {
                    aBroken.Add($"solution {i}: Tangency{k} {aTanThrow}");
                    continue;
                }
                Point2D P = Pnt(aT, $"t{k}.{i}.pnt");
                if (Math.Abs(P.DistanceTo(C) - R) > aTol || ElCLib.Value(Num(aT, $"t{k}.{i}.parsol"), aSol).DistanceTo(P) > aTol)
                {
                    aBroken.Add($"solution {i}: tangency point {k} off the solution or its parameter");
                }
                if (anA.Point is Point2D aPoint)
                {
                    if (P.DistanceTo(aPoint) > aTol)
                    {
                        aBroken.Add($"solution {i}: doesn't pass through point {k}");
                    }
                    continue;
                }
                anA.Curve!.D1(Num(aT, $"t{k}.{i}.pararg"), out Point2D aOn, out Vector2D aTangent);
                if (aOn.DistanceTo(P) > aTol)
                {
                    aBroken.Add($"solution {i}: tangency point {k} off the argument or its parameter");
                }
                else if (aTangent.Magnitude > 0 && Math.Abs(aTangent.Normalized().Dot(new Vector2D(C, P))) > aTol)
                {
                    aBroken.Add($"solution {i}: not tangent to argument {k}");
                }
            }
        }
        return aBroken;
    }

    private static double LawTolerance(double theTol) => Math.Max(1.0e-6, 10 * theTol);

    /// <summary>True if a differing case is an intended difference (see the class's remarks); otherwise why not.</summary>
    internal static bool IsIntendedDifference(Rec theNative, Rec thePort, Laws theLaws, out string theWhy)
    {
        theWhy = "";
        List<string> aBroken = BrokenLaws(thePort, theLaws);
        if (aBroken.Count > 0)
        {
            theWhy = "the port breaks a law: " + aBroken[0];
            return false;
        }
        double aTol = LawTolerance(theLaws.Tolerance);
        List<Circle2D> aPort = Circles(thePort);
        foreach (Circle2D aNative in Circles(theNative))
        {
            if (IsSolution(aNative, theLaws, aTol)
                && !aPort.Any(p => p.Location.DistanceTo(aNative.Location) <= aTol && Math.Abs(p.Radius - aNative.Radius) <= aTol))
            {
                theWhy = FormattableString.Invariant($"native's solution ({aNative.Location.X}, {aNative.Location.Y}) r {aNative.Radius} is lost");
                return false;
            }
        }
        return true;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Lines

    /// <summary>
    /// The laws of a line solver: tangent to its arguments (Lin2d2Tan) or to one, at an angle to a line (Lin2dTanObl,
    /// whose second token is the intersection with that line).
    /// </summary>
    internal sealed record LineLaws(LawArg[] Args, Line2D? Oblique, double Angle, double Tolerance);

    private static Line2D Line(Dictionary<string, string> theT, string theName)
        => new(Pnt(theT, theName + ".loc"), new Direction2D(Num(theT, theName + ".d.x"), Num(theT, theName + ".d.y")));

    private static double Side(in Line2D theL, in Point2D theP) => theL.Direction.XY() ^ (theP.XY() - theL.Location.XY());

    /// <summary>The laws solution i of a record breaks, judged from its own tokens (native's solutions are judged so too).</summary>
    private static List<string> LineSolutionLaws(Dictionary<string, string> theT, int i, LineLaws theLaws, double theTol)
    {
        List<string> aBroken = [];
        if (theT.TryGetValue($"s.{i}", out string? aThrow))
        {
            aBroken.Add($"solution {i} {aThrow}");
            return aBroken;
        }
        Line2D aSol = Line(theT, $"s.{i}");
        if (theLaws.Oblique is Line2D aRef)
        {
            Xy aWanted = new(Math.Cos(theLaws.Angle) * aRef.Direction.X - Math.Sin(theLaws.Angle) * aRef.Direction.Y,
                             Math.Sin(theLaws.Angle) * aRef.Direction.X + Math.Cos(theLaws.Angle) * aRef.Direction.Y);
            if (Math.Abs(aSol.Direction.XY() ^ aWanted) > theTol)
            {
                aBroken.Add($"solution {i}: not at the angle");
            }
            bool isParallel = Math.Abs(aSol.Direction.XY() ^ aRef.Direction.XY()) <= 1.0e-12;
            if (theT.TryGetValue($"t2.{i}", out string? aIThrow))
            {
                if (!isParallel || !aIThrow.Contains("IsParallel", StringComparison.Ordinal))
                {
                    aBroken.Add($"solution {i}: Intersection2 {aIThrow}");
                }
            }
            else
            {
                Point2D aI = Pnt(theT, $"t2.{i}.pnt");
                if (Math.Abs(Side(aSol, aI)) > theTol || Math.Abs(Side(aRef, aI)) > theTol
                    || ElCLib.Value(Num(theT, $"t2.{i}.parsol"), aSol).DistanceTo(aI) > theTol
                    || ElCLib.Value(Num(theT, $"t2.{i}.pararg"), aRef).DistanceTo(aI) > theTol)
                {
                    aBroken.Add($"solution {i}: intersection with the line");
                }
            }
        }
        for (int k = 1; k <= theLaws.Args.Length; k++)
        {
            LawArg anA = theLaws.Args[k - 1];
            if (theT.TryGetValue($"t{k}.{i}", out string? aTThrow))
            {
                aBroken.Add($"solution {i}: Tangency{k} {aTThrow}");
                continue;
            }
            Point2D P = Pnt(theT, $"t{k}.{i}.pnt");
            if (Math.Abs(Side(aSol, P)) > theTol || ElCLib.Value(Num(theT, $"t{k}.{i}.parsol"), aSol).DistanceTo(P) > theTol)
            {
                aBroken.Add($"solution {i}: tangency point {k} off the solution or its parameter");
            }
            if (anA.Point is Point2D aPoint)
            {
                if (P.DistanceTo(aPoint) > theTol)
                {
                    aBroken.Add($"solution {i}: doesn't pass through point {k}");
                }
                continue;
            }
            anA.Curve!.D1(Num(theT, $"t{k}.{i}.pararg"), out Point2D aOn, out Vector2D aTangent);
            if (aOn.DistanceTo(P) > theTol)
            {
                aBroken.Add($"solution {i}: tangency point {k} off the argument or its parameter");
            }
            else if (aTangent.Magnitude > 0 && Math.Abs(aTangent.Normalized().XY() ^ aSol.Direction.XY()) > theTol)
            {
                aBroken.Add($"solution {i}: not tangent to argument {k}");
            }
            if (anA.Q != Qualifier.Unqualified && Analytic(anA) is Circle2D c && c.Radius > theTol
                && (Side(aSol, c.Location) >= 0 ? Qualifier.Enclosing : Qualifier.Outside) != anA.Q)
            {
                aBroken.Add($"solution {i}: qualifier {anA.Q} of argument {k}");
            }
        }
        return aBroken;
    }

    private static int Count(Dictionary<string, string> theT)
        => theT.TryGetValue("done", out string? aDone) && aDone == "1" && theT.TryGetValue("nb", out string? aNb)
            ? int.Parse(aNb, CultureInfo.InvariantCulture) : 0;

    /// <summary>True if a differing line case is an intended difference; otherwise why not.</summary>
    internal static bool IsIntendedDifference(Rec theNative, Rec thePort, LineLaws theLaws, out string theWhy)
    {
        theWhy = "";
        double aTol = LawTolerance(theLaws.Tolerance);
        Dictionary<string, string> aP = Tokens(thePort);
        if (aP.TryGetValue("ctor", out string? aThrow))
        {
            theWhy = "the constructor " + aThrow;
            return false;
        }
        List<Line2D> aPort = [];
        for (int i = 1; i <= Count(aP); i++)
        {
            List<string> aBroken = LineSolutionLaws(aP, i, theLaws, aTol);
            if (aBroken.Count > 0)
            {
                theWhy = "the port breaks a law: " + aBroken[0];
                return false;
            }
            aPort.Add(Line(aP, $"s.{i}"));
        }
        Dictionary<string, string> aN = Tokens(theNative);
        for (int i = 1; i <= Count(aN); i++)
        {
            if (LineSolutionLaws(aN, i, theLaws, aTol).Count > 0)
            {
                continue;
            }
            Line2D aNative = Line(aN, $"s.{i}");
            if (!aPort.Any(p => Math.Abs(p.Direction.XY() ^ aNative.Direction.XY()) <= aTol && p.DistanceTo(aNative.Location) <= aTol))
            {
                theWhy = FormattableString.Invariant($"native's solution through ({aNative.Location.X}, {aNative.Location.Y}) is lost");
                return false;
            }
        }
        return true;
    }
}
