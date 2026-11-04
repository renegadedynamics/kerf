// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.ModelingAlgorithms.Tests.GccLaws;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The laws of GccAna's line solvers (<see cref="GccLaws.CheckLines"/>) on random arguments, every constructor (Kerf
/// unit 10K4): Lin2d2Tan's lines tangent to two circles or points, Lin2dTanObl's at an angle to a line, Lin2dTanPer's
/// perpendicular to a line or a circle; each tangent with the qualifier asked, WhichQualifier the circle's side, the
/// tangency and intersection points where they are, no exception.
/// </summary>
public class GccAna_LinesLawsTests(ITestOutputHelper theOutput)
{
    private const double SolverTol = 1.0e-7;
    private const double LawTol = 1.0e-6;
    private const int Cases = 1500;

    private static readonly Qualifier[] theLineQualifiers = [Qualifier.Unqualified, Qualifier.Unqualified, Qualifier.Enclosing, Qualifier.Outside];

    /// <summary>A circle argument of a line solver: unqualified, enclosing or outside.</summary>
    private static CircArg RandomCircle(Random theR) => new(GccLaws.RandomCircle(theR), theLineQualifiers[theR.Next(theLineQualifiers.Length)]);

    internal static GccLaws.Lines View(AnalyticLin2d2Tan S) => new()
    {
        IsDone = () => S.IsDone,
        Count = () => S.SolutionCount,
        Solution = S.ThisSolution,
        Qualifiers = i =>
        {
            S.WhichQualifier(i, out Qualifier a, out Qualifier b);
            return [a, b];
        },
        Tangency = (i, k) =>
        {
            double a, b;
            Point2D p;
            if (k == 1)
            {
                S.Tangency1(i, out a, out b, out p);
            }
            else
            {
                S.Tangency2(i, out a, out b, out p);
            }
            return (a, b, p);
        },
    };

    internal static GccLaws.Lines View(AnalyticLin2dTanObl S) => new()
    {
        IsDone = () => S.IsDone,
        Count = () => S.SolutionCount,
        Solution = S.ThisSolution,
        Qualifiers = i =>
        {
            S.WhichQualifier(i, out Qualifier a);
            return [a];
        },
        Tangency = (i, k) =>
        {
            S.Tangency1(i, out double a, out double b, out Point2D p);
            return (a, b, p);
        },
        Intersection = i =>
        {
            S.Intersection2(i, out double a, out double b, out Point2D p);
            return (a, b, p);
        },
    };

    internal static GccLaws.Lines View(Lin2dTanPer S) => new()
    {
        IsDone = () => S.IsDone,
        Count = () => S.SolutionCount,
        Solution = S.ThisSolution,
        Qualifiers = i =>
        {
            S.WhichQualifier(i, out Qualifier a);
            return [a];
        },
        Tangency = (i, k) =>
        {
            S.Tangency1(i, out double a, out double b, out Point2D p);
            return (a, b, p);
        },
        Intersection = i =>
        {
            S.Intersection2(i, out double a, out double b, out Point2D p);
            return (a, b, p);
        },
    };

    /// <summary>The direction law of a line at an angle to another.</summary>
    internal static Func<Line2D, string?> Oblique(Line2D theLine, double theAngle) => l =>
    {
        Xy aWanted = new(Math.Cos(theAngle) * theLine.Direction.X - Math.Sin(theAngle) * theLine.Direction.Y,
                         Math.Sin(theAngle) * theLine.Direction.X + Math.Cos(theAngle) * theLine.Direction.Y);
        return Math.Abs(l.Direction.XY() ^ aWanted) <= 1.0e-9 ? null : "not at the angle";
    };

    /// <summary>The direction law of a line perpendicular to a line, or to a circle (through its centre).</summary>
    internal static Func<Line2D, string?> Perpendicular(object theSecond, double theTol) => l => theSecond switch
    {
        Line2D t => Math.Abs(l.Direction.XY().Dot(t.Direction.XY())) <= 1.0e-9 ? null : "not perpendicular",
        Circle2D c => Math.Abs(SignedDistance(l, c.Location)) <= theTol ? null : "not through the circle's centre",
        _ => "?",
    };

    public static TheoryData<string> Kinds() => new()
    {
        "2Tan.CC", "2Tan.CP", "2Tan.PP", "Obl.C", "Obl.P", "Per.Pl", "Per.Pc", "Per.Cl", "Per.Cc",
    };

    [Theory]
    [MemberData(nameof(Kinds))]
    public void Random_arguments_obey_the_laws(string theKind)
    {
        Random aR = new(5000 + theKind.Sum(c => c * 7));
        int aNbBroken = 0, aNbSolutions = 0;
        List<string> aReport = [];
        Dictionary<string, int> aTally = [];
        for (int n = 0; n < Cases; n++)
        {
            List<string> aBroken;
            string aCase;
            switch (theKind)
            {
                case "2Tan.CC":
                case "2Tan.CP":
                case "2Tan.PP":
                    {
                        Arg a = theKind == "2Tan.PP" ? RandomPntArg(aR) : RandomCircle(aR);
                        Arg b = theKind == "2Tan.CC" ? RandomCircle(aR) : RandomPntArg(aR);
                        aCase = Describe([a, b]);
                        aBroken = CheckLines(() =>
                        {
                            AnalyticLin2d2Tan S = (a, b) switch
                            {
                                (CircArg x, CircArg y) => new(Q(x), Q(y), SolverTol),
                                (CircArg x, PntArg y) => new(Q(x), y.P, SolverTol),
                                _ => new(((PntArg)a).P, ((PntArg)b).P, SolverTol),
                            };
                            aNbSolutions += S.IsDone ? S.SolutionCount : 0;
                            return View(S);
                        }, [a, b], null, null, LawTol);
                        break;
                    }
                case "Obl.C":
                case "Obl.P":
                    {
                        Arg a = theKind == "Obl.C" ? RandomCircle(aR) : RandomPntArg(aR);
                        Line2D aLine = RandomLine(aR);
                        // some angles 0 and pi: solutions parallel to the line
                        double anAngle = aR.Next(6) switch { 0 => 0.0, 1 => Math.PI, _ => (aR.NextDouble() - 0.5) * 2 * Math.PI };
                        aCase = Describe([a]) + $"; line {aLine.Location} {aLine.Direction}; angle {anAngle}";
                        aBroken = CheckLines(() =>
                        {
                            AnalyticLin2dTanObl S = a is CircArg x ? new(Q(x), aLine, anAngle) : new(((PntArg)a).P, aLine, anAngle);
                            aNbSolutions += S.IsDone ? S.SolutionCount : 0;
                            return View(S);
                        }, [a], Oblique(aLine, anAngle), aLine, LawTol);
                        break;
                    }
                default:
                    {
                        Arg a = theKind[4] == 'C' ? RandomCircle(aR) : RandomPntArg(aR);
                        object aSecond = theKind[5] == 'l' ? RandomLine(aR) : GccLaws.RandomCircle(aR);
                        aCase = Describe([a]) + $"; perpendicular to {aSecond}";
                        aBroken = CheckLines(() =>
                        {
                            Lin2dTanPer S = (a, aSecond) switch
                            {
                                (CircArg x, Line2D l) => new(Q(x), l),
                                (CircArg x, Circle2D c) => new(Q(x), c),
                                (PntArg x, Line2D l) => new(x.P, l),
                                _ => new(((PntArg)a).P, (Circle2D)aSecond),
                            };
                            aNbSolutions += S.IsDone ? S.SolutionCount : 0;
                            return View(S);
                        }, [a], Perpendicular(aSecond, LawTol), aSecond, LawTol);
                        break;
                    }
            }
            Tally(aTally, aBroken.Select(b => System.Text.RegularExpressions.Regex.Replace(b, @"^solution \d+ \([^)]*\) d \([^)]*\): ", "")));
            if (aBroken.Count > 0)
            {
                aNbBroken++;
                if (aReport.Count < 20)
                {
                    aReport.Add(aCase + ": " + string.Join(" | ", aBroken.Take(3)));
                }
            }
        }
        theOutput.WriteLine($"{theKind}: {Cases} cases, {aNbSolutions} solutions, {aNbBroken} cases break a law");
        theOutput.WriteLine(string.Join(Environment.NewLine, aTally.OrderByDescending(t => t.Value).Select(t => $"  {t.Value,5} {t.Key}")));
        theOutput.WriteLine(string.Join(Environment.NewLine, aReport));
        Assert.True(aNbBroken == 0, string.Join(Environment.NewLine, aReport));
    }
}
