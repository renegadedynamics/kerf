// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.ModelingAlgorithms.Tests.GccLaws;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Circ2d2TanOn's laws (<see cref="GccLaws"/>) on random arguments, every constructor (Kerf unit 10K4): each
/// solution tangent to its two arguments with the qualifiers asked and centred on the line or circle, CenterOn3 its
/// centre, WhichQualifier the solution's, the tangency points where the circles touch, no exception.
/// </summary>
public class GccAna_Circ2d2TanOnLawsTests(ITestOutputHelper theOutput)
{
    private const double SolverTol = 1.0e-7;
    private const double LawTol = 1.0e-6;
    private const int Cases = 1500;

    internal static GccLaws.Circles View(AnalyticCirc2d2TanOn S) => new()
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
        IsTheSame = (i, k) => k == 1 ? S.IsTheSame1(i) : S.IsTheSame2(i),
        CenterOn = i =>
        {
            S.CenterOn3(i, out double a, out Point2D p);
            return (a, p);
        },
    };

    internal static AnalyticCirc2d2TanOn Solve(Arg a, Arg b, Locus theOn, double theTol) => (a, b, theOn) switch
    {
        (CircArg x, CircArg y, OnLine l) => new(Q(x), Q(y), l.L, theTol),
        (CircArg x, LinArg y, OnLine l) => new(Q(x), Q(y), l.L, theTol),
        (LinArg x, LinArg y, OnLine l) => new(Q(x), Q(y), l.L, theTol),
        (CircArg x, PntArg y, OnLine l) => new(Q(x), y.P, l.L, theTol),
        (LinArg x, PntArg y, OnLine l) => new(Q(x), y.P, l.L, theTol),
        (PntArg x, PntArg y, OnLine l) => new(x.P, y.P, l.L, theTol),
        (CircArg x, CircArg y, OnCircle c) => new(Q(x), Q(y), c.C, theTol),
        (CircArg x, LinArg y, OnCircle c) => new(Q(x), Q(y), c.C, theTol),
        (LinArg x, LinArg y, OnCircle c) => new(Q(x), Q(y), c.C, theTol),
        (CircArg x, PntArg y, OnCircle c) => new(Q(x), y.P, c.C, theTol),
        (LinArg x, PntArg y, OnCircle c) => new(Q(x), y.P, c.C, theTol),
        (PntArg x, PntArg y, OnCircle c) => new(x.P, y.P, c.C, theTol),
        _ => throw new ArgumentException("argument order"),
    };

    public static TheoryData<string> Kinds() => new() { "CCl", "CLl", "LLl", "CPl", "LPl", "PPl", "CCc", "CLc", "LLc", "CPc", "LPc", "PPc" };

    private static Arg RandomArg(Random theR, char theKind) => theKind switch
    {
        'C' => RandomCircArg(theR),
        'L' => RandomLinArg(theR),
        _ => RandomPntArg(theR),
    };

    [Theory]
    [MemberData(nameof(Kinds))]
    public void Random_arguments_obey_the_laws(string theKinds)
    {
        Random aR = new(2000 + theKinds.Sum(c => c * 7));
        int aNbBroken = 0, aNbSolutions = 0;
        List<string> aReport = [];
        Dictionary<string, int> aTally = [];
        for (int n = 0; n < Cases; n++)
        {
            Arg[] anArgs = [RandomArg(aR, theKinds[0]), RandomArg(aR, theKinds[1])];
            Locus anOn = theKinds[2] == 'l' ? new OnLine(RandomLine(aR)) : new OnCircle(RandomCircle(aR));
            List<string> aBroken = Check(() =>
            {
                AnalyticCirc2d2TanOn S = Solve(anArgs[0], anArgs[1], anOn, SolverTol);
                aNbSolutions += S.IsDone ? S.SolutionCount : 0;
                return View(S);
            }, anArgs, anOn, null, LawTol);
            Tally(aTally, aBroken);
            if (aBroken.Count > 0)
            {
                aNbBroken++;
                if (aReport.Count < 20)
                {
                    aReport.Add(Describe(anArgs, anOn) + ": " + string.Join(" | ", aBroken.Take(3)));
                }
            }
        }
        theOutput.WriteLine($"{theKinds}: {Cases} cases, {aNbSolutions} solutions, {aNbBroken} cases break a law");
        theOutput.WriteLine(string.Join(Environment.NewLine, aTally.OrderByDescending(t => t.Value).Select(t => $"  {t.Value,5} {t.Key}")));
        theOutput.WriteLine(string.Join(Environment.NewLine, aReport));
        Assert.True(aNbBroken == 0, string.Join(Environment.NewLine, aReport));
    }
}
