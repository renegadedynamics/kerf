// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.ModelingAlgorithms.Tests.GccLaws;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Circ2d3Tan's laws (<see cref="GccLaws"/>) on random arguments, every constructor (Kerf unit 10K4): each
/// solution tangent to its three arguments with the qualifiers asked, WhichQualifier the solution's, the tangency
/// points where the circles touch, no exception.
/// </summary>
public class GccAna_Circ2d3TanLawsTests(ITestOutputHelper theOutput)
{
    private const double SolverTol = 1.0e-7;
    private const double LawTol = 1.0e-6;
    private const int Cases = 1500;

    internal static GccLaws.Circles View(AnalyticCirc2d3Tan S) => new()
    {
        IsDone = () => S.IsDone,
        Count = () => S.SolutionCount,
        Solution = S.ThisSolution,
        Qualifiers = i =>
        {
            S.WhichQualifier(i, out Qualifier a, out Qualifier b, out Qualifier c);
            return [a, b, c];
        },
        Tangency = (i, k) =>
        {
            double a, b;
            Point2D p;
            switch (k)
            {
                case 1: S.Tangency1(i, out a, out b, out p); break;
                case 2: S.Tangency2(i, out a, out b, out p); break;
                default: S.Tangency3(i, out a, out b, out p); break;
            }
            return (a, b, p);
        },
        IsTheSame = (i, k) => k switch { 1 => S.IsTheSame1(i), 2 => S.IsTheSame2(i), _ => S.IsTheSame3(i) },
    };

    /// <summary>Builds the solver for three arguments of any kind (circles first, then lines, then points, as GccAna orders them).</summary>
    internal static AnalyticCirc2d3Tan Solve(Arg a, Arg b, Arg c, double theTol) => (a, b, c) switch
    {
        (CircArg x, CircArg y, CircArg z) => new(Q(x), Q(y), Q(z), theTol),
        (CircArg x, CircArg y, LinArg z) => new(Q(x), Q(y), Q(z), theTol),
        (CircArg x, LinArg y, LinArg z) => new(Q(x), Q(y), Q(z), theTol),
        (LinArg x, LinArg y, LinArg z) => new(Q(x), Q(y), Q(z), theTol),
        (CircArg x, CircArg y, PntArg z) => new(Q(x), Q(y), z.P, theTol),
        (CircArg x, LinArg y, PntArg z) => new(Q(x), Q(y), z.P, theTol),
        (LinArg x, LinArg y, PntArg z) => new(Q(x), Q(y), z.P, theTol),
        (CircArg x, PntArg y, PntArg z) => new(Q(x), y.P, z.P, theTol),
        (LinArg x, PntArg y, PntArg z) => new(Q(x), y.P, z.P, theTol),
        (PntArg x, PntArg y, PntArg z) => new(x.P, y.P, z.P, theTol),
        _ => throw new ArgumentException("argument order"),
    };

    public static TheoryData<string> Kinds() => new() { "CCC", "CCL", "CLL", "LLL", "CCP", "CLP", "LLP", "CPP", "LPP", "PPP" };

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
        Random aR = new(1000 + theKinds.Sum(c => c * 7));
        int aNbBroken = 0, aNbSolutions = 0;
        List<string> aReport = [];
        Dictionary<string, int> aTally = [];
        for (int n = 0; n < Cases; n++)
        {
            Arg[] anArgs = [RandomArg(aR, theKinds[0]), RandomArg(aR, theKinds[1]), RandomArg(aR, theKinds[2])];
            List<string> aBroken = Check(() =>
            {
                AnalyticCirc2d3Tan S = Solve(anArgs[0], anArgs[1], anArgs[2], SolverTol);
                aNbSolutions += S.IsDone ? S.SolutionCount : 0;
                return View(S);
            }, anArgs, null, null, LawTol);
            Tally(aTally, aBroken);
            if (aBroken.Count > 0)
            {
                aNbBroken++;
                if (aReport.Count < 20)
                {
                    aReport.Add(Describe(anArgs) + ": " + string.Join(" | ", aBroken.Take(3)));
                }
            }
        }
        theOutput.WriteLine($"{theKinds}: {Cases} cases, {aNbSolutions} solutions, {aNbBroken} cases break a law");
        theOutput.WriteLine(string.Join(Environment.NewLine, aTally.OrderByDescending(t => t.Value).Select(t => $"  {t.Value,5} {t.Key}")));
        theOutput.WriteLine(string.Join(Environment.NewLine, aReport));
        Assert.True(aNbBroken == 0, string.Join(Environment.NewLine, aReport));
    }

    /// <summary>
    /// OCCT's corpus cases geometry/circ2d3Tan/CircleCirclePoint_13 and _14 (Kerf decision 1551): the circles (0, 0) r 10
    /// and (30, 0) r 20 with the point (20, -10) have one solution, (20, 0) r 10, and with the point (30, -20) on the second
    /// circle two, that circle itself and (30, 40) r 60; OCCT listed the last of each twice.
    /// </summary>
    [Theory]
    [InlineData(20.0, -10.0, 1)]
    [InlineData(30.0, -20.0, 2)]
    public void Corpus_two_circles_and_a_point_each_solution_once(double theX, double theY, int theNb)
    {
        Arg[] anArgs =
        [
            new CircArg(new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 10), Qualifier.Unqualified),
            new CircArg(new Circle2D(new Axis2D(new Point2D(30, 0), new Direction2D(1, 0)), 20), Qualifier.Unqualified),
            new PntArg(new Point2D(theX, theY)),
        ];
        int aNb = 0;
        List<string> aBroken = Check(() =>
        {
            AnalyticCirc2d3Tan S = Solve(anArgs[0], anArgs[1], anArgs[2], SolverTol);
            aNb = S.SolutionCount;
            return View(S);
        }, anArgs, null, null, LawTol);
        Assert.Empty(aBroken);
        Assert.Equal(theNb, aNb);
    }
}
