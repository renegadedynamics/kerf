// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepBlend's walkers against native OCCT, bit for bit and step by step: tools/blendrec records them with
/// Kerf.Modeling's OCCT V8_0_1 (BRepBlend is not bound), <see cref="BlendRecorder"/> makes the same calls on the port,
/// and the texts are compared case by case. Each case records every point of the walking line (guide parameter,
/// points, parameters, tangents), the extremities (points, parameters, tangents, the arcs they lie on with their
/// transitions), the transitions and statuses, then the line's approximation by BRepBlend_AppSurface (and
/// BRepBlend_AppSurf): poles, weights, knots, 2D curves, tolerances. The configurations: a constant-radius blend
/// between two planes (a walk to the faces' restrictions, a walk to a bound continued with <c>Continu</c>, an
/// evolving radius), a plane and a cylinder, two B-splines, in each of the eight "quadrants"; the first sections
/// (inside, and from outside moved onto a restriction); the curve/surface walking; the surface/restriction and
/// restriction/restriction line builders (constant and evolving radius, stopping on the restrictions' ends and the
/// face's boundary).
/// </summary>
public class BRepBlendNativeTests
{
    private static readonly Lazy<Dictionary<string, List<string>>> thePort = new(() =>
    {
        BlendRecorder aRec = new();
        aRec.RecordWalking();
        return BlendFuncNativeTests.Split(aRec.Text);
    });

    private static void Compare(Func<string, bool> theFilter, int theExpectedCases)
    {
        Dictionary<string, List<string>> aPort =
            thePort.Value.Where(kv => theFilter(kv.Key) && !IsBallSection(kv.Key)).ToDictionary(kv => kv.Key, kv => kv.Value);
        BlendFuncNativeTests.Compare(aPort, k => BlendFuncNativeTests.IsWalk(k) && theFilter(k) && !IsBallSection(k),
                                     theExpectedCases);
    }

    // The constant-radius walks whose faces' normals at the contacts leave the plane square to the guide: since
    // decision 2156 their sections are the rolling ball's, where OCCT's are the 2D fillet in that plane, so they
    // differ from native (planes, whose normals stay in it, still match bit for bit).
    private static readonly string[] THE_BALL_SECTIONS =
    [
        "case walk planecyl 1", "case walk planecyl 2", "case walk planecyl 7", "case walk planecyl 8",
        "case walk bspline 1", "case walk bspline 2", "case walk bspline 3", "case walk bspline 4",
        "case walk bspline 5", "case walk bspline 6", "case walk bspline 7", "case walk bspline 8",
        "case walk firstsection 3", "case walk firstsection 4",
        // SurfRstConstRad's (decision 2158): the same walk to the last few bits
        "case walk surfrst 1 1", "case walk surfrst 1 3", "case walk surfrst 1 5", "case walk surfrst 1 7",
        // The evolving radius's (decision 31204): the same walks to the last few bits
        "case walk surfrstevol 1 7", "case walk rstrstevol 1", "case walk rstrstevol 3",
    ];

    private static bool IsBallSection(string theKey) => THE_BALL_SECTIONS.Contains(theKey);

    [Fact]
    public void Walking_OnTheBall_WalksAndApproximatesWhereNativeDoes()
    {
        // Decision 2156: the cases that differ from native still walk, end and are approximated where native's are,
        // with points spaced as native's to within a few.
        Dictionary<string, List<string>> aNative = BlendFuncNativeTests.Recording;
        foreach (string aKey in THE_BALL_SECTIONS)
        {
            List<string> aN = aNative[aKey], aP = thePort.Value[aKey];
            foreach (string aTag in new[] { "perform 1", "appsurface 1", "appsurf 1", "exception" })
            {
                Assert.True(aN.Contains(aTag) == aP.Contains(aTag), $"{aKey}: {aTag}");
            }

            string? aNLine = aN.FirstOrDefault(l => l.StartsWith("line ", StringComparison.Ordinal));
            string? aPLine = aP.FirstOrDefault(l => l.StartsWith("line ", StringComparison.Ordinal));
            Assert.Equal(aNLine is null, aPLine is null);
            if (aNLine is not null)
            {
                Assert.InRange(int.Parse(aPLine!.Split(' ')[1]), int.Parse(aNLine.Split(' ')[1]) - 3,
                               int.Parse(aNLine.Split(' ')[1]) + 3);
            }
        }
    }

    private static bool Is(string theKey, string theName)
        => theKey.StartsWith($"case walk {theName} ", StringComparison.Ordinal);

    [Fact]
    public void Walking_SurfSurf_MatchesNativeBitForBit()
    {
        Compare(k => BlendRecorder.THE_WALKS.Any(w => Is(k, w.Name)), BlendRecorder.THE_WALKS.Length * 8 - 12);
    }

    [Fact]
    public void Walking_FirstSections_MatchNativeBitForBit()
    {
        Compare(k => Is(k, "firstsection"), 8 - 2);
    }

    [Fact]
    public void CSWalking_MatchesNativeBitForBit()
    {
        Compare(k => Is(k, "cs"), 8);
    }

    [Fact]
    public void SurfRstLineBuilder_MatchesNativeBitForBit()
    {
        Compare(k => BlendRecorder.THE_SURFRSTS.Any(c => Is(k, c.Name)), BlendRecorder.THE_SURFRSTS.Length * 16 - 5);
    }

    [Fact]
    public void RstRstLineBuilder_MatchesNativeBitForBit()
    {
        Compare(k => BlendRecorder.THE_RSTRSTS.Any(c => Is(k, c.Name)), BlendRecorder.THE_RSTRSTS.Length * 4 - 2);
    }

    [Fact]
    public void Recording_WalksAndApproximates()
    {
        // The recording is not vacuous: the main configurations walk past several points and are approximated.
        Dictionary<string, List<string>> aNative = BlendFuncNativeTests.Recording;
        foreach (string aKey in new[]
                 {
                     "case walk planeplane 1", "case walk planecyl 1", "case walk bspline 2", "case walk cs 1",
                     "case walk surfrst 1 1", "case walk rstrst 4", "case walk surfrstevol 1 7",
                 })
        {
            List<string> aLines = aNative[aKey];
            Assert.Contains("perform 1", aLines);
            string aLine = aLines.First(l => l.StartsWith("line ", StringComparison.Ordinal));
            Assert.True(int.Parse(aLine.Split(' ')[1]) >= 6, aKey);
            Assert.Contains(aLines, l => l == "appsurface 1" || l == "appsurf 1");
        }
    }
}
