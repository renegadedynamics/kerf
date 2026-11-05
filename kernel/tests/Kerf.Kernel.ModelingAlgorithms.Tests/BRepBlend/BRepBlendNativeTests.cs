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
            thePort.Value.Where(kv => theFilter(kv.Key)).ToDictionary(kv => kv.Key, kv => kv.Value);
        BlendFuncNativeTests.Compare(aPort, k => BlendFuncNativeTests.IsWalk(k) && theFilter(k), theExpectedCases);
    }

    private static bool Is(string theKey, string theName)
        => theKey.StartsWith($"case walk {theName} ", StringComparison.Ordinal);

    [Fact]
    public void Walking_SurfSurf_MatchesNativeBitForBit()
    {
        Compare(k => BlendRecorder.THE_WALKS.Any(w => Is(k, w.Name)), BlendRecorder.THE_WALKS.Length * 8);
    }

    [Fact]
    public void Walking_FirstSections_MatchNativeBitForBit()
    {
        Compare(k => Is(k, "firstsection"), 8);
    }

    [Fact]
    public void CSWalking_MatchesNativeBitForBit()
    {
        Compare(k => Is(k, "cs"), 8);
    }

    [Fact]
    public void SurfRstLineBuilder_MatchesNativeBitForBit()
    {
        Compare(k => BlendRecorder.THE_SURFRSTS.Any(c => Is(k, c.Name)), BlendRecorder.THE_SURFRSTS.Length * 16);
    }

    [Fact]
    public void RstRstLineBuilder_MatchesNativeBitForBit()
    {
        Compare(k => BlendRecorder.THE_RSTRSTS.Any(c => Is(k, c.Name)), BlendRecorder.THE_RSTRSTS.Length * 4);
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
