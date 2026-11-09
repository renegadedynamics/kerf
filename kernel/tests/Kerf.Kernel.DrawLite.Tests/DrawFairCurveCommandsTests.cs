// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Fairing;
using Kerf.Geometry;
using Kerf.Engine;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 5Q's fair curve commands (GeometryTest_FairCurveCommands.cxx): <c>battencurve</c>, <c>minvarcurve</c> and the
/// editing commands give the curves FairCurve gives through the same calls (which tools/fairrec's recordings compare
/// with native OCCT bit for bit, Draw's sequences included), the batten is a 2D curve to other commands, Draw's
/// messages, and the corpus (no case of OCCT's tests uses these commands).
/// </summary>
public class DrawFairCurveCommandsTests(ITestOutputHelper output)
{
    private static DrawInterpreter New() => new(new StringWriter { NewLine = "\n" });

    private static string Error(DrawInterpreter theDI, string theScript) =>
        Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    private static DrawFairCurve Batten(DrawInterpreter theDI, string theName) => Assert.IsType<DrawFairCurve>(theDI.GetDrawable(theName));

    private static void SameCurve(BSplineCurve2D theExpected, BSplineCurve2D theActual)
    {
        Assert.Equal(theExpected.Degree, theActual.Degree);
        Assert.Equal(theExpected.Poles.ToArray(), theActual.Poles.ToArray());
        Assert.Equal(theExpected.Knots.ToArray(), theActual.Knots.ToArray());
        Assert.Equal(theExpected.Multiplicities.ToArray(), theActual.Multiplicities.ToArray());
    }

    [Fact]
    public void Battencurve_computes_as_Draw()
    {
        DrawInterpreter i = New();
        i.Eval("point p1 0 0; point p2 10 0; battencurve p1 p2 30 -20 0.5 b");
        DrawFairCurve b = Batten(i, "b");
        Assert.Equal("DrawFairCurve_Batten", b.TypeName);

        // Draw computes with the defaults, then DrawFairCurve_Batten's constructor computes again with 1.0e-2
        Batten aBat = new(new Point2D(0, 0), new Point2D(10, 0), 0.5);
        aBat.SetAngle1(30 * Math.PI / 180);
        aBat.SetAngle2(-20 * Math.PI / 180);
        Assert.True(aBat.Compute(out AnalysisCode aCode));
        Assert.Equal(AnalysisCode.OK, aCode);
        aBat.Compute(out _, 50, 1.0e-2);
        SameCurve(aBat.Curve(), b.Curve);
        Assert.True(b.Curve.IsFrozen);

        // the editing commands: each changes the batten and computes again
        i.Eval("setangle 2 10 b; freeslide b; setheight 0.8 b; setslope 0.01 b; point p3 9 1; setpoint 2 p3 b; freeangle 1 b; setslide 1.1 b");
        aBat.SetAngle2(10 * Math.PI / 180);
        aBat.Compute(out _, 50, 1.0e-2);
        aBat.SetFreeSliding(true);
        aBat.Compute(out _, 50, 1.0e-2);
        aBat.SetHeight(0.8);
        aBat.Compute(out _, 50, 1.0e-2);
        aBat.SetSlope(0.01);
        aBat.Compute(out _, 50, 1.0e-2);
        aBat.SetP2(new Point2D(9, 1));
        aBat.Compute(out _, 50, 1.0e-2);
        aBat.SetConstraintOrder1(0);
        aBat.Compute(out _, 50, 1.0e-2);
        aBat.SetFreeSliding(false);
        aBat.SetSlidingFactor(1.1);
        aBat.Compute(out _, 50, 1.0e-2);
        SameCurve(aBat.Curve(), b.Curve);
        Assert.Equal(0, b.Batten.GetConstraintOrder1());
        Assert.Equal(1.1, b.Batten.GetSlidingFactor());

        // setangle on a free side constrains it again
        i.Eval("setangle 1 5 b");
        Assert.Equal(1, b.Batten.GetConstraintOrder1());
    }

    [Fact]
    public void Minvarcurve_computes_as_Draw()
    {
        DrawInterpreter i = New();
        i.Eval("point p1 0 0; point p2 8 2; minvarcurve p1 p2 20 40 0.3 m");
        DrawFairCurve m = Batten(i, "m");
        Assert.Equal("DrawFairCurve_MinimalVariation", m.TypeName);

        MinimalVariation aMVC = new(new Point2D(0, 0), new Point2D(8, 2), 0.3);
        aMVC.SetAngle1(20 * Math.PI / 180);
        aMVC.SetAngle2(40 * Math.PI / 180);
        aMVC.Compute(out _);
        aMVC.Compute(out _, 50, 1.0e-2);
        SameCurve(aMVC.Curve(), m.Curve);

        i.Eval("setcurvature 1 0.1 m; setphysicalratio 0.5 m; setcurvature 2 -0.05 m; freecurvature 1 m");
        aMVC.SetCurvature1(0.1);
        aMVC.SetConstraintOrder1(2);
        aMVC.Compute(out _, 50, 1.0e-2);
        aMVC.SetPhysicalRatio(0.5);
        aMVC.Compute(out _, 50, 1.0e-2);
        aMVC.SetCurvature2(-0.05);
        aMVC.SetConstraintOrder2(2);
        aMVC.Compute(out _, 50, 1.0e-2);
        aMVC.SetConstraintOrder1(1);
        aMVC.Compute(out _, 50, 1.0e-2);
        SameCurve(aMVC.Curve(), m.Curve);
        Assert.Equal(-0.05, ((MinimalVariation)m.Batten).GetCurvature2());
        Assert.Equal(0.5, ((MinimalVariation)m.Batten).GetPhysicalRatio());

        // the batten commands apply to a minimal variation curve too
        i.Eval("setangle 1 25 m");
        Assert.Equal(25 * Math.PI / 180, m.Batten.GetAngle1());
    }

    [Fact]
    public void A_batten_is_a_2d_curve_to_other_commands()
    {
        DrawInterpreter i = New();
        i.Eval("point p1 0 0; point p2 10 0; battencurve p1 p2 30 -20 0.5 b");
        Assert.Contains("2d curve", i.Eval("dtyp b"), StringComparison.Ordinal);

        // dump: the curve, then FairCurve_Batten::Dump
        string aDump = i.Eval("dump b");
        Assert.Contains("BSplineCurve", aDump, StringComparison.Ordinal);
        Assert.Contains("  Batten       |Old  |   New\n", aDump, StringComparison.Ordinal);
        Assert.Contains(" FreeSliding   |      0 | 0\n", aDump, StringComparison.Ordinal);
        Assert.Contains("AnalysisCode : Ok\n", aDump, StringComparison.Ordinal);

        // copy gives a plain B-spline
        i.Eval("copy b c");
        Assert.Same(Batten(i, "b").Curve, Assert.IsType<DrawGeometry>(i.GetDrawable("c")).Geometry);

        // the 2D transformations read the batten's curve (Draw-lite binds the transformed curve to the name)
        BSplineCurve2D aCurve = Batten(i, "b").Curve;
        i.Eval("2dtranslate b 1 2");
        var aMoved = Assert.IsType<BSplineCurve2D>(Assert.IsType<DrawGeometry>(i.GetDrawable("b")).Geometry);
        Assert.Equal(aCurve.Pole(3).X + 1, aMoved.Pole(3).X, 1e-12);
        Assert.Equal(aCurve.Pole(3).Y + 2, aMoved.Pole(3).Y, 1e-12);
    }

    [Fact]
    public void Fair_curve_commands_messages_as_Draw()
    {
        DrawInterpreter i = New();
        i.Eval("point p1 0 0; point p2 10 0; battencurve p1 p2 30 -20 0.5 b");
        Assert.Equal("Waiting 6 arguments\n", Error(i, "battencurve p1 p2 30 -20 b"));
        Assert.Equal("Waiting 6 arguments\n", Error(i, "minvarcurve p1 p2 30 -20 b"));
        Assert.Equal("Waiting 3 arguments\n", Error(i, "setangle 1 b"));
        Assert.Equal("Waiting 1 arguments\n", Error(i, "freeslide"));
        Assert.Equal("", Error(i, "battencurve nothing p2 30 -20 0.5 x"));
        Assert.Equal("", Error(i, "setslope 0.1 nothing"));
        // a batten is not a minimal variation curve
        Assert.Equal("", Error(i, "setcurvature 1 0.1 b"));
        Assert.Equal("An exception was caught Standard_NullValue: FairCurve : P1 and P2 are confused",
                     Error(i, "battencurve p1 p1 30 -20 0.5 x"));
        Assert.Equal("An exception was caught Standard_NegativeValue: FairCurve : Height is no positive",
                     Error(i, "setheight -1 b"));
        i.Eval("minvarcurve p1 p2 10 10 0.5 m");
        Assert.StartsWith("An exception was caught Standard_DomainError", Error(i, "setphysicalratio 2 m"), StringComparison.Ordinal);
    }

    [Fact]
    public void Corpus_has_no_case_using_the_fair_curve_commands()
    {
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        Assert.True(Directory.Exists(aTests), $"No OCCT source at {aTests}: run tools/get-occt.ps1");
        string[] aNames = ["battencurve", "minvarcurve", "setpoint", "setangle", "setslide", "freeangle", "freeslide",
                           "setheight", "setslope", "setcurvature", "freecurvature", "setphysicalratio"];
        List<DrawCaseScan> aScans = [.. SharedCorpus.Scans.Where(s => aNames.Any(s.Commands.Contains))];
        output.WriteLine($"{aScans.Count} cases use the fair curve commands");
        Assert.Empty(aScans);
    }
}
