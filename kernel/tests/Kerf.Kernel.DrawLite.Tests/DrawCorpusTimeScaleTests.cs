// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// The corpus's time scale (decision 1669): coverage runs multiply the runner's limits, so instrumentation alone fails no
/// case, a hung case still stops, and unset nothing changes. The cases are small scripts in a corpus of their own.
/// </summary>
public sealed class DrawCorpusTimeScaleTests : IDisposable
{
    private readonly string myTests = Directory.CreateTempSubdirectory("kerf-time-scale").FullName;

    public void Dispose() => Directory.Delete(myTests, true);

    private DrawTestCase Case(string theName, string theScript)
    {
        string aGrid = Path.Combine(myTests, "bugs", "modalg_5");
        Directory.CreateDirectory(aGrid);
        File.Copy(Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests", "parse.rules"), Path.Combine(myTests, "parse.rules"), true);
        string aPath = Path.Combine(aGrid, theName);
        File.WriteAllText(aPath, theScript + "\nputs \"TEST COMPLETED\"\n");
        return new DrawTestCase("bugs", "modalg_5", theName, aPath);
    }

    private DrawCaseRun Run(DrawTestCase theCase, double theScale, double theLimitSeconds)
        => new DrawCorpus(myTests) { TimeScale = theScale }
            .RunAndCheck(theCase, DrawResourcesTests.ResourcesDirectory, null, TimeSpan.FromSeconds(theLimitSeconds));

    [Theory]
    [InlineData(null, 1.0)]
    [InlineData("", 1.0)]
    [InlineData("1", 1.0)]
    [InlineData("2.5", 2.5)]
    public void The_scale_comes_from_the_variable_and_is_1_when_unset(string? theValue, double theScale)
        => Assert.Equal(theScale, DrawCorpus.TimeScaleFrom(theValue));

    [Theory]
    [InlineData("five")]
    [InlineData("0")]
    [InlineData("-2")]
    [InlineData("Infinity")]
    public void A_value_that_isnt_a_positive_number_is_refused(string theValue)
        => Assert.Contains(DrawCorpus.TimeScaleVariable, Assert.Throws<InvalidOperationException>(() => DrawCorpus.TimeScaleFrom(theValue)).Message, StringComparison.Ordinal);

    [Fact]
    public void A_scaled_limit_still_stops_a_hung_case_but_no_sooner_than_the_limit_times_the_scale()
    {
        DrawTestCase aHung = Case("hung", "while 1 {}");
        var aClock = Stopwatch.StartNew();
        DrawCaseRun aRun = Run(aHung, 5, 0.2);
        aClock.Stop();

        Assert.Contains("Process killed by elapsed limit", aRun.Log, StringComparison.Ordinal);
        Assert.False(aRun.Passed, aRun.Status);
        Assert.True(aClock.Elapsed >= TimeSpan.FromSeconds(1), $"stopped after {aClock.Elapsed}, before 0.2 s x 5");
    }

    [Fact]
    public void A_case_outlasting_the_limit_is_killed_at_scale_1_and_finishes_when_scaled()
    {
        // a wall-clock wait of 1.5 s against a limit of 1 s, or of 1 s x 1000
        DrawTestCase aSlow = Case("slow", "set t [clock milliseconds]\nwhile {[clock milliseconds] - $t < 1500} {}\nputs done");

        DrawCaseRun aKilled = Run(aSlow, 1, 1);
        Assert.Contains("Process killed by elapsed limit", aKilled.Log, StringComparison.Ordinal);
        Assert.False(aKilled.Passed, aKilled.Status);

        DrawCaseRun aFinished = Run(aSlow, 1000, 1);
        Assert.DoesNotContain("Process killed", aFinished.Log, StringComparison.Ordinal);
        Assert.True(aFinished.Passed, aFinished.Status);
    }

    private const string LowPerformance = "puts \"Error: low performance\"";

    [Fact]
    public void The_bug24157_time_check_is_passed_over_only_above_scale_1()
    {
        DrawTestCase aCase = Case("bug24157_1", LowPerformance);

        Assert.False(Run(aCase, 1, 60).Passed);
        Assert.False(Run(aCase, 0.5, 60).Passed);
        DrawCaseRun aScaled = Run(aCase, 5, 60);
        Assert.True(aScaled.Passed, aScaled.Status);
        Assert.Contains("Passed over at time scale 5", aScaled.Log, StringComparison.Ordinal);
        Assert.DoesNotContain("low performance", aScaled.Log, StringComparison.Ordinal);
    }

    [Fact]
    public void Passing_over_the_time_check_hides_nothing_else()
    {
        // another case's identical line is not passed over; nor is another error in a bug24157 case
        Assert.False(Run(Case("bug99999", LowPerformance), 5, 60).Passed);
        Assert.False(Run(Case("bug24157_2", LowPerformance + "\nputs \"Error: wrong volume\""), 5, 60).Passed);
    }
}
