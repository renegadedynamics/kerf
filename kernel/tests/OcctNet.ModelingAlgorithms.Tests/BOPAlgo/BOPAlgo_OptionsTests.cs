// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of BOPAlgo_Options, BOPAlgo_Algo and the alerts (no GTests in OCCT).

using Kerf.Engine.Booleans;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Engine.Booleans.GlueMode;
using static Kerf.Engine.Booleans.Operation;
using static Kerf.Engine.Foundation.Gravity;

namespace OcctNet.ModelingAlgorithms.Tests.BOPAlgo;

/// <summary>An algorithm with four steps: two constant ones (20% and 10%) and two in the ratio 1:3.</summary>
internal sealed class TestBOPAlgo : BooleanAlgorithm
{
    public NCollection_Array1<double>? Steps;

    public override void Perform(global::Kerf.Engine.Foundation.ProgressRange theRange = default)
    {
        PISteps aSteps = new(4);
        AnalyzeProgress(100.0, aSteps);
        Steps = aSteps.Steps();
        CheckData();
    }

    protected override void FillPIConstants(double theWhole, PISteps theSteps)
    {
        theSteps.SetStep(0, 0.2 * theWhole);
        theSteps.SetStep(2, 0.1 * theWhole);
        theSteps.SetStep(7, 1.0); // out of range: ignored
    }

    protected override void FillPISteps(PISteps theSteps)
    {
        theSteps.SetStep(1, 1.0);
        theSteps.SetStep(3, 3.0);
    }

    public bool TestUserBreak(global::Kerf.Engine.Foundation.ProgressScope thePS) => UserBreak(thePS);
}

public class BOPAlgo_OptionsTests
{
    [Fact]
    public void Defaults()
    {
        Options anOptions = new();
        Assert.False(Options.GetParallelMode());
        Assert.False(anOptions.RunParallel());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), anOptions.FuzzyValue());
        Assert.False(anOptions.UseOBB());
        Assert.False(anOptions.HasErrors);
        Assert.False(anOptions.HasWarnings);
        Assert.NotNull(anOptions.GetReport());
    }

    [Fact]
    public void Setters()
    {
        Options anOptions = new();
        anOptions.SetRunParallel(true);
        Assert.True(anOptions.RunParallel());
        anOptions.SetUseOBB(true);
        Assert.True(anOptions.UseOBB());
        anOptions.SetFuzzyValue(1.0e-3);
        Assert.Equal(1.0e-3, anOptions.FuzzyValue());
        // the fuzzy value is at least Precision::Confusion()
        anOptions.SetFuzzyValue(1.0e-12);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), anOptions.FuzzyValue());
        anOptions.SetFuzzyValue(-1.0);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), anOptions.FuzzyValue());
    }

    [Fact]
    public void ErrorsAndWarnings()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Options anOptions = new();
        anOptions.AddWarning(new AlertTooSmallEdge(anEdge));
        Assert.True(anOptions.HasWarnings);
        Assert.False(anOptions.HasErrors);
        Assert.True(anOptions.HasWarning(typeof(AlertTooSmallEdge)));
        Assert.False(anOptions.HasError(typeof(AlertTooSmallEdge)));

        anOptions.AddError(new AlertSelfInterferingShape(anEdge));
        anOptions.AddError(new AlertSelfInterferingShape(anEdge)); // shape alerts don't merge
        Assert.True(anOptions.HasErrors);
        Assert.True(anOptions.HasError(typeof(AlertSelfInterferingShape)));
        Assert.False(anOptions.HasError(typeof(AlertWithShape)));
        Assert.Equal(2, anOptions.GetReport().GetAlerts(Fail).Extent());
        var anAlert = Assert.IsType<AlertSelfInterferingShape>(anOptions.GetReport().GetAlerts(Fail).First());
        Assert.True(anAlert.GetShape().IsSame(anEdge));

        StringWriter anErrors = new();
        anOptions.DumpErrors(anErrors);
        Assert.Equal("BOPAlgo_AlertSelfInterferingShape\nBOPAlgo_AlertSelfInterferingShape\n", anErrors.ToString());
        StringWriter aWarnings = new();
        anOptions.DumpWarnings(aWarnings);
        Assert.Equal("BOPAlgo_AlertTooSmallEdge\n", aWarnings.ToString());

        anOptions.ClearWarnings();
        Assert.False(anOptions.HasWarnings);
        Assert.True(anOptions.HasErrors);
        anOptions.Clear();
        Assert.False(anOptions.HasErrors);
    }

    [Fact]
    public void SimpleErrorAddedTwice_OcctIssue()
    {
        // Message_Report's merge removes the first alert (occt-issues.md, unit 6A): the same simple error added twice
        // leaves no error, as in OCCT
        Options anOptions = new();
        anOptions.AddError(new AlertTooFewArguments());
        Assert.True(anOptions.HasErrors);
        anOptions.AddError(new AlertTooFewArguments());
        Assert.False(anOptions.HasErrors);
    }

    [Fact]
    public void UserBreak_AddsTheAlert()
    {
        TestBOPAlgo anAlgo = new();
        using (global::Kerf.Engine.Foundation.ProgressScope aScope = new(new global::Kerf.Engine.Foundation.ProgressIndicator().Start(), "test", 1))
        {
            Assert.False(anAlgo.TestUserBreak(aScope));
        }
        Assert.False(anAlgo.HasErrors);

        using CancellationTokenSource aSource = new();
        aSource.Cancel();
        using (global::Kerf.Engine.Foundation.ProgressScope aScope = new(new global::Kerf.Engine.Foundation.ProgressIndicator(aSource.Token).Start(), "test", 1))
        {
            Assert.True(anAlgo.TestUserBreak(aScope));
        }
        Assert.True(anAlgo.HasError(typeof(AlertUserBreak)));
    }

    [Fact]
    public void AnalyzeProgress_Normalises()
    {
        TestBOPAlgo anAlgo = new();
        anAlgo.AddError(new AlertBuilderFailed());
        anAlgo.Perform();
        // the constant steps keep their values, the others share the remaining 70 in the ratio 1:3
        Assert.Equal(20.0, anAlgo.Steps![0], 12);
        Assert.Equal(17.5, anAlgo.Steps[1], 12);
        Assert.Equal(10.0, anAlgo.Steps[2], 12);
        Assert.Equal(52.5, anAlgo.Steps[3], 12);
        // CheckData clears the errors
        Assert.False(anAlgo.HasErrors);

        PISteps aSteps = new(2);
        Assert.Equal(0.0, aSteps.GetStep(5));
        Assert.Equal(0.0, aSteps.GetStep(-1));
    }

    [Fact]
    public void MessageTexts()
    {
        _ = new Options();
        Assert.True(global::Kerf.Engine.Foundation.MsgFile.HasMsg("BOPAlgo_LOAD_CHECKER"));
        Assert.Equal("Some of the arguments are self-interfering shapes",
                     global::Kerf.Engine.Foundation.MsgFile.Msg(new AlertSelfInterferingShape(new Shape()).GetMessageKey()));
        Assert.Equal("Unable to make closed edge on face.", global::Kerf.Engine.Foundation.MsgFile.Msg("BOPAlgo_AlertUnableToMakeClosedEdgeOnFace"));
    }

    [Fact]
    public void SectionAttribute_IsAnEquatableValue()
    {
        SectionAttribute aDefault = new();
        Assert.True(aDefault.Approximation() && aDefault.PCurveOnS1() && aDefault.PCurveOnS2());
        SectionAttribute anOther = aDefault.WithApproximation(false).WithPCurveOnS2(false);
        Assert.False(anOther.Approximation());
        Assert.True(anOther.PCurveOnS1());
        Assert.False(anOther.PCurveOnS2());
        Assert.True(aDefault.Approximation()); // unchanged
        Assert.Equal(new SectionAttribute(false, true, false), anOther);
        Assert.NotEqual(aDefault, anOther);
        Assert.Equal(new SectionAttribute(false, true, false).GetHashCode(), anOther.GetHashCode());
    }

    [Fact]
    public void Enums()
    {
        Assert.Equal(0, (int)GlueMode.GlueOff);
        Assert.Equal(2, (int)GlueMode.GlueFull);
        Assert.Equal(3, (int)Operation.CUT21);
        Assert.Equal(5, (int)Operation.Unknown);
        Assert.Equal(11, (int)CheckStatus.NotValid);
    }
}

/// <summary>The global parallel mode is process-wide: its test runs alone.</summary>
[Collection(nameof(BOPAlgoGlobalStateCollection))]
public class BOPAlgo_GlobalParallelModeTests
{
    [Fact]
    public void GlobalParallelMode_IsTheDefaultOfNewAlgorithms()
    {
        Assert.False(Options.GetParallelMode());
        try
        {
            Options.SetParallelMode(true);
            Assert.True(new Options().RunParallel());
        }
        finally
        {
            Options.SetParallelMode(false);
        }
        Assert.False(new Options().RunParallel());
    }
}

[CollectionDefinition(nameof(BOPAlgoGlobalStateCollection), DisableParallelization = true)]
public sealed class BOPAlgoGlobalStateCollection;
