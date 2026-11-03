// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Healing.Process;
using Kerf.Kernel.DataExchange.Tests.StepGen;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests.STEPControl;

/// <summary>
/// STEP import's healing (its "FromSTEP" shape processing: ShapeFix_Shape and the operators the resource file names)
/// on a frozen shape leaves it untouched (Kerf unit 10H4): the operators work in a working overlay, not on a copy.
/// </summary>
public class StepHealingInputsUnchangedTests
{
    private static string DataStep(string theName) =>
        Path.Combine(StepRoundTripTests.Metadata("OcctSourceDir"), "data", "step", theName);

    [Theory]
    [InlineData("screw.step")]
    [InlineData("linkrods.step")]
    public void The_import_sequence_leaves_a_frozen_shape_untouched(string theName)
    {
        string aPath = DataStep(theName);
        if (!File.Exists(aPath))
        {
            return;
        }
        var aReader = new StepReader();
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadFile(aPath));
        aReader.TransferRoots();
        Shape aShape = aReader.OneShape();
        Assert.True(aShape.TShape()!.IsFrozen);
        ShapeSnapshot aSnapshot = ShapeSnapshot.Of(aShape);

        OperLibrary.Init();
        var aContext = new ShapeContext(aShape, "STEP");
        ShapeProcess.Perform(aContext, "FromSTEP");
        Shape aResult = aContext.Result();

        aSnapshot.AssertUnchanged(aShape);
        Assert.False(aResult.IsNull);
        Assert.True(aResult.TShape()!.IsFrozen);
    }
}
