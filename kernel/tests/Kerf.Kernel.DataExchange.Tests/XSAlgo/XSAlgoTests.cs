// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Kerf.Kernel.DataExchange.Tests.Interface;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DataExchange.Tests.XSAlgo;

/// <summary>XSAlgo's statics, parameter maps, processor and container (the native comparison is XSAlgoNativeTests).</summary>
[Collection(ProcessWideMessages.Name)]
public class XSAlgoTests
{
    [Fact]
    public void Init_declares_the_statics_once_per_set()
    {
        var aStatics = new global::Kerf.Engine.Exchange.Framework.StaticSet();
        Kerf.Engine.Exchange.Framework.XSAlgo.Init(aStatics);
        Assert.True(aStatics.IsPresent("read.precision.mode")); // Standards
        Assert.Equal(0, aStatics.IVal("read.stdsameparameter.mode"));
        Assert.Equal("Off", aStatics.CVal("read.stdsameparameter.mode"));
        Assert.Equal("MM", aStatics.CVal("xstep.cascade.unit"));
        Assert.Equal(2, aStatics.IVal("xstep.cascade.unit"));
        aStatics.SetCVal("xstep.cascade.unit", "M");
        Kerf.Engine.Exchange.Framework.XSAlgo.Init(aStatics); // once per set: the value stays
        Assert.Equal(6, aStatics.IVal("xstep.cascade.unit"));
        var anOther = new global::Kerf.Engine.Exchange.Framework.StaticSet();
        Kerf.Engine.Exchange.Framework.XSAlgo.Init(anOther);
        Assert.Equal(2, anOther.IVal("xstep.cascade.unit")); // sets are independent
        Assert.Same(Kerf.Engine.Exchange.Framework.XSAlgo.AlgoContainer(), Kerf.Engine.Exchange.Framework.XSAlgo.AlgoContainer());
        Assert.True(global::Kerf.Engine.Healing.Process.ShapeProcess.FindOperator("FixShape", out _));
    }

    [Fact]
    public void Parameter_maps()
    {
        var aDefaults = new ShapeFixParameters();
        Assert.Equal(1.0e-6, aDefaults.Tolerance3d);
        Assert.Equal(ShapeType.Vertex, aDefaults.DetalizationLevel);
        Assert.Equal(ShapeFixParameters.FixMode.NotFix, aDefaults.CreateOpenSolidMode);
        Assert.Equal(aDefaults, new ShapeFixParameters());
        Assert.NotEqual(aDefaults, aDefaults with { FixWireMode = ShapeFixParameters.FixMode.Fix });

        var aMap = new Dictionary<string, string>();
        global::Kerf.Engine.Exchange.Framework.ShapeProcessor.FillParameterMap(aDefaults, false, aMap);
        Assert.Equal(54, aMap.Count); // the native map's hash is checked by XSAlgoNativeTests
        Assert.Equal("1e-06", aMap.GetValue("FixShape.Tolerance3d"));
        Assert.Equal("1", aMap.GetValue("FixShape.MaxTolerance3d"));
        Assert.Equal("7", aMap.GetValue("DetalizationLevel"));
        Assert.Equal("0", aMap.GetValue("NonManifold"));
        Assert.Equal("-1", aMap.GetValue("FixShape.FixFreeShellMode"));
        Assert.Equal("1", aMap.GetValue("FixShape.AutoCorrectPrecisionMode"));

        // SetParameter replaces only when asked
        global::Kerf.Engine.Exchange.Framework.ShapeProcessor.SetParameter("FixShape.Tolerance3d", 0.01, false, aMap);
        Assert.Equal("1e-06", aMap.GetValue("FixShape.Tolerance3d"));
        global::Kerf.Engine.Exchange.Framework.ShapeProcessor.SetParameter("FixShape.Tolerance3d", 0.01, true, aMap);
        Assert.Equal("0.01", aMap.GetValue("FixShape.Tolerance3d"));
        global::Kerf.Engine.Exchange.Framework.ShapeProcessor.SetParameter("New", "x", false, aMap);
        Assert.Equal("x", aMap.GetValue("New"));

        // SetShapeFixParameters: the parameters replace, the additional ones are only added
        var anAdditional = new Dictionary<string, string>();
        anAdditional["FixShape.FixWireMode"] = "0";
        anAdditional["Extra"] = "1";
        var aTarget = new Dictionary<string, string>();
        aTarget["FixShape.FixWireMode"] = "5";
        global::Kerf.Engine.Exchange.Framework.ShapeProcessor.SetShapeFixParameters(aDefaults with { FixWireMode = ShapeFixParameters.FixMode.Fix },
                                                    anAdditional, aTarget);
        Assert.Equal("1", aTarget.GetValue("FixShape.FixWireMode"));
        Assert.Equal("1", aTarget.GetValue("Extra"));
    }

    [Fact]
    public void The_actors_keep_copies_of_their_parameters()
    {
        var anActor = new global::Kerf.Engine.Exchange.Framework.Transfer.ActorOfTransientProcess();
        Assert.Empty(anActor.GetShapeFixParameters());
        Assert.False(anActor.GetProcessingFlags().Item2);
        var aMap = new Dictionary<string, string>();
        aMap["a"] = "1";
        anActor.SetShapeFixParameters(aMap);
        aMap["b"] = "2";
        Assert.Single(anActor.GetShapeFixParameters()); // a copy at the entry
        anActor.GetShapeFixParameters()["c"] = "3";
        Assert.Single(anActor.GetShapeFixParameters()); // a copy at the exit
        anActor.SetShapeFixParameters(new ShapeFixParameters());
        Assert.Equal(55, anActor.GetShapeFixParameters().Count);
        var aFlags = new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.FixShape);
        anActor.SetProcessingFlags(aFlags);
        Assert.Equal((aFlags, true), anActor.GetProcessingFlags());

        var aWriter = new global::Kerf.Engine.Exchange.Framework.Transfer.ActorOfFinderProcess();
        aWriter.SetShapeFixParameters(new ShapeFixParameters(), aMap);
        Assert.Equal(56, aWriter.GetShapeFixParameters().Count);
        aWriter.SetShapeProcessFlags(aFlags);
        Assert.True(aWriter.GetShapeProcessFlags().Item2);
    }

    [Fact]
    public void ReadProcessingData_reads_a_resource_file_or_the_statics()
    {
        global::Kerf.Engine.Exchange.Framework.StaticSet aStatics = XSAlgoNativeTests.StepStatics();
        (Dictionary<string, string> aParams, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags aFlags) =
            global::Kerf.Engine.Exchange.Framework.ShapeProcessor.ReadProcessingData("read.step.resource.name", "read.step.sequence", aStatics);
        Assert.True(aFlags.Test(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.FixShape));
        Assert.Equal(1, aFlags.Count());
        Assert.Equal("&Runtime.Tolerance", aParams.GetValue("FixShape.Tolerance3d"));
        Assert.False(aParams.ContainsKey("exec.op"));

        // no such resource file: the statics' values, in the scope named by a static
        aStatics.Init("test", "test.resource", 't', "NoSuchResource");
        aStatics.Init("test", "test.scope", 't', "xstep");
        (aParams, aFlags) = global::Kerf.Engine.Exchange.Framework.ShapeProcessor.ReadProcessingData("test.resource", "test.scope", aStatics);
        Assert.Equal(0, aFlags.Count());
        Assert.Equal("MM", aParams.GetValue("cascade.unit"));
    }

    [Fact]
    public void The_processor_never_changes_its_input()
    {
        Shape aBox = new MakeBox(1.0, 2.0, 3.0).Shape();
        ulong aBefore = XSAlgoNativeTests.BRepHash(aBox);
        var aParams = new Dictionary<string, string>();
        aParams["DetalizationLevel"] = ((int)ShapeType.Face).ToString(System.Globalization.CultureInfo.InvariantCulture);
        aParams["NonManifold"] = "1";
        var aProcessor = new global::Kerf.Engine.Exchange.Framework.ShapeProcessor(aParams);
        Assert.Null(aProcessor.GetContext());
        Shape aResult = aProcessor.ProcessShape(aBox, new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.SplitCommonVertex,
                                                                                               global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.DirectFaces));
        Assert.Equal(aBefore, XSAlgoNativeTests.BRepHash(aBox));
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Face, aProcessor.GetContext()!.GetDetalisation());
        Assert.True(aProcessor.GetContext()!.IsNonManifold);
        Assert.True(aProcessor.ProcessShape(new Shape(), new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags()).IsNull);

        // merging an empty history changes nothing
        var TP = new global::Kerf.Engine.Exchange.Framework.Transfer.TransientProcess();
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetShapeResult(TP, new TCollection_HAsciiString("x"), aBox);
        aProcessor.MergeTransferInfo(TP, 1);
        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(TP, TP.Mapped(1)).IsSame(aBox));
        new global::Kerf.Engine.Exchange.Framework.ShapeProcessor(new ShapeFixParameters()).MergeTransferInfo(TP, 1); // no context: nothing
        new global::Kerf.Engine.Exchange.Framework.ShapeProcessor(new ShapeFixParameters()).MergeTransferInfo(new global::Kerf.Engine.Exchange.Framework.Transfer.FinderProcess());
    }

    [Fact]
    public void The_container_runs_a_sequence_or_default_fixes()
    {
        global::Kerf.Engine.Exchange.Framework.StaticSet aStatics = XSAlgoNativeTests.StepStatics();
        Shape aBox = new MakeBox(1.0, 2.0, 3.0).Shape();
        Transient? anInfo = null;
        global::Kerf.Engine.Exchange.Framework.FrameworkAlgoContainer aContainer = Kerf.Engine.Exchange.Framework.XSAlgo.AlgoContainer();
        Shape aResult = aContainer.ProcessShape(aBox, 1e-3, 1.0, "read.step.resource.name", "read.step.sequence", ref anInfo,
                                                       aStatics);
        var aContext = Assert.IsType<global::Kerf.Engine.Healing.Process.ShapeContext>(anInfo);
        Assert.Equal(ShapeType.Vertex, aContext.GetDetalisation());
        Assert.Equal("0.001", aContext.ResourceManager()!.Value("Runtime.Tolerance"));
        Assert.True(aResult.IsSame(aBox) || aResult.TShape()!.IsFrozen);

        // the context is reused
        Transient? aSame = anInfo;
        aContainer.ProcessShape(aBox, 1e-3, 1.0, "read.step.resource.name", "write.none", ref anInfo, aStatics, default, true);
        Assert.Same(aSame, anInfo);
        Assert.True(aContext.IsNonManifold);
        Assert.Equal("DirectFaces", aContext.ResourceManager()!.Value(".exec.op"));
        Assert.True(aContainer.ProcessShape(new Shape(), 0, 0, "", "", ref anInfo, aStatics).IsNull);

        // CheckPCurve through the container, on a box's working copy
        Shape aCopy = TestCopies.ExactCopyForWrite(aBox, out _);
        var aFace = ShapeCasts.Face(new Explorer(aCopy, ShapeType.Face).Current());
        var anEdge = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        Assert.True(aContainer.CheckPCurve(anEdge, aFace, 1e-3, false, aStatics));
        // and not on a published shape
        var aFace2 = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        var anEdge2 = ShapeCasts.Edge(new Explorer(aFace2, ShapeType.Edge).Current());
        Assert.ThrowsAny<FailureException>(() => aContainer.CheckPCurve(anEdge2, aFace2, 1e-3, false, aStatics));
    }
}
