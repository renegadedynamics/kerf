// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using OcctNet.DataExchange.Tests.Interface;
using static Kerf.Topology.ShapeType;

namespace OcctNet.DataExchange.Tests.XSAlgo;

/// <summary>XSAlgo's statics, parameter maps, processor and container (the native comparison is XSAlgoNativeTests).</summary>
[Collection(ProcessWideMessages.Name)]
public class XSAlgoTests
{
    [Fact]
    public void Init_declares_the_statics_once_per_set()
    {
        var aStatics = new Interface_StaticSet();
        Occt.XSAlgo.Init(aStatics);
        Assert.True(aStatics.IsPresent("read.precision.mode")); // Standards
        Assert.Equal(0, aStatics.IVal("read.stdsameparameter.mode"));
        Assert.Equal("Off", aStatics.CVal("read.stdsameparameter.mode"));
        Assert.Equal("MM", aStatics.CVal("xstep.cascade.unit"));
        Assert.Equal(2, aStatics.IVal("xstep.cascade.unit"));
        aStatics.SetCVal("xstep.cascade.unit", "M");
        Occt.XSAlgo.Init(aStatics); // once per set: the value stays
        Assert.Equal(6, aStatics.IVal("xstep.cascade.unit"));
        var anOther = new Interface_StaticSet();
        Occt.XSAlgo.Init(anOther);
        Assert.Equal(2, anOther.IVal("xstep.cascade.unit")); // sets are independent
        Assert.Same(Occt.XSAlgo.AlgoContainer(), Occt.XSAlgo.AlgoContainer());
        Assert.True(global::Kerf.Engine.Healing.Process.ShapeProcess.FindOperator("FixShape", out _));
    }

    [Fact]
    public void Parameter_maps()
    {
        var aDefaults = new DE_ShapeFixParameters();
        Assert.Equal(1.0e-6, aDefaults.Tolerance3d);
        Assert.Equal(ShapeType.Vertex, aDefaults.DetalizationLevel);
        Assert.Equal(DE_ShapeFixParameters.FixMode.NotFix, aDefaults.CreateOpenSolidMode);
        Assert.Equal(aDefaults, new DE_ShapeFixParameters());
        Assert.NotEqual(aDefaults, aDefaults with { FixWireMode = DE_ShapeFixParameters.FixMode.Fix });

        var aMap = new NCollection_DataMap<string, string>();
        XSAlgo_ShapeProcessor.FillParameterMap(aDefaults, false, aMap);
        Assert.Equal(54, aMap.Extent()); // the native map's hash is checked by XSAlgoNativeTests
        Assert.Equal("1e-06", aMap.Find("FixShape.Tolerance3d"));
        Assert.Equal("1", aMap.Find("FixShape.MaxTolerance3d"));
        Assert.Equal("7", aMap.Find("DetalizationLevel"));
        Assert.Equal("0", aMap.Find("NonManifold"));
        Assert.Equal("-1", aMap.Find("FixShape.FixFreeShellMode"));
        Assert.Equal("1", aMap.Find("FixShape.AutoCorrectPrecisionMode"));

        // SetParameter replaces only when asked
        XSAlgo_ShapeProcessor.SetParameter("FixShape.Tolerance3d", 0.01, false, aMap);
        Assert.Equal("1e-06", aMap.Find("FixShape.Tolerance3d"));
        XSAlgo_ShapeProcessor.SetParameter("FixShape.Tolerance3d", 0.01, true, aMap);
        Assert.Equal("0.01", aMap.Find("FixShape.Tolerance3d"));
        XSAlgo_ShapeProcessor.SetParameter("New", "x", false, aMap);
        Assert.Equal("x", aMap.Find("New"));

        // SetShapeFixParameters: the parameters replace, the additional ones are only added
        var anAdditional = new NCollection_DataMap<string, string>();
        anAdditional.Bind("FixShape.FixWireMode", "0");
        anAdditional.Bind("Extra", "1");
        var aTarget = new NCollection_DataMap<string, string>();
        aTarget.Bind("FixShape.FixWireMode", "5");
        XSAlgo_ShapeProcessor.SetShapeFixParameters(aDefaults with { FixWireMode = DE_ShapeFixParameters.FixMode.Fix },
                                                    anAdditional, aTarget);
        Assert.Equal("1", aTarget.Find("FixShape.FixWireMode"));
        Assert.Equal("1", aTarget.Find("Extra"));
    }

    [Fact]
    public void The_actors_keep_copies_of_their_parameters()
    {
        var anActor = new Transfer_ActorOfTransientProcess();
        Assert.Equal(0, anActor.GetShapeFixParameters().Extent());
        Assert.False(anActor.GetProcessingFlags().Item2);
        var aMap = new NCollection_DataMap<string, string>();
        aMap.Bind("a", "1");
        anActor.SetShapeFixParameters(aMap);
        aMap.Bind("b", "2");
        Assert.Equal(1, anActor.GetShapeFixParameters().Extent()); // a copy at the entry
        anActor.GetShapeFixParameters().Bind("c", "3");
        Assert.Equal(1, anActor.GetShapeFixParameters().Extent()); // a copy at the exit
        anActor.SetShapeFixParameters(new DE_ShapeFixParameters());
        Assert.Equal(55, anActor.GetShapeFixParameters().Extent());
        var aFlags = new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.FixShape);
        anActor.SetProcessingFlags(aFlags);
        Assert.Equal((aFlags, true), anActor.GetProcessingFlags());

        var aWriter = new Transfer_ActorOfFinderProcess();
        aWriter.SetShapeFixParameters(new DE_ShapeFixParameters(), aMap);
        Assert.Equal(56, aWriter.GetShapeFixParameters().Extent());
        aWriter.SetShapeProcessFlags(aFlags);
        Assert.True(aWriter.GetShapeProcessFlags().Item2);
    }

    [Fact]
    public void ReadProcessingData_reads_a_resource_file_or_the_statics()
    {
        Interface_StaticSet aStatics = XSAlgoNativeTests.StepStatics();
        (NCollection_DataMap<string, string> aParams, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags aFlags) =
            XSAlgo_ShapeProcessor.ReadProcessingData("read.step.resource.name", "read.step.sequence", aStatics);
        Assert.True(aFlags.Test(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.FixShape));
        Assert.Equal(1, aFlags.Count());
        Assert.Equal("&Runtime.Tolerance", aParams.Find("FixShape.Tolerance3d"));
        Assert.False(aParams.IsBound("exec.op"));

        // no such resource file: the statics' values, in the scope named by a static
        aStatics.Init("test", "test.resource", 't', "NoSuchResource");
        aStatics.Init("test", "test.scope", 't', "xstep");
        (aParams, aFlags) = XSAlgo_ShapeProcessor.ReadProcessingData("test.resource", "test.scope", aStatics);
        Assert.Equal(0, aFlags.Count());
        Assert.Equal("MM", aParams.Find("cascade.unit"));
    }

    [Fact]
    public void The_processor_never_changes_its_input()
    {
        Shape aBox = new MakeBox(1.0, 2.0, 3.0).Shape();
        ulong aBefore = XSAlgoNativeTests.BRepHash(aBox);
        var aParams = new NCollection_DataMap<string, string>();
        aParams.Bind("DetalizationLevel", ((int)ShapeType.Face).ToString(System.Globalization.CultureInfo.InvariantCulture));
        aParams.Bind("NonManifold", "1");
        var aProcessor = new XSAlgo_ShapeProcessor(aParams);
        Assert.Null(aProcessor.GetContext());
        Shape aResult = aProcessor.ProcessShape(aBox, new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.SplitCommonVertex,
                                                                                               global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.DirectFaces));
        Assert.Equal(aBefore, XSAlgoNativeTests.BRepHash(aBox));
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Face, aProcessor.GetContext()!.GetDetalisation());
        Assert.True(aProcessor.GetContext()!.IsNonManifold);
        Assert.True(aProcessor.ProcessShape(new Shape(), new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags()).IsNull);

        // merging an empty history changes nothing
        var TP = new Transfer_TransientProcess();
        Occt.TransferBRep.SetShapeResult(TP, new TCollection_HAsciiString("x"), aBox);
        aProcessor.MergeTransferInfo(TP, 1);
        Assert.True(Occt.TransferBRep.ShapeResult(TP, TP.Mapped(1)).IsSame(aBox));
        new XSAlgo_ShapeProcessor(new DE_ShapeFixParameters()).MergeTransferInfo(TP, 1); // no context: nothing
        new XSAlgo_ShapeProcessor(new DE_ShapeFixParameters()).MergeTransferInfo(new Transfer_FinderProcess());
    }

    [Fact]
    public void The_container_runs_a_sequence_or_default_fixes()
    {
        Interface_StaticSet aStatics = XSAlgoNativeTests.StepStatics();
        Shape aBox = new MakeBox(1.0, 2.0, 3.0).Shape();
        Transient? anInfo = null;
        XSAlgo_AlgoContainer aContainer = Occt.XSAlgo.AlgoContainer();
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
        Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out _);
        var aFace = ShapeCasts.Face(new Explorer(aCopy, ShapeType.Face).Current());
        var anEdge = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        Assert.True(aContainer.CheckPCurve(anEdge, aFace, 1e-3, false, aStatics));
        // and not on a published shape
        var aFace2 = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        var anEdge2 = ShapeCasts.Edge(new Explorer(aFace2, ShapeType.Edge).Current());
        Assert.ThrowsAny<FailureException>(() => aContainer.CheckPCurve(anEdge2, aFace2, 1e-3, false, aStatics));
    }
}
