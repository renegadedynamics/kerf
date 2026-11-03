// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Exchange.Step.Import.BuilderError;
using static Kerf.Engine.Exchange.Step.Import.GeometricToolError;
using static Kerf.Engine.Exchange.Step.Export.MakeFaceError;

namespace Kerf.Kernel.DataExchange.Tests.TopoStep;

/// <summary>Unit tests of StepToTopoDS and TopoDSToStep (unit 8E): their contracts beyond the native comparisons.</summary>
public sealed class TopoStepTests
{
    private static string BRepText(in Shape theShape)
    {
        var aS = new StringWriter { NewLine = "\n" };
        BrepTools.Write(theShape, aS);
        return aS.ToString();
    }

    private static FinderProcess NewFP()
    {
        var aModel = new StepModel();
        aModel.SetProtocol(TopoStepRecorder.Protocol());
        var aFP = new FinderProcess();
        aFP.SetModel(aModel);
        return aFP;
    }

    [Fact]
    public void Decoders_give_OCCTs_texts()
    {
        Assert.Equal("Builder Done", global::Kerf.Engine.Exchange.Step.Import.StepToTopoDS.DecodeBuilderError(global::Kerf.Engine.Exchange.Step.Import.BuilderError.BuilderDone)!.ToCString());
        Assert.Equal(" an Edge is degenerated", global::Kerf.Engine.Exchange.Step.Import.StepToTopoDS.DecodeGeometricToolError(global::Kerf.Engine.Exchange.Step.Import.GeometricToolError.GeometricToolIsDegenerated));
        Assert.Equal("The Face has no Outer Wire", global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.DecodeFaceError(global::Kerf.Engine.Exchange.Step.Export.MakeFaceError.InfiniteFace)!.ToCString());
    }

    [Fact]
    public void PointPair_is_unordered_and_by_reference()
    {
        var P1 = new StepCartesianPoint();
        var P2 = new StepCartesianPoint();
        var P3 = new StepCartesianPoint();
        Assert.Equal(new global::Kerf.Engine.Exchange.Step.Import.PointPair(P1, P2), new global::Kerf.Engine.Exchange.Step.Import.PointPair(P2, P1));
        Assert.Equal(new global::Kerf.Engine.Exchange.Step.Import.PointPair(P1, P2).GetHashCode(), new global::Kerf.Engine.Exchange.Step.Import.PointPair(P2, P1).GetHashCode());
        Assert.NotEqual(new global::Kerf.Engine.Exchange.Step.Import.PointPair(P1, P2), new global::Kerf.Engine.Exchange.Step.Import.PointPair(P1, P3));
    }

    [Fact]
    public void Writers_never_change_a_frozen_input()
    {
        global::Kerf.Engine.Healing.ShapeAlgo.Init();
        Shape aBox = new MakeBox(10, 20, 30).Shape(); // frozen
        string aBefore = BRepText(aBox);
        Solid aSolid = ShapeCasts.Solid(aBox);
        var aMSB = new global::Kerf.Engine.Exchange.Step.Export.ManifoldSolidBrepMaker(aSolid, NewFP());
        Assert.True(aMSB.IsDone);
        var aSBSM = new global::Kerf.Engine.Exchange.Step.Export.MakeShellBasedSurfaceModel(aSolid, NewFP());
        Assert.True(aSBSM.IsDone);
        var aGCS = new global::Kerf.Engine.Exchange.Step.Export.MakeGeometricCurveSet(aBox, NewFP());
        Assert.True(aGCS.IsDone);
        Assert.Equal(aBefore, BRepText(aBox));

        // 6 faces, 12 edges, 8 vertices; each written once
        List<string> aLines = TopoStepRecorder.Written(aMSB.Value());
        Assert.Equal(6, aLines.Count(l => l.Contains("ADVANCED_FACE", StringComparison.Ordinal)));
        Assert.Equal(12, aLines.Count(l => l.Contains("EDGE_CURVE", StringComparison.Ordinal)));
        Assert.Equal(8, aLines.Count(l => l.Contains("VERTEX_POINT", StringComparison.Ordinal)));
    }

    [Fact]
    public void A_written_box_reads_back_as_a_frozen_solid()
    {
        global::Kerf.Engine.Healing.ShapeAlgo.Init();
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        var aMSB = new global::Kerf.Engine.Exchange.Step.Export.ManifoldSolidBrepMaker(ShapeCasts.Solid(aBox), NewFP());

        // the entities into a model of their own, then translated back
        var aModel = new StepModel();
        aModel.SetProtocol(TopoStepRecorder.Protocol());
        aModel.AddWithRefs(aMSB.Value(), TopoStepRecorder.Protocol());
        var aTP = new TransientProcess();
        aTP.SetModel(aModel);
        var aBuilder = new global::Kerf.Engine.Exchange.Step.Import.ImportBuilder();
        aBuilder.Init(aMSB.Value(), aTP);
        Assert.True(aBuilder.IsDone);
        Assert.False(aBuilder.InternalValue().TShape()!.IsFrozen);
        Shape aSolid = aBuilder.Value();
        Assert.True(aSolid.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Solid, aSolid.Type);
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aSolid, aProps);
        Assert.Equal(6000.0, aProps.Mass(), 1e-6);
    }

    [Fact]
    public void UpdateParam3d_reverses_a_frozen_curve_in_a_copy()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 2);
        aPoles.SetValue(1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, new Point3D(10, 0, 0));
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots.SetValue(1, 0);
        aKnots.SetValue(2, 1);
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults.SetValue(1, 2);
        aMults.SetValue(2, 2);
        var aCurve = new BSplineCurve3D(aPoles, aKnots, aMults, 1);
        aCurve.Freeze();
        Curve3D aRef = aCurve;
        double w1 = 0.8, w2 = 0.2;
        global::Kerf.Engine.Exchange.Step.Import.GeometricTool.UpdateParam3d(ref aRef, ref w1, ref w2, 1e-7, true);
        Assert.NotSame(aCurve, aRef);
        Assert.Equal(new Point3D(0, 0, 0), aCurve.StartPoint); // the shared curve unchanged
        Assert.Equal(new Point3D(10, 0, 0), ((BSplineCurve3D)aRef).StartPoint);
        Assert.True(w1 < w2);
    }

    [Fact]
    public void A_null_starting_object_takes_a_check_as_OCCT()
    {
        var aTP = new TransientProcess();
        aTP.AddWarning(null, "on a missing entity");
        Assert.Equal(1, aTP.MappedCount);
        Assert.Null(aTP.Mapped(1));
        Assert.NotNull(aTP.Find(null));
    }
}
