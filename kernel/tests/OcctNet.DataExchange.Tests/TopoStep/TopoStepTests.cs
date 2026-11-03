// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.StepToTopoDS_BuilderError;
using static Occt.StepToTopoDS_GeometricToolError;
using static Occt.TopoDSToStep_MakeFaceError;

namespace OcctNet.DataExchange.Tests.TopoStep;

/// <summary>Unit tests of StepToTopoDS and TopoDSToStep (unit 8E): their contracts beyond the native comparisons.</summary>
public sealed class TopoStepTests
{
    private static string BRepText(in Shape theShape)
    {
        var aS = new StringWriter { NewLine = "\n" };
        BrepTools.Write(theShape, aS);
        return aS.ToString();
    }

    private static Transfer_FinderProcess NewFP()
    {
        var aModel = new StepData_StepModel();
        aModel.SetProtocol(TopoStepRecorder.Protocol());
        var aFP = new Transfer_FinderProcess();
        aFP.SetModel(aModel);
        return aFP;
    }

    [Fact]
    public void Decoders_give_OCCTs_texts()
    {
        Assert.Equal("Builder Done", StepToTopoDS.DecodeBuilderError(StepToTopoDS_BuilderDone)!.ToCString());
        Assert.Equal(" an Edge is degenerated", StepToTopoDS.DecodeGeometricToolError(StepToTopoDS_GeometricToolIsDegenerated));
        Assert.Equal("The Face has no Outer Wire", TopoDSToStep.DecodeFaceError(TopoDSToStep_InfiniteFace)!.ToCString());
    }

    [Fact]
    public void PointPair_is_unordered_and_by_reference()
    {
        var P1 = new StepGeom_CartesianPoint();
        var P2 = new StepGeom_CartesianPoint();
        var P3 = new StepGeom_CartesianPoint();
        Assert.Equal(new StepToTopoDS_PointPair(P1, P2), new StepToTopoDS_PointPair(P2, P1));
        Assert.Equal(new StepToTopoDS_PointPair(P1, P2).GetHashCode(), new StepToTopoDS_PointPair(P2, P1).GetHashCode());
        Assert.NotEqual(new StepToTopoDS_PointPair(P1, P2), new StepToTopoDS_PointPair(P1, P3));
    }

    [Fact]
    public void Writers_never_change_a_frozen_input()
    {
        ShapeAlgo.Init();
        Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape(); // frozen
        string aBefore = BRepText(aBox);
        Solid aSolid = ShapeCasts.Solid(aBox);
        var aMSB = new TopoDSToStep_MakeManifoldSolidBrep(aSolid, NewFP());
        Assert.True(aMSB.IsDone());
        var aSBSM = new TopoDSToStep_MakeShellBasedSurfaceModel(aSolid, NewFP());
        Assert.True(aSBSM.IsDone());
        var aGCS = new TopoDSToStep_MakeGeometricCurveSet(aBox, NewFP());
        Assert.True(aGCS.IsDone());
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
        ShapeAlgo.Init();
        Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        var aMSB = new TopoDSToStep_MakeManifoldSolidBrep(ShapeCasts.Solid(aBox), NewFP());

        // the entities into a model of their own, then translated back
        var aModel = new StepData_StepModel();
        aModel.SetProtocol(TopoStepRecorder.Protocol());
        aModel.AddWithRefs(aMSB.Value(), TopoStepRecorder.Protocol());
        var aTP = new Transfer_TransientProcess();
        aTP.SetModel(aModel);
        var aBuilder = new StepToTopoDS_Builder();
        aBuilder.Init(aMSB.Value(), aTP);
        Assert.True(aBuilder.IsDone());
        Assert.False(aBuilder.InternalValue().TShape()!.IsFrozen);
        Shape aSolid = aBuilder.Value();
        Assert.True(aSolid.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Solid, aSolid.ShapeType());
        var aProps = new Properties();
        BRepGProp.VolumeProperties(aSolid, aProps);
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
        StepToTopoDS_GeometricTool.UpdateParam3d(ref aRef, ref w1, ref w2, 1e-7, true);
        Assert.NotSame(aCurve, aRef);
        Assert.Equal(new Point3D(0, 0, 0), aCurve.StartPoint()); // the shared curve unchanged
        Assert.Equal(new Point3D(10, 0, 0), ((BSplineCurve3D)aRef).StartPoint());
        Assert.True(w1 < w2);
    }

    [Fact]
    public void A_null_starting_object_takes_a_check_as_OCCT()
    {
        var aTP = new Transfer_TransientProcess();
        aTP.AddWarning(null, "on a missing entity");
        Assert.Equal(1, aTP.NbMapped());
        Assert.Null(aTP.Mapped(1));
        Assert.NotNull(aTP.Find(null));
    }
}
