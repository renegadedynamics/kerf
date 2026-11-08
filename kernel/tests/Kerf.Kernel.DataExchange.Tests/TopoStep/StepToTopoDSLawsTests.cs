// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Import;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// The reader's placements and messages by what they are (unit 12Q, decision 1753): a mapped item's placement takes its
/// origin frame onto its target frame, so a shape moves with it and keeps its volume; and each translator's error has
/// its message.
/// </summary>
public class StepToTopoDSLawsTests
{
    private static TCollection_HAsciiString Name() => new("");

    private static StepCartesianPoint Point(double x, double y, double z)
    {
        var aP = new StepCartesianPoint();
        aP.Init3D(Name(), x, y, z);
        return aP;
    }

    private static StepDirection Direction(double x, double y, double z)
    {
        var aRatios = new NCollection_HArray1<double>(1, 3);
        aRatios.SetValue(1, x);
        aRatios.SetValue(2, y);
        aRatios.SetValue(3, z);
        var aD = new StepDirection();
        aD.Init(Name(), aRatios);
        return aD;
    }

    private static StepAxis2Placement3d Placement(Point3D theAt, Direction3D theAxis, Direction3D theRef)
    {
        var aP = new StepAxis2Placement3d();
        aP.Init(Name(), Point(theAt.X, theAt.Y, theAt.Z), true, Direction(theAxis.X, theAxis.Y, theAxis.Z), true, Direction(theRef.X, theRef.Y, theRef.Z));
        return aP;
    }

    [Fact]
    public void A_mapping_takes_its_origin_frame_onto_its_target()
    {
        // from the frame at the origin to one at (1, 2, 3) turned a quarter about Z: X goes to Y
        var aMake = new MakeTransformed();
        Assert.True(aMake.Compute(Placement(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)),
                                  Placement(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(0, 1, 0))));
        Transform3D aT = aMake.Transformation();
        Assert.True(new Point3D(0, 0, 0).Transformed(aT).DistanceTo(new Point3D(1, 2, 3)) < 1e-12);
        Assert.True(new Point3D(1, 0, 0).Transformed(aT).DistanceTo(new Point3D(1, 3, 3)) < 1e-12);
        Assert.True(new Point3D(0, 0, 5).Transformed(aT).DistanceTo(new Point3D(1, 2, 8)) < 1e-12);

        // a box moved by it keeps its volume, and its centre goes where the mapping takes it
        Shape aBox = new MakeBox(1, 2, 3).Shape();
        Assert.True(aMake.Transform(ref aBox));
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aBox, aProps);
        Assert.Equal(6, aProps.Mass(), 9);
        Assert.True(aProps.CentreOfMass().DistanceTo(new Point3D(0.5, 1, 1.5).Transformed(aT)) < 1e-9);
    }

    [Fact]
    public void A_mapping_onto_itself_moves_nothing_and_one_without_frames_is_none()
    {
        var aMake = new MakeTransformed();
        StepAxis2Placement3d aFrame = Placement(new Point3D(4, 5, 6), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1));
        Assert.True(aMake.Compute(aFrame, aFrame));
        foreach (Point3D aP in new[] { new Point3D(0, 0, 0), new Point3D(4, 5, 6), new Point3D(-1, 2, 9) })
        {
            Assert.True(aP.Transformed(aMake.Transformation()).DistanceTo(aP) < 1e-12);
        }
        // a mapping not made leaves the identity, which moves no shape
        Assert.False(aMake.Compute(null!, aFrame));
        Shape aBox = new MakeBox(1, 1, 1).Shape();
        Assert.False(aMake.Transform(ref aBox));
        Assert.Equal(TransformForm.Identity, aMake.Transformation().Form);

        // an operator that moves by (1, 0, 0)
        var anOperator = new StepCartesianTransformationOperator3d();
        anOperator.SetLocalOrigin(Point(1, 0, 0));
        Assert.True(aMake.Compute(anOperator));
        Assert.True(new Point3D(0, 0, 0).Transformed(aMake.Transformation()).DistanceTo(new Point3D(1, 0, 0)) < 1e-12);
    }

    [Fact]
    public void Each_translator_error_has_its_message()
    {
        Assert.Equal("Builder Done", StepToTopoDS.DecodeBuilderError(BuilderError.BuilderDone)!.ToCString());
        Assert.Equal("Builder Other", StepToTopoDS.DecodeBuilderError(BuilderError.BuilderOther)!.ToCString());
        Assert.Equal("Translate Shell Done", StepToTopoDS.DecodeShellError(TranslateShellError.TranslateShellDone)!.ToCString());
        Assert.Equal("Translate Shell Other", StepToTopoDS.DecodeShellError(TranslateShellError.TranslateShellOther)!.ToCString());
        Assert.Equal("Translate Face Done", StepToTopoDS.DecodeFaceError(TranslateFaceError.TranslateFaceDone)!.ToCString());
        Assert.Equal("Translate Face Other", StepToTopoDS.DecodeFaceError(TranslateFaceError.TranslateFaceOther)!.ToCString());
        Assert.Equal("Translate Edge Done", StepToTopoDS.DecodeEdgeError(TranslateEdgeError.TranslateEdgeDone)!.ToCString());
        Assert.Equal("Translate Edge Other", StepToTopoDS.DecodeEdgeError(TranslateEdgeError.TranslateEdgeOther)!.ToCString());
        Assert.Equal("Translate Vertex Done", StepToTopoDS.DecodeVertexError(TranslateVertexError.TranslateVertexDone)!.ToCString());
        Assert.Equal("Translate Vertex Other", StepToTopoDS.DecodeVertexError(TranslateVertexError.TranslateVertexOther)!.ToCString());
        Assert.Equal("Translate VertexLoop Done", StepToTopoDS.DecodeVertexLoopError(TranslateVertexLoopError.TranslateVertexLoopDone)!.ToCString());
        Assert.Equal("Translate VertexLoop Other", StepToTopoDS.DecodeVertexLoopError(TranslateVertexLoopError.TranslateVertexLoopOther)!.ToCString());
        Assert.Equal("Translate PolyLoop Done", StepToTopoDS.DecodePolyLoopError(TranslatePolyLoopError.TranslatePolyLoopDone)!.ToCString());
        Assert.Equal("Translate PolyLoop Other", StepToTopoDS.DecodePolyLoopError(TranslatePolyLoopError.TranslatePolyLoopOther)!.ToCString());
        Assert.Equal(" Geometric Tool is done", StepToTopoDS.DecodeGeometricToolError(GeometricToolError.GeometricToolDone));
        Assert.Equal(" an Edge is degenerated", StepToTopoDS.DecodeGeometricToolError(GeometricToolError.GeometricToolIsDegenerated));
        Assert.Equal(" SurfaceCurve does not contain a PCurve lying on the BasisSurface", StepToTopoDS.DecodeGeometricToolError(GeometricToolError.GeometricToolHasNoPCurve));
        Assert.Equal(" the update of 3D-Parameters failed", StepToTopoDS.DecodeGeometricToolError(GeometricToolError.GeometricToolWrong3dParameters));
        Assert.Equal(" the projection of a VertexPoint on the curve3d failed", StepToTopoDS.DecodeGeometricToolError(GeometricToolError.GeometricToolNoProjectiOnCurve));
        Assert.Equal(" GeometricTool failed", StepToTopoDS.DecodeGeometricToolError(GeometricToolError.GeometricToolOther));
    }
}
