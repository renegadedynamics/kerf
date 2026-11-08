// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Step.Export;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// What may be written as a faceted B-rep (unit 12Q, decision 1753): faces flat, as planes or as splines of degree one
/// with two poles each way, and their pcurves straight. And the export's messages, one per error.
/// </summary>
public class FacetedToolLawsTests
{
    // The poles of a patch over (0, 0) to (2, 3): theNbU of them along x, the inner ones lifted by theLift
    private static NCollection_Array2<Point3D> Poles(int theNbU, double theLift)
    {
        var aPoles = new NCollection_Array2<Point3D>(1, theNbU, 1, 2);
        for (int i = 1; i <= theNbU; i++)
        {
            double x = 2.0 * (i - 1) / (theNbU - 1);
            double z = i > 1 && i < theNbU ? theLift : 0;
            aPoles.SetValue(i, 1, new Point3D(x, 0, z));
            aPoles.SetValue(i, 2, new Point3D(x, 3, z));
        }
        return aPoles;
    }

    private static Shape FaceOn(Surface theSurface) => new MakeFace(theSurface, 1e-7).Shape();

    private static BSplineSurface Spline(int theNbU, double theLift)
    {
        int aDegree = theNbU - 1;
        var aKnots = NCollection_Array1<double>.From([0.0, 1.0]);
        var aUMults = NCollection_Array1<int>.From([aDegree + 1, aDegree + 1]);
        var aVMults = NCollection_Array1<int>.From([2, 2]);
        return new BSplineSurface(Poles(theNbU, theLift), aKnots, aKnots, aUMults, aVMults, aDegree, 1);
    }

    [Fact]
    public void A_flat_spline_or_bezier_of_degree_one_is_faceted()
    {
        Assert.Equal(FacetedError.FacetedDone, FacetedTool.CheckTopoDSShape(FaceOn(Spline(2, 0))));
        Assert.Equal(FacetedError.FacetedDone, FacetedTool.CheckTopoDSShape(FaceOn(new BezierSurface(Poles(2, 0)))));
        Assert.Equal(FacetedError.FacetedDone, FacetedTool.CheckTopoDSShape(new MakeBox(1, 2, 3).Shape()));
    }

    [Fact]
    public void A_spline_or_bezier_of_higher_degree_is_not_even_when_flat()
    {
        // the check is by degree and poles, not by shape: a flat quadratic is refused as a bent one is
        foreach (double aLift in new[] { 0.0, 1.0 })
        {
            Assert.Equal(FacetedError.SurfaceNotPlane, FacetedTool.CheckTopoDSShape(FaceOn(Spline(3, aLift))));
            Assert.Equal(FacetedError.SurfaceNotPlane, FacetedTool.CheckTopoDSShape(FaceOn(new BezierSurface(Poles(3, aLift)))));
        }
        Assert.Equal(FacetedError.SurfaceNotPlane, FacetedTool.CheckTopoDSShape(new ApiMakeCylinder(1, 2).Shape()));
    }

    [Fact]
    public void Each_error_has_its_message()
    {
        Assert.Equal("Builder Done", TopoDSToStep.DecodeBuilderError(ExportBuilderError.BuilderDone)!.ToCString());
        Assert.Equal("None of the Shell Faces has been mapped", TopoDSToStep.DecodeBuilderError(ExportBuilderError.NoFaceMapped)!.ToCString());
        Assert.Equal("Other Error in Builder", TopoDSToStep.DecodeBuilderError(ExportBuilderError.BuilderOther)!.ToCString());
        Assert.Equal("Face Done", TopoDSToStep.DecodeFaceError(MakeFaceError.FaceDone)!.ToCString());
        Assert.Equal("Other Error in Make STEP face", TopoDSToStep.DecodeFaceError(MakeFaceError.FaceOther)!.ToCString());
        Assert.Equal("The Face has no Outer Wire", TopoDSToStep.DecodeFaceError(MakeFaceError.InfiniteFace)!.ToCString());
        Assert.Equal("The Face is Internal or External", TopoDSToStep.DecodeFaceError(MakeFaceError.NonManifoldFace)!.ToCString());
        Assert.Equal("None of the Face Wires has been mapped", TopoDSToStep.DecodeFaceError(MakeFaceError.NoWireMapped)!.ToCString());
        Assert.Equal("Wire Done", TopoDSToStep.DecodeWireError(MakeWireError.WireDone)!.ToCString());
        Assert.Equal("Other Error in Make STEP wire", TopoDSToStep.DecodeWireError(MakeWireError.WireOther)!.ToCString());
        Assert.Equal("The Wire is Internal or External", TopoDSToStep.DecodeWireError(MakeWireError.NonManifoldWire)!.ToCString());
        Assert.Equal("Edge Done", TopoDSToStep.DecodeEdgeError(MakeEdgeError.EdgeDone)!.ToCString());
        Assert.Equal("Other Error in Make STEP Edge", TopoDSToStep.DecodeEdgeError(MakeEdgeError.EdgeOther)!.ToCString());
        Assert.Equal("The Edge is Internal or External", TopoDSToStep.DecodeEdgeError(MakeEdgeError.NonManifoldEdge)!.ToCString());
        Assert.Equal("Vertex Done", TopoDSToStep.DecodeVertexError(MakeVertexError.VertexDone)!.ToCString());
        Assert.Equal("Other Error in Make STEP Vertex", TopoDSToStep.DecodeVertexError(MakeVertexError.VertexOther)!.ToCString());
    }
}
