// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of the fillet builders (unit 7N2): the seam with the reconstruction, the working copy, the frozen input,
/// the queries and the diagnostics.
/// </summary>
public class ChFi3dBuilderTests
{
    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(100, 100, 100).Shape();

    private static double[] VertexTolerances(in TopoDS_Shape S)
    {
        List<double> r = [];
        for (TopExp_Explorer ex = new(S, TopAbs_VERTEX); ex.More(); ex.Next())
        {
            r.Add(BRep_Tool.Tolerance(TopoDS.Vertex(ex.Current())));
        }
        return r.ToArray();
    }

    [Fact]
    public void ComputeStopsAtTheSeamUntilTheReconstructionLands()
    {
        // ChFi3d_Builder::Compute calls myCoup->Perform(myDS) with the data structure complete: until unit 7R, the
        // stand-in reconstruction throws there (decision 992's seam)
        TopoDS_Shape aBox = Box();
        ChFi3d_FilBuilder aB = new(aBox);
        aB.Add(10, ChFi3dBuilderNativeTests.EdgeOf(aBox, 5));
        NotImplementedException e = Assert.Throws<NotImplementedException>(aB.Compute);
        Assert.StartsWith("7R", e.Message);
        Assert.True(aB.myDS.DS().NbSurfaces() > 0);
    }

    [Fact]
    public void TheFrozenInputIsCopiedAndLeftUnchanged()
    {
        // OCCT raises the tolerances of the vertices the fillets end at (ChFi3d_IndexPointInDS, Compute's
        // UpdateVertex) and gives the elementary spines' edges 3D curves: on the builder's working copy, never on the
        // caller's frozen shape (decisions 1005, 10203)
        TopoDS_Shape aBox = Box();
        Assert.True(BRepTools.HoldsFrozen(aBox));
        double[] aBefore = VertexTolerances(aBox);
        (ChFi3d_FilBuilder aB, ChFi3dBuilderNativeTests.RecordingReconstruction aRec) =
            ChFi3dBuilderNativeTests.RunFillet(aBox, Enumerable.Range(1, 12).Select(k => new ChFi3dBuilderNativeTests.CaseEdge(k, 10)));
        Assert.Equal(1, aRec.NbPerform);
        Assert.False(aB.WorkingShape().IsSame(aBox));
        Assert.Equal(aBefore, VertexTolerances(aBox));
        // every shape of the data structure is the working copy's
        TopOpeBRepDS_DataStructure DS = aRec.HDS!.DS();
        NCollection_IndexedMap<TopoDS_Shape> aWork = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aB.WorkingShape(), aWork);
        NCollection_IndexedMap<TopoDS_Shape> anInput = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aBox, anInput);
        for (int i = 1; i <= DS.NbShapes(); i++)
        {
            Assert.True(aWork.Contains(DS.Shape(i)));
            Assert.False(anInput.Contains(DS.Shape(i)));
        }
    }

    [Fact]
    public void TheCallersEdgesAndVerticesAreMappedOntoTheWorkingCopy()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Edge E = ChFi3dBuilderNativeTests.EdgeOf(aBox, 5);
        ChFi3d_FilBuilder aB = new(aBox);
        aB.Add(10, E);
        Assert.Equal(1, aB.NbElements());
        Assert.Equal(1, aB.Contains(E));
        Assert.Equal(1, aB.Contains(E, out int IinC));
        Assert.Equal(1, IinC);
        Assert.Equal(10.0, aB.Radius(1, E));
        // the contour's ends are handed back as the caller's vertices
        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        TopoDS_Vertex F = aB.FirstVertex(1), L = aB.LastVertex(1);
        Assert.True((F.IsSame(V1) && L.IsSame(V2)) || (F.IsSame(V2) && L.IsSame(V1)));
        Assert.Equal(0.0, aB.Abscissa(1, F), 9);
        Assert.Equal(100.0, aB.Length(1), 9);
        aB.Remove(E);
        Assert.Equal(0, aB.NbElements());
    }

    [Fact]
    public void ANoEdgeComputeThrowsAsOcct()
    {
        ChFi3d_FilBuilder aB = new(Box());
        Standard_Failure e = Assert.Throws<Standard_Failure>(aB.Compute);
        Assert.Equal("There are no suitable edges for chamfer or fillet", e.Message);
    }

    [Fact]
    public void ARadiusTooLargeGivesTheStripeStatusesBuild123dReads()
    {
        // build123d.NET's "fillet too large" (MalformedInputTests): every contour faulty, ChFiDS_StartsolFailure (no start solution),
        // Perform never reached (native: tools/dsrec's b3d_fail_box_fillet_too_large)
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 1, 1).Shape();
        (ChFi3d_FilBuilder aB, ChFi3dBuilderNativeTests.RecordingReconstruction aRec) =
            ChFi3dBuilderNativeTests.RunFillet(aBox, Enumerable.Range(1, 12).Select(k => new ChFi3dBuilderNativeTests.CaseEdge(k, 5)));
        Assert.Equal(0, aRec.NbPerform);
        Assert.False(aB.IsDone());
        Assert.Equal(12, aB.NbFaultyContours());
        for (int ic = 1; ic <= aB.NbElements(); ic++)
        {
            Assert.Equal(ChFiDS_ErrorStatus.ChFiDS_StartsolFailure, aB.StripeStatus(ic));
        }
        Assert.Throws<Standard_NoSuchObject>(() => aB.Shape());
    }

    [Fact]
    public void TheBuilderIsRepeatable()
    {
        // two builders on the same input give the same data structure (no state left behind between runs, plan 2.3)
        DsCase aCase = DsRecordings.Cases["box_corner"];
        string? a = ChFi3dBuilderNativeTests.Run(aCase).Rec.Text;
        string? b = ChFi3dBuilderNativeTests.Run(aCase).Rec.Text;
        Assert.Equal(a, b);
    }
}
