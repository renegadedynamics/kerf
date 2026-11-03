// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of the fillet builders (unit 7N2): the seam with the reconstruction, the working copy, the frozen input,
/// the queries and the diagnostics.
/// </summary>
public class ChFi3dBuilderTests
{
    private static Shape Box() => new MakeBox(100, 100, 100).Shape();

    private static double[] VertexTolerances(in Shape S)
    {
        List<double> r = [];
        for (Explorer ex = new(S, ShapeType.Vertex); ex.More(); ex.Next())
        {
            r.Add(Tool.Tolerance(ShapeCasts.Vertex(ex.Current())));
        }
        return r.ToArray();
    }

    [Fact]
    public void ComputeRunsThroughTheReconstruction()
    {
        // ChFi3d_Builder::Compute hands the data structure to myCoup->Perform: unit 7R's reconstruction on the
        // Boolean engine (decision 992's seam)
        Shape aBox = Box();
        FilBuilder aB = new(aBox);
        Assert.IsType<BOPReconstruction>(aB.Builder());
        aB.Add(10, ChFi3dBuilderNativeTests.EdgeOf(aBox, 5));
        aB.Compute();
        Assert.True(aB.IsDone);
        Assert.True(new Analyzer(aB.Shape()).IsValid());
        Assert.Equal(1, ReconComparison.Count(aB.Shape(), ShapeType.Solid));
    }

    [Fact]
    public void TheFrozenInputIsCopiedAndLeftUnchanged()
    {
        // OCCT raises the tolerances of the vertices the fillets end at (ChFi3d_IndexPointInDS, Compute's
        // UpdateVertex) and gives the elementary spines' edges 3D curves: on the builder's working copy, never on the
        // caller's frozen shape (decisions 1005, 1087)
        Shape aBox = Box();
        Assert.True(BrepTools.HoldsFrozen(aBox));
        double[] aBefore = VertexTolerances(aBox);
        (FilBuilder aB, ChFi3dBuilderNativeTests.RecordingReconstruction aRec) =
            ChFi3dBuilderNativeTests.RunFillet(aBox, Enumerable.Range(1, 12).Select(k => new ChFi3dBuilderNativeTests.CaseEdge(k, 10)));
        Assert.Equal(1, aRec.NbPerform);
        Assert.False(aB.WorkingShape().IsSame(aBox));
        Assert.Equal(aBefore, VertexTolerances(aBox));
        // every shape of the data structure is the working copy's
        DsDataStructure DS = aRec.HDS!.DS();
        NCollection_IndexedMap<Shape> aWork = new(ShapeHasher.Instance);
        Exploration.MapShapes(aB.WorkingShape(), aWork);
        NCollection_IndexedMap<Shape> anInput = new(ShapeHasher.Instance);
        Exploration.MapShapes(aBox, anInput);
        for (int i = 1; i <= DS.NbShapes(); i++)
        {
            Assert.True(aWork.Contains(DS.Shape(i)));
            Assert.False(anInput.Contains(DS.Shape(i)));
        }
    }

    [Fact]
    public void TheCallersEdgesAndVerticesAreMappedOntoTheWorkingCopy()
    {
        Shape aBox = Box();
        Edge E = ChFi3dBuilderNativeTests.EdgeOf(aBox, 5);
        FilBuilder aB = new(aBox);
        aB.Add(10, E);
        Assert.Equal(1, aB.NbElements());
        Assert.Equal(1, aB.Contains(E));
        Assert.Equal(1, aB.Contains(E, out int IinC));
        Assert.Equal(1, IinC);
        Assert.Equal(10.0, aB.Radius(1, E));
        // the contour's ends are handed back as the caller's vertices
        Exploration.Vertices(E, out Vertex V1, out Vertex V2);
        Vertex F = aB.FirstVertex(1), L = aB.LastVertex(1);
        Assert.True((F.IsSame(V1) && L.IsSame(V2)) || (F.IsSame(V2) && L.IsSame(V1)));
        Assert.Equal(0.0, aB.Abscissa(1, F), 9);
        Assert.Equal(100.0, aB.Length(1), 9);
        aB.Remove(E);
        Assert.Equal(0, aB.NbElements());
    }

    [Fact]
    public void ANoEdgeComputeThrowsAsOcct()
    {
        FilBuilder aB = new(Box());
        FailureException e = Assert.Throws<FailureException>(aB.Compute);
        Assert.Equal("There are no suitable edges for chamfer or fillet", e.Message);
    }

    [Fact]
    public void ARadiusTooLargeGivesTheStripeStatusesBuild123dReads()
    {
        // build123d.NET's "fillet too large" (MalformedInputTests): every contour faulty, ChFiDS_StartsolFailure (no start solution),
        // Perform never reached (native: tools/dsrec's b3d_fail_box_fillet_too_large)
        Shape aBox = new MakeBox(1, 1, 1).Shape();
        (FilBuilder aB, ChFi3dBuilderNativeTests.RecordingReconstruction aRec) =
            ChFi3dBuilderNativeTests.RunFillet(aBox, Enumerable.Range(1, 12).Select(k => new ChFi3dBuilderNativeTests.CaseEdge(k, 5)));
        Assert.Equal(0, aRec.NbPerform);
        Assert.False(aB.IsDone);
        Assert.Equal(12, aB.FaultyContourCount);
        for (int ic = 1; ic <= aB.NbElements(); ic++)
        {
            Assert.Equal(ErrorStatus.StartsolFailure, aB.StripeStatus(ic));
        }
        Assert.Throws<NoSuchObjectException>(() => aB.Shape());
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
