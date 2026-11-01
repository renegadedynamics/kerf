// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of <see cref="ChFi3d_BOPReconstruction"/>'s queries (the members of <c>TopOpeBRepBuild_HBuilder</c>
/// ChFi3d calls) on native data structures (<see cref="ReconRecordings"/>), and of its writes.
/// </summary>
public class ReconstructionTests
{
    private static ReconComparison.Run Box(string theName = "box_edge") =>
        ReconComparison.Reconstruct(ReconRecordings.Cases[theName]);

    [Fact]
    public void TheFactoryGivesTheReconstruction()
    {
        IChFi3d_Reconstruction R = ChFi3d_Reconstruction.Create(new TopOpeBRepDS_BuildTool());
        Assert.IsType<ChFi3d_BOPReconstruction>(R);
        Assert.Null(R.DataStructure());
        Assert.Empty(R.NewFaces(1));
        Assert.Empty(R.NewEdges(1));
    }

    [Fact]
    public void OneEdgeFilleted()
    {
        // box_edge: one fillet surface bounded by four curves; two lines on the side faces, two arcs on the ends
        ReconComparison.Run aRun = Box();
        TopOpeBRepDS_DataStructure aDS = aRun.HDS.DS();
        Assert.Same(aRun.HDS, aRun.Builder.DataStructure());
        Assert.Single(aRun.Builder.NewFaces(1));
        Assert.Empty(aRun.Builder.NewFaces(2)); // a second copy of the surface, without interferences
        Assert.Equal(TopAbs_FACE, aRun.Builder.NewFaces(1).First().ShapeType());
        for (int iC = 1; iC <= aDS.NbCurves(); iC++)
        {
            // curves 3 and 4 have no points: no edge, as natively
            Assert.Equal(iC is 3 or 4 ? 0 : 1, aRun.Builder.NewEdges(iC).Extent());
        }
        Assert.Equal(Math.PI * 10 / 2, ReconComparison.EdgeLength(aRun.Builder.NewEdges(5).First()), 9);
        Assert.Equal(100.0, ReconComparison.EdgeLength(aRun.Builder.NewEdges(1).First()), 9);

        // the solid merged: one solid, of one closed shell, of seven faces
        TopoDS_Shape aSolid = aDS.Shape(1);
        Assert.Equal(TopAbs_SOLID, aSolid.ShapeType());
        NCollection_List<TopoDS_Shape> aMerged = aRun.Builder.Merged(aSolid, TopAbs_IN);
        Assert.Single(aMerged);
        Assert.Equal(TopAbs_SOLID, aMerged.First().ShapeType());
        Assert.Equal(7, ReconComparison.Count(aMerged.First(), TopAbs_FACE));
        Assert.Equal(1.0e6 - (100.0 - 25.0 * Math.PI) * 100.0, ReconComparison.Volume(aMerged.First()), 6);
        Assert.Empty(aRun.Builder.Merged(aSolid, TopAbs_OUT));
        Assert.Empty(aRun.Builder.Merged(aSolid, TopAbs_ON));
        Assert.True(new BRepCheck_Analyzer(aRun.Result).IsValid());
    }

    [Fact]
    public void TheEdgesCutKeepTheirPartsIN()
    {
        // each edge the fillet cuts keeps one part, 90 long (100 less the radius): the part OUT is under the fillet
        ReconComparison.Run aRun = Box();
        TopOpeBRepDS_DataStructure aDS = aRun.HDS.DS();
        int aNbEdges = 0;
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            TopoDS_Shape E = aDS.Shape(i);
            if (E.ShapeType() != TopAbs_EDGE)
            {
                continue;
            }
            aNbEdges++;
            Assert.True(aRun.Builder.IsSplit(E, TopAbs_IN));
            Assert.False(aRun.Builder.IsSplit(E, TopAbs_OUT));
            Assert.False(aRun.Builder.IsSplit(E, TopAbs_ON));
            NCollection_List<TopoDS_Shape> aParts = aRun.Builder.Splits(E, TopAbs_IN);
            Assert.Single(aParts);
            Assert.Equal(90.0, ReconComparison.EdgeLength(aParts.First()), 9);
            Assert.Empty(aRun.Builder.Splits(E, TopAbs_OUT));
        }
        Assert.Equal(4, aNbEdges);
    }

    [Fact]
    public void TheFacesCutAreSplitTheOthersNot()
    {
        // box_edge: the two faces along the edge and the two at its ends are cut; the two others are left whole
        ReconComparison.Run aRun = Box();
        NCollection_IndexedMap<TopoDS_Shape> aFaces = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aRun.Input, TopAbs_FACE, aFaces);
        NCollection_IndexedMap<TopoDS_Shape> aResultFaces = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aRun.Result, TopAbs_FACE, aResultFaces);
        int aNbSplit = 0;
        for (int i = 1; i <= aFaces.Extent(); i++)
        {
            TopoDS_Shape F = aFaces[i];
            if (aRun.Builder.IsSplit(F, TopAbs_IN))
            {
                aNbSplit++;
                Assert.Single(aRun.Builder.Splits(F, TopAbs_IN));
                Assert.False(aResultFaces.Contains(F));
                Assert.True(aResultFaces.Contains(aRun.Builder.Splits(F, TopAbs_IN).First()));
                // HBuilder's Merged binds only solids and shells for the fillets (FilBuilder::SetRegul reads it)
                Assert.Empty(aRun.Builder.Merged(F, TopAbs_IN));
            }
            else
            {
                Assert.True(aResultFaces.Contains(F)); // the input's face itself
            }
        }
        Assert.Equal(4, aNbSplit);
    }

    [Fact]
    public void TheInputIsNotWritten()
    {
        // the input is the builder's working copy: the reconstruction writes only into shapes it makes. Frozen, any
        // write would throw (ReconComparison freezes it); not frozen, its text is unchanged.
        foreach (string aName in new[] { "box_corner", "cylinder_top_edge", "b3d_box_minus_cylinder_hole_rim", "H8" })
        {
            ReconCase aCase = ReconRecordings.Cases[aName];
            ReconComparison.Reconstruct(aCase, theFreezeInput: true);

            TopoDS_Shape anInput = aCase.Input;
            string aBefore = BRepText(anInput);
            TopOpeBRepDS_HDataStructure aHDS = TopOpeBRepDS_TextDump.Read(aCase.DsText, anInput);
            ChFi3d_BOPReconstruction R = new(new TopOpeBRepDS_BuildTool());
            R.Perform(aHDS);
            R.MergeSolid(aHDS.DS().Shape(1), TopAbs_IN);
            Assert.Equal(aBefore, BRepText(anInput));
        }
    }

    [Fact]
    public void PerformAgainStartsAfresh()
    {
        ReconCase aCase = ReconRecordings.Cases["box_edge"];
        TopoDS_Shape anInput = aCase.Input;
        TopOpeBRepDS_HDataStructure aHDS = TopOpeBRepDS_TextDump.Read(aCase.DsText, anInput);
        ChFi3d_BOPReconstruction R = new(new TopOpeBRepDS_BuildTool());
        R.Perform(aHDS);
        TopoDS_Shape aSolid = aHDS.DS().Shape(1);
        R.MergeSolid(aSolid, TopAbs_IN);
        TopoDS_Shape aFirst = R.Merged(aSolid, TopAbs_IN).First();
        R.MergeSolid(aSolid, TopAbs_OUT); // only IN is built, as ChFi3d asks
        Assert.Single(R.Merged(aSolid, TopAbs_IN));
        R.Perform(aHDS);
        Assert.Empty(R.Merged(aSolid, TopAbs_IN));
        R.MergeSolid(aSolid, TopAbs_IN);
        Assert.False(aFirst.IsSame(R.Merged(aSolid, TopAbs_IN).First()));
        Assert.Equal(ReconComparison.Volume(aFirst), ReconComparison.Volume(R.Merged(aSolid, TopAbs_IN).First()), 6);
    }

    [Fact]
    public void ASeamAndAClosedCurve()
    {
        // cylinder_top_edge: the torus's seam carries two pcurves; the circles are closed edges on one vertex
        ReconComparison.Run aRun = Box("cylinder_top_edge");
        TopoDS_Face aTorus = TopoDS.Face(aRun.Builder.NewFaces(1).First());
        TopoDS_Edge aSeam = TopoDS.Edge(aRun.Builder.NewEdges(5).First());
        Assert.True(BRep_Tool.IsClosed(aSeam, aTorus));
        TopoDS_Edge aCircle = TopoDS.Edge(aRun.Builder.NewEdges(2).First());
        TopExp.Vertices(aCircle, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        Assert.True(V1.IsSame(V2));
        Assert.True(new BRepCheck_Analyzer(aRun.Result).IsValid());
    }

    [Fact]
    public void AShellStaysAShell()
    {
        ReconComparison.Run aRun = Box("shell_box_without_top");
        TopoDS_Shape aShell = aRun.HDS.DS().Shape(1);
        Assert.Equal(TopAbs_SHELL, aShell.ShapeType());
        NCollection_List<TopoDS_Shape> aMerged = aRun.Builder.Merged(aShell, TopAbs_IN);
        Assert.Single(aMerged);
        Assert.Equal(TopAbs_SHELL, aMerged.First().ShapeType());
        Assert.False(BRep_Tool.IsClosed(aMerged.First()));
    }

    [Fact]
    public void AVoidKeepsItsShell()
    {
        ReconComparison.Run aRun = Box("solid_with_void_inner_edge");
        TopoDS_Shape aSolid = aRun.Builder.Merged(aRun.HDS.DS().Shape(1), TopAbs_IN).First();
        Assert.Equal(2, ReconComparison.Count(aSolid, TopAbs_SHELL));
    }

    private static string BRepText(in TopoDS_Shape S)
    {
        StringWriter w = new();
        BRepTools.Write(S, w);
        return w.ToString();
    }
}
