// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Fillets;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of <see cref="BOPReconstruction"/>'s queries (the members of <c>TopOpeBRepBuild_HBuilder</c>
/// ChFi3d calls) on native data structures (<see cref="ReconRecordings"/>), and of its writes.
/// </summary>
public class ReconstructionTests
{
    private static ReconComparison.Run Box(string theName = "box_edge") =>
        ReconComparison.Reconstruct(ReconRecordings.Cases[theName]);

    [Fact]
    public void TheFactoryGivesTheReconstruction()
    {
        Reconstruction R = Reconstruction.Create(new DsBuildTool());
        Assert.IsType<BOPReconstruction>(R);
        Assert.Null(R.DataStructure());
        Assert.Empty(R.NewFaces(1));
        Assert.Empty(R.NewEdges(1));
    }

    [Fact]
    public void OneEdgeFilleted()
    {
        // box_edge: one fillet surface bounded by four curves; two lines on the side faces, two arcs on the ends
        ReconComparison.Run aRun = Box();
        DsDataStructure aDS = aRun.HDS.DS();
        Assert.Same(aRun.HDS, aRun.Builder.DataStructure());
        Assert.Single(aRun.Builder.NewFaces(1));
        Assert.Empty(aRun.Builder.NewFaces(2)); // a second copy of the surface, without interferences
        Assert.Equal(ShapeType.Face, aRun.Builder.NewFaces(1).First().Type);
        for (int iC = 1; iC <= aDS.NbCurves(); iC++)
        {
            // curves 3 and 4 have no points: no edge, as natively
            Assert.Equal(iC is 3 or 4 ? 0 : 1, aRun.Builder.NewEdges(iC).Extent());
        }
        Assert.Equal(Math.PI * 10 / 2, ReconComparison.EdgeLength(aRun.Builder.NewEdges(5).First()), 9);
        Assert.Equal(100.0, ReconComparison.EdgeLength(aRun.Builder.NewEdges(1).First()), 9);

        // the solid merged: one solid, of one closed shell, of seven faces
        Shape aSolid = aDS.Shape(1);
        Assert.Equal(ShapeType.Solid, aSolid.Type);
        NCollection_List<Shape> aMerged = aRun.Builder.Merged(aSolid, In);
        Assert.Single(aMerged);
        Assert.Equal(ShapeType.Solid, aMerged.First().Type);
        Assert.Equal(7, ReconComparison.Count(aMerged.First(), ShapeType.Face));
        Assert.Equal(1.0e6 - (100.0 - 25.0 * Math.PI) * 100.0, ReconComparison.Volume(aMerged.First()), 6);
        // the solid and its shell are split too, as the HBuilder answers (decision 1161: BRepFilletAPI's history)
        Assert.True(aRun.Builder.IsSplit(aSolid, In));
        Assert.True(aRun.Builder.Splits(aSolid, In).First().IsSame(aMerged.First()));
        Shape aShell = aDS.Shape(14);
        Assert.Equal(ShapeType.Shell, aShell.Type);
        Assert.Single(aRun.Builder.Splits(aShell, In));
        Assert.Empty(aRun.Builder.Merged(aSolid, Out));
        Assert.Empty(aRun.Builder.Merged(aSolid, On));
        Assert.True(new Analyzer(aRun.Result).IsValid());
    }

    [Fact]
    public void TheEdgesCutKeepTheirPartsIN()
    {
        // each edge the fillet cuts keeps one part, 90 long (100 less the radius): the part OUT is under the fillet
        ReconComparison.Run aRun = Box();
        DsDataStructure aDS = aRun.HDS.DS();
        int aNbEdges = 0;
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            Shape E = aDS.Shape(i);
            if (E.Type != ShapeType.Edge)
            {
                continue;
            }
            aNbEdges++;
            Assert.True(aRun.Builder.IsSplit(E, In));
            Assert.False(aRun.Builder.IsSplit(E, Out));
            Assert.False(aRun.Builder.IsSplit(E, On));
            NCollection_List<Shape> aParts = aRun.Builder.Splits(E, In);
            Assert.Single(aParts);
            Assert.Equal(90.0, ReconComparison.EdgeLength(aParts.First()), 9);
            Assert.Empty(aRun.Builder.Splits(E, Out));
        }
        Assert.Equal(4, aNbEdges);
    }

    [Fact]
    public void TheFacesCutAreSplitTheOthersNot()
    {
        // box_edge: the two faces along the edge and the two at its ends are cut; the two others are left whole
        ReconComparison.Run aRun = Box();
        IndexedSet<Shape> aFaces = new(ShapeHasher.Instance);
        Exploration.MapShapes(aRun.Input, ShapeType.Face, aFaces);
        IndexedSet<Shape> aResultFaces = new(ShapeHasher.Instance);
        Exploration.MapShapes(aRun.Result, ShapeType.Face, aResultFaces);
        int aNbSplit = 0;
        for (int i = 0; i < aFaces.Count; i++)
        {
            Shape F = aFaces[i];
            if (aRun.Builder.IsSplit(F, In))
            {
                aNbSplit++;
                Assert.Single(aRun.Builder.Splits(F, In));
                Assert.False(aResultFaces.Contains(F));
                Assert.True(aResultFaces.Contains(aRun.Builder.Splits(F, In).First()));
                // GMergeFaces binds a face's pieces too, and FilBuilder::SetRegul reads them to code the fillet's edges (decision 1719)
                Assert.Equal(aRun.Builder.Splits(F, In), aRun.Builder.Merged(F, In));
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

            Shape anInput = aCase.Input;
            string aBefore = BRepText(anInput);
            DsHDataStructure aHDS = TextDump.Read(aCase.DsText, anInput);
            BOPReconstruction R = new(new DsBuildTool());
            R.Perform(aHDS);
            R.MergeSolid(aHDS.DS().Shape(1), In);
            Assert.Equal(aBefore, BRepText(anInput));
        }
    }

    [Fact]
    public void PerformAgainStartsAfresh()
    {
        ReconCase aCase = ReconRecordings.Cases["box_edge"];
        Shape anInput = aCase.Input;
        DsHDataStructure aHDS = TextDump.Read(aCase.DsText, anInput);
        BOPReconstruction R = new(new DsBuildTool());
        R.Perform(aHDS);
        Shape aSolid = aHDS.DS().Shape(1);
        R.MergeSolid(aSolid, In);
        Shape aFirst = R.Merged(aSolid, In).First();
        R.MergeSolid(aSolid, Out); // only IN is built, as ChFi3d asks
        Assert.Single(R.Merged(aSolid, In));
        R.Perform(aHDS);
        Assert.Empty(R.Merged(aSolid, In));
        R.MergeSolid(aSolid, In);
        Assert.False(aFirst.IsSame(R.Merged(aSolid, In).First()));
        Assert.Equal(ReconComparison.Volume(aFirst), ReconComparison.Volume(R.Merged(aSolid, In).First()), 6);
    }

    [Fact]
    public void ASeamAndAClosedCurve()
    {
        // cylinder_top_edge: the torus's seam carries two pcurves; the circles are closed edges on one vertex
        ReconComparison.Run aRun = Box("cylinder_top_edge");
        Face aTorus = ShapeCasts.Face(aRun.Builder.NewFaces(1).First());
        Edge aSeam = ShapeCasts.Edge(aRun.Builder.NewEdges(5).First());
        Assert.True(Tool.IsClosed(aSeam, aTorus));
        Edge aCircle = ShapeCasts.Edge(aRun.Builder.NewEdges(2).First());
        Exploration.Vertices(aCircle, out Vertex V1, out Vertex V2);
        Assert.True(V1.IsSame(V2));
        Assert.True(new Analyzer(aRun.Result).IsValid());
    }

    [Fact]
    public void AShellStaysAShell()
    {
        ReconComparison.Run aRun = Box("shell_box_without_top");
        Shape aShell = aRun.HDS.DS().Shape(1);
        Assert.Equal(ShapeType.Shell, aShell.Type);
        NCollection_List<Shape> aMerged = aRun.Builder.Merged(aShell, In);
        Assert.Single(aMerged);
        Assert.Equal(ShapeType.Shell, aMerged.First().Type);
        Assert.False(Tool.IsClosed(aMerged.First()));
    }

    [Fact]
    public void AVoidKeepsItsShell()
    {
        ReconComparison.Run aRun = Box("solid_with_void_inner_edge");
        Shape aSolid = aRun.Builder.Merged(aRun.HDS.DS().Shape(1), In).First();
        Assert.Equal(2, ReconComparison.Count(aSolid, ShapeType.Shell));
    }

    private static string BRepText(in Shape S)
    {
        StringWriter w = new();
        BrepTools.Write(S, w);
        return w.ToString();
    }
}
