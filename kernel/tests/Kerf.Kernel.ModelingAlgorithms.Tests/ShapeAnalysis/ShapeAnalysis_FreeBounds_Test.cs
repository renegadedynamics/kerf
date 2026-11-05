// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeAnalysis_FreeBounds_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of ShapeAnalysis_FreeBounds.</summary>
public class ShapeAnalysis_FreeBoundsTest
{
    private sealed class TestData
    {
        public Edge Edge1;
        public Edge Edge2;
        public Edge Edge3;
        public Edge InternalEdge;
        public Edge ExternalEdge;
        public Wire Loop = new();
        public Wire InternalWire = new();
        public Wire ExternalWire = new();
        public Wire EmptyWire = new();
    }

    private static bool makeTestData(TestData theData)
    {
        var aBuilder = new BrepBuilder();
        Vertex aVertex1 = new(), aVertex2 = new(), aVertex3 = new(), aNonManifoldVertex = new();
        aBuilder.MakeVertex(ref aVertex1, new Point3D(0.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());
        aBuilder.MakeVertex(ref aVertex2, new Point3D(1.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());
        aBuilder.MakeVertex(ref aVertex3, new Point3D(0.0, 1.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());
        aBuilder.MakeVertex(ref aNonManifoldVertex, new Point3D(2.0, -1.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());

        var anEdgeMaker1 = new MakeEdge(aVertex1, aVertex2);
        var anEdgeMaker2 = new MakeEdge(aVertex2, aVertex3);
        var anEdgeMaker3 = new MakeEdge(aVertex3, aVertex1);
        var aNonManifoldMaker = new MakeEdge(aVertex1, aNonManifoldVertex);
        if (!anEdgeMaker1.IsDone || !anEdgeMaker2.IsDone || !anEdgeMaker3.IsDone
            || !aNonManifoldMaker.IsDone)
        {
            return false;
        }

        theData.Edge1 = anEdgeMaker1.Edge();
        theData.Edge2 = anEdgeMaker2.Edge();
        theData.Edge3 = anEdgeMaker3.Edge();
        theData.InternalEdge = aNonManifoldMaker.Edge();
        theData.InternalEdge.Orient(Internal);
        theData.ExternalEdge = aNonManifoldMaker.Edge();
        theData.ExternalEdge.Orient(External);

        aBuilder.MakeWire(ref theData.Loop);
        aBuilder.Add(theData.Loop, theData.Edge1);
        aBuilder.Add(theData.Loop, theData.Edge2);
        aBuilder.Add(theData.Loop, theData.Edge3);
        aBuilder.MakeWire(ref theData.InternalWire);
        aBuilder.Add(theData.InternalWire, theData.InternalEdge);
        aBuilder.MakeWire(ref theData.ExternalWire);
        aBuilder.Add(theData.ExternalWire, theData.ExternalEdge);
        aBuilder.MakeWire(ref theData.EmptyWire);
        return true;
    }

    private static void checkResult(NCollection_HSequence<Shape>? theResult, int theNbWires, int theNbEdges,
                                    int theNbInternalEdges, int theNbExternalEdges, bool theHasClosedLoop)
    {
        Assert.NotNull(theResult);
        Assert.Equal(theNbWires, theResult.Length());

        int aNbEdges = 0;
        int aNbInternalEdges = 0;
        int aNbExternalEdges = 0;
        bool hasClosedLoop = false;
        for (int i = 1; i <= theResult.Length(); i++)
        {
            Shape aWire = theResult.Value(i);
            int aNbWireEdges = 0;
            for (var anExp = new Explorer(aWire, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                ++aNbEdges;
                ++aNbWireEdges;
                if (anExp.Current().Orientation == Internal)
                {
                    ++aNbInternalEdges;
                }
                else if (anExp.Current().Orientation == External)
                {
                    ++aNbExternalEdges;
                }
            }
            if (aNbWireEdges == 3 && aWire.ClosedFlag)
            {
                hasClosedLoop = true;
            }
        }
        Assert.Equal(theNbEdges, aNbEdges);
        Assert.Equal(theNbInternalEdges, aNbInternalEdges);
        Assert.Equal(theNbExternalEdges, aNbExternalEdges);
        Assert.Equal(theHasClosedLoop, hasClosedLoop);
    }

    internal static Face MakeSquareFace(in Point3D theA, in Point3D theB, in Point3D theC, in Point3D theD)
    {
        var aMakeWire = new MakeWire();
        aMakeWire.Add(new MakeEdge(theA, theB).Edge());
        aMakeWire.Add(new MakeEdge(theB, theC).Edge());
        aMakeWire.Add(new MakeEdge(theC, theD).Edge());
        aMakeWire.Add(new MakeEdge(theD, theA).Edge());
        return new MakeFace(aMakeWire.Wire()).Face();
    }

    private static NCollection_HSequence<Shape> Seq(params Shape[] theShapes)
    {
        var aSeq = new NCollection_HSequence<Shape>();
        foreach (Shape aShape in theShapes)
        {
            aSeq.Append(aShape);
        }
        return aSeq;
    }

    [Fact]
    public void ConnectWires_InternalOnlyWireAfterManifoldWire()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<Shape> anInput = Seq(aData.Loop, aData.InternalWire);

        NCollection_HSequence<Shape> aResult =
            FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 2, 4, 1, 0, true);
    }

    [Fact]
    public void ConnectWires_InternalOnlyWireBeforeManifoldWire()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<Shape> anInput = Seq(aData.InternalWire, aData.Loop);

        NCollection_HSequence<Shape> aResult =
            FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 2, 4, 1, 0, true);
    }

    [Fact]
    public void ConnectWires_OnlyNonManifoldWires()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<Shape> anInput = Seq(aData.InternalWire, aData.ExternalWire);

        NCollection_HSequence<Shape> aResult =
            FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 2, 2, 1, 1, false);
    }

    [Fact]
    public void ConnectWires_EmptyWireBeforeValidWires()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<Shape> anInput = Seq(aData.EmptyWire, aData.InternalWire, aData.Loop);

        NCollection_HSequence<Shape> aResult =
            FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 2, 4, 1, 0, true);
    }

    [Fact]
    public void ConnectWires_OnlyEmptyWire()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<Shape> anInput = Seq(aData.EmptyWire);

        NCollection_HSequence<Shape> aResult =
            FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 0, 0, 0, 0, false);
    }

    [Fact]
    public void ConnectWires_MixedManifoldAndInternalEdges()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        var aBuilder = new BrepBuilder();
        Wire aMixedWire = new();
        aBuilder.MakeWire(ref aMixedWire);
        aBuilder.Add(aMixedWire, aData.Edge1);
        aBuilder.Add(aMixedWire, aData.Edge2);
        aBuilder.Add(aMixedWire, aData.Edge3);
        aBuilder.Add(aMixedWire, aData.InternalEdge);

        NCollection_HSequence<Shape> anInput = Seq(aMixedWire);

        NCollection_HSequence<Shape> aResult =
            FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 1, 4, 1, 0, false);
    }

    [Fact]
    public void ConnectEdges_InternalEdgeAfterManifoldEdges()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<Shape> anInput = Seq(aData.Edge1, aData.Edge2, aData.Edge3, aData.InternalEdge);

        NCollection_HSequence<Shape> aResult =
            FreeBounds.ConnectEdgesToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 1, 3, 0, 0, true);
    }

    [Fact]
    public void ConnectEdges_InternalEdgeBeforeManifoldEdges()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<Shape> anInput = Seq(aData.InternalEdge, aData.Edge1, aData.Edge2, aData.Edge3);

        NCollection_HSequence<Shape> aResult =
            FreeBounds.ConnectEdgesToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 1, 3, 0, 0, true);
    }

    [Fact]
    public void ConnectEdges_ExternalEdgeIsSkipped()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<Shape> anInput = Seq(aData.Edge1, aData.Edge2, aData.Edge3, aData.ExternalEdge);

        NCollection_HSequence<Shape> aResult =
            FreeBounds.ConnectEdgesToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 1, 3, 0, 0, true);
    }

    // Baseline: a single closed face alone does not trigger the bug (see below).
    [Fact]
    public void SingleFaceNoCrash()
    {
        Face aFace = MakeSquareFace(new Point3D(0.0, 0.0, 0.0), new Point3D(1.0, 0.0, 0.0),
                                           new Point3D(1.0, 1.0, 0.0), new Point3D(0.0, 1.0, 0.0));

        var aFreeBounds = new FreeBounds(aFace, 0.01, true, true);
        _ = aFreeBounds.GetClosedWires();
        _ = aFreeBounds.GetOpenWires();
    }

    // Regression: two disjoint closed faces used to crash SplitWires (see PR description).
    [Fact]
    public void TwoDisjointFacesNoCrash()
    {
        Face aFaceA = MakeSquareFace(new Point3D(0.0, 0.0, 0.0), new Point3D(1.0, 0.0, 0.0),
                                            new Point3D(1.0, 1.0, 0.0), new Point3D(0.0, 1.0, 0.0));
        Face aFaceB = MakeSquareFace(new Point3D(5.0, 5.0, 0.0), new Point3D(6.0, 5.0, 0.0),
                                            new Point3D(6.0, 6.0, 0.0), new Point3D(5.0, 6.0, 0.0));

        var aBuilder = new BrepBuilder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aFaceA);
        aBuilder.Add(aCompound, aFaceB);

        var aFreeBounds = new FreeBounds(aCompound, 0.01, true, true);
        Compound aClosed = aFreeBounds.GetClosedWires();
        Compound anOpen = aFreeBounds.GetOpenWires();

        int aNbClosed = 0;
        for (var anExp = new Explorer(aClosed, ShapeType.Wire); anExp.More(); anExp.Next())
        {
            ++aNbClosed;
        }
        int aNbOpen = 0;
        for (var anExp = new Explorer(anOpen, ShapeType.Wire); anExp.More(); anExp.Next())
        {
            ++aNbOpen;
        }

        Assert.Equal(2, aNbClosed);
        Assert.Equal(0, aNbOpen);
    }
}
