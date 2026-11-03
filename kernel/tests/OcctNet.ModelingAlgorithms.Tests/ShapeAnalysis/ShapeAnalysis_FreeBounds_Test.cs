// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeAnalysis_FreeBounds_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of ShapeAnalysis_FreeBounds.</summary>
public class ShapeAnalysis_FreeBoundsTest
{
    private sealed class TestData
    {
        public TopoDS_Edge Edge1;
        public TopoDS_Edge Edge2;
        public TopoDS_Edge Edge3;
        public TopoDS_Edge InternalEdge;
        public TopoDS_Edge ExternalEdge;
        public TopoDS_Wire Loop = new();
        public TopoDS_Wire InternalWire = new();
        public TopoDS_Wire ExternalWire = new();
        public TopoDS_Wire EmptyWire = new();
    }

    private static bool makeTestData(TestData theData)
    {
        var aBuilder = new BRep_Builder();
        TopoDS_Vertex aVertex1 = new(), aVertex2 = new(), aVertex3 = new(), aNonManifoldVertex = new();
        aBuilder.MakeVertex(ref aVertex1, new Point3D(0.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());
        aBuilder.MakeVertex(ref aVertex2, new Point3D(1.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());
        aBuilder.MakeVertex(ref aVertex3, new Point3D(0.0, 1.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());
        aBuilder.MakeVertex(ref aNonManifoldVertex, new Point3D(2.0, -1.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion());

        var anEdgeMaker1 = new BRepBuilderAPI_MakeEdge(aVertex1, aVertex2);
        var anEdgeMaker2 = new BRepBuilderAPI_MakeEdge(aVertex2, aVertex3);
        var anEdgeMaker3 = new BRepBuilderAPI_MakeEdge(aVertex3, aVertex1);
        var aNonManifoldMaker = new BRepBuilderAPI_MakeEdge(aVertex1, aNonManifoldVertex);
        if (!anEdgeMaker1.IsDone() || !anEdgeMaker2.IsDone() || !anEdgeMaker3.IsDone()
            || !aNonManifoldMaker.IsDone())
        {
            return false;
        }

        theData.Edge1 = anEdgeMaker1.Edge();
        theData.Edge2 = anEdgeMaker2.Edge();
        theData.Edge3 = anEdgeMaker3.Edge();
        theData.InternalEdge = aNonManifoldMaker.Edge();
        theData.InternalEdge.Orientation(TopAbs_INTERNAL);
        theData.ExternalEdge = aNonManifoldMaker.Edge();
        theData.ExternalEdge.Orientation(TopAbs_EXTERNAL);

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

    private static void checkResult(NCollection_HSequence<TopoDS_Shape>? theResult, int theNbWires, int theNbEdges,
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
            TopoDS_Shape aWire = theResult.Value(i);
            int aNbWireEdges = 0;
            for (var anExp = new TopExp_Explorer(aWire, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                ++aNbEdges;
                ++aNbWireEdges;
                if (anExp.Current().Orientation() == TopAbs_INTERNAL)
                {
                    ++aNbInternalEdges;
                }
                else if (anExp.Current().Orientation() == TopAbs_EXTERNAL)
                {
                    ++aNbExternalEdges;
                }
            }
            if (aNbWireEdges == 3 && aWire.Closed())
            {
                hasClosedLoop = true;
            }
        }
        Assert.Equal(theNbEdges, aNbEdges);
        Assert.Equal(theNbInternalEdges, aNbInternalEdges);
        Assert.Equal(theNbExternalEdges, aNbExternalEdges);
        Assert.Equal(theHasClosedLoop, hasClosedLoop);
    }

    internal static TopoDS_Face MakeSquareFace(in Point3D theA, in Point3D theB, in Point3D theC, in Point3D theD)
    {
        var aMakeWire = new BRepBuilderAPI_MakeWire();
        aMakeWire.Add(new BRepBuilderAPI_MakeEdge(theA, theB).Edge());
        aMakeWire.Add(new BRepBuilderAPI_MakeEdge(theB, theC).Edge());
        aMakeWire.Add(new BRepBuilderAPI_MakeEdge(theC, theD).Edge());
        aMakeWire.Add(new BRepBuilderAPI_MakeEdge(theD, theA).Edge());
        return new BRepBuilderAPI_MakeFace(aMakeWire.Wire()).Face();
    }

    private static NCollection_HSequence<TopoDS_Shape> Seq(params TopoDS_Shape[] theShapes)
    {
        var aSeq = new NCollection_HSequence<TopoDS_Shape>();
        foreach (TopoDS_Shape aShape in theShapes)
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
        NCollection_HSequence<TopoDS_Shape> anInput = Seq(aData.Loop, aData.InternalWire);

        NCollection_HSequence<TopoDS_Shape> aResult =
            ShapeAnalysis_FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 2, 4, 1, 0, true);
    }

    [Fact]
    public void ConnectWires_InternalOnlyWireBeforeManifoldWire()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<TopoDS_Shape> anInput = Seq(aData.InternalWire, aData.Loop);

        NCollection_HSequence<TopoDS_Shape> aResult =
            ShapeAnalysis_FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 2, 4, 1, 0, true);
    }

    [Fact]
    public void ConnectWires_OnlyNonManifoldWires()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<TopoDS_Shape> anInput = Seq(aData.InternalWire, aData.ExternalWire);

        NCollection_HSequence<TopoDS_Shape> aResult =
            ShapeAnalysis_FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 2, 2, 1, 1, false);
    }

    [Fact]
    public void ConnectWires_EmptyWireBeforeValidWires()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<TopoDS_Shape> anInput = Seq(aData.EmptyWire, aData.InternalWire, aData.Loop);

        NCollection_HSequence<TopoDS_Shape> aResult =
            ShapeAnalysis_FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 2, 4, 1, 0, true);
    }

    [Fact]
    public void ConnectWires_OnlyEmptyWire()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<TopoDS_Shape> anInput = Seq(aData.EmptyWire);

        NCollection_HSequence<TopoDS_Shape> aResult =
            ShapeAnalysis_FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 0, 0, 0, 0, false);
    }

    [Fact]
    public void ConnectWires_MixedManifoldAndInternalEdges()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        var aBuilder = new BRep_Builder();
        TopoDS_Wire aMixedWire = new();
        aBuilder.MakeWire(ref aMixedWire);
        aBuilder.Add(aMixedWire, aData.Edge1);
        aBuilder.Add(aMixedWire, aData.Edge2);
        aBuilder.Add(aMixedWire, aData.Edge3);
        aBuilder.Add(aMixedWire, aData.InternalEdge);

        NCollection_HSequence<TopoDS_Shape> anInput = Seq(aMixedWire);

        NCollection_HSequence<TopoDS_Shape> aResult =
            ShapeAnalysis_FreeBounds.ConnectWiresToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 1, 4, 1, 0, false);
    }

    [Fact]
    public void ConnectEdges_InternalEdgeAfterManifoldEdges()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<TopoDS_Shape> anInput = Seq(aData.Edge1, aData.Edge2, aData.Edge3, aData.InternalEdge);

        NCollection_HSequence<TopoDS_Shape> aResult =
            ShapeAnalysis_FreeBounds.ConnectEdgesToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 1, 3, 0, 0, true);
    }

    [Fact]
    public void ConnectEdges_InternalEdgeBeforeManifoldEdges()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<TopoDS_Shape> anInput = Seq(aData.InternalEdge, aData.Edge1, aData.Edge2, aData.Edge3);

        NCollection_HSequence<TopoDS_Shape> aResult =
            ShapeAnalysis_FreeBounds.ConnectEdgesToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 1, 3, 0, 0, true);
    }

    [Fact]
    public void ConnectEdges_ExternalEdgeIsSkipped()
    {
        var aData = new TestData();
        Assert.True(makeTestData(aData));
        NCollection_HSequence<TopoDS_Shape> anInput = Seq(aData.Edge1, aData.Edge2, aData.Edge3, aData.ExternalEdge);

        NCollection_HSequence<TopoDS_Shape> aResult =
            ShapeAnalysis_FreeBounds.ConnectEdgesToWires(anInput, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        checkResult(aResult, 1, 3, 0, 0, true);
    }

    // Baseline: a single closed face alone does not trigger the bug (see below).
    [Fact]
    public void SingleFaceNoCrash()
    {
        TopoDS_Face aFace = MakeSquareFace(new Point3D(0.0, 0.0, 0.0), new Point3D(1.0, 0.0, 0.0),
                                           new Point3D(1.0, 1.0, 0.0), new Point3D(0.0, 1.0, 0.0));

        var aFreeBounds = new ShapeAnalysis_FreeBounds(aFace, 0.01, true, true);
        _ = aFreeBounds.GetClosedWires();
        _ = aFreeBounds.GetOpenWires();
    }

    // Regression: two disjoint closed faces used to crash SplitWires (see PR description).
    [Fact]
    public void TwoDisjointFacesNoCrash()
    {
        TopoDS_Face aFaceA = MakeSquareFace(new Point3D(0.0, 0.0, 0.0), new Point3D(1.0, 0.0, 0.0),
                                            new Point3D(1.0, 1.0, 0.0), new Point3D(0.0, 1.0, 0.0));
        TopoDS_Face aFaceB = MakeSquareFace(new Point3D(5.0, 5.0, 0.0), new Point3D(6.0, 5.0, 0.0),
                                            new Point3D(6.0, 6.0, 0.0), new Point3D(5.0, 6.0, 0.0));

        var aBuilder = new BRep_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aFaceA);
        aBuilder.Add(aCompound, aFaceB);

        var aFreeBounds = new ShapeAnalysis_FreeBounds(aCompound, 0.01, true, true);
        TopoDS_Compound aClosed = aFreeBounds.GetClosedWires();
        TopoDS_Compound anOpen = aFreeBounds.GetOpenWires();

        int aNbClosed = 0;
        for (var anExp = new TopExp_Explorer(aClosed, TopAbs_WIRE); anExp.More(); anExp.Next())
        {
            ++aNbClosed;
        }
        int aNbOpen = 0;
        for (var anExp = new TopExp_Explorer(anOpen, TopAbs_WIRE); anExp.More(); anExp.Next())
        {
            ++aNbOpen;
        }

        Assert.Equal(2, aNbClosed);
        Assert.Equal(0, aNbOpen);
    }
}
