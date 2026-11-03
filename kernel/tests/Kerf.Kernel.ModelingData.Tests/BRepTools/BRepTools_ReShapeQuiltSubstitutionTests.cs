// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// BRepTools_ReShape, BRepTools_Quilt and BRepTools_Substitution on frozen inputs: OCCT's results, frozen when handed
/// out, and inputs left unchanged (every TShape, flag, child and representation, by identity).
/// </summary>
public class BRepTools_ReShapeQuiltSubstitutionTests
{
    private static Shape SubShape(in Shape theShape, ShapeType theType, int theIndex)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap[theIndex];
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static bool Contains(in Shape theShape, in Shape theSub)
    {
        for (var anExp = new Explorer(theShape, theSub.ShapeType()); anExp.More(); anExp.Next())
        {
            if (anExp.Current().IsSame(theSub))
            {
                return true;
            }
        }

        return false;
    }

    [Fact]
    public void ReShape_ReplaceVertexOfFrozenBox_RebuildsAncestorsAndLeavesInputUnchanged()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);

        Vertex aV = ShapeCasts.Vertex(SubShape(aBox, ShapeType.Vertex, 1));
        Vertex aNewV = TestShapes.Frozen(TestShapes.Vertex(new Point3D(-0.1, 0, 0)));
        var aReShape = new ReShape();
        aReShape.Replace(aV, aNewV);
        Shape aResult = aReShape.Apply(aBox);

        aSnapshot.AssertUnchanged(aBox);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.False(aResult.IsSame(aBox));
        Assert.Equal(8, Count(aResult, ShapeType.Vertex));
        Assert.Equal(12, Count(aResult, ShapeType.Edge));
        Assert.Equal(6, Count(aResult, ShapeType.Face));
        Assert.True(Contains(aResult, aNewV));
        Assert.False(Contains(aResult, aV));

        // The three faces away from the vertex are shared, the three at it are rebuilt.
        int aShared = 0;
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aShared += Contains(aResult, anExp.Current()) ? 1 : 0;
        }

        Assert.Equal(3, aShared);

        // Rebuilt edges keep their ranges (CopyRanges), rebuilt wires and shells their closure.
        for (var anExp = new Explorer(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            Shape aNewE = aReShape.Value(anE);
            Tool.Range(anE, out double f1, out double l1);
            Tool.Range(ShapeCasts.Edge(aNewE), out double f2, out double l2);
            Assert.Equal(f1, f2);
            Assert.Equal(l1, l2);
        }

        for (var anExp = new Explorer(aResult, ShapeType.Wire); anExp.More(); anExp.Next())
        {
            Assert.True(anExp.Current().Closed());
        }

        // The history answers for the input's sub-shapes.
        History aHistory = aReShape.History();
        Assert.Single(aHistory.Modified(aV));
        Assert.True(aHistory.Modified(aV)[0].IsSame(aNewV));
        int aModifiedEdges = 0;
        for (int i = 1; i <= Count(aBox, ShapeType.Edge); i++)
        {
            foreach (Shape aM in aHistory.Modified(SubShape(aBox, ShapeType.Edge, i)))
            {
                Assert.True(aM.TShape()!.IsFrozen);
                aModifiedEdges++;
            }
        }

        Assert.Equal(3, aModifiedEdges);
        Assert.True(aHistory.Modified(aBox).Count == 1);
        aSnapshot.AssertUnchanged(aBox);
    }

    [Fact]
    public void ReShape_RemoveFaceOfFrozenBox()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        Shape aF = SubShape(aBox, ShapeType.Face, 2);

        var aReShape = new ReShape();
        aReShape.Remove(aF);
        Assert.True(aReShape.IsRecorded(aF));
        Assert.Equal(-1, aReShape.Status(aF, out Shape aNewSh));
        Assert.True(aNewSh.IsNull);

        Shape aResult = aReShape.Apply(aBox);
        aSnapshot.AssertUnchanged(aBox);
        Assert.Equal(5, Count(aResult, ShapeType.Face));
        Assert.True(aResult.TShape()!.IsFrozen);

        // The rebuilt shell is no longer closed.
        Shape aShell = SubShape(aResult, ShapeType.Shell, 1);
        Assert.False(aShell.Closed());

        History aHistory = aReShape.History();
        Assert.True(aHistory.IsRemoved(aF));
        Assert.Empty(aHistory.Modified(aF));
    }

    [Fact]
    public void ReShape_ApplyUntil_StopsAtTheType()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Vertex aV = ShapeCasts.Vertex(SubShape(aBox, ShapeType.Vertex, 1));
        var aReShape = new ReShape();
        aReShape.Replace(aV, TestShapes.Frozen(TestShapes.Vertex(new Point3D(5, 5, 5))));
        // faces are neither rebuilt nor explored
        Shape aResult = aReShape.Apply(aBox, ShapeType.Face);
        Assert.True(aResult.IsSame(aBox));
    }

    [Fact]
    public void ReShape_ReversedAndInternalOrientations()
    {
        Vertex aV1 = TestShapes.Frozen(TestShapes.Vertex(new Point3D(0, 0, 0)));
        Vertex aV2 = TestShapes.Frozen(TestShapes.Vertex(new Point3D(1, 0, 0)));
        Edge aE = TestShapes.Frozen(TestShapes.Edge(aV1, aV2));
        Edge aE2 = TestShapes.Frozen(TestShapes.Edge(aV1, aV2));

        var aReShape = new ReShape();
        // Recording a REVERSED shape records both reversed.
        aReShape.Replace(aE.Reversed(), aE2);
        Assert.Equal(Reversed, aReShape.Value(aE).Orientation());
        Assert.Equal(Forward, aReShape.Value(aE.Reversed()).Orientation());
        // INTERNAL keeps its orientation.
        Assert.Equal(Internal, aReShape.Value(aE.Oriented(Internal)).Orientation());
        Assert.Equal(1, aReShape.Status(aE, out Shape aNewSh));
        Assert.True(aNewSh.IsSame(aE2));
        Assert.True(aReShape.IsNewShape(aE2));
        Assert.False(aReShape.IsNewShape(aE));
    }

    [Fact]
    public void ReShape_ConsiderLocation()
    {
        Vertex aA = TestShapes.Frozen(TestShapes.Vertex(new Point3D(0, 0, 0)));
        Vertex aB = TestShapes.Frozen(TestShapes.Vertex(new Point3D(1, 0, 0)));
        var aT = new Transform3D();
        aT = aT.WithTranslation(new Vector3D(0, 0, 5));
        var aL = new Placement(aT);

        // Without the mode, a located copy of a recorded shape isn't recorded (IsSame compares locations).
        var aPlain = new ReShape();
        aPlain.Replace(aA, aB);
        Assert.False(aPlain.IsRecorded(aA.Located(aL)));

        var aReShape = new ReShape();
        aReShape.ModeConsiderLocation(true);
        Assert.True(aReShape.ModeConsiderLocation());
        aReShape.Replace(aA, aB);
        Assert.True(aReShape.IsRecorded(aA.Located(aL)));
        Shape aValue = aReShape.Value(aA.Located(aL));
        Assert.True(aValue.IsPartner(aB));
        Assert.True(aValue.Location().IsEqual(aL));
    }

    [Fact]
    public void ReShape_Merge_HistoryPresentsThePartsAlike()
    {
        Vertex aV1 = TestShapes.Frozen(TestShapes.Vertex(new Point3D(0, 0, 0)));
        Vertex aV2 = TestShapes.Frozen(TestShapes.Vertex(new Point3D(1, 0, 0)));
        Vertex aV3 = TestShapes.Frozen(TestShapes.Vertex(new Point3D(2, 0, 0)));
        Edge aE1 = TestShapes.Frozen(TestShapes.Edge(aV1, aV2));
        Edge aE2 = TestShapes.Frozen(TestShapes.Edge(aV2, aV3));
        Edge aProduct = TestShapes.Frozen(TestShapes.Edge(aV1, aV3));

        var aReShape = new ReShape();
        aReShape.Merge([aE1, aE2], aProduct);
        // the ordinary part is removed by Value, the main one replaced
        Assert.True(aReShape.Value(aE1).IsSame(aProduct));
        Assert.True(aReShape.Value(aE2).IsNull);
        History aHistory = aReShape.History();
        Assert.True(aHistory.Modified(aE1)[0].IsSame(aProduct));
        Assert.True(aHistory.Modified(aE2)[0].IsSame(aProduct));
    }

    [Fact]
    public void ReShape_InternalApply_LeavesTheResultUnfrozen_PublicFreezes()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Vertex aV = ShapeCasts.Vertex(SubShape(aBox, ShapeType.Vertex, 1));
        var aReShape = new ReShape();
        aReShape.Replace(aV, TestShapes.Frozen(TestShapes.Vertex(new Point3D(-1, 0, 0))));
        Shape anInternal = aReShape.InternalApply(aBox);
        Assert.False(anInternal.TShape()!.IsFrozen);
        // the recorded intermediate results are unfrozen too until handed out
        Shape anEdge = aReShape.InternalValue(SubShape(aBox, ShapeType.Edge, 1));
        Assert.False(anEdge.TShape()!.IsFrozen);
        Assert.True(aReShape.Value(SubShape(aBox, ShapeType.Edge, 1)).TShape()!.IsFrozen);
        Assert.True(aReShape.Apply(aBox).TShape()!.IsFrozen);
    }

    [Fact]
    public void ReShape_CopyVertex()
    {
        Vertex aV = TestShapes.Frozen(TestShapes.Vertex(new Point3D(1, 2, 3)));
        var aReShape = new ReShape();
        Vertex aCopy = aReShape.CopyVertex(aV, 0.5);
        Assert.False(aCopy.IsSame(aV));
        Assert.Equal(0.5, Tool.Tolerance(aCopy));
        Assert.Equal(1e-7, Tool.Tolerance(aV));
        // (the public Value would freeze the copy: ported code reads InternalValue)
        Assert.True(aReShape.InternalValue(aV).IsSame(aCopy));
        // the recorded copy is changed in place
        Vertex aSecond = aReShape.CopyVertex(aV, new Point3D(4, 5, 6), 0.25);
        Assert.True(aSecond.IsSame(aCopy));
        Assert.Equal(4.0, Tool.Pnt(aCopy).X());

        // a recorded frozen replacement can't be changed in place: OCCT's guard for locked shapes
        var aFrozenCase = new ReShape();
        aFrozenCase.Replace(aV, TestShapes.Frozen(TestShapes.Vertex(new Point3D(0, 0, 0))));
        Assert.Throws<LockedShapeException>(() => aFrozenCase.CopyVertex(aV, 0.5));
    }

    [Fact]
    public void Quilt_FacesOfABox_GiveOneClosedShell()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        var aQuilt = new Quilt();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aQuilt.Add(anExp.Current());
        }

        Shape aShells = aQuilt.Shells();
        aSnapshot.AssertUnchanged(aBox);
        Assert.True(aShells.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Compound, aShells.ShapeType());
        Assert.Equal(1, aShells.ChildCount);
        Shape aShell = new Iterator(aShells).Value();
        Assert.True(aShell.Closed());
        Assert.True(aShell.Orientable());
        Assert.Equal(6, Count(aShell, ShapeType.Face));
        Assert.Equal(12, Count(aShell, ShapeType.Edge));
    }

    [Fact]
    public void Quilt_OpenFaces_GiveOpenShells()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aQuilt = new Quilt();
        // two opposite faces: two shells, each open
        aQuilt.Add(SubShape(aBox, ShapeType.Face, 1));
        aQuilt.Add(SubShape(aBox, ShapeType.Face, 2));
        Shape aShells = aQuilt.Shells();
        Assert.Equal(2, aShells.ChildCount);
        for (var anIt = new Iterator(aShells); anIt.More(); anIt.Next())
        {
            Assert.False(anIt.Value().Closed());
        }
    }

    // Two unit squares side by side, each with its own edges; binding the shared side of the second to the first's
    // copies the second face with the first's edge, which receives a pcurve on it.
    private static (Face, Face, Edge, Edge) TwoSquares()
    {
        Vertex[] aV1 = [TestShapes.Vertex(new Point3D(0, 0, 0)), TestShapes.Vertex(new Point3D(1, 0, 0)),
                               TestShapes.Vertex(new Point3D(1, 1, 0)), TestShapes.Vertex(new Point3D(0, 1, 0))];
        Vertex[] aV2 = [TestShapes.Vertex(new Point3D(1, 0, 0)), TestShapes.Vertex(new Point3D(2, 0, 0)),
                               TestShapes.Vertex(new Point3D(2, 1, 0)), TestShapes.Vertex(new Point3D(1, 1, 0))];
        var anEdges1 = new Dictionary<(int, int), Edge>();
        var anEdges2 = new Dictionary<(int, int), Edge>();
        Face aF1 = TestShapes.PolygonFace(aV1, new Direction3D(0, 0, 1), anEdges1, [0, 1, 2, 3]);
        Face aF2 = TestShapes.PolygonFace(aV2, new Direction3D(0, 0, 1), anEdges2, [0, 1, 2, 3]);
        // the first square's (1,0)-(1,1) edge and the second's (1,0)-(1,1) edge: same direction
        return (aF1, aF2, anEdges1[(1, 2)], anEdges2[(0, 3)]);
    }

    [Fact]
    public void Quilt_BindEdge_GluesTheFaces()
    {
        (Face aF1, Face aF2, Edge anE1, Edge anE2) = TwoSquares();
        // the second face is a caller's frozen shape; the first face's edge will receive a pcurve, so it can't be
        TestShapes.Frozen(aF2);
        var aSnapshot2 = ShapeSnapshot.Of(aF2);
        var aQuilt = new Quilt();
        aQuilt.Add(aF1);
        aQuilt.Bind(anE2, anE1);
        aQuilt.Add(aF2);

        Assert.True(aQuilt.IsCopied(aF2));
        Assert.False(aQuilt.IsCopied(aF1));
        Assert.True(aQuilt.IsCopied(anE2));
        Assert.True(aQuilt.Copy(anE2).IsSame(anE1));
        Shape aCopy = aQuilt.Copy(aF2);
        Assert.True(aCopy.TShape()!.IsFrozen);
        Assert.True(Contains(aCopy, anE1));
        Assert.NotNull(Tool.CurveOnSurface(anE1, ShapeCasts.Face(aCopy), out _, out _));
        aSnapshot2.AssertUnchanged(aF2);

        Shape aShells = aQuilt.Shells();
        Assert.Equal(1, aShells.ChildCount);
        Shape aShell = new Iterator(aShells).Value();
        Assert.Equal(2, Count(aShell, ShapeType.Face));
        Assert.Equal(7, Count(aShell, ShapeType.Edge));
        Assert.False(aShell.Closed());
    }

    [Fact]
    public void Quilt_BindToAFrozenEdge_Throws()
    {
        (Face aF1, Face aF2, Edge anE1, Edge anE2) = TwoSquares();
        TestShapes.Frozen(aF1);
        TestShapes.Frozen(aF2);
        var aQuilt = new Quilt();
        aQuilt.Add(aF1);
        aQuilt.Bind(anE2, anE1);
        // OCCT gives the frozen first edge a pcurve on the second face in place
        Assert.Throws<LockedShapeException>(() => aQuilt.Add(aF2));
    }

    [Fact]
    public void Substitution_ReplaceAnEdgeOfAFrozenFace()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Face aF = ShapeCasts.Face(SubShape(aBox, ShapeType.Face, 1));
        var aSnapshot = ShapeSnapshot.Of(aF);
        Edge anE = ShapeCasts.Edge(SubShape(aF, ShapeType.Edge, 1));

        // replace the edge by two halves, and remove another edge
        Vertex aV1 = Exploration.FirstVertex(anE), aV2 = Exploration.LastVertex(anE);
        Point3D aMid = Tool.Pnt(aV1).Translated(new Vector3D(Tool.Pnt(aV1), Tool.Pnt(aV2)) * 0.5);
        Vertex aVM = TestShapes.Frozen(TestShapes.Vertex(aMid));
        var aNew = new NCollection_List<Shape>();
        aNew.Append(TestShapes.Frozen(TestShapes.Edge(aV1, aVM)));
        aNew.Append(TestShapes.Frozen(TestShapes.Edge(aVM, aV2)));
        Shape anOther = SubShape(aF, ShapeType.Edge, 3);

        var aSubst = new Substitution();
        aSubst.Substitute(anE, aNew);
        aSubst.Substitute(anOther, new NCollection_List<Shape>());
        Assert.Throws<ConstructionErrorException>(() => aSubst.Substitute(anE, aNew));
        aSubst.Build(aF);

        aSnapshot.AssertUnchanged(aF);
        Assert.True(aSubst.IsCopied(aF));
        Assert.True(aSubst.IsCopied(anE));
        Assert.True(aSubst.IsCopied(anOther));
        Assert.Empty(aSubst.Copy(anOther));
        Assert.Throws<NoSuchObjectException>(() => aSubst.Copy(SubShape(aF, ShapeType.Edge, 2)));
        var aCopies = aSubst.Copy(aF);
        Assert.Single(aCopies);
        Shape aNewF = aCopies[0];
        Assert.True(aNewF.TShape()!.IsFrozen);
        Assert.Equal(Forward, aNewF.Orientation());
        Assert.Equal(4, Count(aNewF, ShapeType.Edge));
        Assert.False(Contains(aNewF, anE));
        Assert.False(Contains(aNewF, anOther));

        aSubst.Clear();
        Assert.False(aSubst.IsCopied(aF));
    }

    [Fact]
    public void Substitution_EmptyContainerDisappears()
    {
        Vertex aV1 = TestShapes.Frozen(TestShapes.Vertex(new Point3D(0, 0, 0)));
        Vertex aV2 = TestShapes.Frozen(TestShapes.Vertex(new Point3D(1, 0, 0)));
        Edge anE = TestShapes.Frozen(TestShapes.Edge(aV1, aV2));
        var aB = new BrepBuilder();
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        TestShapes.Frozen(aW);

        var aSubst = new Substitution();
        aSubst.Substitute(anE, new NCollection_List<Shape>());
        aSubst.Build(aW);
        Assert.True(aSubst.IsCopied(aW));
        Assert.Empty(aSubst.Copy(aW));
    }
}
