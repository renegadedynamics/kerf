// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// BRepTools_ReShape, BRepTools_Quilt and BRepTools_Substitution on frozen inputs: OCCT's results, frozen when handed
/// out, and inputs left unchanged (every TShape, flag, child and representation, by identity).
/// </summary>
public class BRepTools_ReShapeQuiltSubstitutionTests
{
    private static TopoDS_Shape SubShape(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType, int theIndex)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap[theIndex];
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static bool Contains(in TopoDS_Shape theShape, in TopoDS_Shape theSub)
    {
        for (var anExp = new TopExp_Explorer(theShape, theSub.ShapeType()); anExp.More(); anExp.Next())
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
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);

        TopoDS_Vertex aV = TopoDS.Vertex(SubShape(aBox, TopAbs_VERTEX, 1));
        TopoDS_Vertex aNewV = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(-0.1, 0, 0)));
        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aV, aNewV);
        TopoDS_Shape aResult = aReShape.Apply(aBox);

        aSnapshot.AssertUnchanged(aBox);
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.False(aResult.IsSame(aBox));
        Assert.Equal(8, Count(aResult, TopAbs_VERTEX));
        Assert.Equal(12, Count(aResult, TopAbs_EDGE));
        Assert.Equal(6, Count(aResult, TopAbs_FACE));
        Assert.True(Contains(aResult, aNewV));
        Assert.False(Contains(aResult, aV));

        // The three faces away from the vertex are shared, the three at it are rebuilt.
        int aShared = 0;
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aShared += Contains(aResult, anExp.Current()) ? 1 : 0;
        }

        Assert.Equal(3, aShared);

        // Rebuilt edges keep their ranges (CopyRanges), rebuilt wires and shells their closure.
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            TopoDS_Shape aNewE = aReShape.Value(anE);
            BRep_Tool.Range(anE, out double f1, out double l1);
            BRep_Tool.Range(TopoDS.Edge(aNewE), out double f2, out double l2);
            Assert.Equal(f1, f2);
            Assert.Equal(l1, l2);
        }

        for (var anExp = new TopExp_Explorer(aResult, TopAbs_WIRE); anExp.More(); anExp.Next())
        {
            Assert.True(anExp.Current().Closed());
        }

        // The history answers for the input's sub-shapes.
        BRepTools_History aHistory = aReShape.History();
        Assert.Single(aHistory.Modified(aV));
        Assert.True(aHistory.Modified(aV)[0].IsSame(aNewV));
        int aModifiedEdges = 0;
        for (int i = 1; i <= Count(aBox, TopAbs_EDGE); i++)
        {
            foreach (TopoDS_Shape aM in aHistory.Modified(SubShape(aBox, TopAbs_EDGE, i)))
            {
                Assert.True(aM.TShape()!.IsFrozen());
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
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aF = SubShape(aBox, TopAbs_FACE, 2);

        var aReShape = new BRepTools_ReShape();
        aReShape.Remove(aF);
        Assert.True(aReShape.IsRecorded(aF));
        Assert.Equal(-1, aReShape.Status(aF, out TopoDS_Shape aNewSh));
        Assert.True(aNewSh.IsNull());

        TopoDS_Shape aResult = aReShape.Apply(aBox);
        aSnapshot.AssertUnchanged(aBox);
        Assert.Equal(5, Count(aResult, TopAbs_FACE));
        Assert.True(aResult.TShape()!.IsFrozen());

        // The rebuilt shell is no longer closed.
        TopoDS_Shape aShell = SubShape(aResult, TopAbs_SHELL, 1);
        Assert.False(aShell.Closed());

        BRepTools_History aHistory = aReShape.History();
        Assert.True(aHistory.IsRemoved(aF));
        Assert.Empty(aHistory.Modified(aF));
    }

    [Fact]
    public void ReShape_ApplyUntil_StopsAtTheType()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Vertex aV = TopoDS.Vertex(SubShape(aBox, TopAbs_VERTEX, 1));
        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aV, TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(5, 5, 5))));
        // faces are neither rebuilt nor explored
        TopoDS_Shape aResult = aReShape.Apply(aBox, TopAbs_FACE);
        Assert.True(aResult.IsSame(aBox));
    }

    [Fact]
    public void ReShape_ReversedAndInternalOrientations()
    {
        TopoDS_Vertex aV1 = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(0, 0, 0)));
        TopoDS_Vertex aV2 = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(1, 0, 0)));
        TopoDS_Edge aE = TestShapes.Frozen(TestShapes.Edge(aV1, aV2));
        TopoDS_Edge aE2 = TestShapes.Frozen(TestShapes.Edge(aV1, aV2));

        var aReShape = new BRepTools_ReShape();
        // Recording a REVERSED shape records both reversed.
        aReShape.Replace(aE.Reversed(), aE2);
        Assert.Equal(TopAbs_REVERSED, aReShape.Value(aE).Orientation());
        Assert.Equal(TopAbs_FORWARD, aReShape.Value(aE.Reversed()).Orientation());
        // INTERNAL keeps its orientation.
        Assert.Equal(TopAbs_INTERNAL, aReShape.Value(aE.Oriented(TopAbs_INTERNAL)).Orientation());
        Assert.Equal(1, aReShape.Status(aE, out TopoDS_Shape aNewSh));
        Assert.True(aNewSh.IsSame(aE2));
        Assert.True(aReShape.IsNewShape(aE2));
        Assert.False(aReShape.IsNewShape(aE));
    }

    [Fact]
    public void ReShape_ConsiderLocation()
    {
        TopoDS_Vertex aA = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(0, 0, 0)));
        TopoDS_Vertex aB = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(1, 0, 0)));
        var aT = new gp_Trsf();
        aT.SetTranslation(new gp_Vec(0, 0, 5));
        var aL = new TopLoc_Location(aT);

        // Without the mode, a located copy of a recorded shape isn't recorded (IsSame compares locations).
        var aPlain = new BRepTools_ReShape();
        aPlain.Replace(aA, aB);
        Assert.False(aPlain.IsRecorded(aA.Located(aL)));

        var aReShape = new BRepTools_ReShape();
        aReShape.ModeConsiderLocation(true);
        Assert.True(aReShape.ModeConsiderLocation());
        aReShape.Replace(aA, aB);
        Assert.True(aReShape.IsRecorded(aA.Located(aL)));
        TopoDS_Shape aValue = aReShape.Value(aA.Located(aL));
        Assert.True(aValue.IsPartner(aB));
        Assert.True(aValue.Location().IsEqual(aL));
    }

    [Fact]
    public void ReShape_Merge_HistoryPresentsThePartsAlike()
    {
        TopoDS_Vertex aV1 = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(0, 0, 0)));
        TopoDS_Vertex aV2 = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(1, 0, 0)));
        TopoDS_Vertex aV3 = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(2, 0, 0)));
        TopoDS_Edge aE1 = TestShapes.Frozen(TestShapes.Edge(aV1, aV2));
        TopoDS_Edge aE2 = TestShapes.Frozen(TestShapes.Edge(aV2, aV3));
        TopoDS_Edge aProduct = TestShapes.Frozen(TestShapes.Edge(aV1, aV3));

        var aReShape = new BRepTools_ReShape();
        aReShape.Merge([aE1, aE2], aProduct);
        // the ordinary part is removed by Value, the main one replaced
        Assert.True(aReShape.Value(aE1).IsSame(aProduct));
        Assert.True(aReShape.Value(aE2).IsNull());
        BRepTools_History aHistory = aReShape.History();
        Assert.True(aHistory.Modified(aE1)[0].IsSame(aProduct));
        Assert.True(aHistory.Modified(aE2)[0].IsSame(aProduct));
    }

    [Fact]
    public void ReShape_InternalApply_LeavesTheResultUnfrozen_PublicFreezes()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Vertex aV = TopoDS.Vertex(SubShape(aBox, TopAbs_VERTEX, 1));
        var aReShape = new BRepTools_ReShape();
        aReShape.Replace(aV, TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(-1, 0, 0))));
        TopoDS_Shape anInternal = aReShape.InternalApply(aBox);
        Assert.False(anInternal.TShape()!.IsFrozen());
        // the recorded intermediate results are unfrozen too until handed out
        TopoDS_Shape anEdge = aReShape.InternalValue(SubShape(aBox, TopAbs_EDGE, 1));
        Assert.False(anEdge.TShape()!.IsFrozen());
        Assert.True(aReShape.Value(SubShape(aBox, TopAbs_EDGE, 1)).TShape()!.IsFrozen());
        Assert.True(aReShape.Apply(aBox).TShape()!.IsFrozen());
    }

    [Fact]
    public void ReShape_CopyVertex()
    {
        TopoDS_Vertex aV = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(1, 2, 3)));
        var aReShape = new BRepTools_ReShape();
        TopoDS_Vertex aCopy = aReShape.CopyVertex(aV, 0.5);
        Assert.False(aCopy.IsSame(aV));
        Assert.Equal(0.5, BRep_Tool.Tolerance(aCopy));
        Assert.Equal(1e-7, BRep_Tool.Tolerance(aV));
        // (the public Value would freeze the copy: ported code reads InternalValue)
        Assert.True(aReShape.InternalValue(aV).IsSame(aCopy));
        // the recorded copy is changed in place
        TopoDS_Vertex aSecond = aReShape.CopyVertex(aV, new gp_Pnt(4, 5, 6), 0.25);
        Assert.True(aSecond.IsSame(aCopy));
        Assert.Equal(4.0, BRep_Tool.Pnt(aCopy).X());

        // a recorded frozen replacement can't be changed in place: OCCT's guard for locked shapes
        var aFrozenCase = new BRepTools_ReShape();
        aFrozenCase.Replace(aV, TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(0, 0, 0))));
        Assert.Throws<TopoDS_LockedShape>(() => aFrozenCase.CopyVertex(aV, 0.5));
    }

    [Fact]
    public void Quilt_FacesOfABox_GiveOneClosedShell()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        var aQuilt = new BRepTools_Quilt();
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aQuilt.Add(anExp.Current());
        }

        TopoDS_Shape aShells = aQuilt.Shells();
        aSnapshot.AssertUnchanged(aBox);
        Assert.True(aShells.TShape()!.IsFrozen());
        Assert.Equal(TopAbs_COMPOUND, aShells.ShapeType());
        Assert.Equal(1, aShells.NbChildren());
        TopoDS_Shape aShell = new TopoDS_Iterator(aShells).Value();
        Assert.True(aShell.Closed());
        Assert.True(aShell.Orientable());
        Assert.Equal(6, Count(aShell, TopAbs_FACE));
        Assert.Equal(12, Count(aShell, TopAbs_EDGE));
    }

    [Fact]
    public void Quilt_OpenFaces_GiveOpenShells()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aQuilt = new BRepTools_Quilt();
        // two opposite faces: two shells, each open
        aQuilt.Add(SubShape(aBox, TopAbs_FACE, 1));
        aQuilt.Add(SubShape(aBox, TopAbs_FACE, 2));
        TopoDS_Shape aShells = aQuilt.Shells();
        Assert.Equal(2, aShells.NbChildren());
        for (var anIt = new TopoDS_Iterator(aShells); anIt.More(); anIt.Next())
        {
            Assert.False(anIt.Value().Closed());
        }
    }

    // Two unit squares side by side, each with its own edges; binding the shared side of the second to the first's
    // copies the second face with the first's edge, which receives a pcurve on it.
    private static (TopoDS_Face, TopoDS_Face, TopoDS_Edge, TopoDS_Edge) TwoSquares()
    {
        TopoDS_Vertex[] aV1 = [TestShapes.Vertex(new gp_Pnt(0, 0, 0)), TestShapes.Vertex(new gp_Pnt(1, 0, 0)),
                               TestShapes.Vertex(new gp_Pnt(1, 1, 0)), TestShapes.Vertex(new gp_Pnt(0, 1, 0))];
        TopoDS_Vertex[] aV2 = [TestShapes.Vertex(new gp_Pnt(1, 0, 0)), TestShapes.Vertex(new gp_Pnt(2, 0, 0)),
                               TestShapes.Vertex(new gp_Pnt(2, 1, 0)), TestShapes.Vertex(new gp_Pnt(1, 1, 0))];
        var anEdges1 = new Dictionary<(int, int), TopoDS_Edge>();
        var anEdges2 = new Dictionary<(int, int), TopoDS_Edge>();
        TopoDS_Face aF1 = TestShapes.PolygonFace(aV1, new gp_Dir(0, 0, 1), anEdges1, [0, 1, 2, 3]);
        TopoDS_Face aF2 = TestShapes.PolygonFace(aV2, new gp_Dir(0, 0, 1), anEdges2, [0, 1, 2, 3]);
        // the first square's (1,0)-(1,1) edge and the second's (1,0)-(1,1) edge: same direction
        return (aF1, aF2, anEdges1[(1, 2)], anEdges2[(0, 3)]);
    }

    [Fact]
    public void Quilt_BindEdge_GluesTheFaces()
    {
        (TopoDS_Face aF1, TopoDS_Face aF2, TopoDS_Edge anE1, TopoDS_Edge anE2) = TwoSquares();
        // the second face is a caller's frozen shape; the first face's edge will receive a pcurve, so it can't be
        TestShapes.Frozen(aF2);
        var aSnapshot2 = ShapeSnapshot.Of(aF2);
        var aQuilt = new BRepTools_Quilt();
        aQuilt.Add(aF1);
        aQuilt.Bind(anE2, anE1);
        aQuilt.Add(aF2);

        Assert.True(aQuilt.IsCopied(aF2));
        Assert.False(aQuilt.IsCopied(aF1));
        Assert.True(aQuilt.IsCopied(anE2));
        Assert.True(aQuilt.Copy(anE2).IsSame(anE1));
        TopoDS_Shape aCopy = aQuilt.Copy(aF2);
        Assert.True(aCopy.TShape()!.IsFrozen());
        Assert.True(Contains(aCopy, anE1));
        Assert.NotNull(BRep_Tool.CurveOnSurface(anE1, TopoDS.Face(aCopy), out _, out _));
        aSnapshot2.AssertUnchanged(aF2);

        TopoDS_Shape aShells = aQuilt.Shells();
        Assert.Equal(1, aShells.NbChildren());
        TopoDS_Shape aShell = new TopoDS_Iterator(aShells).Value();
        Assert.Equal(2, Count(aShell, TopAbs_FACE));
        Assert.Equal(7, Count(aShell, TopAbs_EDGE));
        Assert.False(aShell.Closed());
    }

    [Fact]
    public void Quilt_BindToAFrozenEdge_Throws()
    {
        (TopoDS_Face aF1, TopoDS_Face aF2, TopoDS_Edge anE1, TopoDS_Edge anE2) = TwoSquares();
        TestShapes.Frozen(aF1);
        TestShapes.Frozen(aF2);
        var aQuilt = new BRepTools_Quilt();
        aQuilt.Add(aF1);
        aQuilt.Bind(anE2, anE1);
        // OCCT gives the frozen first edge a pcurve on the second face in place
        Assert.Throws<TopoDS_LockedShape>(() => aQuilt.Add(aF2));
    }

    [Fact]
    public void Substitution_ReplaceAnEdgeOfAFrozenFace()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Face aF = TopoDS.Face(SubShape(aBox, TopAbs_FACE, 1));
        var aSnapshot = ShapeSnapshot.Of(aF);
        TopoDS_Edge anE = TopoDS.Edge(SubShape(aF, TopAbs_EDGE, 1));

        // replace the edge by two halves, and remove another edge
        TopoDS_Vertex aV1 = TopExp.FirstVertex(anE), aV2 = TopExp.LastVertex(anE);
        gp_Pnt aMid = BRep_Tool.Pnt(aV1).Translated(new gp_Vec(BRep_Tool.Pnt(aV1), BRep_Tool.Pnt(aV2)) * 0.5);
        TopoDS_Vertex aVM = TestShapes.Frozen(TestShapes.Vertex(aMid));
        var aNew = new NCollection_List<TopoDS_Shape>();
        aNew.Append(TestShapes.Frozen(TestShapes.Edge(aV1, aVM)));
        aNew.Append(TestShapes.Frozen(TestShapes.Edge(aVM, aV2)));
        TopoDS_Shape anOther = SubShape(aF, TopAbs_EDGE, 3);

        var aSubst = new BRepTools_Substitution();
        aSubst.Substitute(anE, aNew);
        aSubst.Substitute(anOther, new NCollection_List<TopoDS_Shape>());
        Assert.Throws<Standard_ConstructionError>(() => aSubst.Substitute(anE, aNew));
        aSubst.Build(aF);

        aSnapshot.AssertUnchanged(aF);
        Assert.True(aSubst.IsCopied(aF));
        Assert.True(aSubst.IsCopied(anE));
        Assert.True(aSubst.IsCopied(anOther));
        Assert.Empty(aSubst.Copy(anOther));
        Assert.Throws<Standard_NoSuchObject>(() => aSubst.Copy(SubShape(aF, TopAbs_EDGE, 2)));
        var aCopies = aSubst.Copy(aF);
        Assert.Single(aCopies);
        TopoDS_Shape aNewF = aCopies[0];
        Assert.True(aNewF.TShape()!.IsFrozen());
        Assert.Equal(TopAbs_FORWARD, aNewF.Orientation());
        Assert.Equal(4, Count(aNewF, TopAbs_EDGE));
        Assert.False(Contains(aNewF, anE));
        Assert.False(Contains(aNewF, anOther));

        aSubst.Clear();
        Assert.False(aSubst.IsCopied(aF));
    }

    [Fact]
    public void Substitution_EmptyContainerDisappears()
    {
        TopoDS_Vertex aV1 = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(0, 0, 0)));
        TopoDS_Vertex aV2 = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(1, 0, 0)));
        TopoDS_Edge anE = TestShapes.Frozen(TestShapes.Edge(aV1, aV2));
        var aB = new BRep_Builder();
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        TestShapes.Frozen(aW);

        var aSubst = new BRepTools_Substitution();
        aSubst.Substitute(anE, new NCollection_List<TopoDS_Shape>());
        aSubst.Build(aW);
        Assert.True(aSubst.IsCopied(aW));
        Assert.Empty(aSubst.Copy(aW));
    }
}
