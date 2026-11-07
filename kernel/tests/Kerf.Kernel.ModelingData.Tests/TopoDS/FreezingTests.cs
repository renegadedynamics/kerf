// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The freezing rules of docs/phase3-design.md, section 4: geometry and meshes freeze when attached (4.1), shape
/// graphs freeze on publication (4.2), frozen implies Locked (4.3), the flags (4.4) and BRep_Builder (4.5).
/// </summary>
public class FreezingTests
{
    private static Edge FirstEdge(in Shape theShape) => ShapeCasts.Edge(new Explorer(theShape, ShapeType.Edge).Current());

    private static Face FirstFace(in Shape theShape) => ShapeCasts.Face(new Explorer(theShape, ShapeType.Face).Current());

    private static Vertex FirstVertex(in Shape theShape) => ShapeCasts.Vertex(new Explorer(theShape, ShapeType.Vertex).Current());

    // 4.1 Geometry, triangulations and polygons freeze when attached

    [Fact]
    public void Builder_freezes_the_geometry_it_attaches()
    {
        var aBuilder = new BrepBuilder();
        var aLine = new LineCurve3D(new Point3D(), new Direction3D(1, 0, 0));
        Assert.False(aLine.IsFrozen);
        Edge anEdge = new();
        aBuilder.MakeEdge(ref anEdge, aLine, 1e-7);
        Assert.True(aLine.IsFrozen);
        // a frozen curve can't change; its copy can
        EngineAssert.Throws<ImmutableObjectException>(() => aLine.SetLocation(new Point3D(1, 1, 1)));
        var aCopy = (LineCurve3D)aLine.Copy();
        Assert.False(aCopy.IsFrozen);
        aCopy.SetLocation(new Point3D(1, 1, 1));

        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        Face aFace = new();
        aBuilder.MakeFace(ref aFace, aPlane, 1e-7);
        Assert.True(aPlane.IsFrozen);

        var aPC = new LineCurve2D(new Point2D(), new Direction2D(1, 0));
        aBuilder.UpdateEdge(anEdge, aPC, aFace, 1e-7);
        Assert.True(aPC.IsFrozen);

        var aSurf2 = new PlaneSurface(new CoordinateSystem(Constants.YOZ()));
        aBuilder.UpdateFace(aFace, aSurf2, new Placement(), 1e-7);
        Assert.True(aSurf2.IsFrozen);
    }

    [Fact]
    public void Representations_freeze_the_geometry_they_hold()
    {
        var aCircle = new CircleCurve3D(Constants.XOY(), 1.0);
        _ = new Curve3DRepresentation(aCircle, new Placement());
        Assert.True(aCircle.IsFrozen);

        var aPC1 = new LineCurve2D(new Point2D(), new Direction2D(0, 1));
        var aPC2 = new LineCurve2D(new Point2D(1, 0), new Direction2D(0, 1));
        var aCyl = new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 1.0);
        _ = new CurveOnClosedSurface(aPC1, aPC2, aCyl, new Placement(), Continuity.C0);
        Assert.True(aPC1.IsFrozen && aPC2.IsFrozen && aCyl.IsFrozen);

        var aS1 = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        var aS2 = new PlaneSurface(new CoordinateSystem(Constants.ZOX()));
        _ = new CurveOn2Surfaces(aS1, aS2, new Placement(), new Placement(), Continuity.G1);
        Assert.True(aS1.IsFrozen && aS2.IsFrozen);

        var aTFace = new BrepTFace();
        var aSurf = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        aTFace.Surface(aSurf);
        Assert.True(aSurf.IsFrozen);
    }

    [Fact]
    public void Poly_mutators_throw_once_attached()
    {
        var aNodes = new NCollection_Array1<Point3D>(1, 2);
        aNodes[1] = new Point3D();
        aNodes[2] = new Point3D(1, 0, 0);
        var aPoly = new Polygon3D(aNodes);
        aPoly.Deflection(0.1); // a builder fills it before it is attached
        var aBuilder = new BrepBuilder();
        Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        aBuilder.UpdateEdge(anEdge, aPoly);
        Assert.True(aPoly.IsFrozen);
        EngineAssert.Throws<ImmutableObjectException>(() => aPoly.Deflection(0.2));
        EngineAssert.Throws<ImmutableObjectException>(() => aPoly.ChangeNodes());
        Assert.Equal(0.1, aPoly.Deflection());

        var aTri = new Triangulation(3, 1, false);
        aTri.SetNode(1, new Point3D());
        aTri.SetNode(2, new Point3D(1, 0, 0));
        aTri.SetNode(3, new Point3D(0, 1, 0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        Face aFace = new();
        aBuilder.MakeFace(ref aFace, aTri);
        EngineAssert.Throws<ImmutableObjectException>(() => aTri.SetNode(1, new Point3D(5, 5, 5)));
        EngineAssert.Throws<ImmutableObjectException>(() => aTri.ComputeNormals());
        EngineAssert.Throws<ImmutableObjectException>(() => aTri.SetMeshPurpose(MeshPurpose.MeshPurpose_Calculation));
        EngineAssert.Throws<ImmutableObjectException>(() => aTri.Clear());
        // a copy can be changed (and so meshing works on copies)
        Triangulation aCopy = aTri.Copy();
        aCopy.ComputeNormals();
        Assert.True(aCopy.HasNormals);
        // the cached range is a cache, consistent with the frozen nodes: it can still be computed
        aTri.UpdateCachedMinMax();
        Assert.True(aTri.HasCachedMinMax);

        var aPolyOnTri = new PolygonOnTriangulation(2, false);
        var aPoly2D = new Polygon2D(2);
        aBuilder.UpdateEdge(anEdge, aPolyOnTri, aTri, new Placement());
        aBuilder.UpdateEdge(anEdge, aPoly2D, aFace);
        EngineAssert.Throws<ImmutableObjectException>(() => aPolyOnTri.SetNode(1, 2));
        EngineAssert.Throws<ImmutableObjectException>(() => aPoly2D.ChangeNodes());
    }

    [Fact]
    public void A_shared_triangulation_keeps_its_purpose()
    {
        // OCCT marks the active triangulation of a face in its purpose; a frozen one (here attached to another face)
        // keeps its purpose (docs/decisions.md, 3106)
        var aBuilder = new BrepBuilder();
        var aTri = new Triangulation(3, 1, false);
        Face aFace1 = new(), aFace2 = new();
        aBuilder.MakeFace(ref aFace1, aTri);
        Assert.Equal(MeshPurpose.MeshPurpose_Active, aTri.MeshPurpose());
        aBuilder.MakeFace(ref aFace2, new Triangulation(3, 1, false));
        aBuilder.UpdateFace(aFace2, aTri, false); // replaces the active triangulation of face 2
        Assert.Same(aTri, Tool.Triangulation(aFace2, out _));
        aBuilder.UpdateFace(aFace1, (Triangulation?)null); // clears face 1's
        Assert.Null(Tool.Triangulation(aFace1, out _));
        Assert.Equal(MeshPurpose.MeshPurpose_Active, aTri.MeshPurpose());
    }

    // 4.2 TShapes and representations freeze on publication

    [Fact]
    public void Built_shapes_are_unfrozen_and_FreezeGraph_freezes_the_whole_graph()
    {
        Solid aBox = TestShapes.Box(1, 2, 3);
        Assert.False(aBox.TShape()!.IsFrozen);
        Assert.False(FirstEdge(aBox).TShape()!.IsFrozen);

        aBox.TShape()!.FreezeGraph();
        foreach (ShapeType aType in new[] { ShapeType.Shell, ShapeType.Face, ShapeType.Wire, ShapeType.Edge, ShapeType.Vertex })
        {
            for (var anExp = new Explorer(aBox, aType); anExp.More(); anExp.Next())
            {
                Assert.True(anExp.Current().TShape()!.IsFrozen);
            }
        }

        // with the representations, and through them the geometry
        Edge anEdge = FirstEdge(aBox);
        foreach (CurveRepresentation aCR in ((BrepTEdge)anEdge.TShape()!).CurveRepresentations())
        {
            Assert.True(aCR.IsFrozen);
        }

        Assert.Equal(3, ((BrepTEdge)anEdge.TShape()!).CurveRepresentations().Count); // a 3D curve and two pcurves
        // a vertex INTERNAL to an edge gets a point representation, frozen with it
        var aBuilder = new BrepBuilder();
        Edge anOther = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        Vertex aV = TestShapes.Vertex(new Point3D(0.5, 0, 0));
        aBuilder.Add(anOther, aV.Oriented(Internal));
        aBuilder.UpdateVertex(ShapeCasts.Vertex(aV.Oriented(Internal)), 0.5, anOther, 1e-7);
        Assert.Single(((BrepTVertex)aV.TShape()!).PointRepresentations());
        Assert.Equal(0.5, Tool.Parameter(ShapeCasts.Vertex(aV.Oriented(Internal)), anOther));
        TestShapes.Frozen(anOther);
        foreach (PointRepresentation aPR in ((BrepTVertex)aV.TShape()!).PointRepresentations())
        {
            Assert.True(aPR.IsFrozen);
        }
    }

    [Fact]
    public void Freezing_a_parent_keeps_frozen_children_and_freezes_new_ones()
    {
        Vertex aFrozen = TestShapes.Frozen(TestShapes.Vertex(new Point3D()));
        Vertex anUnfrozen = TestShapes.Vertex(new Point3D(1, 0, 0));
        Edge anEdge = TestShapes.Edge(aFrozen, anUnfrozen); // a new shape sharing a frozen input
        Assert.False(anEdge.TShape()!.IsFrozen);
        TestShapes.Frozen(anEdge);
        Assert.True(anUnfrozen.TShape()!.IsFrozen);
    }

    [Fact]
    public void MakeCompound_returns_a_frozen_compound()
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Shape anEdge = TestShapes.Frozen(TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0)));
        Compound aCompound = Builder.MakeCompound(aBox, anEdge, aBox.Moved(new Placement(Transform3D.Translation(new Vector3D(5, 0, 0)))));
        Assert.True(aCompound.TShape()!.IsFrozen);
        Assert.True(aCompound.Locked());
        Assert.Equal(3, aCompound.ChildCount);
        var anIt = new Iterator(aCompound);
        Assert.True(anIt.Value() == aBox);
        // the children's Free flags don't change (OCCT clears them; docs/phase3-design.md, section 4.4)
        Assert.True(aBox.Free() && anEdge.Free());

        // from an enumerable, and unfrozen internal shapes are frozen with it
        Vertex aV = TestShapes.Vertex(new Point3D());
        Compound aSecond = Builder.MakeCompound(new List<Shape> { aV });
        Assert.True(aV.TShape()!.IsFrozen);
        Assert.True(aSecond.TShape()!.IsFrozen);

        Assert.Equal(0, Builder.MakeCompound().ChildCount);
        EngineAssert.Throws<NullObjectException>(() => Builder.MakeCompound(new Shape()));
    }

    [Fact]
    public void MakeWire_MakeShell_MakeSolid_and_MakeCompSolid_return_frozen_shapes()
    {
        // decision 1104: OCCT's MakeX then Add, for callers, as MakeCompound
        Edge anE1 = TestShapes.Frozen(TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0)));
        Edge anE2 = TestShapes.Frozen(TestShapes.Edge(new Point3D(1, 0, 0), new Point3D(1, 1, 0)));
        Wire aWire = Builder.MakeWire([anE1, anE2]);
        Assert.True(aWire.TShape()!.IsFrozen);
        Assert.Equal(2, aWire.ChildCount);
        Assert.False(aWire.ClosedFlag);
        Assert.True(new Iterator(aWire).Value() == anE1);

        Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aFaces = new List<Shape>();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces.Add(anExp.Current());
        }
        Shell aShell = Builder.MakeShell(aFaces);
        Assert.True(aShell.TShape()!.IsFrozen);
        Assert.Equal(6, aShell.ChildCount);
        Assert.False(aShell.ClosedFlag); // as OCCT's builder leaves it
        Solid aSolid = Builder.MakeSolid([aShell]);
        Assert.True(aSolid.TShape()!.IsFrozen);
        Assert.True(new Iterator(aSolid).Value() == aShell);
        CompSolid aCS = Builder.MakeCompSolid([aSolid, aSolid.Moved(new Placement(Transform3D.Translation(new Vector3D(1, 0, 0))))]);
        Assert.True(aCS.TShape()!.IsFrozen);
        Assert.Equal(2, aCS.ChildCount);

        // OCCT's rules: only faces into a shell, solids into a compsolid; no null shape
        EngineAssert.Throws<IncompatibleShapesException>(() => Builder.MakeShell([anE1]));
        EngineAssert.Throws<IncompatibleShapesException>(() => Builder.MakeCompSolid([aShell]));
        EngineAssert.Throws<IncompatibleShapesException>(() => Builder.MakeWire([aShell]));
        EngineAssert.Throws<NullObjectException>(() => Builder.MakeSolid([new Shape()]));
        Assert.Equal(0, Builder.MakeShell([]).ChildCount);
    }

    [Fact]
    public void A_history_copy_has_the_same_maps_and_is_independent()
    {
        // decision 1104: OCCT's implicit copy constructor
        Edge anE1 = TestShapes.Frozen(TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0)));
        Edge anE2 = TestShapes.Frozen(TestShapes.Edge(new Point3D(1, 0, 0), new Point3D(1, 1, 0)));
        Edge anE3 = TestShapes.Frozen(TestShapes.Edge(new Point3D(1, 1, 0), new Point3D(0, 1, 0)));
        var aH = new History();
        aH.AddGenerated(anE1, anE3);
        aH.Remove(anE1); // a shape removed that also generated: Merge into an empty history drops the removal
        aH.AddModified(anE2, anE1);
        var aCopy = new History(aH);
        Assert.True(aCopy.IsRemoved(anE1));
        Assert.True(aCopy.Generated(anE1).Single().IsSame(anE3));
        Assert.True(aCopy.Modified(anE2).Single().IsSame(anE1));
        aH.AddModified(anE2, anE3);
        Assert.Single(aCopy.Modified(anE2));
        aH.Clear();
        Assert.True(aCopy.HasModified && aCopy.HasGenerated && aCopy.HasRemoved);
    }

    [Fact]
    public void FreezeGraph_survives_deep_nesting_and_cycles()
    {
        // built from the inside out, as OCCT requires (a shape is filled before it is added)
        var aBuilder = new Builder();
        Compound aLeaf = new();
        aBuilder.MakeCompound(ref aLeaf);
        Compound aRoot = aLeaf;
        for (int i = 0; i < Builder.MaxCompoundNesting; i++)
        {
            Compound aParent = new();
            aBuilder.MakeCompound(ref aParent);
            aBuilder.Add(aParent, aRoot);
            aRoot = aParent;
        }

        // one more would overflow a stack in the walks of a shape (decision 1854)
        Compound aTooDeep = new();
        aBuilder.MakeCompound(ref aTooDeep);
        Compound aRootForTheCheck = aRoot;
        EngineAssert.Throws<ConstructionErrorException>(() => new Builder().Add(aTooDeep, aRootForTheCheck));

        aRoot.TShape()!.FreezeGraph();
        Assert.True(aLeaf.TShape()!.IsFrozen);

        // OCCT's Free flag prevents cycles, but Free(true) can make one
        Compound aA = new(), aB = new();
        aBuilder.MakeCompound(ref aA);
        aBuilder.MakeCompound(ref aB);
        aBuilder.Add(aA, aB);
        aB.Free(true);
        aBuilder.Add(aB, aA);
        aA.TShape()!.FreezeGraph();
        Assert.True(aA.TShape()!.IsFrozen && aB.TShape()!.IsFrozen);
    }

    // 4.3 Frozen implies Locked

    [Fact]
    public void Frozen_shapes_are_locked_and_the_builder_refuses_them()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Edge anEdge = FirstEdge(aBox);
        Face aFace = FirstFace(aBox);
        Vertex aVertex = FirstVertex(aBox);
        Assert.True(anEdge.Locked());
        Assert.False(anEdge.TShape()!.LockedFlag()); // files carry the stored bit

        var aBuilder = new BrepBuilder();
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.UpdateEdge(anEdge, 0.1));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.UpdateEdge(anEdge, new LineCurve3D(Constants.OX()), 0.1));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.UpdateEdge(anEdge, new LineCurve2D(Constants.OX2d()), aFace, 0.1));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.Range(anEdge, 0, 1));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.SameParameter(anEdge, false));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.Degenerated(anEdge, true));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.Continuity(anEdge, aFace, aFace, Continuity.C1));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.UpdateVertex(aVertex, new Point3D(), 0.1));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.UpdateVertex(aVertex, 0.1));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.UpdateVertex(aVertex, 0.0, anEdge, 0.1));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.UpdateFace(aFace, 0.1));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.NaturalRestriction(aFace, true));
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.UpdateFace(aFace, (Triangulation?)null));
        // the Make methods refuse to replace a variable holding a locked shape, as in OCCT
        Edge aVariable = anEdge;
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.MakeEdge(ref aVariable));
        Face aFaceVariable = aFace;
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.MakeFace(ref aFaceVariable, new PlaneSurface(new CoordinateSystem(Constants.XOY())), 1e-7));
    }

    [Fact]
    public void Paths_around_the_builder_throw_ImmutableObject()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Edge anEdge = FirstEdge(aBox);
        var aTE = (BrepTEdge)anEdge.TShape()!;
        var aTV = (BrepTVertex)FirstVertex(aBox).TShape()!;
        var aTF = (BrepTFace)FirstFace(aBox).TShape()!;

        EngineAssert.Throws<ImmutableObjectException>(() => aTE.ChangeCurves());
        EngineAssert.Throws<ImmutableObjectException>(() => aTE.Tolerance(1.0));
        EngineAssert.Throws<ImmutableObjectException>(() => aTE.UpdateTolerance(1.0));
        EngineAssert.Throws<ImmutableObjectException>(() => aTE.SameRange(false));
        EngineAssert.Throws<ImmutableObjectException>(() => aTV.ChangePoints());
        EngineAssert.Throws<ImmutableObjectException>(() => aTV.Pnt(new Point3D(1, 1, 1)));
        EngineAssert.Throws<ImmutableObjectException>(() => aTF.Surface(new PlaneSurface(new CoordinateSystem(Constants.XOY()))));
        EngineAssert.Throws<ImmutableObjectException>(() => aTF.Triangulation(null));
        EngineAssert.Throws<ImmutableObjectException>(() => aTF.Location(new Placement()));

        var aCR = (GCurve)aTE.Curves().First();
        EngineAssert.Throws<ImmutableObjectException>(() => aCR.SetRange(0, 2));
        EngineAssert.Throws<ImmutableObjectException>(() => aCR.Location(new Placement()));
        CurveRepresentation aPCurveRep = aTE.CurveRepresentations().First(r => r.IsCurveOnSurface());
        EngineAssert.Throws<ImmutableObjectException>(() => aPCurveRep.PCurve(new LineCurve2D(Constants.OX2d())));
        EngineAssert.Throws<ImmutableObjectException>(() => Tool.SetUVPoints(anEdge, FirstFace(aBox), new Point2D(), new Point2D()));
        // a copy of a representation is unfrozen and shares the geometry
        CurveRepresentation aCopy = aCR.Copy();
        Assert.False(aCopy.IsFrozen);
        Assert.Same(aCR.Curve3D(), aCopy.Curve3D());
    }

    [Fact]
    public void The_child_list_of_a_frozen_shape_cant_change()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aBuilder = new Builder();
        // the solid is free (it was never added to anything), so OCCT's Free check passes and the frozen check throws
        Assert.True(aBox.Free());
        Shell aShell = new();
        aBuilder.MakeShell(ref aShell);
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.Add(aBox, aShell));
        Shape anOldShell = new Iterator(aBox).Value();
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.Remove(aBox, anOldShell));
        Assert.Equal(1, aBox.ChildCount);
        // a shape inside it isn't free: OCCT's own check throws first
        Face aFace = FirstFace(aBox);
        EngineAssert.Throws<DomainErrorException>(() => aBuilder.Remove(aFace, new Iterator(aFace).Value()));
    }

    [Fact]
    public void UpdateFaceUVPoints_stops_at_frozen_edges()
    {
        // a new face built on frozen edges (shared with a published shape) isn't frozen; its edges are
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Face aFace = FirstFace(aBox);
        var aBuilder = new BrepBuilder();
        Face aNewFace = ShapeCasts.Face(aFace.EmptyCopied());
        aBuilder.Add(aNewFace, new Iterator(aFace).Value());
        Assert.False(aNewFace.TShape()!.IsFrozen);
        BrepTools.UpdateFaceUVPoints(aNewFace); // OCCT returns at the first locked edge
        BrepTools.Update(aNewFace);
        Assert.True(aNewFace.Checked());
    }

    // 4.4 The flags

    [Fact]
    public void Free_flag_of_frozen_children_is_kept()
    {
        var aBuilder = new Builder();
        Vertex aFrozen = TestShapes.Frozen(TestShapes.Vertex(new Point3D()));
        Vertex anUnfrozen = TestShapes.Vertex(new Point3D());
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aFrozen);
        aBuilder.Add(aCompound, anUnfrozen);
        Assert.True(aFrozen.Free()); // Kerf: frozen children keep their flag
        Assert.False(anUnfrozen.Free()); // OCCT clears it

        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Free(false));
        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Free(true));
        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Locked(false));
        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Locked(true));
        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Closed(false));
        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Orientable(false));
        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Infinite(true));
        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Convex(false));
        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Modified(true));
        EngineAssert.Throws<ImmutableObjectException>(() => aFrozen.Checked(true));
    }

    [Fact]
    public void Cache_completions_return_at_once_on_frozen_shapes()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Face aFace = FirstFace(aBox);
        Assert.False(aFace.Checked());
        BrepTools.Update(aFace);
        BrepTools.Update(aBox);
        Assert.False(aFace.Checked());

        // an unfrozen face is updated and marked
        Solid anUnfrozen = TestShapes.Box(1, 1, 1);
        BrepTools.Update(anUnfrozen);
        Assert.True(FirstFace(anUnfrozen).Checked());
    }

    // 4.5 BRep_Builder

    [Fact]
    public void A_frozen_child_can_be_added_to_an_unfrozen_parent()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aBuilder = new BrepBuilder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBox);
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        aBuilder.Add(aWire, FirstEdge(aBox)); // a frozen edge in a new wire
        aBuilder.Add(aCompound, aWire);
        Assert.Equal(2, aCompound.ChildCount);

        // and a new edge on frozen vertices can be given parameters: the vertices' point representations would
        // change, so this needs a copy (here it throws)
        Exploration.Vertices(FirstEdge(aBox), out Vertex aV1, out Vertex aV2);
        var aLine = new LineCurve3D(Tool.Pnt(aV1), new Direction3D(new Vector3D(Tool.Pnt(aV1), Tool.Pnt(aV2))));
        Edge aNewEdge = TestShapes.Edge(aLine, 0, 1, aV1, aV2);
        EngineAssert.Throws<ImmutableObjectException>(() => aBuilder.UpdateVertex(aV1, 0.5, aNewEdge, 1e-7));
    }
}
