// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// The freezing rules of docs/phase3-design.md, section 4: geometry and meshes freeze when attached (4.1), shape
/// graphs freeze on publication (4.2), frozen implies Locked (4.3), the flags (4.4) and BRep_Builder (4.5).
/// </summary>
public class FreezingTests
{
    private static TopoDS_Edge FirstEdge(in TopoDS_Shape theShape) => TopoDS.Edge(new TopExp_Explorer(theShape, TopAbs_EDGE).Current());

    private static TopoDS_Face FirstFace(in TopoDS_Shape theShape) => TopoDS.Face(new TopExp_Explorer(theShape, TopAbs_FACE).Current());

    private static TopoDS_Vertex FirstVertex(in TopoDS_Shape theShape) => TopoDS.Vertex(new TopExp_Explorer(theShape, TopAbs_VERTEX).Current());

    // 4.1 Geometry, triangulations and polygons freeze when attached

    [Fact]
    public void Builder_freezes_the_geometry_it_attaches()
    {
        var aBuilder = new BRep_Builder();
        var aLine = new Geom_Line(new gp_Pnt(), new gp_Dir(1, 0, 0));
        Assert.False(aLine.IsFrozen());
        TopoDS_Edge anEdge = new();
        aBuilder.MakeEdge(ref anEdge, aLine, 1e-7);
        Assert.True(aLine.IsFrozen());
        // a frozen curve can't change; its copy can
        Assert.Throws<Standard_ImmutableObject>(() => aLine.SetLocation(new gp_Pnt(1, 1, 1)));
        var aCopy = (Geom_Line)aLine.Copy();
        Assert.False(aCopy.IsFrozen());
        aCopy.SetLocation(new gp_Pnt(1, 1, 1));

        var aPlane = new Geom_Plane(new gp_Ax3(gp.XOY()));
        TopoDS_Face aFace = new();
        aBuilder.MakeFace(ref aFace, aPlane, 1e-7);
        Assert.True(aPlane.IsFrozen());

        var aPC = new Geom2d_Line(new gp_Pnt2d(), new gp_Dir2d(1, 0));
        aBuilder.UpdateEdge(anEdge, aPC, aFace, 1e-7);
        Assert.True(aPC.IsFrozen());

        var aSurf2 = new Geom_Plane(new gp_Ax3(gp.YOZ()));
        aBuilder.UpdateFace(aFace, aSurf2, new TopLoc_Location(), 1e-7);
        Assert.True(aSurf2.IsFrozen());
    }

    [Fact]
    public void Representations_freeze_the_geometry_they_hold()
    {
        var aCircle = new Geom_Circle(gp.XOY(), 1.0);
        _ = new BRep_Curve3D(aCircle, new TopLoc_Location());
        Assert.True(aCircle.IsFrozen());

        var aPC1 = new Geom2d_Line(new gp_Pnt2d(), new gp_Dir2d(0, 1));
        var aPC2 = new Geom2d_Line(new gp_Pnt2d(1, 0), new gp_Dir2d(0, 1));
        var aCyl = new Geom_CylindricalSurface(new gp_Ax3(gp.XOY()), 1.0);
        _ = new BRep_CurveOnClosedSurface(aPC1, aPC2, aCyl, new TopLoc_Location(), GeomAbs_Shape.GeomAbs_C0);
        Assert.True(aPC1.IsFrozen() && aPC2.IsFrozen() && aCyl.IsFrozen());

        var aS1 = new Geom_Plane(new gp_Ax3(gp.XOY()));
        var aS2 = new Geom_Plane(new gp_Ax3(gp.ZOX()));
        _ = new BRep_CurveOn2Surfaces(aS1, aS2, new TopLoc_Location(), new TopLoc_Location(), GeomAbs_Shape.GeomAbs_G1);
        Assert.True(aS1.IsFrozen() && aS2.IsFrozen());

        var aTFace = new BRep_TFace();
        var aSurf = new Geom_Plane(new gp_Ax3(gp.XOY()));
        aTFace.Surface(aSurf);
        Assert.True(aSurf.IsFrozen());
    }

    [Fact]
    public void Poly_mutators_throw_once_attached()
    {
        var aNodes = new NCollection_Array1<gp_Pnt>(1, 2);
        aNodes[1] = new gp_Pnt();
        aNodes[2] = new gp_Pnt(1, 0, 0);
        var aPoly = new Poly_Polygon3D(aNodes);
        aPoly.Deflection(0.1); // a builder fills it before it is attached
        var aBuilder = new BRep_Builder();
        TopoDS_Edge anEdge = TestShapes.Edge(new gp_Pnt(), new gp_Pnt(1, 0, 0));
        aBuilder.UpdateEdge(anEdge, aPoly);
        Assert.True(aPoly.IsFrozen());
        Assert.Throws<Standard_ImmutableObject>(() => aPoly.Deflection(0.2));
        Assert.Throws<Standard_ImmutableObject>(() => aPoly.ChangeNodes());
        Assert.Equal(0.1, aPoly.Deflection());

        var aTri = new Poly_Triangulation(3, 1, false);
        aTri.SetNode(1, new gp_Pnt());
        aTri.SetNode(2, new gp_Pnt(1, 0, 0));
        aTri.SetNode(3, new gp_Pnt(0, 1, 0));
        aTri.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        TopoDS_Face aFace = new();
        aBuilder.MakeFace(ref aFace, aTri);
        Assert.Throws<Standard_ImmutableObject>(() => aTri.SetNode(1, new gp_Pnt(5, 5, 5)));
        Assert.Throws<Standard_ImmutableObject>(() => aTri.ComputeNormals());
        Assert.Throws<Standard_ImmutableObject>(() => aTri.SetMeshPurpose(Poly_MeshPurpose.Poly_MeshPurpose_Calculation));
        Assert.Throws<Standard_ImmutableObject>(() => aTri.Clear());
        // a copy can be changed (and so meshing works on copies)
        Poly_Triangulation aCopy = aTri.Copy();
        aCopy.ComputeNormals();
        Assert.True(aCopy.HasNormals());
        // the cached range is a cache, consistent with the frozen nodes: it can still be computed
        aTri.UpdateCachedMinMax();
        Assert.True(aTri.HasCachedMinMax());

        var aPolyOnTri = new Poly_PolygonOnTriangulation(2, false);
        var aPoly2D = new Poly_Polygon2D(2);
        aBuilder.UpdateEdge(anEdge, aPolyOnTri, aTri, new TopLoc_Location());
        aBuilder.UpdateEdge(anEdge, aPoly2D, aFace);
        Assert.Throws<Standard_ImmutableObject>(() => aPolyOnTri.SetNode(1, 2));
        Assert.Throws<Standard_ImmutableObject>(() => aPoly2D.ChangeNodes());
    }

    [Fact]
    public void A_shared_triangulation_keeps_its_purpose()
    {
        // OCCT marks the active triangulation of a face in its purpose; a frozen one (here attached to another face)
        // keeps its purpose (docs/decisions.md, 3106)
        var aBuilder = new BRep_Builder();
        var aTri = new Poly_Triangulation(3, 1, false);
        TopoDS_Face aFace1 = new(), aFace2 = new();
        aBuilder.MakeFace(ref aFace1, aTri);
        Assert.Equal(Poly_MeshPurpose.Poly_MeshPurpose_Active, aTri.MeshPurpose());
        aBuilder.MakeFace(ref aFace2, new Poly_Triangulation(3, 1, false));
        aBuilder.UpdateFace(aFace2, aTri, false); // replaces the active triangulation of face 2
        Assert.Same(aTri, BRep_Tool.Triangulation(aFace2, out _));
        aBuilder.UpdateFace(aFace1, (Poly_Triangulation?)null); // clears face 1's
        Assert.Null(BRep_Tool.Triangulation(aFace1, out _));
        Assert.Equal(Poly_MeshPurpose.Poly_MeshPurpose_Active, aTri.MeshPurpose());
    }

    // 4.2 TShapes and representations freeze on publication

    [Fact]
    public void Built_shapes_are_unfrozen_and_FreezeGraph_freezes_the_whole_graph()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 2, 3);
        Assert.False(aBox.TShape()!.IsFrozen());
        Assert.False(FirstEdge(aBox).TShape()!.IsFrozen());

        aBox.TShape()!.FreezeGraph();
        foreach (TopAbs_ShapeEnum aType in new[] { TopAbs_SHELL, TopAbs_FACE, TopAbs_WIRE, TopAbs_EDGE, TopAbs_VERTEX })
        {
            for (var anExp = new TopExp_Explorer(aBox, aType); anExp.More(); anExp.Next())
            {
                Assert.True(anExp.Current().TShape()!.IsFrozen());
            }
        }

        // with the representations, and through them the geometry
        TopoDS_Edge anEdge = FirstEdge(aBox);
        foreach (BRep_CurveRepresentation aCR in ((BRep_TEdge)anEdge.TShape()!).CurveRepresentations())
        {
            Assert.True(aCR.IsFrozen());
        }

        Assert.Equal(3, ((BRep_TEdge)anEdge.TShape()!).CurveRepresentations().Count); // a 3D curve and two pcurves
        // a vertex INTERNAL to an edge gets a point representation, frozen with it
        var aBuilder = new BRep_Builder();
        TopoDS_Edge anOther = TestShapes.Edge(new gp_Pnt(), new gp_Pnt(1, 0, 0));
        TopoDS_Vertex aV = TestShapes.Vertex(new gp_Pnt(0.5, 0, 0));
        aBuilder.Add(anOther, aV.Oriented(TopAbs_INTERNAL));
        aBuilder.UpdateVertex(TopoDS.Vertex(aV.Oriented(TopAbs_INTERNAL)), 0.5, anOther, 1e-7);
        Assert.Single(((BRep_TVertex)aV.TShape()!).PointRepresentations());
        Assert.Equal(0.5, BRep_Tool.Parameter(TopoDS.Vertex(aV.Oriented(TopAbs_INTERNAL)), anOther));
        TestShapes.Frozen(anOther);
        foreach (BRep_PointRepresentation aPR in ((BRep_TVertex)aV.TShape()!).PointRepresentations())
        {
            Assert.True(aPR.IsFrozen());
        }
    }

    [Fact]
    public void Freezing_a_parent_keeps_frozen_children_and_freezes_new_ones()
    {
        TopoDS_Vertex aFrozen = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt()));
        TopoDS_Vertex anUnfrozen = TestShapes.Vertex(new gp_Pnt(1, 0, 0));
        TopoDS_Edge anEdge = TestShapes.Edge(aFrozen, anUnfrozen); // a new shape sharing a frozen input
        Assert.False(anEdge.TShape()!.IsFrozen());
        TestShapes.Frozen(anEdge);
        Assert.True(anUnfrozen.TShape()!.IsFrozen());
    }

    [Fact]
    public void MakeCompound_returns_a_frozen_compound()
    {
        TopoDS_Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Shape anEdge = TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(), new gp_Pnt(1, 0, 0)));
        TopoDS_Compound aCompound = TopoDS_Builder.MakeCompound(aBox, anEdge, aBox.Moved(new TopLoc_Location(gp_Trsf.Translation(new gp_Vec(5, 0, 0)))));
        Assert.True(aCompound.TShape()!.IsFrozen());
        Assert.True(aCompound.Locked());
        Assert.Equal(3, aCompound.NbChildren());
        var anIt = new TopoDS_Iterator(aCompound);
        Assert.True(anIt.Value() == aBox);
        // the children's Free flags don't change (OCCT clears them; docs/phase3-design.md, section 4.4)
        Assert.True(aBox.Free() && anEdge.Free());

        // from an enumerable, and unfrozen internal shapes are frozen with it
        TopoDS_Vertex aV = TestShapes.Vertex(new gp_Pnt());
        TopoDS_Compound aSecond = TopoDS_Builder.MakeCompound(new List<TopoDS_Shape> { aV });
        Assert.True(aV.TShape()!.IsFrozen());
        Assert.True(aSecond.TShape()!.IsFrozen());

        Assert.Equal(0, TopoDS_Builder.MakeCompound().NbChildren());
        Assert.Throws<Standard_NullObject>(() => TopoDS_Builder.MakeCompound(new TopoDS_Shape()));
    }

    [Fact]
    public void MakeWire_MakeShell_MakeSolid_and_MakeCompSolid_return_frozen_shapes()
    {
        // decision 1104: OCCT's MakeX then Add, for callers, as MakeCompound
        TopoDS_Edge anE1 = TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(), new gp_Pnt(1, 0, 0)));
        TopoDS_Edge anE2 = TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0)));
        TopoDS_Wire aWire = TopoDS_Builder.MakeWire([anE1, anE2]);
        Assert.True(aWire.TShape()!.IsFrozen());
        Assert.Equal(2, aWire.NbChildren());
        Assert.False(aWire.Closed());
        Assert.True(new TopoDS_Iterator(aWire).Value() == anE1);

        TopoDS_Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aFaces = new List<TopoDS_Shape>();
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aFaces.Add(anExp.Current());
        }
        TopoDS_Shell aShell = TopoDS_Builder.MakeShell(aFaces);
        Assert.True(aShell.TShape()!.IsFrozen());
        Assert.Equal(6, aShell.NbChildren());
        Assert.False(aShell.Closed()); // as OCCT's builder leaves it
        TopoDS_Solid aSolid = TopoDS_Builder.MakeSolid([aShell]);
        Assert.True(aSolid.TShape()!.IsFrozen());
        Assert.True(new TopoDS_Iterator(aSolid).Value() == aShell);
        TopoDS_CompSolid aCS = TopoDS_Builder.MakeCompSolid([aSolid, aSolid.Moved(new TopLoc_Location(gp_Trsf.Translation(new gp_Vec(1, 0, 0))))]);
        Assert.True(aCS.TShape()!.IsFrozen());
        Assert.Equal(2, aCS.NbChildren());

        // OCCT's rules: only faces into a shell, solids into a compsolid; no null shape
        Assert.Throws<TopoDS_UnCompatibleShapes>(() => TopoDS_Builder.MakeShell([anE1]));
        Assert.Throws<TopoDS_UnCompatibleShapes>(() => TopoDS_Builder.MakeCompSolid([aShell]));
        Assert.Throws<TopoDS_UnCompatibleShapes>(() => TopoDS_Builder.MakeWire([aShell]));
        Assert.Throws<Standard_NullObject>(() => TopoDS_Builder.MakeSolid([new TopoDS_Shape()]));
        Assert.Equal(0, TopoDS_Builder.MakeShell([]).NbChildren());
    }

    [Fact]
    public void A_history_copy_has_the_same_maps_and_is_independent()
    {
        // decision 1104: OCCT's implicit copy constructor
        TopoDS_Edge anE1 = TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(), new gp_Pnt(1, 0, 0)));
        TopoDS_Edge anE2 = TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0)));
        TopoDS_Edge anE3 = TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0)));
        var aH = new BRepTools_History();
        aH.AddGenerated(anE1, anE3);
        aH.Remove(anE1); // a shape removed that also generated: Merge into an empty history drops the removal
        aH.AddModified(anE2, anE1);
        var aCopy = new BRepTools_History(aH);
        Assert.True(aCopy.IsRemoved(anE1));
        Assert.True(aCopy.Generated(anE1).Single().IsSame(anE3));
        Assert.True(aCopy.Modified(anE2).Single().IsSame(anE1));
        aH.AddModified(anE2, anE3);
        Assert.Single(aCopy.Modified(anE2));
        aH.Clear();
        Assert.True(aCopy.HasModified() && aCopy.HasGenerated() && aCopy.HasRemoved());
    }

    [Fact]
    public void FreezeGraph_survives_deep_nesting_and_cycles()
    {
        // built from the inside out, as OCCT requires (a shape is filled before it is added)
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aLeaf = new();
        aBuilder.MakeCompound(ref aLeaf);
        TopoDS_Compound aRoot = aLeaf;
        for (int i = 0; i < 100_000; i++)
        {
            TopoDS_Compound aParent = new();
            aBuilder.MakeCompound(ref aParent);
            aBuilder.Add(aParent, aRoot);
            aRoot = aParent;
        }

        aRoot.TShape()!.FreezeGraph();
        Assert.True(aLeaf.TShape()!.IsFrozen());

        // OCCT's Free flag prevents cycles, but Free(true) can make one
        TopoDS_Compound aA = new(), aB = new();
        aBuilder.MakeCompound(ref aA);
        aBuilder.MakeCompound(ref aB);
        aBuilder.Add(aA, aB);
        aB.Free(true);
        aBuilder.Add(aB, aA);
        aA.TShape()!.FreezeGraph();
        Assert.True(aA.TShape()!.IsFrozen() && aB.TShape()!.IsFrozen());
    }

    // 4.3 Frozen implies Locked

    [Fact]
    public void Frozen_shapes_are_locked_and_the_builder_refuses_them()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Edge anEdge = FirstEdge(aBox);
        TopoDS_Face aFace = FirstFace(aBox);
        TopoDS_Vertex aVertex = FirstVertex(aBox);
        Assert.True(anEdge.Locked());
        Assert.False(anEdge.TShape()!.LockedFlag()); // files carry the stored bit

        var aBuilder = new BRep_Builder();
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.UpdateEdge(anEdge, 0.1));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.UpdateEdge(anEdge, new Geom_Line(gp.OX()), 0.1));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.UpdateEdge(anEdge, new Geom2d_Line(gp.OX2d()), aFace, 0.1));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.Range(anEdge, 0, 1));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.SameParameter(anEdge, false));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.Degenerated(anEdge, true));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.Continuity(anEdge, aFace, aFace, GeomAbs_Shape.GeomAbs_C1));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.UpdateVertex(aVertex, new gp_Pnt(), 0.1));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.UpdateVertex(aVertex, 0.1));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.UpdateVertex(aVertex, 0.0, anEdge, 0.1));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.UpdateFace(aFace, 0.1));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.NaturalRestriction(aFace, true));
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.UpdateFace(aFace, (Poly_Triangulation?)null));
        // the Make methods refuse to replace a variable holding a locked shape, as in OCCT
        TopoDS_Edge aVariable = anEdge;
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.MakeEdge(ref aVariable));
        TopoDS_Face aFaceVariable = aFace;
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.MakeFace(ref aFaceVariable, new Geom_Plane(new gp_Ax3(gp.XOY())), 1e-7));
    }

    [Fact]
    public void Paths_around_the_builder_throw_ImmutableObject()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Edge anEdge = FirstEdge(aBox);
        var aTE = (BRep_TEdge)anEdge.TShape()!;
        var aTV = (BRep_TVertex)FirstVertex(aBox).TShape()!;
        var aTF = (BRep_TFace)FirstFace(aBox).TShape()!;

        Assert.Throws<Standard_ImmutableObject>(() => aTE.ChangeCurves());
        Assert.Throws<Standard_ImmutableObject>(() => aTE.Tolerance(1.0));
        Assert.Throws<Standard_ImmutableObject>(() => aTE.UpdateTolerance(1.0));
        Assert.Throws<Standard_ImmutableObject>(() => aTE.SameRange(false));
        Assert.Throws<Standard_ImmutableObject>(() => aTV.ChangePoints());
        Assert.Throws<Standard_ImmutableObject>(() => aTV.Pnt(new gp_Pnt(1, 1, 1)));
        Assert.Throws<Standard_ImmutableObject>(() => aTF.Surface(new Geom_Plane(new gp_Ax3(gp.XOY()))));
        Assert.Throws<Standard_ImmutableObject>(() => aTF.Triangulation(null));
        Assert.Throws<Standard_ImmutableObject>(() => aTF.Location(new TopLoc_Location()));

        var aCR = (BRep_GCurve)aTE.Curves().First();
        Assert.Throws<Standard_ImmutableObject>(() => aCR.SetRange(0, 2));
        Assert.Throws<Standard_ImmutableObject>(() => aCR.Location(new TopLoc_Location()));
        BRep_CurveRepresentation aPCurveRep = aTE.CurveRepresentations().First(r => r.IsCurveOnSurface());
        Assert.Throws<Standard_ImmutableObject>(() => aPCurveRep.PCurve(new Geom2d_Line(gp.OX2d())));
        Assert.Throws<Standard_ImmutableObject>(() => BRep_Tool.SetUVPoints(anEdge, FirstFace(aBox), new gp_Pnt2d(), new gp_Pnt2d()));
        // a copy of a representation is unfrozen and shares the geometry
        BRep_CurveRepresentation aCopy = aCR.Copy();
        Assert.False(aCopy.IsFrozen());
        Assert.Same(aCR.Curve3D(), aCopy.Curve3D());
    }

    [Fact]
    public void The_child_list_of_a_frozen_shape_cant_change()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aBuilder = new TopoDS_Builder();
        // the solid is free (it was never added to anything), so OCCT's Free check passes and the frozen check throws
        Assert.True(aBox.Free());
        TopoDS_Shell aShell = new();
        aBuilder.MakeShell(ref aShell);
        Assert.Throws<Standard_ImmutableObject>(() => aBuilder.Add(aBox, aShell));
        TopoDS_Shape anOldShell = new TopoDS_Iterator(aBox).Value();
        Assert.Throws<Standard_ImmutableObject>(() => aBuilder.Remove(aBox, anOldShell));
        Assert.Equal(1, aBox.NbChildren());
        // a shape inside it isn't free: OCCT's own check throws first
        TopoDS_Face aFace = FirstFace(aBox);
        Assert.Throws<TopoDS_FrozenShape>(() => aBuilder.Remove(aFace, new TopoDS_Iterator(aFace).Value()));
    }

    [Fact]
    public void UpdateFaceUVPoints_stops_at_frozen_edges()
    {
        // a new face built on frozen edges (shared with a published shape) isn't frozen; its edges are
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Face aFace = FirstFace(aBox);
        var aBuilder = new BRep_Builder();
        TopoDS_Face aNewFace = TopoDS.Face(aFace.EmptyCopied());
        aBuilder.Add(aNewFace, new TopoDS_Iterator(aFace).Value());
        Assert.False(aNewFace.TShape()!.IsFrozen());
        BRepTools.UpdateFaceUVPoints(aNewFace); // OCCT returns at the first locked edge
        BRepTools.Update(aNewFace);
        Assert.True(aNewFace.Checked());
    }

    // 4.4 The flags

    [Fact]
    public void Free_flag_of_frozen_children_is_kept()
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Vertex aFrozen = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt()));
        TopoDS_Vertex anUnfrozen = TestShapes.Vertex(new gp_Pnt());
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aFrozen);
        aBuilder.Add(aCompound, anUnfrozen);
        Assert.True(aFrozen.Free()); // OCCT.NET: frozen children keep their flag
        Assert.False(anUnfrozen.Free()); // OCCT clears it

        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Free(false));
        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Free(true));
        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Locked(false));
        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Locked(true));
        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Closed(false));
        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Orientable(false));
        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Infinite(true));
        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Convex(false));
        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Modified(true));
        Assert.Throws<Standard_ImmutableObject>(() => aFrozen.Checked(true));
    }

    [Fact]
    public void Cache_completions_return_at_once_on_frozen_shapes()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Face aFace = FirstFace(aBox);
        Assert.False(aFace.Checked());
        BRepTools.Update(aFace);
        BRepTools.Update(aBox);
        Assert.False(aFace.Checked());

        // an unfrozen face is updated and marked
        TopoDS_Solid anUnfrozen = TestShapes.Box(1, 1, 1);
        BRepTools.Update(anUnfrozen);
        Assert.True(FirstFace(anUnfrozen).Checked());
    }

    // 4.5 BRep_Builder

    [Fact]
    public void A_frozen_child_can_be_added_to_an_unfrozen_parent()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aBuilder = new BRep_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBox);
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        aBuilder.Add(aWire, FirstEdge(aBox)); // a frozen edge in a new wire
        aBuilder.Add(aCompound, aWire);
        Assert.Equal(2, aCompound.NbChildren());

        // and a new edge on frozen vertices can be given parameters: the vertices' point representations would
        // change, so this needs a copy (here it throws)
        TopExp.Vertices(FirstEdge(aBox), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        var aLine = new Geom_Line(BRep_Tool.Pnt(aV1), new gp_Dir(new gp_Vec(BRep_Tool.Pnt(aV1), BRep_Tool.Pnt(aV2))));
        TopoDS_Edge aNewEdge = TestShapes.Edge(aLine, 0, 1, aV1, aV2);
        Assert.Throws<TopoDS_LockedShape>(() => aBuilder.UpdateVertex(aV1, 0.5, aNewEdge, 1e-7));
    }
}
