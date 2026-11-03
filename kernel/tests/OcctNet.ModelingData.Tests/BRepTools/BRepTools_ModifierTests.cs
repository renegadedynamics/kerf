// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// BRepTools_History, BRepTools_Modifier with its modifications, BRepTools_PurgeLocations, and the copy for write with
/// the functional twins of BRepTools' in-place statics (docs/phase3-design.md, section 4.6), on frozen inputs: the
/// inputs stay unchanged (their whole graph, by <see cref="ShapeSnapshot"/>), the results are frozen and have the values
/// of OCCT's in-place operations. The values against native OCCT are in the oracle tests
/// (BRepToolsModifierOracleTests).
/// </summary>
public class BRepTools_ModifierTests
{
    private static List<TopoDS_Shape> SubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return [.. aMap];
    }

    private static void AssertAllNew(in TopoDS_Shape theInput, in TopoDS_Shape theResult)
    {
        var anInput = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theInput, aMap);
        foreach (TopoDS_Shape aS in aMap)
        {
            anInput.Add(aS.TShape()!);
        }

        aMap.Clear();
        TopExp.MapShapes(theResult, aMap);
        foreach (TopoDS_Shape aS in aMap)
        {
            Assert.DoesNotContain(aS.TShape()!, anInput);
        }
    }

    // A box whose first face has a triangulation and whose edges on it have polygons on it, unfrozen.
    private static TopoDS_Solid MeshedBox()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 2, 3);
        TopoDS_Face aFace = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        var aTri = new Triangulation(4, 2, false);
        var aBuilder = new BRep_Builder();
        aBuilder.UpdateFace(aFace, aTri);
        for (var anExp = new TopExp_Explorer(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aBuilder.UpdateEdge(TopoDS.Edge(anExp.Current()), new PolygonOnTriangulation(2, false), aTri, new Placement());
        }

        return aBox;
    }

    // History

    [Fact]
    public void History_records_modifications_generations_and_removals()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        List<TopoDS_Shape> aFaces = SubShapes(aBox, TopAbs_FACE);
        List<TopoDS_Shape> anEdges = SubShapes(aBox, TopAbs_EDGE);
        var aHistory = new BRepTools_History();
        Assert.False(aHistory.HasModified() || aHistory.HasGenerated() || aHistory.HasRemoved());

        aHistory.AddModified(aFaces[0], aFaces[1]);
        aHistory.AddModified(aFaces[0], aFaces[2]);
        aHistory.AddGenerated(anEdges[0], aFaces[3]);
        aHistory.Remove(aFaces[4]);
        Assert.Equal([aFaces[1], aFaces[2]], aHistory.Modified(aFaces[0]));
        Assert.Equal([aFaces[3]], aHistory.Generated(anEdges[0]));
        Assert.True(aHistory.IsRemoved(aFaces[4]));
        Assert.Empty(aHistory.Modified(aFaces[5]));

        // modified and removed exclude each other: the later call wins
        aHistory.AddModified(aFaces[4], aFaces[5]);
        Assert.False(aHistory.IsRemoved(aFaces[4]));
        aHistory.Remove(aFaces[4]);
        Assert.Empty(aHistory.Modified(aFaces[4]));

        // a shape generated and then modified from the same shape is only modified
        aHistory.AddModified(anEdges[0], aFaces[3]);
        Assert.Empty(aHistory.Generated(anEdges[0]));
        aHistory.ReplaceModified(aFaces[0], aFaces[5]);
        Assert.Equal([aFaces[5]], aHistory.Modified(aFaces[0]));

        // unsupported types (wires, shells, compounds) aren't recorded
        TopoDS_Shape aWire = SubShapes(aBox, TopAbs_WIRE)[0];
        aHistory.AddModified(aWire, aWire);
        aHistory.Remove(aWire);
        Assert.Empty(aHistory.Modified(aWire));
        Assert.False(aHistory.IsRemoved(aWire));

        // the returned lists are copies
        ISchematiqList<TopoDS_Shape> aList = aHistory.Modified(aFaces[0]);
        aHistory.AddModified(aFaces[0], aFaces[1]);
        Assert.Single(aList);

        var aText = new StringWriter();
        aHistory.Dump(aText);
        // as in OCCT, the edge whose generated face became modified keeps an empty list of generated shapes
        Assert.Equal("History contains:\n - 1 Deleted shapes;\n - 2 Modified shapes;\n - 1 Generated shapes.\n", aText.ToString());
        aHistory.Clear();
        Assert.False(aHistory.HasModified() || aHistory.HasRemoved());
    }

    [Fact]
    public void History_merge_answers_for_the_first_inputs()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        List<TopoDS_Shape> f = SubShapes(aBox, TopAbs_FACE);
        List<TopoDS_Shape> e = SubShapes(aBox, TopAbs_EDGE);
        var aH12 = new BRepTools_History();
        aH12.AddModified(f[0], f[1]);   // f0 -> f1 -> f2
        aH12.Remove(f[3]);              // f3 removed
        aH12.AddGenerated(e[0], f[4]);  // e0 generates f4, which is removed next
        aH12.AddModified(e[1], e[2]);   // e1 -> e2, which generates f5 next
        var aH23 = new BRepTools_History();
        aH23.AddModified(f[1], f[2]);
        aH23.Remove(f[4]);
        aH23.AddGenerated(e[2], f[5]);
        aH23.AddModified(e[3], e[4]);   // not in the first history: kept as is

        aH12.Merge(aH23);
        Assert.Equal([f[2]], aH12.Modified(f[0]));
        Assert.True(aH12.IsRemoved(f[3]));
        Assert.Empty(aH12.Generated(e[0]));
        Assert.True(aH12.IsRemoved(e[0]));
        Assert.Equal([e[2]], aH12.Modified(e[1]));
        Assert.Equal([f[5]], aH12.Generated(e[1]));
        Assert.Equal([e[4]], aH12.Modified(e[3]));

        // merging nothing changes nothing
        aH12.Merge(null);
        aH12.Merge(new BRepTools_History());
        Assert.Equal([f[2]], aH12.Modified(f[0]));
    }

    // Modifier

    [Fact]
    public void Copy_of_a_frozen_shape_is_new_topology_and_leaves_the_input_unchanged()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(MeshedBox());
        var aSnapshot = ShapeSnapshot.Of(aBox);
        foreach ((bool aCopyGeom, bool aCopyMesh) in new[] { (false, false), (false, true), (true, false), (true, true) })
        {
            var aModifier = new BRepTools_Modifier(aBox, new BRepTools_CopyModification(aCopyGeom, aCopyMesh));
            Assert.True(aModifier.IsDone());
            TopoDS_Shape anInternal = aModifier.InternalModifiedShape(aBox);
            Assert.False(anInternal.TShape()!.IsFrozen);
            TopoDS_Shape aCopy = aModifier.ModifiedShape(aBox);
            Assert.True(aCopy.IsEqual(anInternal));
            Assert.True(aCopy.TShape()!.IsFrozen);
            AssertAllNew(aBox, aCopy);
            aSnapshot.AssertUnchanged(aBox);

            // the geometry is shared unless copied
            TopoDS_Edge anEdge = TopoDS.Edge(SubShapes(aBox, TopAbs_EDGE)[0]);
            TopoDS_Edge aNewEdge = TopoDS.Edge(aModifier.ModifiedShape(anEdge));
            Geom_Curve aCurve = BRep_Tool.Curve(anEdge, out _, out _)!;
            Geom_Curve aNewCurve = BRep_Tool.Curve(aNewEdge, out _, out _)!;
            Assert.Equal(!aCopyGeom, ReferenceEquals(aCurve, aNewCurve));
            TopoDS_Face aFace = TopoDS.Face(SubShapes(aBox, TopAbs_FACE)[0]);
            Triangulation? aNewTri = BRep_Tool.Triangulation(TopoDS.Face(aModifier.ModifiedShape(aFace)), out _);
            Assert.Equal(aCopyMesh, aNewTri is not null);
            if (aNewTri is not null)
            {
                Assert.Equal(!aCopyGeom, ReferenceEquals(aNewTri, BRep_Tool.Triangulation(aFace, out _)));
            }
        }
    }

    [Fact]
    public void Transformation_moves_points_and_scales_tolerances()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        Transform3D aScale = new();
        aScale.SetScale(new Point3D(1, 1, 1), 2.0);
        var aModifier = new BRepTools_Modifier(aBox, new BRepTools_TrsfModification(aScale));
        TopoDS_Shape aResult = aModifier.ModifiedShape(aBox);
        foreach (TopoDS_Shape aV in SubShapes(aBox, TopAbs_VERTEX))
        {
            TopoDS_Vertex aNewV = TopoDS.Vertex(aModifier.ModifiedShape(aV));
            Assert.True(BRep_Tool.Pnt(aNewV).IsEqual(BRep_Tool.Pnt(TopoDS.Vertex(aV)).Transformed(aScale), 1e-12));
            Assert.Equal(2.0 * BRep_Tool.Tolerance(TopoDS.Vertex(aV)), BRep_Tool.Tolerance(aNewV));
        }

        Assert.Equal(TopAbs_FORWARD, aModifier.ModifiedShape(SubShapes(aBox, TopAbs_FACE)[0]).Orientation());
        aSnapshot.AssertUnchanged(aBox);
        AssertAllNew(aBox, aResult);

        // a mirror reverses the faces' new surfaces, so the faces are reversed
        Transform3D aMirror = new();
        aMirror.SetMirror(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)));
        var aMirrored = new BRepTools_Modifier(aBox, new BRepTools_TrsfModification(aMirror));
        Assert.Equal(TopAbs_REVERSED, aMirrored.ModifiedShape(SubShapes(aBox, TopAbs_FACE)[0]).Orientation());
        aSnapshot.AssertUnchanged(aBox);
    }

    [Fact]
    public void General_transformation_needs_NURBS_geometry()
    {
        // OCCT's BRepTools_GTrsfModification transforms B-spline and Bezier geometry only (GTransform converts first)
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aG = new GeneralTransform3D(new Matrix3x3(2, 0, 0, 0, 1, 0, 0, 0, 1), new Xyz());
        Assert.Throws<NoSuchObjectException>(() => new BRepTools_Modifier(aBox, new BRepTools_GTrsfModification(aG)));
    }

    [Fact]
    public void NURBS_conversion_of_a_frozen_edge()
    {
        TopoDS_Edge anEdge = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(3, 4, 0)));
        var aSnapshot = ShapeSnapshot.Of(anEdge);
        var aModifier = new BRepTools_Modifier(anEdge, new BRepTools_NurbsConvertModification());
        TopoDS_Edge aNewEdge = TopoDS.Edge(aModifier.ModifiedShape(anEdge));
        Geom_Curve aCurve = BRep_Tool.Curve(aNewEdge, out double f, out double l)!;
        Assert.IsType<Geom_BSplineCurve>(aCurve);
        Assert.Equal(0.0, f);
        Assert.Equal(5.0, l);
        Assert.True(aCurve.Value(2.5).IsEqual(new Point3D(1.5, 2, 0), 1e-12));
        aSnapshot.AssertUnchanged(anEdge);
        AssertAllNew(anEdge, aNewEdge);

        // OCCT's mutable input updates the input's vertices in place: on a frozen edge that throws, as on a locked one
        var aMutable = new BRepTools_Modifier(true);
        aMutable.Init(anEdge);
        Assert.Throws<TopoDS_LockedShape>(() => aMutable.Perform(new BRepTools_NurbsConvertModification()));
        aSnapshot.AssertUnchanged(anEdge);
    }

    [Fact]
    public void NURBS_conversion_of_a_box_projects_the_pcurves_on_the_planes()
    {
        // the box's faces carry no pcurves (BRep_Tool computes them on the planes): ProjLib_ComputeApprox projects the
        // converted edges again
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        var aModification = new BRepTools_NurbsConvertModification();
        var aModifier = new BRepTools_Modifier(aBox, aModification);
        TopoDS_Shape aResult = aModifier.ModifiedShape(aBox);
        Assert.True(aResult.TShape()!.IsFrozen);
        AssertNurbsAndConsistent(aResult, 6, 12);
        aSnapshot.AssertUnchanged(aBox);
        AssertAllNew(aBox, aResult);

        // on a plane the parametrization doesn't change: the new pcurve of each edge has the old one's points
        foreach (TopoDS_Shape aF in SubShapes(aBox, TopAbs_FACE))
        {
            TopoDS_Face aNewF = TopoDS.Face(aModifier.ModifiedShape(aF));
            foreach (TopoDS_Shape anE in SubShapes(aF, TopAbs_EDGE))
            {
                TopoDS_Edge aNewE = TopoDS.Edge(aModifier.ModifiedShape(anE));
                Geom2d_Curve anOld = BRep_Tool.CurveOnSurface(TopoDS.Edge(anE), TopoDS.Face(aF), out double f, out double l)!;
                Geom2d_Curve aNew = BRep_Tool.CurveOnSurface(aNewE, aNewF, out double nf, out double nl)!;
                Assert.Equal(f, nf, 1e-12);
                Assert.Equal(l, nl, 1e-12);
                Assert.True(anOld.Value(0.5 * (f + l)).IsEqual(aNew.Value(0.5 * (nf + nl)), 1e-9));
            }
        }
    }

    [Fact]
    public void NURBS_conversion_of_a_cylinder_face_projects_the_circles_and_the_seam()
    {
        // the circles are projected again with ProjLib_ComputeApproxOnPolarSurface, the seam with both its pcurves
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aSnapshot = ShapeSnapshot.Of(aFace);
        var aModifier = new BRepTools_Modifier(aFace, new BRepTools_NurbsConvertModification());
        TopoDS_Face aResult = TopoDS.Face(aModifier.ModifiedShape(aFace));
        Assert.True(aResult.TShape()!.IsFrozen);
        AssertNurbsAndConsistent(aResult, 1, 3);
        aSnapshot.AssertUnchanged(aFace);
        AssertAllNew(aFace, aResult);

        // the seam stays a seam, its two pcurves a period apart on the B-spline surface
        TopoDS_Edge aSeam = TopoDS.Edge(SubShapes(aResult, TopAbs_EDGE).Single(e => BRep_Tool.IsClosed(TopoDS.Edge(e), aResult)));
        Geom2d_Curve aC1 = BRep_Tool.CurveOnSurface(aSeam, aResult, out double f, out double l)!;
        Geom2d_Curve aC2 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aSeam.Reversed()), aResult, out _, out _)!;
        Geom_Surface aSurf = BRep_Tool.Surface(aResult)!;
        aSurf.Bounds(out double u1, out double u2, out _, out _);
        double aMid = 0.5 * (f + l);
        Assert.Equal(u2 - u1, Math.Abs(aC1.Value(aMid).X() - aC2.Value(aMid).X()), 1e-7);
    }

    // Every face of the result on a B-spline surface, every edge's curve and pcurves B-splines, the pcurves on the
    // surface within the edge's tolerance of the 3D curve.
    private static void AssertNurbsAndConsistent(in TopoDS_Shape theResult, int theNbFaces, int theNbEdges)
    {
        Assert.Equal(theNbFaces, SubShapes(theResult, TopAbs_FACE).Count);
        Assert.Equal(theNbEdges, SubShapes(theResult, TopAbs_EDGE).Count);
        foreach (TopoDS_Shape aS in SubShapes(theResult, TopAbs_FACE))
        {
            TopoDS_Face aF = TopoDS.Face(aS);
            Geom_Surface aSurf = BRep_Tool.Surface(aF, out Placement aSL)!;
            Assert.IsType<Geom_BSplineSurface>(aSurf);
            foreach (TopoDS_Shape anS in SubShapes(aF, TopAbs_EDGE))
            {
                TopoDS_Edge anE = TopoDS.Edge(anS);
                Geom_Curve aC = BRep_Tool.Curve(anE, out Placement aCL, out double f, out double l)!;
                Assert.IsType<Geom_BSplineCurve>(aC);
                foreach (TopoDS_Edge anOriented in new[] { anE, TopoDS.Edge(anE.Reversed()) })
                {
                    Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(anOriented, aF, out double pf, out double pl, out bool aStored)!;
                    Assert.True(aStored);
                    Assert.IsType<Geom2d_BSplineCurve>(aPC);
                    Assert.Equal(f, pf, 1e-12);
                    Assert.Equal(l, pl, 1e-12);
                    double aTol = BRep_Tool.Tolerance(anE);
                    for (int i = 0; i <= 10; i++)
                    {
                        double t = f + (l - f) * i / 10;
                        Point2D aUV = aPC.Value(t);
                        Point3D aOnS = aSurf.Value(aUV.X(), aUV.Y()).Transformed(aSL.Transformation());
                        Point3D aOnC = aC.Value(t).Transformed(aCL.Transformation());
                        Assert.True(aOnS.Distance(aOnC) <= aTol, $"{aOnS.Distance(aOnC)} > {aTol}");
                    }
                }
            }
        }
    }

    [Fact]
    public void ModifiedShape_of_an_unknown_shape_throws()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aModifier = new BRepTools_Modifier(aBox, new BRepTools_CopyModification());
        Assert.Throws<NoSuchObjectException>(() => aModifier.ModifiedShape(TestShapes.Vertex(new Point3D())));
        Assert.Throws<NullObjectException>(() => new BRepTools_Modifier().Perform(new BRepTools_CopyModification()));
    }

    // Copy for write

    [Fact]
    public void CopyForWrite_copies_the_structure_shares_the_geometry_and_keeps_the_meshes()
    {
        TopoDS_Solid aBox = MeshedBox();
        TopoDS_Face aFace = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        // a second triangulation, active
        var aTri1 = BRep_Tool.Triangulation(aFace, out _)!;
        var aTri2 = new Triangulation(3, 1, false);
        ((BRep_TFace)aFace.TShape()!).Triangulations(new NCollection_List<Triangulation>([aTri1, aTri2]), aTri2);
        TestShapes.Frozen(aBox);
        var aSnapshot = ShapeSnapshot.Of(aBox);

        TopoDS_Shape aCopy = BRepTools.CopyForWrite(aBox, out BRepTools_History aHistory);
        Assert.False(aCopy.TShape()!.IsFrozen);
        AssertAllNew(aBox, aCopy);
        TopoDS_Face aNewFace = TopoDS.Face(aHistory.Modified(aFace)[0]);
        Assert.Same(BRep_Tool.Surface(aFace), BRep_Tool.Surface(aNewFace));
        Assert.Equal([aTri1, aTri2], ((BRep_TFace)aNewFace.TShape()!).TriangulationList());
        Assert.Same(aTri2, BRep_Tool.Triangulation(aNewFace, out _));
        foreach (TopoDS_Shape aS in SubShapes(aBox, TopAbs_SHAPE))
        {
            if (BRepTools_History.IsSupportedType(aS))
            {
                Assert.Single(aHistory.Modified(aS));
            }
        }

        // the copy can be written into; the input is untouched
        new BRep_Builder().UpdateFace(aNewFace, 0.5);
        Assert.Equal(0.5, BRep_Tool.Tolerance(aNewFace));
        aSnapshot.AssertUnchanged(aBox);
        Assert.True(BRepTools.CopyForWrite(new TopoDS_Shape(), out _).IsNull());
    }

    [Fact]
    public void CopyForWrite_history_merges_with_the_next_algorithm()
    {
        // what an algorithm writing into a frozen input does (design 4.6): copy, run on the copy, compose the histories
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aCopy = BRepTools.CopyForWrite(aBox, out BRepTools_History aHistory);
        List<TopoDS_Shape> aFaces = SubShapes(aBox, TopAbs_FACE);
        TopoDS_Shape aCopiedFace = aHistory.Modified(aFaces[0])[0];
        TopoDS_Shape aCopiedFace1 = aHistory.Modified(aFaces[1])[0];

        var aReShape = new BRepTools_ReShape();
        aReShape.Remove(aCopiedFace);
        TopoDS_Shape aResult = aReShape.Apply(aCopy);
        aHistory.Merge(aReShape.History());

        Assert.True(aHistory.IsRemoved(aFaces[0]));
        Assert.Equal([aCopiedFace1], aHistory.Modified(aFaces[1]));
        Assert.Equal(5, SubShapes(aResult, TopAbs_FACE).Count);
        Assert.True(aResult.TShape()!.IsFrozen);
        aSnapshot.AssertUnchanged(aBox);
    }

    // The functional twins: the twin on a frozen shape has the values of the in-place static on an unfrozen copy.

    private static void AssertTwin(Func<TopoDS_Shape> theShape, Action<TopoDS_Shape> theInPlace,
                                   Func<TopoDS_Shape, TopoDS_Shape> theTwin)
    {
        TopoDS_Shape aShape = theShape();
        TopoDS_Shape anUnfrozen = BRepTools.CopyForWrite(aShape, out _);
        TopoDS_Shape aFrozen = BRepTools.CopyForWrite(aShape, out _);
        aFrozen.TShape()!.FreezeGraph();
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        theInPlace(anUnfrozen);
        TopoDS_Shape aResult = theTwin(aFrozen);
        Assert.True(aResult.TShape()!.IsFrozen);
        aSnapshot.AssertUnchanged(aFrozen);
        ShapeSnapshot.Of(anUnfrozen).AssertSameValues(ShapeSnapshot.Of(aResult));
        AssertAllNew(aFrozen, aResult);
    }

    [Fact]
    public void Twins_of_the_in_place_statics_answer_as_the_statics()
    {
        AssertTwin(() => MeshedBox(), s => BRepTools.Clean(s), s => BRepTools.Cleaned(s));
        AssertTwin(() => MeshedBox(), s => BRepTools.Clean(s, true), s => BRepTools.Cleaned(s, out _, true));
        AssertTwin(() => MeshedBox(), s => BRepTools.CleanGeometry(s), s => BRepTools.GeometryCleaned(s));
        AssertTwin(() => MeshedBox(), s => BRepTools.RemoveUnusedPCurves(s), s => BRepTools.UnusedPCurvesRemoved(s));
        AssertTwin(() => MeshedBox(), s => BRepTools.Update(s), s => BRepTools.Updated(s));
        AssertTwin(() => TestShapes.CylinderFace(1, 2), s => BRepTools.UpdateFaceUVPoints(TopoDS.Face(s)),
                   s => BRepTools.FaceUVPointsUpdated(TopoDS.Face(s)));
        AssertTwin(() =>
                   {
                       var aBuilder = new BRep_Builder();
                       TopoDS_Compound aCompound = new();
                       aBuilder.MakeCompound(ref aCompound);
                       aBuilder.Add(aCompound, TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0)));
                       aBuilder.Add(aCompound, TestShapes.Edge(new Point3D(0, 1, 0), new Point3D(1, 1, 0)).Oriented(TopAbs_INTERNAL));
                       return aCompound;
                   },
                   s => BRepTools.RemoveInternals(s), s => BRepTools.InternalsRemoved(s));
        AssertTwin(() => MeshedBox(), s => BRepTools.UnloadAllTriangulations(s), s => BRepTools.AllTriangulationsUnloaded(s, out _));
        AssertTwin(() => MeshedBox(), s => BRepTools.LoadAllTriangulations(s), s => BRepTools.AllTriangulationsLoaded(s, out _));
        AssertTwin(() => MeshedBox(), s => BRepTools.LoadTriangulation(s), s => BRepTools.TriangulationLoaded(s, out _));
        AssertTwin(() => MeshedBox(), s => BRepTools.UnloadTriangulation(s), s => BRepTools.TriangulationUnloaded(s, out _));

        // two triangulations on a face: activate the first
        AssertTwin(() =>
                   {
                       TopoDS_Solid aBox = MeshedBox();
                       TopoDS_Face aFace = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
                       var aTri1 = BRep_Tool.Triangulation(aFace, out _)!;
                       ((BRep_TFace)aFace.TShape()!).Triangulations(
                           new NCollection_List<Triangulation>([aTri1, new Triangulation(3, 1, false)]), null);
                       return aBox;
                   },
                   s => Assert.True(BRepTools.ActivateTriangulation(s, 1)),
                   s =>
                   {
                       TopoDS_Shape aResult = BRepTools.TriangulationActivated(s, 1, out bool anIsActivated);
                       Assert.True(anIsActivated);
                       return aResult;
                   });
    }

    [Fact]
    public void Twin_of_EvalAndUpdateTol_raises_the_tolerance_of_a_copy()
    {
        // an edge on a plane whose pcurve is 0.01 off its 3D curve
        TopoDS_Edge anEdge = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        Geom_Curve aC3d = BRep_Tool.Curve(anEdge, out double f, out double l)!;
        var aPlane = new Geom_Plane(new CoordinateSystem(Constants.XOY()));
        var aC2d = new Geom2d_Line(new Point2D(0, 0.01), new Direction2D(1, 0));
        double aTol0 = BRep_Tool.Tolerance(anEdge);
        TopoDS_Edge aResult = BRepTools.TolEvaluatedAndUpdated(anEdge, aC3d, aC2d, aPlane, f, l, out double aTol);
        Assert.Equal(0.01, aTol, 1e-12);
        Assert.Equal(aTol, BRep_Tool.Tolerance(aResult));
        Assert.Equal(aTol0, BRep_Tool.Tolerance(anEdge));
        Assert.True(aResult.TShape()!.IsFrozen);
    }

    // PurgeLocations

    [Fact]
    public void PurgeLocations_applies_the_scaling_locations_to_the_geometry()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Transform3D aScale = new();
        aScale.SetScale(new Point3D(1, 0, 0), 1.5);
        Transform3D aMove = Transform3D.Translation(new Vector3D(0, 0, 10));
        TopoDS_Shape aPlaced = aBox.Moved(new Placement(aScale)).Moved(new Placement(aMove));
        var aSnapshot = ShapeSnapshot.Of(aPlaced);

        var aPurge = new BRepTools_PurgeLocations();
        Assert.True(aPurge.Perform(aPlaced));
        Assert.True(aPurge.IsDone());
        TopoDS_Shape aResult = aPurge.GetResult();
        Assert.True(aResult.TShape()!.IsFrozen);
        aSnapshot.AssertUnchanged(aPlaced);

        // no location that isn't a rigid motion is left, and the vertices are where the placed box had them
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, aMap, false, false);
        foreach (TopoDS_Shape aS in aMap)
        {
            Transform3D aT = aS.Location().Transformation();
            Assert.False(aT.IsNegative);
            Assert.Equal(1.0, Math.Abs(aT.ScaleFactor()), 1e-14);
        }

        var aBefore = SubShapes(aPlaced, TopAbs_VERTEX).Select(v => BRep_Tool.Pnt(TopoDS.Vertex(v))).ToList();
        var anAfter = SubShapes(aResult, TopAbs_VERTEX).Select(v => BRep_Tool.Pnt(TopoDS.Vertex(v))).ToList();
        Assert.Equal(aBefore.Count, anAfter.Count);
        for (int i = 0; i < aBefore.Count; i++)
        {
            Assert.True(aBefore[i].IsEqual(anAfter[i], 1e-12));
        }

        // a shape with rigid locations only is left as it is
        var aRigid = new BRepTools_PurgeLocations();
        TopoDS_Shape aMoved = aBox.Moved(new Placement(aMove));
        Assert.True(aRigid.Perform(aMoved));
        Assert.True(aRigid.GetResult().IsEqual(aMoved));
        Assert.True(aRigid.ModifiedShape(aBox).IsEqual(aBox));
    }
}
