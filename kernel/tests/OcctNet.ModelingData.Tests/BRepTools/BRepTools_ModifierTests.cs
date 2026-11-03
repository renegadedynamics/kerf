// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    private static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return [.. aMap];
    }

    private static void AssertAllNew(in Shape theInput, in Shape theResult)
    {
        var anInput = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theInput, aMap);
        foreach (Shape aS in aMap)
        {
            anInput.Add(aS.TShape()!);
        }

        aMap.Clear();
        Exploration.MapShapes(theResult, aMap);
        foreach (Shape aS in aMap)
        {
            Assert.DoesNotContain(aS.TShape()!, anInput);
        }
    }

    // A box whose first face has a triangulation and whose edges on it have polygons on it, unfrozen.
    private static Solid MeshedBox()
    {
        Solid aBox = TestShapes.Box(1, 2, 3);
        Face aFace = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        var aTri = new Triangulation(4, 2, false);
        var aBuilder = new BrepBuilder();
        aBuilder.UpdateFace(aFace, aTri);
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aBuilder.UpdateEdge(ShapeCasts.Edge(anExp.Current()), new PolygonOnTriangulation(2, false), aTri, new Placement());
        }

        return aBox;
    }

    // History

    [Fact]
    public void History_records_modifications_generations_and_removals()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        List<Shape> aFaces = SubShapes(aBox, ShapeType.Face);
        List<Shape> anEdges = SubShapes(aBox, ShapeType.Edge);
        var aHistory = new History();
        Assert.False(aHistory.HasModified || aHistory.HasGenerated || aHistory.HasRemoved);

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
        Shape aWire = SubShapes(aBox, ShapeType.Wire)[0];
        aHistory.AddModified(aWire, aWire);
        aHistory.Remove(aWire);
        Assert.Empty(aHistory.Modified(aWire));
        Assert.False(aHistory.IsRemoved(aWire));

        // the returned lists are copies
        ISchematiqList<Shape> aList = aHistory.Modified(aFaces[0]);
        aHistory.AddModified(aFaces[0], aFaces[1]);
        Assert.Single(aList);

        var aText = new StringWriter();
        aHistory.Dump(aText);
        // as in OCCT, the edge whose generated face became modified keeps an empty list of generated shapes
        Assert.Equal("History contains:\n - 1 Deleted shapes;\n - 2 Modified shapes;\n - 1 Generated shapes.\n", aText.ToString());
        aHistory.Clear();
        Assert.False(aHistory.HasModified || aHistory.HasRemoved);
    }

    [Fact]
    public void History_merge_answers_for_the_first_inputs()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        List<Shape> f = SubShapes(aBox, ShapeType.Face);
        List<Shape> e = SubShapes(aBox, ShapeType.Edge);
        var aH12 = new History();
        aH12.AddModified(f[0], f[1]);   // f0 -> f1 -> f2
        aH12.Remove(f[3]);              // f3 removed
        aH12.AddGenerated(e[0], f[4]);  // e0 generates f4, which is removed next
        aH12.AddModified(e[1], e[2]);   // e1 -> e2, which generates f5 next
        var aH23 = new History();
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
        aH12.Merge(new History());
        Assert.Equal([f[2]], aH12.Modified(f[0]));
    }

    // Modifier

    [Fact]
    public void Copy_of_a_frozen_shape_is_new_topology_and_leaves_the_input_unchanged()
    {
        Solid aBox = TestShapes.Frozen(MeshedBox());
        var aSnapshot = ShapeSnapshot.Of(aBox);
        foreach ((bool aCopyGeom, bool aCopyMesh) in new[] { (false, false), (false, true), (true, false), (true, true) })
        {
            var aModifier = new Modifier(aBox, new CopyModification(aCopyGeom, aCopyMesh));
            Assert.True(aModifier.IsDone);
            Shape anInternal = aModifier.InternalModifiedShape(aBox);
            Assert.False(anInternal.TShape()!.IsFrozen);
            Shape aCopy = aModifier.ModifiedShape(aBox);
            Assert.True(aCopy.IsEqual(anInternal));
            Assert.True(aCopy.TShape()!.IsFrozen);
            AssertAllNew(aBox, aCopy);
            aSnapshot.AssertUnchanged(aBox);

            // the geometry is shared unless copied
            Edge anEdge = ShapeCasts.Edge(SubShapes(aBox, ShapeType.Edge)[0]);
            Edge aNewEdge = ShapeCasts.Edge(aModifier.ModifiedShape(anEdge));
            Curve3D aCurve = Tool.Curve(anEdge, out _, out _)!;
            Curve3D aNewCurve = Tool.Curve(aNewEdge, out _, out _)!;
            Assert.Equal(!aCopyGeom, ReferenceEquals(aCurve, aNewCurve));
            Face aFace = ShapeCasts.Face(SubShapes(aBox, ShapeType.Face)[0]);
            Triangulation? aNewTri = Tool.Triangulation(ShapeCasts.Face(aModifier.ModifiedShape(aFace)), out _);
            Assert.Equal(aCopyMesh, aNewTri is not null);
            if (aNewTri is not null)
            {
                Assert.Equal(!aCopyGeom, ReferenceEquals(aNewTri, Tool.Triangulation(aFace, out _)));
            }
        }
    }

    [Fact]
    public void Transformation_moves_points_and_scales_tolerances()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        Transform3D aScale = new();
        aScale.SetScale(new Point3D(1, 1, 1), 2.0);
        var aModifier = new Modifier(aBox, new TrsfModification(aScale));
        Shape aResult = aModifier.ModifiedShape(aBox);
        foreach (Shape aV in SubShapes(aBox, ShapeType.Vertex))
        {
            Vertex aNewV = ShapeCasts.Vertex(aModifier.ModifiedShape(aV));
            Assert.True(Tool.Pnt(aNewV).IsEqual(Tool.Pnt(ShapeCasts.Vertex(aV)).Transformed(aScale), 1e-12));
            Assert.Equal(2.0 * Tool.Tolerance(ShapeCasts.Vertex(aV)), Tool.Tolerance(aNewV));
        }

        Assert.Equal(Forward, aModifier.ModifiedShape(SubShapes(aBox, ShapeType.Face)[0]).Orientation());
        aSnapshot.AssertUnchanged(aBox);
        AssertAllNew(aBox, aResult);

        // a mirror reverses the faces' new surfaces, so the faces are reversed
        Transform3D aMirror = new();
        aMirror.SetMirror(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)));
        var aMirrored = new Modifier(aBox, new TrsfModification(aMirror));
        Assert.Equal(Reversed, aMirrored.ModifiedShape(SubShapes(aBox, ShapeType.Face)[0]).Orientation());
        aSnapshot.AssertUnchanged(aBox);
    }

    [Fact]
    public void General_transformation_needs_NURBS_geometry()
    {
        // OCCT's BRepTools_GTrsfModification transforms B-spline and Bezier geometry only (GTransform converts first)
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aG = new GeneralTransform3D(new Matrix3x3(2, 0, 0, 0, 1, 0, 0, 0, 1), new Xyz());
        Assert.Throws<NoSuchObjectException>(() => new Modifier(aBox, new GTrsfModification(aG)));
    }

    [Fact]
    public void NURBS_conversion_of_a_frozen_edge()
    {
        Edge anEdge = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(3, 4, 0)));
        var aSnapshot = ShapeSnapshot.Of(anEdge);
        var aModifier = new Modifier(anEdge, new NurbsConvertModification());
        Edge aNewEdge = ShapeCasts.Edge(aModifier.ModifiedShape(anEdge));
        Curve3D aCurve = Tool.Curve(aNewEdge, out double f, out double l)!;
        Assert.IsType<BSplineCurve3D>(aCurve);
        Assert.Equal(0.0, f);
        Assert.Equal(5.0, l);
        Assert.True(aCurve.Value(2.5).IsEqual(new Point3D(1.5, 2, 0), 1e-12));
        aSnapshot.AssertUnchanged(anEdge);
        AssertAllNew(anEdge, aNewEdge);

        // OCCT's mutable input updates the input's vertices in place: on a frozen edge that throws, as on a locked one
        var aMutable = new Modifier(true);
        aMutable.Init(anEdge);
        Assert.Throws<LockedShapeException>(() => aMutable.Perform(new NurbsConvertModification()));
        aSnapshot.AssertUnchanged(anEdge);
    }

    [Fact]
    public void NURBS_conversion_of_a_box_projects_the_pcurves_on_the_planes()
    {
        // the box's faces carry no pcurves (BRep_Tool computes them on the planes): ProjLib_ComputeApprox projects the
        // converted edges again
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        var aModification = new NurbsConvertModification();
        var aModifier = new Modifier(aBox, aModification);
        Shape aResult = aModifier.ModifiedShape(aBox);
        Assert.True(aResult.TShape()!.IsFrozen);
        AssertNurbsAndConsistent(aResult, 6, 12);
        aSnapshot.AssertUnchanged(aBox);
        AssertAllNew(aBox, aResult);

        // on a plane the parametrization doesn't change: the new pcurve of each edge has the old one's points
        foreach (Shape aF in SubShapes(aBox, ShapeType.Face))
        {
            Face aNewF = ShapeCasts.Face(aModifier.ModifiedShape(aF));
            foreach (Shape anE in SubShapes(aF, ShapeType.Edge))
            {
                Edge aNewE = ShapeCasts.Edge(aModifier.ModifiedShape(anE));
                Curve2D anOld = Tool.CurveOnSurface(ShapeCasts.Edge(anE), ShapeCasts.Face(aF), out double f, out double l)!;
                Curve2D aNew = Tool.CurveOnSurface(aNewE, aNewF, out double nf, out double nl)!;
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
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aSnapshot = ShapeSnapshot.Of(aFace);
        var aModifier = new Modifier(aFace, new NurbsConvertModification());
        Face aResult = ShapeCasts.Face(aModifier.ModifiedShape(aFace));
        Assert.True(aResult.TShape()!.IsFrozen);
        AssertNurbsAndConsistent(aResult, 1, 3);
        aSnapshot.AssertUnchanged(aFace);
        AssertAllNew(aFace, aResult);

        // the seam stays a seam, its two pcurves a period apart on the B-spline surface
        Edge aSeam = ShapeCasts.Edge(SubShapes(aResult, ShapeType.Edge).Single(e => Tool.IsClosed(ShapeCasts.Edge(e), aResult)));
        Curve2D aC1 = Tool.CurveOnSurface(aSeam, aResult, out double f, out double l)!;
        Curve2D aC2 = Tool.CurveOnSurface(ShapeCasts.Edge(aSeam.Reversed()), aResult, out _, out _)!;
        Surface aSurf = Tool.Surface(aResult)!;
        aSurf.Bounds(out double u1, out double u2, out _, out _);
        double aMid = 0.5 * (f + l);
        Assert.Equal(u2 - u1, Math.Abs(aC1.Value(aMid).X() - aC2.Value(aMid).X()), 1e-7);
    }

    // Every face of the result on a B-spline surface, every edge's curve and pcurves B-splines, the pcurves on the
    // surface within the edge's tolerance of the 3D curve.
    private static void AssertNurbsAndConsistent(in Shape theResult, int theNbFaces, int theNbEdges)
    {
        Assert.Equal(theNbFaces, SubShapes(theResult, ShapeType.Face).Count);
        Assert.Equal(theNbEdges, SubShapes(theResult, ShapeType.Edge).Count);
        foreach (Shape aS in SubShapes(theResult, ShapeType.Face))
        {
            Face aF = ShapeCasts.Face(aS);
            Surface aSurf = Tool.Surface(aF, out Placement aSL)!;
            Assert.IsType<BSplineSurface>(aSurf);
            foreach (Shape anS in SubShapes(aF, ShapeType.Edge))
            {
                Edge anE = ShapeCasts.Edge(anS);
                Curve3D aC = Tool.Curve(anE, out Placement aCL, out double f, out double l)!;
                Assert.IsType<BSplineCurve3D>(aC);
                foreach (Edge anOriented in new[] { anE, ShapeCasts.Edge(anE.Reversed()) })
                {
                    Curve2D aPC = Tool.CurveOnSurface(anOriented, aF, out double pf, out double pl, out bool aStored)!;
                    Assert.True(aStored);
                    Assert.IsType<BSplineCurve2D>(aPC);
                    Assert.Equal(f, pf, 1e-12);
                    Assert.Equal(l, pl, 1e-12);
                    double aTol = Tool.Tolerance(anE);
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
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aModifier = new Modifier(aBox, new CopyModification());
        Assert.Throws<NoSuchObjectException>(() => aModifier.ModifiedShape(TestShapes.Vertex(new Point3D())));
        Assert.Throws<NullObjectException>(() => new Modifier().Perform(new CopyModification()));
    }

    // Copy for write

    [Fact]
    public void CopyForWrite_copies_the_structure_shares_the_geometry_and_keeps_the_meshes()
    {
        Solid aBox = MeshedBox();
        Face aFace = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        // a second triangulation, active
        var aTri1 = Tool.Triangulation(aFace, out _)!;
        var aTri2 = new Triangulation(3, 1, false);
        ((BrepTFace)aFace.TShape()!).Triangulations(new NCollection_List<Triangulation>([aTri1, aTri2]), aTri2);
        TestShapes.Frozen(aBox);
        var aSnapshot = ShapeSnapshot.Of(aBox);

        Shape aCopy = BrepTools.CopyForWrite(aBox, out History aHistory);
        Assert.False(aCopy.TShape()!.IsFrozen);
        AssertAllNew(aBox, aCopy);
        Face aNewFace = ShapeCasts.Face(aHistory.Modified(aFace)[0]);
        Assert.Same(Tool.Surface(aFace), Tool.Surface(aNewFace));
        Assert.Equal([aTri1, aTri2], ((BrepTFace)aNewFace.TShape()!).TriangulationList());
        Assert.Same(aTri2, Tool.Triangulation(aNewFace, out _));
        foreach (Shape aS in SubShapes(aBox, ShapeType.Shape))
        {
            if (History.IsSupportedType(aS))
            {
                Assert.Single(aHistory.Modified(aS));
            }
        }

        // the copy can be written into; the input is untouched
        new BrepBuilder().UpdateFace(aNewFace, 0.5);
        Assert.Equal(0.5, Tool.Tolerance(aNewFace));
        aSnapshot.AssertUnchanged(aBox);
        Assert.True(BrepTools.CopyForWrite(new Shape(), out _).IsNull);
    }

    [Fact]
    public void CopyForWrite_history_merges_with_the_next_algorithm()
    {
        // what an algorithm writing into a frozen input does (design 4.6): copy, run on the copy, compose the histories
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aSnapshot = ShapeSnapshot.Of(aBox);
        Shape aCopy = BrepTools.CopyForWrite(aBox, out History aHistory);
        List<Shape> aFaces = SubShapes(aBox, ShapeType.Face);
        Shape aCopiedFace = aHistory.Modified(aFaces[0])[0];
        Shape aCopiedFace1 = aHistory.Modified(aFaces[1])[0];

        var aReShape = new ReShape();
        aReShape.Remove(aCopiedFace);
        Shape aResult = aReShape.Apply(aCopy);
        aHistory.Merge(aReShape.History());

        Assert.True(aHistory.IsRemoved(aFaces[0]));
        Assert.Equal([aCopiedFace1], aHistory.Modified(aFaces[1]));
        Assert.Equal(5, SubShapes(aResult, ShapeType.Face).Count);
        Assert.True(aResult.TShape()!.IsFrozen);
        aSnapshot.AssertUnchanged(aBox);
    }

    // The functional twins: the twin on a frozen shape has the values of the in-place static on an unfrozen copy.

    private static void AssertTwin(Func<Shape> theShape, Action<Shape> theInPlace,
                                   Func<Shape, Shape> theTwin)
    {
        Shape aShape = theShape();
        Shape anUnfrozen = BrepTools.CopyForWrite(aShape, out _);
        Shape aFrozen = BrepTools.CopyForWrite(aShape, out _);
        aFrozen.TShape()!.FreezeGraph();
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        theInPlace(anUnfrozen);
        Shape aResult = theTwin(aFrozen);
        Assert.True(aResult.TShape()!.IsFrozen);
        aSnapshot.AssertUnchanged(aFrozen);
        ShapeSnapshot.Of(anUnfrozen).AssertSameValues(ShapeSnapshot.Of(aResult));
        AssertAllNew(aFrozen, aResult);
    }

    [Fact]
    public void Twins_of_the_in_place_statics_answer_as_the_statics()
    {
        AssertTwin(() => MeshedBox(), s => BrepTools.Clean(s), s => BrepTools.Cleaned(s));
        AssertTwin(() => MeshedBox(), s => BrepTools.Clean(s, true), s => BrepTools.Cleaned(s, out _, true));
        AssertTwin(() => MeshedBox(), s => BrepTools.CleanGeometry(s), s => BrepTools.GeometryCleaned(s));
        AssertTwin(() => MeshedBox(), s => BrepTools.RemoveUnusedPCurves(s), s => BrepTools.UnusedPCurvesRemoved(s));
        AssertTwin(() => MeshedBox(), s => BrepTools.Update(s), s => BrepTools.Updated(s));
        AssertTwin(() => TestShapes.CylinderFace(1, 2), s => BrepTools.UpdateFaceUVPoints(ShapeCasts.Face(s)),
                   s => BrepTools.FaceUVPointsUpdated(ShapeCasts.Face(s)));
        AssertTwin(() =>
                   {
                       var aBuilder = new BrepBuilder();
                       Compound aCompound = new();
                       aBuilder.MakeCompound(ref aCompound);
                       aBuilder.Add(aCompound, TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0)));
                       aBuilder.Add(aCompound, TestShapes.Edge(new Point3D(0, 1, 0), new Point3D(1, 1, 0)).Oriented(Internal));
                       return aCompound;
                   },
                   s => BrepTools.RemoveInternals(s), s => BrepTools.InternalsRemoved(s));
        AssertTwin(() => MeshedBox(), s => BrepTools.UnloadAllTriangulations(s), s => BrepTools.AllTriangulationsUnloaded(s, out _));
        AssertTwin(() => MeshedBox(), s => BrepTools.LoadAllTriangulations(s), s => BrepTools.AllTriangulationsLoaded(s, out _));
        AssertTwin(() => MeshedBox(), s => BrepTools.LoadTriangulation(s), s => BrepTools.TriangulationLoaded(s, out _));
        AssertTwin(() => MeshedBox(), s => BrepTools.UnloadTriangulation(s), s => BrepTools.TriangulationUnloaded(s, out _));

        // two triangulations on a face: activate the first
        AssertTwin(() =>
                   {
                       Solid aBox = MeshedBox();
                       Face aFace = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
                       var aTri1 = Tool.Triangulation(aFace, out _)!;
                       ((BrepTFace)aFace.TShape()!).Triangulations(
                           new NCollection_List<Triangulation>([aTri1, new Triangulation(3, 1, false)]), null);
                       return aBox;
                   },
                   s => Assert.True(BrepTools.ActivateTriangulation(s, 1)),
                   s =>
                   {
                       Shape aResult = BrepTools.TriangulationActivated(s, 1, out bool anIsActivated);
                       Assert.True(anIsActivated);
                       return aResult;
                   });
    }

    [Fact]
    public void Twin_of_EvalAndUpdateTol_raises_the_tolerance_of_a_copy()
    {
        // an edge on a plane whose pcurve is 0.01 off its 3D curve
        Edge anEdge = TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        Curve3D aC3d = Tool.Curve(anEdge, out double f, out double l)!;
        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        var aC2d = new LineCurve2D(new Point2D(0, 0.01), new Direction2D(1, 0));
        double aTol0 = Tool.Tolerance(anEdge);
        Edge aResult = BrepTools.TolEvaluatedAndUpdated(anEdge, aC3d, aC2d, aPlane, f, l, out double aTol);
        Assert.Equal(0.01, aTol, 1e-12);
        Assert.Equal(aTol, Tool.Tolerance(aResult));
        Assert.Equal(aTol0, Tool.Tolerance(anEdge));
        Assert.True(aResult.TShape()!.IsFrozen);
    }

    // PurgeLocations

    [Fact]
    public void PurgeLocations_applies_the_scaling_locations_to_the_geometry()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Transform3D aScale = new();
        aScale.SetScale(new Point3D(1, 0, 0), 1.5);
        Transform3D aMove = Transform3D.Translation(new Vector3D(0, 0, 10));
        Shape aPlaced = aBox.Moved(new Placement(aScale)).Moved(new Placement(aMove));
        var aSnapshot = ShapeSnapshot.Of(aPlaced);

        var aPurge = new PurgeLocations();
        Assert.True(aPurge.Perform(aPlaced));
        Assert.True(aPurge.IsDone);
        Shape aResult = aPurge.GetResult();
        Assert.True(aResult.TShape()!.IsFrozen);
        aSnapshot.AssertUnchanged(aPlaced);

        // no location that isn't a rigid motion is left, and the vertices are where the placed box had them
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, aMap, false, false);
        foreach (Shape aS in aMap)
        {
            Transform3D aT = aS.Location().Transformation();
            Assert.False(aT.IsNegative);
            Assert.Equal(1.0, Math.Abs(aT.ScaleFactor()), 1e-14);
        }

        var aBefore = SubShapes(aPlaced, ShapeType.Vertex).Select(v => Tool.Pnt(ShapeCasts.Vertex(v))).ToList();
        var anAfter = SubShapes(aResult, ShapeType.Vertex).Select(v => Tool.Pnt(ShapeCasts.Vertex(v))).ToList();
        Assert.Equal(aBefore.Count, anAfter.Count);
        for (int i = 0; i < aBefore.Count; i++)
        {
            Assert.True(aBefore[i].IsEqual(anAfter[i], 1e-12));
        }

        // a shape with rigid locations only is left as it is
        var aRigid = new PurgeLocations();
        Shape aMoved = aBox.Moved(new Placement(aMove));
        Assert.True(aRigid.Perform(aMoved));
        Assert.True(aRigid.GetResult().IsEqual(aMoved));
        Assert.True(aRigid.ModifiedShape(aBox).IsEqual(aBox));
    }
}
