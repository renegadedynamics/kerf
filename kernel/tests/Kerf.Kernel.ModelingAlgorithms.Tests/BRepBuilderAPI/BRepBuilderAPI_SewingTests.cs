// Port of: src/ModelingAlgorithms/TKOffset/GTests/BRepBuilderAPI_Sewing_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's BRepBuilderAPI_Sewing GTests (TKOffset's), on BRepPrimAPI_MakeBox's (frozen) box.</summary>
public class BRepBuilderAPI_SewingTest
{
    [Fact]
    public void SewTwoFaces()
    {
        var aBoxMaker = new MakeBox(10.0, 10.0, 10.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        var anExp = new Explorer(aBox, ShapeType.Face);
        Assert.True(anExp.More());
        Face aFace1 = ShapeCasts.Face(anExp.Current());
        anExp.Next();
        Assert.True(anExp.More());
        Face aFace2 = ShapeCasts.Face(anExp.Current());

        var aSewing = new Sewing();
        aSewing.Add(aFace1);
        aSewing.Add(aFace2);
        aSewing.Perform();

        Shape aResult = aSewing.SewedShape();
        Assert.False(aResult.IsNull);
    }

    [Fact]
    public void NbFreeEdges()
    {
        var aBoxMaker = new MakeBox(10.0, 10.0, 10.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        var anExp = new Explorer(aBox, ShapeType.Face);
        Assert.True(anExp.More());
        Face aFace1 = ShapeCasts.Face(anExp.Current());
        anExp.Next();
        Assert.True(anExp.More());
        Face aFace2 = ShapeCasts.Face(anExp.Current());

        var aSewing = new Sewing();
        aSewing.Add(aFace1);
        aSewing.Add(aFace2);
        aSewing.Perform();

        // Two faces from a box share one edge; the remaining edges are free.
        Assert.True(aSewing.FreeEdgeCount > 0);
    }

    [Fact]
    public void NbDegeneratedShapes()
    {
        var aBoxMaker = new MakeBox(10.0, 10.0, 10.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        var anExp = new Explorer(aBox, ShapeType.Face);
        Assert.True(anExp.More());
        Face aFace1 = ShapeCasts.Face(anExp.Current());
        anExp.Next();
        Assert.True(anExp.More());
        Face aFace2 = ShapeCasts.Face(anExp.Current());

        var aSewing = new Sewing();
        aSewing.Add(aFace1);
        aSewing.Add(aFace2);
        aSewing.Perform();

        // Planar faces should have no degenerated shapes.
        Assert.Equal(0, aSewing.DegeneratedShapeCount);
    }

    [Fact]
    public void SetTolerance()
    {
        var aSewing = new Sewing();
        const double aTol = 0.01;
        aSewing.SetTolerance(aTol);
        Assert.Equal(aTol, aSewing.Tolerance(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SewAllBoxFaces()
    {
        var aBoxMaker = new MakeBox(10.0, 10.0, 10.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        var aSewing = new Sewing();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aSewing.Add(anExp.Current());
        }
        aSewing.Perform();

        Shape aResult = aSewing.SewedShape();
        Assert.False(aResult.IsNull);

        // All 6 faces of a box form a closed shell, so there should be no free edges.
        Assert.Equal(0, aSewing.FreeEdgeCount);
    }
}

/// <summary>
/// Known answers of sewing and fast sewing: exploded faces sewn back, gaps within and beyond the tolerance, cutting of
/// a bound at a T-junction (sections and their boundaries), degenerated edges, frozen inputs never written and the
/// caller's own sub-shapes kept where sewing changed nothing, the internal (unfreezing) forms, and BRepBuilderAPI_FastSewing.
/// </summary>
public class BRepBuilderAPI_SewingTests
{
    private static int Count(in Shape theS, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theS, theType, aMap);
        return aMap.Count;
    }

    private static bool AllFrozen(in Shape theS)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theS, aMap);
        for (int i = 0; i < aMap.Count; i++)
        {
            if (!aMap[i].TShape()!.IsFrozen)
            {
                return false;
            }
        }
        return theS.TShape()!.IsFrozen;
    }

    // The faces of a box, each copied apart so that no two share an edge or a vertex.
    private static List<Shape> ExplodedBox(double theDx = 10.0)
    {
        Shape aBox = new MakeBox(theDx, theDx, theDx).Shape();
        var aFaces = new List<Shape>();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces.Add(new Copy(anExp.Current()).Shape());
        }
        return aFaces;
    }

    private static Face Rect(double theX0, double theX1, double theY0, double theY1) =>
        new MakeFace(new Plane3D(), theX0, theX1, theY0, theY1).Face();

    [Fact]
    public void Exploded_box_faces_are_sewn_into_a_closed_shell_and_the_inputs_are_not_written()
    {
        List<Shape> aFaces = ExplodedBox();
        var aSnaps = aFaces.Select(f => ShapeSnapshot.Of(f)).ToList();

        var aSewing = new Sewing(1.0e-6);
        foreach (Shape aF in aFaces)
        {
            aSewing.Add(aF);
        }
        aSewing.Perform();

        Shape aResult = aSewing.SewedShape();
        Assert.Equal(ShapeType.Shell, aResult.ShapeType());
        Assert.True(aResult.Closed());
        Assert.True(AllFrozen(aResult));
        Assert.Equal(6, Count(aResult, ShapeType.Face));
        Assert.Equal(12, Count(aResult, ShapeType.Edge));
        Assert.Equal(8, Count(aResult, ShapeType.Vertex));
        Assert.Equal(0, aSewing.FreeEdgeCount);
        Assert.Equal(12, aSewing.ContigousEdgeCount);
        Assert.Equal(0, aSewing.MultipleEdgeCount);
        Assert.True(new Analyzer(aResult).IsValid());

        for (int i = 0; i < aFaces.Count; i++)
        {
            aSnaps[i].AssertUnchanged(aFaces[i]);
            Assert.True(aSewing.IsModified(aFaces[i]));
            Shape aMod = aSewing.Modified(aFaces[i]);
            Assert.True(aMod.TShape()!.IsFrozen);
            // The face's surface is the caller's.
            Assert.Same(Tool.Surface(ShapeCasts.Face(aFaces[i])), Tool.Surface(ShapeCasts.Face(aMod)));
        }

        // Each contiguous edge has two sections, each a caller's edge, bound to it.
        for (int i = 1; i <= aSewing.ContigousEdgeCount; i++)
        {
            Assert.Equal(2, aSewing.ContigousEdgeCouple(i).Count);
            Edge anEdge = aSewing.ContigousEdge(i);
            Assert.True(aSewing.IsModifiedSubShape(ShapeCasts.Edge(aSewing.ContigousEdgeCouple(i)[0])));
            Assert.False(aSewing.WhichFace(anEdge).IsNull && aSewing.WhichFace(ShapeCasts.Edge(aSewing.ContigousEdgeCouple(i)[0])).IsNull);
        }
    }

    [Fact]
    public void Faces_already_sharing_their_edges_are_kept_as_the_callers()
    {
        Shape aBox = new MakeBox(10.0, 10.0, 10.0).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aSewing = new Sewing();
        var aFaces = new List<Shape>();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces.Add(anExp.Current());
            aSewing.Add(anExp.Current());
        }
        aSewing.Perform();

        Shape aResult = aSewing.SewedShape();
        Assert.Equal(ShapeType.Shell, aResult.ShapeType());
        aSnap.AssertUnchanged(aBox);
        foreach (Shape aF in aFaces)
        {
            Assert.False(aSewing.IsModified(aF));
            Assert.True(aSewing.Modified(aF).IsSame(aF));
        }
        // Every face of the result is one of the caller's faces.
        for (var anExp = new Explorer(aResult, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Shape aCurrent = anExp.Current();
            Assert.Contains(aFaces, f => f.IsSame(aCurrent));
        }
    }

    [Fact]
    public void A_gap_within_the_tolerance_is_sewn_and_one_beyond_it_is_left_free()
    {
        Face aF1 = Rect(0.0, 1.0, 0.0, 1.0);
        Face aF2 = Rect(1.0 + 1.0e-4, 2.0, 0.0, 1.0);

        var aSewing = new Sewing(1.0e-3);
        aSewing.Add(aF1);
        aSewing.Add(aF2);
        aSewing.Perform();
        Assert.Equal(ShapeType.Shell, aSewing.SewedShape().ShapeType());
        Assert.Equal(1, aSewing.ContigousEdgeCount);
        Assert.Equal(6, aSewing.FreeEdgeCount);
        // The vertices at the gap are merged: their tolerance covers half the gap.
        Edge aCommon = aSewing.ContigousEdge(1);
        Exploration.Vertices(aCommon, out Vertex aV1, out Vertex aV2);
        Assert.True(Tool.Tolerance(aV1) >= 0.5e-4);
        Assert.True(Tool.Tolerance(aV2) >= 0.5e-4);

        var aTight = new Sewing(1.0e-6);
        aTight.Add(aF1);
        aTight.Add(aF2);
        aTight.Perform();
        Assert.Equal(ShapeType.Compound, aTight.SewedShape().ShapeType());
        Assert.Equal(0, aTight.ContigousEdgeCount);
        Assert.Equal(8, aTight.FreeEdgeCount);
        // Nothing was sewn: the faces are the caller's.
        Assert.True(aTight.Modified(aF1).IsSame(aF1));
        Assert.True(aTight.FreeEdge(1).TShape()!.IsFrozen);
    }

    [Fact]
    public void A_bound_is_cut_at_a_T_junction_and_its_sections_know_their_boundary()
    {
        Face aA = Rect(0.0, 2.0, -1.0, 0.0);
        Face aB = Rect(0.0, 1.0, 0.0, 1.0);
        Face aC = Rect(1.0, 2.0, 0.0, 1.0);

        // The edge of A along y = 0.
        Edge aTop = new();
        for (var anExp = new Explorer(aA, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
            if (Math.Abs(Tool.Pnt(aV1).Y()) < 1e-9 && Math.Abs(Tool.Pnt(aV2).Y()) < 1e-9)
            {
                aTop = anE;
            }
        }
        Assert.False(aTop.IsNull);

        var aSewing = new Sewing(1.0e-6);
        aSewing.Add(aA);
        aSewing.Add(aB);
        aSewing.Add(aC);
        aSewing.Perform();

        Shape aResult = aSewing.SewedShape();
        Assert.Equal(ShapeType.Shell, aResult.ShapeType());
        Assert.Equal(3, Count(aResult, ShapeType.Face));
        Assert.Equal(3, aSewing.ContigousEdgeCount);
        Assert.Equal(7, aSewing.FreeEdgeCount);
        Assert.True(new Analyzer(aResult).IsValid());

        // OCCT: EdgeRegularity clears the merged edges before CreateOutputInformations reads them, so after sewing no
        // section is bound to its boundary (occt-issues.md, 6S).
        Assert.Equal(0, NbBoundSections(aSewing, aTop, aA));
        Assert.True(aSewing.IsModifiedSubShape(aTop));
        Assert.Equal(ShapeType.Wire, aSewing.ModifiedSubShape(aTop).ShapeType());

        // Analysis only (no sewn shape): the two contiguous edges along y = 0 each have a section of A's edge, bound
        // to it.
        var anAnalysis = new Sewing(1.0e-6, false);
        anAnalysis.Add(aA);
        anAnalysis.Add(aB);
        anAnalysis.Add(aC);
        anAnalysis.Perform();
        Assert.True(anAnalysis.SewedShape().IsNull);
        Assert.Equal(3, anAnalysis.ContigousEdgeCount);
        Assert.Equal(2, NbBoundSections(anAnalysis, aTop, aA));
    }

    private static int NbBoundSections(Sewing theSewing, in Edge theBound, in Face theFace)
    {
        int aNbBound = 0;
        for (int i = 1; i <= theSewing.ContigousEdgeCount; i++)
        {
            foreach (Shape aSec in theSewing.ContigousEdgeCouple(i))
            {
                Edge aSection = ShapeCasts.Edge(aSec);
                if (theSewing.IsSectionBound(aSection))
                {
                    aNbBound++;
                    Assert.True(theSewing.SectionToBoundary(aSection).IsSame(theBound));
                    Assert.True(theSewing.WhichFace(aSection).IsSame(theFace));
                }
            }
        }
        return aNbBound;
    }

    [Fact]
    public void The_degenerated_edge_of_a_cone_is_reported()
    {
        Shape aCone = new ApiMakeCone(2.0, 0.0, 3.0).Shape();
        var aSewing = new Sewing();
        for (var anExp = new Explorer(aCone, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aSewing.Add(new Copy(anExp.Current()).Shape());
        }
        aSewing.Perform();
        Assert.True(aSewing.DegeneratedShapeCount >= 1);
        Assert.True(Tool.Degenerated(ShapeCasts.Edge(aSewing.DegeneratedShape(1))));
        Assert.Equal(0, aSewing.FreeEdgeCount);
        Assert.True(new Analyzer(aSewing.SewedShape()).IsValid());
    }

    [Fact]
    public void Analysis_only_finds_free_edges_without_a_sewn_shape()
    {
        var aSewing = new Sewing(1.0e-6, false, false);
        foreach (Shape aF in ExplodedBox())
        {
            aSewing.Add(aF);
        }
        aSewing.Perform();
        Assert.True(aSewing.SewedShape().IsNull);
        Assert.Equal(0, aSewing.FreeEdgeCount);
        Assert.Equal(12, aSewing.ContigousEdgeCount);
        // An open box: the edges around the missing face are free.
        List<Shape> aFive = ExplodedBox();
        aFive.RemoveAt(0);
        var anOpen = new Sewing(1.0e-6, false, false);
        foreach (Shape aF in aFive)
        {
            anOpen.Add(aF);
        }
        anOpen.Perform();
        Assert.Equal(4, anOpen.FreeEdgeCount);
        Assert.Equal(8, anOpen.ContigousEdgeCount);
        for (int i = 1; i <= anOpen.FreeEdgeCount; i++)
        {
            Assert.True(anOpen.FreeEdge(i).TShape()!.IsFrozen);
        }
    }

    [Fact]
    public void Init_and_Load_clear_the_results_and_the_options_keep_their_defaults()
    {
        var aSewing = new Sewing(1.0e-8);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aSewing.Tolerance());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aSewing.MinTolerance());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aSewing.MaxTolerance());
        Assert.True(aSewing.FaceMode());
        Assert.False(aSewing.FloatingEdgesMode());
        Assert.True(aSewing.SameParameterMode());
        Assert.False(aSewing.LocalTolerancesMode());
        Assert.False(aSewing.NonManifoldMode());

        aSewing.Init(0.5, true, true, true, true);
        Assert.Equal(0.5, aSewing.Tolerance());
        Assert.Equal(0.5e-4, aSewing.MinTolerance());
        Assert.True(aSewing.NonManifoldMode());

        foreach (Shape aF in ExplodedBox())
        {
            aSewing.Add(aF);
        }
        aSewing.Perform();
        Assert.False(aSewing.SewedShape().IsNull);
        aSewing.Load(new Shape());
        Assert.True(aSewing.SewedShape().IsNull);
        Assert.Equal(0, aSewing.ContigousEdgeCount);

        var aWriter = new StringWriter();
        aSewing.Dump(aWriter);
        Assert.Contains("Number of input shapes      : 0", aWriter.ToString());
    }

    [Fact]
    public void A_sewn_shape_can_be_sewn_again()
    {
        var aSewing = new Sewing();
        foreach (Shape aF in ExplodedBox())
        {
            aSewing.Add(aF);
        }
        aSewing.Perform();
        Shape aShell = aSewing.SewedShape();

        var anAgain = new Sewing();
        anAgain.Add(aShell);
        anAgain.Perform();
        // A new shell (BRepTools_Quilt builds its shells), holding the same faces.
        Shape aResult = anAgain.SewedShape();
        Assert.Equal(ShapeType.Shell, aResult.ShapeType());
        Assert.False(anAgain.IsModified(aShell));
        var aFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aShell, ShapeType.Face, aFaces);
        for (var anExp = new Explorer(aResult, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Assert.True(aFaces.Contains(anExp.Current()));
        }
    }

    [Fact]
    public void Unfrozen_inputs_are_sewn_in_place_as_in_OCCT()
    {
        // Ported callers pass shapes they built themselves (unfrozen): no copy is made, and the internal forms hand
        // the result out without freezing it.
        var aFaces = new List<Shape>();
        var aMaker = new MakeFace(new Plane3D(), 0.0, 1.0, 0.0, 1.0);
        aFaces.Add(aMaker.InternalFace());
        aMaker = new MakeFace(new Plane3D(), 1.0, 2.0, 0.0, 1.0);
        aFaces.Add(aMaker.InternalFace());
        Assert.False(aFaces[0].TShape()!.IsFrozen);

        var aSewing = new Sewing();
        aSewing.Add(aFaces[0]);
        aSewing.Add(aFaces[1]);
        aSewing.Perform();
        Shape aResult = aSewing.InternalSewedShape();
        Assert.False(aResult.TShape()!.IsFrozen);
        Assert.Equal(1, aSewing.ContigousEdgeCount);
        Assert.False(aSewing.InternalFreeEdge(1).TShape()!.IsFrozen);
    }

    // ---------------------------------------------------------------- fast sewing

    private static BezierSurface Patch(double theX0, double theX1, double theY0, double theY1)
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(theX0, theY0, 0.0);
        aPoles[2, 1] = new Point3D(theX1, theY0, 0.0);
        aPoles[1, 2] = new Point3D(theX0, theY1, 0.0);
        aPoles[2, 2] = new Point3D(theX1, theY1, 0.0);
        return new BezierSurface(aPoles);
    }

    [Fact]
    public void FastSewing_joins_patches_along_their_common_boundary()
    {
        var aFast = new FastSewing(1.0e-6);
        Assert.True(aFast.Add(Patch(0.0, 1.0, 0.0, 1.0)));
        Assert.True(aFast.Add(Patch(1.0, 2.0, 0.0, 1.0)));
        Face aFace = new MakeFace(Patch(0.0, 1.0, 1.0, 2.0), 1.0e-6).Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        Assert.True(aFast.Add(aFace));
        aFast.Perform();
        Assert.Equal(0u, aFast.GetStatuses());

        Shape aResult = aFast.GetResult();
        Assert.True(AllFrozen(aResult));
        Assert.Equal(1, Count(aResult, ShapeType.Shell));
        Assert.Equal(3, Count(aResult, ShapeType.Face));
        Assert.Equal(10, Count(aResult, ShapeType.Edge));
        Assert.Equal(8, Count(aResult, ShapeType.Vertex));
        Assert.True(new Analyzer(aResult).IsValid());
        aSnap.AssertUnchanged(aFace);

        var aWriter = new StringWriter();
        aFast.GetStatuses(aWriter);
        Assert.Equal("Fast Sewing OK!\n", aWriter.ToString());
    }

    [Fact]
    public void FastSewing_reports_what_it_cannot_sew()
    {
        var aFast = new FastSewing();
        aFast.Perform();
        Assert.Equal((uint)FastSewing.FS_Statuses.FS_EmptyInput, aFast.GetStatuses());

        Assert.False(aFast.Add(new PlaneSurface(new Plane3D())));
        Assert.False(aFast.Add(new TrimmedSurface(new PlaneSurface(new Plane3D()), 0.0, 1.0, 0.0, 1.0)));
        Assert.False(aFast.Add((Surface?)null));
        uint aStatus = aFast.GetStatuses();
        Assert.NotEqual(0u, aStatus & (uint)FastSewing.FS_Statuses.FS_InfiniteSurface);
        Assert.NotEqual(0u, aStatus & (uint)FastSewing.FS_Statuses.FS_NotNaturalBoundsFace);
        Assert.NotEqual(0u, aStatus & (uint)FastSewing.FS_Statuses.FS_FaceWithNullSurface);

        var aWriter = new StringWriter();
        aFast.GetStatuses(aWriter);
        Assert.Contains("Source face has the surface with infinite boundaries.", aWriter.ToString());
        Assert.Contains("Empty source data.", aWriter.ToString());
    }

    [Fact]
    public void FastSewing_marks_degenerated_patches()
    {
        // A patch with two coincident corners has a degenerated edge.
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(1.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2, 2] = new Point3D(1.0, 1.0, 0.0);
        var aFast = new FastSewing();
        Assert.True(aFast.Add(new BezierSurface(aPoles)));
        aFast.Perform();
        Assert.NotEqual(0u, aFast.GetStatuses() & (uint)FastSewing.FS_Statuses.FS_Degenerated);
        Assert.Equal(1, Count(aFast.GetResult(), ShapeType.Face));
    }
}
