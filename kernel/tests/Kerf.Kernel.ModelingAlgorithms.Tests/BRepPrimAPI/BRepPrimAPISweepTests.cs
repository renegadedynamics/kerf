// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of the sweeps (BRepSweep through BRepPrimAPI_MakePrism and MakeRevol): the topology and the global
/// properties of prisms of vertices, edges, wires, faces and shells (finite, semi-infinite and infinite, with and without
/// copy and canonization) and of revolutions (whole and partial, profiles touching the axis, invalid profiles crossing
/// it); their histories (Generated, IsDeleted, FirstShape, LastShape); and frozen profiles, which are never changed and
/// whose unchanged sub-shapes the result shares.
/// </summary>
public class BRepPrimAPISweepTests
{
    private const double Tol = 1e-9;

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
        return true;
    }

    private static bool Contains(in Shape theShape, in Shape theSub)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        return aMap.Contains(theSub);
    }

    private static Wire Polygon(bool theClosed, params Point3D[] thePoints)
    {
        var aMP = new MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aMP.Add(aP);
        }
        if (theClosed)
        {
            aMP.Close();
        }
        return aMP.Wire();
    }

    private static Face Face(params Point3D[] thePoints) =>
        new MakeFace(Polygon(true, thePoints), true).Face();

    private static Face UnitSquare() =>
        Face(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0));

    private static double Volume(in Shape theS) => MassPropertiesBuilder.VolumeProperties(theS).Mass();

    private static double Area(in Shape theS) => MassPropertiesBuilder.SurfaceProperties(theS).Mass();

    private static bool IsValid(in Shape theS) => new Analyzer(theS).IsValid();

    // ------------------------------------------------------------ prisms

    [Fact]
    public void Prism_of_a_vertex_is_an_edge_of_the_vector_s_length()
    {
        Vertex aV = new MakeVertex(new Point3D(1, 2, 3)).Vertex();
        var aMP = new ApiMakePrism(aV, new Vector3D(0, 3, 4));
        Shape aS = aMP.Shape();
        Assert.Equal(ShapeType.Edge, aS.ShapeType());
        Assert.Equal(5.0, MassPropertiesBuilder.LinearProperties(aS).Mass(), Tol);
        Assert.True(aMP.FirstShape().IsSame(aV));
        Assert.True(Tool.Pnt(ShapeCasts.Vertex(aMP.LastShape())).IsEqual(new Point3D(1, 5, 7), Tol));
        Assert.True(AllFrozen(aS));
    }

    [Fact]
    public void Prism_of_an_edge_is_a_planar_face_and_of_an_open_wire_a_shell()
    {
        Edge anE = new MakeEdge(new Point3D(0, 0, 0), new Point3D(2, 0, 0)).Edge();
        Shape aF = new ApiMakePrism(anE, new Vector3D(0, 0, 3)).Shape();
        Assert.Equal(global::Kerf.Topology.ShapeType.Face, aF.ShapeType());
        Assert.Equal(6.0, Area(aF), Tol);
        Assert.IsType<PlaneSurface>(Tool.Surface(ShapeCasts.Face(aF)));
        Assert.Equal(4, Count(aF, ShapeType.Edge));

        Wire aW = Polygon(false, new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0));
        Shape aSh = new ApiMakePrism(aW, new Vector3D(0, 0, 2)).Shape();
        Assert.Equal(ShapeType.Shell, aSh.ShapeType());
        Assert.Equal(3, Count(aSh, global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(10, Count(aSh, ShapeType.Edge));
        Assert.Equal(8, Count(aSh, ShapeType.Vertex));
        Assert.Equal(6.0, Area(aSh), Tol);
        Assert.False(aSh.Closed());
    }

    [Fact]
    public void Prism_of_a_face_is_a_valid_solid()
    {
        Face aF = Face(new Point3D(0, 0, 0), new Point3D(2, 0, 0), new Point3D(2, 1, 0), new Point3D(0, 1, 0));
        var aMP = new ApiMakePrism(aF, new Vector3D(0, 0, 3));
        Shape aS = aMP.Shape();
        Assert.Equal(ShapeType.Solid, aS.ShapeType());
        Assert.Equal(6, Count(aS, global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(12, Count(aS, ShapeType.Edge));
        Assert.Equal(8, Count(aS, ShapeType.Vertex));
        Assert.Equal(6.0, Volume(aS), Tol);
        Assert.Equal(2 * (2 + 6 + 3), Area(aS), Tol);
        Assert.True(IsValid(aS));

        // downwards: the shell is oriented by DirectSolid, so the volume stays positive
        Shape aDown = new ApiMakePrism(aF, new Vector3D(0, 0, -3)).Shape();
        Assert.Equal(6.0, Volume(aDown), Tol);
        Assert.True(IsValid(aDown));
    }

    [Fact]
    public void Prism_of_a_disk_is_a_cylinder_canonized_or_an_extrusion()
    {
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        Wire aW = new MakeWire(new MakeEdge(aCirc).Edge()).Wire();
        Face aDisk = new MakeFace(aW, true).Face();

        Shape aCyl = new ApiMakePrism(aDisk, new Vector3D(0, 0, 5)).Shape();
        Assert.Equal(Math.PI * 4 * 5, Volume(aCyl), 1e-9);
        Assert.Equal(3, Count(aCyl, global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(3, Count(aCyl, ShapeType.Edge)); // two circles and the seam
        bool aHasCylinder = false;
        for (var anExp = new Explorer(aCyl, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            aHasCylinder |= Tool.Surface(ShapeCasts.Face(anExp.Current())) is CylindricalSurface;
        }
        Assert.True(aHasCylinder);
        Assert.True(IsValid(aCyl));

        Shape anExtr = new ApiMakePrism(aDisk, new Vector3D(0, 0, 5), false, false).Shape();
        bool aHasExtrusion = false;
        for (var anExp = new Explorer(anExtr, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            aHasExtrusion |= Tool.Surface(ShapeCasts.Face(anExp.Current())) is ExtrusionSurface;
        }
        Assert.True(aHasExtrusion);
        Assert.Equal(Math.PI * 4 * 5, Volume(anExtr), 1e-6);
    }

    [Fact]
    public void Prism_of_a_shell_is_a_compsolid()
    {
        // a shell of one face on a plane
        Shape aShell = new MakeShell(new PlaneSurface(new CoordinateSystem()), 0, 2, 0, 1).Shape();
        Shape aCS = new ApiMakePrism(aShell, new Vector3D(0, 0, 1)).Shape();
        Assert.Equal(ShapeType.CompSolid, aCS.ShapeType());
        Assert.Equal(1, Count(aCS, ShapeType.Solid));
        Assert.Equal(2.0, Volume(aCS), Tol);
    }

    [Fact]
    public void Semi_infinite_and_infinite_prisms()
    {
        Face aF = UnitSquare();
        var aSemi = new ApiMakePrism(aF, new Direction3D(0, 0, 1), false);
        Shape aS = aSemi.Shape();
        Assert.Equal(ShapeType.Solid, aS.ShapeType());
        Assert.Equal(5, Count(aS, global::Kerf.Topology.ShapeType.Face)); // the base and four half-infinite walls
        Assert.True(aSemi.FirstShape().IsSame(aF));
        Assert.True(aSemi.LastShape().IsNull);

        var anInf = new ApiMakePrism(aF, new Direction3D(0, 0, 1), true);
        Shape anI = anInf.Shape();
        Assert.Equal(4, Count(anI, global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(0, Count(anI, ShapeType.Vertex));
        Assert.True(anInf.FirstShape().IsNull);
        Assert.True(anInf.LastShape().IsNull);
    }

    [Fact]
    public void A_null_vector_throws()
    {
        Assert.ThrowsAny<ConstructionErrorException>(() => new ApiMakePrism(UnitSquare(), new Vector3D(0, 0, 0)));
    }

    [Fact]
    public void Prism_of_a_frozen_face_shares_its_unchanged_sub_shapes_and_leaves_it_unchanged()
    {
        Face aF = UnitSquare();
        Assert.True(aF.TShape()!.IsFrozen);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aF);
        var aMP = new ApiMakePrism(aF, new Vector3D(0, 0, 1));
        Shape aS = aMP.Shape();
        aSnap.AssertUnchanged(aF);
        Assert.Equal(1.0, Volume(aS), Tol);
        Assert.True(IsValid(aS));
        Assert.True(AllFrozen(aS));

        // the walls are planes, which need no pcurves: the result holds the face's own vertices and edges
        for (var anExp = new Explorer(aF, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Assert.True(Contains(aS, anExp.Current()));
            Assert.False(aMP.IsDeleted(anExp.Current()));
            Shape aGen = Assert.Single(aMP.Generated(anExp.Current()));
            Assert.Equal(ShapeType.Edge, aGen.ShapeType());
            Assert.True(Contains(aS, aGen));
            Assert.True(aMP.FirstShape(anExp.Current()).IsSame(anExp.Current()));
        }
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(Contains(aS, anExp.Current()));
            Shape aFirst = aMP.FirstShape(anExp.Current());
            Assert.Equal(ShapeType.Edge, aFirst.ShapeType());
            Assert.True(Contains(aS, aFirst));
            Assert.True(Contains(aS, aMP.LastShape(anExp.Current())));
            Shape aWall = Assert.Single(aMP.Generated(anExp.Current()));
            Assert.Equal(global::Kerf.Topology.ShapeType.Face, aWall.ShapeType());
            Assert.True(Contains(aS, aWall));
        }
        Shape aBottom = aMP.FirstShape();
        Assert.Equal(global::Kerf.Topology.ShapeType.Face, aBottom.ShapeType());
        Assert.True(Contains(aS, aBottom));
        Assert.Equal(ShapeType.Solid, Assert.Single(aMP.Generated(aF)).ShapeType());

        // a copy shares nothing with the base
        var aCopied = new ApiMakePrism(aF, new Vector3D(0, 0, 1), true);
        for (var anExp = new Explorer(aF, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Assert.False(Contains(aCopied.Shape(), anExp.Current()));
        }
        aSnap.AssertUnchanged(aF);
        Assert.Equal(1.0, Volume(aCopied.Shape()), Tol);
    }

    [Fact]
    public void Prism_of_a_frozen_edge_keeps_the_edge_when_only_its_vertices_change_nothing()
    {
        // an edge without pcurves: the planar wall gets none (planes need none), so the edge itself is unchanged
        Edge anE = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        var aMP = new ApiMakePrism(anE, new Vector3D(0, 1, 0));
        Shape aS = aMP.Shape();
        aSnap.AssertUnchanged(anE);
        Assert.True(Contains(aS, anE));
        Assert.True(aMP.FirstShape().IsSame(anE));
        Assert.Equal(1.0, Area(aS), Tol);
    }

    [Fact]
    public void Prism_of_a_frozen_disk_gives_back_a_changed_copy_as_its_first_shape()
    {
        // decision 1102: natively FirstShape() IsSame the caller's disk, whose circle gets its pcurve on the wall in
        // place; the caller's frozen disk can't take it, so the bottom face is a copy holding a changed copy of the
        // circle, and FirstShape(disk) and the history answer for the caller's disk with it
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        Face aDisk = new MakeFace(new MakeWire(new MakeEdge(aCirc).Edge()).Wire(), true).Face();
        Assert.True(aDisk.TShape()!.IsFrozen);
        var aMP = new ApiMakePrism(aDisk, new Vector3D(0, 0, 1));
        Shape aS = aMP.Shape();
        Shape aBottom = aMP.FirstShape();
        Assert.False(aBottom.IsSame(aDisk));
        Assert.True(aMP.FirstShape(aDisk).IsSame(aBottom));
        Assert.True(Contains(aS, aBottom));
        Edge aCircle = ShapeCasts.Edge(new Explorer(aDisk, ShapeType.Edge).Current());
        Assert.False(Contains(aS, aCircle));
        Edge aCopy = ShapeCasts.Edge(aMP.FirstShape(aCircle));
        Assert.True(Contains(aS, aCopy));
        Face aWall = ShapeCasts.Face(Assert.Single(aMP.Generated(aCircle)));
        Assert.Null(Tool.CurveOnSurface(aCircle, aWall, out _, out _));
        Assert.NotNull(Tool.CurveOnSurface(aCopy, aWall, out _, out _));
    }

    [Fact]
    public void An_unfrozen_profile_is_swept_in_place_as_OCCT_does()
    {
        // ported code passing its own unfrozen shape: no copy, the result holds the profile's own edges, which get
        // their pcurves on the walls in place
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        var aMW = new MakeWire(new MakeEdge(aCirc).InternalEdge());
        Face aDisk = new MakeFace(aMW.InternalWire(), true).InternalFace();
        Assert.False(aDisk.TShape()!.IsFrozen);
        var aMP = new ApiMakePrism(aDisk, new Vector3D(0, 0, 1));
        Shape aS = aMP.Shape();
        Assert.True(aMP.FirstShape().IsSame(aDisk));
        Edge aCircle = ShapeCasts.Edge(new Explorer(aDisk, ShapeType.Edge).Current());
        Assert.True(Contains(aS, aCircle));
        Face aWall = ShapeCasts.Face(Assert.Single(aMP.Generated(aCircle)));
        Assert.NotNull(Tool.CurveOnSurface(aCircle, aWall, out _, out _));
        Assert.Equal(Math.PI, Volume(aS), 1e-9);
    }

    // ------------------------------------------------------------ revolutions

    private static readonly Axis3D OZ = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));

    private static Face XZRectangle(double x0, double x1, double z0, double z1) =>
        Face(new Point3D(x0, 0, z0), new Point3D(x1, 0, z0), new Point3D(x1, 0, z1), new Point3D(x0, 0, z1));

    [Fact]
    public void Revolution_of_a_rectangle_off_the_axis_is_a_tube()
    {
        Face aF = XZRectangle(1, 2, 0, 1);
        var aMR = new ApiMakeRevol(aF, OZ);
        Shape aS = aMR.Shape();
        Assert.Equal(ShapeType.Solid, aS.ShapeType());
        Assert.Equal(Math.PI * (4 - 1) * 1, Volume(aS), 1e-9); // Pappus
        Assert.Equal(4, Count(aS, global::Kerf.Topology.ShapeType.Face));
        Assert.True(IsValid(aS));
        Assert.False(aMR.HasDegenerated);
        // whole turn: the first and last shapes are the generating face itself
        Assert.True(aMR.FirstShape().IsSame(aMR.LastShape()));

        // half a turn: the two caps close it
        var aHalf = new ApiMakeRevol(aF, OZ, Math.PI);
        Assert.Equal(Math.PI * 3 / 2, Volume(aHalf.Shape()), 1e-9);
        Assert.Equal(6, Count(aHalf.Shape(), global::Kerf.Topology.ShapeType.Face));
        Assert.True(IsValid(aHalf.Shape()));

        // a negative angle turns the other way, the same volume
        var aNeg = new ApiMakeRevol(aF, OZ, -Math.PI / 2);
        Assert.Equal(Math.PI * 3 / 4, Volume(aNeg.Shape()), 1e-9);
    }

    [Fact]
    public void Revolution_of_a_rectangle_on_the_axis_is_a_cylinder()
    {
        Face aF = XZRectangle(0, 1, 0, 2);
        var aMR = new ApiMakeRevol(aF, OZ);
        Shape aS = aMR.Shape();
        Assert.Equal(Math.PI * 2, Volume(aS), 1e-9);
        Assert.Equal(3, Count(aS, global::Kerf.Topology.ShapeType.Face)); // the edge on the axis generates nothing
        Assert.True(IsValid(aS));

        // a quarter: two planar caps
        var aQ = new ApiMakeRevol(aF, OZ, Math.PI / 2);
        Assert.Equal(Math.PI / 2, Volume(aQ.Shape()), 1e-9);
        Assert.Equal(5, Count(aQ.Shape(), global::Kerf.Topology.ShapeType.Face));
        Assert.True(IsValid(aQ.Shape()));
    }

    [Fact]
    public void Revolution_of_a_triangle_with_a_vertex_on_the_axis_is_a_cone_with_degenerated_edges()
    {
        // apex on the axis at the top, base edge on the XY plane from the axis
        Face aF = Face(new Point3D(0, 0, 0), new Point3D(2, 0, 0), new Point3D(0, 0, 3));
        var aMR = new ApiMakeRevol(aF, OZ);
        Shape aS = aMR.Shape();
        Assert.Equal(Math.PI * 4 * 3 / 3, Volume(aS), 1e-9);
        Assert.True(aMR.HasDegenerated);
        Assert.True(IsValid(aS));

        // a partial turn of a triangle off the base: the degenerated edge at the apex is shared by the cone and both
        // caps' boundaries, and is copied for each face but the first
        var aPart = new ApiMakeRevol(aF, OZ, 1.0);
        Assert.Equal(Math.PI * 4 * 3 / 3 / (2 * Math.PI), Volume(aPart.Shape()), 1e-9);
        Assert.True(aPart.HasDegenerated);
        Assert.True(IsValid(aPart.Shape()));
        foreach (Shape aD in aPart.Degenerated())
        {
            Assert.True(Tool.Degenerated(ShapeCasts.Edge(aD)));
            Assert.True(aD.TShape()!.IsFrozen);
        }
    }

    [Fact]
    public void Revolution_of_a_disk_off_the_axis_is_a_torus()
    {
        var aCirc = new Circle3D(new Frame3D(new Point3D(3, 0, 0), new Direction3D(0, 1, 0)), 1.0);
        Wire aW = new MakeWire(new MakeEdge(aCirc).Edge()).Wire();
        Face aDisk = new MakeFace(aW, true).Face();
        Shape aS = new ApiMakeRevol(aDisk, OZ).Shape();
        Assert.Equal(2 * Math.PI * Math.PI * 3 * 1, Volume(aS), 1e-6);
        bool aHasTorus = false;
        for (var anExp = new Explorer(aS, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            aHasTorus |= Tool.Surface(ShapeCasts.Face(anExp.Current())) is ToroidalSurface;
        }
        Assert.True(aHasTorus);
        Assert.True(IsValid(aS));
    }

    [Fact]
    public void Revolutions_of_a_vertex_and_an_edge()
    {
        Vertex aV = new MakeVertex(new Point3D(2, 0, 1)).Vertex();
        Shape aCircle = new ApiMakeRevol(aV, OZ).Shape();
        Assert.Equal(ShapeType.Edge, aCircle.ShapeType());
        Assert.Equal(4 * Math.PI, MassPropertiesBuilder.LinearProperties(aCircle).Mass(), 1e-9);

        Edge anE = new MakeEdge(new Point3D(1, 0, 0), new Point3D(1, 0, 2)).Edge();
        var aMR = new ApiMakeRevol(anE, OZ, Math.PI);
        Shape aF = aMR.Shape();
        Assert.Equal(global::Kerf.Topology.ShapeType.Face, aF.ShapeType());
        Assert.Equal(2 * Math.PI, Area(aF), 1e-9);
        Assert.IsType<CylindricalSurface>(Tool.Surface(ShapeCasts.Face(aF)));
        Exploration.Vertices(anE, out Vertex aV1, out Vertex _);
        Shape anArc = Assert.Single(aMR.Generated(aV1));
        Assert.Equal(ShapeType.Edge, anArc.ShapeType());
        // the edge gets a pcurve on the cylinder: the result holds a changed copy of the frozen edge
        Shape aFirst = aMR.FirstShape();
        Assert.Equal(ShapeType.Edge, aFirst.ShapeType());
        Assert.False(aFirst.IsSame(anE));
        Assert.True(Contains(aF, aFirst));
        Assert.True(aMR.FirstShape(anE).IsSame(aFirst));
        Assert.Equal(ShapeType.Edge, aMR.LastShape().ShapeType());
        Assert.False(aMR.LastShape().IsSame(aFirst));
    }

    [Fact]
    public void Revolution_of_an_edge_crossing_the_axis_is_null()
    {
        Edge anE = new MakeEdge(new Point3D(-1, 0, 1), new Point3D(1, 0, 1)).Edge();
        var aMR = new ApiMakeRevol(anE, OZ);
        Assert.False(aMR.IsDone);
        Assert.ThrowsAny<NotDoneException>(() => aMR.Shape());
    }

    [Fact]
    public void Revolution_of_a_frozen_face_leaves_it_unchanged_and_answers_its_history()
    {
        Face aF = XZRectangle(1, 2, 0, 1);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aF);
        var aMR = new ApiMakeRevol(aF, OZ, Math.PI / 3);
        Shape aS = aMR.Shape();
        aSnap.AssertUnchanged(aF);
        Assert.True(AllFrozen(aS));
        Assert.True(IsValid(aS));
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.False(aMR.IsDeleted(anExp.Current()));
            Shape aGen = Assert.Single(aMR.Generated(anExp.Current()));
            Assert.Equal(global::Kerf.Topology.ShapeType.Face, aGen.ShapeType());
            Assert.True(Contains(aS, aGen));
            Assert.True(Contains(aS, aMR.FirstShape(anExp.Current())));
            Assert.True(Contains(aS, aMR.LastShape(anExp.Current())));
        }
        for (var anExp = new Explorer(aF, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Assert.True(Contains(aS, anExp.Current())); // unchanged: the face's own
        }
        Assert.Equal(Math.PI * 3 / 6, Volume(aS), 1e-9);
    }
}
