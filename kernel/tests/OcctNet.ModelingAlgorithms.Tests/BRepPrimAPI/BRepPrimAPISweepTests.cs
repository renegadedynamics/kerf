// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

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

    private static int Count(in TopoDS_Shape theS, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theS, theType, aMap);
        return aMap.Extent();
    }

    private static bool AllFrozen(in TopoDS_Shape theS)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theS, aMap);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            if (!aMap[i].TShape()!.IsFrozen())
            {
                return false;
            }
        }
        return true;
    }

    private static bool Contains(in TopoDS_Shape theShape, in TopoDS_Shape theSub)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        return aMap.Contains(theSub);
    }

    private static TopoDS_Wire Polygon(bool theClosed, params gp_Pnt[] thePoints)
    {
        var aMP = new BRepBuilderAPI_MakePolygon();
        foreach (gp_Pnt aP in thePoints)
        {
            aMP.Add(aP);
        }
        if (theClosed)
        {
            aMP.Close();
        }
        return aMP.Wire();
    }

    private static TopoDS_Face Face(params gp_Pnt[] thePoints) =>
        new BRepBuilderAPI_MakeFace(Polygon(true, thePoints), true).Face();

    private static TopoDS_Face UnitSquare() =>
        Face(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0));

    private static double Volume(in TopoDS_Shape theS) => BRepGProp.VolumeProperties(theS).Mass();

    private static double Area(in TopoDS_Shape theS) => BRepGProp.SurfaceProperties(theS).Mass();

    private static bool IsValid(in TopoDS_Shape theS) => new BRepCheck_Analyzer(theS).IsValid();

    // ------------------------------------------------------------ prisms

    [Fact]
    public void Prism_of_a_vertex_is_an_edge_of_the_vector_s_length()
    {
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(1, 2, 3)).Vertex();
        var aMP = new BRepPrimAPI_MakePrism(aV, new gp_Vec(0, 3, 4));
        TopoDS_Shape aS = aMP.Shape();
        Assert.Equal(TopAbs_EDGE, aS.ShapeType());
        Assert.Equal(5.0, BRepGProp.LinearProperties(aS).Mass(), Tol);
        Assert.True(aMP.FirstShape().IsSame(aV));
        Assert.True(BRep_Tool.Pnt(TopoDS.Vertex(aMP.LastShape())).IsEqual(new gp_Pnt(1, 5, 7), Tol));
        Assert.True(AllFrozen(aS));
    }

    [Fact]
    public void Prism_of_an_edge_is_a_planar_face_and_of_an_open_wire_a_shell()
    {
        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(2, 0, 0)).Edge();
        TopoDS_Shape aF = new BRepPrimAPI_MakePrism(anE, new gp_Vec(0, 0, 3)).Shape();
        Assert.Equal(TopAbs_FACE, aF.ShapeType());
        Assert.Equal(6.0, Area(aF), Tol);
        Assert.IsType<Geom_Plane>(BRep_Tool.Surface(TopoDS.Face(aF)));
        Assert.Equal(4, Count(aF, TopAbs_EDGE));

        TopoDS_Wire aW = Polygon(false, new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0));
        TopoDS_Shape aSh = new BRepPrimAPI_MakePrism(aW, new gp_Vec(0, 0, 2)).Shape();
        Assert.Equal(TopAbs_SHELL, aSh.ShapeType());
        Assert.Equal(3, Count(aSh, TopAbs_FACE));
        Assert.Equal(10, Count(aSh, TopAbs_EDGE));
        Assert.Equal(8, Count(aSh, TopAbs_VERTEX));
        Assert.Equal(6.0, Area(aSh), Tol);
        Assert.False(aSh.Closed());
    }

    [Fact]
    public void Prism_of_a_face_is_a_valid_solid()
    {
        TopoDS_Face aF = Face(new gp_Pnt(0, 0, 0), new gp_Pnt(2, 0, 0), new gp_Pnt(2, 1, 0), new gp_Pnt(0, 1, 0));
        var aMP = new BRepPrimAPI_MakePrism(aF, new gp_Vec(0, 0, 3));
        TopoDS_Shape aS = aMP.Shape();
        Assert.Equal(TopAbs_SOLID, aS.ShapeType());
        Assert.Equal(6, Count(aS, TopAbs_FACE));
        Assert.Equal(12, Count(aS, TopAbs_EDGE));
        Assert.Equal(8, Count(aS, TopAbs_VERTEX));
        Assert.Equal(6.0, Volume(aS), Tol);
        Assert.Equal(2 * (2 + 6 + 3), Area(aS), Tol);
        Assert.True(IsValid(aS));

        // downwards: the shell is oriented by DirectSolid, so the volume stays positive
        TopoDS_Shape aDown = new BRepPrimAPI_MakePrism(aF, new gp_Vec(0, 0, -3)).Shape();
        Assert.Equal(6.0, Volume(aDown), Tol);
        Assert.True(IsValid(aDown));
    }

    [Fact]
    public void Prism_of_a_disk_is_a_cylinder_canonized_or_an_extrusion()
    {
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        TopoDS_Wire aW = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aCirc).Edge()).Wire();
        TopoDS_Face aDisk = new BRepBuilderAPI_MakeFace(aW, true).Face();

        TopoDS_Shape aCyl = new BRepPrimAPI_MakePrism(aDisk, new gp_Vec(0, 0, 5)).Shape();
        Assert.Equal(Math.PI * 4 * 5, Volume(aCyl), 1e-9);
        Assert.Equal(3, Count(aCyl, TopAbs_FACE));
        Assert.Equal(3, Count(aCyl, TopAbs_EDGE)); // two circles and the seam
        bool aHasCylinder = false;
        for (var anExp = new TopExp_Explorer(aCyl, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aHasCylinder |= BRep_Tool.Surface(TopoDS.Face(anExp.Current())) is Geom_CylindricalSurface;
        }
        Assert.True(aHasCylinder);
        Assert.True(IsValid(aCyl));

        TopoDS_Shape anExtr = new BRepPrimAPI_MakePrism(aDisk, new gp_Vec(0, 0, 5), false, false).Shape();
        bool aHasExtrusion = false;
        for (var anExp = new TopExp_Explorer(anExtr, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aHasExtrusion |= BRep_Tool.Surface(TopoDS.Face(anExp.Current())) is Geom_SurfaceOfLinearExtrusion;
        }
        Assert.True(aHasExtrusion);
        Assert.Equal(Math.PI * 4 * 5, Volume(anExtr), 1e-6);
    }

    [Fact]
    public void Prism_of_a_shell_is_a_compsolid()
    {
        // a shell of one face on a plane
        TopoDS_Shape aShell = new BRepBuilderAPI_MakeShell(new Geom_Plane(new gp_Ax3()), 0, 2, 0, 1).Shape();
        TopoDS_Shape aCS = new BRepPrimAPI_MakePrism(aShell, new gp_Vec(0, 0, 1)).Shape();
        Assert.Equal(TopAbs_COMPSOLID, aCS.ShapeType());
        Assert.Equal(1, Count(aCS, TopAbs_SOLID));
        Assert.Equal(2.0, Volume(aCS), Tol);
    }

    [Fact]
    public void Semi_infinite_and_infinite_prisms()
    {
        TopoDS_Face aF = UnitSquare();
        var aSemi = new BRepPrimAPI_MakePrism(aF, new gp_Dir(0, 0, 1), false);
        TopoDS_Shape aS = aSemi.Shape();
        Assert.Equal(TopAbs_SOLID, aS.ShapeType());
        Assert.Equal(5, Count(aS, TopAbs_FACE)); // the base and four half-infinite walls
        Assert.True(aSemi.FirstShape().IsSame(aF));
        Assert.True(aSemi.LastShape().IsNull());

        var anInf = new BRepPrimAPI_MakePrism(aF, new gp_Dir(0, 0, 1), true);
        TopoDS_Shape anI = anInf.Shape();
        Assert.Equal(4, Count(anI, TopAbs_FACE));
        Assert.Equal(0, Count(anI, TopAbs_VERTEX));
        Assert.True(anInf.FirstShape().IsNull());
        Assert.True(anInf.LastShape().IsNull());
    }

    [Fact]
    public void A_null_vector_throws()
    {
        Assert.ThrowsAny<Standard_ConstructionError>(() => new BRepPrimAPI_MakePrism(UnitSquare(), new gp_Vec(0, 0, 0)));
    }

    [Fact]
    public void Prism_of_a_frozen_face_shares_its_unchanged_sub_shapes_and_leaves_it_unchanged()
    {
        TopoDS_Face aF = UnitSquare();
        Assert.True(aF.TShape()!.IsFrozen());
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aF);
        var aMP = new BRepPrimAPI_MakePrism(aF, new gp_Vec(0, 0, 1));
        TopoDS_Shape aS = aMP.Shape();
        aSnap.AssertUnchanged(aF);
        Assert.Equal(1.0, Volume(aS), Tol);
        Assert.True(IsValid(aS));
        Assert.True(AllFrozen(aS));

        // the walls are planes, which need no pcurves: the result holds the face's own vertices and edges
        for (var anExp = new TopExp_Explorer(aF, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            Assert.True(Contains(aS, anExp.Current()));
            Assert.False(aMP.IsDeleted(anExp.Current()));
            TopoDS_Shape aGen = Assert.Single(aMP.Generated(anExp.Current()));
            Assert.Equal(TopAbs_EDGE, aGen.ShapeType());
            Assert.True(Contains(aS, aGen));
            Assert.True(aMP.FirstShape(anExp.Current()).IsSame(anExp.Current()));
        }
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(Contains(aS, anExp.Current()));
            TopoDS_Shape aFirst = aMP.FirstShape(anExp.Current());
            Assert.Equal(TopAbs_EDGE, aFirst.ShapeType());
            Assert.True(Contains(aS, aFirst));
            Assert.True(Contains(aS, aMP.LastShape(anExp.Current())));
            TopoDS_Shape aWall = Assert.Single(aMP.Generated(anExp.Current()));
            Assert.Equal(TopAbs_FACE, aWall.ShapeType());
            Assert.True(Contains(aS, aWall));
        }
        TopoDS_Shape aBottom = aMP.FirstShape();
        Assert.Equal(TopAbs_FACE, aBottom.ShapeType());
        Assert.True(Contains(aS, aBottom));
        Assert.Equal(TopAbs_SOLID, Assert.Single(aMP.Generated(aF)).ShapeType());

        // a copy shares nothing with the base
        var aCopied = new BRepPrimAPI_MakePrism(aF, new gp_Vec(0, 0, 1), true);
        for (var anExp = new TopExp_Explorer(aF, TopAbs_VERTEX); anExp.More(); anExp.Next())
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
        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Edge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        var aMP = new BRepPrimAPI_MakePrism(anE, new gp_Vec(0, 1, 0));
        TopoDS_Shape aS = aMP.Shape();
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
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
        TopoDS_Face aDisk = new BRepBuilderAPI_MakeFace(new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aCirc).Edge()).Wire(), true).Face();
        Assert.True(aDisk.TShape()!.IsFrozen());
        var aMP = new BRepPrimAPI_MakePrism(aDisk, new gp_Vec(0, 0, 1));
        TopoDS_Shape aS = aMP.Shape();
        TopoDS_Shape aBottom = aMP.FirstShape();
        Assert.False(aBottom.IsSame(aDisk));
        Assert.True(aMP.FirstShape(aDisk).IsSame(aBottom));
        Assert.True(Contains(aS, aBottom));
        TopoDS_Edge aCircle = TopoDS.Edge(new TopExp_Explorer(aDisk, TopAbs_EDGE).Current());
        Assert.False(Contains(aS, aCircle));
        TopoDS_Edge aCopy = TopoDS.Edge(aMP.FirstShape(aCircle));
        Assert.True(Contains(aS, aCopy));
        TopoDS_Face aWall = TopoDS.Face(Assert.Single(aMP.Generated(aCircle)));
        Assert.Null(BRep_Tool.CurveOnSurface(aCircle, aWall, out _, out _));
        Assert.NotNull(BRep_Tool.CurveOnSurface(aCopy, aWall, out _, out _));
    }

    [Fact]
    public void An_unfrozen_profile_is_swept_in_place_as_OCCT_does()
    {
        // ported code passing its own unfrozen shape: no copy, the result holds the profile's own edges, which get
        // their pcurves on the walls in place
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
        var aMW = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aCirc).InternalEdge());
        TopoDS_Face aDisk = new BRepBuilderAPI_MakeFace(aMW.InternalWire(), true).InternalFace();
        Assert.False(aDisk.TShape()!.IsFrozen());
        var aMP = new BRepPrimAPI_MakePrism(aDisk, new gp_Vec(0, 0, 1));
        TopoDS_Shape aS = aMP.Shape();
        Assert.True(aMP.FirstShape().IsSame(aDisk));
        TopoDS_Edge aCircle = TopoDS.Edge(new TopExp_Explorer(aDisk, TopAbs_EDGE).Current());
        Assert.True(Contains(aS, aCircle));
        TopoDS_Face aWall = TopoDS.Face(Assert.Single(aMP.Generated(aCircle)));
        Assert.NotNull(BRep_Tool.CurveOnSurface(aCircle, aWall, out _, out _));
        Assert.Equal(Math.PI, Volume(aS), 1e-9);
    }

    // ------------------------------------------------------------ revolutions

    private static readonly gp_Ax1 OZ = new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));

    private static TopoDS_Face XZRectangle(double x0, double x1, double z0, double z1) =>
        Face(new gp_Pnt(x0, 0, z0), new gp_Pnt(x1, 0, z0), new gp_Pnt(x1, 0, z1), new gp_Pnt(x0, 0, z1));

    [Fact]
    public void Revolution_of_a_rectangle_off_the_axis_is_a_tube()
    {
        TopoDS_Face aF = XZRectangle(1, 2, 0, 1);
        var aMR = new BRepPrimAPI_MakeRevol(aF, OZ);
        TopoDS_Shape aS = aMR.Shape();
        Assert.Equal(TopAbs_SOLID, aS.ShapeType());
        Assert.Equal(Math.PI * (4 - 1) * 1, Volume(aS), 1e-9); // Pappus
        Assert.Equal(4, Count(aS, TopAbs_FACE));
        Assert.True(IsValid(aS));
        Assert.False(aMR.HasDegenerated());
        // whole turn: the first and last shapes are the generating face itself
        Assert.True(aMR.FirstShape().IsSame(aMR.LastShape()));

        // half a turn: the two caps close it
        var aHalf = new BRepPrimAPI_MakeRevol(aF, OZ, Math.PI);
        Assert.Equal(Math.PI * 3 / 2, Volume(aHalf.Shape()), 1e-9);
        Assert.Equal(6, Count(aHalf.Shape(), TopAbs_FACE));
        Assert.True(IsValid(aHalf.Shape()));

        // a negative angle turns the other way, the same volume
        var aNeg = new BRepPrimAPI_MakeRevol(aF, OZ, -Math.PI / 2);
        Assert.Equal(Math.PI * 3 / 4, Volume(aNeg.Shape()), 1e-9);
    }

    [Fact]
    public void Revolution_of_a_rectangle_on_the_axis_is_a_cylinder()
    {
        TopoDS_Face aF = XZRectangle(0, 1, 0, 2);
        var aMR = new BRepPrimAPI_MakeRevol(aF, OZ);
        TopoDS_Shape aS = aMR.Shape();
        Assert.Equal(Math.PI * 2, Volume(aS), 1e-9);
        Assert.Equal(3, Count(aS, TopAbs_FACE)); // the edge on the axis generates nothing
        Assert.True(IsValid(aS));

        // a quarter: two planar caps
        var aQ = new BRepPrimAPI_MakeRevol(aF, OZ, Math.PI / 2);
        Assert.Equal(Math.PI / 2, Volume(aQ.Shape()), 1e-9);
        Assert.Equal(5, Count(aQ.Shape(), TopAbs_FACE));
        Assert.True(IsValid(aQ.Shape()));
    }

    [Fact]
    public void Revolution_of_a_triangle_with_a_vertex_on_the_axis_is_a_cone_with_degenerated_edges()
    {
        // apex on the axis at the top, base edge on the XY plane from the axis
        TopoDS_Face aF = Face(new gp_Pnt(0, 0, 0), new gp_Pnt(2, 0, 0), new gp_Pnt(0, 0, 3));
        var aMR = new BRepPrimAPI_MakeRevol(aF, OZ);
        TopoDS_Shape aS = aMR.Shape();
        Assert.Equal(Math.PI * 4 * 3 / 3, Volume(aS), 1e-9);
        Assert.True(aMR.HasDegenerated());
        Assert.True(IsValid(aS));

        // a partial turn of a triangle off the base: the degenerated edge at the apex is shared by the cone and both
        // caps' boundaries, and is copied for each face but the first
        var aPart = new BRepPrimAPI_MakeRevol(aF, OZ, 1.0);
        Assert.Equal(Math.PI * 4 * 3 / 3 / (2 * Math.PI), Volume(aPart.Shape()), 1e-9);
        Assert.True(aPart.HasDegenerated());
        Assert.True(IsValid(aPart.Shape()));
        foreach (TopoDS_Shape aD in aPart.Degenerated())
        {
            Assert.True(BRep_Tool.Degenerated(TopoDS.Edge(aD)));
            Assert.True(aD.TShape()!.IsFrozen());
        }
    }

    [Fact]
    public void Revolution_of_a_disk_off_the_axis_is_a_torus()
    {
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(3, 0, 0), new gp_Dir(0, 1, 0)), 1.0);
        TopoDS_Wire aW = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aCirc).Edge()).Wire();
        TopoDS_Face aDisk = new BRepBuilderAPI_MakeFace(aW, true).Face();
        TopoDS_Shape aS = new BRepPrimAPI_MakeRevol(aDisk, OZ).Shape();
        Assert.Equal(2 * Math.PI * Math.PI * 3 * 1, Volume(aS), 1e-6);
        bool aHasTorus = false;
        for (var anExp = new TopExp_Explorer(aS, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aHasTorus |= BRep_Tool.Surface(TopoDS.Face(anExp.Current())) is Geom_ToroidalSurface;
        }
        Assert.True(aHasTorus);
        Assert.True(IsValid(aS));
    }

    [Fact]
    public void Revolutions_of_a_vertex_and_an_edge()
    {
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(2, 0, 1)).Vertex();
        TopoDS_Shape aCircle = new BRepPrimAPI_MakeRevol(aV, OZ).Shape();
        Assert.Equal(TopAbs_EDGE, aCircle.ShapeType());
        Assert.Equal(4 * Math.PI, BRepGProp.LinearProperties(aCircle).Mass(), 1e-9);

        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(new gp_Pnt(1, 0, 0), new gp_Pnt(1, 0, 2)).Edge();
        var aMR = new BRepPrimAPI_MakeRevol(anE, OZ, Math.PI);
        TopoDS_Shape aF = aMR.Shape();
        Assert.Equal(TopAbs_FACE, aF.ShapeType());
        Assert.Equal(2 * Math.PI, Area(aF), 1e-9);
        Assert.IsType<Geom_CylindricalSurface>(BRep_Tool.Surface(TopoDS.Face(aF)));
        TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex _);
        TopoDS_Shape anArc = Assert.Single(aMR.Generated(aV1));
        Assert.Equal(TopAbs_EDGE, anArc.ShapeType());
        // the edge gets a pcurve on the cylinder: the result holds a changed copy of the frozen edge
        TopoDS_Shape aFirst = aMR.FirstShape();
        Assert.Equal(TopAbs_EDGE, aFirst.ShapeType());
        Assert.False(aFirst.IsSame(anE));
        Assert.True(Contains(aF, aFirst));
        Assert.True(aMR.FirstShape(anE).IsSame(aFirst));
        Assert.Equal(TopAbs_EDGE, aMR.LastShape().ShapeType());
        Assert.False(aMR.LastShape().IsSame(aFirst));
    }

    [Fact]
    public void Revolution_of_an_edge_crossing_the_axis_is_null()
    {
        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(new gp_Pnt(-1, 0, 1), new gp_Pnt(1, 0, 1)).Edge();
        var aMR = new BRepPrimAPI_MakeRevol(anE, OZ);
        Assert.False(aMR.IsDone());
        Assert.ThrowsAny<StdFail_NotDone>(() => aMR.Shape());
    }

    [Fact]
    public void Revolution_of_a_frozen_face_leaves_it_unchanged_and_answers_its_history()
    {
        TopoDS_Face aF = XZRectangle(1, 2, 0, 1);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aF);
        var aMR = new BRepPrimAPI_MakeRevol(aF, OZ, Math.PI / 3);
        TopoDS_Shape aS = aMR.Shape();
        aSnap.AssertUnchanged(aF);
        Assert.True(AllFrozen(aS));
        Assert.True(IsValid(aS));
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.False(aMR.IsDeleted(anExp.Current()));
            TopoDS_Shape aGen = Assert.Single(aMR.Generated(anExp.Current()));
            Assert.Equal(TopAbs_FACE, aGen.ShapeType());
            Assert.True(Contains(aS, aGen));
            Assert.True(Contains(aS, aMR.FirstShape(anExp.Current())));
            Assert.True(Contains(aS, aMR.LastShape(anExp.Current())));
        }
        for (var anExp = new TopExp_Explorer(aF, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            Assert.True(Contains(aS, anExp.Current())); // unchanged: the face's own
        }
        Assert.Equal(Math.PI * 3 / 6, Volume(aS), 1e-9);
    }
}
