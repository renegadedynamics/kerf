// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using OcctNet.TestHarness;
using static Occt.ChFi2d_ConstructionError;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ChFi2d_Builder through BRepFilletAPI_MakeFillet2d (unit 5R; OCCT has no GTests for ChFi2d or MakeFillet2d): fillets
/// and chamfers on frozen faces, their history, the statuses, removal and modification, and the face left unchanged.
/// The values against native OCCT are in the oracle (BRepFilletAPI) and in <see cref="ChFi2d_NativeTests"/>.
/// </summary>
public class ChFi2d_BuilderTests
{
    // A frozen planar face on the closed polygon of the points, as a caller holds it.
    private static TopoDS_Face Polygon(params Point3D[] thePoints)
    {
        var aPoly = new BRepBuilderAPI_MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aPoly.Add(aP);
        }
        aPoly.Close();
        return new BRepBuilderAPI_MakeFace(aPoly.Wire(), true).Face();
    }

    private static TopoDS_Face Square(double theSize = 10.0) =>
        Polygon(new Point3D(0, 0, 0), new Point3D(theSize, 0, 0), new Point3D(theSize, theSize, 0), new Point3D(0, theSize, 0));

    private static List<TopoDS_Shape> Sub(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        NCollection_IndexedMap<TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return Enumerable.Range(1, aMap.Extent()).Select(i => aMap.FindKey(i)).ToList();
    }

    private static double Area(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void Fillets_on_every_corner_round_a_square_without_touching_it()
    {
        TopoDS_Face aSquare = Square();
        string aBefore = BRepTextComparison.Write(aSquare);
        var aMaker = new BRepFilletAPI_MakeFillet2d(aSquare);
        Assert.Equal(ChFi2d_Ready, aMaker.Status());
        foreach (TopoDS_Shape aV in Sub(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX))
        {
            TopoDS_Edge aFillet = aMaker.AddFillet(TopoDS.Vertex(aV), 2.0);
            Assert.Equal(ChFi2d_IsDone, aMaker.Status());
            Assert.True(aFillet.TShape()!.IsFrozen);
        }
        aMaker.Build();
        Assert.True(aMaker.IsDone());
        TopoDS_Shape aResult = aMaker.Shape();
        Assert.Equal(4, aMaker.NbFillet());
        Assert.Equal(0, aMaker.NbChamfer());
        Assert.Equal(4, aMaker.NbCurves());
        Assert.Equal(8, Sub(aResult, TopAbs_ShapeEnum.TopAbs_EDGE).Count);
        Assert.Equal(100.0 - (4.0 - Math.PI) * 4.0, Area(aResult), 1.0e-9);
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(aBefore, BRepTextComparison.Write(aSquare));

        // every edge of the square is trimmed: its descendant is in the result, and is built from it
        List<TopoDS_Shape> aResultEdges = Sub(aResult, TopAbs_ShapeEnum.TopAbs_EDGE);
        foreach (TopoDS_Shape anE in Sub(aSquare, TopAbs_ShapeEnum.TopAbs_EDGE))
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anE);
            Assert.True(aMaker.IsModified(anEdge));
            Assert.True(aMaker.HasDescendant(anEdge));
            TopoDS_Shape aDescendant = Assert.Single(aMaker.Modified(anEdge));
            Assert.Contains(aResultEdges, e => e.IsSame(aDescendant));
            Assert.True(aMaker.BasisEdge(TopoDS.Edge(aDescendant)).IsSame(anEdge));
        }
        Assert.True(aMaker.NewEdges(1)[0].IsSame(aMaker.FilletEdges()[0]));
    }

    [Fact]
    public void A_chamfer_trims_its_two_edges_and_leaves_the_others()
    {
        TopoDS_Face aSquare = Square();
        var aMaker = new BRepFilletAPI_MakeFillet2d(aSquare);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, aMap);
        NCollection_List<TopoDS_Shape> anEdges = aMap.FindFromIndex(1);
        TopoDS_Edge E1 = TopoDS.Edge(anEdges.First()), E2 = TopoDS.Edge(anEdges.Last());
        TopoDS_Edge aChamfer = aMaker.AddChamfer(E1, E2, 1.0, 2.0);
        Assert.Equal(ChFi2d_IsDone, aMaker.Status());
        Assert.False(aChamfer.IsNull());
        aMaker.Build();
        TopoDS_Shape aResult = aMaker.Shape();
        Assert.Equal(1, aMaker.NbChamfer());
        Assert.Equal(5, Sub(aResult, TopAbs_ShapeEnum.TopAbs_EDGE).Count);
        Assert.Equal(100.0 - 1.0, Area(aResult), 1.0e-9);

        // the chamfer is a segment of length sqrt(1 + 4)
        var aCurve = new BRepAdaptor_Curve(aChamfer);
        Assert.Equal(Math.Sqrt(5.0), aCurve.Value(aCurve.FirstParameter()).Distance(aCurve.Value(aCurve.LastParameter())), 1.0e-12);

        Assert.True(aMaker.IsModified(E1));
        Assert.True(aMaker.IsModified(E2));
        TopoDS_Edge anUntouched = TopoDS.Edge(Sub(aSquare, TopAbs_ShapeEnum.TopAbs_EDGE)
                                             .First(e => !e.IsSame(E1) && !e.IsSame(E2)));
        Assert.False(aMaker.IsModified(anUntouched));
        // OCCT's Modified reads the history with Find: an edge without descendant throws
        Assert.ThrowsAny<NoSuchObjectException>(() => aMaker.Modified(anUntouched));
        // the untouched edges are the caller's own
        Assert.Contains(Sub(aResult, TopAbs_ShapeEnum.TopAbs_EDGE), e => e.IsSame(anUntouched));
    }

    [Fact]
    public void A_chamfer_by_distance_and_angle()
    {
        TopoDS_Face aSquare = Square();
        var aMaker = new BRepFilletAPI_MakeFillet2d(aSquare);
        TopoDS_Vertex aV = TopoDS.Vertex(Sub(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX)[0]);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, aMap);
        TopoDS_Edge E = TopoDS.Edge(aMap.FindFromKey(aV).First());
        aMaker.AddChamfer(E, aV, 2.0, Math.PI / 4.0);
        Assert.Equal(ChFi2d_IsDone, aMaker.Status());
        aMaker.Build();
        // a 45 degree chamfer 2 along the edge cuts a right isosceles triangle of legs 2
        Assert.Equal(100.0 - 2.0, Area(aMaker.Shape()), 1.0e-9);
    }

    [Fact]
    public void Statuses_follow_OCCT()
    {
        // a null face, a face that isn't planar
        Assert.Equal(ChFi2d_NoFace, new BRepFilletAPI_MakeFillet2d(new TopoDS_Face()).Status());
        TopoDS_Shape aCylinder = new BRepPrimAPI_MakeCylinder(1.0, 2.0).Shape();
        TopoDS_Face aLateral = TopoDS.Face(Sub(aCylinder, TopAbs_ShapeEnum.TopAbs_FACE)
                                           .First(f => BRep_Tool.Surface(TopoDS.Face(f)) is Geom_CylindricalSurface));
        Assert.Equal(ChFi2d_NotPlanar, new BRepFilletAPI_MakeFillet2d(aLateral).Status());

        TopoDS_Face aSquare = Square();
        var aMaker = new BRepFilletAPI_MakeFillet2d(aSquare);
        // a vertex not on the face
        aMaker.AddFillet(new BRepBuilderAPI_MakeVertex(new Point3D(5, 5, 0)).Vertex(), 1.0);
        Assert.Equal(ChFi2d_ConnexionError, aMaker.Status());
        // a radius too large for the edges
        TopoDS_Vertex aV = TopoDS.Vertex(Sub(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX)[0]);
        aMaker.AddFillet(aV, 20.0);
        Assert.Equal(ChFi2d_ComputationError, aMaker.Status());
        aMaker.Build();
        Assert.False(aMaker.IsDone());
        Assert.ThrowsAny<NotDoneException>(() => aMaker.Shape());
        // a chamfer distance not positive
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, aMap);
        aMaker.AddChamfer(TopoDS.Edge(aMap.FindFromIndex(2).First()), TopoDS.Edge(aMap.FindFromIndex(2).Last()), -1.0, 1.0);
        Assert.Equal(ChFi2d_ParametersError, aMaker.Status());
        // a fillet next to a fillet: the second vertex's edges include the first fillet
        TopoDS_Edge aFillet = aMaker.AddFillet(aV, 1.0);
        Assert.Equal(ChFi2d_IsDone, aMaker.Status());
        TopExp.Vertices(aFillet, out TopoDS_Vertex aFV, out _);
        aMaker.AddFillet(aFV, 0.5);
        Assert.Equal(ChFi2d_NotAuthorized, aMaker.Status());

        // two collinear edges are tangent
        TopoDS_Face aFlat = Polygon(new Point3D(0, 0, 0), new Point3D(5, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0));
        var aFlatMaker = new BRepFilletAPI_MakeFillet2d(aFlat);
        TopoDS_Vertex aMid = TopoDS.Vertex(Sub(aFlat, TopAbs_ShapeEnum.TopAbs_VERTEX)
                                           .First(v => BRep_Tool.Pnt(TopoDS.Vertex(v)).Distance(new Point3D(5, 0, 0)) < 1e-9));
        aFlatMaker.AddFillet(aMid, 1.0);
        Assert.Equal(ChFi2d_TangencyError, aFlatMaker.Status());
    }

    [Fact]
    public void Removing_and_modifying_a_fillet()
    {
        TopoDS_Face aSquare = Square();
        var aMaker = new BRepFilletAPI_MakeFillet2d(aSquare);
        TopoDS_Vertex aV = TopoDS.Vertex(Sub(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX)[2]);
        TopoDS_Edge aFillet = aMaker.AddFillet(aV, 1.0);
        TopoDS_Edge aBigger = aMaker.ModifyFillet(aFillet, 3.0);
        Assert.Equal(ChFi2d_IsDone, aMaker.Status());
        Assert.Equal(1, aMaker.NbFillet());
        aMaker.Build();
        Assert.Equal(100.0 - (4.0 - Math.PI) * 9.0 / 4.0, Area(aMaker.Shape()), 1.0e-9);

        TopoDS_Vertex aBack = aMaker.RemoveFillet(aBigger);
        Assert.True(aBack.IsSame(aV));
        Assert.Equal(0, aMaker.NbFillet());
        aMaker.Build();
        TopoDS_Shape aResult = aMaker.Shape();
        Assert.Equal(100.0, Area(aResult), 1.0e-9);
        // the square's own edges are back
        foreach (TopoDS_Shape anE in Sub(aSquare, TopAbs_ShapeEnum.TopAbs_EDGE))
        {
            Assert.Contains(Sub(aResult, TopAbs_ShapeEnum.TopAbs_EDGE), e => e.IsSame(anE));
        }
        // removing an edge that isn't a fillet does nothing and gives a null vertex
        Assert.True(aMaker.RemoveFillet(TopoDS.Edge(Sub(aSquare, TopAbs_ShapeEnum.TopAbs_EDGE)[0])).IsNull());
    }

    [Fact]
    public void The_builder_on_a_face_already_filleted()
    {
        TopoDS_Face aSquare = Square();
        var aMaker = new BRepFilletAPI_MakeFillet2d(aSquare);
        foreach (TopoDS_Shape aV in Sub(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX).Take(2))
        {
            aMaker.AddFillet(TopoDS.Vertex(aV), 1.5);
        }
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, aMap);
        aMaker.AddChamfer(TopoDS.Edge(aMap.FindFromIndex(3).First()), TopoDS.Edge(aMap.FindFromIndex(3).Last()), 1.0, 1.0);
        aMaker.Build();
        TopoDS_Face aModified = TopoDS.Face(aMaker.Shape());

        // Init(RefFace, ModFace) finds the fillets (circles) and the chamfers (lines) of the derived face
        var aBuilder = new ChFi2d_Builder();
        aBuilder.Init(aSquare, aModified);
        Assert.Equal(ChFi2d_Ready, aBuilder.Status());
        Assert.Equal(2, aBuilder.NbFillet());
        Assert.Equal(1, aBuilder.NbChamfer());
        TopoDS_Vertex aBack = aBuilder.RemoveFillet(TopoDS.Edge(aBuilder.FilletEdges()[0]));
        Assert.False(aBack.IsNull());
        Assert.Equal(1, aBuilder.NbFillet());
        Assert.Equal(100.0 - 0.5 - (4.0 - Math.PI) * 2.25 / 4.0, Area(aBuilder.Result()), 1.0e-9);

        // ChFi2d_Builder(F) builds the 3D curves the face's edges lack; these have them, so nothing is copied
        var aDirect = new ChFi2d_Builder(aSquare);
        Assert.Equal(ChFi2d_Ready, aDirect.Status());
        aDirect.AddFillet(TopoDS.Vertex(Sub(aSquare, TopAbs_ShapeEnum.TopAbs_VERTEX)[0]), 1.0);
        Assert.Equal(ChFi2d_IsDone, aDirect.Status());
    }

    [Fact]
    public void FilletAPI_and_ChamferAPI_on_two_segments()
    {
        TopoDS_Edge E1 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        TopoDS_Edge E2 = new BRepBuilderAPI_MakeEdge(new Point3D(10, 0, 0), new Point3D(10, 10, 0)).Edge();
        var aPln = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));

        // segments with a common point: the analytical algorithm
        var aFillet = new ChFi2d_FilletAPI(E1, E2, aPln);
        Assert.True(aFillet.Perform(2.0));
        Assert.Equal(1, aFillet.NbResults(new Point3D(10, 0, 0)));
        TopoDS_Edge M1 = new(), M2 = new();
        TopoDS_Edge anArc = aFillet.Result(new Point3D(10, 0, 0), ref M1, ref M2);
        var aCurve = new BRepAdaptor_Curve(anArc);
        Assert.Equal(CurveKind.Circle, aCurve.GetType());
        Assert.Equal(2.0, aCurve.Circle().Radius(), 1.0e-12);
        Assert.True(aCurve.Circle().Location().IsEqual(new Point3D(8, 2, 0), 1.0e-12));
        Assert.Equal(8.0, new BRepAdaptor_Curve(M1).LastParameter() - new BRepAdaptor_Curve(M1).FirstParameter(), 1.0e-12);

        // apart by a gap: the iterative algorithm
        TopoDS_Edge E3 = new BRepBuilderAPI_MakeEdge(new Point3D(10, 1.0e-3, 0), new Point3D(10, 10, 0)).Edge();
        var anIterative = new ChFi2d_FilletAlgo(E1, E3, aPln);
        Assert.True(anIterative.Perform(2.0));
        TopoDS_Edge anArc2 = anIterative.Result(new Point3D(10, 0, 0), ref M1, ref M2);
        Assert.Equal(2.0, new BRepAdaptor_Curve(anArc2).Circle().Radius(), 1.0e-12);

        // a chamfer of lengths 1 and 3
        var aChamfer = new ChFi2d_ChamferAPI(E1, E2);
        Assert.True(aChamfer.Perform());
        TopoDS_Edge aSegment = aChamfer.Result(ref M1, ref M2, 1.0, 3.0);
        var aSeg = new BRepAdaptor_Curve(aSegment);
        Assert.True(aSeg.Value(aSeg.FirstParameter()).IsEqual(new Point3D(9, 0, 0), 1.0e-12));
        Assert.True(aSeg.Value(aSeg.LastParameter()).IsEqual(new Point3D(10, 3, 0), 1.0e-12));
        // too long for the edge
        Assert.True(aChamfer.Result(ref M1, ref M2, 20.0, 1.0).IsNull());

        // the analytical algorithm needs two edges
        Assert.ThrowsAny<TypeMismatchException>(() => new ChFi2d_AnaFilletAlgo(new BRepBuilderAPI_MakeWire(E1).Wire(), aPln));
        Assert.ThrowsAny<ConstructionErrorException>(() => new ChFi2d_FilletAlgo(new BRepBuilderAPI_MakeWire(E1).Wire(), aPln));
    }
}
