// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Planar;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.TestHarness;
using static Kerf.Engine.Fillets.Planar.ConstructionError;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ChFi2d_Builder through BRepFilletAPI_MakeFillet2d (unit 5R; OCCT has no GTests for ChFi2d or MakeFillet2d): fillets
/// and chamfers on frozen faces, their history, the statuses, removal and modification, and the face left unchanged.
/// The values against native OCCT are in the oracle (BRepFilletAPI) and in <see cref="ChFi2d_NativeTests"/>.
/// </summary>
public class ChFi2d_BuilderTests
{
    // A frozen planar face on the closed polygon of the points, as a caller holds it.
    private static Face Polygon(params Point3D[] thePoints)
    {
        var aPoly = new MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aPoly.Add(aP);
        }
        aPoly.Close();
        return new MakeFace(aPoly.Wire(), true).Face();
    }

    private static Face Square(double theSize = 10.0) =>
        Polygon(new Point3D(0, 0, 0), new Point3D(theSize, 0, 0), new Point3D(theSize, theSize, 0), new Point3D(0, theSize, 0));

    private static List<Shape> Sub(in Shape theShape, ShapeType theType)
    {
        IndexedSet<Shape> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return Enumerable.Range(1, aMap.Count).Select(i => aMap[i - 1]).ToList();
    }

    private static double Area(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void Fillets_on_every_corner_round_a_square_without_touching_it()
    {
        Face aSquare = Square();
        string aBefore = BRepTextComparison.Write(aSquare);
        var aMaker = new MakeFillet2D(aSquare);
        Assert.Equal(ConstructionError.Ready, aMaker.Status());
        foreach (Shape aV in Sub(aSquare, ShapeType.Vertex))
        {
            Edge aFillet = aMaker.AddFillet(ShapeCasts.Vertex(aV), 2.0);
            Assert.Equal(ConstructionError.IsDone, aMaker.Status());
            Assert.True(aFillet.TShape()!.IsFrozen);
        }
        aMaker.Build();
        Assert.True(aMaker.IsDone);
        Shape aResult = aMaker.Shape();
        Assert.Equal(4, aMaker.FilletCount);
        Assert.Equal(0, aMaker.ChamferCount);
        Assert.Equal(4, aMaker.CurveCount);
        Assert.Equal(8, Sub(aResult, ShapeType.Edge).Count);
        Assert.Equal(100.0 - (4.0 - Math.PI) * 4.0, Area(aResult), 1.0e-9);
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(aBefore, BRepTextComparison.Write(aSquare));

        // every edge of the square is trimmed: its descendant is in the result, and is built from it
        List<Shape> aResultEdges = Sub(aResult, ShapeType.Edge);
        foreach (Shape anE in Sub(aSquare, ShapeType.Edge))
        {
            Edge anEdge = ShapeCasts.Edge(anE);
            Assert.True(aMaker.IsModified(anEdge));
            Assert.True(aMaker.HasDescendant(anEdge));
            Shape aDescendant = Assert.Single(aMaker.Modified(anEdge));
            Assert.Contains(aResultEdges, e => e.IsSame(aDescendant));
            Assert.True(aMaker.BasisEdge(ShapeCasts.Edge(aDescendant)).IsSame(anEdge));
        }
        Assert.True(aMaker.NewEdges(1)[0].IsSame(aMaker.FilletEdges()[0]));
    }

    [Fact]
    public void A_chamfer_trims_its_two_edges_and_leaves_the_others()
    {
        Face aSquare = Square();
        var aMaker = new MakeFillet2D(aSquare);
        IndexedMap<Shape, NCollection_List<Shape>> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aSquare, ShapeType.Vertex, ShapeType.Edge, aMap);
        NCollection_List<Shape> anEdges = aMap.ValueAt(0);
        Edge E1 = ShapeCasts.Edge(anEdges.First()), E2 = ShapeCasts.Edge(anEdges.Last());
        Edge aChamfer = aMaker.AddChamfer(E1, E2, 1.0, 2.0);
        Assert.Equal(ConstructionError.IsDone, aMaker.Status());
        Assert.False(aChamfer.IsNull);
        aMaker.Build();
        Shape aResult = aMaker.Shape();
        Assert.Equal(1, aMaker.ChamferCount);
        Assert.Equal(5, Sub(aResult, ShapeType.Edge).Count);
        Assert.Equal(100.0 - 1.0, Area(aResult), 1.0e-9);

        // the chamfer is a segment of length sqrt(1 + 4)
        var aCurve = new EdgeCurveAdaptor(aChamfer);
        Assert.Equal(Math.Sqrt(5.0), aCurve.Value(aCurve.FirstParameter()).DistanceTo(aCurve.Value(aCurve.LastParameter())), 1.0e-12);

        Assert.True(aMaker.IsModified(E1));
        Assert.True(aMaker.IsModified(E2));
        Edge anUntouched = ShapeCasts.Edge(Sub(aSquare, ShapeType.Edge)
                                             .First(e => !e.IsSame(E1) && !e.IsSame(E2)));
        Assert.False(aMaker.IsModified(anUntouched));
        // OCCT's Modified reads the history with Find: an edge without descendant throws
        EngineAssert.ThrowsAny<NoSuchObjectException>(() => aMaker.Modified(anUntouched));
        // the untouched edges are the caller's own
        Assert.Contains(Sub(aResult, ShapeType.Edge), e => e.IsSame(anUntouched));
    }

    [Fact]
    public void A_chamfer_by_distance_and_angle()
    {
        Face aSquare = Square();
        var aMaker = new MakeFillet2D(aSquare);
        Vertex aV = ShapeCasts.Vertex(Sub(aSquare, ShapeType.Vertex)[0]);
        IndexedMap<Shape, NCollection_List<Shape>> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aSquare, ShapeType.Vertex, ShapeType.Edge, aMap);
        Edge E = ShapeCasts.Edge(aMap.GetValue(aV).First());
        aMaker.AddChamfer(E, aV, 2.0, Math.PI / 4.0);
        Assert.Equal(ConstructionError.IsDone, aMaker.Status());
        aMaker.Build();
        // a 45 degree chamfer 2 along the edge cuts a right isosceles triangle of legs 2
        Assert.Equal(100.0 - 2.0, Area(aMaker.Shape()), 1.0e-9);
    }

    [Fact]
    public void Statuses_follow_OCCT()
    {
        // a null face, a face that isn't planar
        Assert.Equal(ConstructionError.NoFace, new MakeFillet2D(new Face()).Status());
        Shape aCylinder = new ApiMakeCylinder(1.0, 2.0).Shape();
        Face aLateral = ShapeCasts.Face(Sub(aCylinder, ShapeType.Face)
                                           .First(f => Tool.Surface(ShapeCasts.Face(f)) is CylindricalSurface));
        Assert.Equal(ConstructionError.NotPlanar, new MakeFillet2D(aLateral).Status());

        Face aSquare = Square();
        var aMaker = new MakeFillet2D(aSquare);
        // a vertex not on the face
        aMaker.AddFillet(new MakeVertex(new Point3D(5, 5, 0)).Vertex(), 1.0);
        Assert.Equal(ConstructionError.ConnexionError, aMaker.Status());
        // a radius too large for the edges
        Vertex aV = ShapeCasts.Vertex(Sub(aSquare, ShapeType.Vertex)[0]);
        aMaker.AddFillet(aV, 20.0);
        Assert.Equal(ConstructionError.ComputationError, aMaker.Status());
        aMaker.Build();
        Assert.False(aMaker.IsDone);
        EngineAssert.ThrowsAny<NotDoneException>(() => aMaker.Shape());
        // a chamfer distance not positive
        IndexedMap<Shape, NCollection_List<Shape>> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aSquare, ShapeType.Vertex, ShapeType.Edge, aMap);
        aMaker.AddChamfer(ShapeCasts.Edge(aMap.ValueAt(1).First()), ShapeCasts.Edge(aMap.ValueAt(1).Last()), -1.0, 1.0);
        Assert.Equal(ConstructionError.ParametersError, aMaker.Status());
        // a fillet next to a fillet: the second vertex's edges include the first fillet
        Edge aFillet = aMaker.AddFillet(aV, 1.0);
        Assert.Equal(ConstructionError.IsDone, aMaker.Status());
        Exploration.Vertices(aFillet, out Vertex aFV, out _);
        aMaker.AddFillet(aFV, 0.5);
        Assert.Equal(ConstructionError.NotAuthorized, aMaker.Status());

        // two collinear edges are tangent
        Face aFlat = Polygon(new Point3D(0, 0, 0), new Point3D(5, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0));
        var aFlatMaker = new MakeFillet2D(aFlat);
        Vertex aMid = ShapeCasts.Vertex(Sub(aFlat, ShapeType.Vertex)
                                           .First(v => Tool.Pnt(ShapeCasts.Vertex(v)).DistanceTo(new Point3D(5, 0, 0)) < 1e-9));
        aFlatMaker.AddFillet(aMid, 1.0);
        Assert.Equal(ConstructionError.TangencyError, aFlatMaker.Status());
    }

    [Fact]
    public void Removing_and_modifying_a_fillet()
    {
        Face aSquare = Square();
        var aMaker = new MakeFillet2D(aSquare);
        Vertex aV = ShapeCasts.Vertex(Sub(aSquare, ShapeType.Vertex)[2]);
        Edge aFillet = aMaker.AddFillet(aV, 1.0);
        Edge aBigger = aMaker.ModifyFillet(aFillet, 3.0);
        Assert.Equal(ConstructionError.IsDone, aMaker.Status());
        Assert.Equal(1, aMaker.FilletCount);
        aMaker.Build();
        Assert.Equal(100.0 - (4.0 - Math.PI) * 9.0 / 4.0, Area(aMaker.Shape()), 1.0e-9);

        Vertex aBack = aMaker.RemoveFillet(aBigger);
        Assert.True(aBack.IsSame(aV));
        Assert.Equal(0, aMaker.FilletCount);
        aMaker.Build();
        Shape aResult = aMaker.Shape();
        Assert.Equal(100.0, Area(aResult), 1.0e-9);
        // the square's own edges are back
        foreach (Shape anE in Sub(aSquare, ShapeType.Edge))
        {
            Assert.Contains(Sub(aResult, ShapeType.Edge), e => e.IsSame(anE));
        }
        // removing an edge that isn't a fillet does nothing and gives a null vertex
        Assert.True(aMaker.RemoveFillet(ShapeCasts.Edge(Sub(aSquare, ShapeType.Edge)[0])).IsNull);
    }

    [Fact]
    public void The_builder_on_a_face_already_filleted()
    {
        Face aSquare = Square();
        var aMaker = new MakeFillet2D(aSquare);
        foreach (Shape aV in Sub(aSquare, ShapeType.Vertex).Take(2))
        {
            aMaker.AddFillet(ShapeCasts.Vertex(aV), 1.5);
        }
        IndexedMap<Shape, NCollection_List<Shape>> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aSquare, ShapeType.Vertex, ShapeType.Edge, aMap);
        aMaker.AddChamfer(ShapeCasts.Edge(aMap.ValueAt(2).First()), ShapeCasts.Edge(aMap.ValueAt(2).Last()), 1.0, 1.0);
        aMaker.Build();
        Face aModified = ShapeCasts.Face(aMaker.Shape());

        // Init(RefFace, ModFace) finds the fillets (circles) and the chamfers (lines) of the derived face
        var aBuilder = new PlanarBuilder();
        aBuilder.Init(aSquare, aModified);
        Assert.Equal(ConstructionError.Ready, aBuilder.Status());
        Assert.Equal(2, aBuilder.FilletCount);
        Assert.Equal(1, aBuilder.ChamferCount);
        Vertex aBack = aBuilder.RemoveFillet(ShapeCasts.Edge(aBuilder.FilletEdges()[0]));
        Assert.False(aBack.IsNull);
        Assert.Equal(1, aBuilder.FilletCount);
        Assert.Equal(100.0 - 0.5 - (4.0 - Math.PI) * 2.25 / 4.0, Area(aBuilder.Result()), 1.0e-9);

        // ChFi2d_Builder(F) builds the 3D curves the face's edges lack; these have them, so nothing is copied
        var aDirect = new PlanarBuilder(aSquare);
        Assert.Equal(ConstructionError.Ready, aDirect.Status());
        aDirect.AddFillet(ShapeCasts.Vertex(Sub(aSquare, ShapeType.Vertex)[0]), 1.0);
        Assert.Equal(ConstructionError.IsDone, aDirect.Status());
    }

    // The fillet of radius theR of the corner of e1 and e2 (in that order) in the plane z = 0: its arc, and e1 and e2
    // shrunk to it, each ending where the arc does; the arc tangent to both (its centre at theR from the line, at the
    // circle's radius plus or minus theR from the circle's centre).
    private static void CheckFillet(Edge theE1, Edge theE2, double theR)
    {
        var anAlgo = new AnaFilletAlgo(theE1, theE2, new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        Assert.True(anAlgo.Perform(theR));
        Edge M1 = new(), M2 = new();
        Edge anArc = anAlgo.Result(ref M1, ref M2);
        Assert.False(anArc.IsNull || M1.IsNull || M2.IsNull);
        var aFillet = new EdgeCurveAdaptor(anArc);
        Assert.Equal(theR, aFillet.Circle().Radius, 1.0e-9);
        Point3D aCentre = aFillet.Circle().Location;
        foreach (Edge anE in new[] { theE1, theE2 })
        {
            var aC = new EdgeCurveAdaptor(anE);
            double aGap = aC.GetType() == CurveKind.Line
                              ? aC.Line().DistanceTo(aCentre)
                              : Math.Abs(Math.Abs(aC.Circle().Location.DistanceTo(aCentre) - aC.Circle().Radius));
            Assert.Equal(theR, aGap, 1.0e-9);
        }
        // the arc's ends are the shrunk edges' ends at the corner
        Point3D aStart = aFillet.Value(aFillet.FirstParameter()), anEnd = aFillet.Value(aFillet.LastParameter());
        bool Touches(Edge theEdge, Point3D theP)
        {
            var aC = new EdgeCurveAdaptor(theEdge);
            return aC.Value(aC.FirstParameter()).DistanceTo(theP) < 1.0e-7 || aC.Value(aC.LastParameter()).DistanceTo(theP) < 1.0e-7;
        }
        Assert.True((Touches(M1, aStart) && Touches(M2, anEnd)) || (Touches(M1, anEnd) && Touches(M2, aStart)));
    }

    [Fact]
    public void A_fillet_of_a_moved_arc()
    {
        // a quarter arc of radius 5 about the origin, moved by (10, 0, 0): from (15, 0) to (10, 5), then up the line
        // x = 10. OCCT's isCW took the arc's middle point without the edge's location, and its shrinking projected the
        // located ends on the curve without it (occt-issues.md, unit 5R's ChFi2d_AnaFilletAlgo isCW row; decision
        // 16715).
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        Edge anArc = new MakeEdge(aCircle, 0.0, Math.PI / 2).Edge();
        Edge aMoved = ShapeCasts.Edge(anArc.Moved(new Placement(Transform3D.Translation(new Vector3D(10, 0, 0)))));
        Edge aLine = new MakeEdge(new Point3D(10, 5, 0), new Point3D(10, 15, 0)).Edge();
        CheckFillet(aMoved, aLine, 1.0);
        // the same arc built in place
        Edge aPlaced = new MakeEdge(new CircleCurve3D(new Frame3D(new Point3D(10, 0, 0), new Direction3D(0, 0, 1)), 5.0), 0.0, Math.PI / 2).Edge();
        CheckFillet(aPlaced, aLine, 1.0);
    }

    [Fact]
    public void A_fillet_of_edges_of_different_orientations()
    {
        // the second edge runs from (10, 10) to the corner and is reversed in the wire: shrunk, it keeps (10, 10) and
        // the fillet's end, (10, 2). OCCT chose its ends by the first edge's orientation, and kept the corner's part
        // (occt-issues.md, unit 5R's ChFi2d_AnaFilletAlgo::Perform row; decision 1473).
        Edge E1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Edge E2 = ShapeCasts.Edge(new MakeEdge(new Point3D(10, 10, 0), new Point3D(10, 0, 0)).Edge().Reversed());
        CheckFillet(E1, E2, 2.0);
        var anAlgo = new AnaFilletAlgo(E1, E2, new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        Assert.True(anAlgo.Perform(2.0));
        Edge M1 = new(), M2 = new();
        anAlgo.Result(ref M1, ref M2);
        var aC2 = new EdgeCurveAdaptor(M2);
        Point3D[] anEnds = [aC2.Value(aC2.FirstParameter()), aC2.Value(aC2.LastParameter())];
        Assert.Contains(anEnds, p => p.DistanceTo(new Point3D(10, 10, 0)) < 1.0e-9);
        Assert.Contains(anEnds, p => p.DistanceTo(new Point3D(10, 2, 0)) < 1.0e-9);
        var aC1 = new EdgeCurveAdaptor(M1);
        Assert.True(aC1.Value(aC1.FirstParameter()).DistanceTo(new Point3D(0, 0, 0)) < 1.0e-9);
        Assert.True(aC1.Value(aC1.LastParameter()).DistanceTo(new Point3D(8, 0, 0)) < 1.0e-9);
    }

    [Fact]
    public void A_face_with_a_hole_keeps_its_hole()
    {
        // a square of 10 with a square hole of 4: a fillet of radius 2 on an outer corner, a chamfer of 1 by 1 on an
        // inner one, then the fillet removed. Each result has its two wires, closed, and the area of the cut corners
        // (OCCT put every edge into one wire: occt-issues.md, unit 5R's ChFi2d_Builder::BuildNewWire row; decision
        // 16717)
        var aPoly = new MakePolygon();
        foreach (Point3D aP in new[] { new Point3D(3, 3, 0), new Point3D(3, 7, 0), new Point3D(7, 7, 0), new Point3D(7, 3, 0) })
        {
            aPoly.Add(aP);
        }
        aPoly.Close();
        var aMaker = new MakeFace(Square());
        aMaker.Add(aPoly.Wire());
        Face aHoled = aMaker.Face();
        Assert.Equal(84.0, Area(aHoled), 1.0e-9);

        // (a removal leaves the closed flags unset, as OCCT's did)
        void CheckWires(in Shape theFace, double theArea, bool theFlagged = true)
        {
            List<Shape> aWires = Sub(theFace, ShapeType.Wire);
            Assert.Equal(2, aWires.Count);
            Assert.All(aWires, w => Assert.True(!theFlagged || ShapeCasts.Wire(w).ClosedFlag));
            Assert.Equal(theArea, Area(theFace), 1.0e-9);
            Assert.True(new Analyzer(theFace).IsValid());
        }

        Vertex Corner(in Shape theFace, Point3D theP)
            => Sub(theFace, ShapeType.Vertex).Select(s => ShapeCasts.Vertex(s)).Single(v => Tool.Pnt(v).IsEqual(theP, 1.0e-9));

        var aFillet = new MakeFillet2D(aHoled);
        Edge aFilletEdge = aFillet.AddFillet(Corner(aHoled, new Point3D(10, 10, 0)), 2.0);
        aFillet.Build();
        Assert.True(aFillet.IsDone);
        CheckWires(aFillet.Shape(), 84.0 - 4.0 * (1.0 - Math.PI / 4.0));

        // removed again
        aFillet.RemoveFillet(aFilletEdge);
        aFillet.Build();
        CheckWires(aFillet.Shape(), 84.0, false);

        // a chamfer on the hole's corner (3, 3): the face gains the corner's triangle
        Vertex anInner = Corner(aHoled, new Point3D(3, 3, 0));
        IndexedMap<Shape, NCollection_List<Shape>> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aHoled, ShapeType.Vertex, ShapeType.Edge, aMap);
        NCollection_List<Shape> anEdges = aMap.GetValue(anInner);
        var aChamfer = new MakeFillet2D(aHoled);
        aChamfer.AddChamfer(ShapeCasts.Edge(anEdges.First()), ShapeCasts.Edge(anEdges.Last()), 1.0, 1.0);
        aChamfer.Build();
        Assert.True(aChamfer.IsDone);
        CheckWires(aChamfer.Shape(), 84.0 + 0.5);
    }

    [Fact]
    public void FilletAPI_and_ChamferAPI_on_two_segments()
    {
        Edge E1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Edge E2 = new MakeEdge(new Point3D(10, 0, 0), new Point3D(10, 10, 0)).Edge();
        var aPln = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));

        // segments with a common point: the analytical algorithm
        var aFillet = new FilletApi(E1, E2, aPln);
        Assert.True(aFillet.Perform(2.0));
        Assert.Equal(1, aFillet.NbResults(new Point3D(10, 0, 0)));
        Edge M1 = new(), M2 = new();
        Edge anArc = aFillet.Result(new Point3D(10, 0, 0), ref M1, ref M2);
        var aCurve = new EdgeCurveAdaptor(anArc);
        Assert.Equal(CurveKind.Circle, aCurve.GetType());
        Assert.Equal(2.0, aCurve.Circle().Radius, 1.0e-12);
        Assert.True(aCurve.Circle().Location.IsEqual(new Point3D(8, 2, 0), 1.0e-12));
        Assert.Equal(8.0, new EdgeCurveAdaptor(M1).LastParameter() - new EdgeCurveAdaptor(M1).FirstParameter(), 1.0e-12);

        // apart by a gap: the iterative algorithm
        Edge E3 = new MakeEdge(new Point3D(10, 1.0e-3, 0), new Point3D(10, 10, 0)).Edge();
        var anIterative = new FilletAlgo(E1, E3, aPln);
        Assert.True(anIterative.Perform(2.0));
        Edge anArc2 = anIterative.Result(new Point3D(10, 0, 0), ref M1, ref M2);
        Assert.Equal(2.0, new EdgeCurveAdaptor(anArc2).Circle().Radius, 1.0e-12);

        // a chamfer of lengths 1 and 3
        var aChamfer = new ChamferApi(E1, E2);
        Assert.True(aChamfer.Perform());
        Edge aSegment = aChamfer.Result(ref M1, ref M2, 1.0, 3.0);
        var aSeg = new EdgeCurveAdaptor(aSegment);
        Assert.True(aSeg.Value(aSeg.FirstParameter()).IsEqual(new Point3D(9, 0, 0), 1.0e-12));
        Assert.True(aSeg.Value(aSeg.LastParameter()).IsEqual(new Point3D(10, 3, 0), 1.0e-12));
        // too long for the edge
        Assert.True(aChamfer.Result(ref M1, ref M2, 20.0, 1.0).IsNull);

        // the analytical algorithm needs two edges
        EngineAssert.ThrowsAny<TypeMismatchException>(() => new AnaFilletAlgo(new MakeWire(E1).Wire(), aPln));
        EngineAssert.ThrowsAny<ConstructionErrorException>(() => new FilletAlgo(new MakeWire(E1).Wire(), aPln));
    }
}
