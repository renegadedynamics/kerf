// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepLib_FuseEdges on frozen inputs: collinear segments of a planar face, circle arcs on a cylinder
/// face, B-splines of the same curve and G1 B-splines (SetConcatBSpl), a closed periodic B-spline (whose curve the
/// algorithm must not change in place), avoided edges, located shapes. Results are frozen, inputs unchanged.
/// </summary>
public class BRepLib_FuseEdgesTests
{
    private const double Conf = 1e-7;

    private static int Count(in Shape S, ShapeType T)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(S, T, aMap);
        return aMap.Count;
    }

    // [0,2] x [0,1] on z = 0, six edges: A(0,0) B(1,0) C(2,0) D(2,1) E(1,1) F(0,1); AB-BC and DE-EF collinear.
    private static Face SixEdgeRectangle(out Vertex[] theVertices, out Dictionary<(int, int), Edge> theEdges)
    {
        theVertices =
        [
            TestShapes.Vertex(new Point3D(0, 0, 0)), TestShapes.Vertex(new Point3D(1, 0, 0)),
            TestShapes.Vertex(new Point3D(2, 0, 0)), TestShapes.Vertex(new Point3D(2, 1, 0)),
            TestShapes.Vertex(new Point3D(1, 1, 0)), TestShapes.Vertex(new Point3D(0, 1, 0)),
        ];
        theEdges = [];
        return TestShapes.Frozen(TestShapes.PolygonFace(theVertices, new Direction3D(0, 0, 1), theEdges, [0, 1, 2, 3, 4, 5]));
    }

    [Fact]
    public void Collinear_segments_of_a_planar_face_are_fused()
    {
        Face aF = SixEdgeRectangle(out Vertex[] aV, out _);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aF);

        var aFuse = new FuseEdges(aF);
        Assert.Equal(2, aFuse.NbVertices());

        IReadOnlyDictionary<int, IReadOnlyList<Shape>> anEdges = aFuse.Edges();
        Assert.Equal(2, anEdges.Count);
        Assert.All(anEdges.Values, aList => Assert.Equal(2, aList.Count));
        Assert.All(anEdges.Values, aList => Assert.All(aList, anE => Assert.True(anE.TShape()!.IsFrozen)));

        IReadOnlyDictionary<int, Shape> aResultEdges = aFuse.ResultEdges();
        Assert.Equal(2, aResultEdges.Count);
        foreach (Shape aS in aResultEdges.Values)
        {
            Edge anE = ShapeCasts.Edge(aS);
            Assert.True(anE.TShape()!.IsFrozen);
            Curve3D aC = Tool.Curve(anE, out double f, out double l)!;
            Assert.IsType<LineCurve3D>(aC);
            Assert.Equal(2.0, l - f, 1e-12);
            // the pcurve on the face's plane is the old edge's, over the new range
            Curve2D aPC = Tool.CurveOnSurface(anE, aF, out double pf, out double pl)!;
            Assert.IsType<LineCurve2D>(aPC);
            Assert.Equal(2.0, pl - pf, 1e-12);
            Point3D aP3 = aC.Value(f);
            Point2D aP2 = aPC.Value(pf);
            Point3D aOnPlane = Tool.Surface(aF)!.Value(aP2.X, aP2.Y);
            Assert.True(aP3.DistanceTo(aOnPlane) < Conf);
        }

        Shape aResult = aFuse.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Face, aResult.Type);
        Assert.Equal(4, Count(aResult, ShapeType.Edge));
        Assert.Equal(4, Count(aResult, ShapeType.Vertex));
        var aVertices = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, ShapeType.Vertex, aVertices);
        Assert.False(aVertices.Contains(aV[1]));
        Assert.False(aVertices.Contains(aV[4]));
        Assert.True(aVertices.Contains(aV[0]) && aVertices.Contains(aV[2]) && aVertices.Contains(aV[3])
                    && aVertices.Contains(aV[5]));

        IReadOnlyDictionary<Shape, Shape> aFaces = aFuse.Faces();
        Assert.Single(aFaces);
        Assert.True(aFaces.ContainsKey(aF));
        Assert.True(aFaces[aF].IsSame(aResult));

        // the area is unchanged
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aResult, aProps);
        Assert.Equal(2.0, Math.Abs(aProps.Mass()), 1e-10);

        aBefore.AssertUnchanged(aF);
    }

    [Fact]
    public void Avoided_edges_are_not_fused()
    {
        Face aF = SixEdgeRectangle(out _, out Dictionary<(int, int), Edge> anEdges);
        var anAvoid = new IndexedSet<Shape>(ShapeHasher.Instance);
        anAvoid.Add(anEdges[(0, 1)]);

        var aFuse = new FuseEdges(aF, true);
        aFuse.AvoidEdges(anAvoid);
        anAvoid.Clear(); // the algorithm keeps its own copy
        Assert.Equal(1, aFuse.NbVertices());
        Shape aResult = aFuse.Shape();
        Assert.Equal(5, Count(aResult, ShapeType.Edge));
        var aResultEdges = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, ShapeType.Edge, aResultEdges);
        Assert.True(aResultEdges.Contains(anEdges[(0, 1)]));
        Assert.True(aResultEdges.Contains(anEdges[(1, 2)]));
    }

    [Fact]
    public void A_located_face_is_fused_in_place()
    {
        Face aF0 = SixEdgeRectangle(out _, out _);
        var aLoc = new Placement(Transform3D.Rotation(new Axis3D(new Point3D(1, 2, 3), new Direction3D(0, 1, 1)), 0.7));
        Face aF = ShapeCasts.Face(aF0.Moved(aLoc));
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aF);

        var aFuse = new FuseEdges(aF);
        Shape aResult = aFuse.Shape();
        Assert.Equal(4, Count(aResult, ShapeType.Edge));

        var aProps0 = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aF, aProps0);
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aResult, aProps);
        Assert.Equal(aProps0.Mass(), aProps.Mass(), 1e-10);
        Assert.True(aProps0.CentreOfMass().DistanceTo(aProps.CentreOfMass()) < 1e-9);

        // each fused edge's 3D ends are its vertices
        for (var anExp = new Explorer(aResult, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            var aC = new EdgeCurveAdaptor(anE);
            Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
            Assert.True(aC.Value(aC.FirstParameter()).DistanceTo(Tool.Pnt(aV1)) < Conf);
            Assert.True(aC.Value(aC.LastParameter()).DistanceTo(Tool.Pnt(aV2)) < Conf);
        }
        aBefore.AssertUnchanged(aF);
    }

    [Fact]
    public void A_shape_without_collinear_edges_is_returned_as_is()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aFuse = new FuseEdges(aBox);
        Assert.Equal(0, aFuse.NbVertices());
        Assert.Empty(aFuse.Edges());
        Assert.Empty(aFuse.ResultEdges());
        Assert.Empty(aFuse.Faces());
        Assert.True(aFuse.Shape().IsEqual(aBox));
    }

    [Fact]
    public void Null_shape_throws()
    {
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NullObjectException>(() => new FuseEdges(new Shape()));
    }

    [Fact]
    public void Circle_arcs_on_a_cylinder_face_are_fused_into_the_circle()
    {
        // the cylinder's lateral face with its bottom circle in two arcs [0, pi] and [pi, 2 pi]
        var aB = new BrepBuilder();
        double r = 2, h = 3, aTwoPi = 2 * Math.PI;
        var aSurf = new CylindricalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r);
        Vertex aV0 = TestShapes.Vertex(new Point3D(r, 0, 0));
        Vertex aVPi = TestShapes.Vertex(new Point3D(-r, 0, 0));
        Vertex aV1 = TestShapes.Vertex(new Point3D(r, 0, h));
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r);
        Edge anArc1 = TestShapes.Edge(aCircle, 0, Math.PI, aV0, aVPi);
        Edge anArc2 = TestShapes.Edge(new CircleCurve3D(aCircle.Circle), Math.PI, aTwoPi, aVPi, aV0);
        Edge aTop = TestShapes.Edge(new CircleCurve3D(new Frame3D(new Point3D(0, 0, h), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r),
                                           0, aTwoPi, aV1, aV1);
        Edge aSeam = TestShapes.Edge(new LineCurve3D(new Point3D(r, 0, 0), new Direction3D(0, 0, 1)), 0, h, aV0, aV1);
        Face aF = new();
        aB.MakeFace(ref aF, aSurf, global::Kerf.Engine.Foundation.Precision.Confusion());
        var aBottom2d = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0));
        aB.UpdateEdge(anArc1, aBottom2d, aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.UpdateEdge(anArc2, aBottom2d, aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.UpdateEdge(aTop, new LineCurve2D(new Point2D(0, h), new Direction2D(1, 0)), aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.UpdateEdge(aSeam, new LineCurve2D(new Point2D(aTwoPi, 0), new Direction2D(0, 1)),
                      new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1)), aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anArc1);
        aB.Add(aW, anArc2);
        aB.Add(aW, aSeam);
        aB.Add(aW, aTop.Reversed());
        aB.Add(aW, aSeam.Reversed());
        aW.Closed(true);
        aB.Add(aF, aW);
        aF = TestShapes.Frozen(aF);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aF);

        var aFuse = new FuseEdges(aF);
        Assert.Equal(1, aFuse.NbVertices());
        Shape aResult = aFuse.Shape();
        Assert.Equal(3, Count(aResult, ShapeType.Edge));
        Assert.Equal(2, Count(aResult, ShapeType.Vertex));

        Edge aFused = ShapeCasts.Edge(Assert.Single(aFuse.ResultEdges().Values));
        Curve3D aC = Tool.Curve(aFused, out double f, out double l)!;
        Assert.IsType<CircleCurve3D>(aC);
        Assert.Equal(aTwoPi, l - f, 1e-12);
        Assert.True(Tool.IsClosed(aFused));
        Curve2D aPC = Tool.CurveOnSurface(aFused, ShapeCasts.Face(aResult), out double pf, out double pl)!;
        Assert.Equal(f, pf);
        Assert.Equal(l, pl);
        Assert.Equal(0.0, aPC.Value(pf).X, 1e-12);
        Assert.Equal(0.0, aPC.Value(pf).Y, 1e-12);

        var aProps0 = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aF, aProps0);
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aResult, aProps);
        Assert.Equal(aProps0.Mass(), aProps.Mass(), 1e-9);
        aBefore.AssertUnchanged(aF);
    }

    private static BSplineCurve3D BSpline(params Point3D[] thePoles)
    {
        var aKnots = NCollection_Array1<double>.From([0.0, 1.0]);
        var aMults = NCollection_Array1<int>.From([thePoles.Length, thePoles.Length]);
        return new BSplineCurve3D(NCollection_Array1<Point3D>.From(thePoles), aKnots, aMults, thePoles.Length - 1);
    }

    private static Wire Wire(params Edge[] theEdges)
    {
        var aB = new BrepBuilder();
        Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (Edge anE in theEdges)
        {
            aB.Add(aW, anE);
        }
        return aW;
    }

    // Two G1 B-splines (0,0)-(2,1) and (2,1)-(4,0) on z = 0.
    private static (BSplineCurve3D, BSplineCurve3D) G1BSplines()
        => (BSpline(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0)),
            BSpline(new Point3D(2, 1, 0), new Point3D(3, 1, 0), new Point3D(4, 0, 0)));

    [Fact]
    public void Different_BSplines_are_not_fused_without_SetConcatBSpl()
    {
        (BSplineCurve3D aC1, BSplineCurve3D aC2) = G1BSplines();
        Vertex aV0 = TestShapes.Vertex(aC1.Value(0)), aVm = TestShapes.Vertex(aC1.Value(1)), aV2 = TestShapes.Vertex(aC2.Value(1));
        Wire aW = TestShapes.Frozen(Wire(TestShapes.Edge(aC1, 0, 1, aV0, aVm), TestShapes.Edge(aC2, 0, 1, aVm, aV2)));

        var aFuse = new FuseEdges(aW);
        Assert.Equal(0, aFuse.NbVertices());
        Assert.True(aFuse.Shape().IsEqual(aW));
    }

    [Fact]
    public void G1_BSplines_of_a_wire_are_concatenated_with_SetConcatBSpl()
    {
        (BSplineCurve3D aC1, BSplineCurve3D aC2) = G1BSplines();
        Vertex aV0 = TestShapes.Vertex(aC1.Value(0)), aVm = TestShapes.Vertex(aC1.Value(1)), aV2 = TestShapes.Vertex(aC2.Value(1));
        Wire aW = TestShapes.Frozen(Wire(TestShapes.Edge(aC1, 0, 1, aV0, aVm), TestShapes.Edge(aC2, 0, 1, aVm, aV2)));
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aW);

        var aFuse = new FuseEdges(aW);
        aFuse.SetConcatBSpl();
        Assert.Equal(1, aFuse.NbVertices());
        Shape aResult = aFuse.Shape();
        Assert.Equal(global::Kerf.Topology.ShapeType.Wire, aResult.Type);
        Assert.Equal(1, Count(aResult, ShapeType.Edge));

        Edge anE = ShapeCasts.Edge(Assert.Single(aFuse.ResultEdges().Values));
        BSplineCurve3D aC = Assert.IsType<BSplineCurve3D>(Tool.Curve(anE, out double f, out double l));
        Assert.True(aC.IsFrozen);
        Assert.True(aC.Value(f).DistanceTo(new Point3D(0, 0, 0)) < Conf);
        Assert.True(aC.Value(l).DistanceTo(new Point3D(4, 0, 0)) < Conf);
        // the junction is on the concatenated curve
        var anExt = new ProjectPointOnCurve(new Point3D(2, 1, 0), aC);
        Assert.True(anExt.LowerDistance() < Conf);
        aBefore.AssertUnchanged(aW);
    }

    [Fact]
    public void Edges_of_one_BSpline_are_fused_without_SetConcatBSpl()
    {
        BSplineCurve3D aC = BSpline(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(3, -1, 1), new Point3D(4, 0, 0));
        Vertex aV0 = TestShapes.Vertex(aC.Value(0)), aVm = TestShapes.Vertex(aC.Value(0.4)), aV1 = TestShapes.Vertex(aC.Value(1));
        Wire aW = TestShapes.Frozen(Wire(TestShapes.Edge(aC, 0, 0.4, aV0, aVm), TestShapes.Edge(aC, 0.4, 1, aVm, aV1)));

        var aFuse = new FuseEdges(aW);
        Assert.Equal(1, aFuse.NbVertices());
        Edge anE = ShapeCasts.Edge(Assert.Single(aFuse.ResultEdges().Values));
        Assert.Same(aC, Tool.Curve(anE, out double f, out double l));
        Assert.Equal(0.0, f);
        Assert.Equal(1.0, l);
    }

    [Fact]
    public void A_closed_periodic_BSpline_is_refused_from_its_vertex_without_changing_the_input_curve()
    {
        // a periodic quadratic B-spline over [0, 4], cut at 1 and 3: the fused closed edge starts at a vertex that
        // isn't at the curve's origin, so OCCT moves the curve's origin (MakeClosedCurve) - on a copy in the port
        var aPoles = NCollection_Array1<Point3D>.From([new Point3D(0, 0, 0), new Point3D(4, 0, 0), new Point3D(4, 4, 0), new Point3D(0, 4, 0)]);
        var aKnots = NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0, 4.0]);
        var aMults = NCollection_Array1<int>.From([1, 1, 1, 1, 1]);
        var aC = new BSplineCurve3D(aPoles, aKnots, aMults, 2, true);
        Vertex aVa = TestShapes.Vertex(aC.Value(1)), aVb = TestShapes.Vertex(aC.Value(3));
        Wire aW = Wire(TestShapes.Edge(aC, 1, 3, aVa, aVb), TestShapes.Edge(aC, 3, 5, aVb, aVa));
        aW.Closed(true);
        aW = TestShapes.Frozen(aW);
        Assert.True(aC.IsFrozen);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aW);
        double aFirst = aC.FirstParameter;

        var aFuse = new FuseEdges(aW);
        Assert.Equal(1, aFuse.NbVertices());
        Shape aResult = aFuse.Shape();
        Edge anE = ShapeCasts.Edge(Assert.Single(aFuse.ResultEdges().Values));
        Curve3D aNewC = Tool.Curve(anE, out double f, out double l)!;
        Assert.NotSame(aC, aNewC);
        Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
        Assert.True(aV1.IsSame(aV2));
        Assert.True(aNewC.Value(f).DistanceTo(Tool.Pnt(aV1)) < Conf);
        Assert.True(aNewC.Value(l).DistanceTo(Tool.Pnt(aV1)) < Conf);
        Assert.Equal(1, Count(aResult, ShapeType.Edge));

        Assert.Equal(aFirst, aC.FirstParameter);
        aBefore.AssertUnchanged(aW);
    }

    [Fact]
    public void Internal_forms_do_not_freeze()
    {
        Face aF = SixEdgeRectangle(out _, out _);
        var aFuse = new FuseEdges(aF);
        var aResultEdges = new Dictionary<int, Shape>();
        aFuse.ResultEdges(aResultEdges);
        Assert.All(aResultEdges, anItem => Assert.False(anItem.Value.TShape()!.IsFrozen));
        var aFaces = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        aFuse.Faces(aFaces);
        Assert.Single(aFaces);
        Assert.False(aFuse.InternalShape().TShape()!.IsFrozen);
        var anEdges = new Dictionary<int, NCollection_List<Shape>>();
        aFuse.Edges(anEdges);
        Assert.Equal(2, anEdges.Count);
        Assert.True(aFuse.Shape().TShape()!.IsFrozen);
    }

    // Waits for BRepLib.SameParameter (fork S1): the pcurves of the G1 B-splines are concatenated, so OCCT forces
    // SameParameter on the new edge.
    [Fact]
    public void G1_BSplines_of_a_planar_face_are_concatenated_with_their_pcurves()
    {
        (BSplineCurve3D aC1, BSplineCurve3D aC2) = G1BSplines();
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        Vertex aV0 = TestShapes.Vertex(aC1.Value(0)), aVm = TestShapes.Vertex(aC1.Value(1)), aV2 = TestShapes.Vertex(aC2.Value(1));
        Edge anE1 = TestShapes.Edge(aC1, 0, 1, aV0, aVm), anE2 = TestShapes.Edge(aC2, 0, 1, aVm, aV2);
        Edge anE3 = TestShapes.Edge(aV2, aV0);
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, global::Kerf.Engine.Foundation.Precision.Confusion());
        static BSplineCurve2D To2d(BSplineCurve3D theC)
        {
            var aPoles = new NCollection_Array1<Point2D>(1, theC.PoleCount);
            for (int i = 1; i <= theC.PoleCount; i++)
            {
                aPoles[i] = new Point2D(theC.Pole(i).X, theC.Pole(i).Y);
            }
            return new BSplineCurve2D(aPoles, NCollection_Array1<double>.From([0.0, 1.0]),
                                           NCollection_Array1<int>.From([theC.PoleCount, theC.PoleCount]), theC.Degree);
        }
        aB.UpdateEdge(anE1, To2d(aC1), aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.UpdateEdge(anE2, To2d(aC2), aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.UpdateEdge(anE3, new LineCurve2D(new Point2D(4, 0), new Direction2D(-1, 0)), aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        Wire aW = Wire(anE1, anE2, anE3);
        aW.Closed(true);
        aB.Add(aF, aW);
        aF = TestShapes.Frozen(aF);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aF);

        var aFuse = new FuseEdges(aF);
        aFuse.SetConcatBSpl();
        Shape aResult = aFuse.Shape();
        Assert.Equal(2, Count(aResult, ShapeType.Edge));
        Edge anE = ShapeCasts.Edge(Assert.Single(aFuse.ResultEdges().Values));
        Assert.True(Tool.SameParameter(anE));
        Curve2D aPC = Tool.CurveOnSurface(anE, ShapeCasts.Face(aResult), out double pf, out double pl)!;
        Assert.IsType<BSplineCurve2D>(aPC);
        Tool.Range(anE, out double f, out double l);
        Assert.Equal(f, pf, 1e-9);
        Assert.Equal(l, pl, 1e-9);
        aBefore.AssertUnchanged(aF);
    }
}
