// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepLib_FuseEdges on frozen inputs: collinear segments of a planar face, circle arcs on a cylinder
/// face, B-splines of the same curve and G1 B-splines (SetConcatBSpl), a closed periodic B-spline (whose curve the
/// algorithm must not change in place), avoided edges, located shapes. Results are frozen, inputs unchanged.
/// </summary>
public class BRepLib_FuseEdgesTests
{
    private const double Conf = 1e-7;

    private static int Count(in TopoDS_Shape S, TopAbs_ShapeEnum T)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, T, aMap);
        return aMap.Extent();
    }

    // [0,2] x [0,1] on z = 0, six edges: A(0,0) B(1,0) C(2,0) D(2,1) E(1,1) F(0,1); AB-BC and DE-EF collinear.
    private static TopoDS_Face SixEdgeRectangle(out TopoDS_Vertex[] theVertices, out Dictionary<(int, int), TopoDS_Edge> theEdges)
    {
        theVertices =
        [
            TestShapes.Vertex(new gp_Pnt(0, 0, 0)), TestShapes.Vertex(new gp_Pnt(1, 0, 0)),
            TestShapes.Vertex(new gp_Pnt(2, 0, 0)), TestShapes.Vertex(new gp_Pnt(2, 1, 0)),
            TestShapes.Vertex(new gp_Pnt(1, 1, 0)), TestShapes.Vertex(new gp_Pnt(0, 1, 0)),
        ];
        theEdges = [];
        return TestShapes.Frozen(TestShapes.PolygonFace(theVertices, new gp_Dir(0, 0, 1), theEdges, [0, 1, 2, 3, 4, 5]));
    }

    [Fact]
    public void Collinear_segments_of_a_planar_face_are_fused()
    {
        TopoDS_Face aF = SixEdgeRectangle(out TopoDS_Vertex[] aV, out _);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aF);

        var aFuse = new BRepLib_FuseEdges(aF);
        Assert.Equal(2, aFuse.NbVertices());

        IReadOnlyDictionary<int, ISchematiqList<TopoDS_Shape>> anEdges = aFuse.Edges();
        Assert.Equal(2, anEdges.Count);
        Assert.All(anEdges.Values, aList => Assert.Equal(2, aList.Count));
        Assert.All(anEdges.Values, aList => Assert.All(aList, anE => Assert.True(anE.TShape()!.IsFrozen())));

        IReadOnlyDictionary<int, TopoDS_Shape> aResultEdges = aFuse.ResultEdges();
        Assert.Equal(2, aResultEdges.Count);
        foreach (TopoDS_Shape aS in aResultEdges.Values)
        {
            TopoDS_Edge anE = TopoDS.Edge(aS);
            Assert.True(anE.TShape()!.IsFrozen());
            Geom_Curve aC = BRep_Tool.Curve(anE, out double f, out double l)!;
            Assert.IsType<Geom_Line>(aC);
            Assert.Equal(2.0, l - f, 1e-12);
            // the pcurve on the face's plane is the old edge's, over the new range
            Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(anE, aF, out double pf, out double pl)!;
            Assert.IsType<Geom2d_Line>(aPC);
            Assert.Equal(2.0, pl - pf, 1e-12);
            gp_Pnt aP3 = aC.Value(f);
            gp_Pnt2d aP2 = aPC.Value(pf);
            gp_Pnt aOnPlane = BRep_Tool.Surface(aF)!.Value(aP2.X(), aP2.Y());
            Assert.True(aP3.Distance(aOnPlane) < Conf);
        }

        TopoDS_Shape aResult = aFuse.Shape();
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.Equal(TopAbs_FACE, aResult.ShapeType());
        Assert.Equal(4, Count(aResult, TopAbs_EDGE));
        Assert.Equal(4, Count(aResult, TopAbs_VERTEX));
        var aVertices = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, TopAbs_VERTEX, aVertices);
        Assert.False(aVertices.Contains(aV[1]));
        Assert.False(aVertices.Contains(aV[4]));
        Assert.True(aVertices.Contains(aV[0]) && aVertices.Contains(aV[2]) && aVertices.Contains(aV[3])
                    && aVertices.Contains(aV[5]));

        IReadOnlyDictionary<TopoDS_Shape, TopoDS_Shape> aFaces = aFuse.Faces();
        Assert.Single(aFaces);
        Assert.True(aFaces.ContainsKey(aF));
        Assert.True(aFaces[aF].IsSame(aResult));

        // the area is unchanged
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(aResult, aProps);
        Assert.Equal(2.0, Math.Abs(aProps.Mass()), 1e-10);

        aBefore.AssertUnchanged(aF);
    }

    [Fact]
    public void Avoided_edges_are_not_fused()
    {
        TopoDS_Face aF = SixEdgeRectangle(out _, out Dictionary<(int, int), TopoDS_Edge> anEdges);
        var anAvoid = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        anAvoid.Add(anEdges[(0, 1)]);

        var aFuse = new BRepLib_FuseEdges(aF, true);
        aFuse.AvoidEdges(anAvoid);
        anAvoid.Clear(); // the algorithm keeps its own copy
        Assert.Equal(1, aFuse.NbVertices());
        TopoDS_Shape aResult = aFuse.Shape();
        Assert.Equal(5, Count(aResult, TopAbs_EDGE));
        var aResultEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, TopAbs_EDGE, aResultEdges);
        Assert.True(aResultEdges.Contains(anEdges[(0, 1)]));
        Assert.True(aResultEdges.Contains(anEdges[(1, 2)]));
    }

    [Fact]
    public void A_located_face_is_fused_in_place()
    {
        TopoDS_Face aF0 = SixEdgeRectangle(out _, out _);
        var aLoc = new TopLoc_Location(gp_Trsf.Rotation(new gp_Ax1(new gp_Pnt(1, 2, 3), new gp_Dir(0, 1, 1)), 0.7));
        TopoDS_Face aF = TopoDS.Face(aF0.Moved(aLoc));
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aF);

        var aFuse = new BRepLib_FuseEdges(aF);
        TopoDS_Shape aResult = aFuse.Shape();
        Assert.Equal(4, Count(aResult, TopAbs_EDGE));

        var aProps0 = new GProp_GProps();
        BRepGProp.SurfaceProperties(aF, aProps0);
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(aResult, aProps);
        Assert.Equal(aProps0.Mass(), aProps.Mass(), 1e-10);
        Assert.True(aProps0.CentreOfMass().Distance(aProps.CentreOfMass()) < 1e-9);

        // each fused edge's 3D ends are its vertices
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            var aC = new BRepAdaptor_Curve(anE);
            TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            Assert.True(aC.Value(aC.FirstParameter()).Distance(BRep_Tool.Pnt(aV1)) < Conf);
            Assert.True(aC.Value(aC.LastParameter()).Distance(BRep_Tool.Pnt(aV2)) < Conf);
        }
        aBefore.AssertUnchanged(aF);
    }

    [Fact]
    public void A_shape_without_collinear_edges_is_returned_as_is()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aFuse = new BRepLib_FuseEdges(aBox);
        Assert.Equal(0, aFuse.NbVertices());
        Assert.Empty(aFuse.Edges());
        Assert.Empty(aFuse.ResultEdges());
        Assert.Empty(aFuse.Faces());
        Assert.True(aFuse.Shape().IsEqual(aBox));
    }

    [Fact]
    public void Null_shape_throws()
    {
        Assert.ThrowsAny<Standard_NullObject>(() => new BRepLib_FuseEdges(new TopoDS_Shape()));
    }

    [Fact]
    public void Circle_arcs_on_a_cylinder_face_are_fused_into_the_circle()
    {
        // the cylinder's lateral face with its bottom circle in two arcs [0, pi] and [pi, 2 pi]
        var aB = new BRep_Builder();
        double r = 2, h = 3, aTwoPi = 2 * Math.PI;
        var aSurf = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r);
        TopoDS_Vertex aV0 = TestShapes.Vertex(new gp_Pnt(r, 0, 0));
        TopoDS_Vertex aVPi = TestShapes.Vertex(new gp_Pnt(-r, 0, 0));
        TopoDS_Vertex aV1 = TestShapes.Vertex(new gp_Pnt(r, 0, h));
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r);
        TopoDS_Edge anArc1 = TestShapes.Edge(aCircle, 0, Math.PI, aV0, aVPi);
        TopoDS_Edge anArc2 = TestShapes.Edge(new Geom_Circle(aCircle.Circ()), Math.PI, aTwoPi, aVPi, aV0);
        TopoDS_Edge aTop = TestShapes.Edge(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, h), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r),
                                           0, aTwoPi, aV1, aV1);
        TopoDS_Edge aSeam = TestShapes.Edge(new Geom_Line(new gp_Pnt(r, 0, 0), new gp_Dir(0, 0, 1)), 0, h, aV0, aV1);
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aSurf, Precision.Confusion());
        var aBottom2d = new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        aB.UpdateEdge(anArc1, aBottom2d, aF, Precision.Confusion());
        aB.UpdateEdge(anArc2, aBottom2d, aF, Precision.Confusion());
        aB.UpdateEdge(aTop, new Geom2d_Line(new gp_Pnt2d(0, h), new gp_Dir2d(1, 0)), aF, Precision.Confusion());
        aB.UpdateEdge(aSeam, new Geom2d_Line(new gp_Pnt2d(aTwoPi, 0), new gp_Dir2d(0, 1)),
                      new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1)), aF, Precision.Confusion());
        TopoDS_Wire aW = new();
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

        var aFuse = new BRepLib_FuseEdges(aF);
        Assert.Equal(1, aFuse.NbVertices());
        TopoDS_Shape aResult = aFuse.Shape();
        Assert.Equal(3, Count(aResult, TopAbs_EDGE));
        Assert.Equal(2, Count(aResult, TopAbs_VERTEX));

        TopoDS_Edge aFused = TopoDS.Edge(Assert.Single(aFuse.ResultEdges().Values));
        Geom_Curve aC = BRep_Tool.Curve(aFused, out double f, out double l)!;
        Assert.IsType<Geom_Circle>(aC);
        Assert.Equal(aTwoPi, l - f, 1e-12);
        Assert.True(BRep_Tool.IsClosed(aFused));
        Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(aFused, TopoDS.Face(aResult), out double pf, out double pl)!;
        Assert.Equal(f, pf);
        Assert.Equal(l, pl);
        Assert.Equal(0.0, aPC.Value(pf).X(), 1e-12);
        Assert.Equal(0.0, aPC.Value(pf).Y(), 1e-12);

        var aProps0 = new GProp_GProps();
        BRepGProp.SurfaceProperties(aF, aProps0);
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(aResult, aProps);
        Assert.Equal(aProps0.Mass(), aProps.Mass(), 1e-9);
        aBefore.AssertUnchanged(aF);
    }

    private static Geom_BSplineCurve BSpline(params gp_Pnt[] thePoles)
    {
        var aKnots = NCollection_Array1<double>.From([0.0, 1.0]);
        var aMults = NCollection_Array1<int>.From([thePoles.Length, thePoles.Length]);
        return new Geom_BSplineCurve(NCollection_Array1<gp_Pnt>.From(thePoles), aKnots, aMults, thePoles.Length - 1);
    }

    private static TopoDS_Wire Wire(params TopoDS_Edge[] theEdges)
    {
        var aB = new BRep_Builder();
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (TopoDS_Edge anE in theEdges)
        {
            aB.Add(aW, anE);
        }
        return aW;
    }

    // Two G1 B-splines (0,0)-(2,1) and (2,1)-(4,0) on z = 0.
    private static (Geom_BSplineCurve, Geom_BSplineCurve) G1BSplines()
        => (BSpline(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(2, 1, 0)),
            BSpline(new gp_Pnt(2, 1, 0), new gp_Pnt(3, 1, 0), new gp_Pnt(4, 0, 0)));

    [Fact]
    public void Different_BSplines_are_not_fused_without_SetConcatBSpl()
    {
        (Geom_BSplineCurve aC1, Geom_BSplineCurve aC2) = G1BSplines();
        TopoDS_Vertex aV0 = TestShapes.Vertex(aC1.Value(0)), aVm = TestShapes.Vertex(aC1.Value(1)), aV2 = TestShapes.Vertex(aC2.Value(1));
        TopoDS_Wire aW = TestShapes.Frozen(Wire(TestShapes.Edge(aC1, 0, 1, aV0, aVm), TestShapes.Edge(aC2, 0, 1, aVm, aV2)));

        var aFuse = new BRepLib_FuseEdges(aW);
        Assert.Equal(0, aFuse.NbVertices());
        Assert.True(aFuse.Shape().IsEqual(aW));
    }

    [Fact]
    public void G1_BSplines_of_a_wire_are_concatenated_with_SetConcatBSpl()
    {
        (Geom_BSplineCurve aC1, Geom_BSplineCurve aC2) = G1BSplines();
        TopoDS_Vertex aV0 = TestShapes.Vertex(aC1.Value(0)), aVm = TestShapes.Vertex(aC1.Value(1)), aV2 = TestShapes.Vertex(aC2.Value(1));
        TopoDS_Wire aW = TestShapes.Frozen(Wire(TestShapes.Edge(aC1, 0, 1, aV0, aVm), TestShapes.Edge(aC2, 0, 1, aVm, aV2)));
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aW);

        var aFuse = new BRepLib_FuseEdges(aW);
        aFuse.SetConcatBSpl();
        Assert.Equal(1, aFuse.NbVertices());
        TopoDS_Shape aResult = aFuse.Shape();
        Assert.Equal(TopAbs_WIRE, aResult.ShapeType());
        Assert.Equal(1, Count(aResult, TopAbs_EDGE));

        TopoDS_Edge anE = TopoDS.Edge(Assert.Single(aFuse.ResultEdges().Values));
        Geom_BSplineCurve aC = Assert.IsType<Geom_BSplineCurve>(BRep_Tool.Curve(anE, out double f, out double l));
        Assert.True(aC.IsFrozen());
        Assert.True(aC.Value(f).Distance(new gp_Pnt(0, 0, 0)) < Conf);
        Assert.True(aC.Value(l).Distance(new gp_Pnt(4, 0, 0)) < Conf);
        // the junction is on the concatenated curve
        var anExt = new GeomAPI_ProjectPointOnCurve(new gp_Pnt(2, 1, 0), aC);
        Assert.True(anExt.LowerDistance() < Conf);
        aBefore.AssertUnchanged(aW);
    }

    [Fact]
    public void Edges_of_one_BSpline_are_fused_without_SetConcatBSpl()
    {
        Geom_BSplineCurve aC = BSpline(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 2, 0), new gp_Pnt(3, -1, 1), new gp_Pnt(4, 0, 0));
        TopoDS_Vertex aV0 = TestShapes.Vertex(aC.Value(0)), aVm = TestShapes.Vertex(aC.Value(0.4)), aV1 = TestShapes.Vertex(aC.Value(1));
        TopoDS_Wire aW = TestShapes.Frozen(Wire(TestShapes.Edge(aC, 0, 0.4, aV0, aVm), TestShapes.Edge(aC, 0.4, 1, aVm, aV1)));

        var aFuse = new BRepLib_FuseEdges(aW);
        Assert.Equal(1, aFuse.NbVertices());
        TopoDS_Edge anE = TopoDS.Edge(Assert.Single(aFuse.ResultEdges().Values));
        Assert.Same(aC, BRep_Tool.Curve(anE, out double f, out double l));
        Assert.Equal(0.0, f);
        Assert.Equal(1.0, l);
    }

    [Fact]
    public void A_closed_periodic_BSpline_is_refused_from_its_vertex_without_changing_the_input_curve()
    {
        // a periodic quadratic B-spline over [0, 4], cut at 1 and 3: the fused closed edge starts at a vertex that
        // isn't at the curve's origin, so OCCT moves the curve's origin (MakeClosedCurve) - on a copy in the port
        var aPoles = NCollection_Array1<gp_Pnt>.From([new gp_Pnt(0, 0, 0), new gp_Pnt(4, 0, 0), new gp_Pnt(4, 4, 0), new gp_Pnt(0, 4, 0)]);
        var aKnots = NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0, 4.0]);
        var aMults = NCollection_Array1<int>.From([1, 1, 1, 1, 1]);
        var aC = new Geom_BSplineCurve(aPoles, aKnots, aMults, 2, true);
        TopoDS_Vertex aVa = TestShapes.Vertex(aC.Value(1)), aVb = TestShapes.Vertex(aC.Value(3));
        TopoDS_Wire aW = Wire(TestShapes.Edge(aC, 1, 3, aVa, aVb), TestShapes.Edge(aC, 3, 5, aVb, aVa));
        aW.Closed(true);
        aW = TestShapes.Frozen(aW);
        Assert.True(aC.IsFrozen());
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aW);
        double aFirst = aC.FirstParameter();

        var aFuse = new BRepLib_FuseEdges(aW);
        Assert.Equal(1, aFuse.NbVertices());
        TopoDS_Shape aResult = aFuse.Shape();
        TopoDS_Edge anE = TopoDS.Edge(Assert.Single(aFuse.ResultEdges().Values));
        Geom_Curve aNewC = BRep_Tool.Curve(anE, out double f, out double l)!;
        Assert.NotSame(aC, aNewC);
        TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        Assert.True(aV1.IsSame(aV2));
        Assert.True(aNewC.Value(f).Distance(BRep_Tool.Pnt(aV1)) < Conf);
        Assert.True(aNewC.Value(l).Distance(BRep_Tool.Pnt(aV1)) < Conf);
        Assert.Equal(1, Count(aResult, TopAbs_EDGE));

        Assert.Equal(aFirst, aC.FirstParameter());
        aBefore.AssertUnchanged(aW);
    }

    [Fact]
    public void Internal_forms_do_not_freeze()
    {
        TopoDS_Face aF = SixEdgeRectangle(out _, out _);
        var aFuse = new BRepLib_FuseEdges(aF);
        var aResultEdges = new NCollection_DataMap<int, TopoDS_Shape>();
        aFuse.ResultEdges(aResultEdges);
        Assert.All(aResultEdges, anItem => Assert.False(anItem.Value.TShape()!.IsFrozen()));
        var aFaces = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        aFuse.Faces(aFaces);
        Assert.Equal(1, aFaces.Extent());
        Assert.False(aFuse.InternalShape().TShape()!.IsFrozen());
        var anEdges = new NCollection_DataMap<int, NCollection_List<TopoDS_Shape>>();
        aFuse.Edges(anEdges);
        Assert.Equal(2, anEdges.Extent());
        Assert.True(aFuse.Shape().TShape()!.IsFrozen());
    }

    // Waits for BRepLib.SameParameter (fork S1): the pcurves of the G1 B-splines are concatenated, so OCCT forces
    // SameParameter on the new edge.
    [Fact]
    public void G1_BSplines_of_a_planar_face_are_concatenated_with_their_pcurves()
    {
        (Geom_BSplineCurve aC1, Geom_BSplineCurve aC2) = G1BSplines();
        var aB = new BRep_Builder();
        var aPlane = new Geom_Plane(new gp_Ax3(gp.XOY()));
        TopoDS_Vertex aV0 = TestShapes.Vertex(aC1.Value(0)), aVm = TestShapes.Vertex(aC1.Value(1)), aV2 = TestShapes.Vertex(aC2.Value(1));
        TopoDS_Edge anE1 = TestShapes.Edge(aC1, 0, 1, aV0, aVm), anE2 = TestShapes.Edge(aC2, 0, 1, aVm, aV2);
        TopoDS_Edge anE3 = TestShapes.Edge(aV2, aV0);
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aPlane, Precision.Confusion());
        static Geom2d_BSplineCurve To2d(Geom_BSplineCurve theC)
        {
            var aPoles = new NCollection_Array1<gp_Pnt2d>(1, theC.NbPoles());
            for (int i = 1; i <= theC.NbPoles(); i++)
            {
                aPoles[i] = new gp_Pnt2d(theC.Pole(i).X(), theC.Pole(i).Y());
            }
            return new Geom2d_BSplineCurve(aPoles, NCollection_Array1<double>.From([0.0, 1.0]),
                                           NCollection_Array1<int>.From([theC.NbPoles(), theC.NbPoles()]), theC.Degree());
        }
        aB.UpdateEdge(anE1, To2d(aC1), aF, Precision.Confusion());
        aB.UpdateEdge(anE2, To2d(aC2), aF, Precision.Confusion());
        aB.UpdateEdge(anE3, new Geom2d_Line(new gp_Pnt2d(4, 0), new gp_Dir2d(-1, 0)), aF, Precision.Confusion());
        TopoDS_Wire aW = Wire(anE1, anE2, anE3);
        aW.Closed(true);
        aB.Add(aF, aW);
        aF = TestShapes.Frozen(aF);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aF);

        var aFuse = new BRepLib_FuseEdges(aF);
        aFuse.SetConcatBSpl();
        TopoDS_Shape aResult = aFuse.Shape();
        Assert.Equal(2, Count(aResult, TopAbs_EDGE));
        TopoDS_Edge anE = TopoDS.Edge(Assert.Single(aFuse.ResultEdges().Values));
        Assert.True(BRep_Tool.SameParameter(anE));
        Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(anE, TopoDS.Face(aResult), out double pf, out double pl)!;
        Assert.IsType<Geom2d_BSplineCurve>(aPC);
        BRep_Tool.Range(anE, out double f, out double l);
        Assert.Equal(f, pf, 1e-9);
        Assert.Equal(l, pl, 1e-9);
        aBefore.AssertUnchanged(aF);
    }
}
