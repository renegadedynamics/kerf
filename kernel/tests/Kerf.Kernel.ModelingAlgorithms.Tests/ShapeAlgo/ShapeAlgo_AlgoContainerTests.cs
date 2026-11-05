// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeAlgo_AlgoContainer, which the STEP reader and writer reach through ShapeAlgo (unit 11I): connecting wires,
/// making wires homogeneous, and approximating degree-1 B-splines, against what its header documents.
/// </summary>
public class ShapeAlgo_AlgoContainerTests
{
    private static readonly AlgoContainer Algo = new();

    private static Edge Segment(double theX1, double theY1, double theX2, double theY2)
        => new MakeEdge(new Point3D(theX1, theY1, 0), new Point3D(theX2, theY2, 0)).Edge();

    private static Wire Polyline(params (double X, double Y)[] thePoints)
    {
        var aData = new ExtendWireData();
        for (int i = 0; i + 1 < thePoints.Length; i++)
        {
            aData.Add(Segment(thePoints[i].X, thePoints[i].Y, thePoints[i + 1].X, thePoints[i + 1].Y));
        }
        return aData.Wire();
    }

    private static List<double> EdgeLengths(in Wire theWire)
    {
        List<double> aLengths = [];
        for (Iterator it = new(theWire); it.More(); it.Next())
        {
            Tool.Curve(ShapeCasts.Edge(it.Value()), out _, out double aFirst, out double aLast);
            aLengths.Add(aLast - aFirst);
        }
        return aLengths;
    }

    // ConnectNextWire: "finds the best way to connect", reversing either if needed

    private static (WireAnalysis Analysis, ExtendWireData Built) Started()
    {
        var aBuilt = new ExtendWireData(Polyline((0, 0), (1, 0)));
        return (new WireAnalysis(aBuilt, new Face(), 1e-7), aBuilt);
    }

    [Fact]
    public void Connecting_an_empty_wire_does_nothing_and_to_an_empty_wire_adds_it()
    {
        (WireAnalysis anAnalysis, ExtendWireData aBuilt) = Started();
        Assert.True(Algo.ConnectNextWire(anAnalysis, new ExtendWireData(), 1, out double aDist, out bool aRev, out bool aNextRev));
        Assert.Equal(1, aBuilt.EdgeCount);
        Assert.Equal((0.0, false, false), (aDist, aRev, aNextRev));

        var anEmpty = new ExtendWireData();
        Assert.True(Algo.ConnectNextWire(new WireAnalysis(anEmpty, new Face(), 1e-7), new ExtendWireData(Polyline((0, 0), (1, 0))), 1, out _, out _, out _));
        Assert.Equal(1, anEmpty.EdgeCount);
    }

    [Fact]
    public void A_wire_starting_at_the_tail_connects_as_it_is()
    {
        (WireAnalysis anAnalysis, ExtendWireData aBuilt) = Started();
        var aNext = new ExtendWireData(Polyline((1, 0), (2, 0)));
        Assert.True(Algo.ConnectNextWire(anAnalysis, aNext, 1, out double aDist, out bool aRev, out bool aNextRev));
        Assert.Equal(2, aBuilt.EdgeCount);
        Assert.Equal(0.0, aDist, 1e-9);
        Assert.False(aRev);
        Assert.False(aNextRev);
    }

    [Fact]
    public void A_wire_ending_at_the_tail_is_reversed()
    {
        (WireAnalysis anAnalysis, ExtendWireData aBuilt) = Started();
        var aNext = new ExtendWireData(Polyline((2, 0), (1, 0)));
        Assert.True(Algo.ConnectNextWire(anAnalysis, aNext, 1, out double aDist, out bool aRev, out bool aNextRev));
        Assert.Equal(2, aBuilt.EdgeCount);
        Assert.Equal(0.0, aDist, 1e-9);
        Assert.False(aRev);
        Assert.True(aNextRev);
    }

    [Fact]
    public void A_wire_at_the_head_reverses_the_built_wire()
    {
        // it starts at the head: the built wire turns round, and the next stays as it is
        (WireAnalysis anAnalysis, ExtendWireData aBuilt) = Started();
        var aNext = new ExtendWireData(Polyline((0, 0), (-1, 0)));
        Assert.True(Algo.ConnectNextWire(anAnalysis, aNext, 1, out double aDist, out bool aRev, out bool aNextRev));
        Assert.Equal(2, aBuilt.EdgeCount);
        Assert.Equal(0.0, aDist, 1e-9);
        Assert.True(aRev);
        Assert.False(aNextRev);

        // it ends at the head: both turn round
        (anAnalysis, aBuilt) = Started();
        aNext = new ExtendWireData(Polyline((-1, 0), (0, 0)));
        Assert.True(Algo.ConnectNextWire(anAnalysis, aNext, 1, out aDist, out aRev, out aNextRev));
        Assert.Equal(2, aBuilt.EdgeCount);
        Assert.Equal(0.0, aDist, 1e-9);
        Assert.True(aRev);
        Assert.True(aNextRev);
    }

    [Fact]
    public void A_wire_beyond_the_tolerance_does_not_connect()
    {
        (WireAnalysis anAnalysis, ExtendWireData aBuilt) = Started();
        var aNext = new ExtendWireData(Polyline((5, 5), (6, 5)));
        Assert.False(Algo.ConnectNextWire(anAnalysis, aNext, 0.1, out _, out _, out _));
        Assert.Equal(1, aBuilt.EdgeCount);
    }

    // HomoWires: "the same number of edges, the edges of the same rank having the same ratio"

    [Fact]
    public void Single_edges_are_already_homogeneous()
    {
        Assert.True(Algo.HomoWires(Polyline((0, 0), (1, 0)), Polyline((0, 1), (5, 1)), out Wire aW1, out Wire aW2, false));
        Assert.Equal([1.0], EdgeLengths(aW1));
        Assert.Equal([5.0], EdgeLengths(aW2));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Homogeneous_wires_have_edges_of_the_same_share(bool theSwapped)
    {
        // shares of the first: 1/2, 1/2; of the second: 1/4, 1/4, 1/2. Together, cut at 1/4, 1/2 and 1
        Wire aShort = Polyline((0, 0), (1, 0), (2, 0));
        Wire aLong = Polyline((0, 1), (1, 1), (2, 1), (4, 1));
        Wire aFirst = theSwapped ? aLong : aShort, aSecond = theSwapped ? aShort : aLong;

        Assert.True(Algo.HomoWires(aFirst, aSecond, out Wire aW1, out Wire aW2, false));

        List<double> aL1 = EdgeLengths(aW1), aL2 = EdgeLengths(aW2);
        Assert.Equal(aL1.Count, aL2.Count);
        Assert.Equal(3, aL1.Count);
        double aSum1 = aL1.Sum(), aSum2 = aL2.Sum();
        Assert.Equal(theSwapped ? 4.0 : 2.0, aSum1, 1e-9);
        Assert.Equal(theSwapped ? 2.0 : 4.0, aSum2, 1e-9);
        double[] aShares = [0.25, 0.25, 0.5];
        for (int i = 0; i < 3; i++)
        {
            Assert.Equal(aShares[i], aL1[i] / aSum1, 1e-9);
            Assert.Equal(aShares[i], aL2[i] / aSum2, 1e-9);
        }
    }

    [Fact]
    public void Homogeneous_wires_of_reversed_edges_cut_each_edge_from_its_other_end()
    {
        // both wires' edges are reversed, so each piece comes off the top of the curve's range
        Edge aE1 = Segment(0, 0, 3, 0), aE2 = Segment(3, 0, 4, 0);
        Edge aE3 = Segment(0, 1, 1, 1), aE4 = Segment(1, 1, 4, 1);
        var aData1 = new ExtendWireData();
        aData1.Add(aE1);
        aData1.Add(aE2);
        aData1.Reverse();
        var aData2 = new ExtendWireData();
        aData2.Add(aE3);
        aData2.Add(aE4);
        aData2.Reverse();

        Assert.True(Algo.HomoWires(aData1.Wire(), aData2.Wire(), out Wire aW1, out Wire aW2, false));

        Assert.Equal(EdgeLengths(aW1).Count, EdgeLengths(aW2).Count);
        Assert.Equal(4.0, EdgeLengths(aW1).Sum(), 1e-9);
        Assert.Equal(4.0, EdgeLengths(aW2).Sum(), 1e-9);
        // shares 3/4, 1/4 against 1/4, 3/4: cut at 1/4, 3/4 and 1
        Assert.Equal(3, EdgeLengths(aW1).Count);
    }

    // ApproxBSplineCurve

    private static Point3D OnCircle(double theU) => new(5 * Math.Cos(theU), 5 * Math.Sin(theU), 0);

    private static BSplineCurve3D DegreeOne(Point3D[] thePoles)
    {
        double[] aKnots = new double[thePoles.Length];
        int[] aMults = new int[thePoles.Length];
        for (int i = 0; i < thePoles.Length; i++)
        {
            aKnots[i] = i;
            aMults[i] = i == 0 || i == thePoles.Length - 1 ? 2 : 1;
        }
        return new BSplineCurve3D(thePoles, aKnots, aMults, 1);
    }

    [Fact]
    public void A_curve_of_higher_degree_comes_back_as_it_is()
    {
        BSplineCurve3D aCurve = new([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)], [0.0, 1.0], [3, 3], 2);
        var aSequence = new NCollection_Sequence<Curve3D>();
        Algo.ApproxBSplineCurve(aCurve, aSequence);
        Assert.Equal(1, aSequence.Length());
        Assert.Same(aCurve, aSequence[1]);
    }

    [Fact]
    public void A_polyline_is_approximated_through_its_ends_and_near_its_points()
    {
        Point3D[] aPoles = Enumerable.Range(0, 11).Select(i => OnCircle(i * 0.1)).ToArray();
        var aSequence = new NCollection_Sequence<Curve3D>();
        Algo.ApproxBSplineCurve(DegreeOne(aPoles), aSequence);

        Assert.True(aSequence.Length() >= 1);
        Curve3D aFirst = aSequence[1], aLast = aSequence[aSequence.Length()];
        Assert.True(aFirst.Value(aFirst.FirstParameter).DistanceTo(aPoles[0]) < 1e-6);
        Assert.True(aLast.Value(aLast.LastParameter).DistanceTo(aPoles[10]) < 1e-6);
        // the arc's sagitta between the points is 5 (1 - cos 0.05), about 0.006: every point of the result is near the circle
        for (int i = 1; i <= aSequence.Length(); i++)
        {
            Curve3D aCurve = aSequence[i];
            for (int k = 0; k <= 10; k++)
            {
                Point3D aPoint = aCurve.Value(aCurve.FirstParameter + k * (aCurve.LastParameter - aCurve.FirstParameter) / 10);
                Assert.Equal(5.0, Math.Sqrt(aPoint.X * aPoint.X + aPoint.Y * aPoint.Y), 0.05);
            }
        }
    }

    [Fact]
    public void A_polyline_is_cut_at_its_repeated_poles()
    {
        // the pole at index 5 is repeated: two parts, meeting there
        Point3D[] aPoles = [.. Enumerable.Range(0, 6).Select(i => OnCircle(i * 0.1)), OnCircle(0.5), .. Enumerable.Range(6, 5).Select(i => OnCircle(i * 0.1))];
        var aSequence = new NCollection_Sequence<Curve3D>();
        Algo.ApproxBSplineCurve(DegreeOne(aPoles), aSequence);

        Assert.True(aSequence.Length() >= 2);
        Curve3D aFirst = aSequence[1], aLast = aSequence[aSequence.Length()];
        Assert.True(aFirst.Value(aFirst.FirstParameter).DistanceTo(aPoles[0]) < 1e-6);
        Assert.True(aLast.Value(aLast.LastParameter).DistanceTo(aPoles[^1]) < 1e-6);
        bool aMeets = false;
        for (int i = 1; i < aSequence.Length(); i++)
        {
            aMeets |= aSequence[i].Value(aSequence[i].LastParameter).DistanceTo(OnCircle(0.5)) < 1e-6;
        }
        Assert.True(aMeets, "no part ends at the repeated pole");
    }

    [Fact]
    public void A_2d_polyline_is_approximated_and_cut_the_same_way()
    {
        Point2D On(double theU) => new(5 * Math.Cos(theU), 5 * Math.Sin(theU));
        Point2D[] aPoles = [.. Enumerable.Range(0, 6).Select(i => On(i * 0.1)), On(0.5), .. Enumerable.Range(6, 5).Select(i => On(i * 0.1))];
        double[] aKnots = Enumerable.Range(0, aPoles.Length).Select(i => (double)i).ToArray();
        int[] aMults = Enumerable.Range(0, aPoles.Length).Select(i => i == 0 || i == aPoles.Length - 1 ? 2 : 1).ToArray();
        var aSequence = new NCollection_Sequence<Curve2D>();

        Algo.ApproxBSplineCurve(new BSplineCurve2D(aPoles, aKnots, aMults, 1), aSequence);

        Assert.True(aSequence.Length() >= 2);
        Curve2D aFirst = aSequence[1], aLast = aSequence[aSequence.Length()];
        Assert.True(aFirst.Value(aFirst.FirstParameter).DistanceTo(aPoles[0]) < 1e-6);
        Assert.True(aLast.Value(aLast.LastParameter).DistanceTo(aPoles[^1]) < 1e-6);

        BSplineCurve2D aQuadratic = new([new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)], [0.0, 1.0], [3, 3], 2);
        Algo.ApproxBSplineCurve(aQuadratic, aSequence);
        Assert.Same(aQuadratic, aSequence[1]);
    }

    // The rest passes on to the healing tools: each by its result

    [Fact]
    public void A_C0_spline_splits_into_C1_pieces_at_its_kink()
    {
        // two quadratic arcs joined with a kink at (2, 0): knot 1 has full multiplicity less one
        BSplineCurve3D aKinked = new([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0), new Point3D(3, 3, 0), new Point3D(4, 0, 0)],
                                     [0.0, 1.0, 2.0], [3, 2, 3], 2);
        NCollection_HSequence<BoundedCurve3D>? aPieces = null;
        Assert.True(Algo.C0BSplineToSequenceOfC1BSplineCurve(aKinked, ref aPieces));
        Assert.Equal(2, aPieces!.Length());
        Assert.True(aPieces[1].EndPoint.IsEqual(new Point3D(2, 0, 0), 1e-9));
        Assert.True(aPieces[2].StartPoint.IsEqual(new Point3D(2, 0, 0), 1e-9));

        BSplineCurve2D aKinked2d = new([new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0), new Point2D(3, 3), new Point2D(4, 0)],
                                       [0.0, 1.0, 2.0], [3, 2, 3], 2);
        NCollection_HSequence<BoundedCurve2D>? aPieces2d = null;
        Assert.True(Algo.C0BSplineToSequenceOfC1BSplineCurve(aKinked2d, ref aPieces2d));
        Assert.Equal(2, aPieces2d!.Length());
    }

    [Fact]
    public void A_shape_on_C1_geometry_is_unchanged_by_conversion_to_C1()
    {
        Shape aBox = new MakeBox(1, 2, 3).Shape();
        Shape aResult = Algo.C0ShapeToC1Shape(aBox, 1e-3);
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(aResult, aProps);
        Assert.Equal(6.0, aProps.Mass(), 1e-9);
        int aFaces = 0;
        for (Explorer ex = new(aResult, ShapeType.Face); ex.More(); ex.Next())
        {
            aFaces++;
        }
        Assert.Equal(6, aFaces);
    }

    [Fact]
    public void A_surface_and_a_curve_convert_to_splines_of_the_same_geometry()
    {
        CylindricalSurface aCylinder = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))), 2.0);
        BSplineSurface? aSurface = Algo.ConvertSurfaceToBSpline(aCylinder, 0, Math.PI / 2, 0, 3);
        Assert.NotNull(aSurface);
        aSurface!.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Point3D aMid = aSurface.Value((aU1 + aU2) / 2, (aV1 + aV2) / 2);
        Assert.Equal(2.0, Math.Sqrt(aMid.X * aMid.X + aMid.Y * aMid.Y), 1e-3);

        CircleCurve3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        BSplineCurve3D? aCurve = Algo.ConvertCurveToBSpline(aCircle, 0, Math.PI / 2, 1e-4, Continuity.C1, 100, 8);
        Assert.NotNull(aCurve);
        Assert.True(aCurve!.StartPoint.IsEqual(new Point3D(2, 0, 0), 1e-4));
        Assert.True(aCurve.EndPoint.IsEqual(new Point3D(0, 2, 0), 1e-4));
    }

    [Fact]
    public void A_faces_outer_wire_and_bounds()
    {
        Plane3D aPlane = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))));
        Wire anOuter = Polyline((0, 0), (10, 0), (10, 5), (0, 5), (0, 0));
        // a hole, wound the other way
        Wire aHole = Polyline((2, 2), (2, 3), (3, 3), (3, 2), (2, 2));
        MakeFace aMaker = new(aPlane, anOuter);
        aMaker.Add(aHole);
        Face aFace = aMaker.Face();

        Assert.True(Algo.OuterWire(aFace).IsSame(anOuter));
        Algo.GetFaceUVBounds(aFace, out double aUMin, out double aUMax, out double aVMin, out double aVMax);
        Assert.Equal(0.0, aUMin, 1e-9);
        Assert.Equal(10.0, aUMax, 1e-9);
        Assert.Equal(0.0, aVMin, 1e-9);
        Assert.Equal(5.0, aVMax, 1e-9);
    }

    [Fact]
    public void A_plane_has_no_periodic_form()
    {
        Assert.Null(Algo.ConvertToPeriodic(new PlaneSurface(new Plane3D(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)))))));
    }

    // The containers

    [Fact]
    public void The_tool_container_makes_its_tools_and_can_be_replaced()
    {
        var aContainer = new AlgoContainer();
        ToolContainer? aTools = aContainer.ToolContainer();
        Assert.NotNull(aTools);
        Assert.IsType<ShapeFixer>(aTools!.FixShape());
        Assert.IsType<EdgeProjAuxFixer>(aTools.EdgeProjAux());
        var aMine = new ToolContainer();
        aContainer.SetToolContainer(aMine);
        Assert.Same(aMine, aContainer.ToolContainer());
    }

    [Fact]
    public void A_container_set_in_a_flow_stays_in_that_flow()
    {
        var aMine = new AlgoContainer();
        AlgoContainer? aBefore = ShapeAlgo.AlgoContainer();
        Kerf.Engine.Foundation.OperationContext aContext = Kerf.Engine.Foundation.OperationContext.Current;
        var aThread = new Thread(() =>
        {
            ShapeAlgo.SetAlgoContainer(aMine);
            Assert.Same(aMine, ShapeAlgo.AlgoContainer());
        });
        aThread.Start();
        aThread.Join();
        Assert.Same(aBefore, ShapeAlgo.AlgoContainer());

        ShapeAlgo.Init();
        Assert.NotNull(ShapeAlgo.AlgoContainer());
    }
}
