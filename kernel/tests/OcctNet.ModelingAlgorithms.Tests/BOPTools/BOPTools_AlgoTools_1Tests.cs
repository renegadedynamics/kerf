// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.TestHarness;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests.BOPTools;

/// <summary>
/// BOPTools_AlgoTools' tolerance corrections (BOPTools_AlgoTools_1.cxx) on unfrozen copies of data/occ shapes whose
/// tolerances were shrunk: the tolerances grow back to cover the deviations, serially and in parallel alike.
/// </summary>
public class BOPTools_AlgoTools_1Tests
{
    private const double theShrunk = 1.0e-10;

    public static TheoryData<string> Files() => ["bottle.brep", "Pump_Nut.brep", "mal_ecrou.brep", "face1.brep", "wing.brep"];

    // An unfrozen copy of the data/occ shape with every edge and vertex tolerance set to 1e-10 (read as 1e-7).
    private static Shape ShrunkCopy(string theFile)
    {
        Shape aShape = BrepTools.ExactCopyForWrite(BRepMeshModelKit.ReadDataOcc(theFile), out _);
        for (Explorer anExp = new(aShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            ((BrepTEdge)anExp.Current().TShape()!).Tolerance(theShrunk);
        }
        for (Explorer anExp = new(aShape, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            ((BrepTVertex)anExp.Current().TShape()!).Tolerance(theShrunk);
        }
        return aShape;
    }

    private static NCollection_IndexedMap<Shape> EmptyMap() => new(ShapeHasher.Instance);

    [Theory]
    [MemberData(nameof(Files))]
    public void CorrectTolerances_grows_edges_to_cover_their_pcurves(string theFile)
    {
        Shape aShape = ShrunkCopy(theFile);
        bool aWasValid = new Analyzer(aShape).IsValid();
        AlgoTools.CorrectTolerances(aShape, EmptyMap(), 1.0, true);
        // "in terms of BRepCheck": the analyzer runs the same BRepLib_ValidateEdge (22 samples, not exact)
        Assert.True(new Analyzer(aShape).IsValid(), theFile);
        int aChecked = 0, aGrown = 0;
        for (Explorer aFExp = new(aShape, ShapeType.Face); aFExp.More(); aFExp.Next())
        {
            Face aF = ShapeCasts.Face(aFExp.Current());
            for (Explorer anEExp = new(aF, ShapeType.Edge); anEExp.More(); anEExp.Next())
            {
                Edge aE = ShapeCasts.Edge(anEExp.Current());
                if (Tool.Degenerated(aE) || !AlgoTools.ComputeTolerance(aF, aE, out double aMaxDist, out _))
                {
                    continue;
                }
                double aTol = Tool.Tolerance(aE);
                // the exact deviation can exceed the sampled one several times (bottle.brep: 1.74e-5 against 4.26e-6),
                // but not by orders of magnitude
                Assert.True(aTol >= 0.1 * aMaxDist || aMaxDist < global::Kerf.Engine.Foundation.Precision.Confusion(), $"{theFile}: tolerance {aTol} below deviation {aMaxDist}");
                aChecked++;
                if (aTol > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    aGrown++;
                }
            }
        }
        Assert.True(aChecked > 0);
        Assert.True(aWasValid || aGrown > 0, $"{theFile}: invalid, and no tolerance grew");
    }

    [Theory]
    [MemberData(nameof(Files))]
    public void CorrectPointOnCurve_covers_the_curve_ends(string theFile)
    {
        Shape aShape = ShrunkCopy(theFile);
        AlgoTools.CorrectPointOnCurve(aShape, EmptyMap(), 1.0);
        for (Explorer anEExp = new(aShape, ShapeType.Edge); anEExp.More(); anEExp.Next())
        {
            Edge aE = ShapeCasts.Edge(anEExp.Current());
            Curve3D? aC = Tool.Curve(aE, out double aT1, out double aT2);
            if (aC is null)
            {
                continue;
            }
            Exploration.Vertices(aE, out Vertex aV1, out Vertex aV2);
            Assert.True(Tool.Pnt(aV1).Distance(aC.Value(aT1)) <= Tool.Tolerance(aV1), theFile);
            Assert.True(Tool.Pnt(aV2).Distance(aC.Value(aT2)) <= Tool.Tolerance(aV2), theFile);
        }
    }

    [Theory]
    [MemberData(nameof(Files))]
    public void CorrectShapeTolerances_orders_face_edge_and_vertex_tolerances(string theFile)
    {
        Shape aShape = ShrunkCopy(theFile);
        // one edge with a large tolerance: its vertices must follow it
        Edge aBig = ShapeCasts.Edge(new Explorer(aShape, ShapeType.Edge).Current());
        ((BrepTEdge)aBig.TShape()!).Tolerance(1.0e-3);
        AlgoTools.CorrectShapeTolerances(aShape, EmptyMap(), true);
        Exploration.Vertices(aBig, out Vertex aB1, out Vertex aB2);
        Assert.True(Tool.Tolerance(aB1) >= 1.0e-3 && Tool.Tolerance(aB2) >= 1.0e-3, theFile);
        // every edge at least its faces' tolerance (the vertices are raised to the edges' tolerances before that pass)
        for (Explorer aFExp = new(aShape, ShapeType.Face); aFExp.More(); aFExp.Next())
        {
            Face aF = ShapeCasts.Face(aFExp.Current());
            double aTolF = Tool.Tolerance(aF);
            for (Explorer anEExp = new(aF, ShapeType.Edge); anEExp.More(); anEExp.Next())
            {
                Assert.True(Tool.Tolerance(ShapeCasts.Edge(anEExp.Current())) >= aTolF, theFile);
            }
        }
    }

    [Fact]
    public void The_map_to_avoid_and_the_maximal_tolerance_are_respected()
    {
        Shape aShape = ShrunkCopy("bottle.brep");
        NCollection_IndexedMap<Shape> aMap = EmptyMap();
        Exploration.MapShapes(aShape, ShapeType.Edge, aMap);
        Exploration.MapShapes(aShape, ShapeType.Vertex, aMap);
        AlgoTools.CorrectTolerances(aShape, aMap, 1.0);
        AlgoTools.CorrectShapeTolerances(aShape, aMap);
        foreach (Shape aS in aMap)
        {
            // BRep_Tool.Tolerance reads at least Precision::Confusion
            Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aS.ShapeType() == ShapeType.Edge ? Tool.Tolerance(ShapeCasts.Edge(aS)) : Tool.Tolerance(ShapeCasts.Vertex(aS)));
        }
        // a maximal tolerance below every needed one changes nothing
        Shape aShape2 = ShrunkCopy("bottle.brep");
        AlgoTools.CorrectTolerances(aShape2, EmptyMap(), 1.0e-12);
        for (Explorer anExp = new(aShape2, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Tool.Tolerance(ShapeCasts.Edge(anExp.Current())));
        }
    }

    [Theory]
    [MemberData(nameof(Files))]
    public void Corrections_in_parallel_give_the_serial_result(string theFile)
    {
        foreach (bool aParallel in new[] { false, true })
        {
            DeterminismHarness.AssertSerialEqualsParallel(() =>
            {
                Shape aShape = ShrunkCopy(theFile);
                AlgoTools.CorrectTolerances(aShape, EmptyMap(), 1.0, aParallel);
                AlgoTools.CorrectShapeTolerances(aShape, EmptyMap(), aParallel);
                return BRepTextComparison.Write(aShape);
            }, 2);
        }
        // and the flag doesn't change the result
        string Run(bool theParallel)
        {
            Shape aShape = ShrunkCopy(theFile);
            AlgoTools.CorrectTolerances(aShape, EmptyMap(), 1.0, theParallel);
            AlgoTools.CorrectShapeTolerances(aShape, EmptyMap(), theParallel);
            return BRepTextComparison.Write(aShape);
        }
        Assert.Equal(Run(false), Run(true));
    }

    [Fact]
    public void A_frozen_shape_is_not_written()
    {
        Shape aShape = ShrunkCopy("bottle.brep");
        aShape.TShape()!.FreezeGraph();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => AlgoTools.CorrectTolerances(aShape, EmptyMap(), 1.0));
    }

    [Fact]
    public void ComputeTolerance_matches_BRepLib_CheckCurveOnSurface()
    {
        Shape aShape = BRepMeshModelKit.ReadDataOcc("Pump_Nut.brep");
        Face aF = ShapeCasts.Face(new Explorer(aShape, ShapeType.Face).Current());
        Edge aE = ShapeCasts.Edge(new Explorer(aF, ShapeType.Edge).Current());
        Assert.True(AlgoTools.ComputeTolerance(aF, aE, out double aD, out double aT));
        LibraryCheckCurveOnSurface aCS = new(aE, aF);
        aCS.Perform();
        Assert.Equal(aCS.MaxDistance(), aD);
        Assert.Equal(aCS.MaxParameter(), aT);
    }
}

/// <summary>BOPTools_Set: equality as sets of sub-shapes, and a hash that agrees with it.</summary>
public class BOPTools_SetTests
{
    [Fact]
    public void Sets_of_the_same_sub_shapes_are_equal()
    {
        Shape aBox = new MakeBox(1, 2, 3).Shape();
        Shape aReversed = aBox.Reversed();
        ToolSet aS1 = new(), aS2 = new(), aS3 = new();
        aS1.Add(aBox, ShapeType.Face);
        aS2.Add(aReversed, ShapeType.Face);
        aS3.Add(aBox, ShapeType.Edge);
        Assert.Equal(6, aS1.ShapeCount);
        Assert.Equal(24, aS3.ShapeCount); // each edge once per face
        Assert.True(aS1.IsEqual(aS2));
        Assert.Equal(aS1, aS2);
        Assert.Equal(aS1.GetHashCode(), aS2.GetHashCode());
        Assert.Equal(aS1.GetSum(), aS2.GetSum());
        Assert.False(aS1.IsEqual(aS3));
        Assert.True(aS1.Shape().IsEqual(aBox));

        Shape anOther = new MakeBox(1, 2, 3).Shape();
        ToolSet aS4 = new();
        aS4.Add(anOther, ShapeType.Face);
        Assert.False(aS1.IsEqual(aS4));

        ToolSet aCopy = new(aS1);
        Assert.True(aCopy.IsEqual(aS1));
        aCopy.Add(anOther, ShapeType.Face);
        Assert.True(aS1.IsEqual(aS2)); // the copy doesn't share the list
        ToolSet anAssigned = new();
        anAssigned.Assign(aS3);
        Assert.Equal(aS3, anAssigned);

        NCollection_IndexedDataMap<ToolSet, Shape> aMap = new(EqualityComparer<ToolSet>.Default);
        aMap.Add(aS1, aBox);
        Assert.True(aMap.Contains(aS2));
        Assert.False(aMap.Contains(aS4));
    }

    [Fact]
    public void Degenerated_edges_are_left_out_and_internal_shapes_count_twice()
    {
        Shape aSphere = new MakeSphere(2).Shape();
        ToolSet aS = new();
        aS.Add(aSphere, ShapeType.Edge);
        // the sphere's seam twice (FORWARD and REVERSED in its face), its two degenerated poles left out
        Assert.Equal(2, aS.ShapeCount);

        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Compound aC = new();
        BrepBuilder aBB = new();
        aBB.MakeCompound(ref aC);
        aBB.Add(aC, anEdge.Oriented(Orientation.Internal));
        ToolSet aSI = new();
        aSI.Add(aC, ShapeType.Edge);
        Assert.Equal(2, aSI.ShapeCount);

        ToolSet anEmpty = new();
        anEmpty.Add(aC, ShapeType.Face);
        Assert.Equal(0, anEmpty.ShapeCount);
        Assert.Equal(0UL, anEmpty.GetSum());
    }
}
