// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;

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
    private static TopoDS_Shape ShrunkCopy(string theFile)
    {
        TopoDS_Shape aShape = BRepTools.ExactCopyForWrite(BRepMeshModelKit.ReadDataOcc(theFile), out _);
        for (TopExp_Explorer anExp = new(aShape, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            ((BRep_TEdge)anExp.Current().TShape()!).Tolerance(theShrunk);
        }
        for (TopExp_Explorer anExp = new(aShape, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            ((BRep_TVertex)anExp.Current().TShape()!).Tolerance(theShrunk);
        }
        return aShape;
    }

    private static NCollection_IndexedMap<TopoDS_Shape> EmptyMap() => new(TopTools_ShapeMapHasher.Instance);

    [Theory]
    [MemberData(nameof(Files))]
    public void CorrectTolerances_grows_edges_to_cover_their_pcurves(string theFile)
    {
        TopoDS_Shape aShape = ShrunkCopy(theFile);
        bool aWasValid = new BRepCheck_Analyzer(aShape).IsValid();
        BOPTools_AlgoTools.CorrectTolerances(aShape, EmptyMap(), 1.0, true);
        // "in terms of BRepCheck": the analyzer runs the same BRepLib_ValidateEdge (22 samples, not exact)
        Assert.True(new BRepCheck_Analyzer(aShape).IsValid(), theFile);
        int aChecked = 0, aGrown = 0;
        for (TopExp_Explorer aFExp = new(aShape, TopAbs_FACE); aFExp.More(); aFExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aFExp.Current());
            for (TopExp_Explorer anEExp = new(aF, TopAbs_EDGE); anEExp.More(); anEExp.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(anEExp.Current());
                if (BRep_Tool.Degenerated(aE) || !BOPTools_AlgoTools.ComputeTolerance(aF, aE, out double aMaxDist, out _))
                {
                    continue;
                }
                double aTol = BRep_Tool.Tolerance(aE);
                // the exact deviation can exceed the sampled one several times (bottle.brep: 1.74e-5 against 4.26e-6),
                // but not by orders of magnitude
                Assert.True(aTol >= 0.1 * aMaxDist || aMaxDist < Precision.Confusion(), $"{theFile}: tolerance {aTol} below deviation {aMaxDist}");
                aChecked++;
                if (aTol > Precision.Confusion())
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
        TopoDS_Shape aShape = ShrunkCopy(theFile);
        BOPTools_AlgoTools.CorrectPointOnCurve(aShape, EmptyMap(), 1.0);
        for (TopExp_Explorer anEExp = new(aShape, TopAbs_EDGE); anEExp.More(); anEExp.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(anEExp.Current());
            Geom_Curve? aC = BRep_Tool.Curve(aE, out double aT1, out double aT2);
            if (aC is null)
            {
                continue;
            }
            TopExp.Vertices(aE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            Assert.True(BRep_Tool.Pnt(aV1).Distance(aC.Value(aT1)) <= BRep_Tool.Tolerance(aV1), theFile);
            Assert.True(BRep_Tool.Pnt(aV2).Distance(aC.Value(aT2)) <= BRep_Tool.Tolerance(aV2), theFile);
        }
    }

    [Theory]
    [MemberData(nameof(Files))]
    public void CorrectShapeTolerances_orders_face_edge_and_vertex_tolerances(string theFile)
    {
        TopoDS_Shape aShape = ShrunkCopy(theFile);
        // one edge with a large tolerance: its vertices must follow it
        TopoDS_Edge aBig = TopoDS.Edge(new TopExp_Explorer(aShape, TopAbs_EDGE).Current());
        ((BRep_TEdge)aBig.TShape()!).Tolerance(1.0e-3);
        BOPTools_AlgoTools.CorrectShapeTolerances(aShape, EmptyMap(), true);
        TopExp.Vertices(aBig, out TopoDS_Vertex aB1, out TopoDS_Vertex aB2);
        Assert.True(BRep_Tool.Tolerance(aB1) >= 1.0e-3 && BRep_Tool.Tolerance(aB2) >= 1.0e-3, theFile);
        // every edge at least its faces' tolerance (the vertices are raised to the edges' tolerances before that pass)
        for (TopExp_Explorer aFExp = new(aShape, TopAbs_FACE); aFExp.More(); aFExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aFExp.Current());
            double aTolF = BRep_Tool.Tolerance(aF);
            for (TopExp_Explorer anEExp = new(aF, TopAbs_EDGE); anEExp.More(); anEExp.Next())
            {
                Assert.True(BRep_Tool.Tolerance(TopoDS.Edge(anEExp.Current())) >= aTolF, theFile);
            }
        }
    }

    [Fact]
    public void The_map_to_avoid_and_the_maximal_tolerance_are_respected()
    {
        TopoDS_Shape aShape = ShrunkCopy("bottle.brep");
        NCollection_IndexedMap<TopoDS_Shape> aMap = EmptyMap();
        TopExp.MapShapes(aShape, TopAbs_EDGE, aMap);
        TopExp.MapShapes(aShape, TopAbs_VERTEX, aMap);
        BOPTools_AlgoTools.CorrectTolerances(aShape, aMap, 1.0);
        BOPTools_AlgoTools.CorrectShapeTolerances(aShape, aMap);
        foreach (TopoDS_Shape aS in aMap)
        {
            // BRep_Tool.Tolerance reads at least Precision::Confusion
            Assert.Equal(Precision.Confusion(), aS.ShapeType() == TopAbs_EDGE ? BRep_Tool.Tolerance(TopoDS.Edge(aS)) : BRep_Tool.Tolerance(TopoDS.Vertex(aS)));
        }
        // a maximal tolerance below every needed one changes nothing
        TopoDS_Shape aShape2 = ShrunkCopy("bottle.brep");
        BOPTools_AlgoTools.CorrectTolerances(aShape2, EmptyMap(), 1.0e-12);
        for (TopExp_Explorer anExp = new(aShape2, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.Equal(Precision.Confusion(), BRep_Tool.Tolerance(TopoDS.Edge(anExp.Current())));
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
                TopoDS_Shape aShape = ShrunkCopy(theFile);
                BOPTools_AlgoTools.CorrectTolerances(aShape, EmptyMap(), 1.0, aParallel);
                BOPTools_AlgoTools.CorrectShapeTolerances(aShape, EmptyMap(), aParallel);
                return BRepTextComparison.Write(aShape);
            }, 2);
        }
        // and the flag doesn't change the result
        string Run(bool theParallel)
        {
            TopoDS_Shape aShape = ShrunkCopy(theFile);
            BOPTools_AlgoTools.CorrectTolerances(aShape, EmptyMap(), 1.0, theParallel);
            BOPTools_AlgoTools.CorrectShapeTolerances(aShape, EmptyMap(), theParallel);
            return BRepTextComparison.Write(aShape);
        }
        Assert.Equal(Run(false), Run(true));
    }

    [Fact]
    public void A_frozen_shape_is_not_written()
    {
        TopoDS_Shape aShape = ShrunkCopy("bottle.brep");
        aShape.TShape()!.FreezeGraph();
        Assert.ThrowsAny<Standard_Failure>(() => BOPTools_AlgoTools.CorrectTolerances(aShape, EmptyMap(), 1.0));
    }

    [Fact]
    public void ComputeTolerance_matches_BRepLib_CheckCurveOnSurface()
    {
        TopoDS_Shape aShape = BRepMeshModelKit.ReadDataOcc("Pump_Nut.brep");
        TopoDS_Face aF = TopoDS.Face(new TopExp_Explorer(aShape, TopAbs_FACE).Current());
        TopoDS_Edge aE = TopoDS.Edge(new TopExp_Explorer(aF, TopAbs_EDGE).Current());
        Assert.True(BOPTools_AlgoTools.ComputeTolerance(aF, aE, out double aD, out double aT));
        BRepLib_CheckCurveOnSurface aCS = new(aE, aF);
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
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        TopoDS_Shape aReversed = aBox.Reversed();
        BOPTools_Set aS1 = new(), aS2 = new(), aS3 = new();
        aS1.Add(aBox, TopAbs_FACE);
        aS2.Add(aReversed, TopAbs_FACE);
        aS3.Add(aBox, TopAbs_EDGE);
        Assert.Equal(6, aS1.NbShapes());
        Assert.Equal(24, aS3.NbShapes()); // each edge once per face
        Assert.True(aS1.IsEqual(aS2));
        Assert.Equal(aS1, aS2);
        Assert.Equal(aS1.GetHashCode(), aS2.GetHashCode());
        Assert.Equal(aS1.GetSum(), aS2.GetSum());
        Assert.False(aS1.IsEqual(aS3));
        Assert.True(aS1.Shape().IsEqual(aBox));

        TopoDS_Shape anOther = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        BOPTools_Set aS4 = new();
        aS4.Add(anOther, TopAbs_FACE);
        Assert.False(aS1.IsEqual(aS4));

        BOPTools_Set aCopy = new(aS1);
        Assert.True(aCopy.IsEqual(aS1));
        aCopy.Add(anOther, TopAbs_FACE);
        Assert.True(aS1.IsEqual(aS2)); // the copy doesn't share the list
        BOPTools_Set anAssigned = new();
        anAssigned.Assign(aS3);
        Assert.Equal(aS3, anAssigned);

        NCollection_IndexedDataMap<BOPTools_Set, TopoDS_Shape> aMap = new(EqualityComparer<BOPTools_Set>.Default);
        aMap.Add(aS1, aBox);
        Assert.True(aMap.Contains(aS2));
        Assert.False(aMap.Contains(aS4));
    }

    [Fact]
    public void Degenerated_edges_are_left_out_and_internal_shapes_count_twice()
    {
        TopoDS_Shape aSphere = new BRepPrimAPI_MakeSphere(2).Shape();
        BOPTools_Set aS = new();
        aS.Add(aSphere, TopAbs_EDGE);
        // the sphere's seam twice (FORWARD and REVERSED in its face), its two degenerated poles left out
        Assert.Equal(2, aS.NbShapes());

        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Edge();
        TopoDS_Compound aC = new();
        BRep_Builder aBB = new();
        aBB.MakeCompound(ref aC);
        aBB.Add(aC, anEdge.Oriented(TopAbs_Orientation.TopAbs_INTERNAL));
        BOPTools_Set aSI = new();
        aSI.Add(aC, TopAbs_EDGE);
        Assert.Equal(2, aSI.NbShapes());

        BOPTools_Set anEmpty = new();
        anEmpty.Add(aC, TopAbs_FACE);
        Assert.Equal(0, anEmpty.NbShapes());
        Assert.Equal(0UL, anEmpty.GetSum());
    }
}
