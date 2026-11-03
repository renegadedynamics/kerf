// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests.BOPDS;

/// <summary>
/// The iterators of interfering pairs: sorted pairs of shapes of different arguments, by type; self-interference
/// pairs; sub-set pairs; the extra intersection in parallel giving the serial result.
/// </summary>
public class BOPDS_IteratorTests
{
    private static readonly (ShapeType, ShapeType)[] theTypes =
    [
        (ShapeType.Vertex, ShapeType.Vertex), (ShapeType.Vertex, ShapeType.Edge), (ShapeType.Edge, ShapeType.Edge),
        (ShapeType.Vertex, ShapeType.Face), (ShapeType.Edge, ShapeType.Face), (ShapeType.Face, ShapeType.Face),
    ];

    private static List<(int, int)> Pairs(BOPDS_Iterator theIt, ShapeType theT1, ShapeType theT2)
    {
        List<(int, int)> aPairs = [];
        for (theIt.Initialize(theT1, theT2); theIt.More(); theIt.Next())
        {
            theIt.Value(out int n1, out int n2);
            aPairs.Add((n1, n2));
        }
        Assert.Equal(aPairs.Count, theIt.ExpectedLength());
        return aPairs;
    }

    private static Dictionary<(ShapeType, ShapeType), List<(int, int)>> AllPairs(BOPDS_Iterator theIt) =>
        theTypes.ToDictionary(t => t, t => Pairs(theIt, t.Item1, t.Item2));

    [Fact]
    public void OverlappingBoxes_PairsOfDifferentArgumentsSortedByType()
    {
        BOPDS_DS aDS = BOPDS_DSTests.MakeDS(BOPDS_DSTests.Box(0, 0, 0), BOPDS_DSTests.Box(5, 5, 5));
        BOPDS_Iterator anIt = new();
        anIt.SetDS(aDS);
        Assert.Same(aDS, anIt.DS());
        anIt.Prepare();

        var anAll = AllPairs(anIt);
        foreach (((ShapeType aT1, ShapeType aT2), List<(int, int)> aPairs) in anAll)
        {
            foreach ((int n1, int n2) in aPairs)
            {
                // Value gives the shape of the higher type index (vertex 7 > edge 6 > face 4) first
                ShapeType aType1 = aDS.ShapeInfo(n1).ShapeType(), aType2 = aDS.ShapeInfo(n2).ShapeType();
                Assert.True((int)aType1 >= (int)aType2);
                Assert.Equal(BOPDS_Tools.TypeToInteger(aT1, aT2), BOPDS_Tools.TypeToInteger(aType1, aType2));
                Assert.NotEqual(aDS.Rank(n1), aDS.Rank(n2));
                Assert.False(aDS.ShapeInfo(n1).Box().IsOut(aDS.ShapeInfo(n2).Box()));
            }
        }
        // the face/face pairs: each face of one box against the faces of the other it reaches
        List<(int, int)> aFF = anAll[(ShapeType.Face, ShapeType.Face)];
        Assert.NotEmpty(aFF);
        // sorted: by the smaller index, then the larger
        List<(int, int)> anOrdered = [.. aFF.Select(p => (Math.Min(p.Item1, p.Item2), Math.Max(p.Item1, p.Item2)))];
        Assert.Equal(anOrdered.OrderBy(p => p.Item1).ThenBy(p => p.Item2), anOrdered);
        Assert.Equal(Math.Max(1, aFF.Count / 2), Math.Max(1, anIt.BlockLength()));

        // Initializing a type with no index (wire/face) gives nothing
        anIt.Initialize(ShapeType.Wire, ShapeType.Face);
        Assert.False(anIt.More());
        Assert.Equal(0, anIt.ExpectedLength());
    }

    [Fact]
    public void DisjointBoxes_NoPairs()
    {
        BOPDS_DS aDS = BOPDS_DSTests.MakeDS(BOPDS_DSTests.Box(0, 0, 0), BOPDS_DSTests.Box(20, 0, 0));
        BOPDS_Iterator anIt = new();
        anIt.SetDS(aDS);
        anIt.Prepare();
        Assert.All(AllPairs(anIt).Values, Assert.Empty);

        // without a data structure Prepare does nothing
        BOPDS_Iterator anEmpty = new();
        anEmpty.Prepare();
        anEmpty.Initialize(ShapeType.Face, ShapeType.Face);
        Assert.False(anEmpty.More());
    }

    [Fact]
    public void TouchingBoxes_FaceFacePairsAndOBBFilter()
    {
        // Boxes sharing the face x = 10: the shared faces' boxes interfere
        BOPDS_DS aDS = BOPDS_DSTests.MakeDS(BOPDS_DSTests.Box(0, 0, 0), BOPDS_DSTests.Box(10, 0, 0));
        BOPDS_Iterator anIt = new();
        anIt.SetDS(aDS);
        anIt.Prepare();
        var aWithout = AllPairs(anIt);
        Assert.NotEmpty(aWithout[(ShapeType.Face, ShapeType.Face)]);

        // the boxes are axis-aligned: the oriented boxes filter nothing more out
        IntTools_Context aContext = new();
        anIt.Prepare(aContext, true, global::Kerf.Engine.Foundation.Precision.Confusion());
        var aWith = AllPairs(anIt);
        foreach (var aKey in theTypes)
        {
            Assert.Equal(aWithout[aKey], aWith[aKey]);
        }
    }

    [Fact]
    public void IntersectExt_ParallelGivesTheSerialPairs()
    {
        // the third box's vertices at x = 10 lie on the first box's face
        BOPDS_DS aDS = BOPDS_DSTests.MakeDS(BOPDS_DSTests.Box(0, 0, 0), BOPDS_DSTests.Box(5, 5, 5),
                                           BOPDS_DSTests.Box(10, 2, 2, 5));
        NCollection_IntMap anIndices = new();
        for (int i = 0; i < aDS.NbSourceShapes(); ++i)
        {
            if (aDS.ShapeInfo(i).ShapeType() == ShapeType.Vertex)
            {
                anIndices.Add(i);
            }
        }

        Dictionary<(ShapeType, ShapeType), List<(int, int)>> Run(bool theParallel)
        {
            BOPDS_Iterator anIt = new();
            anIt.SetDS(aDS);
            anIt.SetRunParallel(theParallel);
            Assert.Equal(theParallel, anIt.RunParallel());
            anIt.Prepare();
            anIt.IntersectExt(anIndices);
            return AllPairs(anIt);
        }

        Dictionary<(ShapeType, ShapeType), List<(int, int)>> aSerial;
        using (global::Kerf.Engine.Foundation.OsdParallel.ForceSerialScope())
        {
            aSerial = Run(false);
        }
        Assert.NotEmpty(aSerial[(ShapeType.Vertex, ShapeType.Face)]);
        // the extra lists hold V/V, V/E, V/F only; E/E is kept but never filled
        Assert.Empty(aSerial[(ShapeType.Edge, ShapeType.Edge)]);

        foreach (int aDegree in new[] { 1, 2, 4, 8 })
        {
            for (int aRepeat = 0; aRepeat < 3; ++aRepeat)
            {
                using (global::Kerf.Engine.Foundation.OsdParallel.MaxDegreeOfParallelismScope(aDegree))
                {
                    var aParallel = Run(true);
                    foreach (var aKey in theTypes)
                    {
                        Assert.Equal(aSerial[aKey], aParallel[aKey]);
                    }
                }
            }
        }
    }

    [Fact]
    public void IteratorSI_SelfInterferencesAndLevels()
    {
        // One box: within the argument, only shapes that aren't sub-shapes of each other
        BOPDS_DS aDS = BOPDS_DSTests.MakeDS(BOPDS_DSTests.Box(0, 0, 0));
        BOPDS_IteratorSI anIt = new();
        anIt.SetDS(aDS);
        anIt.Prepare();
        var anAll = AllPairs(anIt);
        // adjacent faces touch along their common edge
        Assert.Equal(12, anAll[(ShapeType.Face, ShapeType.Face)].Count);
        foreach ((int n1, int n2) in anAll[(ShapeType.Edge, ShapeType.Face)])
        {
            Assert.False(aDS.ShapeInfo(n2).HasSubShape(n1));
        }
        // adjacent edges touch at their common vertex; the vertices are apart
        Assert.NotEmpty(anAll[(ShapeType.Edge, ShapeType.Edge)]);
        Assert.Empty(anAll[(ShapeType.Vertex, ShapeType.Vertex)]);

        anIt.UpdateByLevelOfCheck(2);
        var aLevel2 = AllPairs(anIt);
        Assert.Equal(anAll[(ShapeType.Edge, ShapeType.Edge)], aLevel2[(ShapeType.Edge, ShapeType.Edge)]);
        Assert.Empty(aLevel2[(ShapeType.Face, ShapeType.Face)]);
        anIt.UpdateByLevelOfCheck(1);
        Assert.Empty(Pairs(anIt, ShapeType.Edge, ShapeType.Edge));
    }

    [Fact]
    public void SubIterator_PairsBetweenTwoSubSets()
    {
        BOPDS_DS aDS = BOPDS_DSTests.MakeDS(BOPDS_DSTests.Box(0, 0, 0), BOPDS_DSTests.Box(5, 5, 5));
        NCollection_List<int> aFaces1 = new(), aFaces2 = new();
        for (int i = 0; i < aDS.NbSourceShapes(); ++i)
        {
            if (aDS.ShapeInfo(i).ShapeType() == ShapeType.Face)
            {
                (aDS.Rank(i) == 0 ? aFaces1 : aFaces2).Append(i);
            }
        }

        BOPDS_SubIterator aSubIt = new();
        aSubIt.SetDS(aDS);
        aSubIt.SetSubSet1(aFaces1);
        aSubIt.SetSubSet2(aFaces2);
        Assert.Same(aFaces1, aSubIt.SubSet1());
        aSubIt.Prepare();
        aSubIt.Initialize();
        List<(int, int)> aPairs = [];
        for (; aSubIt.More(); aSubIt.Next())
        {
            aSubIt.Value(out int n1, out int n2);
            aPairs.Add((n1, n2));
        }
        Assert.Equal(aPairs.Count, aSubIt.ExpectedLength());

        // the same pairs as the main iterator's face/face pairs
        BOPDS_Iterator anIt = new();
        anIt.SetDS(aDS);
        anIt.Prepare();
        List<(int, int)> aFF = Pairs(anIt, ShapeType.Face, ShapeType.Face);
        Assert.Equal(aFF.Select(p => (Math.Min(p.Item1, p.Item2), Math.Max(p.Item1, p.Item2))),
                     aPairs.Select(p => (Math.Min(p.Item1, p.Item2), Math.Max(p.Item1, p.Item2))));

        // an empty sub-set gives nothing
        aSubIt.SetSubSet2(new NCollection_List<int>());
        aSubIt.Prepare();
        aSubIt.Initialize();
        Assert.False(aSubIt.More());
    }
}
