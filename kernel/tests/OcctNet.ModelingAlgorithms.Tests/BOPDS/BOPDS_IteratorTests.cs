// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests.BOPDS;

/// <summary>
/// The iterators of interfering pairs: sorted pairs of shapes of different arguments, by type; self-interference
/// pairs; sub-set pairs; the extra intersection in parallel giving the serial result.
/// </summary>
public class BOPDS_IteratorTests
{
    private static readonly (TopAbs_ShapeEnum, TopAbs_ShapeEnum)[] theTypes =
    [
        (TopAbs_VERTEX, TopAbs_VERTEX), (TopAbs_VERTEX, TopAbs_EDGE), (TopAbs_EDGE, TopAbs_EDGE),
        (TopAbs_VERTEX, TopAbs_FACE), (TopAbs_EDGE, TopAbs_FACE), (TopAbs_FACE, TopAbs_FACE),
    ];

    private static List<(int, int)> Pairs(BOPDS_Iterator theIt, TopAbs_ShapeEnum theT1, TopAbs_ShapeEnum theT2)
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

    private static Dictionary<(TopAbs_ShapeEnum, TopAbs_ShapeEnum), List<(int, int)>> AllPairs(BOPDS_Iterator theIt) =>
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
        foreach (((TopAbs_ShapeEnum aT1, TopAbs_ShapeEnum aT2), List<(int, int)> aPairs) in anAll)
        {
            foreach ((int n1, int n2) in aPairs)
            {
                // Value gives the shape of the higher type index (vertex 7 > edge 6 > face 4) first
                TopAbs_ShapeEnum aType1 = aDS.ShapeInfo(n1).ShapeType(), aType2 = aDS.ShapeInfo(n2).ShapeType();
                Assert.True((int)aType1 >= (int)aType2);
                Assert.Equal(BOPDS_Tools.TypeToInteger(aT1, aT2), BOPDS_Tools.TypeToInteger(aType1, aType2));
                Assert.NotEqual(aDS.Rank(n1), aDS.Rank(n2));
                Assert.False(aDS.ShapeInfo(n1).Box().IsOut(aDS.ShapeInfo(n2).Box()));
            }
        }
        // the face/face pairs: each face of one box against the faces of the other it reaches
        List<(int, int)> aFF = anAll[(TopAbs_FACE, TopAbs_FACE)];
        Assert.NotEmpty(aFF);
        // sorted: by the smaller index, then the larger
        List<(int, int)> anOrdered = [.. aFF.Select(p => (Math.Min(p.Item1, p.Item2), Math.Max(p.Item1, p.Item2)))];
        Assert.Equal(anOrdered.OrderBy(p => p.Item1).ThenBy(p => p.Item2), anOrdered);
        Assert.Equal(Math.Max(1, aFF.Count / 2), Math.Max(1, anIt.BlockLength()));

        // Initializing a type with no index (wire/face) gives nothing
        anIt.Initialize(TopAbs_WIRE, TopAbs_FACE);
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
        anEmpty.Initialize(TopAbs_FACE, TopAbs_FACE);
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
        Assert.NotEmpty(aWithout[(TopAbs_FACE, TopAbs_FACE)]);

        // the boxes are axis-aligned: the oriented boxes filter nothing more out
        IntTools_Context aContext = new();
        anIt.Prepare(aContext, true, Precision.Confusion());
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
        NCollection_Map<int> anIndices = new();
        for (int i = 0; i < aDS.NbSourceShapes(); ++i)
        {
            if (aDS.ShapeInfo(i).ShapeType() == TopAbs_VERTEX)
            {
                anIndices.Add(i);
            }
        }

        Dictionary<(TopAbs_ShapeEnum, TopAbs_ShapeEnum), List<(int, int)>> Run(bool theParallel)
        {
            BOPDS_Iterator anIt = new();
            anIt.SetDS(aDS);
            anIt.SetRunParallel(theParallel);
            Assert.Equal(theParallel, anIt.RunParallel());
            anIt.Prepare();
            anIt.IntersectExt(anIndices);
            return AllPairs(anIt);
        }

        Dictionary<(TopAbs_ShapeEnum, TopAbs_ShapeEnum), List<(int, int)>> aSerial;
        using (OSD_Parallel.ForceSerialScope())
        {
            aSerial = Run(false);
        }
        Assert.NotEmpty(aSerial[(TopAbs_VERTEX, TopAbs_FACE)]);
        // the extra lists hold V/V, V/E, V/F only; E/E is kept but never filled
        Assert.Empty(aSerial[(TopAbs_EDGE, TopAbs_EDGE)]);

        foreach (int aDegree in new[] { 1, 2, 4, 8 })
        {
            for (int aRepeat = 0; aRepeat < 3; ++aRepeat)
            {
                using (OSD_Parallel.MaxDegreeOfParallelismScope(aDegree))
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
        Assert.Equal(12, anAll[(TopAbs_FACE, TopAbs_FACE)].Count);
        foreach ((int n1, int n2) in anAll[(TopAbs_EDGE, TopAbs_FACE)])
        {
            Assert.False(aDS.ShapeInfo(n2).HasSubShape(n1));
        }
        // adjacent edges touch at their common vertex; the vertices are apart
        Assert.NotEmpty(anAll[(TopAbs_EDGE, TopAbs_EDGE)]);
        Assert.Empty(anAll[(TopAbs_VERTEX, TopAbs_VERTEX)]);

        anIt.UpdateByLevelOfCheck(2);
        var aLevel2 = AllPairs(anIt);
        Assert.Equal(anAll[(TopAbs_EDGE, TopAbs_EDGE)], aLevel2[(TopAbs_EDGE, TopAbs_EDGE)]);
        Assert.Empty(aLevel2[(TopAbs_FACE, TopAbs_FACE)]);
        anIt.UpdateByLevelOfCheck(1);
        Assert.Empty(Pairs(anIt, TopAbs_EDGE, TopAbs_EDGE));
    }

    [Fact]
    public void SubIterator_PairsBetweenTwoSubSets()
    {
        BOPDS_DS aDS = BOPDS_DSTests.MakeDS(BOPDS_DSTests.Box(0, 0, 0), BOPDS_DSTests.Box(5, 5, 5));
        NCollection_List<int> aFaces1 = new(), aFaces2 = new();
        for (int i = 0; i < aDS.NbSourceShapes(); ++i)
        {
            if (aDS.ShapeInfo(i).ShapeType() == TopAbs_FACE)
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
        List<(int, int)> aFF = Pairs(anIt, TopAbs_FACE, TopAbs_FACE);
        Assert.Equal(aFF.Select(p => (Math.Min(p.Item1, p.Item2), Math.Max(p.Item1, p.Item2))),
                     aPairs.Select(p => (Math.Min(p.Item1, p.Item2), Math.Max(p.Item1, p.Item2))));

        // an empty sub-set gives nothing
        aSubIt.SetSubSet2(new NCollection_List<int>());
        aSubIt.Prepare();
        aSubIt.Initialize();
        Assert.False(aSubIt.More());
    }
}
