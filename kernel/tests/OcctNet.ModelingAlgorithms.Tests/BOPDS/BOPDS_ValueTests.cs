// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests.BOPDS;

/// <summary>The value classes of BOPDS (paves, pairs, ranges, points) and BOPDS_Tools.</summary>
public class BOPDS_ValueTests
{
    [Fact]
    public void Pave_DefaultsAndComparisons()
    {
        BOPDS_Pave aDefault = new();
        Assert.Equal(-1, aDefault.Index());
        Assert.Equal(99.0, aDefault.Parameter());

        BOPDS_Pave aP1 = new(3, 0.5), aP2 = new(4, 0.5), aP3 = new(3, 0.5);
        // OCCT's < compares parameters only; == compares index and parameter
        Assert.False(aP1.IsLess(aP2));
        Assert.False(aP2.IsLess(aP1));
        Assert.False(aP1.IsEqual(aP2));
        Assert.True(aP1.IsEqual(aP3));
        Assert.Equal(aP1, aP3);
        Assert.Equal(aP1.GetHashCode(), aP3.GetHashCode());
        Assert.Equal(new BOPDS_Pave(3, 0.0).GetHashCode(), new BOPDS_Pave(3, -0.0).GetHashCode());

        aP1.Contents(out int anIndex, out double aParam);
        Assert.Equal((3, 0.5), (anIndex, aParam));
        Assert.Equal(new BOPDS_Pave(7, 0.5), aP1.WithIndex(7));

        using StringWriter aWriter = new();
        aP1.Dump(aWriter);
        Assert.Equal(" { 3 0.500  }", aWriter.ToString());
    }

    [Fact]
    public void Pave_SortKeepsTiedParametersInOrder()
    {
        // MSVC's std::sort keeps the order of tied elements in ranges of up to 32 (insertion sort)
        BOPDS_Pave[] aPaves = [new(1, 2.0), new(2, 1.0), new(3, 2.0), new(4, 1.0), new(5, 0.0)];
        std_sort.sort(aPaves.AsSpan(), new BOPDS_Pave.LessThan());
        Assert.Equal(new[] { 5, 2, 4, 1, 3 }, aPaves.Select(p => p.Index()));
    }

    [Fact]
    public void Pair_IsUnorderedInMapsAndOrderedForSorts()
    {
        BOPDS_Pair a12 = new(1, 2), a21 = new(2, 1);
        Assert.True(a12.IsEqual(a21));
        Assert.Equal(a12, a21);
        Assert.Equal(a12.GetHashCode(), a21.GetHashCode());
        Assert.True(a12 < a21);
        Assert.False(a21 < a12);

        NCollection_Map<BOPDS_Pair> aMap = new();
        Assert.True(aMap.Add(a12));
        Assert.False(aMap.Add(a21));

        BOPDS_Pair aDefault = new();
        aDefault.Indices(out int i1, out int i2);
        Assert.Equal((-1, -1), (i1, i2));

        BOPDS_Pair[] aPairs = [new(3, 1), new(1, 5), new(1, 2), new(0, 9)];
        std_sort.stable_sort(aPairs.AsSpan(), new BOPDS_Pair.LessThan());
        Assert.Equal(new[] { "BOPDS_Pair(0, 9)", "BOPDS_Pair(1, 2)", "BOPDS_Pair(1, 5)", "BOPDS_Pair(3, 1)" },
                     aPairs.Select(p => p.ToString()));
    }

    [Fact]
    public void IndexRange_Contains()
    {
        BOPDS_IndexRange aRange = new(3, 7);
        Assert.True(aRange.Contains(3));
        Assert.True(aRange.Contains(7));
        Assert.False(aRange.Contains(8));
        Assert.False(aRange.Contains(2));
        Assert.Equal(new BOPDS_IndexRange(), new BOPDS_IndexRange(0, 0));
        using StringWriter aWriter = new();
        aRange.Dump(aWriter);
        Assert.Equal(" range: 3 7", aWriter.ToString());
    }

    [Fact]
    public void Point_Defaults()
    {
        BOPDS_Point aPoint = new();
        Assert.Equal(new gp_Pnt(99, 99, 99), aPoint.Pnt());
        Assert.Equal(new gp_Pnt2d(99, 99), aPoint.Pnt2D1());
        Assert.Equal(new gp_Pnt2d(99, 99), aPoint.Pnt2D2());
        Assert.Equal(-1, aPoint.Index());

        // a vector of points default-constructs as OCCT's
        NCollection_DynamicArray<BOPDS_Point> aPoints = new();
        aPoints.Appended().SetIndex(4);
        Assert.Equal(4, aPoints.Value(0).Index());
        Assert.Equal(new gp_Pnt(99, 99, 99), aPoints.Value(0).Pnt());
    }

    [Fact]
    public void Tools_TypeToInteger()
    {
        Assert.Equal(0, BOPDS_Tools.TypeToInteger(TopAbs_VERTEX, TopAbs_VERTEX));
        Assert.Equal(1, BOPDS_Tools.TypeToInteger(TopAbs_EDGE, TopAbs_VERTEX));
        Assert.Equal(1, BOPDS_Tools.TypeToInteger(TopAbs_VERTEX, TopAbs_EDGE));
        Assert.Equal(2, BOPDS_Tools.TypeToInteger(TopAbs_EDGE, TopAbs_EDGE));
        Assert.Equal(3, BOPDS_Tools.TypeToInteger(TopAbs_FACE, TopAbs_VERTEX));
        Assert.Equal(4, BOPDS_Tools.TypeToInteger(TopAbs_EDGE, TopAbs_FACE));
        Assert.Equal(5, BOPDS_Tools.TypeToInteger(TopAbs_FACE, TopAbs_FACE));
        Assert.Equal(6, BOPDS_Tools.TypeToInteger(TopAbs_SOLID, TopAbs_VERTEX));
        Assert.Equal(7, BOPDS_Tools.TypeToInteger(TopAbs_EDGE, TopAbs_SOLID));
        Assert.Equal(8, BOPDS_Tools.TypeToInteger(TopAbs_SOLID, TopAbs_FACE));
        Assert.Equal(9, BOPDS_Tools.TypeToInteger(TopAbs_SOLID, TopAbs_SOLID));
        Assert.Equal(-1, BOPDS_Tools.TypeToInteger(TopAbs_WIRE, TopAbs_FACE));
        Assert.Equal(-1, BOPDS_Tools.TypeToInteger(TopAbs_COMPOUND, TopAbs_SOLID));
        Assert.Equal(8, BOPDS_Tools.TypeToInteger(TopAbs_SHAPE));

        Assert.True(BOPDS_Tools.HasBRep(TopAbs_FACE));
        Assert.False(BOPDS_Tools.HasBRep(TopAbs_SOLID));
        Assert.True(BOPDS_Tools.IsInterfering(TopAbs_SOLID));
        Assert.False(BOPDS_Tools.IsInterfering(TopAbs_SHELL));
    }

    [Fact]
    public void CoupleOfPaveBlocks_HoldsTheBlocks()
    {
        BOPDS_PaveBlock aPB1 = new(), aPB2 = new();
        BOPDS_CoupleOfPaveBlocks aCouple = new(aPB1, aPB2);
        Assert.Same(aPB1, aCouple.PaveBlock1());
        Assert.Same(aPB2, aCouple.PaveBlock2());
        Assert.Equal(-1, aCouple.Index());
        Assert.Equal(-1, aCouple.IndexInterf());
        Assert.Equal(0.0, aCouple.Tolerance());
        aCouple.SetIndexInterf(3);
        aCouple.SetTolerance(1e-3);
        Assert.Equal(3, aCouple.IndexInterf());
        Assert.Equal(1e-3, aCouple.Tolerance());
    }
}
