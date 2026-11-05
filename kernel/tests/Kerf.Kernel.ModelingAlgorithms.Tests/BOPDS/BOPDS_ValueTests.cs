// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation.Std;
using Kerf.Geometry;
using System.Runtime.InteropServices;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BOPDS;

/// <summary>The value classes of BOPDS (paves, pairs, ranges, points) and BOPDS_Tools.</summary>
public class BOPDS_ValueTests
{
    [Fact]
    public void Pave_DefaultsAndComparisons()
    {
        Pave aDefault = new();
        Assert.Equal(-1, aDefault.Index());
        Assert.Equal(99.0, aDefault.Parameter());

        Pave aP1 = new(3, 0.5), aP2 = new(4, 0.5), aP3 = new(3, 0.5);
        // OCCT's < compares parameters only; == compares index and parameter
        Assert.False(aP1.IsLess(aP2));
        Assert.False(aP2.IsLess(aP1));
        Assert.False(aP1.IsEqual(aP2));
        Assert.True(aP1.IsEqual(aP3));
        Assert.Equal(aP1, aP3);
        Assert.Equal(aP1.GetHashCode(), aP3.GetHashCode());
        Assert.Equal(new Pave(3, 0.0).GetHashCode(), new Pave(3, -0.0).GetHashCode());

        aP1.Contents(out int anIndex, out double aParam);
        Assert.Equal((3, 0.5), (anIndex, aParam));
        Assert.Equal(new Pave(7, 0.5), aP1.WithIndex(7));

        using StringWriter aWriter = new();
        aP1.Dump(aWriter);
        Assert.Equal(" { 3 0.500  }", aWriter.ToString());
    }

    [Fact]
    public void Pave_SortKeepsTiedParametersInOrder()
    {
        // MSVC's std::sort keeps the order of tied elements in ranges of up to 32 (insertion sort)
        Pave[] aPaves = [new(1, 2.0), new(2, 1.0), new(3, 2.0), new(4, 1.0), new(5, 0.0)];
        Sorting.Sort(aPaves.AsSpan(), new Pave.LessThan());
        Assert.Equal(new[] { 5, 2, 4, 1, 3 }, aPaves.Select(p => p.Index()));
    }

    [Fact]
    public void Pair_IsUnorderedInMapsAndOrderedForSorts()
    {
        Pair a12 = new(1, 2), a21 = new(2, 1);
        Assert.True(a12.IsEqual(a21));
        Assert.Equal(a12, a21);
        Assert.Equal(a12.GetHashCode(), a21.GetHashCode());
        Assert.True(a12 < a21);
        Assert.False(a21 < a12);

        HashSet<Pair> aMap = new();
        Assert.True(aMap.Add(a12));
        Assert.False(aMap.Add(a21));

        Pair aDefault = new();
        aDefault.Indices(out int i1, out int i2);
        Assert.Equal((-1, -1), (i1, i2));

        Pair[] aPairs = [new(3, 1), new(1, 5), new(1, 2), new(0, 9)];
        Sorting.stable_sort(aPairs.AsSpan(), new Pair.LessThan());
        Assert.Equal(new[] { "Pair(0, 9)", "Pair(1, 2)", "Pair(1, 5)", "Pair(3, 1)" },
                     aPairs.Select(p => p.ToString()));
    }

    [Fact]
    public void IndexRange_Contains()
    {
        IndexRange aRange = new(3, 7);
        Assert.True(aRange.Contains(3));
        Assert.True(aRange.Contains(7));
        Assert.False(aRange.Contains(8));
        Assert.False(aRange.Contains(2));
        Assert.Equal(new IndexRange(), new IndexRange(0, 0));
        using StringWriter aWriter = new();
        aRange.Dump(aWriter);
        Assert.Equal(" range: 3 7", aWriter.ToString());
    }

    [Fact]
    public void Point_Defaults()
    {
        DataPoint aPoint = new();
        Assert.Equal(new Point3D(99, 99, 99), aPoint.Pnt());
        Assert.Equal(new Point2D(99, 99), aPoint.Pnt2D1());
        Assert.Equal(new Point2D(99, 99), aPoint.Pnt2D2());
        Assert.Equal(-1, aPoint.Index());

        // a vector of points default-constructs as OCCT's
        List<DataPoint> aPoints = new();
        aPoints.SetAt(1, new DataPoint()); // grows with a default-constructed element at 0
        ref DataPoint aFirst = ref CollectionsMarshal.AsSpan(aPoints)[0];
        aFirst.SetIndex(4);
        Assert.Equal(4, aPoints[0].Index());
        Assert.Equal(new Point3D(99, 99, 99), aPoints[0].Pnt());
    }

    [Fact]
    public void Tools_TypeToInteger()
    {
        Assert.Equal(0, DataTools.TypeToInteger(Vertex, Vertex));
        Assert.Equal(1, DataTools.TypeToInteger(Edge, Vertex));
        Assert.Equal(1, DataTools.TypeToInteger(Vertex, Edge));
        Assert.Equal(2, DataTools.TypeToInteger(Edge, Edge));
        Assert.Equal(3, DataTools.TypeToInteger(Face, Vertex));
        Assert.Equal(4, DataTools.TypeToInteger(Edge, Face));
        Assert.Equal(5, DataTools.TypeToInteger(Face, Face));
        Assert.Equal(6, DataTools.TypeToInteger(Solid, Vertex));
        Assert.Equal(7, DataTools.TypeToInteger(Edge, Solid));
        Assert.Equal(8, DataTools.TypeToInteger(Solid, Face));
        Assert.Equal(9, DataTools.TypeToInteger(Solid, Solid));
        Assert.Equal(-1, DataTools.TypeToInteger(Wire, Face));
        Assert.Equal(-1, DataTools.TypeToInteger(Compound, Solid));
        Assert.Equal(8, DataTools.TypeToInteger(Shape));

        Assert.True(DataTools.HasBRep(Face));
        Assert.False(DataTools.HasBRep(Solid));
        Assert.True(DataTools.IsInterfering(Solid));
        Assert.False(DataTools.IsInterfering(Shell));
    }

    [Fact]
    public void CoupleOfPaveBlocks_HoldsTheBlocks()
    {
        PaveBlock aPB1 = new(), aPB2 = new();
        CoupleOfPaveBlocks aCouple = new(aPB1, aPB2);
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
