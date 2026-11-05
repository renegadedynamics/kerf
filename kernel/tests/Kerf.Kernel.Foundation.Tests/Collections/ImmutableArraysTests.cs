// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests.Collections;

/// <summary>The arrays the kernel returns as read-only lists (question 50).</summary>
public class ImmutableArraysTests
{
    [Fact]
    public void Reads_as_a_list_and_a_span()
    {
        var anArray = ImmutableArrays.Copy(new Point3D(1, 2, 3), new Point3D(4, 5, 6), new Point3D(7, 8, 9));
        IReadOnlyList<Point3D> aList = anArray;

        Assert.Equal(3, aList.Count);
        Assert.Equal(new Point3D(4, 5, 6), aList[1]);
        Assert.Equal(3, anArray.AsSpan().Length);
        Assert.Equal([new Point3D(1, 2, 3), new Point3D(4, 5, 6), new Point3D(7, 8, 9)], aList);
    }

    [Fact]
    public void Copies_its_input()
    {
        double[] aValues = [1, 2, 3];
        var aList = ImmutableArrays.Copy<double>(aValues);
        aValues[0] = 99;

        Assert.Equal(1, aList[0]);
    }

    [Fact]
    public void Own_takes_the_array_without_copying()
    {
        double[] aValues = [1, 2, 3];
        var aList = ImmutableArrays.Own(aValues);
        aValues[0] = 99;   // what its callers must not do

        Assert.Equal(99, aList[0]);
    }

    [Fact]
    public void An_arrays_immutable_copy_leaves_later_changes_out()
    {
        var anArray = new NCollection_Array1<double>(1, 3);
        anArray.SetValue(1, 1.0);
        var aCopy = anArray.ToImmutableArray();
        anArray.SetValue(1, 2.0);

        Assert.Equal(3, aCopy.Length);
        Assert.Equal(1.0, aCopy[0]);
    }

    [Fact]
    public void Empty_list_reads_as_empty()
    {
        IReadOnlyList<double> anEmpty = ImmutableArrays.Copy<double>();

        Assert.Empty(anEmpty);
        Assert.Throws<IndexOutOfRangeException>(() => anEmpty[0]);
    }
}
