// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Schematiq.Collections;

namespace Kerf.Kernel.Foundation.Tests.Collections;

public class PackedListTests
{
    [Fact]
    public void Reads_as_a_list_and_a_span()
    {
        var aList = PackedList.Create(new Point3D(1, 2, 3), new Point3D(4, 5, 6), new Point3D(7, 8, 9));

        Assert.Equal(3, aList.Count);
        Assert.Equal(new Point3D(4, 5, 6), aList[1]);
        Assert.Equal(new Point3D(1, 2, 3), aList.PeekFront);
        Assert.Equal(new Point3D(7, 8, 9), aList.Peek);
        Assert.Equal(3, aList.AsSpan().Length);
        Assert.Equal([new Point3D(1, 2, 3), new Point3D(4, 5, 6), new Point3D(7, 8, 9)], aList);
    }

    [Fact]
    public void Copies_its_input()
    {
        double[] aValues = [1, 2, 3];
        var aList = PackedList.Create<double>(aValues);
        aValues[0] = 99;

        Assert.Equal(1, aList[0]);
    }

    [Fact]
    public void Subset_shares_the_array()
    {
        var aList = PackedList.Create<double>(1, 2, 3, 4, 5);
        var aSubset = Assert.IsType<PackedList<double>>(aList.Subset(1, 3));

        Assert.Equal([2, 3, 4], aSubset.AsSpan().ToArray());
        Assert.Same(aList, aList.Subset(0, 5));
        Assert.Equal([1, 2, 3, 4], aList.Pop());
        Assert.Equal([2, 3, 4, 5], aList.PopFront());
    }

    [Fact]
    public void Edits_return_new_lists_and_leave_the_original()
    {
        var aList = PackedList.Create<double>(1, 2, 3);

        Assert.Equal([1, 9, 3], aList.SetItem(1, 9));
        Assert.Equal([1, 2, 3, 4], aList.Push(4));
        Assert.Equal([0, 1, 2, 3], aList.PushFront(0));
        Assert.Equal([1, 5, 2, 3], aList.Insert(1, 5));
        Assert.Equal([1, 3], aList.RemoveAt(1));
        Assert.Equal([3, 2, 1], aList.Reverse());
        Assert.Equal([1, 2, 3], aList);
    }

    [Fact]
    public void Empty_list_behaves_as_Bento_s()
    {
        ISchematiqList<double> anEmpty = PackedList.Create<double>();

        Assert.True(anEmpty.IsEmpty);
        Assert.Equal(0, anEmpty.Depth);
        Assert.Throws<InvalidOperationException>(() => anEmpty.Peek);
        Assert.Throws<ArgumentOutOfRangeException>(() => anEmpty[0]);
    }
}
