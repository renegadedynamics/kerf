// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Kernel.Foundation.Tests;

public class GridTests
{
    [Fact]
    public void IndexesRowByRowFromZero()
    {
        Grid<int> aGrid = new(new[] { 1, 2, 3, 4, 5, 6 }, 2, 3);
        Assert.Equal(2, aGrid.RowCount);
        Assert.Equal(3, aGrid.ColumnCount);
        Assert.Equal(1, aGrid[0, 0]);
        Assert.Equal(3, aGrid[0, 2]);
        Assert.Equal(4, aGrid[1, 0]);
        Assert.Equal(new[] { 4, 5, 6 }, aGrid.Row(1).ToArray());
        Assert.Equal(6, aGrid.ToArray()[1, 2]);
    }

    [Fact]
    public void ChecksItsBounds()
    {
        Grid<int> aGrid = new(new[] { 1, 2, 3, 4, 5, 6 }, 2, 3);
        Assert.Throws<ArgumentOutOfRangeException>(() => aGrid[0, 3]);
        Assert.Throws<ArgumentOutOfRangeException>(() => aGrid[2, 0]);
        Assert.Throws<ArgumentOutOfRangeException>(() => aGrid[-1, 0]);
        Assert.Throws<ArgumentOutOfRangeException>(() => aGrid.Row(2).Length);
        Assert.Throws<ArgumentOutOfRangeException>(() => new Grid<int>(new[] { 1, 2, 3 }, 2, 2));
    }

    [Fact]
    public void DefaultIsEmpty()
    {
        Grid<double> aGrid = default;
        Assert.True(aGrid.IsEmpty);
        Assert.Equal(0, aGrid.RowCount);
        Assert.Empty(aGrid.ToArray());
    }

    [Fact]
    public void CopiesAnArray2WhateverItsBounds()
    {
        NCollection_Array2<double> anArray = new(3, 4, -1, 1);
        anArray[3, -1] = 1.0;
        anArray[4, 1] = 6.0;
        Grid<double> aGrid = anArray.ToGrid();
        Assert.Equal(2, aGrid.RowCount);
        Assert.Equal(3, aGrid.ColumnCount);
        Assert.Equal(1.0, aGrid[0, 0]);
        Assert.Equal(6.0, aGrid[1, 2]);
        anArray[3, -1] = 9.0;
        Assert.Equal(1.0, aGrid[0, 0]);
    }
}
