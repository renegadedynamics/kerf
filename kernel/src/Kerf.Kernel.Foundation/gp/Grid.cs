// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>
/// A read-only two-dimensional grid of values, indexed from 0: a B-spline or Bézier surface's poles and weights
/// (rows in U, columns in V), or a grid of points to fit (docs/api/README.md, section 6).
/// </summary>
/// <remarks>
/// A value over row-major storage it doesn't copy: whoever builds one hands it storage nobody writes to again. The
/// default value is the empty grid.
/// </remarks>
/// <typeparam name="T">The element type.</typeparam>
public readonly struct Grid<T>
{
    private readonly ReadOnlyMemory<T> _values;

    /// <summary>A grid over <paramref name="values"/>, row by row.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A count is negative, or the values are not
    /// <paramref name="rowCount"/> × <paramref name="columnCount"/>.</exception>
    public Grid(ReadOnlyMemory<T> values, int rowCount, int columnCount)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(rowCount);
        ArgumentOutOfRangeException.ThrowIfNegative(columnCount);
        if ((long)rowCount * columnCount != values.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(values), values.Length,
                                                  $"A {rowCount} x {columnCount} grid needs {(long)rowCount * columnCount} values.");
        }

        _values = values;
        RowCount = rowCount;
        ColumnCount = columnCount;
    }

    /// <summary>The number of rows.</summary>
    public int RowCount { get; }

    /// <summary>The number of columns.</summary>
    public int ColumnCount { get; }

    /// <summary>True if the grid has no values.</summary>
    public bool IsEmpty => _values.IsEmpty;

    /// <summary>The values, row by row.</summary>
    public ReadOnlyMemory<T> Values => _values;

    /// <summary>The value at a row and a column.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The row or the column is outside the grid.</exception>
    public T this[int row, int column]
    {
        get
        {
            if ((uint)row >= (uint)RowCount)
            {
                throw new ArgumentOutOfRangeException(nameof(row), row, "The row is outside the grid.");
            }

            if ((uint)column >= (uint)ColumnCount)
            {
                throw new ArgumentOutOfRangeException(nameof(column), column, "The column is outside the grid.");
            }

            return _values.Span[row * ColumnCount + column];
        }
    }

    /// <summary>The values of one row.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The row is outside the grid.</exception>
    public ReadOnlySpan<T> Row(int row)
    {
        if ((uint)row >= (uint)RowCount)
        {
            throw new ArgumentOutOfRangeException(nameof(row), row, "The row is outside the grid.");
        }

        return _values.Span.Slice(row * ColumnCount, ColumnCount);
    }

    /// <summary>A copy of the values as a rectangular array.</summary>
    public T[,] ToArray()
    {
        T[,] result = new T[RowCount, ColumnCount];
        ReadOnlySpan<T> values = _values.Span;
        for (int row = 0; row < RowCount; row++)
        {
            for (int column = 0; column < ColumnCount; column++)
            {
                result[row, column] = values[row * ColumnCount + column];
            }
        }

        return result;
    }
}
