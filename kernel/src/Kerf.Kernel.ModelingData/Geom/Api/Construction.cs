// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Unit 10L1: the B-splines' and Béziers' public constructors (docs/api/geometry.md), from spans and grids indexed from 0,
// in place of OCCT's 1-based arrays. Each builds the curve with OCCT's constructor (internal) and keeps a copy of it, so
// that OCCT's construction errors become ArgumentExceptions.

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Geometry;

public sealed partial class BSplineCurve3D
{
    /// <summary>A B-spline curve from its poles, knots and multiplicities (all indexed from 0), its degree, whether it is
    /// periodic, and its weights (empty: not rational).</summary>
    /// <exception cref="ArgumentException">The data don't make a B-spline (OCCT's conditions: the multiplicities' sum, the
    /// knots increasing, the weights positive, the degree from 1 to <see cref="MaxDegree"/>).</exception>
    public BSplineCurve3D(ReadOnlySpan<Point3D> poles, ReadOnlySpan<double> knots, ReadOnlySpan<int> multiplicities, int degree,
                          bool periodic = false, ReadOnlySpan<double> weights = default)
        : this(GeometryArrays.Checked(GeometryArrays.Of(poles), GeometryArrays.Of(knots), GeometryArrays.Of(multiplicities), degree, periodic,
                                      weights.IsEmpty ? null : GeometryArrays.Of(weights),
                                      static (p, k, m, d, per, w) => w is null ? new BSplineCurve3D(p, k, m, d, per) : new BSplineCurve3D(p, w, k, m, d, per)))
    {
    }
}

public sealed partial class BSplineCurve2D
{
    /// <summary>A 2D B-spline curve from its poles, knots and multiplicities (all indexed from 0), its degree, whether it is
    /// periodic, and its weights (empty: not rational).</summary>
    /// <exception cref="ArgumentException">The data don't make a B-spline.</exception>
    public BSplineCurve2D(ReadOnlySpan<Point2D> poles, ReadOnlySpan<double> knots, ReadOnlySpan<int> multiplicities, int degree,
                          bool periodic = false, ReadOnlySpan<double> weights = default)
        : this(GeometryArrays.Checked(GeometryArrays.Of(poles), GeometryArrays.Of(knots), GeometryArrays.Of(multiplicities), degree, periodic,
                                      weights.IsEmpty ? null : GeometryArrays.Of(weights),
                                      static (p, k, m, d, per, w) => w is null ? new BSplineCurve2D(p, k, m, d, per) : new BSplineCurve2D(p, w, k, m, d, per)))
    {
    }
}

public sealed partial class BezierCurve3D
{
    /// <summary>A Bézier curve from its poles (indexed from 0) and its weights (empty: not rational).</summary>
    /// <exception cref="ArgumentException">Fewer than two poles, more than <see cref="MaxDegree"/> + 1, or a weight not
    /// positive.</exception>
    public BezierCurve3D(ReadOnlySpan<Point3D> poles, ReadOnlySpan<double> weights = default)
        : this(GeometryArrays.Checked(GeometryArrays.Of(poles), weights.IsEmpty ? null : GeometryArrays.Of(weights),
                                      static (p, w) => w is null ? new BezierCurve3D(p) : new BezierCurve3D(p, w)))
    {
    }
}

public sealed partial class BezierCurve2D
{
    /// <summary>A 2D Bézier curve from its poles (indexed from 0) and its weights (empty: not rational).</summary>
    /// <exception cref="ArgumentException">Fewer than two poles, too many, or a weight not positive.</exception>
    public BezierCurve2D(ReadOnlySpan<Point2D> poles, ReadOnlySpan<double> weights = default)
        : this(GeometryArrays.Checked(GeometryArrays.Of(poles), weights.IsEmpty ? null : GeometryArrays.Of(weights),
                                      static (p, w) => w is null ? new BezierCurve2D(p) : new BezierCurve2D(p, w)))
    {
    }
}

public sealed partial class BSplineSurface
{
    /// <summary>A B-spline surface from its poles (rows along U, columns along V), knots and multiplicities in U and V,
    /// degrees, periodicity, and weights (an empty grid: not rational).</summary>
    /// <exception cref="ArgumentException">The data don't make a B-spline surface.</exception>
    public BSplineSurface(Grid<Point3D> poles, ReadOnlySpan<double> uKnots, ReadOnlySpan<double> vKnots,
                          ReadOnlySpan<int> uMultiplicities, ReadOnlySpan<int> vMultiplicities, int uDegree, int vDegree,
                          bool uPeriodic = false, bool vPeriodic = false, Grid<double> weights = default)
        : this(Make(GeometryArrays.Of(poles), weights.IsEmpty ? null : GeometryArrays.Of(weights), GeometryArrays.Of(uKnots),
                    GeometryArrays.Of(vKnots), GeometryArrays.Of(uMultiplicities), GeometryArrays.Of(vMultiplicities), uDegree, vDegree,
                    uPeriodic, vPeriodic))
    {
    }

    private static BSplineSurface Make(NCollection_Array2<Point3D> poles, NCollection_Array2<double>? weights,
                                       NCollection_Array1<double> uKnots, NCollection_Array1<double> vKnots,
                                       NCollection_Array1<int> uMultiplicities, NCollection_Array1<int> vMultiplicities,
                                       int uDegree, int vDegree, bool uPeriodic, bool vPeriodic)
        => GeometryArrays.Checked(() => weights is null
            ? new BSplineSurface(poles, uKnots, vKnots, uMultiplicities, vMultiplicities, uDegree, vDegree, uPeriodic, vPeriodic)
            : new BSplineSurface(poles, weights, uKnots, vKnots, uMultiplicities, vMultiplicities, uDegree, vDegree, uPeriodic, vPeriodic));
}

public sealed partial class BezierSurface
{
    /// <summary>A Bézier surface from its poles (rows along U, columns along V) and weights (an empty grid: not
    /// rational).</summary>
    /// <exception cref="ArgumentException">Fewer than two poles in a direction, too many, or a weight not positive.</exception>
    public BezierSurface(Grid<Point3D> poles, Grid<double> weights = default)
        : this(Make(GeometryArrays.Of(poles), weights.IsEmpty ? null : GeometryArrays.Of(weights)))
    {
    }

    private static BezierSurface Make(NCollection_Array2<Point3D> poles, NCollection_Array2<double>? weights)
        => GeometryArrays.Checked(() => weights is null ? new BezierSurface(poles) : new BezierSurface(poles, weights));
}

/// <summary>Spans and grids to OCCT's 1-based arrays, and OCCT's construction errors to argument exceptions.</summary>
internal static class GeometryArrays
{
    public static NCollection_Array1<T> Of<T>(ReadOnlySpan<T> values)
    {
        if (values.IsEmpty)
        {
            // OCCT has no empty array: an unallocated one is rejected the same way
            return new NCollection_Array1<T>();
        }

        var array = new NCollection_Array1<T>(1, values.Length);

        for (int i = 0; i < values.Length; i++)
        {
            array.SetValue(i + 1, values[i]);
        }

        return array;
    }

    public static NCollection_Array2<T> Of<T>(Grid<T> grid)
    {
        if (grid.IsEmpty)
        {
            return new NCollection_Array2<T>();
        }

        var array = new NCollection_Array2<T>(1, grid.RowCount, 1, grid.ColumnCount);
        for (int row = 0; row < grid.RowCount; row++)
        {
            for (int column = 0; column < grid.ColumnCount; column++)
            {
                array.SetValue(row + 1, column + 1, grid[row, column]);
            }
        }

        return array;
    }

    public static TCurve Checked<TPole, TCurve>(NCollection_Array1<TPole> poles, NCollection_Array1<double> knots, NCollection_Array1<int> multiplicities,
                                                int degree, bool periodic, NCollection_Array1<double>? weights,
                                                Func<NCollection_Array1<TPole>, NCollection_Array1<double>, NCollection_Array1<int>, int, bool, NCollection_Array1<double>?, TCurve> make)
        => Checked(() => make(poles, knots, multiplicities, degree, periodic, weights));

    public static TCurve Checked<TPole, TCurve>(NCollection_Array1<TPole> poles, NCollection_Array1<double>? weights,
                                                Func<NCollection_Array1<TPole>, NCollection_Array1<double>?, TCurve> make)
        => Checked(() => make(poles, weights));

    public static T Checked<T>(Func<T> make)
    {
        try
        {
            return make();
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            throw new ArgumentException(Refusal.Unwrapped(e).Message is { Length: > 0 } message ? message : "The data don't make this geometry.", e);
        }
    }
}
