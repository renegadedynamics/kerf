// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// Frozen splines share equal knots and multiplicities (unit 11Q, sixth slice)
public class SharedKnotsTests
{
    private static BSplineCurve3D Spline(double z, double last = 2.0)
    {
        NCollection_Array1<Point3D> poles = new(1, 5);
        for (int i = 1; i <= 5; i++)
            poles[i] = new Point3D(i, z, i * i);
        return new BSplineCurve3D(poles, NCollection_Array1<double>.From([0.0, 1.0, last]), NCollection_Array1<int>.From([4, 1, 4]), 3);
    }

    [Fact]
    public void Frozen_splines_with_the_same_knots_share_them_and_read_the_same()
    {
        BSplineCurve3D first = Spline(1), second = Spline(2), third = Spline(3, 2.5);
        Point3D before = second.Value(0.7);
        Assert.NotSame(first.InternalKnots(), second.InternalKnots());

        first.Freeze();
        second.Freeze();
        third.Freeze();

        Assert.Same(first.InternalKnots(), second.InternalKnots());
        Assert.Same(first.InternalMultiplicities(), second.InternalMultiplicities());
        Assert.NotSame(first.InternalKnots(), third.InternalKnots());
        Assert.Equal(before, second.Value(0.7));
        Assert.Equal(new[] { 0.0, 1.0, 2.0 }, second.InternalKnots().AsSpan().ToArray());
        Assert.Equal(new[] { 0.0, 1.0, 2.5 }, third.InternalKnots().AsSpan().ToArray());
    }

    [Fact]
    public void A_copy_of_a_frozen_spline_has_its_own_knots_to_change()
    {
        BSplineCurve3D first = Spline(4), second = Spline(5);
        first.Freeze();
        second.Freeze();

        var copy = (BSplineCurve3D)first.Copy();
        Assert.False(copy.IsFrozen);
        copy.SetKnot(2, 1.5);

        Assert.Equal(1.5, copy.InternalKnots()[2]);
        Assert.Equal(1.0, first.InternalKnots()[2]);
        Assert.Equal(1.0, second.InternalKnots()[2]);
        Assert.Throws<ImmutableObjectException>(() => first.SetKnot(2, 1.5));
    }

    [Fact]
    public void Arrays_are_shared_only_when_equal_bit_for_bit_with_the_same_bounds()
    {
        NCollection_Array1<double> a = NCollection_Array1<double>.From([0.25, 0.5, 0.125]);
        Assert.Same(a, SharedArrays<double>.Shared(a));
        Assert.Same(a, SharedArrays<double>.Shared(NCollection_Array1<double>.From([0.25, 0.5, 0.125])));

        NCollection_Array1<double> bounds = NCollection_Array1<double>.From([0.25, 0.5, 0.125], lower: 0);
        Assert.Same(bounds, SharedArrays<double>.Shared(bounds));

        NCollection_Array1<double> zero = NCollection_Array1<double>.From([0.0, 7.5]);
        NCollection_Array1<double> minusZero = NCollection_Array1<double>.From([-0.0, 7.5]);
        Assert.Same(zero, SharedArrays<double>.Shared(zero));
        Assert.Same(minusZero, SharedArrays<double>.Shared(minusZero));

        // Empty and long arrays are left alone
        var empty = new NCollection_Array1<double>();
        Assert.Same(empty, SharedArrays<double>.Shared(empty));
        double[] many = new double[33];
        NCollection_Array1<double> longOne = NCollection_Array1<double>.From(many), longTwo = NCollection_Array1<double>.From(many);
        Assert.Same(longOne, SharedArrays<double>.Shared(longOne));
        Assert.Same(longTwo, SharedArrays<double>.Shared(longTwo));
    }

    [Fact]
    public void A_view_of_changing_storage_is_never_offered_to_others()
    {
        double[] storage = [0.3125, 0.6875];
        var view = new NCollection_Array1<double>(storage, 0, 1, 2);
        Assert.Same(view, SharedArrays<double>.Shared(view));

        NCollection_Array1<double> equal = NCollection_Array1<double>.From([0.3125, 0.6875]);
        Assert.Same(equal, SharedArrays<double>.Shared(equal));
        storage[0] = 9.0;
        Assert.Equal(0.3125, SharedArrays<double>.Shared(NCollection_Array1<double>.From([0.3125, 0.6875]))[1]);
    }
}
