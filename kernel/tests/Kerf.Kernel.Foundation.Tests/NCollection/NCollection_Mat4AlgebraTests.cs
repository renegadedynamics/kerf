// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>NCollection_Mat4 and its Vec4, by the algebra they stand for (unit 11I): products and inverses by hand.</summary>
public class NCollection_Mat4AlgebraTests
{
    private static NCollection_Mat4<double> Sample()
    {
        // translation by (1, 2, 3) after scaling by (2, 4, 5)
        NCollection_Mat4<double> aMat = NCollection_Mat4<double>.Identity().WithDiagonal(new NCollection_Vec4<double>(2.0, 4.0, 5.0, 1.0));
        return aMat.WithColumn(3, new NCollection_Vec4<double>(1.0, 2.0, 3.0, 1.0));
    }

    [Fact]
    public void Identity_and_zero()
    {
        Assert.True(NCollection_Mat4<double>.Identity().IsIdentity());
        Assert.False(NCollection_Mat4<double>.Identity().IsZero());
        Assert.True(NCollection_Mat4<double>.Zero().IsZero());
        Assert.False(NCollection_Mat4<double>.Zero().IsIdentity());
        Assert.Equal(4, NCollection_Mat4<double>.Rows());
        Assert.Equal(4, NCollection_Mat4<double>.Cols());
    }

    [Fact]
    public void Rows_columns_and_the_diagonal_read_back_what_went_in()
    {
        NCollection_Mat4<double> aMat = Sample();
        Assert.Equal(new NCollection_Vec4<double>(1.0, 2.0, 3.0, 1.0), aMat.GetColumn(3));
        Assert.Equal(new NCollection_Vec4<double>(2.0, 0.0, 0.0, 1.0), aMat.GetRow(0));
        Assert.Equal(new NCollection_Vec4<double>(2.0, 4.0, 5.0, 1.0), aMat.GetDiagonal());
        Assert.Equal(2.0, aMat[1, 3]);
        Assert.Equal(7.0, aMat.WithValue(1, 3, 7)[1, 3]);
        Assert.Equal(2.0, aMat[1, 3]);
        Assert.Equal(9.0, aMat.WithRow(2, new NCollection_Vec4<double>(9.0, 8.0, 7.0, 6.0))[2, 0]);
        Assert.Equal(6.0, aMat.WithRow(2, new NCollection_Vec4<double>(9.0, 8.0, 7.0, 6.0))[2, 3]);
        Assert.Equal(9.0, aMat.WithRow(2, new NCollection_Vec3<double>(9.0, 8.0, 7.0))[2, 0]);
        Assert.Equal(9.0, aMat.WithColumn(2, new NCollection_Vec3<double>(9.0, 8.0, 7.0))[0, 2]);
        Assert.Equal(new NCollection_Vec4<double>(7.0, 8.0, 9.0, 1.0), NCollection_Mat4<double>.Identity().WithDiagonal(new NCollection_Vec3<double>(7.0, 8.0, 9.0)).GetDiagonal());
        Assert.Equal(4.0, aMat.GetMat3()[1, 1]);
    }

    [Fact]
    public void A_matrix_times_a_point_scales_then_translates()
    {
        NCollection_Vec4<double> aPoint = Sample() * new NCollection_Vec4<double>(1.0, 1.0, 1.0, 1.0);
        Assert.Equal(new NCollection_Vec4<double>(3.0, 6.0, 8.0, 1.0), aPoint);

        // translated composes on the right: the translation is scaled first
        NCollection_Vec4<double> aMoved = NCollection_Mat4<double>.Identity().Translated(new NCollection_Vec3<double>(1.0, 2.0, 3.0))
                                          * new NCollection_Vec4<double>(1.0, 1.0, 1.0, 1.0);
        Assert.Equal(new NCollection_Vec4<double>(2.0, 3.0, 4.0, 1.0), aMoved);
    }

    [Fact]
    public void Products_sums_and_scales_element_by_element()
    {
        NCollection_Mat4<double> aMat = Sample();
        Assert.Equal(aMat, aMat * NCollection_Mat4<double>.Identity());
        Assert.Equal(aMat, NCollection_Mat4<double>.Multiply(NCollection_Mat4<double>.Identity(), aMat));
        Assert.Equal(aMat, NCollection_Mat4<double>.Identity().Multiplied(aMat));

        Assert.Equal(4.0, (aMat * 2.0)[0, 0]);
        Assert.Equal(1.0, (aMat / 2.0)[0, 0]);
        Assert.Equal(4.0, (aMat + aMat)[0, 0]);
        Assert.True((aMat - aMat).IsZero());
        Assert.Equal(-2.0, (-aMat)[0, 0]);

        // a transpose moves the translation to the bottom row
        NCollection_Mat4<double> aTransposed = aMat.Transposed();
        Assert.Equal(3.0, aTransposed[3, 2]);
        Assert.Equal(aMat, aTransposed.Transposed());
    }

    [Fact]
    public void An_inverse_undoes_the_matrix_and_gives_its_determinant()
    {
        NCollection_Mat4<double> aMat = Sample();
        Assert.True(aMat.Inverted(out NCollection_Mat4<double> anInverse, out double aDet));
        Assert.Equal(40.0, aDet, 9);
        NCollection_Mat4<double> aProduct = aMat * anInverse;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                Assert.Equal(i == j ? 1.0 : 0.0, aProduct[i, j], 9);
            }
        }
        Assert.Equal(0.5, aMat.Inverted()[0, 0], 12);
        Assert.True(aMat.Inverted(out _));
        Assert.Equal(40.0, aMat.DeterminantMat3(), 9);

        // no inverse of a singular matrix
        Assert.False(NCollection_Mat4<double>.Zero().Inverted(out _));
        Assert.False(NCollection_Mat4<double>.Zero().Inverted(out _, out double aZeroDet));
        Assert.Equal(0.0, aZeroDet);
        EngineAssert.Throws<Kerf.Engine.Foundation.ConstructionErrorException>(() => NCollection_Mat4<double>.Zero().Inverted());
    }

    [Fact]
    public void Equality_copy_and_conversion()
    {
        NCollection_Mat4<double> aMat = Sample();
        Assert.True(aMat == Sample());
        Assert.True(aMat != NCollection_Mat4<double>.Identity());
        Assert.True(aMat.Equals((object)Sample()));
        Assert.False(aMat.Equals("a matrix"));
        Assert.True(aMat.IsEqual(Sample()));
        Assert.Equal(aMat.GetHashCode(), Sample().GetHashCode());
        Assert.NotEmpty(aMat.ToString());
        double[] aCopy = new double[16];
        aMat.CopyTo(aCopy);
        Assert.Equal(aMat[1, 3], aCopy[3 * 4 + 1]);
        Assert.Equal(2f, NCollection_Mat4<float>.From(aMat)[0, 0]);
    }

    [Fact]
    public void Vec4s_swizzle_and_combine()
    {
        NCollection_Vec4<double> aV = new(1.0, 2.0, 3.0, 4.0);
        Assert.Equal((1.0, 2.0, 3.0, 4.0), (aV.R(), aV.G(), aV.B(), aV.A()));
        Assert.Equal(new NCollection_Vec2<double>(4.0, 1.0), aV.Wx());
        Assert.Equal(new NCollection_Vec2<double>(1.0, 3.0), aV.Xz());
        Assert.Equal(new NCollection_Vec2<double>(4.0, 3.0), aV.Wz());
        Assert.Equal(new NCollection_Vec3<double>(3.0, 2.0, 1.0), aV.Zyx());
        Assert.Equal(new NCollection_Vec3<double>(4.0, 2.0, 1.0), aV.Wyx());
        Assert.Equal(new NCollection_Vec3<double>(1.0, 3.0, 4.0), aV.Xzw());
        Assert.Equal(new NCollection_Vec3<double>(3.0, 4.0, 1.0), aV.Zwx());
        Assert.Equal(new NCollection_Vec4<double>(1.0, 2.0, 0.0, 0.0), new NCollection_Vec4<double>(new NCollection_Vec2<double>(1.0, 2.0)));
        Assert.Equal(new NCollection_Vec4<double>(1.0, 2.0, 3.0, 9.0), new NCollection_Vec4<double>(new NCollection_Vec3<double>(1.0, 2.0, 3.0), 9.0));
        Assert.Equal(new NCollection_Vec4<double>(5.0), new NCollection_Vec4<double>(5.0, 5.0, 5.0, 5.0));
        Assert.Equal(new NCollection_Vec4<float>(1f, 2f, 3f, 4f), NCollection_Vec4<float>.From(aV));
        Assert.Equal(new NCollection_Vec4<double>(1.0, 9.0, 3.0, 4.0), aV.WithValue(1, 9.0));
        Assert.Equal(new NCollection_Vec4<double>(2.0, 4.0, 6.0, 8.0), aV.Added(aV));
        Assert.Equal(new NCollection_Vec4<double>(0.0), aV.Subtracted(aV));
        Assert.Equal(new NCollection_Vec4<double>(0.5, 1.0, 1.5, 2.0), aV.Divided(2.0));
        Assert.Equal(new NCollection_Vec4<double>(1.0), aV.Divided(aV));
        Assert.Equal(new NCollection_Vec4<double>(1.0, 4.0, 9.0, 16.0), aV.Multiplied(aV));
    }
}
