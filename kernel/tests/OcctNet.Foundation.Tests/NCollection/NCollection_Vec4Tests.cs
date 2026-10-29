// Port of: src/FoundationClasses/TKernel/GTests/NCollection_Vec4_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class NCollection_Vec4Test
{
    [Fact]
    public void BasicConstruction()
    {
        NCollection_Vec4<float> aVec1 = new();
        Assert.Equal(0.0f, aVec1.x());
        Assert.Equal(0.0f, aVec1.y());
        Assert.Equal(0.0f, aVec1.z());
        Assert.Equal(0.0f, aVec1.w());

        NCollection_Vec4<float> aVec2 = new(1.0f, 2.0f, 3.0f, 4.0f);
        Assert.Equal(1.0f, aVec2.x());
        Assert.Equal(2.0f, aVec2.y());
        Assert.Equal(3.0f, aVec2.z());
        Assert.Equal(4.0f, aVec2.w());
    }

    [Fact]
    public void XyzMethod()
    {
        NCollection_Vec4<float> aVec = new(2.0f, 3.0f, 4.0f, 5.0f);
        NCollection_Vec3<float> aVec3 = aVec.xyz();

        Assert.Equal(2.0f, aVec3.x());
        Assert.Equal(3.0f, aVec3.y());
        Assert.Equal(4.0f, aVec3.z());
    }

    [Fact]
    public void MatrixMultiplicationAndTransformation()
    {
        NCollection_Mat4<float> aMatrix = new();
        aMatrix.Translate(new NCollection_Vec3<float>(4.0f, 3.0f, 1.0f));

        var aPoints1 = new NCollection_Vec4<float>[8];
        for (int aX = 0; aX < 2; ++aX)
        {
            for (int aY = 0; aY < 2; ++aY)
            {
                for (int aZ = 0; aZ < 2; ++aZ)
                {
                    aPoints1[aX * 2 * 2 + aY * 2 + aZ] = new NCollection_Vec4<float>(-1.0f + 2.0f * aX,
                                                                                    -1.0f + 2.0f * aY,
                                                                                    -1.0f + 2.0f * aZ,
                                                                                    1.0f);
                }
            }
        }

        var aPoints2 = new NCollection_Vec3<float>[8];
        for (int aPntIdx = 0; aPntIdx < 8; ++aPntIdx)
        {
            aPoints1[aPntIdx] = aMatrix * aPoints1[aPntIdx];
            aPoints2[aPntIdx] = aPoints1[aPntIdx].xyz() / aPoints1[aPntIdx].w();
        }

        float aSquareModulus = aPoints2[7].SquareModulus();
        Assert.Equal(45.0f, aSquareModulus, 0.001f);

        float aExpectedSquareMod0 = 3.0f * 3.0f + 2.0f * 2.0f + 0.0f * 0.0f;
        Assert.Equal(aExpectedSquareMod0, aPoints2[0].SquareModulus(), 0.001f);
    }

    [Fact]
    public void ComponentAccess()
    {
        NCollection_Vec4<float> aVec = new(1.5f, 2.5f, 3.5f, 4.5f);

        Assert.Equal(1.5f, aVec.x());
        Assert.Equal(2.5f, aVec.y());
        Assert.Equal(3.5f, aVec.z());
        Assert.Equal(4.5f, aVec.w());

        // C++ writes through x() = ...; the port writes through the internal indexer setter
        aVec[0] = 10.0f;
        aVec[1] = 20.0f;
        aVec[2] = 30.0f;
        aVec[3] = 40.0f;

        Assert.Equal(10.0f, aVec.x());
        Assert.Equal(20.0f, aVec.y());
        Assert.Equal(30.0f, aVec.z());
        Assert.Equal(40.0f, aVec.w());
    }

    [Fact]
    public void HomogeneousCoordinateDivision()
    {
        NCollection_Vec4<float> aVec = new(8.0f, 12.0f, 16.0f, 4.0f);
        NCollection_Vec3<float> aResult = aVec.xyz() / aVec.w();

        Assert.Equal(2.0f, aResult.x());
        Assert.Equal(3.0f, aResult.y());
        Assert.Equal(4.0f, aResult.z());
    }
}

/// <summary>Tests of the port's own (OCCT has no GTests for Vec2, Vec3, Mat3 and Mat4 beyond the above).</summary>
public class NCollection_VecMatTest
{
    [Fact]
    public void Vec3_CrossDotNormalize()
    {
        NCollection_Vec3<double> aX = NCollection_Vec3<double>.DX();
        NCollection_Vec3<double> aY = NCollection_Vec3<double>.DY();
        Assert.Equal(NCollection_Vec3<double>.DZ(), NCollection_Vec3<double>.Cross(aX, aY));
        Assert.Equal(0.0, aX.Dot(aY));
        NCollection_Vec3<double> aV = new(3.0, 0.0, 4.0);
        Assert.Equal(5.0, aV.Modulus());
        Assert.Equal(new NCollection_Vec3<double>(0.6, 0.0, 0.8), aV.Normalized());
        Assert.Equal(new NCollection_Vec3<double>(), new NCollection_Vec3<double>().Normalized());
        Assert.Equal(new NCollection_Vec3<double>(1.5, 0.0, 2.0), NCollection_Vec3<double>.GetLERP(new NCollection_Vec3<double>(), aV, 0.5));
    }

    [Fact]
    public void Vec_DivisionByScalar()
    {
        // Vec3/Vec4 of reals multiply by the inverse, as OCCT's specialisation; Vec2 divides
        NCollection_Vec3<double> aV3 = new NCollection_Vec3<double>(1.0, 2.0, 3.0) / 3.0;
        Assert.Equal(1.0 * (1.0 / 3.0), aV3.x());
        NCollection_Vec2<double> aV2 = new NCollection_Vec2<double>(1.0, 2.0) / 3.0;
        Assert.Equal(1.0 / 3.0, aV2.x());
        NCollection_Vec3<int> aVi = new NCollection_Vec3<int>(7, 8, 9) / 2;
        Assert.Equal(new NCollection_Vec3<int>(3, 4, 4), aVi);
    }

    [Fact]
    public void Vec_CwiseAndConversion()
    {
        NCollection_Vec4<int> aA = new(1, -5, 3, 8);
        NCollection_Vec4<int> aB = new(2, -6, 3, 0);
        Assert.Equal(new NCollection_Vec4<int>(1, -6, 3, 0), aA.cwiseMin(aB));
        Assert.Equal(new NCollection_Vec4<int>(2, -5, 3, 8), aA.cwiseMax(aB));
        Assert.Equal(8, aA.maxComp());
        Assert.Equal(-5, aA.minComp());
        Assert.Equal(new NCollection_Vec4<int>(1, 5, 3, 8), aA.cwiseAbs());
        Assert.Equal(new NCollection_Vec2<int>(1, -2), NCollection_Vec2<int>.From(new NCollection_Vec2<double>(1.9, -2.9)));
        Assert.Equal("NCollection_Vec3(1, 2.5, -3)", new NCollection_Vec3<double>(1.0, 2.5, -3.0).ToString());
        Assert.Equal(new NCollection_Vec3<double>(2.0, 1.0, 3.0), new NCollection_Vec3<double>(1.0, 2.0, 3.0).yxz());
    }

    [Fact]
    public void Mat3_Inverse()
    {
        NCollection_Mat3<double> aMat = new();
        Assert.True(aMat.IsIdentity());
        aMat.SetRow(0, new NCollection_Vec3<double>(2.0, 0.0, 1.0));
        aMat.SetRow(1, new NCollection_Vec3<double>(0.0, 3.0, 0.0));
        aMat.SetRow(2, new NCollection_Vec3<double>(1.0, 0.0, 1.0));
        Assert.Equal(3.0, aMat.Determinant());
        NCollection_Mat3<double> anInv = aMat.Inverted();
        Assert.True((aMat * anInv).IsIdentity());
        // OCCT: Adjoint() is the cofactor matrix (the transpose of the adjugate it is documented as)
        Assert.Equal(aMat.Adjoint().Transposed(), anInv * 3.0);

        NCollection_Mat3<double> aSingular = NCollection_Mat3<double>.Zero();
        NCollection_Mat3<double> anUntouched = new();
        Assert.False(aSingular.Inverted(ref anUntouched, out double aDet));
        Assert.Equal(0.0, aDet);
        Assert.True(anUntouched.IsIdentity());
        Assert.ThrowsAny<Standard_ConstructionError>(() => aSingular.Inverted());
    }

    [Fact]
    public void Mat4_InverseAndTranspose()
    {
        NCollection_Mat4<double> aMat = new();
        aMat.Translate(new NCollection_Vec3<double>(1.0, 2.0, 3.0));
        aMat.SetValue(0, 0, 2.0);
        Assert.Equal(new NCollection_Vec4<double>(1.0, 2.0, 3.0, 1.0), aMat.GetColumn(3));
        Assert.True(aMat.Inverted(out NCollection_Mat4<double> anInv, out double aDet));
        Assert.Equal(2.0, aDet);
        Assert.True((aMat * anInv).IsIdentity());
        Assert.Equal(aMat, aMat.Transposed().Transposed());
        Assert.Equal(aMat.GetRow(3), aMat.Transposed().GetColumn(3));
        Assert.Equal(2.0, aMat.DeterminantMat3());
        Assert.Equal(new NCollection_Vec3<double>(2.0, 1.0, 1.0), aMat.GetMat3().GetDiagonal());
        // OCCT: Adjoint() is the negated cofactor matrix, not the adjugate it is documented as
        Assert.Equal(aMat.Adjoint(), -(anInv * aDet).Transposed());
        Assert.False(NCollection_Mat4<double>.Zero().Inverted(out _));
        Assert.Equal(new NCollection_Mat4<float>(), NCollection_Mat4<float>.From(new NCollection_Mat4<double>()));
    }
}
