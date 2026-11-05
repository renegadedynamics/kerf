// Port of OCCT's src/ModelingData/TKG3d/GTests/Geom_BSplineSurface_Test.cxx (V8_0_1), plus Kerf's own checks.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Geom_BSplineSurface. WeightsArray_* check copies, since the port
/// returns copies where OCCT returns references (IsDeletable and reference identity don't apply).
/// </summary>
public class Geom_BSplineSurfaceTests
{
    private readonly BSplineSurface myOriginalSurface;

    public Geom_BSplineSurfaceTests()
    {
        // Create a simple BSpline surface for testing
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i, j, (i + j) * 0.1);
            }
        }
        myOriginalSurface = new BSplineSurface(aPoles, Knots(0.0, 1.0), Knots(0.0, 1.0), Mults(3, 3), Mults(3, 3), 2, 2);
    }

    private static NCollection_Array1<double> Knots(params double[] theValues) => NCollection_Array1<double>.From(theValues);

    private static NCollection_Array1<int> Mults(params int[] theValues) => NCollection_Array1<int>.From(theValues);

    /// <summary>The 3 x 3 rational test surface of SetWeight, SetWeightRow and SetWeightCol.</summary>
    private static BSplineSurface RationalBump()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i, j, (i == 2 && j == 2) ? 1.0 : 0.0);
                aWeights[i, j] = 1.0;
            }
        }
        aWeights[2, 2] = 2.0;
        return new BSplineSurface(aPoles, aWeights, Knots(0.0, 1.0), Knots(0.0, 1.0), Mults(3, 3), Mults(3, 3), 2, 2);
    }

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        BSplineSurface aCopiedSurface = new(myOriginalSurface);
        Assert.Equal(myOriginalSurface.UDegree, aCopiedSurface.UDegree);
        Assert.Equal(myOriginalSurface.VDegree, aCopiedSurface.VDegree);
        Assert.Equal(myOriginalSurface.UPoleCount, aCopiedSurface.UPoleCount);
        Assert.Equal(myOriginalSurface.VPoleCount, aCopiedSurface.VPoleCount);
        Assert.Equal(myOriginalSurface.UKnotCount, aCopiedSurface.UKnotCount);
        Assert.Equal(myOriginalSurface.VKnotCount, aCopiedSurface.VKnotCount);
        Assert.Equal(myOriginalSurface.IsUPeriodic, aCopiedSurface.IsUPeriodic);
        Assert.Equal(myOriginalSurface.IsVPeriodic, aCopiedSurface.IsVPeriodic);
    }

    [Fact]
    public void CopyConstructorPoles()
    {
        BSplineSurface aCopiedSurface = new(myOriginalSurface);
        for (int i = 1; i <= myOriginalSurface.UPoleCount; ++i)
        {
            for (int j = 1; j <= myOriginalSurface.VPoleCount; ++j)
            {
                Assert.True(myOriginalSurface.Pole(i, j).IsEqual(aCopiedSurface.Pole(i, j), 1e-10));
            }
        }
    }

    [Fact]
    public void CopyConstructorKnots()
    {
        BSplineSurface aCopiedSurface = new(myOriginalSurface);
        for (int i = 1; i <= myOriginalSurface.UKnotCount; ++i)
        {
            Assert.Equal(myOriginalSurface.UKnot(i), aCopiedSurface.UKnot(i));
            Assert.Equal(myOriginalSurface.UMultiplicity(i), aCopiedSurface.UMultiplicity(i));
        }
        for (int i = 1; i <= myOriginalSurface.VKnotCount; ++i)
        {
            Assert.Equal(myOriginalSurface.VKnot(i), aCopiedSurface.VKnot(i));
            Assert.Equal(myOriginalSurface.VMultiplicity(i), aCopiedSurface.VMultiplicity(i));
        }
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        Geometry3D aCopiedGeom = myOriginalSurface.Copy();
        BSplineSurface? aCopiedSurface = aCopiedGeom as BSplineSurface;
        Assert.NotNull(aCopiedSurface);
        Assert.Equal(myOriginalSurface.UDegree, aCopiedSurface.UDegree);
        Assert.Equal(myOriginalSurface.VDegree, aCopiedSurface.VDegree);
        for (double u = 0.0; u <= 1.0; u += 0.5)
        {
            for (double v = 0.0; v <= 1.0; v += 0.5)
            {
                Assert.True(myOriginalSurface.Value(u, v).IsEqual(aCopiedSurface.Value(u, v), 1e-10));
            }
        }
    }

    [Fact]
    public void CopyIndependence()
    {
        BSplineSurface aCopiedSurface = new(myOriginalSurface);
        Point3D aNewPole = new(10, 10, 10);
        myOriginalSurface.SetPole(2, 2, aNewPole);
        Assert.False(aCopiedSurface.Pole(2, 2).IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void Evaluation_D0()
    {
        myOriginalSurface.D0(0.0, 0.0, out Point3D aPnt);
        Assert.True(aPnt.IsEqual(new Point3D(1, 1, 0.2), 1e-10));
        myOriginalSurface.D0(1.0, 1.0, out aPnt);
        Assert.True(aPnt.IsEqual(new Point3D(3, 3, 0.6), 1e-10));
    }

    [Fact]
    public void Evaluation_D1()
    {
        myOriginalSurface.D1(0.5, 0.5, out _, out Vector3D aD1U, out Vector3D aD1V);
        Assert.True(aD1U.Magnitude > 0.0);
        Assert.True(aD1V.Magnitude > 0.0);
    }

    [Fact]
    public void Evaluation_D2()
    {
        myOriginalSurface.D2(0.5, 0.5, out _, out Vector3D aD1U, out _, out _, out _, out _);
        Assert.True(aD1U.Magnitude > 0.0);
    }

    [Fact]
    public void Evaluation_DN()
    {
        Assert.True(myOriginalSurface.DN(0.5, 0.5, 1, 0).Magnitude > 0.0);
        Assert.True(myOriginalSurface.DN(0.5, 0.5, 0, 1).Magnitude > 0.0);
    }

    [Fact]
    public void Properties()
    {
        Assert.Equal(2, myOriginalSurface.UDegree);
        Assert.Equal(2, myOriginalSurface.VDegree);
        Assert.False(myOriginalSurface.IsUPeriodic);
        Assert.False(myOriginalSurface.IsVPeriodic);
        Assert.False(myOriginalSurface.IsURational);
        Assert.False(myOriginalSurface.IsVRational);
        Assert.Equal(3, myOriginalSurface.UPoleCount);
        Assert.Equal(3, myOriginalSurface.VPoleCount);
        Assert.Equal(2, myOriginalSurface.UKnotCount);
        Assert.Equal(2, myOriginalSurface.VKnotCount);
    }

    [Fact]
    public void Bounds()
    {
        myOriginalSurface.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(0.0, aU1);
        Assert.Equal(1.0, aU2);
        Assert.Equal(0.0, aV1);
        Assert.Equal(1.0, aV2);
    }

    [Fact]
    public void SetPole()
    {
        Point3D aNewPole = new(5, 5, 5);
        myOriginalSurface.SetPole(2, 2, aNewPole);
        Assert.True(myOriginalSurface.Pole(2, 2).IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void SetWeight()
    {
        BSplineSurface aSurf = RationalBump();
        Assert.True(aSurf.IsURational || aSurf.IsVRational);
        Point3D aValBefore = aSurf.Value(0.5, 0.5);
        aSurf.SetWeight(2, 2, 10.0);
        Assert.Equal(10.0, aSurf.Weight(2, 2));
        Point3D aValAfter = aSurf.Value(0.5, 0.5);
        // Higher weight at the elevated center pole pulls surface up
        Assert.True(aValAfter.Z > aValBefore.Z);
    }

    [Fact]
    public void IncreaseDegree()
    {
        Point3D aValBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.IncreaseDegree(4, 3);
        Assert.Equal(4, myOriginalSurface.UDegree);
        Assert.Equal(3, myOriginalSurface.VDegree);
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void InsertUKnot()
    {
        Point3D aValBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.UKnotCount);
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void InsertVKnot()
    {
        Point3D aValBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.InsertVKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.VKnotCount);
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void RemoveUKnot()
    {
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.UKnotCount);
        Point3D aValBefore = myOriginalSurface.Value(0.25, 0.5);
        Assert.True(myOriginalSurface.RemoveUKnot(2, 0, 1e-6));
        Assert.Equal(2, myOriginalSurface.UKnotCount);
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.25, 0.5), 1e-6));
    }

    [Fact]
    public void Segment()
    {
        Point3D aPntBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.Segment(0.25, 0.75, 0.25, 0.75);
        myOriginalSurface.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(0.25, aU1, 1e-10);
        Assert.Equal(0.75, aU2, 1e-10);
        Assert.Equal(0.25, aV1, 1e-10);
        Assert.Equal(0.75, aV2, 1e-10);
        Assert.True(aPntBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-6));
    }

    [Fact]
    public void ExchangeUV()
    {
        Point3D aValBefore = myOriginalSurface.Value(0.3, 0.7);
        int aUDeg = myOriginalSurface.UDegree;
        int aVDeg = myOriginalSurface.VDegree;
        myOriginalSurface.ExchangeUV();
        Assert.Equal(aVDeg, myOriginalSurface.UDegree);
        Assert.Equal(aUDeg, myOriginalSurface.VDegree);
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.7, 0.3), 1e-10));
    }

    [Fact]
    public void UReverse()
    {
        Point3D aValBefore = myOriginalSurface.Value(0.3, 0.5);
        myOriginalSurface.UReverse();
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.7, 0.5), 1e-10));
    }

    [Fact]
    public void VReverse()
    {
        Point3D aValBefore = myOriginalSurface.Value(0.5, 0.3);
        myOriginalSurface.VReverse();
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.7), 1e-10));
    }

    [Fact]
    public void UIso()
    {
        Curve3D anIso = myOriginalSurface.UIso(0.5);
        Assert.NotNull(anIso);

        for (double v = 0.0; v <= 1.0; v += 0.25)
        {
            Point3D aSurfPnt = myOriginalSurface.Value(0.5, v);
            Point3D anIsoPnt = anIso.Value(v);
            Assert.True(aSurfPnt.IsEqual(anIsoPnt, 1e-10));
        }
    }

    [Fact]
    public void VIso()
    {
        Curve3D anIso = myOriginalSurface.VIso(0.5);
        Assert.NotNull(anIso);

        for (double u = 0.0; u <= 1.0; u += 0.25)
        {
            Point3D aSurfPnt = myOriginalSurface.Value(u, 0.5);
            Point3D anIsoPnt = anIso.Value(u);
            Assert.True(aSurfPnt.IsEqual(anIsoPnt, 1e-10));
        }
    }

    [Fact]
    public void Resolution()
    {
        myOriginalSurface.Resolution(1.0, out double aUTol, out double aVTol);
        Assert.True(aUTol > 0.0);
        Assert.True(aVTol > 0.0);
    }

    [Fact]
    public void Transform()
    {
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(10, 20, 30));
        Point3D aPtBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.Transform(aTrsf);
        Point3D aPtAfter = myOriginalSurface.Value(0.5, 0.5);
        Assert.Equal(aPtBefore.X + 10.0, aPtAfter.X, 1e-10);
        Assert.Equal(aPtBefore.Y + 20.0, aPtAfter.Y, 1e-10);
        Assert.Equal(aPtBefore.Z + 30.0, aPtAfter.Z, 1e-10);
    }

    [Fact]
    public void KnotsAccess()
    {
        IReadOnlyList<double> aUKnots = myOriginalSurface.UKnots;
        Assert.Equal(2, aUKnots.Count);
        Assert.Equal(0.0, aUKnots[0]);
        Assert.Equal(1.0, aUKnots[1]);
        Assert.Equal(2, myOriginalSurface.VKnots.Count);
        IReadOnlyList<int> aUMults = myOriginalSurface.UMultiplicities;
        Assert.Equal(2, aUMults.Count);
        Assert.Equal(3, aUMults[0]);
        Assert.Equal(2, myOriginalSurface.VMultiplicities.Count);
    }

    [Fact]
    public void WeightsAccess_NonRational()
    {
        Assert.Null(myOriginalSurface.WeightsOrNull());
    }

    [Fact]
    public void PeriodicSurface_SetUNotPeriodic()
    {
        // U-periodic surface: degree 2, 4 U-knots of mult 1 -> NbUPoles = 1+2 = 3
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(Math.Cos(2.0 * Math.PI * (i - 1) / 3.0), Math.Sin(2.0 * Math.PI * (i - 1) / 3.0), j * 0.5);
            }
        }
        BSplineSurface aSurf = new(aPoles, Knots(0, 1, 2, 3), Knots(0, 1), Mults(1, 1, 1, 1), Mults(3, 3), 2, 2, true, false);
        Assert.True(aSurf.IsUPeriodic);
        Point3D aVal = aSurf.Value(0.5, 0.5);
        aSurf.SetUNotPeriodic();
        Assert.False(aSurf.IsUPeriodic);
        Assert.True(aVal.IsEqual(aSurf.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void PeriodicSurface_SetVNotPeriodic()
    {
        // V-periodic surface: degree 2, 4 V-knots of mult 1 -> NbVPoles = 1+2 = 3
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i * 0.5, Math.Cos(2.0 * Math.PI * (j - 1) / 3.0), Math.Sin(2.0 * Math.PI * (j - 1) / 3.0));
            }
        }
        BSplineSurface aSurf = new(aPoles, Knots(0, 1), Knots(0, 1, 2, 3), Mults(3, 3), Mults(1, 1, 1, 1), 2, 2, false, true);
        Assert.True(aSurf.IsVPeriodic);
        Point3D aVal = aSurf.Value(0.5, 0.5);
        aSurf.SetVNotPeriodic();
        Assert.False(aSurf.IsVPeriodic);
        Assert.True(aVal.IsEqual(aSurf.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void RationalSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i, j, 0);
                aWeights[i, j] = 1.0;
            }
        }
        aWeights[2, 2] = 3.0;
        BSplineSurface aSurf = new(aPoles, aWeights, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(3, 3), 2, 2);
        Assert.True(aSurf.IsURational);
        Assert.True(aSurf.IsVRational);
        Grid<double>? aW = aSurf.WeightsOrNull();
        Assert.NotNull(aW);
        Assert.Equal(3.0, aW.Value[1, 1]);
        Point3D aMid = aSurf.Value(0.5, 0.5);
        // High weight at center should pull surface toward (2,2,0)
        Assert.Equal(2.0, aMid.X, 0.1);
        Assert.Equal(2.0, aMid.Y, 0.1);
    }

    [Fact]
    public void LocalEvaluation()
    {
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.UKnotCount);
        Point3D aGlobal = myOriginalSurface.Value(0.25, 0.5);
        myOriginalSurface.LocalD0(0.25, 0.5, 1, 2, 1, 2, out Point3D aLocal);
        Assert.True(aGlobal.IsEqual(aLocal, 1e-10));
    }

    [Fact]
    public void SetPoleRow()
    {
        NCollection_Array1<Point3D> aNewRow = NCollection_Array1<Point3D>.From([new Point3D(10, 1, 0), new Point3D(10, 2, 0), new Point3D(10, 3, 0)]);
        myOriginalSurface.SetPoleRow(2, aNewRow);
        for (int j = 1; j <= 3; ++j)
        {
            Assert.Equal(10.0, myOriginalSurface.Pole(2, j).X, 1e-10);
        }
    }

    [Fact]
    public void SetPoleCol()
    {
        NCollection_Array1<Point3D> aNewCol = NCollection_Array1<Point3D>.From([new Point3D(1, 10, 0), new Point3D(2, 10, 0), new Point3D(3, 10, 0)]);
        myOriginalSurface.SetPoleCol(2, aNewCol);
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(10.0, myOriginalSurface.Pole(i, 2).Y, 1e-10);
        }
    }

    [Fact]
    public void SetWeightRow()
    {
        BSplineSurface aSurf = RationalBump();
        aSurf.SetWeightRow(2, Knots(3.0, 4.0, 5.0));
        Assert.Equal(3.0, aSurf.Weight(2, 1));
        Assert.Equal(4.0, aSurf.Weight(2, 2));
        Assert.Equal(5.0, aSurf.Weight(2, 3));
    }

    [Fact]
    public void SetWeightCol()
    {
        BSplineSurface aSurf = RationalBump();
        aSurf.SetWeightCol(2, Knots(6.0, 7.0, 8.0));
        Assert.Equal(6.0, aSurf.Weight(1, 2));
        Assert.Equal(7.0, aSurf.Weight(2, 2));
        Assert.Equal(8.0, aSurf.Weight(3, 2));
    }

    [Fact]
    public void RemoveVKnot()
    {
        myOriginalSurface.InsertVKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.VKnotCount);
        Point3D aValBefore = myOriginalSurface.Value(0.5, 0.25);
        Assert.True(myOriginalSurface.RemoveVKnot(2, 0, 1e-6));
        Assert.Equal(2, myOriginalSurface.VKnotCount);
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.25), 1e-6));
    }

    [Fact]
    public void MovePoint()
    {
        Point3D aTarget = new(2, 2, 1);
        myOriginalSurface.MovePoint(0.5, 0.5, aTarget, 1, myOriginalSurface.UPoleCount, 1, myOriginalSurface.VPoleCount,
                                    out _, out _, out _, out _);
        Assert.True(myOriginalSurface.Value(0.5, 0.5).IsEqual(aTarget, 1e-6));
    }

    [Fact]
    public void LocateU()
    {
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        myOriginalSurface.LocateU(0.25, 1e-10, out int anI1, out int anI2);
        Assert.True(anI1 >= 1);
        Assert.True(anI2 <= myOriginalSurface.UKnotCount);
    }

    [Fact]
    public void LocateV()
    {
        myOriginalSurface.InsertVKnot(0.5, 1, 0.0);
        myOriginalSurface.LocateV(0.25, 1e-10, out int anI1, out int anI2);
        Assert.True(anI1 >= 1);
        Assert.True(anI2 <= myOriginalSurface.VKnotCount);
    }

    [Fact]
    public void RationalSurface_SetUNotPeriodic()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(Math.Cos(2.0 * Math.PI * (i - 1) / 3.0), Math.Sin(2.0 * Math.PI * (i - 1) / 3.0), j * 0.5);
                aWeights[i, j] = 1.0 + 0.5 * (i - 1);
            }
        }
        BSplineSurface aSurf = new(aPoles, aWeights, Knots(0, 1, 2, 3), Knots(0, 1), Mults(1, 1, 1, 1), Mults(3, 3), 2, 2, true, false);
        Assert.True(aSurf.IsUPeriodic);
        Assert.True(aSurf.IsURational || aSurf.IsVRational);
        Point3D aVal = aSurf.Value(0.5, 0.5);
        aSurf.SetUNotPeriodic();
        Assert.False(aSurf.IsUPeriodic);
        Assert.True(aVal.IsEqual(aSurf.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void RationalSurface_SetVNotPeriodic()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i * 0.5, Math.Cos(2.0 * Math.PI * (j - 1) / 3.0), Math.Sin(2.0 * Math.PI * (j - 1) / 3.0));
                aWeights[i, j] = 1.0 + 0.5 * (j - 1);
            }
        }
        BSplineSurface aSurf = new(aPoles, aWeights, Knots(0, 1), Knots(0, 1, 2, 3), Mults(3, 3), Mults(1, 1, 1, 1), 2, 2, false, true);
        Assert.True(aSurf.IsVPeriodic);
        Assert.True(aSurf.IsURational || aSurf.IsVRational);
        Point3D aVal = aSurf.Value(0.5, 0.5);
        aSurf.SetVNotPeriodic();
        Assert.False(aSurf.IsVPeriodic);
        Assert.True(aVal.IsEqual(aSurf.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void Evaluation_D3()
    {
        myOriginalSurface.D3(0.5, 0.5, out _, out _, out _, out _, out _, out _, out Vector3D aD3U, out Vector3D aD3V, out _, out _);
        // Degree 2 surface: D3 should be zero
        Assert.Equal(0.0, aD3U.Magnitude, 1e-10);
        Assert.Equal(0.0, aD3V.Magnitude, 1e-10);
    }

    [Fact]
    public void IncreaseUMultiplicity()
    {
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(1, myOriginalSurface.UMultiplicity(2));
        Point3D aValBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.IncreaseUMultiplicity(2, 2);
        Assert.Equal(2, myOriginalSurface.UMultiplicity(2));
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void SetUKnot()
    {
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        myOriginalSurface.SetUKnot(2, 0.6);
        Assert.Equal(0.6, myOriginalSurface.UKnot(2));
    }

    [Fact]
    public void SetVKnot()
    {
        myOriginalSurface.InsertVKnot(0.5, 1, 0.0);
        myOriginalSurface.SetVKnot(2, 0.6);
        Assert.Equal(0.6, myOriginalSurface.VKnot(2));
    }

    [Fact]
    public void RationalSurface_UIso()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i, j, (i + j) * 0.1);
                aWeights[i, j] = 1.0 + 0.5 * ((i - 1) + (j - 1));
            }
        }

        NCollection_Array1<double> anUKnots = new(1, 2), aVKnots = new(1, 2);
        anUKnots[1] = 0.0;
        anUKnots[2] = 1.0;
        aVKnots[1] = 0.0;
        aVKnots[2] = 1.0;

        NCollection_Array1<int> anUMults = new(1, 2), aVMults = new(1, 2);
        anUMults[1] = 3;
        anUMults[2] = 3;
        aVMults[1] = 3;
        aVMults[2] = 3;

        BSplineSurface aSurf = new(aPoles, aWeights, anUKnots, aVKnots, anUMults, aVMults, 2, 2);

        Curve3D anIso = aSurf.UIso(0.5);
        Assert.NotNull(anIso);

        for (double v = 0.0; v <= 1.0; v += 0.25)
        {
            Point3D aSurfPnt = aSurf.Value(0.5, v);
            Point3D anIsoPnt = anIso.Value(v);
            Assert.True(aSurfPnt.IsEqual(anIsoPnt, 1e-10));
        }
    }

    [Fact]
    public void CopyIndependence_Knots()
    {
        BSplineSurface aCopy = new(myOriginalSurface);
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(2, aCopy.UKnotCount);
        Assert.Equal(3, myOriginalSurface.UKnotCount);
    }

    [Fact]
    public void WeightsArray_NonRational_ReturnsUnitWeights()
    {
        Assert.False(myOriginalSurface.IsURational);
        Assert.False(myOriginalSurface.IsVRational);
        Grid<double> aWeights = myOriginalSurface.Weights;
        Assert.Equal(myOriginalSurface.UPoleCount, aWeights.RowCount);
        Assert.Equal(myOriginalSurface.VPoleCount, aWeights.ColumnCount);
        for (int i = 0; i < aWeights.RowCount; ++i)
        {
            for (int j = 0; j < aWeights.ColumnCount; ++j)
            {
                Assert.Equal(1.0, aWeights[i, j]);
            }
        }
    }

    [Fact]
    public void WeightsArray_Rational_ReturnsOwning()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeightsIn = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i, j, 0);
                aWeightsIn[i, j] = (i == 2 && j == 2) ? 2.0 : 1.0;
            }
        }
        BSplineSurface aRational = new(aPoles, aWeightsIn, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(3, 3), 2, 2);
        Grid<double> aWeights = aRational.Weights;
        Assert.Equal(3, aWeights.RowCount);
        Assert.Equal(3, aWeights.ColumnCount);
        Assert.Equal(2.0, aWeights[1, 1]);
        Assert.Equal(1.0, aWeights[0, 0]);
        Assert.Equal(2.0, aRational.Weight(2, 2));
    }

    //=============================================================================================
    // Kerf
    //=============================================================================================

    [Fact]
    public void IsoCurves_AreBSplineCurves_CheckRational()
    {
        // The iso-curves are B-spline curves on the knots of the other direction
        BSplineCurve3D aU = Assert.IsType<BSplineCurve3D>(myOriginalSurface.UIso(0.5));
        Assert.Equal(2, aU.Degree);
        Assert.Equal(3, aU.PoleCount);
        Assert.False(aU.IsRational);
        Assert.IsType<BSplineCurve3D>(myOriginalSurface.VIso(0.5, false));

        // Weights varying along U only: the V iso-curves' weights vary, the U iso-curves' are all equal
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        aWeights.Init(1.0);
        aWeights[2, 1] = aWeights[2, 2] = aWeights[2, 3] = 2.0;
        BSplineSurface aRational = new(new NCollection_Array2<Point3D>(myOriginalSurface.InternalPoles()), aWeights, Knots(0, 1), Knots(0, 1), Mults(3, 3),
                                            Mults(3, 3), 2, 2);
        Assert.True(((BSplineCurve3D)aRational.VIso(0.5)).IsRational);
        // Equal weights: CheckRational (the default) makes the curve non-rational; without it, rational
        Assert.False(((BSplineCurve3D)aRational.UIso(0.5)).IsRational);
        Assert.False(((BSplineCurve3D)aRational.UIso(0.5, true)).IsRational);
        Assert.True(((BSplineCurve3D)aRational.UIso(0.5, false)).IsRational);
        Assert.True(aRational.UIso(0.5, false).Value(0.3).IsEqual(aRational.Value(0.5, 0.3), 1e-14));
        Assert.True(aRational.VIso(0.7, false).Value(0.2).IsEqual(aRational.Value(0.2, 0.7), 1e-14));
    }

    [Fact]
    public void IsClosed_ComparesIsoCurves()
    {
        Assert.False(myOriginalSurface.IsUClosed);
        Assert.False(myOriginalSurface.IsVClosed);

        // A cylinder-like surface: first and last rows of poles coincide, so the first and last U iso-curves do
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 2);
        for (int j = 1; j <= 2; ++j)
        {
            aPoles[1, j] = new Point3D(1, 0, j);
            aPoles[2, j] = new Point3D(0, 1, j);
            aPoles[3, j] = new Point3D(1, 0, j);
        }
        BSplineSurface aClosed = new(aPoles, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(2, 2), 2, 1);
        Assert.True(aClosed.IsUClosed);
        Assert.False(aClosed.IsVClosed);

        // Rational: the weights of the iso-curves must match as well
        NCollection_Array2<double> aWeights = new(1, 3, 1, 2);
        aWeights.Init(1.0);
        aWeights[2, 1] = aWeights[2, 2] = 0.5;
        Assert.True(new BSplineSurface(aPoles, aWeights, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(2, 2), 2, 1).IsUClosed);
        aWeights[3, 1] = 2.0;
        Assert.False(new BSplineSurface(aPoles, aWeights, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(2, 2), 2, 1).IsUClosed);
    }

    [Fact]
    public void FrozenSurface_RejectsMutators_TwinsCopy()
    {
        myOriginalSurface.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => myOriginalSurface.SetPole(1, 1, new Point3D()));
        EngineAssert.Throws<ImmutableObjectException>(() => myOriginalSurface.InsertUKnot(0.5, 1, 0.0));
        EngineAssert.Throws<ImmutableObjectException>(() => myOriginalSurface.UReverse());
        EngineAssert.Throws<ImmutableObjectException>(() => myOriginalSurface.Transform(Transform3D.Translation(new Vector3D(1, 0, 0))));

        BSplineSurface aMoved = myOriginalSurface.WithPoleOneBased(2, 2, new Point3D(5, 5, 5));
        Assert.False(aMoved.IsFrozen);
        Assert.True(aMoved.Pole(2, 2).IsEqual(new Point3D(5, 5, 5), 0.0));
        Assert.True(myOriginalSurface.Pole(2, 2).IsEqual(new Point3D(2, 2, 0.4), 1e-15));

        BSplineSurface aInserted = myOriginalSurface.UKnotInserted(0.5, 1, 0.0);
        Assert.Equal(3, aInserted.UKnotCount);
        Assert.Equal(2, myOriginalSurface.UKnotCount);
        BSplineSurface? aRemoved = aInserted.UKnotRemoved(2, 0, 1e-6);
        Assert.NotNull(aRemoved);
        Assert.Equal(2, aRemoved.UKnotCount);
        Assert.Equal(3, aInserted.UKnotCount);
        Assert.Equal(4, myOriginalSurface.DegreeIncreased(4, 2).UDegree);
        Assert.Equal(0.25, myOriginalSurface.Segmented(0.25, 0.75, 0.0, 1.0).UKnot(1));

        // Resolution caches without counting as a modification
        myOriginalSurface.Resolution(1.0, out double aUTol, out _);
        Assert.True(aUTol > 0.0);
    }

    [Fact]
    public void ReturnedArrays_AreCopies()
    {
        Grid<Point3D> aPoles = myOriginalSurface.Poles;
        Assert.True(aPoles[0, 0].IsEqual(myOriginalSurface.Pole(1, 1), 0.0));
        Assert.Equal(myOriginalSurface.UPoleCount * myOriginalSurface.VPoleCount, aPoles.Values.Length);
        Assert.Equal(new[] { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 }, myOriginalSurface.UKnotSequence);
    }
    [Fact]
    public void EvalRepresentation_DroppedByEdits()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 5);
        aPoles[1, 2] = new Point3D(1, 0, 5);
        aPoles[2, 1] = new Point3D(0, 1, 5);
        aPoles[2, 2] = new Point3D(1, 1, 5);
        BezierSurface aRep = new(aPoles);
        BSplineSurface aSurf = myOriginalSurface.WithEvalRepresentation(new RepSurfaceDesc.Full { Representation = aRep });
        Assert.True(aSurf.HasEvalRepresentation);
        Assert.False(myOriginalSurface.HasEvalRepresentation);
        Assert.True(aSurf.Value(0.5, 0.5).IsEqual(aRep.Value(0.5, 0.5), 0.0));
        Assert.True(aSurf.DN(0.5, 0.5, 1, 0).IsEqual(aRep.DN(0.5, 0.5, 1, 0), 0.0, 0.0));
        Assert.True(((BSplineSurface)aSurf.Copy()).HasEvalRepresentation);

        aSurf.SetPole(1, 1, new Point3D(1, 1, 0.2));
        Assert.False(aSurf.HasEvalRepresentation);
        Assert.True(aSurf.Value(0.5, 0.5).IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-15));
    }

    // OCC30990: Foundation Classes - unexpected change in numerical results on bsplines after 0029769
    // Verify that evaluating a B-Spline surface at a knot gives consistent results regardless of
    // which span was evaluated beforehand (i.e., the cache is updated correctly).
    [Fact]
    public void OCC30990_CacheConsistencyAtKnots()
    {
        // Build a degree-3 B-Spline surface with 3 interior knots in U (4 spans) and 2 in V (3 spans).
        // Poles: 7 x 5
        const int aNbU = 7;
        const int aNbV = 5;
        NCollection_Array2<Point3D> aPoles = new(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            for (int j = 1; j <= aNbV; ++j)
            {
                aPoles[i, j] = new Point3D(i - 1, j - 1, Math.Sin(i * 0.5) * Math.Cos(j * 0.7));
            }
        }

        BSplineSurface aSurf = new(aPoles, Knots(0.0, 0.25, 0.5, 0.75, 1.0), Knots(0.0, 0.5, 1.0),
                                        Mults(4, 1, 1, 1, 4), Mults(4, 1, 4), 3, 3);

        SurfaceAdaptor aAdaptor = new(aSurf);

        // For each interior U knot, verify that evaluation at the knot is consistent
        // regardless of whether the previous evaluation was in the span before or after.
        int aNbErr = 0;
        for (int i = 2; i < aSurf.UKnotCount; ++i)
        {
            double aUknot = aSurf.UKnot(i);
            double aUprev = 0.5 * (aUknot + aSurf.UKnot(i - 1));
            double aUnext = 0.5 * (aUknot + aSurf.UKnot(i + 1));

            for (int j = 1; j < aSurf.VKnotCount; ++j)
            {
                double aV = 0.5 * (aSurf.VKnot(j) + aSurf.VKnot(j + 1));
                aAdaptor.Value(aUprev, aV); // populate cache from span before
                Point3D aP1 = aAdaptor.Value(aUknot, aV);
                aAdaptor.Value(aUnext, aV); // populate cache from span after
                Point3D aP2 = aAdaptor.Value(aUknot, aV);

                if (aP1.X != aP2.X || aP1.Y != aP2.Y || aP1.Z != aP2.Z)
                {
                    ++aNbErr;
                }
            }
        }

        // Same check for interior V knots
        for (int j = 2; j < aSurf.VKnotCount; ++j)
        {
            double aVknot = aSurf.VKnot(j);
            double aVprev = 0.5 * (aVknot + aSurf.VKnot(j - 1));
            double aVnext = 0.5 * (aVknot + aSurf.VKnot(j + 1));

            for (int i = 1; i < aSurf.UKnotCount; ++i)
            {
                double aU = 0.5 * (aSurf.UKnot(i) + aSurf.UKnot(i + 1));
                aAdaptor.Value(aU, aVprev);
                Point3D aP1 = aAdaptor.Value(aU, aVknot);
                aAdaptor.Value(aU, aVnext);
                Point3D aP2 = aAdaptor.Value(aU, aVknot);

                if (aP1.X != aP2.X || aP1.Y != aP2.Y || aP1.Z != aP2.Z)
                {
                    ++aNbErr;
                }
            }
        }

        Assert.Equal(0, aNbErr);
    }
}
