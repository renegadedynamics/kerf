// Port of OCCT's src/ModelingData/TKG3d/GTests/Geom_BSplineSurface_Test.cxx (V8_0_1), plus OCCT.NET's own checks.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Geom_BSplineSurface. Not ported: UIso, VIso and RationalSurface_UIso (the iso-curves are Geom_BSplineCurve,
/// unit 1A: until then UIso/VIso throw Standard_NotImplemented, see IsoCurves_NotImplementedUntil1A) and
/// OCC30990_CacheConsistencyAtKnots (GeomAdaptor_Surface, unit 2A). WeightsArray_* check copies, since the port
/// returns copies where OCCT returns references (IsDeletable and reference identity don't apply).
/// </summary>
public class Geom_BSplineSurfaceTests
{
    private readonly Geom_BSplineSurface myOriginalSurface;

    public Geom_BSplineSurfaceTests()
    {
        // Create a simple BSpline surface for testing
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt(i, j, (i + j) * 0.1);
            }
        }
        myOriginalSurface = new Geom_BSplineSurface(aPoles, Knots(0.0, 1.0), Knots(0.0, 1.0), Mults(3, 3), Mults(3, 3), 2, 2);
    }

    private static NCollection_Array1<double> Knots(params double[] theValues) => NCollection_Array1<double>.From(theValues);

    private static NCollection_Array1<int> Mults(params int[] theValues) => NCollection_Array1<int>.From(theValues);

    /// <summary>The 3 x 3 rational test surface of SetWeight, SetWeightRow and SetWeightCol.</summary>
    private static Geom_BSplineSurface RationalBump()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt(i, j, (i == 2 && j == 2) ? 1.0 : 0.0);
                aWeights[i, j] = 1.0;
            }
        }
        aWeights[2, 2] = 2.0;
        return new Geom_BSplineSurface(aPoles, aWeights, Knots(0.0, 1.0), Knots(0.0, 1.0), Mults(3, 3), Mults(3, 3), 2, 2);
    }

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        Geom_BSplineSurface aCopiedSurface = new(myOriginalSurface);
        Assert.Equal(myOriginalSurface.UDegree(), aCopiedSurface.UDegree());
        Assert.Equal(myOriginalSurface.VDegree(), aCopiedSurface.VDegree());
        Assert.Equal(myOriginalSurface.NbUPoles(), aCopiedSurface.NbUPoles());
        Assert.Equal(myOriginalSurface.NbVPoles(), aCopiedSurface.NbVPoles());
        Assert.Equal(myOriginalSurface.NbUKnots(), aCopiedSurface.NbUKnots());
        Assert.Equal(myOriginalSurface.NbVKnots(), aCopiedSurface.NbVKnots());
        Assert.Equal(myOriginalSurface.IsUPeriodic(), aCopiedSurface.IsUPeriodic());
        Assert.Equal(myOriginalSurface.IsVPeriodic(), aCopiedSurface.IsVPeriodic());
    }

    [Fact]
    public void CopyConstructorPoles()
    {
        Geom_BSplineSurface aCopiedSurface = new(myOriginalSurface);
        for (int i = 1; i <= myOriginalSurface.NbUPoles(); ++i)
        {
            for (int j = 1; j <= myOriginalSurface.NbVPoles(); ++j)
            {
                Assert.True(myOriginalSurface.Pole(i, j).IsEqual(aCopiedSurface.Pole(i, j), 1e-10));
            }
        }
    }

    [Fact]
    public void CopyConstructorKnots()
    {
        Geom_BSplineSurface aCopiedSurface = new(myOriginalSurface);
        for (int i = 1; i <= myOriginalSurface.NbUKnots(); ++i)
        {
            Assert.Equal(myOriginalSurface.UKnot(i), aCopiedSurface.UKnot(i));
            Assert.Equal(myOriginalSurface.UMultiplicity(i), aCopiedSurface.UMultiplicity(i));
        }
        for (int i = 1; i <= myOriginalSurface.NbVKnots(); ++i)
        {
            Assert.Equal(myOriginalSurface.VKnot(i), aCopiedSurface.VKnot(i));
            Assert.Equal(myOriginalSurface.VMultiplicity(i), aCopiedSurface.VMultiplicity(i));
        }
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        Geom_Geometry aCopiedGeom = myOriginalSurface.Copy();
        Geom_BSplineSurface? aCopiedSurface = aCopiedGeom as Geom_BSplineSurface;
        Assert.NotNull(aCopiedSurface);
        Assert.Equal(myOriginalSurface.UDegree(), aCopiedSurface.UDegree());
        Assert.Equal(myOriginalSurface.VDegree(), aCopiedSurface.VDegree());
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
        Geom_BSplineSurface aCopiedSurface = new(myOriginalSurface);
        gp_Pnt aNewPole = new(10, 10, 10);
        myOriginalSurface.SetPole(2, 2, aNewPole);
        Assert.False(aCopiedSurface.Pole(2, 2).IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void Evaluation_D0()
    {
        myOriginalSurface.D0(0.0, 0.0, out gp_Pnt aPnt);
        Assert.True(aPnt.IsEqual(new gp_Pnt(1, 1, 0.2), 1e-10));
        myOriginalSurface.D0(1.0, 1.0, out aPnt);
        Assert.True(aPnt.IsEqual(new gp_Pnt(3, 3, 0.6), 1e-10));
    }

    [Fact]
    public void Evaluation_D1()
    {
        myOriginalSurface.D1(0.5, 0.5, out _, out gp_Vec aD1U, out gp_Vec aD1V);
        Assert.True(aD1U.Magnitude() > 0.0);
        Assert.True(aD1V.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_D2()
    {
        myOriginalSurface.D2(0.5, 0.5, out _, out gp_Vec aD1U, out _, out _, out _, out _);
        Assert.True(aD1U.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_DN()
    {
        Assert.True(myOriginalSurface.DN(0.5, 0.5, 1, 0).Magnitude() > 0.0);
        Assert.True(myOriginalSurface.DN(0.5, 0.5, 0, 1).Magnitude() > 0.0);
    }

    [Fact]
    public void Properties()
    {
        Assert.Equal(2, myOriginalSurface.UDegree());
        Assert.Equal(2, myOriginalSurface.VDegree());
        Assert.False(myOriginalSurface.IsUPeriodic());
        Assert.False(myOriginalSurface.IsVPeriodic());
        Assert.False(myOriginalSurface.IsURational());
        Assert.False(myOriginalSurface.IsVRational());
        Assert.Equal(3, myOriginalSurface.NbUPoles());
        Assert.Equal(3, myOriginalSurface.NbVPoles());
        Assert.Equal(2, myOriginalSurface.NbUKnots());
        Assert.Equal(2, myOriginalSurface.NbVKnots());
    }

    [Fact]
    public void Bounds()
    {
        myOriginalSurface.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(0.0, aU1);
        Assert.Equal(1.0, aU2);
        Assert.Equal(0.0, aV1);
        Assert.Equal(1.0, aV2);
    }

    [Fact]
    public void SetPole()
    {
        gp_Pnt aNewPole = new(5, 5, 5);
        myOriginalSurface.SetPole(2, 2, aNewPole);
        Assert.True(myOriginalSurface.Pole(2, 2).IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void SetWeight()
    {
        Geom_BSplineSurface aSurf = RationalBump();
        Assert.True(aSurf.IsURational() || aSurf.IsVRational());
        gp_Pnt aValBefore = aSurf.Value(0.5, 0.5);
        aSurf.SetWeight(2, 2, 10.0);
        Assert.Equal(10.0, aSurf.Weight(2, 2));
        gp_Pnt aValAfter = aSurf.Value(0.5, 0.5);
        // Higher weight at the elevated center pole pulls surface up
        Assert.True(aValAfter.Z() > aValBefore.Z());
    }

    [Fact]
    public void IncreaseDegree()
    {
        gp_Pnt aValBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.IncreaseDegree(4, 3);
        Assert.Equal(4, myOriginalSurface.UDegree());
        Assert.Equal(3, myOriginalSurface.VDegree());
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void InsertUKnot()
    {
        gp_Pnt aValBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.NbUKnots());
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void InsertVKnot()
    {
        gp_Pnt aValBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.InsertVKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.NbVKnots());
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void RemoveUKnot()
    {
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.NbUKnots());
        gp_Pnt aValBefore = myOriginalSurface.Value(0.25, 0.5);
        Assert.True(myOriginalSurface.RemoveUKnot(2, 0, 1e-6));
        Assert.Equal(2, myOriginalSurface.NbUKnots());
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.25, 0.5), 1e-6));
    }

    [Fact]
    public void Segment()
    {
        gp_Pnt aPntBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.Segment(0.25, 0.75, 0.25, 0.75);
        myOriginalSurface.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(0.25, aU1, 1e-10);
        Assert.Equal(0.75, aU2, 1e-10);
        Assert.Equal(0.25, aV1, 1e-10);
        Assert.Equal(0.75, aV2, 1e-10);
        Assert.True(aPntBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-6));
    }

    [Fact]
    public void ExchangeUV()
    {
        gp_Pnt aValBefore = myOriginalSurface.Value(0.3, 0.7);
        int aUDeg = myOriginalSurface.UDegree();
        int aVDeg = myOriginalSurface.VDegree();
        myOriginalSurface.ExchangeUV();
        Assert.Equal(aVDeg, myOriginalSurface.UDegree());
        Assert.Equal(aUDeg, myOriginalSurface.VDegree());
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.7, 0.3), 1e-10));
    }

    [Fact]
    public void UReverse()
    {
        gp_Pnt aValBefore = myOriginalSurface.Value(0.3, 0.5);
        myOriginalSurface.UReverse();
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.7, 0.5), 1e-10));
    }

    [Fact]
    public void VReverse()
    {
        gp_Pnt aValBefore = myOriginalSurface.Value(0.5, 0.3);
        myOriginalSurface.VReverse();
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.7), 1e-10));
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
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(10, 20, 30));
        gp_Pnt aPtBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.Transform(aTrsf);
        gp_Pnt aPtAfter = myOriginalSurface.Value(0.5, 0.5);
        Assert.Equal(aPtBefore.X() + 10.0, aPtAfter.X(), 1e-10);
        Assert.Equal(aPtBefore.Y() + 20.0, aPtAfter.Y(), 1e-10);
        Assert.Equal(aPtBefore.Z() + 30.0, aPtAfter.Z(), 1e-10);
    }

    [Fact]
    public void KnotsAccess()
    {
        PackedList<double> aUKnots = myOriginalSurface.UKnots();
        Assert.Equal(2, aUKnots.Count);
        Assert.Equal(0.0, aUKnots[0]);
        Assert.Equal(1.0, aUKnots[1]);
        Assert.Equal(2, myOriginalSurface.VKnots().Count);
        PackedList<int> aUMults = myOriginalSurface.UMultiplicities();
        Assert.Equal(2, aUMults.Count);
        Assert.Equal(3, aUMults[0]);
        Assert.Equal(2, myOriginalSurface.VMultiplicities().Count);
    }

    [Fact]
    public void WeightsAccess_NonRational()
    {
        Assert.Null(myOriginalSurface.Weights());
    }

    [Fact]
    public void PeriodicSurface_SetUNotPeriodic()
    {
        // U-periodic surface: degree 2, 4 U-knots of mult 1 -> NbUPoles = 1+2 = 3
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt(Math.Cos(2.0 * Math.PI * (i - 1) / 3.0), Math.Sin(2.0 * Math.PI * (i - 1) / 3.0), j * 0.5);
            }
        }
        Geom_BSplineSurface aSurf = new(aPoles, Knots(0, 1, 2, 3), Knots(0, 1), Mults(1, 1, 1, 1), Mults(3, 3), 2, 2, true, false);
        Assert.True(aSurf.IsUPeriodic());
        gp_Pnt aVal = aSurf.Value(0.5, 0.5);
        aSurf.SetUNotPeriodic();
        Assert.False(aSurf.IsUPeriodic());
        Assert.True(aVal.IsEqual(aSurf.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void PeriodicSurface_SetVNotPeriodic()
    {
        // V-periodic surface: degree 2, 4 V-knots of mult 1 -> NbVPoles = 1+2 = 3
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt(i * 0.5, Math.Cos(2.0 * Math.PI * (j - 1) / 3.0), Math.Sin(2.0 * Math.PI * (j - 1) / 3.0));
            }
        }
        Geom_BSplineSurface aSurf = new(aPoles, Knots(0, 1), Knots(0, 1, 2, 3), Mults(3, 3), Mults(1, 1, 1, 1), 2, 2, false, true);
        Assert.True(aSurf.IsVPeriodic());
        gp_Pnt aVal = aSurf.Value(0.5, 0.5);
        aSurf.SetVNotPeriodic();
        Assert.False(aSurf.IsVPeriodic());
        Assert.True(aVal.IsEqual(aSurf.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void RationalSurface()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt(i, j, 0);
                aWeights[i, j] = 1.0;
            }
        }
        aWeights[2, 2] = 3.0;
        Geom_BSplineSurface aSurf = new(aPoles, aWeights, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(3, 3), 2, 2);
        Assert.True(aSurf.IsURational());
        Assert.True(aSurf.IsVRational());
        NCollection_Array2<double>? aW = aSurf.Weights();
        Assert.NotNull(aW);
        Assert.Equal(3.0, aW[2, 2]);
        gp_Pnt aMid = aSurf.Value(0.5, 0.5);
        // High weight at center should pull surface toward (2,2,0)
        Assert.Equal(2.0, aMid.X(), 0.1);
        Assert.Equal(2.0, aMid.Y(), 0.1);
    }

    [Fact]
    public void LocalEvaluation()
    {
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.NbUKnots());
        gp_Pnt aGlobal = myOriginalSurface.Value(0.25, 0.5);
        myOriginalSurface.LocalD0(0.25, 0.5, 1, 2, 1, 2, out gp_Pnt aLocal);
        Assert.True(aGlobal.IsEqual(aLocal, 1e-10));
    }

    [Fact]
    public void SetPoleRow()
    {
        NCollection_Array1<gp_Pnt> aNewRow = NCollection_Array1<gp_Pnt>.From([new gp_Pnt(10, 1, 0), new gp_Pnt(10, 2, 0), new gp_Pnt(10, 3, 0)]);
        myOriginalSurface.SetPoleRow(2, aNewRow);
        for (int j = 1; j <= 3; ++j)
        {
            Assert.Equal(10.0, myOriginalSurface.Pole(2, j).X(), 1e-10);
        }
    }

    [Fact]
    public void SetPoleCol()
    {
        NCollection_Array1<gp_Pnt> aNewCol = NCollection_Array1<gp_Pnt>.From([new gp_Pnt(1, 10, 0), new gp_Pnt(2, 10, 0), new gp_Pnt(3, 10, 0)]);
        myOriginalSurface.SetPoleCol(2, aNewCol);
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(10.0, myOriginalSurface.Pole(i, 2).Y(), 1e-10);
        }
    }

    [Fact]
    public void SetWeightRow()
    {
        Geom_BSplineSurface aSurf = RationalBump();
        aSurf.SetWeightRow(2, Knots(3.0, 4.0, 5.0));
        Assert.Equal(3.0, aSurf.Weight(2, 1));
        Assert.Equal(4.0, aSurf.Weight(2, 2));
        Assert.Equal(5.0, aSurf.Weight(2, 3));
    }

    [Fact]
    public void SetWeightCol()
    {
        Geom_BSplineSurface aSurf = RationalBump();
        aSurf.SetWeightCol(2, Knots(6.0, 7.0, 8.0));
        Assert.Equal(6.0, aSurf.Weight(1, 2));
        Assert.Equal(7.0, aSurf.Weight(2, 2));
        Assert.Equal(8.0, aSurf.Weight(3, 2));
    }

    [Fact]
    public void RemoveVKnot()
    {
        myOriginalSurface.InsertVKnot(0.5, 1, 0.0);
        Assert.Equal(3, myOriginalSurface.NbVKnots());
        gp_Pnt aValBefore = myOriginalSurface.Value(0.5, 0.25);
        Assert.True(myOriginalSurface.RemoveVKnot(2, 0, 1e-6));
        Assert.Equal(2, myOriginalSurface.NbVKnots());
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.25), 1e-6));
    }

    [Fact]
    public void MovePoint()
    {
        gp_Pnt aTarget = new(2, 2, 1);
        myOriginalSurface.MovePoint(0.5, 0.5, aTarget, 1, myOriginalSurface.NbUPoles(), 1, myOriginalSurface.NbVPoles(),
                                    out _, out _, out _, out _);
        Assert.True(myOriginalSurface.Value(0.5, 0.5).IsEqual(aTarget, 1e-6));
    }

    [Fact]
    public void LocateU()
    {
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        myOriginalSurface.LocateU(0.25, 1e-10, out int anI1, out int anI2);
        Assert.True(anI1 >= 1);
        Assert.True(anI2 <= myOriginalSurface.NbUKnots());
    }

    [Fact]
    public void LocateV()
    {
        myOriginalSurface.InsertVKnot(0.5, 1, 0.0);
        myOriginalSurface.LocateV(0.25, 1e-10, out int anI1, out int anI2);
        Assert.True(anI1 >= 1);
        Assert.True(anI2 <= myOriginalSurface.NbVKnots());
    }

    [Fact]
    public void RationalSurface_SetUNotPeriodic()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt(Math.Cos(2.0 * Math.PI * (i - 1) / 3.0), Math.Sin(2.0 * Math.PI * (i - 1) / 3.0), j * 0.5);
                aWeights[i, j] = 1.0 + 0.5 * (i - 1);
            }
        }
        Geom_BSplineSurface aSurf = new(aPoles, aWeights, Knots(0, 1, 2, 3), Knots(0, 1), Mults(1, 1, 1, 1), Mults(3, 3), 2, 2, true, false);
        Assert.True(aSurf.IsUPeriodic());
        Assert.True(aSurf.IsURational() || aSurf.IsVRational());
        gp_Pnt aVal = aSurf.Value(0.5, 0.5);
        aSurf.SetUNotPeriodic();
        Assert.False(aSurf.IsUPeriodic());
        Assert.True(aVal.IsEqual(aSurf.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void RationalSurface_SetVNotPeriodic()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt(i * 0.5, Math.Cos(2.0 * Math.PI * (j - 1) / 3.0), Math.Sin(2.0 * Math.PI * (j - 1) / 3.0));
                aWeights[i, j] = 1.0 + 0.5 * (j - 1);
            }
        }
        Geom_BSplineSurface aSurf = new(aPoles, aWeights, Knots(0, 1), Knots(0, 1, 2, 3), Mults(3, 3), Mults(1, 1, 1, 1), 2, 2, false, true);
        Assert.True(aSurf.IsVPeriodic());
        Assert.True(aSurf.IsURational() || aSurf.IsVRational());
        gp_Pnt aVal = aSurf.Value(0.5, 0.5);
        aSurf.SetVNotPeriodic();
        Assert.False(aSurf.IsVPeriodic());
        Assert.True(aVal.IsEqual(aSurf.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void Evaluation_D3()
    {
        myOriginalSurface.D3(0.5, 0.5, out _, out _, out _, out _, out _, out _, out gp_Vec aD3U, out gp_Vec aD3V, out _, out _);
        // Degree 2 surface: D3 should be zero
        Assert.Equal(0.0, aD3U.Magnitude(), 1e-10);
        Assert.Equal(0.0, aD3V.Magnitude(), 1e-10);
    }

    [Fact]
    public void IncreaseUMultiplicity()
    {
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(1, myOriginalSurface.UMultiplicity(2));
        gp_Pnt aValBefore = myOriginalSurface.Value(0.5, 0.5);
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
    public void CopyIndependence_Knots()
    {
        Geom_BSplineSurface aCopy = new(myOriginalSurface);
        myOriginalSurface.InsertUKnot(0.5, 1, 0.0);
        Assert.Equal(2, aCopy.NbUKnots());
        Assert.Equal(3, myOriginalSurface.NbUKnots());
    }

    [Fact]
    public void WeightsArray_NonRational_ReturnsUnitWeights()
    {
        Assert.False(myOriginalSurface.IsURational());
        Assert.False(myOriginalSurface.IsVRational());
        NCollection_Array2<double> aWeights = myOriginalSurface.WeightsArray();
        Assert.Equal(myOriginalSurface.NbUPoles(), aWeights.ColLength());
        Assert.Equal(myOriginalSurface.NbVPoles(), aWeights.RowLength());
        for (int i = aWeights.LowerRow(); i <= aWeights.UpperRow(); ++i)
        {
            for (int j = aWeights.LowerCol(); j <= aWeights.UpperCol(); ++j)
            {
                Assert.Equal(1.0, aWeights[i, j]);
            }
        }
        // The port returns a copy (OCCT: the same shared unit-weights view each time)
        Assert.NotSame(aWeights, myOriginalSurface.WeightsArray());
    }

    [Fact]
    public void WeightsArray_Rational_ReturnsOwning()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeightsIn = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt(i, j, 0);
                aWeightsIn[i, j] = (i == 2 && j == 2) ? 2.0 : 1.0;
            }
        }
        Geom_BSplineSurface aRational = new(aPoles, aWeightsIn, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(3, 3), 2, 2);
        NCollection_Array2<double> aWeights = aRational.WeightsArray();
        Assert.Equal(3, aWeights.ColLength());
        Assert.Equal(3, aWeights.RowLength());
        Assert.True(aWeights.IsDeletable());
        Assert.Equal(2.0, aWeights[2, 2]);
        Assert.Equal(1.0, aWeights[1, 1]);
        // A copy: changing it doesn't change the surface
        aWeights[2, 2] = 5.0;
        Assert.Equal(2.0, aRational.Weight(2, 2));
    }

    //=============================================================================================
    // OCCT.NET
    //=============================================================================================

    [Fact]
    public void IsoCurves_NotImplementedUntil1A()
    {
        // TODO(1A): replace by OCCT's UIso, VIso and RationalSurface_UIso once Geom_BSplineCurve is ported.
        Assert.Throws<Standard_NotImplemented>(() => myOriginalSurface.UIso(0.5));
        Assert.Throws<Standard_NotImplemented>(() => myOriginalSurface.VIso(0.5));
        Assert.Throws<Standard_NotImplemented>(() => myOriginalSurface.UIso(0.5, true));
        Assert.Throws<Standard_NotImplemented>(() => myOriginalSurface.VIso(0.5, false));
    }

    [Fact]
    public void IsClosed_ComparesIsoCurves()
    {
        Assert.False(myOriginalSurface.IsUClosed());
        Assert.False(myOriginalSurface.IsVClosed());

        // A cylinder-like surface: first and last rows of poles coincide, so the first and last U iso-curves do
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 2);
        for (int j = 1; j <= 2; ++j)
        {
            aPoles[1, j] = new gp_Pnt(1, 0, j);
            aPoles[2, j] = new gp_Pnt(0, 1, j);
            aPoles[3, j] = new gp_Pnt(1, 0, j);
        }
        Geom_BSplineSurface aClosed = new(aPoles, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(2, 2), 2, 1);
        Assert.True(aClosed.IsUClosed());
        Assert.False(aClosed.IsVClosed());

        // Rational: the weights of the iso-curves must match as well
        NCollection_Array2<double> aWeights = new(1, 3, 1, 2);
        aWeights.Init(1.0);
        aWeights[2, 1] = aWeights[2, 2] = 0.5;
        Assert.True(new Geom_BSplineSurface(aPoles, aWeights, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(2, 2), 2, 1).IsUClosed());
        aWeights[3, 1] = 2.0;
        Assert.False(new Geom_BSplineSurface(aPoles, aWeights, Knots(0, 1), Knots(0, 1), Mults(3, 3), Mults(2, 2), 2, 1).IsUClosed());
    }

    [Fact]
    public void FrozenSurface_RejectsMutators_TwinsCopy()
    {
        myOriginalSurface.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => myOriginalSurface.SetPole(1, 1, new gp_Pnt()));
        Assert.Throws<Standard_ImmutableObject>(() => myOriginalSurface.InsertUKnot(0.5, 1, 0.0));
        Assert.Throws<Standard_ImmutableObject>(() => myOriginalSurface.UReverse());
        Assert.Throws<Standard_ImmutableObject>(() => myOriginalSurface.Transform(gp_Trsf.Translation(new gp_Vec(1, 0, 0))));

        Geom_BSplineSurface aMoved = myOriginalSurface.WithPole(2, 2, new gp_Pnt(5, 5, 5));
        Assert.False(aMoved.IsFrozen());
        Assert.True(aMoved.Pole(2, 2).IsEqual(new gp_Pnt(5, 5, 5), 0.0));
        Assert.True(myOriginalSurface.Pole(2, 2).IsEqual(new gp_Pnt(2, 2, 0.4), 1e-15));

        Geom_BSplineSurface aInserted = myOriginalSurface.UKnotInserted(0.5, 1, 0.0);
        Assert.Equal(3, aInserted.NbUKnots());
        Assert.Equal(2, myOriginalSurface.NbUKnots());
        Geom_BSplineSurface? aRemoved = aInserted.UKnotRemoved(2, 0, 1e-6);
        Assert.NotNull(aRemoved);
        Assert.Equal(2, aRemoved.NbUKnots());
        Assert.Equal(3, aInserted.NbUKnots());
        Assert.Equal(4, myOriginalSurface.DegreeIncreased(4, 2).UDegree());
        Assert.Equal(0.25, myOriginalSurface.Segmented(0.25, 0.75, 0.0, 1.0).UKnot(1));

        // Resolution caches without counting as a modification
        myOriginalSurface.Resolution(1.0, out double aUTol, out _);
        Assert.True(aUTol > 0.0);
    }

    [Fact]
    public void ReturnedArrays_AreCopies()
    {
        NCollection_Array2<gp_Pnt> aPoles = myOriginalSurface.Poles();
        aPoles[1, 1] = new gp_Pnt(100, 100, 100);
        Assert.False(myOriginalSurface.Pole(1, 1).IsEqual(new gp_Pnt(100, 100, 100), 1.0));
        Assert.Equal(new[] { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 }, myOriginalSurface.UKnotSequence());
    }
    [Fact]
    public void EvalRepresentation_DroppedByEdits()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 2, 1, 2);
        aPoles[1, 1] = new gp_Pnt(0, 0, 5);
        aPoles[1, 2] = new gp_Pnt(1, 0, 5);
        aPoles[2, 1] = new gp_Pnt(0, 1, 5);
        aPoles[2, 2] = new gp_Pnt(1, 1, 5);
        Geom_BezierSurface aRep = new(aPoles);
        Geom_BSplineSurface aSurf = myOriginalSurface.WithEvalRepresentation(new GeomEval_RepSurfaceDesc.Full { Representation = aRep });
        Assert.True(aSurf.HasEvalRepresentation());
        Assert.False(myOriginalSurface.HasEvalRepresentation());
        Assert.True(aSurf.Value(0.5, 0.5).IsEqual(aRep.Value(0.5, 0.5), 0.0));
        Assert.True(aSurf.DN(0.5, 0.5, 1, 0).IsEqual(aRep.DN(0.5, 0.5, 1, 0), 0.0, 0.0));
        Assert.True(((Geom_BSplineSurface)aSurf.Copy()).HasEvalRepresentation());

        aSurf.SetPole(1, 1, new gp_Pnt(1, 1, 0.2));
        Assert.False(aSurf.HasEvalRepresentation());
        Assert.True(aSurf.Value(0.5, 0.5).IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-15));
    }
}
