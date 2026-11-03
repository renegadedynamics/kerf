// Port of OCCT's src/ModelingData/TKG3d/GTests/Geom_BezierSurface_Test.cxx (V8_0_1), plus OCCT.NET's own checks.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Geom_BezierSurface. PolesAccess and WeightsArray_* check copies, since the port returns copies where
/// OCCT returns references (IsDeletable and reference identity don't apply).
/// </summary>
public class Geom_BezierSurfaceTests
{
    private readonly Geom_BezierSurface myOriginalSurface;

    public Geom_BezierSurfaceTests()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i, j, (i + j) * 0.1);
            }
        }
        myOriginalSurface = new Geom_BezierSurface(aPoles);
    }

    /// <summary>The 2 x 2 test poles (0,0,0), (1,0,0) / (0,1,0), (1,1,z11).</summary>
    private static NCollection_Array2<Point3D> Poles2x2(double theZ22 = 0.0)
    {
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[1, 2] = new Point3D(1, 0, 0);
        aPoles[2, 1] = new Point3D(0, 1, 0);
        aPoles[2, 2] = new Point3D(1, 1, theZ22);
        return aPoles;
    }

    private static NCollection_Array2<double> Weights2x2(double theW11, double theW12, double theW21, double theW22)
    {
        NCollection_Array2<double> aWeights = new(1, 2, 1, 2);
        aWeights[1, 1] = theW11;
        aWeights[1, 2] = theW12;
        aWeights[2, 1] = theW21;
        aWeights[2, 2] = theW22;
        return aWeights;
    }

    private static NCollection_Array1<Point3D> Points(params Point3D[] thePoints) => NCollection_Array1<Point3D>.From(thePoints);

    private static NCollection_Array1<double> Reals(params double[] theValues) => NCollection_Array1<double>.From(theValues);

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        Geom_BezierSurface aCopiedSurface = new(myOriginalSurface);
        Assert.Equal(myOriginalSurface.UDegree(), aCopiedSurface.UDegree());
        Assert.Equal(myOriginalSurface.VDegree(), aCopiedSurface.VDegree());
        Assert.Equal(myOriginalSurface.NbUPoles(), aCopiedSurface.NbUPoles());
        Assert.Equal(myOriginalSurface.NbVPoles(), aCopiedSurface.NbVPoles());
        Assert.Equal(myOriginalSurface.IsURational(), aCopiedSurface.IsURational());
        Assert.Equal(myOriginalSurface.IsVRational(), aCopiedSurface.IsVRational());
    }

    [Fact]
    public void CopyConstructorPoles()
    {
        Geom_BezierSurface aCopiedSurface = new(myOriginalSurface);
        for (int i = 1; i <= myOriginalSurface.NbUPoles(); ++i)
        {
            for (int j = 1; j <= myOriginalSurface.NbVPoles(); ++j)
            {
                Assert.True(myOriginalSurface.Pole(i, j).IsEqual(aCopiedSurface.Pole(i, j), 1e-10));
            }
        }
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        Geom_BezierSurface? aCopiedSurface = myOriginalSurface.Copy() as Geom_BezierSurface;
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
    public void RationalSurfaceCopyConstructor()
    {
        Geom_BezierSurface aRationalSurface = new(Poles2x2(), Weights2x2(1.0, 2.0, 2.0, 1.0));
        Geom_BezierSurface aCopiedRational = new(aRationalSurface);
        Assert.True(aCopiedRational.IsURational() || aCopiedRational.IsVRational());
        for (int i = 1; i <= aRationalSurface.NbUPoles(); ++i)
        {
            for (int j = 1; j <= aRationalSurface.NbVPoles(); ++j)
            {
                Assert.Equal(aRationalSurface.Weight(i, j), aCopiedRational.Weight(i, j));
            }
        }
    }

    [Fact]
    public void CopyIndependence()
    {
        Geom_BezierSurface aCopiedSurface = new(myOriginalSurface);
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
        Assert.True(aD1U.Magnitude() > 0.0);
        Assert.True(aD1V.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_D2()
    {
        myOriginalSurface.D2(0.5, 0.5, out _, out Vector3D aD1U, out _, out _, out _, out _);
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
        Assert.True(myOriginalSurface.IsCNu(100));
        Assert.True(myOriginalSurface.IsCNv(100));
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
        Point3D aNewPole = new(5, 5, 5);
        myOriginalSurface.SetPole(2, 2, aNewPole);
        Assert.True(myOriginalSurface.Pole(2, 2).IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void SetWeight()
    {
        Geom_BezierSurface aSurf = new(Poles2x2(), Weights2x2(1.0, 1.0, 1.0, 1.0));
        Point3D aValBefore = aSurf.Value(0.5, 0.5);
        aSurf.SetWeight(2, 2, 10.0);
        Assert.Equal(10.0, aSurf.Weight(2, 2));
        Assert.True(aSurf.IsURational() || aSurf.IsVRational());
        Assert.False(aValBefore.IsEqual(aSurf.Value(0.5, 0.5), 1e-6));
    }

    [Fact]
    public void Increase()
    {
        Point3D aValBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.Increase(4, 3);
        Assert.Equal(4, myOriginalSurface.UDegree());
        Assert.Equal(3, myOriginalSurface.VDegree());
        Assert.Equal(5, myOriginalSurface.NbUPoles());
        Assert.Equal(4, myOriginalSurface.NbVPoles());
        Assert.True(aValBefore.IsEqual(myOriginalSurface.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void Segment()
    {
        Point3D aPntBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.Segment(0.25, 0.75, 0.25, 0.75);
        Assert.True(myOriginalSurface.Value(0.5, 0.5).IsEqual(aPntBefore, 1e-6));
    }

    [Fact]
    public void ExchangeUV()
    {
        Point3D aValBefore = myOriginalSurface.Value(0.3, 0.7);
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
        Geom_Curve anIso = myOriginalSurface.UIso(0.5);
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
        Geom_Curve anIso = myOriginalSurface.VIso(0.5);
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
        Transform3D aTrsf = new();
        aTrsf.SetTranslation(new Vector3D(10, 20, 30));
        Point3D aPtBefore = myOriginalSurface.Value(0.5, 0.5);
        myOriginalSurface.Transform(aTrsf);
        Point3D aPtAfter = myOriginalSurface.Value(0.5, 0.5);
        Assert.Equal(aPtBefore.X() + 10.0, aPtAfter.X(), 1e-10);
        Assert.Equal(aPtBefore.Y() + 20.0, aPtAfter.Y(), 1e-10);
        Assert.Equal(aPtBefore.Z() + 30.0, aPtAfter.Z(), 1e-10);
    }

    [Fact]
    public void InsertPoleRowAfter()
    {
        int aNbUBefore = myOriginalSurface.NbUPoles();
        myOriginalSurface.InsertPoleRowAfter(1, Points(new Point3D(1.5, 1, 0), new Point3D(1.5, 2, 0), new Point3D(1.5, 3, 0)));
        Assert.Equal(aNbUBefore + 1, myOriginalSurface.NbUPoles());
        Assert.Equal(3, myOriginalSurface.UDegree());
    }

    [Fact]
    public void InsertPoleColAfter()
    {
        int aNbVBefore = myOriginalSurface.NbVPoles();
        myOriginalSurface.InsertPoleColAfter(1, Points(new Point3D(1, 1.5, 0), new Point3D(2, 1.5, 0), new Point3D(3, 1.5, 0)));
        Assert.Equal(aNbVBefore + 1, myOriginalSurface.NbVPoles());
        Assert.Equal(3, myOriginalSurface.VDegree());
    }

    [Fact]
    public void RemovePoleRow()
    {
        myOriginalSurface.InsertPoleRowAfter(1, Points(new Point3D(1.5, 1, 0), new Point3D(1.5, 2, 0), new Point3D(1.5, 3, 0)));
        Assert.Equal(4, myOriginalSurface.NbUPoles());
        myOriginalSurface.RemovePoleRow(2);
        Assert.Equal(3, myOriginalSurface.NbUPoles());
    }

    [Fact]
    public void RemovePoleCol()
    {
        myOriginalSurface.InsertPoleColAfter(1, Points(new Point3D(1, 1.5, 0), new Point3D(2, 1.5, 0), new Point3D(3, 1.5, 0)));
        Assert.Equal(4, myOriginalSurface.NbVPoles());
        myOriginalSurface.RemovePoleCol(2);
        Assert.Equal(3, myOriginalSurface.NbVPoles());
    }

    [Fact]
    public void SetPoleRow()
    {
        myOriginalSurface.SetPoleRow(2, Points(new Point3D(10, 1, 0), new Point3D(10, 2, 0), new Point3D(10, 3, 0)));
        for (int j = 1; j <= 3; ++j)
        {
            Assert.Equal(10.0, myOriginalSurface.Pole(2, j).X(), 1e-10);
        }
    }

    [Fact]
    public void SetPoleCol()
    {
        myOriginalSurface.SetPoleCol(2, Points(new Point3D(1, 10, 0), new Point3D(2, 10, 0), new Point3D(3, 10, 0)));
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(10.0, myOriginalSurface.Pole(i, 2).Y(), 1e-10);
        }
    }

    [Fact]
    public void WeightsAccess_NonRational()
    {
        Assert.Null(myOriginalSurface.Weights());
    }

    [Fact]
    public void PolesAccess()
    {
        NCollection_Array2<Point3D> aPoles = myOriginalSurface.Poles();
        Assert.Equal(3, aPoles.ColLength());
        Assert.Equal(3, aPoles.RowLength());
        Assert.True(aPoles[1, 1].IsEqual(new Point3D(1, 1, 0.2), 1e-10));
    }

    [Fact]
    public void MaxDegree()
    {
        Assert.True(Geom_BezierSurface.MaxDegree() >= 25);
    }

    [Fact]
    public void RationalSurface_UIso()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        NCollection_Array2<double> aWeights = new(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[1, 2] = new Point3D(1, 0, 0);
        aPoles[2, 1] = new Point3D(0, 1, 0);
        aPoles[2, 2] = new Point3D(1, 1, 0);
        aWeights[1, 1] = 1.0;
        aWeights[1, 2] = 2.0;
        aWeights[2, 1] = 2.0;
        aWeights[2, 2] = 1.0;

        Geom_BezierSurface aSurf = new(aPoles, aWeights);

        Geom_Curve anIso = aSurf.UIso(0.5);
        Assert.NotNull(anIso);

        for (double v = 0.0; v <= 1.0; v += 0.25)
        {
            Point3D aSurfPnt = aSurf.Value(0.5, v);
            Point3D anIsoPnt = anIso.Value(v);
            Assert.True(aSurfPnt.IsEqual(anIsoPnt, 1e-10));
        }
    }

    [Fact]
    public void Evaluation_D3()
    {
        myOriginalSurface.D3(0.5, 0.5, out _, out _, out _, out _, out _, out _, out Vector3D aD3U, out Vector3D aD3V, out _, out _);
        Assert.Equal(0.0, aD3U.Magnitude(), 1e-10);
        Assert.Equal(0.0, aD3V.Magnitude(), 1e-10);
    }

    [Fact]
    public void InsertPoleRowBefore()
    {
        int aNbUBefore = myOriginalSurface.NbUPoles();
        myOriginalSurface.InsertPoleRowBefore(2, Points(new Point3D(0.5, 1, 0), new Point3D(0.5, 2, 0), new Point3D(0.5, 3, 0)));
        Assert.Equal(aNbUBefore + 1, myOriginalSurface.NbUPoles());
        Assert.True(myOriginalSurface.Pole(2, 1).IsEqual(new Point3D(0.5, 1, 0), 1e-10));
    }

    [Fact]
    public void InsertPoleColBefore()
    {
        int aNbVBefore = myOriginalSurface.NbVPoles();
        myOriginalSurface.InsertPoleColBefore(2, Points(new Point3D(1, 0.5, 0), new Point3D(2, 0.5, 0), new Point3D(3, 0.5, 0)));
        Assert.Equal(aNbVBefore + 1, myOriginalSurface.NbVPoles());
        Assert.True(myOriginalSurface.Pole(1, 2).IsEqual(new Point3D(1, 0.5, 0), 1e-10));
    }

    [Fact]
    public void RationalSegment()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i, j, (i + j) * 0.1);
                aWeights[i, j] = 1.0 + 0.3 * ((i - 1) + (j - 1));
            }
        }
        Geom_BezierSurface aSurf = new(aPoles, aWeights);
        Assert.True(aSurf.IsURational() || aSurf.IsVRational());
        Point3D aMid = aSurf.Value(0.5, 0.5);
        aSurf.Segment(0.25, 0.75, 0.25, 0.75);
        Assert.True(aMid.IsEqual(aSurf.Value(0.5, 0.5), 1e-5));
    }

    [Fact]
    public void RationalIncrease()
    {
        Geom_BezierSurface aSurf = new(Poles2x2(1.0), Weights2x2(1.0, 2.0, 2.0, 1.0));
        Point3D aValBefore = aSurf.Value(0.5, 0.5);
        aSurf.Increase(3, 3);
        Assert.Equal(3, aSurf.UDegree());
        Assert.Equal(3, aSurf.VDegree());
        Assert.True(aValBefore.IsEqual(aSurf.Value(0.5, 0.5), 1e-10));
    }

    [Fact]
    public void SetWeightRow()
    {
        Geom_BezierSurface aSurf = new(Poles2x2(), Weights2x2(1.0, 2.0, 3.0, 4.0));
        aSurf.SetWeightRow(1, Reals(5.0, 6.0));
        Assert.Equal(5.0, aSurf.Weight(1, 1));
        Assert.Equal(6.0, aSurf.Weight(1, 2));
    }

    [Fact]
    public void SetWeightCol()
    {
        Geom_BezierSurface aSurf = new(Poles2x2(), Weights2x2(1.0, 2.0, 3.0, 4.0));
        aSurf.SetWeightCol(1, Reals(7.0, 8.0));
        Assert.Equal(7.0, aSurf.Weight(1, 1));
        Assert.Equal(8.0, aSurf.Weight(2, 1));
    }

    [Fact]
    public void InsertPoleRowAfterWithWeights()
    {
        Geom_BezierSurface aSurf = new(Poles2x2(), Weights2x2(1.0, 2.0, 2.0, 1.0));
        aSurf.InsertPoleRowAfter(1, Points(new Point3D(0, 0.5, 0), new Point3D(1, 0.5, 0)), Reals(3.0, 4.0));
        Assert.Equal(3, aSurf.NbUPoles());
        Assert.Equal(3.0, aSurf.Weight(2, 1));
        Assert.Equal(4.0, aSurf.Weight(2, 2));
    }

    [Fact]
    public void VIso_Rational()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        NCollection_Array2<double> aWeights = new(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[1, 2] = new Point3D(1, 0, 0);
        aPoles[2, 1] = new Point3D(0, 1, 0);
        aPoles[2, 2] = new Point3D(1, 1, 0);
        aWeights[1, 1] = 1.0;
        aWeights[1, 2] = 2.0;
        aWeights[2, 1] = 2.0;
        aWeights[2, 2] = 1.0;

        Geom_BezierSurface aSurf = new(aPoles, aWeights);

        Geom_Curve anIso = aSurf.VIso(0.5);
        Assert.NotNull(anIso);

        for (double u = 0.0; u <= 1.0; u += 0.25)
        {
            Point3D aSurfPnt = aSurf.Value(u, 0.5);
            Point3D anIsoPnt = anIso.Value(u);
            Assert.True(aSurfPnt.IsEqual(anIsoPnt, 1e-10));
        }
    }

    [Fact]
    public void SetPoleWithWeight()
    {
        Geom_BezierSurface aSurf = new(Poles2x2(), Weights2x2(1.0, 1.0, 1.0, 2.0));
        Point3D aNewPole = new(0.5, 0.5, 1.0);
        aSurf.SetPole(1, 1, aNewPole, 5.0);
        Assert.True(aSurf.Pole(1, 1).IsEqual(aNewPole, 1e-10));
        Assert.Equal(5.0, aSurf.Weight(1, 1));
    }

    [Fact]
    public void SetPoleColWithWeights()
    {
        Geom_BezierSurface aSurf = new(Poles2x2(), Weights2x2(1.0, 2.0, 2.0, 1.0));
        aSurf.SetPoleCol(2, Points(new Point3D(5, 0, 0), new Point3D(5, 1, 0)), Reals(3.0, 4.0));
        Assert.True(aSurf.Pole(1, 2).IsEqual(new Point3D(5, 0, 0), 1e-10));
        Assert.Equal(3.0, aSurf.Weight(1, 2));
        Assert.Equal(4.0, aSurf.Weight(2, 2));
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
        Geom_BezierSurface aRational = new(Poles2x2(), Weights2x2(1.0, 2.0, 1.0, 1.0));
        NCollection_Array2<double> aWeights = aRational.WeightsArray();
        Assert.Equal(4, aWeights.Size());
        Assert.True(aWeights.IsDeletable());
        Assert.Equal(2.0, aWeights[1, 2]);
        Assert.Equal(1.0, aWeights[1, 1]);
        aWeights[1, 2] = 5.0;
        Assert.Equal(2.0, aRational.Weight(1, 2));
    }

    //=============================================================================================
    // OCCT.NET
    //=============================================================================================

    [Fact]
    public void IsoCurves_AreBezierCurves()
    {
        Geom_BezierCurve aU = Assert.IsType<Geom_BezierCurve>(myOriginalSurface.UIso(0.5));
        Assert.Equal(myOriginalSurface.VDegree(), aU.Degree());
        Assert.False(aU.IsRational());
        Geom_BezierCurve aV = Assert.IsType<Geom_BezierCurve>(myOriginalSurface.VIso(0.5));
        Assert.Equal(myOriginalSurface.UDegree(), aV.Degree());

        // Weights varying along V only (OCCT's "U rational"): the U iso-curves are rational, the V ones are not
        NCollection_Array2<double> aWeights = new(1, myOriginalSurface.NbUPoles(), 1, myOriginalSurface.NbVPoles());
        aWeights.Init(1.0);
        for (int i = 1; i <= myOriginalSurface.NbUPoles(); ++i)
        {
            aWeights[i, 2] = 3.0;
        }
        Geom_BezierSurface aRational = new(myOriginalSurface.Poles(), aWeights);
        Assert.True(aRational.IsURational());
        Assert.False(aRational.IsVRational());
        Assert.True(((Geom_BezierCurve)aRational.UIso(0.4)).IsRational());
        Assert.False(((Geom_BezierCurve)aRational.VIso(0.4)).IsRational());
        Assert.True(aRational.UIso(0.4).Value(0.3).IsEqual(aRational.Value(0.4, 0.3), 1e-14));
        Assert.True(aRational.VIso(0.4).Value(0.3).IsEqual(aRational.Value(0.3, 0.4), 1e-14));
    }

    [Fact]
    public void KnotsOfEquivalentBSpline()
    {
        Assert.Equal(new[] { 0.0, 1.0 }, myOriginalSurface.UKnots());
        Assert.Equal(new[] { 0.0, 1.0 }, myOriginalSurface.VKnots());
        Assert.Equal(new[] { 3, 3 }, myOriginalSurface.UMultiplicities());
        Assert.Equal(new[] { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 }, myOriginalSurface.VKnotSequence());
        myOriginalSurface.Increase(4, 2);
        Assert.Equal(new[] { 5, 5 }, myOriginalSurface.UMultiplicities());
        Assert.Equal(10, myOriginalSurface.UKnotSequence().Count);
    }

    [Fact]
    public void FrozenSurface_RejectsMutators_TwinsCopy()
    {
        myOriginalSurface.Freeze();
        Assert.Throws<ImmutableObjectException>(() => myOriginalSurface.SetPole(1, 1, new Point3D()));
        Assert.Throws<ImmutableObjectException>(() => myOriginalSurface.Increase(3, 3));
        Assert.Throws<ImmutableObjectException>(() => myOriginalSurface.VReverse());

        Geom_BezierSurface aIncreased = myOriginalSurface.Increased(3, 4);
        Assert.False(aIncreased.IsFrozen);
        Assert.Equal(4, aIncreased.VDegree());
        Assert.Equal(2, myOriginalSurface.VDegree());
        Assert.Equal(2, myOriginalSurface.PoleColRemoved(2).NbVPoles());
        Assert.True(myOriginalSurface.Segmented(0.0, 0.5, 0.0, 1.0).Value(1.0, 0.0).IsEqual(myOriginalSurface.Value(0.5, 0.0), 1e-12));
        Geom_BezierSurface aWeighted = myOriginalSurface.WithWeight(2, 2, 3.0);
        Assert.True(aWeighted.IsURational());
        Assert.False(myOriginalSurface.IsURational());
        Assert.True(myOriginalSurface.IsUClosed() == false && myOriginalSurface.IsVClosed() == false);
    }
    [Fact]
    public void EvalRepresentation_KeptByPoleEdits_AsInOCCT()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 5);
        aPoles[1, 2] = new Point3D(1, 0, 5);
        aPoles[2, 1] = new Point3D(0, 1, 5);
        aPoles[2, 2] = new Point3D(1, 1, 5);
        Geom_BezierSurface aRep = new(aPoles);
        Geom_BezierSurface aSurf = myOriginalSurface.WithEvalRepresentation(new GeomEval_RepSurfaceDesc.Full { Representation = aRep });
        Assert.True(aSurf.HasEvalRepresentation());
        Assert.False(myOriginalSurface.HasEvalRepresentation());
        Assert.True(aSurf.Value(0.5, 0.5).IsEqual(aRep.Value(0.5, 0.5), 0.0));
        Assert.True(aSurf.Copy() is Geom_BezierSurface { } aCopy && aCopy.HasEvalRepresentation());

        // OCCT: a pole edit keeps the representation (occt-issues.md), Transform drops it
        aSurf.SetPole(1, 1, new Point3D(-10, -10, -10));
        Assert.True(aSurf.HasEvalRepresentation());
        Assert.True(aSurf.Value(0.0, 0.0).IsEqual(new Point3D(0, 0, 5), 0.0));
        aSurf.Transform(Transform3D.Translation(new Vector3D(0, 0, 1)));
        Assert.False(aSurf.HasEvalRepresentation());
        Assert.True(aSurf.Value(0.0, 0.0).IsEqual(new Point3D(-10, -10, -9), 1e-12));
    }
}
