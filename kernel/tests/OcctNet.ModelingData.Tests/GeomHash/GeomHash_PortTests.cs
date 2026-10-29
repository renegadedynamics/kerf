// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// The port's own tests of GeomHash and Geom2dHash: copies of random geometry of every kind hash and compare equal,
/// moved geometry compares different, and the hashers deduplicate in .NET collections.
/// </summary>
public class GeomHash_PortTests
{
    private static double R(Random theRand, double theMin, double theMax) => theMin + (theMax - theMin) * theRand.NextDouble();

    private static gp_Pnt Pnt(Random r) => new(R(r, -10, 10), R(r, -10, 10), R(r, -10, 10));

    private static gp_Pnt2d Pnt2d(Random r) => new(R(r, -10, 10), R(r, -10, 10));

    private static gp_Ax2 Ax2(Random r)
    {
        var aN = new gp_Vec(R(r, -1, 1), R(r, -1, 1), R(r, -1, 1) + 2.0);
        return new gp_Ax2(Pnt(r), new gp_Dir(aN));
    }

    private static gp_Ax22d Ax22d(Random r) => new(Pnt2d(r), new gp_Dir2d(R(r, -1, 1) + 2.0, R(r, -1, 1)), r.Next(2) == 0);

    private static Geom_BSplineCurve BSpline(Random r, bool theRational)
    {
        int aDeg = r.Next(1, 4);
        int aNbKnots = r.Next(2, 5);
        var aKnots = new NCollection_Array1<double>(1, aNbKnots);
        var aMults = new NCollection_Array1<int>(1, aNbKnots);
        int aNbPoles = 0;
        for (int i = 1; i <= aNbKnots; i++)
        {
            aKnots[i] = i - 1 + R(r, 0, 0.5);
            aMults[i] = i == 1 || i == aNbKnots ? aDeg + 1 : 1;
            aNbPoles += aMults[i];
        }

        aNbPoles -= aDeg + 1;
        var aPoles = new NCollection_Array1<gp_Pnt>(1, aNbPoles);
        var aWeights = new NCollection_Array1<double>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = Pnt(r);
            aWeights[i] = R(r, 0.5, 2);
        }

        return theRational ? new Geom_BSplineCurve(aPoles, aWeights, aKnots, aMults, aDeg)
                           : new Geom_BSplineCurve(aPoles, aKnots, aMults, aDeg);
    }

    internal static Geom_Curve Curve(Random r, int theKind)
    {
        switch (theKind)
        {
            case 0: return new Geom_Line(Pnt(r), new gp_Dir(R(r, 1, 2), R(r, -1, 1), R(r, -1, 1)));
            case 1: return new Geom_Circle(Ax2(r), R(r, 1, 5));
            case 2: return new Geom_Ellipse(Ax2(r), R(r, 5, 10), R(r, 1, 4));
            case 3: return new Geom_Hyperbola(Ax2(r), R(r, 1, 5), R(r, 1, 5));
            case 4: return new Geom_Parabola(Ax2(r), R(r, 1, 5));
            case 5:
            case 6:
            {
                int aNb = r.Next(2, 7);
                var aPoles = new NCollection_Array1<gp_Pnt>(1, aNb);
                var aWeights = new NCollection_Array1<double>(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    aPoles[i] = Pnt(r);
                    aWeights[i] = R(r, 0.5, 2);
                }

                return theKind == 5 ? new Geom_BezierCurve(aPoles) : new Geom_BezierCurve(aPoles, aWeights);
            }
            case 7: return BSpline(r, false);
            case 8: return BSpline(r, true);
            case 9: return new Geom_TrimmedCurve(new Geom_Circle(Ax2(r), R(r, 1, 5)), R(r, 0, 1), R(r, 2, 3));
            default:
            {
                var aCircle = new Geom_Circle(Ax2(r), R(r, 1, 5));
                return new Geom_OffsetCurve(aCircle, R(r, 0.1, 0.5), aCircle.Position().Direction());
            }
        }
    }

    internal static Geom_Surface Surface(Random r, int theKind)
    {
        switch (theKind)
        {
            case 0: return new Geom_Plane(new gp_Ax3(Ax2(r)));
            case 1: return new Geom_CylindricalSurface(new gp_Ax3(Ax2(r)), R(r, 1, 5));
            case 2: return new Geom_ConicalSurface(new gp_Ax3(Ax2(r)), R(r, 0.1, 1), R(r, 0, 3));
            case 3: return new Geom_SphericalSurface(new gp_Ax3(Ax2(r)), R(r, 1, 5));
            case 4: return new Geom_ToroidalSurface(new gp_Ax3(Ax2(r)), R(r, 6, 9), R(r, 1, 5));
            case 5: return new Geom_SurfaceOfRevolution(BSpline(r, false), new gp_Ax1(Pnt(r), new gp_Dir(0, 0, 1)));
            case 6: return new Geom_SurfaceOfLinearExtrusion(BSpline(r, true), new gp_Dir(R(r, -1, 1), R(r, -1, 1), 2.0));
            case 7:
            case 8:
            {
                int aNbU = r.Next(2, 5), aNbV = r.Next(2, 5);
                var aPoles = new NCollection_Array2<gp_Pnt>(1, aNbU, 1, aNbV);
                var aWeights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
                for (int i = 1; i <= aNbU; i++)
                {
                    for (int j = 1; j <= aNbV; j++)
                    {
                        aPoles[i, j] = new gp_Pnt(i + R(r, 0, 0.5), j + R(r, 0, 0.5), R(r, -1, 1));
                        aWeights[i, j] = R(r, 0.5, 2);
                    }
                }

                return theKind == 7 ? new Geom_BezierSurface(aPoles) : new Geom_BezierSurface(aPoles, aWeights);
            }
            case 9:
            {
                var aPoles = new NCollection_Array2<gp_Pnt>(1, 3, 1, 4);
                for (int i = 1; i <= 3; i++)
                {
                    for (int j = 1; j <= 4; j++)
                    {
                        aPoles[i, j] = new gp_Pnt(i, j, R(r, -1, 1));
                    }
                }

                var aUKnots = new NCollection_Array1<double>(1, 2);
                aUKnots[1] = 0.0;
                aUKnots[2] = R(r, 1, 2);
                var aUMults = new NCollection_Array1<int>(1, 2);
                aUMults[1] = 3;
                aUMults[2] = 3;
                var aVKnots = new NCollection_Array1<double>(1, 3);
                aVKnots[1] = 0.0;
                aVKnots[2] = R(r, 0.5, 1);
                aVKnots[3] = 2.0;
                var aVMults = new NCollection_Array1<int>(1, 3);
                aVMults[1] = 3;
                aVMults[2] = 1;
                aVMults[3] = 3;
                return new Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 2, 2);
            }
            case 10: return new Geom_RectangularTrimmedSurface(Surface(r, 3), R(r, 0, 1), R(r, 2, 3), R(r, -1, 0), R(r, 0, 1));
            default: return new Geom_OffsetSurface(Surface(r, 9), R(r, 0.1, 0.5));
        }
    }

    internal static Geom2d_Curve Curve2d(Random r, int theKind)
    {
        switch (theKind)
        {
            case 0: return new Geom2d_Line(Pnt2d(r), new gp_Dir2d(R(r, 1, 2), R(r, -1, 1)));
            case 1: return new Geom2d_Circle(Ax22d(r), R(r, 1, 5));
            case 2: return new Geom2d_Ellipse(Ax22d(r), R(r, 5, 10), R(r, 1, 4));
            case 3: return new Geom2d_Hyperbola(Ax22d(r), R(r, 1, 5), R(r, 1, 5));
            case 4: return new Geom2d_Parabola(Ax22d(r), R(r, 1, 5));
            case 5:
            case 6:
            {
                int aNb = r.Next(2, 7);
                var aPoles = new NCollection_Array1<gp_Pnt2d>(1, aNb);
                var aWeights = new NCollection_Array1<double>(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    aPoles[i] = Pnt2d(r);
                    aWeights[i] = R(r, 0.5, 2);
                }

                return theKind == 5 ? new Geom2d_BezierCurve(aPoles) : new Geom2d_BezierCurve(aPoles, aWeights);
            }
            case 7:
            {
                var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
                for (int i = 1; i <= 4; i++)
                {
                    aPoles[i] = Pnt2d(r);
                }

                var aKnots = new NCollection_Array1<double>(1, 2);
                aKnots[1] = 0.0;
                aKnots[2] = R(r, 1, 2);
                var aMults = new NCollection_Array1<int>(1, 2);
                aMults[1] = 4;
                aMults[2] = 4;
                return new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3);
            }
            case 8: return new Geom2d_TrimmedCurve(new Geom2d_Circle(Ax22d(r), R(r, 1, 5)), R(r, 0, 1), R(r, 2, 3));
            default: return new Geom2d_OffsetCurve(new Geom2d_Circle(Ax22d(r), R(r, 1, 5)), R(r, 0.1, 0.5));
        }
    }

    [Fact]
    public void Copies_OfRandomCurves_AreEqualWithEqualHashes_MovedOnesAreNot()
    {
        var aRand = new Random(20261029);
        var aHasher = new GeomHash_CurveHasher();
        for (int aKind = 0; aKind <= 10; aKind++)
        {
            for (int i = 0; i < 20; i++)
            {
                Geom_Curve aCurve = Curve(aRand, aKind);
                var aCopy = (Geom_Curve)aCurve.Copy();
                Assert.True(aHasher.Equals(aCurve, aCopy), aCurve.GetType().Name);
                Assert.Equal(aHasher.GetHashCode(aCurve), aHasher.GetHashCode(aCopy));
                var aMoved = (Geom_Curve)aCurve.Translated(new gp_Vec(0.5, 0.0, 0.0));
                Assert.False(aHasher.Equals(aCurve, aMoved), aCurve.GetType().Name);
            }
        }
    }

    [Fact]
    public void Copies_OfRandomSurfaces_AreEqualWithEqualHashes_MovedOnesAreNot()
    {
        var aRand = new Random(20261030);
        var aHasher = new GeomHash_SurfaceHasher();
        for (int aKind = 0; aKind <= 11; aKind++)
        {
            for (int i = 0; i < 20; i++)
            {
                Geom_Surface aSurface = Surface(aRand, aKind);
                var aCopy = (Geom_Surface)aSurface.Copy();
                Assert.True(aHasher.Equals(aSurface, aCopy), aSurface.GetType().Name);
                Assert.Equal(aHasher.GetHashCode(aSurface), aHasher.GetHashCode(aCopy));
                var aMoved = (Geom_Surface)aSurface.Translated(new gp_Vec(0.5, 0.0, 0.0));
                Assert.False(aHasher.Equals(aSurface, aMoved), aSurface.GetType().Name);
            }
        }
    }

    [Fact]
    public void Copies_OfRandom2dCurves_AreEqualWithEqualHashes_MovedOnesAreNot()
    {
        var aRand = new Random(20261031);
        var aHasher = new Geom2dHash_CurveHasher();
        for (int aKind = 0; aKind <= 9; aKind++)
        {
            for (int i = 0; i < 20; i++)
            {
                Geom2d_Curve aCurve = Curve2d(aRand, aKind);
                var aCopy = (Geom2d_Curve)aCurve.Copy();
                Assert.True(aHasher.Equals(aCurve, aCopy), aCurve.GetType().Name);
                Assert.Equal(aHasher.GetHashCode(aCurve), aHasher.GetHashCode(aCopy));
                var aMoved = (Geom2d_Curve)aCurve.Translated(new gp_Vec2d(0.5, 0.0));
                Assert.False(aHasher.Equals(aCurve, aMoved), aCurve.GetType().Name);
            }
        }
    }

    [Fact]
    public void Hashers_DeduplicateInHashSetsAndDictionaries()
    {
        var aRand = new Random(7);
        var aCurves = new HashSet<Geom_Curve?>(new GeomHash_CurveHasher());
        var aSurfaces = new Dictionary<Geom_Surface, int>(new GeomHash_SurfaceHasher());
        var aCurves2d = new HashSet<Geom2d_Curve?>(new Geom2dHash_CurveHasher());
        var aDistinct = new List<(Geom_Curve, Geom_Surface, Geom2d_Curve)>();
        for (int i = 0; i < 40; i++)
        {
            aDistinct.Add((Curve(aRand, i % 11), Surface(aRand, i % 12), Curve2d(aRand, i % 10)));
        }

        for (int aPass = 0; aPass < 2; aPass++)
        {
            for (int i = 0; i < aDistinct.Count; i++)
            {
                (Geom_Curve aC, Geom_Surface aS, Geom2d_Curve aC2d) = aDistinct[i];
                aCurves.Add(aPass == 0 ? aC : (Geom_Curve)aC.Copy());
                aSurfaces.TryAdd(aPass == 0 ? aS : (Geom_Surface)aS.Copy(), i);
                aCurves2d.Add(aPass == 0 ? aC2d : (Geom2d_Curve)aC2d.Copy());
            }
        }

        Assert.Equal(40, aCurves.Count);
        Assert.Equal(40, aSurfaces.Count);
        Assert.Equal(40, aCurves2d.Count);
        Assert.Equal(17, aSurfaces[(Geom_Surface)aDistinct[17].Item2.Copy()]);
        Assert.True(aCurves.Add(null));
        Assert.False(aCurves.Add(null));
    }

    [Fact]
    public void MeshHashers_DeduplicateInHashSets()
    {
        var aTris = new HashSet<Poly_Triangulation?>(new GeomHash_TriangulationHasher(1.0e-9, 1.0e-6));
        for (int aPass = 0; aPass < 2; aPass++)
        {
            for (int i = 0; i < 5; i++)
            {
                var aTri = new Poly_Triangulation(3, 1, false);
                aTri.SetNode(1, new gp_Pnt(i, 0.0, 0.0));
                aTri.SetNode(2, new gp_Pnt(i + 1.0, 0.0, 0.0));
                aTri.SetNode(3, new gp_Pnt(i, 1.0, 0.0));
                aTri.SetTriangle(1, new Poly_Triangle(1, 2, 3));
                aTris.Add(aTri);
            }
        }

        Assert.Equal(5, aTris.Count);
    }

    [Fact]
    public void Hashes_AreDeterministic()
    {
        // FNV-1a over the rounded values: the same in every run and on every platform
        var aLine = new Geom_Line(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0.0, 0.0, 1.0));
        Assert.Equal(new GeomHash_CurveHasher().GetHashCode(aLine), new GeomHash_CurveHasher().GetHashCode((Geom_Line)aLine.Copy()));
        Assert.Equal(0xCBF29CE484222325UL, GeomHash_HashUtils.HashBytes([]));
        Assert.Equal(0, GeomHash_HashUtils.ToInt32(0));
    }

    [Fact]
    public void DefaultTolerances_AreOcctsPrecisionValues()
    {
        var aCurveHasher = new GeomHash_CurveHasher();
        Assert.Equal(Precision.Angular(), aCurveHasher.CompTolerance);
        Assert.Equal(Precision.Confusion(), aCurveHasher.HashTolerance);
        Assert.Equal(Precision.Angular(), new GeomHash_SurfaceHasher().CompTolerance);
        Assert.Equal(Precision.Confusion(), new Geom2dHash_CurveHasher().HashTolerance);
        Assert.Equal(Precision.Computational(), new GeomHash_Polygon2DHasher().CompTolerance);
        Assert.Equal(Precision.Computational(), new GeomHash_Polygon3DHasher().HashTolerance);
        Assert.Equal(Precision.Computational(), new GeomHash_PolygonOnTriHasher().CompTolerance);
        Assert.Equal(Precision.Computational(), new GeomHash_TriangulationHasher().HashTolerance);
    }

    [Fact]
    public void UnknownTypes_HashByTypeName_CompareByReference()
    {
        // TestLine derives from Geom_Curve without being one of the types the hasher dispatches on
        var aHasher = new GeomHash_CurveHasher();
        var aLine1 = new TestLine(new gp_Pnt(0, 0, 0), new gp_Vec(1, 0, 0));
        var aLine2 = new TestLine(new gp_Pnt(0, 0, 0), new gp_Vec(1, 0, 0));
        Assert.Equal(aHasher.GetHashCode(aLine1), aHasher.GetHashCode(aLine2));
        Assert.False(aHasher.Equals(aLine1, aLine2));
        Assert.True(aHasher.Equals(aLine1, aLine1));
    }

    [Fact]
    public void BSplineCurves_PeriodicityIsNotCompared_OcctReadsPastThePoles()
    {
        // OCCT: same degree and knots, periodic (4 poles) and non-periodic (3 poles). The non-periodic curve first
        // compares 3 poles and finds the curves equal; the periodic one first reads a 4th pole of the other, past
        // its array (OCCT's release build reads out of bounds; the port's Pole throws)
        var aKnots = new NCollection_Array1<double>(1, 4);
        var aMults = new NCollection_Array1<int>(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aKnots[i] = i - 1.0;
            aMults[i] = i == 1 || i == 4 ? 2 : 1;
        }

        var aPoles4 = new NCollection_Array1<gp_Pnt>(1, 4);
        var aPoles3 = new NCollection_Array1<gp_Pnt>(1, 3);
        for (int i = 1; i <= 4; i++)
        {
            aPoles4[i] = new gp_Pnt(i, i * i, 0.0);
            if (i <= 3)
            {
                aPoles3[i] = aPoles4[i];
            }
        }

        var aPeriodic = new Geom_BSplineCurve(aPoles4, aKnots, aMults, 2, true);
        var aNonPeriodic = new Geom_BSplineCurve(aPoles3, aKnots, aMults, 2, false);
        var aHasher = new GeomHash_CurveHasher();
        Assert.True(aHasher.Equals(aNonPeriodic, aPeriodic));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aHasher.Equals(aPeriodic, aNonPeriodic));
    }
}
