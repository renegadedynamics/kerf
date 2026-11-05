// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Hashing;
using Kerf.Engine.Meshing;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The port's own tests of GeomHash and Geom2dHash: copies of random geometry of every kind hash and compare equal,
/// moved geometry compares different, and the hashers deduplicate in .NET collections.
/// </summary>
public class GeomHash_PortTests
{
    private static double R(Random theRand, double theMin, double theMax) => theMin + (theMax - theMin) * theRand.NextDouble();

    private static Point3D Pnt(Random r) => new(R(r, -10, 10), R(r, -10, 10), R(r, -10, 10));

    private static Point2D Pnt2d(Random r) => new(R(r, -10, 10), R(r, -10, 10));

    private static Frame3D Ax2(Random r)
    {
        var aN = new Vector3D(R(r, -1, 1), R(r, -1, 1), R(r, -1, 1) + 2.0);
        return new Frame3D(Pnt(r), new Direction3D(aN));
    }

    private static Frame2D Ax22d(Random r) => new(Pnt2d(r), new Direction2D(R(r, -1, 1) + 2.0, R(r, -1, 1)), r.Next(2) == 0);

    private static BSplineCurve3D BSpline(Random r, bool theRational)
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
        var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
        var aWeights = new NCollection_Array1<double>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = Pnt(r);
            aWeights[i] = R(r, 0.5, 2);
        }

        return theRational ? new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, aDeg)
                           : new BSplineCurve3D(aPoles, aKnots, aMults, aDeg);
    }

    internal static Curve3D Curve(Random r, int theKind)
    {
        switch (theKind)
        {
            case 0: return new LineCurve3D(Pnt(r), new Direction3D(R(r, 1, 2), R(r, -1, 1), R(r, -1, 1)));
            case 1: return new CircleCurve3D(Ax2(r), R(r, 1, 5));
            case 2: return new EllipseCurve3D(Ax2(r), R(r, 5, 10), R(r, 1, 4));
            case 3: return new HyperbolaCurve3D(Ax2(r), R(r, 1, 5), R(r, 1, 5));
            case 4: return new ParabolaCurve3D(Ax2(r), R(r, 1, 5));
            case 5:
            case 6:
            {
                int aNb = r.Next(2, 7);
                var aPoles = new NCollection_Array1<Point3D>(1, aNb);
                var aWeights = new NCollection_Array1<double>(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    aPoles[i] = Pnt(r);
                    aWeights[i] = R(r, 0.5, 2);
                }

                return theKind == 5 ? new BezierCurve3D(aPoles) : new BezierCurve3D(aPoles, aWeights);
            }
            case 7: return BSpline(r, false);
            case 8: return BSpline(r, true);
            case 9: return new TrimmedCurve3D(new CircleCurve3D(Ax2(r), R(r, 1, 5)), R(r, 0, 1), R(r, 2, 3));
            default:
            {
                var aCircle = new CircleCurve3D(Ax2(r), R(r, 1, 5));
                return new OffsetCurve3D(aCircle, R(r, 0.1, 0.5), aCircle.Position.Direction);
            }
        }
    }

    internal static Surface Surface(Random r, int theKind)
    {
        switch (theKind)
        {
            case 0: return new PlaneSurface(new CoordinateSystem(Ax2(r)));
            case 1: return new CylindricalSurface(new CoordinateSystem(Ax2(r)), R(r, 1, 5));
            case 2: return new ConicalSurface(new CoordinateSystem(Ax2(r)), R(r, 0.1, 1), R(r, 0, 3));
            case 3: return new SphericalSurface(new CoordinateSystem(Ax2(r)), R(r, 1, 5));
            case 4: return new ToroidalSurface(new CoordinateSystem(Ax2(r)), R(r, 6, 9), R(r, 1, 5));
            case 5: return new RevolutionSurface(BSpline(r, false), new Axis3D(Pnt(r), new Direction3D(0, 0, 1)));
            case 6: return new ExtrusionSurface(BSpline(r, true), new Direction3D(R(r, -1, 1), R(r, -1, 1), 2.0));
            case 7:
            case 8:
            {
                int aNbU = r.Next(2, 5), aNbV = r.Next(2, 5);
                var aPoles = new NCollection_Array2<Point3D>(1, aNbU, 1, aNbV);
                var aWeights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
                for (int i = 1; i <= aNbU; i++)
                {
                    for (int j = 1; j <= aNbV; j++)
                    {
                        aPoles[i, j] = new Point3D(i + R(r, 0, 0.5), j + R(r, 0, 0.5), R(r, -1, 1));
                        aWeights[i, j] = R(r, 0.5, 2);
                    }
                }

                return theKind == 7 ? new BezierSurface(aPoles) : new BezierSurface(aPoles, aWeights);
            }
            case 9:
            {
                var aPoles = new NCollection_Array2<Point3D>(1, 3, 1, 4);
                for (int i = 1; i <= 3; i++)
                {
                    for (int j = 1; j <= 4; j++)
                    {
                        aPoles[i, j] = new Point3D(i, j, R(r, -1, 1));
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
                return new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 2, 2);
            }
            case 10: return new TrimmedSurface(Surface(r, 3), R(r, 0, 1), R(r, 2, 3), R(r, -1, 0), R(r, 0, 1));
            default: return new OffsetSurface(Surface(r, 9), R(r, 0.1, 0.5));
        }
    }

    internal static Curve2D Curve2d(Random r, int theKind)
    {
        switch (theKind)
        {
            case 0: return new LineCurve2D(Pnt2d(r), new Direction2D(R(r, 1, 2), R(r, -1, 1)));
            case 1: return new CircleCurve2D(Ax22d(r), R(r, 1, 5));
            case 2: return new EllipseCurve2D(Ax22d(r), R(r, 5, 10), R(r, 1, 4));
            case 3: return new HyperbolaCurve2D(Ax22d(r), R(r, 1, 5), R(r, 1, 5));
            case 4: return new ParabolaCurve2D(Ax22d(r), R(r, 1, 5));
            case 5:
            case 6:
            {
                int aNb = r.Next(2, 7);
                var aPoles = new NCollection_Array1<Point2D>(1, aNb);
                var aWeights = new NCollection_Array1<double>(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    aPoles[i] = Pnt2d(r);
                    aWeights[i] = R(r, 0.5, 2);
                }

                return theKind == 5 ? new BezierCurve2D(aPoles) : new BezierCurve2D(aPoles, aWeights);
            }
            case 7:
            {
                var aPoles = new NCollection_Array1<Point2D>(1, 4);
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
                return new BSplineCurve2D(aPoles, aKnots, aMults, 3);
            }
            case 8: return new TrimmedCurve2D(new CircleCurve2D(Ax22d(r), R(r, 1, 5)), R(r, 0, 1), R(r, 2, 3));
            default: return new OffsetCurve2D(new CircleCurve2D(Ax22d(r), R(r, 1, 5)), R(r, 0.1, 0.5));
        }
    }

    [Fact]
    public void Copies_OfRandomCurves_AreEqualWithEqualHashes_MovedOnesAreNot()
    {
        var aRand = new Random(20261029);
        var aHasher = new CurveHasher();
        for (int aKind = 0; aKind <= 10; aKind++)
        {
            for (int i = 0; i < 20; i++)
            {
                Curve3D aCurve = Curve(aRand, aKind);
                var aCopy = (Curve3D)aCurve.Copy();
                Assert.True(aHasher.Equals(aCurve, aCopy), global::Kerf.Engine.Foundation.OcctTypeNames.Of(aCurve.GetType()));
                Assert.Equal(aHasher.GetHashCode(aCurve), aHasher.GetHashCode(aCopy));
                var aMoved = (Curve3D)aCurve.Translated(new Vector3D(0.5, 0.0, 0.0));
                Assert.False(aHasher.Equals(aCurve, aMoved), global::Kerf.Engine.Foundation.OcctTypeNames.Of(aCurve.GetType()));
            }
        }
    }

    [Fact]
    public void Copies_OfRandomSurfaces_AreEqualWithEqualHashes_MovedOnesAreNot()
    {
        var aRand = new Random(20261030);
        var aHasher = new SurfaceHasher();
        for (int aKind = 0; aKind <= 11; aKind++)
        {
            for (int i = 0; i < 20; i++)
            {
                Surface aSurface = Surface(aRand, aKind);
                var aCopy = (Surface)aSurface.Copy();
                Assert.True(aHasher.Equals(aSurface, aCopy), global::Kerf.Engine.Foundation.OcctTypeNames.Of(aSurface.GetType()));
                Assert.Equal(aHasher.GetHashCode(aSurface), aHasher.GetHashCode(aCopy));
                var aMoved = (Surface)aSurface.Translated(new Vector3D(0.5, 0.0, 0.0));
                Assert.False(aHasher.Equals(aSurface, aMoved), global::Kerf.Engine.Foundation.OcctTypeNames.Of(aSurface.GetType()));
            }
        }
    }

    [Fact]
    public void Copies_OfRandom2dCurves_AreEqualWithEqualHashes_MovedOnesAreNot()
    {
        var aRand = new Random(20261031);
        var aHasher = new CurveHasher2D();
        for (int aKind = 0; aKind <= 9; aKind++)
        {
            for (int i = 0; i < 20; i++)
            {
                Curve2D aCurve = Curve2d(aRand, aKind);
                var aCopy = (Curve2D)aCurve.Copy();
                Assert.True(aHasher.Equals(aCurve, aCopy), global::Kerf.Engine.Foundation.OcctTypeNames.Of(aCurve.GetType()));
                Assert.Equal(aHasher.GetHashCode(aCurve), aHasher.GetHashCode(aCopy));
                var aMoved = (Curve2D)aCurve.Translated(new Vector2D(0.5, 0.0));
                Assert.False(aHasher.Equals(aCurve, aMoved), global::Kerf.Engine.Foundation.OcctTypeNames.Of(aCurve.GetType()));
            }
        }
    }

    [Fact]
    public void Hashers_DeduplicateInHashSetsAndDictionaries()
    {
        var aRand = new Random(7);
        var aCurves = new HashSet<Curve3D?>(new CurveHasher());
        var aSurfaces = new Dictionary<Surface, int>(new SurfaceHasher());
        var aCurves2d = new HashSet<Curve2D?>(new CurveHasher2D());
        var aDistinct = new List<(Curve3D, Surface, Curve2D)>();
        for (int i = 0; i < 40; i++)
        {
            aDistinct.Add((Curve(aRand, i % 11), Surface(aRand, i % 12), Curve2d(aRand, i % 10)));
        }

        for (int aPass = 0; aPass < 2; aPass++)
        {
            for (int i = 0; i < aDistinct.Count; i++)
            {
                (Curve3D aC, Surface aS, Curve2D aC2d) = aDistinct[i];
                aCurves.Add(aPass == 0 ? aC : (Curve3D)aC.Copy());
                aSurfaces.TryAdd(aPass == 0 ? aS : (Surface)aS.Copy(), i);
                aCurves2d.Add(aPass == 0 ? aC2d : (Curve2D)aC2d.Copy());
            }
        }

        Assert.Equal(40, aCurves.Count);
        Assert.Equal(40, aSurfaces.Count);
        Assert.Equal(40, aCurves2d.Count);
        Assert.Equal(17, aSurfaces[(Surface)aDistinct[17].Item2.Copy()]);
        Assert.True(aCurves.Add(null));
        Assert.False(aCurves.Add(null));
    }

    [Fact]
    public void MeshHashers_DeduplicateInHashSets()
    {
        var aTris = new HashSet<Triangulation?>(new TriangulationHasher(1.0e-9, 1.0e-6));
        for (int aPass = 0; aPass < 2; aPass++)
        {
            for (int i = 0; i < 5; i++)
            {
                var aTri = new Triangulation(3, 1, false);
                aTri.SetNode(1, new Point3D(i, 0.0, 0.0));
                aTri.SetNode(2, new Point3D(i + 1.0, 0.0, 0.0));
                aTri.SetNode(3, new Point3D(i, 1.0, 0.0));
                aTri.SetTriangle(1, new Triangle1(1, 2, 3));
                aTris.Add(aTri);
            }
        }

        Assert.Equal(5, aTris.Count);
    }

    [Fact]
    public void Hashes_AreDeterministic()
    {
        // FNV-1a over the rounded values: the same in every run and on every platform
        var aLine = new LineCurve3D(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 0.0, 1.0));
        Assert.Equal(new CurveHasher().GetHashCode(aLine), new CurveHasher().GetHashCode((LineCurve3D)aLine.Copy()));
        Assert.Equal(0xCBF29CE484222325UL, HashUtils.HashBytes([]));
        Assert.Equal(0, HashUtils.ToInt32(0));
    }

    [Fact]
    public void DefaultTolerances_AreOcctsPrecisionValues()
    {
        var aCurveHasher = new CurveHasher();
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Angular(), aCurveHasher.CompTolerance);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aCurveHasher.HashTolerance);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Angular(), new SurfaceHasher().CompTolerance);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), new CurveHasher2D().HashTolerance);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Computational(), new Polygon2DHasher().CompTolerance);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Computational(), new Polygon3DHasher().HashTolerance);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Computational(), new PolygonOnTriHasher().CompTolerance);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Computational(), new TriangulationHasher().HashTolerance);
    }

    [Fact]
    public void UnknownTypes_HashByTypeName_CompareByReference()
    {
        // TestLine derives from Geom_Curve without being one of the types the hasher dispatches on
        var aHasher = new CurveHasher();
        var aLine1 = new TestLine(new Point3D(0, 0, 0), new Vector3D(1, 0, 0));
        var aLine2 = new TestLine(new Point3D(0, 0, 0), new Vector3D(1, 0, 0));
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

        var aPoles4 = new NCollection_Array1<Point3D>(1, 4);
        var aPoles3 = new NCollection_Array1<Point3D>(1, 3);
        for (int i = 1; i <= 4; i++)
        {
            aPoles4[i] = new Point3D(i, i * i, 0.0);
            if (i <= 3)
            {
                aPoles3[i] = aPoles4[i];
            }
        }

        var aPeriodic = new BSplineCurve3D(aPoles4, aKnots, aMults, 2, true);
        var aNonPeriodic = new BSplineCurve3D(aPoles3, aKnots, aMults, 2, false);
        var aHasher = new CurveHasher();
        Assert.True(aHasher.Equals(aNonPeriodic, aPeriodic));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.OutOfRangeException>(() => aHasher.Equals(aPeriodic, aNonPeriodic));
    }
}
