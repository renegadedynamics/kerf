// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// GeomConvert_BSplineSurfaceKnotSplitting, GeomConvert_BSplineSurfaceToBezierSurface and
/// GeomConvert_CompBezierSurfacesToBSplineSurface (OCCT has no GTests for them).
/// </summary>
public class GeomConvert_SurfaceSplittingTests
{
    // U: degree 3, knots 0 1 2 3 with multiplicities 4 1 2 4 (C2 at 1, C1 at 2): 7 poles.
    // V: degree 2, knots 0 0.5 1 with multiplicities 3 1 3 (C1 at 0.5): 4 poles.
    private static readonly double[] s_UKnots = [0.0, 1.0, 2.0, 3.0];
    private static readonly int[] s_UMults = [4, 1, 2, 4];
    private static readonly double[] s_VKnots = [0.0, 0.5, 1.0];
    private static readonly int[] s_VMults = [3, 1, 3];

    private static NCollection_Array1<T> Array<T>(T[] theValues)
    {
        var anArray = new NCollection_Array1<T>(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            anArray[i + 1] = theValues[i];
        }

        return anArray;
    }

    private static NCollection_Array2<Point3D> Poles(int theNbU, int theNbV)
    {
        var aPoles = new NCollection_Array2<Point3D>(1, theNbU, 1, theNbV);
        for (int i = 1; i <= theNbU; i++)
        {
            for (int j = 1; j <= theNbV; j++)
            {
                aPoles[i, j] = new Point3D(i * 1.5 + 0.1 * j * j, j * 2.0 - 0.2 * i, Math.Sin(i + 2.0 * j));
            }
        }

        return aPoles;
    }

    private static BSplineSurface Surface(bool theRational = false)
    {
        NCollection_Array2<Point3D> aPoles = Poles(7, 4);
        if (!theRational)
        {
            return new BSplineSurface(aPoles, Array(s_UKnots), Array(s_VKnots), Array(s_UMults), Array(s_VMults), 3, 2);
        }

        var aWeights = new NCollection_Array2<double>(1, 7, 1, 4);
        for (int i = 1; i <= 7; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aWeights[i, j] = 1.0 + 0.3 * ((i + j) % 3);
            }
        }

        return new BSplineSurface(aPoles, aWeights, Array(s_UKnots), Array(s_VKnots), Array(s_UMults),
                                       Array(s_VMults), 3, 2);
    }

    private static void AssertNear(in Point3D theExpected, in Point3D theActual, double theTol = 1e-10)
        => Assert.True(theExpected.Distance(theActual) <= theTol, $"{theExpected} vs {theActual}");

    [Fact]
    public void KnotSplitting_ByContinuity()
    {
        BSplineSurface aSurf = Surface();

        var aC0 = new BSplineSurfaceKnotSplitting(aSurf, 0, 0);
        Assert.Equal(2, aC0.USplitCount);
        Assert.Equal(2, aC0.VSplitCount);
        Assert.Equal(1, aC0.USplitValue(1));
        Assert.Equal(4, aC0.USplitValue(2));
        Assert.Equal(3, aC0.VSplitValue(2));

        // C1 required: only knots of multiplicity >= Degree split; none inside
        var aC1 = new BSplineSurfaceKnotSplitting(aSurf, 1, 1);
        Assert.Equal(2, aC1.USplitCount);
        Assert.Equal(2, aC1.VSplitCount);

        // C2 required: U splits at knot 3 (C1 only), V at knot 2 (C1 only)
        var aC2 = new BSplineSurfaceKnotSplitting(aSurf, 2, 2);
        var aUSplit = new NCollection_Array1<int>(1, aC2.USplitCount);
        var aVSplit = new NCollection_Array1<int>(1, aC2.VSplitCount);
        aC2.Splitting(aUSplit, aVSplit);
        Assert.Equal([1, 3, 4], aUSplit.AsSpan().ToArray());
        Assert.Equal([1, 2, 3], aVSplit.AsSpan().ToArray());

        // C3: every inner U knot (degree 3, so a simple knot is only C2)
        var aC3 = new BSplineSurfaceKnotSplitting(aSurf, 3, 0);
        Assert.Equal(4, aC3.USplitCount);

        Assert.ThrowsAny<RangeErrorException>(() => new BSplineSurfaceKnotSplitting(aSurf, -1, 0));
        Assert.ThrowsAny<RangeErrorException>(() => aC2.USplitValue(0));
        Assert.ThrowsAny<RangeErrorException>(() => aC2.VSplitValue(4));
    }

    [Fact]
    public void KnotSplitting_SmoothV_ReproducesOcctError()
    {
        // V periodic, degree 2, simple knots: already C1 in V, which OCCT writes into the U split indices
        var aPoles = Poles(7, 4);
        var aVKnots = Array([0.0, 1.0, 2.0, 3.0, 4.0]);
        var aVMults = Array([1, 1, 1, 1, 1]);
        var aSurf = new BSplineSurface(aPoles, Array(s_UKnots), aVKnots, Array(s_UMults), aVMults, 3, 2, false, true);

        var aSplit = new BSplineSurfaceKnotSplitting(aSurf, 2, 1);
        // The U result (1, 3, 4) has been replaced by V's first and last indices
        Assert.Equal(2, aSplit.USplitCount);
        Assert.Equal(aSurf.FirstVKnotIndex(), aSplit.USplitValue(1));
        Assert.Equal(aSurf.LastVKnotIndex(), aSplit.USplitValue(2));
        Assert.ThrowsAny<NullReferenceException>(() => aSplit.VSplitCount);
    }

    private static void CheckPatches(BSplineSurface theSurf, BSplineSurfaceToBezierSurface theConv,
                                     double theU1, double theU2, double theV1, double theV2)
    {
        int aNbU = theConv.UPatchCount, aNbV = theConv.VPatchCount;
        var aUKnots = new NCollection_Array1<double>(1, aNbU + 1);
        var aVKnots = new NCollection_Array1<double>(1, aNbV + 1);
        theConv.UKnots(aUKnots);
        theConv.VKnots(aVKnots);
        Assert.Equal(theU1, aUKnots[1], 12);
        Assert.Equal(theU2, aUKnots[aNbU + 1], 12);
        Assert.Equal(theV1, aVKnots[1], 12);
        Assert.Equal(theV2, aVKnots[aNbV + 1], 12);

        var aPatches = new NCollection_Array2<BezierSurface>(1, aNbU, 1, aNbV);
        theConv.Patches(aPatches);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                BezierSurface aPatch = aPatches[i, j];
                Assert.False(aPatch.IsFrozen);
                Assert.Equal(theSurf.UDegree(), aPatch.UDegree());
                Assert.Equal(theSurf.VDegree(), aPatch.VDegree());
                for (int s = 0; s <= 4; s++)
                {
                    for (int t = 0; t <= 4; t++)
                    {
                        double a = s / 4.0, b = t / 4.0;
                        double u = aUKnots[i] + a * (aUKnots[i + 1] - aUKnots[i]);
                        double v = aVKnots[j] + b * (aVKnots[j + 1] - aVKnots[j]);
                        AssertNear(theSurf.Value(u, v), aPatch.Value(a, b));
                    }
                }
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BezierPatches_EvaluateAsTheSurface(bool theRational)
    {
        BSplineSurface aSurf = Surface(theRational);
        var aConv = new BSplineSurfaceToBezierSurface(aSurf);
        Assert.Equal(3, aConv.UPatchCount);
        Assert.Equal(2, aConv.VPatchCount);
        CheckPatches(aSurf, aConv, 0.0, 3.0, 0.0, 1.0);
        Assert.Equal(theRational, aConv.Patch(1, 1).IsURational || aConv.Patch(1, 1).IsVRational);
        Assert.ThrowsAny<OutOfRangeException>(() => aConv.Patch(4, 1));
        Assert.ThrowsAny<OutOfRangeException>(() => aConv.Patch(1, 0));

        // The input is untouched (the converter works on a copy), and may be frozen
        Assert.Equal(4, aSurf.UKnotCount);
        aSurf.Freeze();
        _ = new BSplineSurfaceToBezierSurface(aSurf);
    }

    [Fact]
    public void BezierPatches_OfARange()
    {
        BSplineSurface aSurf = Surface();
        var aConv = new BSplineSurfaceToBezierSurface(aSurf, 0.5, 2.5, 0.2, 1.0, 1e-6);
        Assert.Equal(3, aConv.UPatchCount);
        Assert.Equal(2, aConv.VPatchCount);
        CheckPatches(aSurf, aConv, 0.5, 2.5, 0.2, 1.0);

        // Bounds within ParametricTolerance / 2 of a knot outside the range snap onto it
        var aSnapped = new BSplineSurfaceToBezierSurface(aSurf, 1.0 - 1e-8, 3.0, 0.0, 0.5 + 1e-8, 1e-6);
        Assert.Equal(2, aSnapped.UPatchCount);
        Assert.Equal(1, aSnapped.VPatchCount);
        CheckPatches(aSurf, aSnapped, 1.0, 3.0, 0.0, 0.5);

        Assert.ThrowsAny<DomainErrorException>(() => new BSplineSurfaceToBezierSurface(aSurf, 1.0, 1.0 + 1e-8, 0.0, 1.0, 1e-6));
    }

    private static NCollection_Array2<BezierSurface> Split(BSplineSurface theSurf)
    {
        var aConv = new BSplineSurfaceToBezierSurface(theSurf);
        var aPatches = new NCollection_Array2<BezierSurface>(1, aConv.UPatchCount, 1, aConv.VPatchCount);
        aConv.Patches(aPatches);
        return aPatches;
    }

    private static BSplineSurface Build(CompBezierSurfacesToBSplineSurface theConv)
        => new(theConv.Poles(), theConv.UKnots(), theConv.VKnots(), theConv.UMultiplicities(), theConv.VMultiplicities(),
               theConv.UDegree(), theConv.VDegree());

    [Fact]
    public void CompBezierSurfaces_PiecewiseBezier_RoundTrip()
    {
        BSplineSurface aSurf = Surface();
        NCollection_Array2<BezierSurface> aPatches = Split(aSurf);
        var aConv = new CompBezierSurfacesToBSplineSurface(aPatches);
        Assert.True(aConv.IsDone);
        Assert.Equal(3, aConv.UDegree());
        Assert.Equal(2, aConv.VDegree());
        Assert.Equal(4, aConv.UKnotCount);
        Assert.Equal(3, aConv.VKnotCount);
        Assert.Equal(10, aConv.UPoleCount);
        Assert.Equal(5, aConv.VPoleCount);
        Assert.Equal([4, 3, 3, 4], aConv.UMultiplicities().AsSpan().ToArray());
        Assert.Equal([0.0, 1.0, 2.0], aConv.VKnots().AsSpan().ToArray());

        BSplineSurface aResult = Build(aConv);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                for (int s = 0; s <= 3; s++)
                {
                    for (int t = 0; t <= 3; t++)
                    {
                        double a = s / 3.0, b = t / 3.0;
                        AssertNear(aPatches[i, j].Value(a, b), aResult.Value(i - 1 + a, j - 1 + b));
                    }
                }
            }
        }

        // The accessors return copies
        aConv.UKnots()[1] = 42.0;
        Assert.Equal(0.0, aConv.UKnots()[1]);
    }

    [Fact]
    public void CompBezierSurfaces_WithKnots_AndContinuity()
    {
        BSplineSurface aSurf = Surface();
        NCollection_Array2<BezierSurface> aPatches = Split(aSurf);

        // C0 with the original knots: the original surface, with every inner knot of multiplicity = degree
        var aC0 = new CompBezierSurfacesToBSplineSurface(aPatches, Array(s_UKnots), Array(s_VKnots));
        Assert.True(aC0.IsDone);
        BSplineSurface aR0 = Build(aC0);

        // C1 in U and V: the knots go back to multiplicity 2 (U) and 1 (V)
        var aC1 = new CompBezierSurfacesToBSplineSurface(aPatches, Array(s_UKnots), Array(s_VKnots),
                                                                     C1, C1, 1e-7);
        Assert.True(aC1.IsDone);
        Assert.Equal([4, 2, 2, 4], aC1.UMultiplicities().AsSpan().ToArray());
        Assert.Equal([3, 1, 3], aC1.VMultiplicities().AsSpan().ToArray());
        BSplineSurface aR1 = Build(aC1);

        for (int s = 0; s <= 12; s++)
        {
            for (int t = 0; t <= 10; t++)
            {
                double u = 3.0 * s / 12.0, v = t / 10.0;
                AssertNear(aSurf.Value(u, v), aR0.Value(u, v));
                AssertNear(aSurf.Value(u, v), aR1.Value(u, v), 1e-8);
            }
        }

        // C2 in U is impossible at knot 3 (the surface is only C1 there)
        var aC2 = new CompBezierSurfacesToBSplineSurface(aPatches, Array(s_UKnots), Array(s_VKnots),
                                                                     C2, C0, 1e-7);
        Assert.False(aC2.IsDone);

        Assert.ThrowsAny<ConstructionErrorException>(
            () => new CompBezierSurfacesToBSplineSurface(aPatches, Array(s_UKnots), Array(s_VKnots),
                                                                     G1, C0, 1e-7));
        // C2 in V needs degree 3 with several V patches
        Assert.ThrowsAny<ConstructionErrorException>(
            () => new CompBezierSurfacesToBSplineSurface(aPatches, Array(s_UKnots), Array(s_VKnots),
                                                                     C0, C2, 1e-7));
    }

    [Fact]
    public void CompBezierSurfaces_PatchesOfDifferentDegrees_AreNotModified()
    {
        // Two patches in U of a degree (2, 2) surface, the second raised to (3, 2), all frozen
        var aPoles = Poles(5, 3);
        var aSurf = new BSplineSurface(aPoles, Array([0.0, 1.0, 2.0]), Array([0.0, 1.0]), Array([3, 2, 3]),
                                            Array([3, 3]), 2, 2);
        NCollection_Array2<BezierSurface> aPatches = Split(aSurf);
        aPatches[2, 1] = aPatches[2, 1].Increased(3, 2);
        aPatches[1, 1].Freeze();
        aPatches[2, 1].Freeze();

        var aConv = new CompBezierSurfacesToBSplineSurface(aPatches);
        Assert.Equal(3, aConv.UDegree());
        Assert.Equal(2, aPatches[1, 1].UDegree());
        BSplineSurface aResult = Build(aConv);
        for (int s = 0; s <= 8; s++)
        {
            double u = 2.0 * s / 8.0;
            AssertNear(aSurf.Value(u, 0.3), aResult.Value(u, 0.3));
        }
    }

    [Fact]
    public void CompBezierSurfaces_WithTolerance_RemovesUKnotsOnly()
    {
        // One patch row in U, three in V of a surface that is C1 in V: OCCT's V removal loop tests the U
        // multiplicity, which stays 0 with a single U patch, so no V knot is removed (occt-issues.md)
        var aPoles = Poles(4, 5);
        var aSurf = new BSplineSurface(aPoles, Array([0.0, 1.0]), Array([0.0, 1.0, 2.0, 3.0]), Array([4, 4]),
                                            Array([3, 1, 1, 3]), 3, 2);
        NCollection_Array2<BezierSurface> aPatches = Split(aSurf);
        var aConv = new CompBezierSurfacesToBSplineSurface(aPatches, 1e-7);
        Assert.True(aConv.IsDone);
        Assert.Equal([3, 2, 2, 3], aConv.VMultiplicities().AsSpan().ToArray());
        Assert.Equal(2, aConv.UKnotCount);

        // Corners of the grid are those of the surface
        BSplineSurface aResult = Build(aConv);
        aResult.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        AssertNear(aSurf.Value(0.0, 0.0), aResult.Value(aU1, aV1));
        AssertNear(aSurf.Value(1.0, 3.0), aResult.Value(aU2, aV2));

        // Three patches in U of a C2 surface (simple knots): the U knots go back to multiplicity 1 (OCCT stops
        // before trying 0)
        var aPoles2 = Poles(6, 3);
        var aSurf2 = new BSplineSurface(aPoles2, Array([0.0, 1.0, 2.0, 3.0]), Array([0.0, 1.0]),
                                             Array([4, 1, 1, 4]), Array([3, 3]), 3, 2);
        var aConv2 = new CompBezierSurfacesToBSplineSurface(Split(aSurf2), 1e-7);
        Assert.Equal([4, 1, 1, 4], aConv2.UMultiplicities().AsSpan().ToArray());
        Assert.Equal(6, aConv2.UPoleCount);
        BSplineSurface aResult2 = Build(aConv2);
        aResult2.Bounds(out aU1, out aU2, out aV1, out aV2);
        AssertNear(aSurf2.Value(0.0, 0.0), aResult2.Value(aU1, aV1), 1e-8);
        AssertNear(aSurf2.Value(3.0, 1.0), aResult2.Value(aU2, aV2), 1e-8);
    }

    [Fact]
    public void CompBezierSurfaces_Rational_NotImplemented()
    {
        NCollection_Array2<BezierSurface> aPatches = Split(Surface(true));
        Assert.ThrowsAny<MethodNotImplementedException>(() => new CompBezierSurfacesToBSplineSurface(aPatches));
    }
}
