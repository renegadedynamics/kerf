// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection.Polyhedra;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntPolyh on B-spline pairs: the couples of interfering triangles and the start points are real contacts and points
/// of both surfaces, and are the same every run, serially and with the BVH builder's parallel loops (decisions 490,
/// 5503). The oracle is indirect (units 5D and 5G).
/// </summary>
public class IntPolyhTests
{
    // A bicubic B-spline surface on [x0, x0 + L] x [y0, y0 + L] with 5 x 5 poles, z = f(x, y) at the poles
    private static SurfaceAdaptor MakeBSpline(double x0, double y0, double L, Func<double, double, double> f)
    {
        NCollection_Array2<Point3D> aPoles = new(1, 5, 1, 5);
        for (int i = 1; i <= 5; i++)
        {
            for (int j = 1; j <= 5; j++)
            {
                double x = x0 + L * (i - 1) / 4.0, y = y0 + L * (j - 1) / 4.0;
                aPoles[i, j] = new Point3D(x, y, f(x, y));
            }
        }

        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 0.5;
        aKnots[3] = 1.0;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        return new SurfaceAdaptor(new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3));
    }

    // A vertical B-spline surface x = x(y, z), as the image of the (y, z) square
    private static SurfaceAdaptor MakeVerticalBSpline(Func<double, double, double> theX, double y0, double z0, double L)
    {
        NCollection_Array2<Point3D> aPoles = new(1, 5, 1, 5);
        for (int i = 1; i <= 5; i++)
        {
            for (int j = 1; j <= 5; j++)
            {
                double y = y0 + L * (i - 1) / 4.0, z = z0 + L * (j - 1) / 4.0;
                aPoles[i, j] = new Point3D(theX(y, z), y, z);
            }
        }

        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 0.5;
        aKnots[3] = 1.0;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        return new SurfaceAdaptor(new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3));
    }

    // A wavy surface and a tilted, gently curved one crossing it
    private static (SurfaceAdaptor, SurfaceAdaptor) CrossingPair()
        => (MakeBSpline(0.0, 0.0, 4.0, (x, y) => 0.6 * Math.Sin(1.3 * x) * Math.Cos(0.9 * y)),
            MakeBSpline(-0.5, -0.3, 5.0, (x, y) => 0.25 * x - 0.5 + 0.05 * (y - 2.0) * (y - 2.0)));

    // Every double of the results, as bits, with the counts
    private static List<long> Fingerprint(PolyhedronIntersection theInter)
    {
        List<long> aBits = [theInter.IsDone ? 1 : 0, theInter.IsParallel ? 1 : 0, theInter.SectionLineCount,
                            theInter.TangentZoneCount];
        for (int iL = 1; iL <= theInter.SectionLineCount; ++iL)
        {
            aBits.Add(theInter.NbPointsInLine(iL));
            for (int iP = 1; iP <= theInter.NbPointsInLine(iL); ++iP)
            {
                theInter.GetLinePoint(iL, iP, out double x, out double y, out double z, out double u1, out double v1,
                                      out double u2, out double v2, out double anAngle);
                foreach (double d in new[] { x, y, z, u1, v1, u2, v2, anAngle })
                {
                    aBits.Add(BitConverter.DoubleToInt64Bits(d));
                }
            }
        }

        for (int iZ = 1; iZ <= theInter.TangentZoneCount; ++iZ)
        {
            theInter.GetTangentZonePoint(iZ, 1, out double x, out double y, out double z, out double u1, out double v1,
                                         out double u2, out double v2);
            foreach (double d in new[] { x, y, z, u1, v1, u2, v2 })
            {
                aBits.Add(BitConverter.DoubleToInt64Bits(d));
            }
        }

        return aBits;
    }

    // The steps of IntPolyh_Intersection's standard intersection (ComputeIntersection)
    private static PolyhedronMaillageAffinage Triangulate(SurfaceAdaptor theS1, SurfaceAdaptor theS2, int theNbS)
    {
        PolyhedronMaillageAffinage aMaillage = new(theS1, theNbS, theNbS, theS2, theNbS, theNbS, 0);
        aMaillage.FillArrayOfPnt(1);
        aMaillage.FillArrayOfPnt(2);
        aMaillage.CommonBox();
        aMaillage.FillArrayOfTriangles(1);
        aMaillage.FillArrayOfTriangles(2);
        aMaillage.FillArrayOfEdges(1);
        aMaillage.FillArrayOfEdges(2);
        aMaillage.TrianglesDeflectionsRefinementBSB();
        aMaillage.TriangleCompare();
        return aMaillage;
    }

    private static List<long> CouplesFingerprint(PolyhedronMaillageAffinage theMaillage)
    {
        List<long> aBits = [theMaillage.GetArrayOfPoints(1).ItemCount, theMaillage.GetArrayOfPoints(2).ItemCount,
                            theMaillage.GetArrayOfTriangles(1).ItemCount, theMaillage.GetArrayOfTriangles(2).ItemCount,
                            theMaillage.GetArrayOfEdges(1).ItemCount, theMaillage.GetArrayOfEdges(2).ItemCount];
        foreach (PolyhedronCouple aCouple in theMaillage.GetCouples())
        {
            aBits.Add(aCouple.FirstValue());
            aBits.Add(aCouple.SecondValue());
            aBits.Add(BitConverter.DoubleToInt64Bits(aCouple.Angle()));
        }

        return aBits;
    }

    [Fact]
    public void BSplinePair_Couples_AreContactsOfMarkedTriangles()
    {
        (SurfaceAdaptor aS1, SurfaceAdaptor aS2) = CrossingPair();
        PolyhedronMaillageAffinage aMaillage = Triangulate(aS1, aS2, 10);

        NCollection_List<PolyhedronCouple> aCouples = aMaillage.GetCouples();
        Assert.True(aCouples.Extent() > 10);

        PolyhedronArray<PolyhedronTriangle> aTr1 = aMaillage.GetArrayOfTriangles(1), aTr2 = aMaillage.GetArrayOfTriangles(2);
        PolyhedronArray<PolyhedronPoint> aPt1 = aMaillage.GetArrayOfPoints(1), aPt2 = aMaillage.GetArrayOfPoints(2);
        int aPrevious = -1;
        foreach (PolyhedronCouple aCouple in aCouples)
        {
            // TriangleCompare walks the BVH pairs sorted by the first triangle, then the second
            Assert.True(aCouple.FirstValue() >= aPrevious);
            aPrevious = aCouple.FirstValue();

            PolyhedronTriangle aT1 = aTr1.Value(aCouple.FirstValue()), aT2 = aTr2.Value(aCouple.SecondValue());
            Assert.True(aT1.HasIntersection && aT2.HasIntersection);
            Assert.True(aT1.IsIntersectionPossible && aT2.IsIntersectionPossible);
            double anAngle = -2.0;
            Assert.Equal(1, aMaillage.TriContact(aPt1.Value(aT1.FirstPoint()), aPt1.Value(aT1.SecondPoint()),
                                                 aPt1.Value(aT1.ThirdPoint()), aPt2.Value(aT2.FirstPoint()),
                                                 aPt2.Value(aT2.SecondPoint()), aPt2.Value(aT2.ThirdPoint()), ref anAngle));
            Assert.Equal(aCouple.Angle(), anAngle);
            Assert.InRange(Math.Abs(anAngle), 0.0, 1.0 + 1e-12);

            // The start points of the couple lie in both triangles' boxes
            PolyhedronStartPoint aSP1 = new(), aSP2 = new();
            int aNb = aMaillage.StartingPointsResearch(aCouple.FirstValue(), aCouple.SecondValue(), ref aSP1, ref aSP2);
            if (aNb >= 1)
            {
                Point3D aP = new(aSP1.X(), aSP1.Y(), aSP1.Z());
                Assert.False(aT1.CachedBoundingBox().IsOut(aP));
                Assert.False(aT2.CachedBoundingBox().IsOut(aP));
                Assert.Equal(aCouple.FirstValue(), aSP1.T1());
                Assert.Equal(aCouple.SecondValue(), aSP1.T2());
            }
        }
    }

    [Fact]
    public void BSplinePair_CouplesAndStartPoints_SameEveryRunAndSerial()
    {
        (SurfaceAdaptor aS1, SurfaceAdaptor aS2) = CrossingPair();
        List<long> aFirst = CouplesFingerprint(Triangulate(aS1, aS2, 10));
        for (int i = 0; i < 3; ++i)
        {
            Assert.Equal(aFirst, CouplesFingerprint(Triangulate(aS1, aS2, 10)));
        }

        using (OsdParallel.ForceSerialScope())
        {
            Assert.Equal(aFirst, CouplesFingerprint(Triangulate(aS1, aS2, 10)));
        }

        // The same for the start points chained by the whole algorithm, on fresh adaptors too
        List<long> aLines = Fingerprint(new PolyhedronIntersection(aS1, aS2));
        for (int i = 0; i < 3; ++i)
        {
            (SurfaceAdaptor aT1, SurfaceAdaptor aT2) = CrossingPair();
            Assert.Equal(aLines, Fingerprint(new PolyhedronIntersection(aT1, aT2)));
        }

        using (OsdParallel.ForceSerialScope())
        {
            Assert.Equal(aLines, Fingerprint(new PolyhedronIntersection(aS1, aS2)));
        }
    }

    [Fact]
    public void BSplinePair_SectionLines_LieOnBothSurfaces()
    {
        (SurfaceAdaptor aS1, SurfaceAdaptor aS2) = CrossingPair();
        PolyhedronIntersection anInter = new(aS1, 15, 15, aS2, 15, 15);
        Assert.True(anInter.IsDone);
        Assert.False(anInter.IsParallel);
        Assert.True(anInter.SectionLineCount > 0);

        int aNbPoints = 0;
        for (int iL = 1; iL <= anInter.SectionLineCount; ++iL)
        {
            for (int iP = 1; iP <= anInter.NbPointsInLine(iL); ++iP)
            {
                anInter.GetLinePoint(iL, iP, out double x, out double y, out double z, out double u1, out double v1,
                                     out double u2, out double v2, out double anAngle);
                Point3D aP = new(x, y, z);
                // The points are on the triangulations, within their deflection of the surfaces
                Assert.True(aS1.Value(u1, v1).Distance(aP) < 0.05, $"line {iL} point {iP} off the first surface");
                Assert.True(aS2.Value(u2, v2).Distance(aP) < 0.05, $"line {iL} point {iP} off the second surface");
                Assert.InRange(Math.Abs(anAngle), 0.0, 1.0 + 1e-12);
                ++aNbPoints;
            }
        }

        Assert.True(aNbPoints > 10);
    }

    [Fact]
    public void TangentBSplines_AdvancedIntersection_SameEveryRun()
    {
        // A bump touching a plane at its top: the standard intersection finds few nearly parallel couples, and the
        // four shifted intersections run
        SurfaceAdaptor aBump = MakeBSpline(0.0, 0.0, 2.0, (x, y) => -0.5 * ((x - 1) * (x - 1) + (y - 1) * (y - 1)));
        SurfaceAdaptor aTop = MakeBSpline(0.0, 0.0, 2.0, (x, y) => -0.0001);

        // IntPolyh_Intersection::IsAdvRequired: no couple, or at most 10 with one nearly parallel
        NCollection_List<PolyhedronCouple> aStd = Triangulate(aBump, aTop, 10).GetCouples();
        Assert.True(aStd.Extent() <= 10);
        Assert.True(aStd.Extent() == 0 || aStd.Any(c => Math.Abs(c.Angle()) > 0.996));

        PolyhedronIntersection anInter = new(aBump, aTop);
        Assert.True(anInter.IsDone);
        List<long> aFirst = Fingerprint(anInter);
        for (int i = 0; i < 3; ++i)
        {
            Assert.Equal(aFirst, Fingerprint(new PolyhedronIntersection(aBump, aTop)));
        }
    }

    [Fact]
    public void BSplinesBeyondBounds_EnlargedRetry_KeepsTheFirstPassTriangles()
    {
        // The vertical surface is just beyond the end of the other: only the enlarged sampling (1% at each end) reaches
        // it. OCCT retries on the same MaillageAffinage, whose triangles keep the first pass's flags and cached boxes
        // (occt-issues.md, unit 5E): every triangle was out of the (empty) common box, so the retry finds nothing.
        SurfaceAdaptor aFlat = MakeBSpline(0.0, 0.0, 1.0, (x, y) => 0.1 * x * y);
        SurfaceAdaptor aWall = MakeVerticalBSpline((y, z) => 1.004, -0.2, -0.5, 1.4);

        PolyhedronMaillageAffinage aMaillage = Triangulate(aFlat, aWall, 10);
        Assert.Equal(0, aMaillage.GetCouples().Extent());
        PolyhedronArray<PolyhedronTriangle> aTriangles = aMaillage.GetArrayOfTriangles(1);
        List<Box> aBoxes = [];
        for (int i = 0; i < aTriangles.ItemCount; ++i)
        {
            aBoxes.Add(aTriangles.Value(i).CachedBoundingBox());
        }

        aMaillage.SetEnlargeZone(true);
        aMaillage.FillArrayOfPnt(1);
        aMaillage.FillArrayOfPnt(2);
        aMaillage.CommonBox();
        aMaillage.FillArrayOfTriangles(1);
        aMaillage.FillArrayOfTriangles(2);
        aMaillage.FillArrayOfEdges(1);
        aMaillage.FillArrayOfEdges(2);
        aMaillage.TrianglesDeflectionsRefinementBSB();
        Assert.Equal(0, aMaillage.TriangleCompare());
        for (int i = 0; i < aBoxes.Count; ++i)
        {
            Assert.False(aTriangles.Value(i).IsIntersectionPossible);
            Assert.Equal(aBoxes[i], aTriangles.Value(i).CachedBoundingBox());
        }

        // A fresh MaillageAffinage on the enlarged sampling finds the contact
        PolyhedronMaillageAffinage aFresh = new(aFlat, 10, 10, aWall, 10, 10, 0);
        aFresh.SetEnlargeZone(true);
        aFresh.FillArrayOfPnt(1);
        aFresh.FillArrayOfPnt(2);
        aFresh.CommonBox();
        aFresh.FillArrayOfTriangles(1);
        aFresh.FillArrayOfTriangles(2);
        aFresh.FillArrayOfEdges(1);
        aFresh.FillArrayOfEdges(2);
        aFresh.TrianglesDeflectionsRefinementBSB();
        Assert.True(aFresh.TriangleCompare() > 0);

        PolyhedronIntersection anInter = new(aFlat, aWall);
        Assert.True(anInter.IsDone);
        Assert.Equal(0, anInter.SectionLineCount);
        Assert.Equal(0, anInter.TangentZoneCount);
    }

    [Fact]
    public void FillArrayOfPnt_ShiftedWithSampling_FillsTheFirstSurface()
    {
        // OCCT: FillArrayOfPnt(SurfID, isShiftFwd, Upars, Vpars) always fills the first surface's points
        (SurfaceAdaptor aS1, SurfaceAdaptor aS2) = CrossingPair();
        PolyhedronMaillageAffinage aMaillage = new(aS1, 10, 10, aS2, 10, 10, 0);
        NCollection_Array1<double> aU = new(), aV = new();
        aMaillage.MakeSampling(2, aU, aV);
        aMaillage.FillArrayOfPnt(2, true, aU, aV);
        Assert.Equal(100, aMaillage.GetArrayOfPoints(1).ItemCount);
        Assert.Equal(0, aMaillage.GetArrayOfPoints(2).ItemCount);
    }

    [Fact]
    public void Array_References_StayValidWhileGrowing()
    {
        PolyhedronArray<PolyhedronTriangle> anArray = new(4);
        anArray.Init(2);
        Assert.Equal(2, anArray.GetN());
        Assert.Equal(-1, anArray.Value(0).FirstPoint());
        Assert.True(anArray.Value(1).IsIntersectionPossible);

        ref PolyhedronTriangle aFirst = ref anArray[0];
        for (int i = 0; i < 2000; ++i)
        {
            anArray[anArray.ItemCount].SetFirstPoint(i);
            anArray.IncrementNbItems();
        }

        aFirst.SetDeflection(5.0);
        Assert.Equal(5.0, anArray.Value(0).Deflection());
        Assert.Equal(2000, anArray.ItemCount);
        Assert.True(anArray.GetN() > 2000);

        // Init again keeps the items, resetting only the last allocated one
        anArray.SetNbItems(0);
        anArray.Init(10);
        Assert.Equal(5.0, anArray.Value(0).Deflection());
        Assert.Equal(7, anArray.Value(7).FirstPoint());
        Assert.Equal(-1, anArray.Value(10).FirstPoint());
    }

    [Fact]
    public void SectionLine_AppendPrependAndCopy()
    {
        PolyhedronSectionLine aLine = new();
        Assert.Equal(0, aLine.GetN());
        aLine.Init(10000);
        Assert.Equal(1, aLine.GetN());
        Assert.Equal(0, aLine.StartPointCount);

        PolyhedronStartPoint aSP = new PolyhedronStartPoint().WithXYZ(1, 2, 3);
        aLine[aLine.StartPointCount] = aSP;
        aLine.IncrementNbStartPoints();
        aLine.Prepend(aSP.WithXYZ(4, 5, 6));
        Assert.Equal(2, aLine.StartPointCount);
        Assert.Equal(4.0, aLine.Value(0).X());
        Assert.Equal(1.0, aLine.Value(1).X());

        PolyhedronSectionLine aCopy = new(aLine);
        aLine[0] = aSP;
        Assert.Equal(4.0, aCopy.Value(0).X());

        PolyhedronArray<PolyhedronSectionLine> aLines = new();
        aLines.Init(3);
        Assert.NotSame(aLines.Value(0), aLines.Value(1));
    }

    [Fact]
    public void Couple_HasherIsOcctsUnorderedEquality()
    {
        HashSet<PolyhedronCouple> aMap = new(PolyhedronCouple.Hasher.Instance);
        Assert.True(aMap.Add(new PolyhedronCouple(3, 7, 0.5)));
        Assert.False(aMap.Add(new PolyhedronCouple(7, 3)));
        Assert.True(aMap.Add(new PolyhedronCouple(3, 8)));
        Assert.NotEqual(new PolyhedronCouple(3, 7, 0.5), new PolyhedronCouple(7, 3, 0.5));
        Assert.True(new PolyhedronCouple(3, 7, 0.5).IsEqual(new PolyhedronCouple(7, 3, 0.1)));
    }

    [Fact]
    public void Dumps_WriteOcctsFormat()
    {
        StringWriter aWriter = new();
        new PolyhedronPoint(1.5, -2.0, 0.0, 0.25, 1.0).Dump(3, aWriter);
        Assert.Equal("\nPoint(  3) : x=+1.500e+00g y=-2.000e+00g z=+0.000e+00g u=+2.500e-01g v=+1.000e+00g poc=  1\n",
                     aWriter.ToString());

        aWriter = new StringWriter();
        new PolyhedronCouple(1, 22, 0.5).Dump(0, aWriter);
        Assert.Equal("\nCouple(  0) :     1    22   0 0.500000\n", aWriter.ToString());
    }
}
