// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_PrmPrmIntersection, its polyhedra and their interference (unit 5D): lines on both surfaces for B-spline
/// pairs and an offset surface, the grid helpers and the voxel bits, the error paths, and OCCT's quirks pinned
/// (occt-issues.md, 5D). The step-by-step comparison with native OCCT is IntPatchPrmPrmNativeTests.
/// </summary>
public class IntPatch_PrmPrmIntersectionTests
{
    private const double THE_TOL = 1.0e-7, THE_FLECHE = 0.01, THE_PAS = 0.05;

    private static PrmPrmIntersection Sampled(Surface theS1, Surface theS2,
                                                        out SurfaceAdaptor theA1, out SurfaceAdaptor theA2)
    {
        theA1 = new SurfaceAdaptor(theS1);
        theA2 = new SurfaceAdaptor(theS2);
        var anInt = new PrmPrmIntersection();
        anInt.Perform(theA1, new TopolTool(theA1), theA2, new TopolTool(theA2), THE_TOL, THE_TOL,
                      THE_FLECHE, THE_PAS);
        return anInt;
    }

    // Every point of every line, and every vertex, is on both surfaces at its parameters.
    private static void AssertLinesOnBothSurfaces(PrmPrmIntersection theInt, SurfaceAdaptorBase theS1,
                                                  SurfaceAdaptorBase theS2, double theTol)
    {
        for (int i = 1; i <= theInt.LineCount; i++)
        {
            var aWL = Assert.IsType<WLine>(theInt.Line(i));
            Assert.Equal(WLine.IntPatch_WLType.WLPrmPrm, aWL.GetCreatingWay());
            Assert.True(aWL.NbPnts() >= 2);
            Assert.True(aWL.VertexCount >= 2);
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                PntOn2S aP = aWL.Point(k);
                aP.Parameters(out double u1, out double v1, out double u2, out double v2);
                Assert.True(theS1.Value(u1, v1).DistanceTo(aP.Value()) < theTol, $"line {i} point {k} off S1");
                Assert.True(theS2.Value(u2, v2).DistanceTo(aP.Value()) < theTol, $"line {i} point {k} off S2");
            }
            for (int k = 1; k <= aWL.VertexCount; k++)
            {
                PatchPoint aV = aWL.Vertex(k);
                aV.Parameters(out double u1, out double v1, out double u2, out double v2);
                Assert.True(theS1.Value(u1, v1).DistanceTo(aV.Value()) < theTol, $"line {i} vertex {k} off S1");
                Assert.True(theS2.Value(u2, v2).DistanceTo(aV.Value()) < theTol, $"line {i} vertex {k} off S2");
            }
        }
    }

    [Fact]
    public void BSplineBSpline_LinesOnBothSurfaces()
    {
        PrmPrmIntersection anInt = Sampled(IntPatchPrmPrmNativeTests.PatchA(), IntPatchPrmPrmNativeTests.PatchB(),
                                                    out var aS1, out var aS2);
        Assert.True(anInt.IsDone);
        Assert.False(anInt.IsEmpty);
        Assert.Equal(3, anInt.LineCount);
        AssertLinesOnBothSurfaces(anInt, aS1, aS2, 1.0e-6);
    }

    [Fact]
    public void OffsetBSpline_LinesOnBothSurfaces()
    {
        PrmPrmIntersection anInt = Sampled(new OffsetSurface(IntPatchPrmPrmNativeTests.PatchA(), 0.15),
                                                    IntPatchPrmPrmNativeTests.PatchB(), out var aS1, out var aS2);
        Assert.True(anInt.IsDone);
        Assert.Equal(2, anInt.LineCount);
        AssertLinesOnBothSurfaces(anInt, aS1, aS2, 1.0e-6);
    }

    [Fact]
    public void Voxels_LinesOnBothSurfaces()
    {
        PrmPrmIntersection anInt = Sampled(IntPatchPrmPrmNativeTests.Bumpy(1, 0.0),
                                                    IntPatchPrmPrmNativeTests.Bumpy(2, 0.12), out var aS1, out var aS2);
        Assert.True(anInt.IsDone);
        Assert.True(anInt.LineCount > 10);
        AssertLinesOnBothSurfaces(anInt, aS1, aS2, 1.0e-6);
    }

    [Fact]
    public void Polyhedra_LinesOnBothSurfaces_AndTheSameWithPolyhedraGivenEitherWay()
    {
        var aS1 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchA());
        var aS2 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchB());
        var aD1 = new TopolTool(aS1);
        var aD2 = new TopolTool(aS2);
        var aPoly1 = new Polyhedron(aS1);
        var aPoly2 = new Polyhedron(aS2);

        var anA = new PrmPrmIntersection();
        anA.Perform(aS1, aPoly1, aD1, aS2, aPoly2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        var aB = new PrmPrmIntersection();
        aB.Perform(aS1, aD1, aS2, aPoly2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        var aC = new PrmPrmIntersection();
        aC.Perform(aS1, aPoly1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);

        Assert.True(anA.LineCount > 0);
        AssertLinesOnBothSurfaces(anA, aS1, aS2, 1.0e-6);
        foreach (PrmPrmIntersection anOther in new[] { aB, aC })
        {
            Assert.Equal(anA.LineCount, anOther.LineCount);
            for (int i = 1; i <= anA.LineCount; i++)
            {
                var aL1 = (WLine)anA.Line(i);
                var aL2 = (WLine)anOther.Line(i);
                Assert.Equal(aL1.NbPnts(), aL2.NbPnts());
                for (int k = 1; k <= aL1.NbPnts(); k++)
                {
                    Assert.Equal(aL1.Point(k), aL2.Point(k));
                }
            }
        }
    }

    [Fact]
    public void SelfIntersection_OffTheDiagonal()
    {
        var aS = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.Folded());
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS, new TopolTool(aS), THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        Assert.True(anInt.IsDone);
        Assert.Equal(1, anInt.LineCount);
        AssertLinesOnBothSurfaces(anInt, aS, aS, 1.0e-6);
        var aWL = (WLine)anInt.Line(1);
        aWL.Point(aWL.NbPnts() / 2).Parameters(out double u1, out double v1, out double u2, out double v2);
        Assert.True(Math.Abs(u1 - u2) + Math.Abs(v1 - v2) > 1.0e-3);
    }

    [Fact]
    public void StartPoint_OneLine_AndNoneFarFromTheSurfaces()
    {
        var aS1 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchA());
        var aS2 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchB());
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS1, new TopolTool(aS1), aS2, new TopolTool(aS2), 0.5, 0.5, 0.5, 0.5, THE_TOL,
                      THE_TOL, THE_FLECHE, THE_PAS);
        Assert.Equal(1, anInt.LineCount);
        AssertLinesOnBothSurfaces(anInt, aS1, aS2, 1.0e-6);

        // Two parallel planes 1 apart: no start point, done and empty.
        var aP1 = new SurfaceAdaptor(new PlaneSurface(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), -1, 1, -1, 1);
        var aP2 = new SurfaceAdaptor(new PlaneSurface(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), -1, 1, -1, 1);
        var anEmpty = new PrmPrmIntersection();
        anEmpty.Perform(aP1, new TopolTool(aP1), aP2, new TopolTool(aP2), 0.0, 0.0, 0.0, 0.0,
                        THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        Assert.True(anEmpty.IsDone);
        Assert.True(anEmpty.IsEmpty);
        Assert.Equal(0, anEmpty.LineCount);
    }

    [Fact]
    public void ListOfPoints_ClosedSurface_AddsTheSeamImages_AndAnEmptyListIsDone()
    {
        var aS1 = new SurfaceAdaptor(new TrimmedSurface(IntPatchPrmPrmNativeTests.Cylinder(), 0.0,
                                                                             2.0 * M_PI, -2.0, 2.0));
        var aS2 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.Patch(0.0, 3.0, -2.0, 2.0));
        var aList = new NCollection_List<PntOn2S>();
        PntOn2S aP = new();
        aP.SetValue(0.0, 0.0, 1.0 / 3.0, 0.5);
        aList.Append(aP);
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS1, new TopolTool(aS1), aS2, new TopolTool(aS2), THE_TOL, THE_TOL,
                      THE_FLECHE, THE_PAS, aList);
        Assert.True(anInt.IsDone);
        // The point on the seam U = 0 gains its image at U = 2 pi, appended to the caller's list.
        Assert.Equal(2, aList.Size());
        aList.Last().Parameters(out double u1, out _, out _, out _);
        Assert.Equal(2.0 * M_PI, u1);

        var anEmpty = new PrmPrmIntersection();
        anEmpty.Perform(aS1, new TopolTool(aS1), aS2, new TopolTool(aS2), THE_TOL, THE_TOL,
                        THE_FLECHE, THE_PAS, new NCollection_List<PntOn2S>());
        Assert.True(anEmpty.IsDone);
        Assert.Equal(0, anEmpty.LineCount);
    }

    [Fact]
    public void NotDone_Throws()
    {
        var anInt = new PrmPrmIntersection();
        Assert.False(anInt.IsDone);
        EngineAssert.Throws<NotDoneException>(() => anInt.LineCount);
        EngineAssert.Throws<NotDoneException>(() => anInt.IsEmpty);
        EngineAssert.Throws<NotDoneException>(() => anInt.Line(1));
    }

    [Fact]
    public void DisjointSurfaces_DoneAndEmpty()
    {
        var aFar = new PlaneSurface(new Point3D(0, 0, 10), new Direction3D(0, 0, 1));
        var aS1 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchA());
        var aS2 = new SurfaceAdaptor(aFar, -2, 2, -2, 2);
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS1, new TopolTool(aS1), aS2, new TopolTool(aS2), THE_TOL, THE_TOL,
                      THE_FLECHE, THE_PAS);
        Assert.True(anInt.IsDone);
        Assert.True(anInt.IsEmpty);
        Assert.Equal(0, anInt.LineCount);
    }

    [Fact]
    public void NewLine_KeepsTheEnds_AndChecksItsArguments()
    {
        PrmPrmIntersection anInt = Sampled(IntPatchPrmPrmNativeTests.PatchA(), IntPatchPrmPrmNativeTests.PatchB(),
                                                    out var aS1, out var aS2);
        var aWL = (WLine)anInt.Line(1);
        int aHigh = Math.Min(aWL.NbPnts(), 6);
        var aNew = Assert.IsType<WLine>(anInt.NewLine(aS1, aS2, 1, 1, aHigh, 20));
        Assert.Equal(aWL.Point(1), aNew.Point(1));
        Assert.Equal(aWL.Point(aHigh), aNew.Point(aNew.NbPnts()));
        Assert.True(aNew.NbPnts() > aHigh);
        for (int k = 1; k <= aNew.NbPnts(); k++)
        {
            PntOn2S aP = aNew.Point(k);
            aP.Parameters(out double u1, out double v1, out double u2, out double v2);
            Assert.True(aS1.Value(u1, v1).DistanceTo(aP.Value()) < 1.0e-5);
            Assert.True(aS2.Value(u2, v2).DistanceTo(aP.Value()) < 1.0e-5);
        }
        EngineAssert.Throws<OutOfRangeException>(() => anInt.NewLine(aS1, aS2, 0, 1, 2, 5));
        EngineAssert.Throws<OutOfRangeException>(() => anInt.NewLine(aS1, aS2, 1, 3, 3, 5));
    }

    [Fact]
    public void Grid_Helpers()
    {
        var anInt = new PrmPrmIntersection();
        Assert.Equal(128, anInt.PointsGrilleCount);
        int t = anInt.GrilleInteger(5, 127, 64);
        Assert.Equal(5 | (127 << 7) | (64 << 14), t);
        anInt.IntegerGrille(t, out int ix, out int iy, out int iz);
        Assert.Equal((5, 127, 64), (ix, iy, iz));
        Assert.Equal(1, anInt.DansGrille(0));
        Assert.Equal(1, anInt.DansGrille(127));
        Assert.Equal(0, anInt.DansGrille(128));
        Assert.Equal(0, anInt.DansGrille(-1));
        Assert.Equal(0, anInt.CodeReject(0, 0, 0, 1, 1, 1, 0.5, 0.5, 0.5));
        Assert.Equal(1 | 16 | 4, anInt.CodeReject(0, 0, 0, 1, 1, 1, -1, 2, -1));
        Assert.Equal(8 | 2 | 32, anInt.CodeReject(0, 0, 0, 1, 1, 1, 2, -1, 2));

        // Remplit marks the three voxels and the triangle between them.
        var aMap = new PrmPrmIntersectionT3Bits(128);
        anInt.Remplit(anInt.GrilleInteger(0, 0, 0), anInt.GrilleInteger(8, 0, 0), anInt.GrilleInteger(0, 8, 0), aMap);
        Assert.NotEqual(0, aMap.Val(anInt.GrilleInteger(0, 0, 0)));
        Assert.NotEqual(0, aMap.Val(anInt.GrilleInteger(4, 0, 0)));
        Assert.NotEqual(0, aMap.Val(anInt.GrilleInteger(0, 4, 0)));
        Assert.Equal(0, aMap.Val(anInt.GrilleInteger(0, 0, 1)));
    }

    [Fact]
    public void T3Bits_AddRazAnd()
    {
        var aM1 = new PrmPrmIntersectionT3Bits(4); // 64 bits, two words
        var aM2 = new PrmPrmIntersectionT3Bits(4);
        aM1.Add(31);
        aM1.Add(40);
        aM1.Add(3);
        aM2.Add(31);
        aM2.Add(40);
        aM2.Add(5);
        Assert.Equal(int.MinValue, aM1.Val(31)); // bit 31 is the sign bit, as in C++
        int anInd = 0;
        Assert.Equal(1, aM1.And(aM2, ref anInd));
        Assert.Equal(31, anInd);
        Assert.Equal(0, aM1.Val(31));
        Assert.Equal(0, aM2.Val(31));
        Assert.Equal(1, aM1.And(aM2, ref anInd));
        Assert.Equal(40, anInd);
        Assert.Equal(0, aM1.And(aM2, ref anInd));
        Assert.Equal(40, anInd);
        aM1.Raz(3);
        Assert.Equal(0, aM1.Val(3));
    }

    [Fact]
    public void Polyhedron_DegenerateTriangles_VoidBoxes_LeftOutOfTheBVH()
    {
        // The sphere's poles collapse a row of triangles.
        var aSphere = new SurfaceAdaptor(new SphericalSurface(new CoordinateSystem(), 1.0));
        var aPoly = new Polyhedron(aSphere, 4, 4);
        int aNbVoid = 0;
        for (int i = 1; i <= aPoly.TriangleCount; i++)
        {
            if (aPoly.ComponentsBounding().Value(i).IsVoid)
            {
                aNbVoid++;
                double aDist = 7.0;
                aPoly.PlaneEquation(i, out Xyz aNormal, ref aDist);
                Assert.Equal(new Xyz(1, 0, 0), aNormal);
                Assert.Equal(7.0, aDist); // left unchanged, as OCCT leaves it
            }
        }
        Assert.Equal(8, aNbVoid);
        Assert.Equal(aPoly.TriangleCount - aNbVoid, new PolyhedronBvh(aPoly).Size());
        Assert.True(aPoly.DeflectionOverEstimation() >= 0.0001);
        aPoly.Triangle(9, out int aP1, out int aP2, out int aP3);
        Assert.True(aPoly.Contain(9, aPoly.Point(aP2)));
        Point3D aMid = new((aPoly.Point(aP1).XYZ() + aPoly.Point(aP2).XYZ() + aPoly.Point(aP3).XYZ()) / 3.0);
        Assert.True(aPoly.Contain(9, aMid));
        Assert.False(aPoly.Contain(9, new Point3D(aMid.XYZ() + (aPoly.Point(aP1).XYZ() - aMid.XYZ()) * 3.0)));
    }

    [Fact]
    public void InterferencePolyhedron_SelfPerformComputesNothing_AsInOcct()
    {
        var aS = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.Folded());
        var aPoly = new Polyhedron(aS, 7, 5);
        var aBuilt = new InterferencePolyhedron(aPoly);
        Assert.True(aBuilt.SectionLineCount > 0);

        // OCCT's Perform(Obje) calls an empty Interference(Obje) (occt-issues.md, 5D).
        var aPerformed = new InterferencePolyhedron();
        aPerformed.Perform(aPoly);
        Assert.Equal(0, aPerformed.SectionLineCount);
        Assert.Equal(0, aPerformed.TangentZoneCount);
        Assert.Equal(aBuilt.GetTolerance(), aPerformed.GetTolerance());

        // Perform on two polyhedra gives what the constructor gives.
        var aS2 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchB());
        var aPoly2 = new Polyhedron(aS2);
        var aPoly1 = new Polyhedron(new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchA()));
        var aC = new InterferencePolyhedron(aPoly1, aPoly2);
        var aP = new InterferencePolyhedron();
        aP.Perform(aPoly1, aPoly2);
        Assert.Equal(aC.SectionLineCount, aP.SectionLineCount);
        Assert.Equal(aC.TangentZoneCount, aP.TangentZoneCount);
        for (int i = 1; i <= aC.SectionLineCount; i++)
        {
            Assert.True(aC.LineValue(i).IsEqual(aP.LineValue(i)));
        }
    }
}
