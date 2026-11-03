// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_PrmPrmIntersection, its polyhedra and their interference (unit 5D): lines on both surfaces for B-spline
/// pairs and an offset surface, the grid helpers and the voxel bits, the error paths, and OCCT's quirks pinned
/// (occt-issues.md, 5D). The step-by-step comparison with native OCCT is IntPatchPrmPrmNativeTests.
/// </summary>
public class IntPatch_PrmPrmIntersectionTests
{
    private const double THE_TOL = 1.0e-7, THE_FLECHE = 0.01, THE_PAS = 0.05;

    private static IntPatch_PrmPrmIntersection Sampled(Geom_Surface theS1, Geom_Surface theS2,
                                                        out GeomAdaptor_Surface theA1, out GeomAdaptor_Surface theA2)
    {
        theA1 = new GeomAdaptor_Surface(theS1);
        theA2 = new GeomAdaptor_Surface(theS2);
        var anInt = new IntPatch_PrmPrmIntersection();
        anInt.Perform(theA1, new Adaptor3d_TopolTool(theA1), theA2, new Adaptor3d_TopolTool(theA2), THE_TOL, THE_TOL,
                      THE_FLECHE, THE_PAS);
        return anInt;
    }

    // Every point of every line, and every vertex, is on both surfaces at its parameters.
    private static void AssertLinesOnBothSurfaces(IntPatch_PrmPrmIntersection theInt, Adaptor3d_Surface theS1,
                                                  Adaptor3d_Surface theS2, double theTol)
    {
        for (int i = 1; i <= theInt.NbLines(); i++)
        {
            var aWL = Assert.IsType<IntPatch_WLine>(theInt.Line(i));
            Assert.Equal(IntPatch_WLine.IntPatch_WLType.IntPatch_WLPrmPrm, aWL.GetCreatingWay());
            Assert.True(aWL.NbPnts() >= 2);
            Assert.True(aWL.NbVertex() >= 2);
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                IntSurf_PntOn2S aP = aWL.Point(k);
                aP.Parameters(out double u1, out double v1, out double u2, out double v2);
                Assert.True(theS1.Value(u1, v1).Distance(aP.Value()) < theTol, $"line {i} point {k} off S1");
                Assert.True(theS2.Value(u2, v2).Distance(aP.Value()) < theTol, $"line {i} point {k} off S2");
            }
            for (int k = 1; k <= aWL.NbVertex(); k++)
            {
                IntPatch_Point aV = aWL.Vertex(k);
                aV.Parameters(out double u1, out double v1, out double u2, out double v2);
                Assert.True(theS1.Value(u1, v1).Distance(aV.Value()) < theTol, $"line {i} vertex {k} off S1");
                Assert.True(theS2.Value(u2, v2).Distance(aV.Value()) < theTol, $"line {i} vertex {k} off S2");
            }
        }
    }

    [Fact]
    public void BSplineBSpline_LinesOnBothSurfaces()
    {
        IntPatch_PrmPrmIntersection anInt = Sampled(IntPatchPrmPrmNativeTests.PatchA(), IntPatchPrmPrmNativeTests.PatchB(),
                                                    out var aS1, out var aS2);
        Assert.True(anInt.IsDone());
        Assert.False(anInt.IsEmpty());
        Assert.Equal(3, anInt.NbLines());
        AssertLinesOnBothSurfaces(anInt, aS1, aS2, 1.0e-6);
    }

    [Fact]
    public void OffsetBSpline_LinesOnBothSurfaces()
    {
        IntPatch_PrmPrmIntersection anInt = Sampled(new Geom_OffsetSurface(IntPatchPrmPrmNativeTests.PatchA(), 0.15),
                                                    IntPatchPrmPrmNativeTests.PatchB(), out var aS1, out var aS2);
        Assert.True(anInt.IsDone());
        Assert.Equal(2, anInt.NbLines());
        AssertLinesOnBothSurfaces(anInt, aS1, aS2, 1.0e-6);
    }

    [Fact]
    public void Voxels_LinesOnBothSurfaces()
    {
        IntPatch_PrmPrmIntersection anInt = Sampled(IntPatchPrmPrmNativeTests.Bumpy(1, 0.0),
                                                    IntPatchPrmPrmNativeTests.Bumpy(2, 0.12), out var aS1, out var aS2);
        Assert.True(anInt.IsDone());
        Assert.True(anInt.NbLines() > 10);
        AssertLinesOnBothSurfaces(anInt, aS1, aS2, 1.0e-6);
    }

    [Fact]
    public void Polyhedra_LinesOnBothSurfaces_AndTheSameWithPolyhedraGivenEitherWay()
    {
        var aS1 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchA());
        var aS2 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchB());
        var aD1 = new Adaptor3d_TopolTool(aS1);
        var aD2 = new Adaptor3d_TopolTool(aS2);
        var aPoly1 = new IntPatch_Polyhedron(aS1);
        var aPoly2 = new IntPatch_Polyhedron(aS2);

        var anA = new IntPatch_PrmPrmIntersection();
        anA.Perform(aS1, aPoly1, aD1, aS2, aPoly2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        var aB = new IntPatch_PrmPrmIntersection();
        aB.Perform(aS1, aD1, aS2, aPoly2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        var aC = new IntPatch_PrmPrmIntersection();
        aC.Perform(aS1, aPoly1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);

        Assert.True(anA.NbLines() > 0);
        AssertLinesOnBothSurfaces(anA, aS1, aS2, 1.0e-6);
        foreach (IntPatch_PrmPrmIntersection anOther in new[] { aB, aC })
        {
            Assert.Equal(anA.NbLines(), anOther.NbLines());
            for (int i = 1; i <= anA.NbLines(); i++)
            {
                var aL1 = (IntPatch_WLine)anA.Line(i);
                var aL2 = (IntPatch_WLine)anOther.Line(i);
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
        var aS = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.Folded());
        var anInt = new IntPatch_PrmPrmIntersection();
        anInt.Perform(aS, new Adaptor3d_TopolTool(aS), THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        Assert.True(anInt.IsDone());
        Assert.Equal(1, anInt.NbLines());
        AssertLinesOnBothSurfaces(anInt, aS, aS, 1.0e-6);
        var aWL = (IntPatch_WLine)anInt.Line(1);
        aWL.Point(aWL.NbPnts() / 2).Parameters(out double u1, out double v1, out double u2, out double v2);
        Assert.True(Math.Abs(u1 - u2) + Math.Abs(v1 - v2) > 1.0e-3);
    }

    [Fact]
    public void StartPoint_OneLine_AndNoneFarFromTheSurfaces()
    {
        var aS1 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchA());
        var aS2 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchB());
        var anInt = new IntPatch_PrmPrmIntersection();
        anInt.Perform(aS1, new Adaptor3d_TopolTool(aS1), aS2, new Adaptor3d_TopolTool(aS2), 0.5, 0.5, 0.5, 0.5, THE_TOL,
                      THE_TOL, THE_FLECHE, THE_PAS);
        Assert.Equal(1, anInt.NbLines());
        AssertLinesOnBothSurfaces(anInt, aS1, aS2, 1.0e-6);

        // Two parallel planes 1 apart: no start point, done and empty.
        var aP1 = new GeomAdaptor_Surface(new Geom_Plane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), -1, 1, -1, 1);
        var aP2 = new GeomAdaptor_Surface(new Geom_Plane(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), -1, 1, -1, 1);
        var anEmpty = new IntPatch_PrmPrmIntersection();
        anEmpty.Perform(aP1, new Adaptor3d_TopolTool(aP1), aP2, new Adaptor3d_TopolTool(aP2), 0.0, 0.0, 0.0, 0.0,
                        THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        Assert.True(anEmpty.IsDone());
        Assert.True(anEmpty.IsEmpty());
        Assert.Equal(0, anEmpty.NbLines());
    }

    [Fact]
    public void ListOfPoints_ClosedSurface_AddsTheSeamImages_AndAnEmptyListIsDone()
    {
        var aS1 = new GeomAdaptor_Surface(new Geom_RectangularTrimmedSurface(IntPatchPrmPrmNativeTests.Cylinder(), 0.0,
                                                                             2.0 * M_PI, -2.0, 2.0));
        var aS2 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.Patch(0.0, 3.0, -2.0, 2.0));
        var aList = new NCollection_List<IntSurf_PntOn2S>();
        IntSurf_PntOn2S aP = new();
        aP.SetValue(0.0, 0.0, 1.0 / 3.0, 0.5);
        aList.Append(aP);
        var anInt = new IntPatch_PrmPrmIntersection();
        anInt.Perform(aS1, new Adaptor3d_TopolTool(aS1), aS2, new Adaptor3d_TopolTool(aS2), THE_TOL, THE_TOL,
                      THE_FLECHE, THE_PAS, aList);
        Assert.True(anInt.IsDone());
        // The point on the seam U = 0 gains its image at U = 2 pi, appended to the caller's list.
        Assert.Equal(2, aList.Size());
        aList.Last().Parameters(out double u1, out _, out _, out _);
        Assert.Equal(2.0 * M_PI, u1);

        var anEmpty = new IntPatch_PrmPrmIntersection();
        anEmpty.Perform(aS1, new Adaptor3d_TopolTool(aS1), aS2, new Adaptor3d_TopolTool(aS2), THE_TOL, THE_TOL,
                        THE_FLECHE, THE_PAS, new NCollection_List<IntSurf_PntOn2S>());
        Assert.True(anEmpty.IsDone());
        Assert.Equal(0, anEmpty.NbLines());
    }

    [Fact]
    public void NotDone_Throws()
    {
        var anInt = new IntPatch_PrmPrmIntersection();
        Assert.False(anInt.IsDone());
        Assert.Throws<NotDoneException>(() => anInt.NbLines());
        Assert.Throws<NotDoneException>(() => anInt.IsEmpty());
        Assert.Throws<NotDoneException>(() => anInt.Line(1));
    }

    [Fact]
    public void DisjointSurfaces_DoneAndEmpty()
    {
        var aFar = new Geom_Plane(new Point3D(0, 0, 10), new Direction3D(0, 0, 1));
        var aS1 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchA());
        var aS2 = new GeomAdaptor_Surface(aFar, -2, 2, -2, 2);
        var anInt = new IntPatch_PrmPrmIntersection();
        anInt.Perform(aS1, new Adaptor3d_TopolTool(aS1), aS2, new Adaptor3d_TopolTool(aS2), THE_TOL, THE_TOL,
                      THE_FLECHE, THE_PAS);
        Assert.True(anInt.IsDone());
        Assert.True(anInt.IsEmpty());
        Assert.Equal(0, anInt.NbLines());
    }

    [Fact]
    public void NewLine_KeepsTheEnds_AndChecksItsArguments()
    {
        IntPatch_PrmPrmIntersection anInt = Sampled(IntPatchPrmPrmNativeTests.PatchA(), IntPatchPrmPrmNativeTests.PatchB(),
                                                    out var aS1, out var aS2);
        var aWL = (IntPatch_WLine)anInt.Line(1);
        int aHigh = Math.Min(aWL.NbPnts(), 6);
        var aNew = Assert.IsType<IntPatch_WLine>(anInt.NewLine(aS1, aS2, 1, 1, aHigh, 20));
        Assert.Equal(aWL.Point(1), aNew.Point(1));
        Assert.Equal(aWL.Point(aHigh), aNew.Point(aNew.NbPnts()));
        Assert.True(aNew.NbPnts() > aHigh);
        for (int k = 1; k <= aNew.NbPnts(); k++)
        {
            IntSurf_PntOn2S aP = aNew.Point(k);
            aP.Parameters(out double u1, out double v1, out double u2, out double v2);
            Assert.True(aS1.Value(u1, v1).Distance(aP.Value()) < 1.0e-5);
            Assert.True(aS2.Value(u2, v2).Distance(aP.Value()) < 1.0e-5);
        }
        Assert.Throws<OutOfRangeException>(() => anInt.NewLine(aS1, aS2, 0, 1, 2, 5));
        Assert.Throws<OutOfRangeException>(() => anInt.NewLine(aS1, aS2, 1, 3, 3, 5));
    }

    [Fact]
    public void Grid_Helpers()
    {
        var anInt = new IntPatch_PrmPrmIntersection();
        Assert.Equal(128, anInt.NbPointsGrille());
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
        var aMap = new IntPatch_PrmPrmIntersection_T3Bits(128);
        anInt.Remplit(anInt.GrilleInteger(0, 0, 0), anInt.GrilleInteger(8, 0, 0), anInt.GrilleInteger(0, 8, 0), aMap);
        Assert.NotEqual(0, aMap.Val(anInt.GrilleInteger(0, 0, 0)));
        Assert.NotEqual(0, aMap.Val(anInt.GrilleInteger(4, 0, 0)));
        Assert.NotEqual(0, aMap.Val(anInt.GrilleInteger(0, 4, 0)));
        Assert.Equal(0, aMap.Val(anInt.GrilleInteger(0, 0, 1)));
    }

    [Fact]
    public void T3Bits_AddRazAnd()
    {
        var aM1 = new IntPatch_PrmPrmIntersection_T3Bits(4); // 64 bits, two words
        var aM2 = new IntPatch_PrmPrmIntersection_T3Bits(4);
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
        var aSphere = new GeomAdaptor_Surface(new Geom_SphericalSurface(new CoordinateSystem(), 1.0));
        var aPoly = new IntPatch_Polyhedron(aSphere, 4, 4);
        int aNbVoid = 0;
        for (int i = 1; i <= aPoly.NbTriangles(); i++)
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
        Assert.Equal(aPoly.NbTriangles() - aNbVoid, new IntPatch_PolyhedronBVH(aPoly).Size());
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
        var aS = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.Folded());
        var aPoly = new IntPatch_Polyhedron(aS, 7, 5);
        var aBuilt = new IntPatch_InterferencePolyhedron(aPoly);
        Assert.True(aBuilt.NbSectionLines() > 0);

        // OCCT's Perform(Obje) calls an empty Interference(Obje) (occt-issues.md, 5D).
        var aPerformed = new IntPatch_InterferencePolyhedron();
        aPerformed.Perform(aPoly);
        Assert.Equal(0, aPerformed.NbSectionLines());
        Assert.Equal(0, aPerformed.NbTangentZones());
        Assert.Equal(aBuilt.GetTolerance(), aPerformed.GetTolerance());

        // Perform on two polyhedra gives what the constructor gives.
        var aS2 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchB());
        var aPoly2 = new IntPatch_Polyhedron(aS2);
        var aPoly1 = new IntPatch_Polyhedron(new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchA()));
        var aC = new IntPatch_InterferencePolyhedron(aPoly1, aPoly2);
        var aP = new IntPatch_InterferencePolyhedron();
        aP.Perform(aPoly1, aPoly2);
        Assert.Equal(aC.NbSectionLines(), aP.NbSectionLines());
        Assert.Equal(aC.NbTangentZones(), aP.NbTangentZones());
        for (int i = 1; i <= aC.NbSectionLines(); i++)
        {
            Assert.True(aC.LineValue(i).IsEqual(aP.LineValue(i)));
        }
    }
}
