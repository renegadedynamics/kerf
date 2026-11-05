// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_WLineTool (unit 5G): the purger on walking lines of the parametric intersections (points still on both
/// surfaces, end vertices kept, equal points removed with the vertices renumbered, degenerate lines dropped); the
/// joining of the cylinder/cylinder walking lines of OCCT's corpus cases bug23178 and bug28222_2, as
/// IntPatch_Intersection calls it; and the extension of two walking lines towards their common point. The comparison
/// with native OCCT is through GeomAPI_IntSS (the oracle).
/// </summary>
public class IntPatch_WLineToolTests
{
    private const double Tol = 1.0e-7;

    private static void AssertOnBoth(WLine theWL, SurfaceAdaptorBase theS1, SurfaceAdaptorBase theS2,
                                     double theTol)
    {
        for (int k = 1; k <= theWL.NbPnts(); k++)
        {
            PntOn2S aP = theWL.Point(k);
            aP.Parameters(out double u1, out double v1, out double u2, out double v2);
            Assert.True(theS1.Value(u1, v1).DistanceTo(aP.Value()) < theTol, $"point {k} off S1");
            Assert.True(theS2.Value(u2, v2).DistanceTo(aP.Value()) < theTol, $"point {k} off S2");
        }
        for (int k = 1; k <= theWL.VertexCount; k++)
        {
            PatchPoint aV = theWL.Vertex(k);
            int anIdx = (int)aV.ParameterOnLine();
            Assert.True(anIdx >= 1 && anIdx <= theWL.NbPnts(), $"vertex {k} off the line");
            Assert.True(theWL.Point(anIdx).Value().DistanceTo(aV.Value()) < theTol, $"vertex {k} not at its point");
        }
    }

    [Fact]
    public void ComputePurgedWLine_on_BSpline_lines_keeps_ends_and_surfaces()
    {
        var aS1 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchA());
        var aS2 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchB());
        var aD1 = new TopolTool(aS1);
        var aD2 = new TopolTool(aS2);
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS1, aD1, aS2, aD2, Tol, Tol, 0.01, 0.05);
        Assert.True(anInt.LineCount > 0);
        int aNbPurged = 0;
        for (int i = 1; i <= anInt.LineCount; i++)
        {
            var aWL = (WLine)anInt.Line(i);
            int aNbBefore = aWL.NbPnts();
            Point3D aFirst = aWL.Point(1).Value(), aLast = aWL.Point(aNbBefore).Value();
            WLine? aRW = WLineTool.ComputePurgedWLine(aWL, aS1, aS2, aD1, aD2);
            Assert.NotNull(aRW);
            Assert.Equal(aNbBefore, aWL.NbPnts()); // the given line is unchanged
            Assert.True(aRW.NbPnts() <= aNbBefore);
            Assert.Equal(aWL.VertexCount, aRW.VertexCount);
            Assert.Equal(aWL.GetCreatingWay(), aRW.GetCreatingWay());
            Assert.Equal(aFirst, aRW.Point(1).Value());
            Assert.Equal(aLast, aRW.Point(aRW.NbPnts()).Value());
            AssertOnBoth(aRW, aS1, aS2, 1.0e-6);
            if (aRW.NbPnts() < aNbBefore)
            {
                aNbPurged++;
            }
        }
        Assert.True(aNbPurged > 0, "the tube criterion removes points of the B-spline lines");
    }

    [Fact]
    public void ComputePurgedWLine_on_ImpPrm_line()
    {
        // A plane against a patch, as ImpPrmIntersection walks it.
        var aS1 = new SurfaceAdaptor(new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0.1), new Direction3D(0.1, 0.2, 1))));
        var aS2 = new SurfaceAdaptor(IntPatchPrmPrmNativeTests.PatchA());
        var aD1 = new TopolTool(aS1);
        var aD2 = new TopolTool(aS2);
        var anInt = new ImpPrmIntersection(aS1, aD1, aS2, aD2, Tol, Tol, 0.01, 0.05);
        Assert.True(anInt.IsDone);
        int aNbW = 0;
        for (int i = 1; i <= anInt.LineCount; i++)
        {
            if (anInt.Line(i) is not WLine aWL)
            {
                continue;
            }
            aNbW++;
            WLine? aRW = WLineTool.ComputePurgedWLine(aWL, aS1, aS2, aD1, aD2);
            Assert.NotNull(aRW);
            Assert.Equal(aWL.Point(1).Value(), aRW.Point(1).Value());
            Assert.Equal(aWL.Point(aWL.NbPnts()).Value(), aRW.Point(aRW.NbPnts()).Value());
            AssertOnBoth(aRW, aS1, aS2, 1.0e-6);
        }
        Assert.True(aNbW > 0);
    }

    private static PntOn2S Pnt(double x)
    {
        var aP = new PntOn2S();
        aP.SetValue(new Point3D(x, 0, 0), x, 0, x, 1);
        return aP;
    }

    private static PatchPoint Vtx(double x, double theParam)
    {
        var aV = new PatchPoint();
        aV.SetValue(new Point3D(x, 0, 0), 1e-7, false);
        aV.SetParameters(x, 0, x, 1);
        aV.SetParameter(theParam);
        return aV;
    }

    [Fact]
    public void ComputePurgedWLine_removes_equal_points_and_renumbers_vertices()
    {
        // Points 0, 1, 1, 2, 3 (a duplicate at index 3): fewer than 30 points, so only the equal points go.
        var aL = new LineOn2S();
        foreach (double x in new[] { 0.0, 1.0, 1.0, 2.0, 3.0 })
        {
            aL.Add(Pnt(x));
        }
        var aW = new WLine(aL, false);
        aW.AddVertex(Vtx(0, 1));
        aW.AddVertex(Vtx(2, 4));
        aW.AddVertex(Vtx(3, 5));
        var aS = new SurfaceAdaptor(new PlaneSurface(new CoordinateSystem()));
        var aD = new TopolTool(aS);
        WLine? aRW = WLineTool.ComputePurgedWLine(aW, aS, aS, aD, aD);
        Assert.NotNull(aRW);
        Assert.Equal(4, aRW.NbPnts());
        Assert.Equal(5, aW.NbPnts());
        Assert.Equal(new[] { 1.0, 3.0, 4.0 },
                     Enumerable.Range(1, 3).Select(k => aRW.Vertex(k).ParameterOnLine()).ToArray());
    }

    [Fact]
    public void ComputePurgedWLine_drops_degenerate_lines()
    {
        var aS = new SurfaceAdaptor(new PlaneSurface(new CoordinateSystem()));
        var aD = new TopolTool(aS);

        var aL2 = new LineOn2S();
        aL2.Add(Pnt(1));
        aL2.Add(Pnt(1));
        Assert.Null(WLineTool.ComputePurgedWLine(new WLine(aL2, false), aS, aS, aD, aD));

        // Three equal points: one is left, so the line is dropped.
        var aL3 = new LineOn2S();
        aL3.Add(Pnt(1));
        aL3.Add(Pnt(1));
        aL3.Add(Pnt(1));
        Assert.Null(WLineTool.ComputePurgedWLine(new WLine(aL3, false), aS, aS, aD, aD));

        // Two distinct points are kept as they are.
        var aL4 = new LineOn2S();
        aL4.Add(Pnt(0));
        aL4.Add(Pnt(1));
        WLine? aRW = WLineTool.ComputePurgedWLine(new WLine(aL4, false), aS, aS, aD, aD);
        Assert.NotNull(aRW);
        Assert.Equal(2, aRW.NbPnts());
    }

    private static CoordinateSystem Ax(double x, double y, double z, double dx, double dy, double dz, double xx, double xy,
                             double xz) =>
        new(new Point3D(x, y, z), new Direction3D(dx, dy, dz), new Direction3D(xx, xy, xz));

    // IntPatch_Intersection's GeomGeomPerfom on two cylinders: the ImpImp lines and points, then JoinWLines.
    private static (NCollection_Sequence<PatchLine>, SurfaceAdaptor, SurfaceAdaptor) JoinCylinders(
        Surface theC1, Surface theC2)
    {
        var aS1 = new SurfaceAdaptor(theC1);
        var aS2 = new SurfaceAdaptor(theC2);
        var anInter = new ImpImpIntersection(aS1, new TopolTool(aS1), aS2,
                                                      new TopolTool(aS2), Tol, Tol);
        Assert.True(anInter.IsDone);
        var aSLin = new NCollection_Sequence<PatchLine>();
        var aSPnt = new NCollection_Sequence<PatchPoint>();
        for (int i = 1; i <= anInter.LineCount; i++)
        {
            PatchLine aL = anInter.Line(i);
            if (aL is WLine aWL)
            {
                aWL.EnablePurging(false);
            }
            if (aL.ArcType() != PatchLineType.Restriction)
            {
                aSLin.Append(aL);
            }
        }
        for (int i = 1; i <= anInter.PntCount; i++)
        {
            aSPnt.Append(anInter.Point(i));
        }
        WLineTool.JoinWLines(aSLin, aSPnt, aS1, aS2, Tol);
        return (aSLin, aS1, aS2);
    }

    // lowalgos/intss/bug23178 and bug28222_2: the eight walking lines of ImpImp become GeomInt's six and four.
    [Fact]
    public void JoinWLines_bug23178_and_bug28222_2()
    {
        var (a23178, aS1, aS2) = JoinCylinders(
            new CylindricalSurface(Ax(4.999999999813, 1.35836143386791, 0.20975890778078, -1, 0, 0, 0, 0, 1), 10),
            new CylindricalSurface(Ax(9.999999999813, 9.1401960568287, 0.11837300583107, 0, -0.75870713028692,
                                           0.651431877061437, 0, -0.651431877061437, -0.75870713028692), 5));
        Assert.Equal(6, a23178.Length());
        for (int i = 1; i <= a23178.Length(); i++)
        {
            AssertOnBoth((WLine)a23178.Value(i), aS1, aS2, 1.0e-6);
        }

        var (a28222, aT1, aT2) = JoinCylinders(
            new CylindricalSurface(Ax(-35, 13.3706576198285, 30.5814570420266, 0, -0.258819045102521,
                                           -0.965925826289068, 0, 0.965925826289068, -0.258819045102521), 11),
            new CylindricalSurface(Ax(0, 0, 0, 1, 0, 0, 0, 0, -1), 16.5));
        Assert.Equal(4, a28222.Length());
        for (int i = 1; i <= a28222.Length(); i++)
        {
            AssertOnBoth((WLine)a28222.Value(i), aT1, aT2, 1.0e-6);
        }
    }

    [Fact]
    public void JoinWLines_is_deterministic()
    {
        string Run()
        {
            var (aSLin, _, _) = JoinCylinders(
                new CylindricalSurface(Ax(4.999999999813, 1.35836143386791, 0.20975890778078, -1, 0, 0, 0, 0, 1),
                                            10),
                new CylindricalSurface(Ax(9.999999999813, 9.1401960568287, 0.11837300583107, 0, -0.75870713028692,
                                               0.651431877061437, 0, -0.651431877061437, -0.75870713028692), 5));
            var aSb = new System.Text.StringBuilder();
            for (int i = 1; i <= aSLin.Length(); i++)
            {
                var aWL = (WLine)aSLin.Value(i);
                for (int k = 1; k <= aWL.NbPnts(); k++)
                {
                    aSb.Append(aWL.Point(k).ToString()).Append(';');
                }
                aSb.Append('|');
            }
            return aSb.ToString();
        }
        string aRef = Run();
        string[] aPar = new string[4];
        Parallel.For(0, aPar.Length, i => aPar[i] = Run());
        Assert.All(aPar, s => Assert.Equal(aRef, s));
    }

    // A plane (z = 0) and a cylinder (axis Z, radius 1) meet in the unit circle; parameters on the plane are (x, y),
    // on the cylinder (t, 0).
    private static PntOn2S CirclePnt(double t)
    {
        var aP = new PntOn2S();
        aP.SetValue(new Point3D(Math.Cos(t), Math.Sin(t), 0), Math.Cos(t), Math.Sin(t), t, 0);
        return aP;
    }

    private static WLine CircleLine(double t0, double t1, int theNb)
    {
        var aL = new LineOn2S();
        for (int i = 0; i < theNb; i++)
        {
            aL.Add(CirclePnt(t0 + (t1 - t0) * i / (theNb - 1)));
        }
        var aW = new WLine(aL, false);
        foreach ((double t, int k) in new[] { (t0, 1), (t1, theNb) })
        {
            var aV = new PatchPoint();
            aV.SetValue(CirclePnt(t));
            aV.SetParameter(k);
            aW.AddVertex(aV);
        }
        return aW;
    }

    private static (SurfaceAdaptor, SurfaceAdaptor, Box2d, Box2d, double[]) PlaneCylinder()
    {
        var aS1 = new SurfaceAdaptor(new PlaneSurface(new CoordinateSystem()));
        var aS2 = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(), 1.0));
        var aB1 = new Box2d();
        aB1.Update(-2, -2, 2, 2);
        var aB2 = new Box2d();
        aB2.Update(0, -1, 2 * Math.PI, 1);
        return (aS1, aS2, aB1, aB2, new[] { 0.0, 0.0, 2 * Math.PI, 0.0 });
    }

    [Fact]
    public void ExtendTwoWLines_joins_lines_across_a_small_gap()
    {
        var (aS1, aS2, aB1, aB2, aPer) = PlaneCylinder();
        WLine aW1 = CircleLine(0.2, 1.0, 9);
        WLine aW2 = CircleLine(1.0 + 1.0e-4, 2.0, 11);
        var aSLin = new NCollection_Sequence<PatchLine>();
        aSLin.Append(aW1);
        aSLin.Append(aW2);
        WLineTool.ExtendTwoWLines(aSLin, aS1, aS2, Tol, aPer, aB1, aB2, new NCollection_List<Point3D>());
        Assert.Equal(1, aSLin.Length());
        var aW = (WLine)aSLin.Value(1);
        Assert.Same(aW1, aW);
        // 9 + 11 points and the common point (the mid point of the gap, added to both lines' ends)
        Assert.Equal(21, aW.NbPnts());
        AssertOnBoth(aW, aS1, aS2, 1.0e-6);
        Assert.Equal(0.2, aW.Point(1).ParametersOnS2Tuple().Item1, 15);
        Assert.Equal(2.0, aW.Point(aW.NbPnts()).ParametersOnS2Tuple().Item1, 15);
        Assert.Equal(1.0, aW.Vertex(1).ParameterOnLine());
        Assert.Equal(aW.NbPnts(), aW.Vertex(aW.VertexCount).ParameterOnLine());
    }

    [Fact]
    public void ExtendTwoWLines_skips_critical_points_and_wide_angles()
    {
        var (aS1, aS2, aB1, aB2, aPer) = PlaneCylinder();
        var aSLin = new NCollection_Sequence<PatchLine>();
        aSLin.Append(CircleLine(0.2, 1.0, 9));
        aSLin.Append(CircleLine(1.0 + 1.0e-4, 2.0, 11));
        var aCrit = new NCollection_List<Point3D>();
        aCrit.Append(CirclePnt(1.0).Value());
        WLineTool.ExtendTwoWLines(aSLin, aS1, aS2, Tol, aPer, aB1, aB2, aCrit);
        Assert.Equal(2, aSLin.Length());
        Assert.Equal(9, ((WLine)aSLin.Value(1)).NbPnts());

        // A gap too wide for the chord's mid point to be on both surfaces: nothing changes.
        var aWide = new NCollection_Sequence<PatchLine>();
        aWide.Append(CircleLine(0.2, 1.0, 9));
        aWide.Append(CircleLine(1.05, 2.0, 11));
        WLineTool.ExtendTwoWLines(aWide, aS1, aS2, Tol, aPer, aB1, aB2, new NCollection_List<Point3D>());
        Assert.Equal(2, aWide.Length());
        Assert.Equal(9, ((WLine)aWide.Value(1)).NbPnts());
        Assert.Equal(11, ((WLine)aWide.Value(2)).NbPnts());
    }
}

internal static class IntSurfPntOn2STestExtensions
{
    internal static (double, double) ParametersOnS2Tuple(this PntOn2S theP)
    {
        theP.ParametersOnS2(out double u, out double v);
        return (u, v);
    }
}
