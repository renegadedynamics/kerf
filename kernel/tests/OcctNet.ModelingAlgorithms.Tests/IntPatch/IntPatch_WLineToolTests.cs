// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntPatch_IType;

namespace OcctNet.ModelingAlgorithms.Tests;

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

    private static void AssertOnBoth(IntPatch_WLine theWL, Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                     double theTol)
    {
        for (int k = 1; k <= theWL.NbPnts(); k++)
        {
            IntSurf_PntOn2S aP = theWL.Point(k);
            aP.Parameters(out double u1, out double v1, out double u2, out double v2);
            Assert.True(theS1.Value(u1, v1).Distance(aP.Value()) < theTol, $"point {k} off S1");
            Assert.True(theS2.Value(u2, v2).Distance(aP.Value()) < theTol, $"point {k} off S2");
        }
        for (int k = 1; k <= theWL.NbVertex(); k++)
        {
            IntPatch_Point aV = theWL.Vertex(k);
            int anIdx = (int)aV.ParameterOnLine();
            Assert.True(anIdx >= 1 && anIdx <= theWL.NbPnts(), $"vertex {k} off the line");
            Assert.True(theWL.Point(anIdx).Value().Distance(aV.Value()) < theTol, $"vertex {k} not at its point");
        }
    }

    [Fact]
    public void ComputePurgedWLine_on_BSpline_lines_keeps_ends_and_surfaces()
    {
        var aS1 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchA());
        var aS2 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchB());
        var aD1 = new Adaptor3d_TopolTool(aS1);
        var aD2 = new Adaptor3d_TopolTool(aS2);
        var anInt = new IntPatch_PrmPrmIntersection();
        anInt.Perform(aS1, aD1, aS2, aD2, Tol, Tol, 0.01, 0.05);
        Assert.True(anInt.NbLines() > 0);
        int aNbPurged = 0;
        for (int i = 1; i <= anInt.NbLines(); i++)
        {
            var aWL = (IntPatch_WLine)anInt.Line(i);
            int aNbBefore = aWL.NbPnts();
            gp_Pnt aFirst = aWL.Point(1).Value(), aLast = aWL.Point(aNbBefore).Value();
            IntPatch_WLine? aRW = IntPatch_WLineTool.ComputePurgedWLine(aWL, aS1, aS2, aD1, aD2);
            Assert.NotNull(aRW);
            Assert.Equal(aNbBefore, aWL.NbPnts()); // the given line is unchanged
            Assert.True(aRW.NbPnts() <= aNbBefore);
            Assert.Equal(aWL.NbVertex(), aRW.NbVertex());
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
        var aS1 = new GeomAdaptor_Surface(new Geom_Plane(new gp_Ax3(new gp_Pnt(0, 0, 0.1), new gp_Dir(0.1, 0.2, 1))));
        var aS2 = new GeomAdaptor_Surface(IntPatchPrmPrmNativeTests.PatchA());
        var aD1 = new Adaptor3d_TopolTool(aS1);
        var aD2 = new Adaptor3d_TopolTool(aS2);
        var anInt = new IntPatch_ImpPrmIntersection(aS1, aD1, aS2, aD2, Tol, Tol, 0.01, 0.05);
        Assert.True(anInt.IsDone());
        int aNbW = 0;
        for (int i = 1; i <= anInt.NbLines(); i++)
        {
            if (anInt.Line(i) is not IntPatch_WLine aWL)
            {
                continue;
            }
            aNbW++;
            IntPatch_WLine? aRW = IntPatch_WLineTool.ComputePurgedWLine(aWL, aS1, aS2, aD1, aD2);
            Assert.NotNull(aRW);
            Assert.Equal(aWL.Point(1).Value(), aRW.Point(1).Value());
            Assert.Equal(aWL.Point(aWL.NbPnts()).Value(), aRW.Point(aRW.NbPnts()).Value());
            AssertOnBoth(aRW, aS1, aS2, 1.0e-6);
        }
        Assert.True(aNbW > 0);
    }

    private static IntSurf_PntOn2S Pnt(double x)
    {
        var aP = new IntSurf_PntOn2S();
        aP.SetValue(new gp_Pnt(x, 0, 0), x, 0, x, 1);
        return aP;
    }

    private static IntPatch_Point Vtx(double x, double theParam)
    {
        var aV = new IntPatch_Point();
        aV.SetValue(new gp_Pnt(x, 0, 0), 1e-7, false);
        aV.SetParameters(x, 0, x, 1);
        aV.SetParameter(theParam);
        return aV;
    }

    [Fact]
    public void ComputePurgedWLine_removes_equal_points_and_renumbers_vertices()
    {
        // Points 0, 1, 1, 2, 3 (a duplicate at index 3): fewer than 30 points, so only the equal points go.
        var aL = new IntSurf_LineOn2S();
        foreach (double x in new[] { 0.0, 1.0, 1.0, 2.0, 3.0 })
        {
            aL.Add(Pnt(x));
        }
        var aW = new IntPatch_WLine(aL, false);
        aW.AddVertex(Vtx(0, 1));
        aW.AddVertex(Vtx(2, 4));
        aW.AddVertex(Vtx(3, 5));
        var aS = new GeomAdaptor_Surface(new Geom_Plane(new gp_Ax3()));
        var aD = new Adaptor3d_TopolTool(aS);
        IntPatch_WLine? aRW = IntPatch_WLineTool.ComputePurgedWLine(aW, aS, aS, aD, aD);
        Assert.NotNull(aRW);
        Assert.Equal(4, aRW.NbPnts());
        Assert.Equal(5, aW.NbPnts());
        Assert.Equal(new[] { 1.0, 3.0, 4.0 },
                     Enumerable.Range(1, 3).Select(k => aRW.Vertex(k).ParameterOnLine()).ToArray());
    }

    [Fact]
    public void ComputePurgedWLine_drops_degenerate_lines()
    {
        var aS = new GeomAdaptor_Surface(new Geom_Plane(new gp_Ax3()));
        var aD = new Adaptor3d_TopolTool(aS);

        var aL2 = new IntSurf_LineOn2S();
        aL2.Add(Pnt(1));
        aL2.Add(Pnt(1));
        Assert.Null(IntPatch_WLineTool.ComputePurgedWLine(new IntPatch_WLine(aL2, false), aS, aS, aD, aD));

        // Three equal points: one is left, so the line is dropped.
        var aL3 = new IntSurf_LineOn2S();
        aL3.Add(Pnt(1));
        aL3.Add(Pnt(1));
        aL3.Add(Pnt(1));
        Assert.Null(IntPatch_WLineTool.ComputePurgedWLine(new IntPatch_WLine(aL3, false), aS, aS, aD, aD));

        // Two distinct points are kept as they are.
        var aL4 = new IntSurf_LineOn2S();
        aL4.Add(Pnt(0));
        aL4.Add(Pnt(1));
        IntPatch_WLine? aRW = IntPatch_WLineTool.ComputePurgedWLine(new IntPatch_WLine(aL4, false), aS, aS, aD, aD);
        Assert.NotNull(aRW);
        Assert.Equal(2, aRW.NbPnts());
    }

    private static gp_Ax3 Ax(double x, double y, double z, double dx, double dy, double dz, double xx, double xy,
                             double xz) =>
        new(new gp_Pnt(x, y, z), new gp_Dir(dx, dy, dz), new gp_Dir(xx, xy, xz));

    // IntPatch_Intersection's GeomGeomPerfom on two cylinders: the ImpImp lines and points, then JoinWLines.
    private static (NCollection_Sequence<IntPatch_Line>, GeomAdaptor_Surface, GeomAdaptor_Surface) JoinCylinders(
        Geom_Surface theC1, Geom_Surface theC2)
    {
        var aS1 = new GeomAdaptor_Surface(theC1);
        var aS2 = new GeomAdaptor_Surface(theC2);
        var anInter = new IntPatch_ImpImpIntersection(aS1, new Adaptor3d_TopolTool(aS1), aS2,
                                                      new Adaptor3d_TopolTool(aS2), Tol, Tol);
        Assert.True(anInter.IsDone());
        var aSLin = new NCollection_Sequence<IntPatch_Line>();
        var aSPnt = new NCollection_Sequence<IntPatch_Point>();
        for (int i = 1; i <= anInter.NbLines(); i++)
        {
            IntPatch_Line aL = anInter.Line(i);
            if (aL is IntPatch_WLine aWL)
            {
                aWL.EnablePurging(false);
            }
            if (aL.ArcType() != IntPatch_Restriction)
            {
                aSLin.Append(aL);
            }
        }
        for (int i = 1; i <= anInter.NbPnts(); i++)
        {
            aSPnt.Append(anInter.Point(i));
        }
        IntPatch_WLineTool.JoinWLines(aSLin, aSPnt, aS1, aS2, Tol);
        return (aSLin, aS1, aS2);
    }

    // lowalgos/intss/bug23178 and bug28222_2: the eight walking lines of ImpImp become GeomInt's six and four.
    [Fact]
    public void JoinWLines_bug23178_and_bug28222_2()
    {
        var (a23178, aS1, aS2) = JoinCylinders(
            new Geom_CylindricalSurface(Ax(4.999999999813, 1.35836143386791, 0.20975890778078, -1, 0, 0, 0, 0, 1), 10),
            new Geom_CylindricalSurface(Ax(9.999999999813, 9.1401960568287, 0.11837300583107, 0, -0.75870713028692,
                                           0.651431877061437, 0, -0.651431877061437, -0.75870713028692), 5));
        Assert.Equal(6, a23178.Length());
        for (int i = 1; i <= a23178.Length(); i++)
        {
            AssertOnBoth((IntPatch_WLine)a23178.Value(i), aS1, aS2, 1.0e-6);
        }

        var (a28222, aT1, aT2) = JoinCylinders(
            new Geom_CylindricalSurface(Ax(-35, 13.3706576198285, 30.5814570420266, 0, -0.258819045102521,
                                           -0.965925826289068, 0, 0.965925826289068, -0.258819045102521), 11),
            new Geom_CylindricalSurface(Ax(0, 0, 0, 1, 0, 0, 0, 0, -1), 16.5));
        Assert.Equal(4, a28222.Length());
        for (int i = 1; i <= a28222.Length(); i++)
        {
            AssertOnBoth((IntPatch_WLine)a28222.Value(i), aT1, aT2, 1.0e-6);
        }
    }

    [Fact]
    public void JoinWLines_is_deterministic()
    {
        string Run()
        {
            var (aSLin, _, _) = JoinCylinders(
                new Geom_CylindricalSurface(Ax(4.999999999813, 1.35836143386791, 0.20975890778078, -1, 0, 0, 0, 0, 1),
                                            10),
                new Geom_CylindricalSurface(Ax(9.999999999813, 9.1401960568287, 0.11837300583107, 0, -0.75870713028692,
                                               0.651431877061437, 0, -0.651431877061437, -0.75870713028692), 5));
            var aSb = new System.Text.StringBuilder();
            for (int i = 1; i <= aSLin.Length(); i++)
            {
                var aWL = (IntPatch_WLine)aSLin.Value(i);
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
    private static IntSurf_PntOn2S CirclePnt(double t)
    {
        var aP = new IntSurf_PntOn2S();
        aP.SetValue(new gp_Pnt(Math.Cos(t), Math.Sin(t), 0), Math.Cos(t), Math.Sin(t), t, 0);
        return aP;
    }

    private static IntPatch_WLine CircleLine(double t0, double t1, int theNb)
    {
        var aL = new IntSurf_LineOn2S();
        for (int i = 0; i < theNb; i++)
        {
            aL.Add(CirclePnt(t0 + (t1 - t0) * i / (theNb - 1)));
        }
        var aW = new IntPatch_WLine(aL, false);
        foreach ((double t, int k) in new[] { (t0, 1), (t1, theNb) })
        {
            var aV = new IntPatch_Point();
            aV.SetValue(CirclePnt(t));
            aV.SetParameter(k);
            aW.AddVertex(aV);
        }
        return aW;
    }

    private static (GeomAdaptor_Surface, GeomAdaptor_Surface, Bnd_Box2d, Bnd_Box2d, double[]) PlaneCylinder()
    {
        var aS1 = new GeomAdaptor_Surface(new Geom_Plane(new gp_Ax3()));
        var aS2 = new GeomAdaptor_Surface(new Geom_CylindricalSurface(new gp_Ax3(), 1.0));
        var aB1 = new Bnd_Box2d();
        aB1.Update(-2, -2, 2, 2);
        var aB2 = new Bnd_Box2d();
        aB2.Update(0, -1, 2 * Math.PI, 1);
        return (aS1, aS2, aB1, aB2, new[] { 0.0, 0.0, 2 * Math.PI, 0.0 });
    }

    [Fact]
    public void ExtendTwoWLines_joins_lines_across_a_small_gap()
    {
        var (aS1, aS2, aB1, aB2, aPer) = PlaneCylinder();
        IntPatch_WLine aW1 = CircleLine(0.2, 1.0, 9);
        IntPatch_WLine aW2 = CircleLine(1.0 + 1.0e-4, 2.0, 11);
        var aSLin = new NCollection_Sequence<IntPatch_Line>();
        aSLin.Append(aW1);
        aSLin.Append(aW2);
        IntPatch_WLineTool.ExtendTwoWLines(aSLin, aS1, aS2, Tol, aPer, aB1, aB2, new NCollection_List<gp_Pnt>());
        Assert.Equal(1, aSLin.Length());
        var aW = (IntPatch_WLine)aSLin.Value(1);
        Assert.Same(aW1, aW);
        // 9 + 11 points and the common point (the mid point of the gap, added to both lines' ends)
        Assert.Equal(21, aW.NbPnts());
        AssertOnBoth(aW, aS1, aS2, 1.0e-6);
        Assert.Equal(0.2, aW.Point(1).ParametersOnS2Tuple().Item1, 15);
        Assert.Equal(2.0, aW.Point(aW.NbPnts()).ParametersOnS2Tuple().Item1, 15);
        Assert.Equal(1.0, aW.Vertex(1).ParameterOnLine());
        Assert.Equal(aW.NbPnts(), aW.Vertex(aW.NbVertex()).ParameterOnLine());
    }

    [Fact]
    public void ExtendTwoWLines_skips_critical_points_and_wide_angles()
    {
        var (aS1, aS2, aB1, aB2, aPer) = PlaneCylinder();
        var aSLin = new NCollection_Sequence<IntPatch_Line>();
        aSLin.Append(CircleLine(0.2, 1.0, 9));
        aSLin.Append(CircleLine(1.0 + 1.0e-4, 2.0, 11));
        var aCrit = new NCollection_List<gp_Pnt>();
        aCrit.Append(CirclePnt(1.0).Value());
        IntPatch_WLineTool.ExtendTwoWLines(aSLin, aS1, aS2, Tol, aPer, aB1, aB2, aCrit);
        Assert.Equal(2, aSLin.Length());
        Assert.Equal(9, ((IntPatch_WLine)aSLin.Value(1)).NbPnts());

        // A gap too wide for the chord's mid point to be on both surfaces: nothing changes.
        var aWide = new NCollection_Sequence<IntPatch_Line>();
        aWide.Append(CircleLine(0.2, 1.0, 9));
        aWide.Append(CircleLine(1.05, 2.0, 11));
        IntPatch_WLineTool.ExtendTwoWLines(aWide, aS1, aS2, Tol, aPer, aB1, aB2, new NCollection_List<gp_Pnt>());
        Assert.Equal(2, aWide.Length());
        Assert.Equal(9, ((IntPatch_WLine)aWide.Value(1)).NbPnts());
        Assert.Equal(11, ((IntPatch_WLine)aWide.Value(2)).NbPnts());
    }
}

internal static class IntSurfPntOn2STestExtensions
{
    internal static (double, double) ParametersOnS2Tuple(this IntSurf_PntOn2S theP)
    {
        theP.ParametersOnS2(out double u, out double v);
        return (u, v);
    }
}
