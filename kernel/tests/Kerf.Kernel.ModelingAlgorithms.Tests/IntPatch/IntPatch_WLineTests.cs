// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The walking-line container against OCCT's semantics: the vertex list (add, prepend, insert, remove, replace),
/// the points (SetPoint), and ComputeVertexParameters on hand-built straight lines, traced through the C++.
/// </summary>
public class IntPatch_WLineTests
{
    // A straight line along X: point k is (x0 + k - 1, 0, 0), (x0 + k - 1, 0) on S1, (x0 + k - 1, 1) on S2.
    internal static LineOn2S StraightLine(int theNb, double theX0 = 0.0)
    {
        LineOn2S aLine = new();
        for (int k = 1; k <= theNb; k++)
        {
            double x = theX0 + k - 1;
            PntOn2S aP = new();
            aP.SetValue(new Point3D(x, 0, 0), x, 0, x, 1);
            aLine.Add(aP);
        }
        return aLine;
    }

    internal static PatchPoint Vtx(double x, double theParam, double theTol = 1e-7)
    {
        PatchPoint aV = new();
        aV.SetValue(new Point3D(x, 0, 0), theTol, false);
        aV.SetParameters(x, 0, x, 1);
        aV.SetParameter(theParam);
        return aV;
    }

    private static Curve2DAdaptorBase Arc() =>
        new CurveAdaptor2D(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), -100, 100);

    [Fact]
    public void Construction_and_flags()
    {
        WLine aW = new(StraightLine(3), false, WalkingTypeTrans.In, WalkingTypeTrans.Out);
        Assert.Equal(PatchLineType.Walking, aW.ArcType());
        Assert.Equal(WalkingTypeTrans.In, aW.TransitionOnS1());
        Assert.Equal(3, aW.NbPnts());
        Assert.False(aW.HasFirstPoint);
        Assert.False(aW.HasLastPoint);
        EngineAssert.Throws<DomainErrorException>(() => aW.FirstPoint());
        Assert.True(aW.IsPurgingAllowed);
        aW.EnablePurging(false);
        Assert.False(aW.IsPurgingAllowed);
        Assert.Equal(WLine.IntPatch_WLType.WLUnknown, aW.GetCreatingWay());
        aW.SetCreatingWayInfo(WLine.IntPatch_WLType.WLPrmPrm);
        Assert.Equal(WLine.IntPatch_WLType.WLPrmPrm, aW.GetCreatingWay());

        aW.SetPeriod(1, 2, 3, 4);
        Assert.Equal((1.0, 2.0, 3.0, 4.0), (aW.U1Period(), aW.V1Period(), aW.U2Period(), aW.V2Period()));

        Assert.False(aW.HasArcOnS1);
        Curve2DAdaptorBase anArc = Arc();
        aW.SetArcOnS2(anArc);
        Assert.True(aW.HasArcOnS2);
        Assert.Same(anArc, aW.GetArcOnS2());
        Assert.Null(aW.GetArcOnS1());
    }

    [Fact]
    public void Vertex_list_add_prepend_insert_remove_replace()
    {
        WLine aW = new(StraightLine(5), false);
        aW.AddVertex(Vtx(1, 2));
        aW.AddVertex(Vtx(0, 1), true);
        aW.InsertVertexBefore(10, Vtx(4, 5)); // past the end: appended
        aW.InsertVertexBefore(3, Vtx(2, 3));
        Assert.Equal(new[] { 1.0, 2.0, 3.0, 5.0 },
                     Enumerable.Range(1, aW.VertexCount).Select(i => aW.Vertex(i).ParameterOnLine()));

        EngineAssert.Throws<OutOfRangeException>(() => aW.RemoveVertex(0));
        EngineAssert.Throws<OutOfRangeException>(() => aW.RemoveVertex(5));
        aW.RemoveVertex(2);
        Assert.Equal(3, aW.VertexCount);

        aW.Replace(1, Vtx(0.5, 1.5));
        Assert.Equal(1.5, aW.Vertex(1).ParameterOnLine());

        // ChangeVertex is a reference to the stored vertex; Vertex is a copy.
        aW.ChangeVertex(1).SetTolerance(0.25);
        Assert.Equal(0.25, aW.Vertex(1).Tolerance());
        PatchPoint aCopy = aW.Vertex(1);
        aCopy.SetTolerance(0.5);
        Assert.Equal(0.25, aW.Vertex(1).Tolerance());

        aW.SetFirstPoint(1);
        aW.SetLastPoint(3);
        Assert.Equal(1.5, aW.FirstPoint(out int anIndF).ParameterOnLine());
        Assert.Equal(1, anIndF);
        Assert.Equal(5.0, aW.LastPoint().ParameterOnLine());

        aW.ClearVertexes();
        Assert.Equal(0, aW.VertexCount);
    }

    [Fact]
    public void SetPoint_writes_the_vertex_into_the_line()
    {
        LineOn2S aCurve = StraightLine(4);
        WLine aW = new(aCurve, false);
        PatchPoint aV = Vtx(7, 2);
        aW.SetPoint(2, aV);
        Assert.Same(aCurve, aW.Curve());
        Assert.Equal(aV.PntOn2S(), aW.Point(2));
        Assert.Equal(7.0, aCurve.Value(2).Value().X);
        Assert.True(aW.IsOutBox(new Point3D(100, 0, 0)));
        Assert.False(aW.IsOutSurf1Box(new Point2D(2, 0)));
    }

    [Fact]
    public void ComputeVertexParameters_substitutes_inserts_and_adds_the_end_vertices()
    {
        WLine aW = new(StraightLine(5), false);
        aW.AddVertex(Vtx(3.5, 4.5)); // between points 4 and 5: inserted
        aW.AddVertex(Vtx(2, 3)); // on point 3: substituted
        aW.ComputeVertexParameters(1e-6);

        Assert.Equal(6, aW.NbPnts());
        Assert.Equal(new Point3D(3.5, 0, 0), aW.Point(5).Value());
        Assert.Equal(new[] { 1.0, 3.0, 5.0, 6.0 },
                     Enumerable.Range(1, aW.VertexCount).Select(i => aW.Vertex(i).ParameterOnLine()));
        Assert.True(aW.HasFirstPoint && aW.HasLastPoint);
        Assert.Equal(new Point3D(0, 0, 0), aW.FirstPoint().Value());
        Assert.Equal(new Point3D(4, 0, 0), aW.LastPoint(out int anIndL).Value());
        Assert.Equal(4, anIndL);
        // The end vertices are made with the largest of the given tolerance and the vertices' ones.
        Assert.Equal(1e-6, aW.FirstPoint().Tolerance());
        aW.FirstPoint().Parameters(out double u1, out double v1, out double u2, out double v2);
        Assert.Equal((0.0, 0.0, 0.0, 1.0), (u1, v1, u2, v2));
    }

    [Fact]
    public void ComputeVertexParameters_removes_coincident_points_and_renumbers_the_vertices()
    {
        LineOn2S aCurve = StraightLine(5);
        aCurve.InsertBefore(3, aCurve.Value(2)); // (0,1,1,2,3,4) along X
        WLine aW = new(aCurve, false);
        aW.AddVertex(Vtx(2, 4));
        aW.ComputeVertexParameters(1e-7);

        Assert.Equal(5, aW.NbPnts());
        Assert.Equal(new[] { 1.0, 3.0, 5.0 },
                     Enumerable.Range(1, aW.VertexCount).Select(i => aW.Vertex(i).ParameterOnLine()));
    }

    [Fact]
    public void ComputeVertexParameters_merges_a_free_vertex_into_an_arc_vertex()
    {
        Curve2DAdaptorBase anArc = Arc();
        WLine aW = new(StraightLine(4), false);
        PatchPoint aOnArc = Vtx(1, 2, 1e-3);
        aOnArc.SetArc(true, anArc, 0.5, new WalkingTransition(false, WalkingTypeTrans.In),
                      new WalkingTransition(false, WalkingTypeTrans.Out));
        aW.AddVertex(aOnArc);
        aW.AddVertex(Vtx(1, 2.5, 1e-3));
        aW.ComputeVertexParameters(1e-7);

        // Both land on point 2; the vertex on no arc is removed; then the end vertices are added.
        Assert.Equal(3, aW.VertexCount);
        Assert.Equal(4, aW.NbPnts());
        PatchPoint aV = aW.Vertex(2);
        Assert.Equal(2.0, aV.ParameterOnLine());
        Assert.True(aV.IsOnDomS1);
        Assert.Same(anArc, aV.ArcOnS1());
        Assert.Equal(WalkingTypeTrans.In, aV.TransitionLineArc1().TransitionType());
    }

    [Fact]
    public void ComputeVertexParameters_reproduces_OCCTs_unshifted_vertex_before_the_line()
    {
        // OCCT: when a vertex before the line is inserted at its start, the other vertices are shifted unless their
        // index is 1 ("j != 1", meant "j != i"): the first vertex keeps parameter 1 while its point moves to 2.
        WLine aW = new(StraightLine(5), false);
        aW.AddVertex(Vtx(-2, 0.2));
        aW.AddVertex(Vtx(-1, 0.5));
        aW.ComputeVertexParameters(1e-7);

        Assert.Equal(7, aW.NbPnts());
        Assert.Equal(new Point3D(-1, 0, 0), aW.Point(1).Value());
        Assert.Equal(new Point3D(-2, 0, 0), aW.Point(2).Value());
        Assert.Equal(2, aW.VertexCount);
        Assert.Equal(1.0, aW.Vertex(1).ParameterOnLine());
        Assert.Equal(new Point3D(-2, 0, 0), aW.Vertex(1).Value());
        Assert.Equal(7.0, aW.Vertex(2).ParameterOnLine());
    }

    [Fact]
    public void ComputeVertexParameters_brings_a_vertex_into_the_period_of_its_point()
    {
        LineOn2S aCurve = StraightLine(3);
        WLine aW = new(aCurve, false);
        aW.SetPeriod(10, 0, 0, 0);
        PatchPoint aV = new();
        aV.SetValue(new Point3D(1, 0, 0), 1e-7, false);
        aV.SetParameters(11, 0, 1, 1); // U1 one period away
        aV.SetParameter(2);
        aW.AddVertex(aV);
        aW.ComputeVertexParameters(1e-7);

        aW.Point(2).Parameters(out double u1, out _, out _, out _);
        Assert.Equal(1.0, u1);
        aW.Vertex(2).Parameters(out double vu1, out _, out _, out _);
        Assert.Equal(1.0, vu1);
    }

    [Fact]
    public void Dump_has_OCCTs_layout()
    {
        WLine aW = new(StraightLine(2), false);
        aW.AddVertex(Vtx(0, 1));
        StringWriter aOut = new();
        aW.Dump(0, aOut);
        string[] aLines = aOut.ToString().Split('\n');
        Assert.Equal(" ----------- D u m p    I n t P a t c h  _  W L i n e  -(begin)------", aLines[0]);
        Assert.Equal("Num    [X  Y  Z]     [U1  V1]   [U2  V2]", aLines[1]);
        Assert.Equal("   1  [+0.00000000000000000000 +0.00000000000000000000 +0.00000000000000000000]  "
                     + "[+0.00000000000000000000 +0.00000000000000000000]  [+0.00000000000000000000 +1.00000000000000000000]",
                     aLines[2]);
        Assert.Contains("----------- IntPatch_Point : ", aOut.ToString());
        Assert.Contains("----> IntSurf_PntOn2S : 1, Pnt (0,0,0)", aOut.ToString());
        Assert.Contains("Toler = 1e-07", aOut.ToString());
        Assert.EndsWith("\n--------------------------------------------------- (end) -------\n", aOut.ToString());

        StringWriter aOut1 = new();
        aW.Dump(1, aOut1);
        Assert.Contains("point p2 +1.00000000000000000000 +0.00000000000000000000 +0.00000000000000000000\n",
                        aOut1.ToString());
        StringWriter aOut3 = new();
        aW.Dump(3, aOut3);
        Assert.Contains("point p2 +1.00000000000000000000 +1.00000000000000000000\n", aOut3.ToString());
    }
}
