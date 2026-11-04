// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>The restriction line: its arcs and parameters, its optional polygon, and ComputeVertexParameters.</summary>
public class IntPatch_RLineTests
{
    private static Curve2DAdaptorBase Arc(double y) =>
        new CurveAdaptor2D(new LineCurve2D(new Point2D(0, y), new Direction2D(1, 0)), -100, 100);

    private static PatchPoint OnArc(Curve2DAdaptorBase theArc, bool theOnFirst, double theParamOnLine,
                                        double theParamOnArc)
    {
        PatchPoint aV = IntPatch_WLineTests.Vtx(theParamOnLine, theParamOnLine);
        aV.SetArc(theOnFirst, theArc, theParamOnArc, new WalkingTransition(false, WalkingTypeTrans.In),
                  new WalkingTransition(false, WalkingTypeTrans.Out));
        return aV;
    }

    [Fact]
    public void Construction_arcs_and_polygon()
    {
        RLine aR = new(false, WalkingSituation.Inside, WalkingSituation.Outside);
        Assert.Equal(PatchLineType.Restriction, aR.ArcType());
        Assert.Equal(WalkingTypeTrans.Touch, aR.TransitionOnS2());
        Assert.Equal(WalkingSituation.Outside, aR.SituationS2());
        Assert.False(aR.IsArcOnS1 || aR.IsArcOnS2);
        Assert.False(aR.HasPolygon);
        EngineAssert.Throws<DomainErrorException>(() => aR.NbPnts());
        EngineAssert.Throws<DomainErrorException>(() => aR.Point(1));
        Assert.Null(aR.Curve());

        LineOn2S aCurve = IntPatch_WLineTests.StraightLine(3);
        aR.Add(aCurve);
        Assert.True(aR.HasPolygon);
        Assert.Same(aCurve, aR.Curve());
        Assert.Equal(3, aR.NbPnts());
        LineOn2S aOther = IntPatch_WLineTests.StraightLine(2);
        aR.SetCurve(aOther);
        Assert.Equal(2, aR.NbPnts());

        Curve2DAdaptorBase anArc = Arc(0);
        aR.SetArcOnS1(anArc);
        Assert.True(aR.IsArcOnS1);
        Assert.Same(anArc, aR.ArcOnS1());
        Assert.Null(aR.ArcOnS2());
    }

    [Fact]
    public void ParamOnS1_spans_the_vertices_only_on_an_arc_of_S1()
    {
        RLine aR = new(false);
        aR.AddVertex(IntPatch_WLineTests.Vtx(0, 2.5));
        aR.AddVertex(IntPatch_WLineTests.Vtx(0, -1.0));
        aR.AddVertex(IntPatch_WLineTests.Vtx(0, 4.0), true);
        aR.ParamOnS1(out double a, out double b);
        Assert.Equal((0.0, 0.0), (a, b));
        aR.SetArcOnS1(Arc(0));
        aR.ParamOnS1(out a, out b);
        Assert.Equal((-1.0, 4.0), (a, b));
        aR.ParamOnS2(out a, out b);
        Assert.Equal((0.0, 0.0), (a, b));
    }

    [Fact]
    public void ComputeVertexParameters_removes_duplicates_sorts_and_sets_the_ends()
    {
        Curve2DAdaptorBase anArc = Arc(0), anOther = Arc(1);
        RLine aR = new(false);
        aR.AddVertex(OnArc(anArc, true, 3, 0.7));
        aR.AddVertex(OnArc(anArc, true, 3, 0.7)); // same arc, parameters: removed
        aR.AddVertex(OnArc(anOther, true, 3, 0.7)); // other arc: kept
        aR.AddVertex(OnArc(anArc, false, 1, 0.1));
        aR.AddVertex(IntPatch_WLineTests.Vtx(0, 1)); // on no arc at the parameter of another: removed
        aR.AddVertex(IntPatch_WLineTests.Vtx(0, 2));
        aR.ComputeVertexParameters(0.0);

        Assert.Equal(new[] { 1.0, 2.0, 3.0, 3.0 },
                     Enumerable.Range(1, aR.VertexCount).Select(i => aR.Vertex(i).ParameterOnLine()));
        Assert.True(aR.Vertex(1).IsOnDomS2);
        Assert.Same(anArc, aR.Vertex(3).ArcOnS1());
        Assert.Same(anOther, aR.Vertex(4).ArcOnS1());
        Assert.True(aR.HasFirstPoint && aR.HasLastPoint);
        Assert.Equal(1.0, aR.FirstPoint().ParameterOnLine());
        Assert.Same(anOther, aR.LastPoint().ArcOnS1());

        EngineAssert.Throws<OutOfRangeException>(() => aR.RemoveVertex(5));
        aR.ChangeVertex(2).SetParameter(2.5);
        Assert.Equal(2.5, aR.Vertex(2).ParameterOnLine());
        aR.Replace(2, IntPatch_WLineTests.Vtx(0, 9));
        Assert.Equal(9.0, aR.Vertex(2).ParameterOnLine());
    }

    [Fact]
    public void Dump_needs_the_polygon()
    {
        RLine aR = new(false);
        EngineAssert.Throws<DomainErrorException>(() => aR.Dump(0, new StringWriter()));
        aR.Add(IntPatch_WLineTests.StraightLine(2));
        StringWriter aOut = new();
        aR.Dump(2, aOut);
        Assert.StartsWith(" ----------- D u m p    I n t P a t c h  _  R L i n e  -(begin)------\n", aOut.ToString());
        Assert.Contains("point p2 +1.00000000000000000000 +0.00000000000000000000\n", aOut.ToString());
    }
}
