// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Bento.Collections;
using static Occt.BRepFill_ThruSectionErrorStatus;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepFill_CompatibleWires on hand-made sections (native parity is in BRepFill_CompatibleWiresNativeTests): the
/// polar method on closed sections with different edge counts, the reordering of closed sections with as many edges,
/// the curvilinear-abscissa method on open sections, point sections, mixed sections, the generated edges, and frozen
/// inputs never written.
/// </summary>
public class BRepFill_CompatibleWiresTests
{
    private static TopoDS_Wire Polygon(double z, double theAngle, double theRadius, int theNb, bool theClosed)
    {
        BRepBuilderAPI_MakePolygon aP = new();
        for (int i = 0; i < theNb; i++)
        {
            double t = theClosed ? theAngle + 2 * Math.PI * i / theNb : theAngle + Math.PI * i / (theNb - 1);
            aP.Add(new gp_Pnt(theRadius * Math.Cos(t), theRadius * Math.Sin(t), z));
        }
        if (theClosed)
        {
            aP.Close();
        }
        return aP.Wire();
    }

    private static TopoDS_Wire PointSection(in gp_Pnt theP)
    {
        BRep_Builder B = new();
        TopoDS_Vertex aV = new();
        B.MakeVertex(ref aV, theP, 1.0e-7);
        TopoDS_Edge anE = new();
        B.MakeEdge(ref anE);
        B.Add(anE, aV.Oriented(TopAbs_Orientation.TopAbs_FORWARD));
        B.Add(anE, aV.Oriented(TopAbs_Orientation.TopAbs_REVERSED));
        B.Degenerated(anE, true);
        TopoDS_Wire aW = new();
        B.MakeWire(ref aW);
        B.Add(aW, anE);
        aW.Closed(true);
        aW.TShape()!.FreezeGraph();
        return aW;
    }

    private static int NbEdges(in TopoDS_Shape theW)
    {
        int n = 0;
        for (BRepTools_WireExplorer anExp = new(TopoDS.Wire(theW)); anExp.More(); anExp.Next())
        {
            n++;
        }
        return n;
    }

    private static string Text(in TopoDS_Shape theShape)
    {
        using StringWriter aW = new(CultureInfo.InvariantCulture);
        BRepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static NCollection_Sequence<TopoDS_Shape> Seq(params TopoDS_Shape[] theShapes)
    {
        NCollection_Sequence<TopoDS_Shape> aSeq = new();
        foreach (TopoDS_Shape aS in theShapes)
        {
            aSeq.Append(aS);
        }
        return aSeq;
    }

    [Fact]
    public void New_algorithm_is_not_done()
    {
        BRepFill_CompatibleWires aCW = new();
        Assert.False(aCW.IsDone());
        Assert.Equal(BRepFill_ThruSectionErrorStatus_NotDone, aCW.GetStatus());
        Assert.Empty(aCW.Shape());
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Closed_sections_with_different_edge_counts_get_as_many_edges(bool theWithRotation)
    {
        TopoDS_Wire aSquare = Polygon(0, 0.3, 2, 4, true), aTriangle = Polygon(3, 0.1, 1.5, 3, true), aHexagon = Polygon(6, 0.0, 2, 6, true);
        BRepFill_CompatibleWires aCW = new(Seq(aSquare, aTriangle, aHexagon));
        aCW.Perform(theWithRotation);
        Assert.True(aCW.IsDone());
        Assert.False(aCW.IsDegeneratedFirstSection());
        Assert.False(aCW.IsDegeneratedLastSection());
        IBentoList<TopoDS_Shape> aRes = aCW.Shape();
        Assert.Equal(3, aRes.Count);
        int aNb = NbEdges(aRes[0]);
        Assert.True(aNb >= 6);
        Assert.All(aRes, w => Assert.Equal(aNb, NbEdges(w)));
        Assert.All(aRes, w => Assert.True(w.TShape()!.IsFrozen()));

        // every input edge generates edges of its own section's result, which cover it
        for (int s = 0; s < 3; s++)
        {
            TopoDS_Shape anIn = new[] { aSquare, aTriangle, aHexagon }[s];
            NCollection_IndexedMap<TopoDS_Shape> aResEdges = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aRes[s], TopAbs_ShapeEnum.TopAbs_EDGE, aResEdges);
            int aTotal = 0;
            for (TopExp_Explorer anExp = new(anIn, TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                IBentoList<TopoDS_Shape> aGen = aCW.GeneratedShapes(TopoDS.Edge(anExp.Current()));
                Assert.NotEmpty(aGen);
                Assert.All(aGen, e => Assert.True(aResEdges.Contains(e)));
                aTotal += aGen.Count;
            }
            Assert.Equal(aNb, aTotal);
        }
        Assert.Equal(13, aCW.Generated().Count);
    }

    [Fact]
    public void Closed_sections_with_as_many_edges_are_reordered_not_cut()
    {
        // same count and only C0 joints: ComputeOrigin chooses origin and direction, no edge is cut
        TopoDS_Wire aW1 = Polygon(0, 0.0, 2, 5, true);
        TopoDS_Wire aW2 = Polygon(3, 2 * Math.PI * 2 / 5 + 0.05, 2, 5, true);
        aW2 = TopoDS.Wire(aW2.Reversed());
        BRepFill_CompatibleWires aCW = new(Seq(aW1, aW2));
        aCW.Perform();
        Assert.True(aCW.IsDone());
        IBentoList<TopoDS_Shape> aRes = aCW.Shape();
        Assert.Equal(5, NbEdges(aRes[1]));
        // the result's edges are the caller's edges
        NCollection_IndexedMap<TopoDS_Shape> anIn = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aW2, TopAbs_ShapeEnum.TopAbs_EDGE, anIn);
        for (TopExp_Explorer anExp = new(aRes[1], TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(anIn.Contains(anExp.Current()));
        }
        // origins match: the second wire starts at the vertex above the first's start, and runs the same way
        BRepTools_WireExplorer e1 = new(TopoDS.Wire(aRes[0])), e2 = new(TopoDS.Wire(aRes[1]));
        gp_Pnt p1 = BRep_Tool.Pnt(e1.CurrentVertex()), p2 = BRep_Tool.Pnt(e2.CurrentVertex());
        Assert.True(Math.Abs(p1.X() - p2.X()) < 0.2 && Math.Abs(p1.Y() - p2.Y()) < 0.2);
        e1.Next();
        e2.Next();
        p1 = BRep_Tool.Pnt(e1.CurrentVertex());
        p2 = BRep_Tool.Pnt(e2.CurrentVertex());
        Assert.True(Math.Abs(p1.X() - p2.X()) < 0.2 && Math.Abs(p1.Y() - p2.Y()) < 0.2);
    }

    [Fact]
    public void Open_sections_are_cut_by_curvilinear_abscissa_and_oriented()
    {
        TopoDS_Wire aW1 = Polygon(0, 0.0, 2, 3, false);
        TopoDS_Wire aW2 = Polygon(3, 0.0, 2, 4, false);
        aW2 = TopoDS.Wire(aW2.Reversed());
        BRepFill_CompatibleWires aCW = new(Seq(aW1, aW2));
        aCW.SetPercent(0.05);
        aCW.Perform();
        Assert.True(aCW.IsDone());
        IBentoList<TopoDS_Shape> aRes = aCW.Shape();
        Assert.Equal(NbEdges(aRes[0]), NbEdges(aRes[1]));
        Assert.True(NbEdges(aRes[0]) >= 3);
        TopExp.Vertices(TopoDS.Wire(aRes[0]), out TopoDS_Vertex a0, out _);
        TopExp.Vertices(TopoDS.Wire(aRes[1]), out TopoDS_Vertex b0, out _);
        gp_Pnt pa = BRep_Tool.Pnt(a0), pb = BRep_Tool.Pnt(b0);
        Assert.True(Math.Abs(pa.X() - pb.X()) < 1e-9 && Math.Abs(pa.Y() - pb.Y()) < 1e-9);
    }

    [Theory]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void Point_sections_are_flagged(bool theFirst, bool theLast)
    {
        List<TopoDS_Shape> aSecs = [];
        if (theFirst)
        {
            aSecs.Add(PointSection(new gp_Pnt(0, 0, -3)));
        }
        aSecs.Add(Polygon(0, 0.0, 2, 4, true));
        aSecs.Add(Polygon(3, 0.2, 2, 3, true));
        if (theLast)
        {
            aSecs.Add(PointSection(new gp_Pnt(0, 0, 6)));
        }
        BRepFill_CompatibleWires aCW = new(Seq([.. aSecs]));
        aCW.Perform();
        Assert.True(aCW.IsDone());
        Assert.Equal(theFirst, aCW.IsDegeneratedFirstSection());
        Assert.Equal(theLast, aCW.IsDegeneratedLastSection());
    }

    [Fact]
    public void Open_and_closed_sections_are_not_processed()
    {
        BRepFill_CompatibleWires aCW = new(Seq(Polygon(0, 0, 2, 4, true), Polygon(3, 0, 2, 4, false)));
        aCW.Perform();
        Assert.False(aCW.IsDone());
        Assert.Equal(BRepFill_ThruSectionErrorStatus_NotSameTopology, aCW.GetStatus());
    }

    [Fact]
    public void SetPercent_takes_only_values_between_0_and_1()
    {
        // the percent only decides whether an intersection near a vertex snaps to it; out-of-range values are ignored,
        // so the results equal the default's (0.1)
        TopoDS_Wire aW1 = Polygon(0, 0.0, 2, 4, true), aW2 = Polygon(3, 0.7, 2, 3, true);
        string Result(double? thePct)
        {
            BRepFill_CompatibleWires aCW = new(Seq(aW1, aW2));
            if (thePct is double p)
            {
                aCW.SetPercent(p);
            }
            aCW.Perform();
            return string.Join("|", aCW.Shape().Select(w => NbEdges(w)));
        }
        Assert.Equal(Result(null), Result(1.5));
        Assert.Equal(Result(null), Result(0.0));
        Assert.Equal(Result(null), Result(-1));
    }

    [Fact]
    public void Frozen_inputs_are_never_written_and_uncut_edges_are_shared()
    {
        TopoDS_Wire aW1 = Polygon(0, 0.0, 2, 4, true), aW2 = Polygon(3, 0.4, 2, 3, true);
        Assert.True(aW1.TShape()!.IsFrozen());
        string t1 = Text(aW1), t2 = Text(aW2);
        BRepFill_CompatibleWires aCW = new(Seq(aW1, aW2));
        aCW.Perform();
        Assert.True(aCW.IsDone());
        Assert.Equal(t1, Text(aW1));
        Assert.Equal(t2, Text(aW2));
        // the 4-edge wire's edges were not cut (the triangle gets cut): the result holds the caller's edges
        NCollection_IndexedMap<TopoDS_Shape> anIn = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aW1, TopAbs_ShapeEnum.TopAbs_EDGE, anIn);
        int aShared = 0;
        for (TopExp_Explorer anExp = new(aCW.Shape()[0], TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (anIn.Contains(anExp.Current()))
            {
                aShared++;
            }
        }
        Assert.True(aShared > 0);
        // the ported callers' forms hand out the algorithm's own containers
        Assert.Same(aCW.InternalShape(), aCW.InternalShape());
        Assert.Same(aCW.InternalGenerated(), aCW.InternalGenerated());
    }
}
