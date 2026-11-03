// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using Schematiq.Collections;
using static Kerf.Engine.Construction.Filling.ThruSectionErrorStatus;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepFill_CompatibleWires on hand-made sections (native parity is in BRepFill_CompatibleWiresNativeTests): the
/// polar method on closed sections with different edge counts, the reordering of closed sections with as many edges,
/// the curvilinear-abscissa method on open sections, point sections, mixed sections, the generated edges, and frozen
/// inputs never written.
/// </summary>
public class BRepFill_CompatibleWiresTests
{
    private static Wire Polygon(double z, double theAngle, double theRadius, int theNb, bool theClosed)
    {
        MakePolygon aP = new();
        for (int i = 0; i < theNb; i++)
        {
            double t = theClosed ? theAngle + 2 * Math.PI * i / theNb : theAngle + Math.PI * i / (theNb - 1);
            aP.Add(new Point3D(theRadius * Math.Cos(t), theRadius * Math.Sin(t), z));
        }
        if (theClosed)
        {
            aP.Close();
        }
        return aP.Wire();
    }

    private static Wire PointSection(in Point3D theP)
    {
        BrepBuilder B = new();
        Vertex aV = new();
        B.MakeVertex(ref aV, theP, 1.0e-7);
        Edge anE = new();
        B.MakeEdge(ref anE);
        B.Add(anE, aV.Oriented(Orientation.Forward));
        B.Add(anE, aV.Oriented(Orientation.Reversed));
        B.Degenerated(anE, true);
        Wire aW = new();
        B.MakeWire(ref aW);
        B.Add(aW, anE);
        aW.Closed(true);
        aW.TShape()!.FreezeGraph();
        return aW;
    }

    private static int NbEdges(in Shape theW)
    {
        int n = 0;
        for (WireExplorer anExp = new(ShapeCasts.Wire(theW)); anExp.More(); anExp.Next())
        {
            n++;
        }
        return n;
    }

    private static string Text(in Shape theShape)
    {
        using StringWriter aW = new(CultureInfo.InvariantCulture);
        BrepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static NCollection_Sequence<Shape> Seq(params Shape[] theShapes)
    {
        NCollection_Sequence<Shape> aSeq = new();
        foreach (Shape aS in theShapes)
        {
            aSeq.Append(aS);
        }
        return aSeq;
    }

    [Fact]
    public void New_algorithm_is_not_done()
    {
        CompatibleWires aCW = new();
        Assert.False(aCW.IsDone);
        Assert.Equal(ThruSectionErrorStatus.ThruSectionErrorStatus_NotDone, aCW.GetStatus());
        Assert.Empty(aCW.Shape());
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Closed_sections_with_different_edge_counts_get_as_many_edges(bool theWithRotation)
    {
        Wire aSquare = Polygon(0, 0.3, 2, 4, true), aTriangle = Polygon(3, 0.1, 1.5, 3, true), aHexagon = Polygon(6, 0.0, 2, 6, true);
        CompatibleWires aCW = new(Seq(aSquare, aTriangle, aHexagon));
        aCW.Perform(theWithRotation);
        Assert.True(aCW.IsDone);
        Assert.False(aCW.IsDegeneratedFirstSection);
        Assert.False(aCW.IsDegeneratedLastSection);
        ISchematiqList<Shape> aRes = aCW.Shape();
        Assert.Equal(3, aRes.Count);
        int aNb = NbEdges(aRes[0]);
        Assert.True(aNb >= 6);
        Assert.All(aRes, w => Assert.Equal(aNb, NbEdges(w)));
        Assert.All(aRes, w => Assert.True(w.TShape()!.IsFrozen));

        // every input edge generates edges of its own section's result, which cover it
        for (int s = 0; s < 3; s++)
        {
            Shape anIn = new[] { aSquare, aTriangle, aHexagon }[s];
            NCollection_IndexedMap<Shape> aResEdges = new(ShapeHasher.Instance);
            Exploration.MapShapes(aRes[s], ShapeType.Edge, aResEdges);
            int aTotal = 0;
            for (Explorer anExp = new(anIn, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                ISchematiqList<Shape> aGen = aCW.GeneratedShapes(ShapeCasts.Edge(anExp.Current()));
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
        Wire aW1 = Polygon(0, 0.0, 2, 5, true);
        Wire aW2 = Polygon(3, 2 * Math.PI * 2 / 5 + 0.05, 2, 5, true);
        aW2 = ShapeCasts.Wire(aW2.Reversed());
        CompatibleWires aCW = new(Seq(aW1, aW2));
        aCW.Perform();
        Assert.True(aCW.IsDone);
        ISchematiqList<Shape> aRes = aCW.Shape();
        Assert.Equal(5, NbEdges(aRes[1]));
        // the result's edges are the caller's edges
        NCollection_IndexedMap<Shape> anIn = new(ShapeHasher.Instance);
        Exploration.MapShapes(aW2, ShapeType.Edge, anIn);
        for (Explorer anExp = new(aRes[1], ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(anIn.Contains(anExp.Current()));
        }
        // origins match: the second wire starts at the vertex above the first's start, and runs the same way
        WireExplorer e1 = new(ShapeCasts.Wire(aRes[0])), e2 = new(ShapeCasts.Wire(aRes[1]));
        Point3D p1 = Tool.Pnt(e1.CurrentVertex()), p2 = Tool.Pnt(e2.CurrentVertex());
        Assert.True(Math.Abs(p1.X() - p2.X()) < 0.2 && Math.Abs(p1.Y() - p2.Y()) < 0.2);
        e1.Next();
        e2.Next();
        p1 = Tool.Pnt(e1.CurrentVertex());
        p2 = Tool.Pnt(e2.CurrentVertex());
        Assert.True(Math.Abs(p1.X() - p2.X()) < 0.2 && Math.Abs(p1.Y() - p2.Y()) < 0.2);
    }

    [Fact]
    public void Open_sections_are_cut_by_curvilinear_abscissa_and_oriented()
    {
        Wire aW1 = Polygon(0, 0.0, 2, 3, false);
        Wire aW2 = Polygon(3, 0.0, 2, 4, false);
        aW2 = ShapeCasts.Wire(aW2.Reversed());
        CompatibleWires aCW = new(Seq(aW1, aW2));
        aCW.SetPercent(0.05);
        aCW.Perform();
        Assert.True(aCW.IsDone);
        ISchematiqList<Shape> aRes = aCW.Shape();
        Assert.Equal(NbEdges(aRes[0]), NbEdges(aRes[1]));
        Assert.True(NbEdges(aRes[0]) >= 3);
        Exploration.Vertices(ShapeCasts.Wire(aRes[0]), out Vertex a0, out _);
        Exploration.Vertices(ShapeCasts.Wire(aRes[1]), out Vertex b0, out _);
        Point3D pa = Tool.Pnt(a0), pb = Tool.Pnt(b0);
        Assert.True(Math.Abs(pa.X() - pb.X()) < 1e-9 && Math.Abs(pa.Y() - pb.Y()) < 1e-9);
    }

    [Theory]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void Point_sections_are_flagged(bool theFirst, bool theLast)
    {
        List<Shape> aSecs = [];
        if (theFirst)
        {
            aSecs.Add(PointSection(new Point3D(0, 0, -3)));
        }
        aSecs.Add(Polygon(0, 0.0, 2, 4, true));
        aSecs.Add(Polygon(3, 0.2, 2, 3, true));
        if (theLast)
        {
            aSecs.Add(PointSection(new Point3D(0, 0, 6)));
        }
        CompatibleWires aCW = new(Seq([.. aSecs]));
        aCW.Perform();
        Assert.True(aCW.IsDone);
        Assert.Equal(theFirst, aCW.IsDegeneratedFirstSection);
        Assert.Equal(theLast, aCW.IsDegeneratedLastSection);
    }

    [Fact]
    public void Open_and_closed_sections_are_not_processed()
    {
        CompatibleWires aCW = new(Seq(Polygon(0, 0, 2, 4, true), Polygon(3, 0, 2, 4, false)));
        aCW.Perform();
        Assert.False(aCW.IsDone);
        Assert.Equal(ThruSectionErrorStatus.ThruSectionErrorStatus_NotSameTopology, aCW.GetStatus());
    }

    [Fact]
    public void SetPercent_takes_only_values_between_0_and_1()
    {
        // the percent only decides whether an intersection near a vertex snaps to it; out-of-range values are ignored,
        // so the results equal the default's (0.1)
        Wire aW1 = Polygon(0, 0.0, 2, 4, true), aW2 = Polygon(3, 0.7, 2, 3, true);
        string Result(double? thePct)
        {
            CompatibleWires aCW = new(Seq(aW1, aW2));
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
        Wire aW1 = Polygon(0, 0.0, 2, 4, true), aW2 = Polygon(3, 0.4, 2, 3, true);
        Assert.True(aW1.TShape()!.IsFrozen);
        string t1 = Text(aW1), t2 = Text(aW2);
        CompatibleWires aCW = new(Seq(aW1, aW2));
        aCW.Perform();
        Assert.True(aCW.IsDone);
        Assert.Equal(t1, Text(aW1));
        Assert.Equal(t2, Text(aW2));
        // the 4-edge wire's edges were not cut (the triangle gets cut): the result holds the caller's edges
        NCollection_IndexedMap<Shape> anIn = new(ShapeHasher.Instance);
        Exploration.MapShapes(aW1, ShapeType.Edge, anIn);
        int aShared = 0;
        for (Explorer anExp = new(aCW.Shape()[0], ShapeType.Edge); anExp.More(); anExp.Next())
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
