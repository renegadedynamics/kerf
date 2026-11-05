// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Sweeping;
using Kerf.Engine.HiddenLines.Presentation;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>The sweep's numbered shapes, and a drawer's own settings over its link (question 49's coverage).</summary>
public class SweepNumShapeTests
{
    [Fact]
    public void A_numbered_shape_compares_and_orients()
    {
        var edge = new NumShape(2, ShapeType.Edge);
        Assert.True(edge == new NumShape(2, ShapeType.Edge));
        Assert.True(edge != new NumShape(1, ShapeType.Edge));
        Assert.True(edge.Equals((object)new NumShape(2, ShapeType.Edge)));
        Assert.False(edge.Equals("other"));
        Assert.Equal(edge.GetHashCode(), new NumShape(2, ShapeType.Edge).GetHashCode());
        Assert.Equal(Orientation.Forward, edge.Orientation());
        Assert.Equal(Orientation.Forward, new NumShape(2, ShapeType.Vertex).Orientation());
        Assert.Equal(Orientation.Reversed, new NumShape(1, ShapeType.Vertex).Orientation());
    }

    [Fact]
    public void A_numbered_edge_has_the_vertices_its_ends_allow()
    {
        var tool = new NumShapeTool(new NumShape(1, ShapeType.Edge));
        Assert.True(tool.HasFirstVertex());
        Assert.True(tool.HasLastVertex());
        Assert.Equal(ShapeType.Vertex, tool.FirstVertex().Type());
        Assert.Equal(ShapeType.Vertex, tool.LastVertex().Type());
        Assert.Equal(ShapeType.Edge, tool.Type(new NumShape(1, ShapeType.Edge)));
        Assert.Equal(Orientation.Forward, tool.Orientation(new NumShape(1, ShapeType.Edge)));

        var open = new NumShapeTool(new NumShape(1, ShapeType.Edge, closed: false, begInf: true, endInf: true));
        Assert.False(open.HasFirstVertex());
        Assert.False(open.HasLastVertex());
    }

    [Fact]
    public void A_drawers_own_setting_gives_way_to_its_link_when_unset()
    {
        var link = new Drawer();
        link.SetMaximalChordialDeviation(0.5);
        var drawer = new Drawer();
        drawer.SetLink(link);
        Assert.Same(link, drawer.Link());
        drawer.SetMaximalChordialDeviation(0.25);
        Assert.Equal(0.25, drawer.MaximalChordialDeviation());
        drawer.UnsetOwnMaximalChordialDeviation();
        Assert.Equal(0.5, drawer.MaximalChordialDeviation());
        drawer.SetAutoTriangulation(false);
        drawer.UnsetOwnIsAutoTriangulation();
        Assert.Equal(link.IsAutoTriangulation, drawer.IsAutoTriangulation);
    }
}
