// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dHatch_Elements_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's Geom2dHatch_Elements GTest (the suite is named after the class, hence the suffix).</summary>
public class Geom2dHatch_ElementsTest
{
    private static Geom2dHatch_Element makeCircleElement()
    {
        Axis2D anAxis = new(new Point2D(0, 0), new Direction2D(1, 0));
        CircleCurve2D aCircle = new(anAxis, 1.0);
        CurveAdaptor2D aCurve = new(aCircle);
        Geom2dHatch_Element anElem = new(aCurve, Forward);
        return anElem;
    }

    // Regression test for bug #38: CurrentEdge in const context must work
    // without void* const-cast hack. Verifies that edge traversal returns
    // valid curve and orientation after Bind + InitWires/InitEdges.
    [Fact]
    public void CurrentEdge_ReturnsValidData()
    {
        Geom2dHatch_Elements anElements = new();
        anElements.Bind(1, makeCircleElement());

        anElements.InitWires();
        Assert.True(anElements.MoreWires());

        anElements.InitEdges();
        Assert.True(anElements.MoreEdges());

        CurveAdaptor2D aEdge = new();
        Orientation anOr = Internal;
        anElements.CurrentEdge(ref aEdge, out anOr);

        Assert.Equal(Forward, anOr);
        // Circle has parameter range [0, 2*PI].
        Assert.Equal(0.0, aEdge.FirstParameter(), 1e-10);
        Assert.True(aEdge.LastParameter() > 6.0);
    }

    // Bind/Find/IsBound work correctly.
    [Fact]
    public void BindAndFind()
    {
        Geom2dHatch_Elements anElements = new();
        Assert.False(anElements.IsBound(1));

        anElements.Bind(1, makeCircleElement());
        Assert.True(anElements.IsBound(1));

        Geom2dHatch_Element anElem = anElements.Find(1);
        Assert.Equal(Forward, anElem.Orientation());
    }

    // Clear removes all elements.
    [Fact]
    public void Clear_RemovesAll()
    {
        Geom2dHatch_Elements anElements = new();
        anElements.Bind(1, makeCircleElement());
        anElements.Bind(2, makeCircleElement());
        Assert.True(anElements.IsBound(1));

        anElements.Clear();
        Assert.False(anElements.IsBound(1));
        Assert.False(anElements.IsBound(2));
    }
}
