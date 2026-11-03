// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_Orientation;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Unit tests of ShapeFix_WireSegment (unit 6H2), ShapeFix_ComposeShell's data storage.</summary>
public class ShapeFix_WireSegmentTests
{
    private static ShapeExtend_WireData Square(bool theClosed)
    {
        var aPoly = new BRepBuilderAPI_MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0),
                                                   new Point3D(0, 1, 0), theClosed);
        return new ShapeExtend_WireData(aPoly.Wire());
    }

    [Fact]
    public void Empty_IsForwardWithNoEdges()
    {
        var aSeg = new ShapeFix_WireSegment();
        Assert.Equal(TopAbs_FORWARD, aSeg.Orientation());
        Assert.Equal(0, aSeg.NbEdges());
        Assert.False(aSeg.WireData().ManifoldMode());
        Assert.False(aSeg.IsVertex());
    }

    [Fact]
    public void FromWireData_CopiesTheEdges_WithIndefinitePatchIndices()
    {
        ShapeExtend_WireData sbwd = Square(true);
        var aSeg = new ShapeFix_WireSegment(sbwd);
        Assert.Equal(TopAbs_EXTERNAL, aSeg.Orientation());
        Assert.Equal(4, aSeg.NbEdges());
        Assert.False(ReferenceEquals(aSeg.WireData(), sbwd));
        Assert.True(aSeg.Edge(2).IsSame(sbwd.Edge(2)));
        Assert.True(aSeg.IsClosed());
        Assert.False(new ShapeFix_WireSegment(Square(false), TopAbs_REVERSED).IsClosed());

        aSeg.GetPatchIndex(1, out int iumin, out int iumax, out int ivmin, out int ivmax);
        Assert.Equal((-32000, 32000, -32000, 32000), (iumin, iumax, ivmin, ivmax));
        Assert.False(aSeg.CheckPatchIndex(1));
    }

    [Fact]
    public void PatchIndices_AreDefinedAndChecked()
    {
        var aSeg = new ShapeFix_WireSegment(Square(true));
        aSeg.DefineIUMin(1, 2);
        aSeg.DefineIUMax(1, 3);
        aSeg.DefineIVMin(1, -1);
        aSeg.DefineIVMax(1, -1);
        aSeg.DefineIUMin(1, 1); // only raises the minimum
        aSeg.GetPatchIndex(1, out int iumin, out int iumax, out int ivmin, out int ivmax);
        Assert.Equal((2, 3, -1, -1), (iumin, iumax, ivmin, ivmax));
        Assert.True(aSeg.CheckPatchIndex(1));

        aSeg.SetPatchIndex(2, 0, 2, 0, 0);
        Assert.False(aSeg.CheckPatchIndex(2));

        TopoDS_Edge anExtra = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(0, 0, 1)).Edge();
        aSeg.AddEdge(0, anExtra, 5, 5, 6, 6);
        aSeg.AddEdge(1, anExtra);
        Assert.Equal(6, aSeg.NbEdges());
        aSeg.GetPatchIndex(6, out iumin, out _, out ivmin, out _);
        Assert.Equal((5, 6), (iumin, ivmin));
        aSeg.GetPatchIndex(2, out iumin, out iumax, out ivmin, out ivmax);
        Assert.Equal((2, 3, -1, -1), (iumin, iumax, ivmin, ivmax));
    }

    [Fact]
    public void Copies_ShareTheWireButNotTheOrientationOrVertex()
    {
        var aSeg = new ShapeFix_WireSegment(Square(true), TopAbs_FORWARD);
        ShapeFix_WireSegment aCopy = aSeg;
        aCopy.Orientation(TopAbs_INTERNAL);
        aCopy.SetVertex(new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 0)).Vertex());
        aCopy.SetEdge(1, aSeg.Edge(2));
        Assert.Equal(TopAbs_FORWARD, aSeg.Orientation());
        Assert.False(aSeg.IsVertex());
        Assert.True(aCopy.IsVertex());
        Assert.True(aSeg.Edge(1).IsSame(aSeg.Edge(2))); // the wire data is shared, as OCCT's handle
    }
}
