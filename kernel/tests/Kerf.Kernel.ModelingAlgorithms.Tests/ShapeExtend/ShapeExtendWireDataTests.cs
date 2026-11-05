// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Known answers of ShapeExtend_WireData: loading wires, editing the list, seams, and making wires.</summary>
public class ShapeExtendWireDataTests
{
    private static Face FirstFace(in Shape theShape, Func<Face, bool>? thePredicate = null)
    {
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Face aF = ShapeCasts.Face(anExp.Current());
            if (thePredicate is null || thePredicate(aF))
            {
                return aF;
            }
        }
        throw new InvalidOperationException("no face");
    }

    private static bool IsPlanar(Face theF) => Tool.Surface(theF) is PlaneSurface;

    // the four sides of the unit square, sharing their vertices: s[i] runs from vertex i to vertex i + 1
    private static Edge[] SquareEdges()
    {
        Vertex[] aV =
        [
            TestShapes.Vertex(new Point3D(0, 0, 0)), TestShapes.Vertex(new Point3D(1, 0, 0)),
            TestShapes.Vertex(new Point3D(1, 1, 0)), TestShapes.Vertex(new Point3D(0, 1, 0)),
        ];
        return [TestShapes.Edge(aV[0], aV[1]), TestShapes.Edge(aV[1], aV[2]), TestShapes.Edge(aV[2], aV[3]), TestShapes.Edge(aV[3], aV[0])];
    }

    private static Wire WireOf(params Shape[] theEdges)
    {
        var aB = new BrepBuilder();
        Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (Shape anE in theEdges)
        {
            aB.Add(aW, anE);
        }
        return aW;
    }

    [Fact]
    public void Box_face_wire_is_loaded_in_order()
    {
        Face aF = FirstFace(new MakeBox(1, 2, 3).Shape());
        Wire aW = BrepTools.OuterWire(aF);
        var aWD = new ExtendWireData(aW);
        Assert.Equal(4, aWD.EdgeCount);
        Assert.Equal(0, aWD.NonManifoldEdgeCount);
        Assert.True(aWD.ManifoldMode());
        // the iterator's edges (with the wire's orientation), backwards for a reversed wire
        var anEdges = new List<Shape>();
        for (var it = new Iterator(aW); it.More(); it.Next())
        {
            anEdges.Add(it.Value());
        }
        if (aW.Orientation == Reversed)
        {
            anEdges.Reverse();
        }
        for (int i = 1; i <= 4; i++)
        {
            Assert.True(aWD.Edge(i).IsEqual(anEdges[i - 1]));
            Assert.True(aWD.Edge(-i).IsEqual(anEdges[i - 1].Reversed()));
            Assert.Equal(i, aWD.Index(aWD.Edge(i)));
            Assert.False(aWD.IsSeam(i));
        }

        Wire aNew = aWD.Wire();
        Assert.True(aNew.TShape()!.IsFrozen);
        Assert.True(aNew.ClosedFlag);
        Assert.Equal(4, aNew.ChildCount);

        Wire anAPI = aWD.WireAPIMake();
        Assert.False(anAPI.IsNull);
        Assert.True(anAPI.TShape()!.IsFrozen);
        Assert.True(anAPI.ClosedFlag);

        Wire anInternal = aWD.InternalWire();
        Assert.False(anInternal.TShape()!.IsFrozen);
    }

    [Fact]
    public void Reversed_wire_is_loaded_backwards()
    {
        Edge[] s = SquareEdges();
        Wire aW = WireOf(s[0], s[1], s[2], s[3]);
        var aWD = new ExtendWireData(ShapeCasts.Wire(aW.Reversed()));
        // the edges are prepended, keeping their own orientation (the iterator gives them reversed with the wire)
        Assert.True(aWD.Edge(1).IsSame(s[3]));
        Assert.True(aWD.Edge(4).IsSame(s[0]));
        Assert.Equal(Reversed, aWD.Edge(1).Orientation);
    }

    [Fact]
    public void Unchained_wire_is_kept_or_reordered()
    {
        Edge[] s = SquareEdges();
        Wire aW = WireOf(s[0], s[2], s[1], s[3]);

        var aWD = new ExtendWireData();
        Assert.False(aWD.Init(aW, true));
        Assert.True(aWD.Edge(2).IsSame(s[2]));
        Assert.True(aWD.Edge(3).IsSame(s[1]));

        Assert.False(aWD.Init(aW, false));
        for (int i = 1; i <= 4; i++)
        {
            Assert.True(aWD.Edge(i).IsSame(s[i - 1]), $"edge {i}");
        }
    }

    [Fact]
    public void Internal_edges_are_kept_apart_in_manifold_mode()
    {
        Edge[] s = SquareEdges();
        Edge anInternal = TestShapes.Edge(new Point3D(0.2, 0.2, 0), new Point3D(0.8, 0.8, 0));
        Wire aW = WireOf(s[0], s[1], anInternal.Oriented(Internal), s[2], s[3]);

        var aWD = new ExtendWireData(aW);
        Assert.Equal(4, aWD.EdgeCount);
        Assert.Equal(1, aWD.NonManifoldEdgeCount);
        Assert.True(aWD.NonmanifoldEdge(1).IsSame(anInternal));
        Assert.True(aWD.NonmanifoldEdge(-1).IsNull);
        Assert.Single(aWD.NonmanifoldEdges());
        Assert.Equal(5, aWD.Wire().ChildCount);

        var aNM = new ExtendWireData(aW, true, false);
        Assert.False(aNM.ManifoldMode());
        Assert.Equal(5, aNM.EdgeCount);
        Assert.Equal(0, aNM.NonManifoldEdgeCount);
        Assert.True(aNM.Edge(5).IsSame(anInternal));
        Assert.False(aNM.Wire().ClosedFlag); // not manifold: the wire isn't marked closed

        // Init from another wire data copies both lists and the mode
        var aCopy = new ExtendWireData();
        aCopy.Init(aWD);
        Assert.Equal(4, aCopy.EdgeCount);
        Assert.Equal(1, aCopy.NonManifoldEdgeCount);
    }

    [Fact]
    public void Add_wire_data_in_non_manifold_mode_adds_the_wrong_edges()
    {
        // OCCT's Add(ShapeExtend_WireData) in non-manifold mode adds the other wire's i-th ordinary edge where it means
        // its i-th non-manifold edge
        Edge[] s = SquareEdges();
        Edge anInternal = TestShapes.Edge(new Point3D(0.2, 0.2, 0), new Point3D(0.8, 0.8, 0));
        var aSource = new ExtendWireData(WireOf(s[0], s[1], anInternal.Oriented(Internal)));
        Assert.Equal(1, aSource.NonManifoldEdgeCount);

        var aTarget = new ExtendWireData();
        aTarget.ManifoldMode() = false;
        aTarget.Add(aSource);
        Assert.Equal(3, aTarget.EdgeCount);
        Assert.True(aTarget.Edge(3).IsEqual(s[0])); // not the internal edge

        var aManifold = new ExtendWireData();
        aManifold.Add(aSource);
        Assert.Equal(2, aManifold.EdgeCount);
        Assert.True(aManifold.NonmanifoldEdge(1).IsSame(anInternal));
        aManifold.Add((ExtendWireData?)null);
        Assert.Equal(2, aManifold.EdgeCount);
    }

    [Fact]
    public void Editing_the_list()
    {
        Edge[] s = SquareEdges();
        var aWD = new ExtendWireData();
        aWD.Add(s[1]);
        aWD.Add(s[3]);
        aWD.Add(s[0], 1);                    // before the first
        aWD.Add((Shape)s[2], 3);       // before the third
        Assert.Equal(4, aWD.EdgeCount);
        for (int i = 1; i <= 4; i++)
        {
            Assert.True(aWD.Edge(i).IsEqual(s[i - 1]));
        }

        aWD.Remove();                         // the last
        Assert.Equal(3, aWD.EdgeCount);
        aWD.Remove(1);
        Assert.True(aWD.Edge(1).IsEqual(s[1]));
        aWD.Set(s[3]);                        // replaces the last
        Assert.True(aWD.Edge(2).IsEqual(s[3]));
        aWD.Set(s[0], 1);
        Assert.True(aWD.Edge(1).IsEqual(s[0]));

        // AddOriented: 0 at the end, 1 reversed at the end, 2 at the start, 3 reversed at the start
        var anOr = new ExtendWireData();
        anOr.AddOriented(s[0], 0);
        anOr.AddOriented(s[1], 1);
        anOr.AddOriented(s[2], 2);
        anOr.AddOriented(s[3], 3);
        anOr.AddOriented(s[3], -1);           // ignored
        Assert.Equal(4, anOr.EdgeCount);
        Assert.True(anOr.Edge(1).IsEqual(s[3].Reversed()));
        Assert.True(anOr.Edge(2).IsEqual(s[2]));
        Assert.True(anOr.Edge(3).IsEqual(s[0]));
        Assert.True(anOr.Edge(4).IsEqual(s[1].Reversed()));

        // a wire added reversed
        var aW = new ExtendWireData();
        aW.AddOriented((Shape)WireOf(s[0], s[1]), 1);
        Assert.Equal(2, aW.EdgeCount);
        Assert.True(aW.Edge(1).IsEqual(s[0].Reversed()));
        aW.Add(WireOf(s[2], s[3]), 1);
        Assert.Equal(4, aW.EdgeCount);
        Assert.True(aW.Edge(1).IsEqual(s[2]));
        Assert.True(aW.Edge(2).IsEqual(s[3]));
    }

    [Fact]
    public void Reverse_and_SetLast()
    {
        Edge[] s = SquareEdges();
        var aWD = new ExtendWireData(WireOf(s[0], s[1], s[2], s[3]));
        aWD.Reverse();
        for (int i = 1; i <= 4; i++)
        {
            Assert.True(aWD.Edge(i).IsEqual(s[4 - i].Reversed()));
        }
        aWD.Reverse();
        aWD.SetLast(2);                       // s[1] last: s[2] s[3] s[0] s[1]
        Assert.True(aWD.Edge(1).IsEqual(s[2]));
        Assert.True(aWD.Edge(4).IsEqual(s[1]));
        aWD.SetLast(0);                       // nothing
        Assert.True(aWD.Edge(1).IsEqual(s[2]));

        // odd count: the middle edge is reversed too
        var anOdd = new ExtendWireData(WireOf(s[0], s[1], s[2]));
        anOdd.Reverse();
        Assert.True(anOdd.Edge(2).IsEqual(s[1].Reversed()));
    }

    [Fact]
    public void Seams_of_a_cylinder_and_degenerated_edges_of_a_sphere()
    {
        Face aCylF = FirstFace(new ApiMakeCylinder(1, 2).Shape(), f => Tool.Surface(f) is CylindricalSurface);
        var aWD = new ExtendWireData(BrepTools.OuterWire(aCylF));
        Assert.Equal(4, aWD.EdgeCount);
        int aSeams = 0;
        for (int i = 1; i <= 4; i++)
        {
            if (aWD.IsSeam(i))
            {
                aSeams++;
                // Index of a seam finds the occurrence of the same orientation
                Assert.Equal(i, aWD.Index(aWD.Edge(i)));
            }
        }
        Assert.Equal(2, aSeams);

        Face aSphF = FirstFace(new MakeSphere(1).Shape());
        var aSph = new ExtendWireData(BrepTools.OuterWire(aSphF));
        int aFirstDeg = 0;
        for (int i = 1; i <= aSph.EdgeCount && aFirstDeg == 0; i++)
        {
            if (Tool.Degenerated(aSph.Edge(i)))
            {
                aFirstDeg = i;
            }
        }
        Assert.True(aFirstDeg > 0);
        Edge aDeg = aSph.Edge(aFirstDeg);
        aSph.SetDegeneratedLast();
        Assert.True(aSph.Edge(aSph.EdgeCount).IsEqual(aDeg));
    }

    [Fact]
    public void Reverse_on_a_face_swaps_the_seam_pcurves()
    {
        Face aFrozen = FirstFace(new ApiMakeCylinder(1, 2).Shape(), f => Tool.Surface(f) is CylindricalSurface);
        Face aF = ShapeCasts.Face(TestCopies.ExactCopyForWrite(aFrozen, out _));
        var aWD = new ExtendWireData(BrepTools.OuterWire(aF));
        int aSeam = Enumerable.Range(1, 4).First(i => aWD.IsSeam(i) && aWD.Edge(i).Orientation == Forward);
        Edge aSeamE = aWD.Edge(aSeam);
        Curve2D aFwd = Tool.CurveOnSurface(aSeamE, aF, out _, out _)!;
        Curve2D aRev = Tool.CurveOnSurface(ShapeCasts.Edge(aSeamE.Reversed()), aF, out _, out _)!;

        aWD.Reverse(aF);
        Assert.Same(aRev, Tool.CurveOnSurface(aSeamE, aF, out _, out _));
        Assert.Same(aFwd, Tool.CurveOnSurface(ShapeCasts.Edge(aSeamE.Reversed()), aF, out _, out _));

        // on the frozen face's wire, the seam edge is frozen: the write throws
        var aFrozenWD = new ExtendWireData(BrepTools.OuterWire(aFrozen));
        EngineAssert.ThrowsAny<FailureException>(() => aFrozenWD.Reverse(aFrozen));
        // without seams nothing is written
        Face aBoxF = FirstFace(new MakeBox(1, 1, 1).Shape());
        var aBoxWD = new ExtendWireData(BrepTools.OuterWire(aBoxF));
        aBoxWD.Reverse(aBoxF);
        Assert.Equal(4, aBoxWD.EdgeCount);
    }

    [Fact]
    public void WireAPIMake_of_disconnected_edges_is_null()
    {
        var aWD = new ExtendWireData();
        aWD.Add(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        aWD.Add(TestShapes.Edge(new Point3D(5, 0, 0), new Point3D(6, 0, 0)));
        Assert.True(aWD.WireAPIMake().IsNull);
        Assert.False(aWD.Wire().ClosedFlag);
        Assert.True(IsPlanar(FirstFace(new MakeBox(1, 1, 1).Shape())));
    }
}
