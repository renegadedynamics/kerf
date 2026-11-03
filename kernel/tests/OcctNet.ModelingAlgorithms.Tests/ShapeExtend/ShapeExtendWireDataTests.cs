// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of ShapeExtend_WireData: loading wires, editing the list, seams, and making wires.</summary>
public class ShapeExtendWireDataTests
{
    private static TopoDS_Face FirstFace(in TopoDS_Shape theShape, Func<TopoDS_Face, bool>? thePredicate = null)
    {
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anExp.Current());
            if (thePredicate is null || thePredicate(aF))
            {
                return aF;
            }
        }
        throw new InvalidOperationException("no face");
    }

    private static bool IsPlanar(TopoDS_Face theF) => BRep_Tool.Surface(theF) is Geom_Plane;

    // the four sides of the unit square, sharing their vertices: s[i] runs from vertex i to vertex i + 1
    private static TopoDS_Edge[] SquareEdges()
    {
        TopoDS_Vertex[] aV =
        [
            TestShapes.Vertex(new Point3D(0, 0, 0)), TestShapes.Vertex(new Point3D(1, 0, 0)),
            TestShapes.Vertex(new Point3D(1, 1, 0)), TestShapes.Vertex(new Point3D(0, 1, 0)),
        ];
        return [TestShapes.Edge(aV[0], aV[1]), TestShapes.Edge(aV[1], aV[2]), TestShapes.Edge(aV[2], aV[3]), TestShapes.Edge(aV[3], aV[0])];
    }

    private static TopoDS_Wire WireOf(params TopoDS_Shape[] theEdges)
    {
        var aB = new BRep_Builder();
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (TopoDS_Shape anE in theEdges)
        {
            aB.Add(aW, anE);
        }
        return aW;
    }

    [Fact]
    public void Box_face_wire_is_loaded_in_order()
    {
        TopoDS_Face aF = FirstFace(new BRepPrimAPI_MakeBox(1, 2, 3).Shape());
        TopoDS_Wire aW = BRepTools.OuterWire(aF);
        var aWD = new ShapeExtend_WireData(aW);
        Assert.Equal(4, aWD.NbEdges());
        Assert.Equal(0, aWD.NbNonManifoldEdges());
        Assert.True(aWD.ManifoldMode());
        // the iterator's edges (with the wire's orientation), backwards for a reversed wire
        var anEdges = new List<TopoDS_Shape>();
        for (var it = new TopoDS_Iterator(aW); it.More(); it.Next())
        {
            anEdges.Add(it.Value());
        }
        if (aW.Orientation() == TopAbs_REVERSED)
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

        TopoDS_Wire aNew = aWD.Wire();
        Assert.True(aNew.TShape()!.IsFrozen);
        Assert.True(aNew.Closed());
        Assert.Equal(4, aNew.NbChildren());

        TopoDS_Wire anAPI = aWD.WireAPIMake();
        Assert.False(anAPI.IsNull());
        Assert.True(anAPI.TShape()!.IsFrozen);
        Assert.True(anAPI.Closed());

        TopoDS_Wire anInternal = aWD.InternalWire();
        Assert.False(anInternal.TShape()!.IsFrozen);
    }

    [Fact]
    public void Reversed_wire_is_loaded_backwards()
    {
        TopoDS_Edge[] s = SquareEdges();
        TopoDS_Wire aW = WireOf(s[0], s[1], s[2], s[3]);
        var aWD = new ShapeExtend_WireData(TopoDS.Wire(aW.Reversed()));
        // the edges are prepended, keeping their own orientation (the iterator gives them reversed with the wire)
        Assert.True(aWD.Edge(1).IsSame(s[3]));
        Assert.True(aWD.Edge(4).IsSame(s[0]));
        Assert.Equal(TopAbs_REVERSED, aWD.Edge(1).Orientation());
    }

    [Fact]
    public void Unchained_wire_is_kept_or_reordered()
    {
        TopoDS_Edge[] s = SquareEdges();
        TopoDS_Wire aW = WireOf(s[0], s[2], s[1], s[3]);

        var aWD = new ShapeExtend_WireData();
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
        TopoDS_Edge[] s = SquareEdges();
        TopoDS_Edge anInternal = TestShapes.Edge(new Point3D(0.2, 0.2, 0), new Point3D(0.8, 0.8, 0));
        TopoDS_Wire aW = WireOf(s[0], s[1], anInternal.Oriented(TopAbs_INTERNAL), s[2], s[3]);

        var aWD = new ShapeExtend_WireData(aW);
        Assert.Equal(4, aWD.NbEdges());
        Assert.Equal(1, aWD.NbNonManifoldEdges());
        Assert.True(aWD.NonmanifoldEdge(1).IsSame(anInternal));
        Assert.True(aWD.NonmanifoldEdge(-1).IsNull());
        Assert.Single(aWD.NonmanifoldEdges());
        Assert.Equal(5, aWD.Wire().NbChildren());

        var aNM = new ShapeExtend_WireData(aW, true, false);
        Assert.False(aNM.ManifoldMode());
        Assert.Equal(5, aNM.NbEdges());
        Assert.Equal(0, aNM.NbNonManifoldEdges());
        Assert.True(aNM.Edge(5).IsSame(anInternal));
        Assert.False(aNM.Wire().Closed()); // not manifold: the wire isn't marked closed

        // Init from another wire data copies both lists and the mode
        var aCopy = new ShapeExtend_WireData();
        aCopy.Init(aWD);
        Assert.Equal(4, aCopy.NbEdges());
        Assert.Equal(1, aCopy.NbNonManifoldEdges());
    }

    [Fact]
    public void Add_wire_data_in_non_manifold_mode_adds_the_wrong_edges()
    {
        // OCCT's Add(ShapeExtend_WireData) in non-manifold mode adds the other wire's i-th ordinary edge where it means
        // its i-th non-manifold edge
        TopoDS_Edge[] s = SquareEdges();
        TopoDS_Edge anInternal = TestShapes.Edge(new Point3D(0.2, 0.2, 0), new Point3D(0.8, 0.8, 0));
        var aSource = new ShapeExtend_WireData(WireOf(s[0], s[1], anInternal.Oriented(TopAbs_INTERNAL)));
        Assert.Equal(1, aSource.NbNonManifoldEdges());

        var aTarget = new ShapeExtend_WireData();
        aTarget.ManifoldMode() = false;
        aTarget.Add(aSource);
        Assert.Equal(3, aTarget.NbEdges());
        Assert.True(aTarget.Edge(3).IsEqual(s[0])); // not the internal edge

        var aManifold = new ShapeExtend_WireData();
        aManifold.Add(aSource);
        Assert.Equal(2, aManifold.NbEdges());
        Assert.True(aManifold.NonmanifoldEdge(1).IsSame(anInternal));
        aManifold.Add((ShapeExtend_WireData?)null);
        Assert.Equal(2, aManifold.NbEdges());
    }

    [Fact]
    public void Editing_the_list()
    {
        TopoDS_Edge[] s = SquareEdges();
        var aWD = new ShapeExtend_WireData();
        aWD.Add(s[1]);
        aWD.Add(s[3]);
        aWD.Add(s[0], 1);                    // before the first
        aWD.Add((TopoDS_Shape)s[2], 3);       // before the third
        Assert.Equal(4, aWD.NbEdges());
        for (int i = 1; i <= 4; i++)
        {
            Assert.True(aWD.Edge(i).IsEqual(s[i - 1]));
        }

        aWD.Remove();                         // the last
        Assert.Equal(3, aWD.NbEdges());
        aWD.Remove(1);
        Assert.True(aWD.Edge(1).IsEqual(s[1]));
        aWD.Set(s[3]);                        // replaces the last
        Assert.True(aWD.Edge(2).IsEqual(s[3]));
        aWD.Set(s[0], 1);
        Assert.True(aWD.Edge(1).IsEqual(s[0]));

        // AddOriented: 0 at the end, 1 reversed at the end, 2 at the start, 3 reversed at the start
        var anOr = new ShapeExtend_WireData();
        anOr.AddOriented(s[0], 0);
        anOr.AddOriented(s[1], 1);
        anOr.AddOriented(s[2], 2);
        anOr.AddOriented(s[3], 3);
        anOr.AddOriented(s[3], -1);           // ignored
        Assert.Equal(4, anOr.NbEdges());
        Assert.True(anOr.Edge(1).IsEqual(s[3].Reversed()));
        Assert.True(anOr.Edge(2).IsEqual(s[2]));
        Assert.True(anOr.Edge(3).IsEqual(s[0]));
        Assert.True(anOr.Edge(4).IsEqual(s[1].Reversed()));

        // a wire added reversed
        var aW = new ShapeExtend_WireData();
        aW.AddOriented((TopoDS_Shape)WireOf(s[0], s[1]), 1);
        Assert.Equal(2, aW.NbEdges());
        Assert.True(aW.Edge(1).IsEqual(s[0].Reversed()));
        aW.Add(WireOf(s[2], s[3]), 1);
        Assert.Equal(4, aW.NbEdges());
        Assert.True(aW.Edge(1).IsEqual(s[2]));
        Assert.True(aW.Edge(2).IsEqual(s[3]));
    }

    [Fact]
    public void Reverse_and_SetLast()
    {
        TopoDS_Edge[] s = SquareEdges();
        var aWD = new ShapeExtend_WireData(WireOf(s[0], s[1], s[2], s[3]));
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
        var anOdd = new ShapeExtend_WireData(WireOf(s[0], s[1], s[2]));
        anOdd.Reverse();
        Assert.True(anOdd.Edge(2).IsEqual(s[1].Reversed()));
    }

    [Fact]
    public void Seams_of_a_cylinder_and_degenerated_edges_of_a_sphere()
    {
        TopoDS_Face aCylF = FirstFace(new BRepPrimAPI_MakeCylinder(1, 2).Shape(), f => BRep_Tool.Surface(f) is Geom_CylindricalSurface);
        var aWD = new ShapeExtend_WireData(BRepTools.OuterWire(aCylF));
        Assert.Equal(4, aWD.NbEdges());
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

        TopoDS_Face aSphF = FirstFace(new BRepPrimAPI_MakeSphere(1).Shape());
        var aSph = new ShapeExtend_WireData(BRepTools.OuterWire(aSphF));
        int aFirstDeg = 0;
        for (int i = 1; i <= aSph.NbEdges() && aFirstDeg == 0; i++)
        {
            if (BRep_Tool.Degenerated(aSph.Edge(i)))
            {
                aFirstDeg = i;
            }
        }
        Assert.True(aFirstDeg > 0);
        TopoDS_Edge aDeg = aSph.Edge(aFirstDeg);
        aSph.SetDegeneratedLast();
        Assert.True(aSph.Edge(aSph.NbEdges()).IsEqual(aDeg));
    }

    [Fact]
    public void Reverse_on_a_face_swaps_the_seam_pcurves()
    {
        TopoDS_Face aFrozen = FirstFace(new BRepPrimAPI_MakeCylinder(1, 2).Shape(), f => BRep_Tool.Surface(f) is Geom_CylindricalSurface);
        TopoDS_Face aF = TopoDS.Face(BRepTools.ExactCopyForWrite(aFrozen, out _));
        var aWD = new ShapeExtend_WireData(BRepTools.OuterWire(aF));
        int aSeam = Enumerable.Range(1, 4).First(i => aWD.IsSeam(i) && aWD.Edge(i).Orientation() == TopAbs_FORWARD);
        TopoDS_Edge aSeamE = aWD.Edge(aSeam);
        Geom2d_Curve aFwd = BRep_Tool.CurveOnSurface(aSeamE, aF, out _, out _)!;
        Geom2d_Curve aRev = BRep_Tool.CurveOnSurface(TopoDS.Edge(aSeamE.Reversed()), aF, out _, out _)!;

        aWD.Reverse(aF);
        Assert.Same(aRev, BRep_Tool.CurveOnSurface(aSeamE, aF, out _, out _));
        Assert.Same(aFwd, BRep_Tool.CurveOnSurface(TopoDS.Edge(aSeamE.Reversed()), aF, out _, out _));

        // on the frozen face's wire, the seam edge is frozen: the write throws
        var aFrozenWD = new ShapeExtend_WireData(BRepTools.OuterWire(aFrozen));
        Assert.ThrowsAny<FailureException>(() => aFrozenWD.Reverse(aFrozen));
        // without seams nothing is written
        TopoDS_Face aBoxF = FirstFace(new BRepPrimAPI_MakeBox(1, 1, 1).Shape());
        var aBoxWD = new ShapeExtend_WireData(BRepTools.OuterWire(aBoxF));
        aBoxWD.Reverse(aBoxF);
        Assert.Equal(4, aBoxWD.NbEdges());
    }

    [Fact]
    public void WireAPIMake_of_disconnected_edges_is_null()
    {
        var aWD = new ShapeExtend_WireData();
        aWD.Add(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        aWD.Add(TestShapes.Edge(new Point3D(5, 0, 0), new Point3D(6, 0, 0)));
        Assert.True(aWD.WireAPIMake().IsNull());
        Assert.False(aWD.Wire().Closed());
        Assert.True(IsPlanar(FirstFace(new BRepPrimAPI_MakeBox(1, 1, 1).Shape())));
    }
}
