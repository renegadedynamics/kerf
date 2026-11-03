// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Features.Local.FeatureOperation;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// LocOpe's classes (unit 7H) on frozen inputs, as callers hold them (decision 1013): the inputs are never written, the
/// results are frozen and share the caller's sub-shapes where nothing changed. The counts are OCCT's on the same cases
/// (checked against native through tools/locoperec where it records them).
/// </summary>
public class LocOpeTests
{
    internal static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    internal static bool AllFrozen(in Shape theShape)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        for (int i = 0; i < aMap.Count; i++)
        {
            if (!aMap[i].TShape()!.IsFrozen)
            {
                return false;
            }
        }
        return true;
    }

    // True if the result holds the caller's own TShape of this sub-shape
    internal static bool Shares(in Shape theResult, in Shape theSub)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theResult, theSub.ShapeType(), aMap);
        return aMap.Contains(theSub);
    }

    internal static double Area(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    internal static double Volume(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    internal static Shape Box() => new MakeBox(10, 20, 30).Shape();

    // The face of the box at z = 30 (its 6th face)
    internal static Face Face(in Shape theShape, int theIndex)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, global::Kerf.Topology.ShapeType.Face, aMap);
        return ShapeCasts.Face(aMap[theIndex - 1]);
    }

    internal static Face TopFace(in Shape theBox)
    {
        for (var anExp = new Explorer(theBox, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z() - 30) < 1e-9)
            {
                return ShapeCasts.Face(anExp.Current());
            }
        }
        throw new InvalidOperationException("no top face");
    }

    internal static Edge Segment(in Point3D theP1, in Point3D theP2) => new MakeEdge(theP1, theP2).Edge();

    [Fact]
    public void Closed_TgtFaces_SampleEdges()
    {
        Shape aBox = Box();
        Face aTop = TopFace(aBox);
        Wire aWire = BrepTools.OuterWire(aTop);
        Assert.True(global::Kerf.Engine.Features.Local.LocOpe.Closed(aWire, aTop));
        var anEdges = new Explorer(aTop, ShapeType.Edge);
        Assert.False(global::Kerf.Engine.Features.Local.LocOpe.Closed(ShapeCasts.Edge(anEdges.Current()), aTop));

        // two faces of a box meet at a right angle
        var aMap = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aBox, ShapeType.Edge, global::Kerf.Topology.ShapeType.Face, aMap);
        Assert.False(global::Kerf.Engine.Features.Local.LocOpe.TgtFaces(ShapeCasts.Edge(aMap.KeyAt(0)), ShapeCasts.Face(aMap.ValueAt(0).First()), ShapeCasts.Face(aMap.ValueAt(0).Last())));

        var aPoints = new NCollection_Sequence<Point3D>();
        global::Kerf.Engine.Features.Local.LocOpe.SampleEdges(aBox, aPoints);
        Assert.Equal(12 * 9 + 8, aPoints.Length());
    }

    [Fact]
    public void SplitShape_VertexOnEdge_SplitsTheEdge_TheBoxIsUnchanged()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Face aTop = TopFace(aBox);
        Edge anEdge = ShapeCasts.Edge(new Explorer(aTop, ShapeType.Edge).Current());
        Tool.Range(anEdge, out double f, out double l);
        Curve3D aC = Tool.Curve(anEdge, out _, out _)!;
        Vertex aV = new MakeVertex(aC.Value((f + l) / 2)).Vertex();

        ShapeSnapshot aVSnap = ShapeSnapshot.Of(aV);
        var aSplit = new global::Kerf.Engine.Features.Local.LocalSplitShape(aBox);
        Assert.True(aSplit.CanSplit(anEdge));
        aSplit.Add(aV, (f + l) / 2, anEdge);
        NCollection_List<Shape> anEdges = aSplit.DescendantShapes(anEdge);
        Assert.Equal(2, anEdges.Extent());
        Shape aRes = aSplit.DescendantShapes(aBox).First();

        aSnap.AssertUnchanged(aBox);
        aVSnap.AssertUnchanged(aV); // OCCT adds its point representations on the new edges: the port's are on a copy
        Assert.True(AllFrozen(aRes));
        Assert.Equal(13, Count(aRes, ShapeType.Edge));
        Assert.Equal(9, Count(aRes, ShapeType.Vertex));
        Assert.True(aSplit.Shape().IsSame(aBox));
        Assert.False(aSplit.CanSplit(anEdge)); // the splitting is over
        // the faces away from the edge are the caller's own
        int aShared = 0;
        for (var anExp = new Explorer(aBox, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            aShared += Shares(aRes, anExp.Current()) ? 1 : 0;
        }
        Assert.Equal(4, aShared);
        Assert.Equal(Area(aBox), Area(aRes), 9);
        Assert.True(new Analyzer(aRes).IsValid());
    }

    [Fact]
    public void SplitShape_OnPortedCodesUnfrozenShape_WritesInPlace_AsOCCT()
    {
        // ported code's own shape (BRepFeat's intermediate results) isn't copied: the vertices get their point
        // representations in place, and the faces away from the split are the shape's own
        Shape aBox = TestCopies.ExactCopyForWrite(Box(), out _);
        Assert.False(aBox.TShape()!.IsFrozen);
        Edge anEdge = ShapeCasts.Edge(new Explorer(TopFace(aBox), ShapeType.Edge).Current());
        Tool.Range(anEdge, out double f, out double l);
        Vertex aV = new MakeVertex(Tool.Curve(anEdge, out _, out _)!.Value((f + l) / 2)).Vertex();
        var aSplit = new global::Kerf.Engine.Features.Local.LocalSplitShape(aBox);
        aSplit.Add(aV, (f + l) / 2, anEdge);
        Shape aRes = aSplit.DescendantShapes(aBox).First();
        Assert.Equal(13, Count(aRes, ShapeType.Edge));
        Exploration.Vertices(anEdge, out Vertex aV1, out _);
        Assert.True(Shares(aRes, aV1)); // the end vertex, written in place, is the shape's own
    }

    [Fact]
    public void WorkingShapes_WriteInAWorkingOverlay_AndShareTheUnchangedSubShapes()
    {
        Shape aBox = Box();
        Face aTop = TopFace(aBox);
        Vertex aV = Exploration.FirstVertex(ShapeCasts.Edge(new Explorer(aTop, ShapeType.Edge).Current()));
        double aTol = Tool.Tolerance(aV);
        var aWork = new global::Kerf.Engine.Features.Local.WorkingShapes();
        using (aWork.Enter())
        {
            new BrepBuilder().UpdateVertex(aV, 0.5); // the caller's frozen vertex, written in the overlay
            Assert.Equal(0.5, Tool.Tolerance(aV));
        }
        Assert.Equal(aTol, Tool.Tolerance(aV)); // outside, the caller's vertex is unchanged
        Assert.True(aWork.IsWritten);
        aWork.Finish([aBox]);
        Shape aResult = aWork.Final(aBox);
        Assert.False(ReferenceEquals(aResult.TShape(), aBox.TShape()));
        Assert.False(aWork.Final(aV).IsSame(aV));
        Assert.Equal(0.5, Tool.Tolerance(ShapeCasts.Vertex(aWork.Final(aV))));
        // a face away from the vertex is the caller's own in the result
        int aShared = 0;
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aShared += Shares(aResult, anExp.Current()) ? 1 : 0;
        }
        Assert.Equal(3, aShared);

        // nothing written: everything comes back as it was
        var aNone = new global::Kerf.Engine.Features.Local.WorkingShapes();
        using (aNone.Enter())
        {
            Assert.Equal(aTol, Tool.Tolerance(aV));
        }
        Assert.False(aNone.IsWritten);
        aNone.Finish([aBox]);
        Assert.True(ReferenceEquals(aNone.Final(aBox).TShape(), aBox.TShape()));
    }

    // Splits the top face of the box by a segment between the middles of two opposite edges
    private static (global::Kerf.Engine.Features.Local.LocalSplitShape Split, Wire Wire) SplitTop(in Shape theBox)
    {
        Face aTop = TopFace(theBox);
        var aSplit = new global::Kerf.Engine.Features.Local.LocalSplitShape(theBox);
        var anEnds = new List<(Edge, double, Point3D)>();
        for (var anExp = new Explorer(aTop, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Curve3D aC = Tool.Curve(anEdge, out double f, out double l)!;
            Point3D aMid = aC.Value((f + l) / 2);
            if (Math.Abs(aMid.Y()) < 1e-9 || Math.Abs(aMid.Y() - 20) < 1e-9)
            {
                anEnds.Add((anEdge, (f + l) / 2, aMid));
            }
        }
        Assert.Equal(2, anEnds.Count);
        Vertex aV1 = new MakeVertex(anEnds[0].Item3).Vertex();
        Vertex aV2 = new MakeVertex(anEnds[1].Item3).Vertex();
        aSplit.Add(aV1, anEnds[0].Item2, anEnds[0].Item1);
        aSplit.Add(aV2, anEnds[1].Item2, anEnds[1].Item1);
        Wire aWire = new MakeWire(new MakeEdge(aV1, aV2).Edge()).Wire();
        Assert.True(aSplit.Add(aWire, aTop));
        return (aSplit, aWire);
    }

    [Fact]
    public void SplitShape_OpenWire_SplitsTheFaceInTwo_TheInputsAreUnchanged()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        (global::Kerf.Engine.Features.Local.LocalSplitShape aSplit, Wire aWire) = SplitTop(aBox);
        ShapeSnapshot aWireSnap = ShapeSnapshot.Of(aWire);
        Face aTop = TopFace(aBox);

        NCollection_List<Shape> aFaces = aSplit.DescendantShapes(aTop);
        Assert.Equal(2, aFaces.Extent());
        Shape aRes = aSplit.DescendantShapes(aBox).First();
        Assert.Equal(1, aSplit.LeftOf(aWire, aTop).Extent());

        aSnap.AssertUnchanged(aBox);
        aWireSnap.AssertUnchanged(aWire);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(7, Count(aRes, global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(15, Count(aRes, ShapeType.Edge));
        Assert.Equal(Area(aBox), Area(aRes), 9);
        // the bottom face is the caller's
        Assert.True(Shares(aRes, Face(aBox, 5)) || Shares(aRes, Face(aBox, 6)));
    }

    [Fact]
    public void SplitShape_ClosedWire_MakesAHole_AndItsInside()
    {
        Shape aBox = Box();
        Face aTop = TopFace(aBox);
        var aCirc = new Circle3D(new Frame3D(new Point3D(5, 10, 30), new Direction3D(0, 0, 1)), 2);
        Wire aWire = new MakeWire(new MakeEdge(aCirc).Edge()).Wire();
        var aSplit = new global::Kerf.Engine.Features.Local.LocalSplitShape(aBox);
        Assert.True(aSplit.Add(aWire, aTop));
        Assert.Equal(2, aSplit.DescendantShapes(aTop).Extent());
        Shape aRes = aSplit.DescendantShapes(aBox).First();
        Assert.Equal(7, Count(aRes, global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(Area(aBox), Area(aRes), 6);
    }

    // The edge along the middle of the top face of the box, from y = 0 to y = 20
    private static Edge MidEdge() => Segment(new Point3D(5, 0, 30), new Point3D(5, 20, 30));

    [Fact]
    public void WiresOnShape_Spliter_SplitTheBox_TheInputsAreUnchanged()
    {
        Shape aBox = Box();
        Face aTop = TopFace(aBox);
        Edge anEdge = MidEdge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        ShapeSnapshot anEdgeSnap = ShapeSnapshot.Of(anEdge);

        var aWOnS = new global::Kerf.Engine.Features.Local.WiresOnShape(aBox);
        aWOnS.Bind(anEdge, aTop);
        var aSpliter = new global::Kerf.Engine.Features.Local.Spliter(aBox);
        aSpliter.Perform(aWOnS);
        Assert.True(aSpliter.IsDone);
        Shape aRes = aSpliter.ResultingShape();

        aSnap.AssertUnchanged(aBox);
        anEdgeSnap.AssertUnchanged(anEdge);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(7, Count(aRes, global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(15, Count(aRes, ShapeType.Edge));
        Assert.Equal(10, Count(aRes, ShapeType.Vertex));
        Assert.Equal(2, aSpliter.DescendantShapes(aTop).Extent());
        Assert.Equal(1, aSpliter.DirectLeft().Extent());
        Assert.True(aSpliter.Left().Extent() >= 1);
        Assert.True(aSpliter.Shape().IsSame(aBox));
        // the faces that weren't split nor touched are the caller's
        Assert.True(Shares(aRes, Face(aBox, 1)) || Shares(aRes, Face(aBox, 2)));
        Assert.Equal(Area(aBox), Area(aRes), 9);
        Assert.True(new Analyzer(aRes).IsValid());

        // the wires-on-shape answers with the caller's shapes
        aWOnS.InitEdgeIterator();
        Assert.True(aWOnS.MoreEdge());
        Assert.True(aWOnS.Edge().IsSame(anEdge));
        Assert.True(aWOnS.OnFace().IsSame(aTop));
    }

    [Fact]
    public void WiresOnShape_Add_BindsTheEdgesToTheFaces()
    {
        Shape aBox = Box();
        var aWOnS = new global::Kerf.Engine.Features.Local.WiresOnShape(aBox);
        var anEdges = new NCollection_Sequence<Shape>();
        anEdges.Append(MidEdge());
        Assert.True(aWOnS.Add(anEdges));
        var aSpliter = new global::Kerf.Engine.Features.Local.Spliter(aBox);
        aSpliter.Perform(aWOnS);
        Assert.True(aSpliter.IsDone);
        Assert.Equal(7, Count(aSpliter.ResultingShape(), global::Kerf.Topology.ShapeType.Face));
    }

    [Fact]
    public void BuildWires_ChainsEdges()
    {
        Shape aBox = Box();
        var aWOnS = new global::Kerf.Engine.Features.Local.WiresOnShape(aBox);
        var anEdges = new NCollection_List<Shape>();
        var aP = new[] { new Point3D(1, 1, 30), new Point3D(2, 1, 30), new Point3D(2, 2, 30) };
        Vertex aV0 = new MakeVertex(aP[0]).Vertex();
        Vertex aV1 = new MakeVertex(aP[1]).Vertex();
        Vertex aV2 = new MakeVertex(aP[2]).Vertex();
        anEdges.Append(new MakeEdge(aV0, aV1).Edge());
        anEdges.Append(new MakeEdge(aV1, aV2).Edge());
        var aBuild = new global::Kerf.Engine.Features.Local.BuildWires(anEdges, aWOnS);
        Assert.True(aBuild.IsDone);
        Assert.Equal(1, aBuild.Result().Extent());
        Assert.Equal(2, Count(aBuild.Result().First(), ShapeType.Edge));
    }

    [Fact]
    public void BuildShape_MakesASolidFromTheFaces()
    {
        Shape aBox = Box();
        var aFaces = new NCollection_List<Shape>();
        for (var anExp = new Explorer(aBox, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces.Append(anExp.Current());
        }
        var aBuild = new global::Kerf.Engine.Features.Local.BuildShape(aFaces);
        Shape aRes = aBuild.Shape();
        Assert.Equal(ShapeType.Solid, aRes.ShapeType());
        Assert.Equal(6000, Volume(aRes), 6);
    }

    [Fact]
    public void Prism_OfAFrozenFace_SharesItsUnchangedEdges()
    {
        Shape aBox = Box();
        Face aTop = TopFace(aBox);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aPrism = new global::Kerf.Engine.Features.Local.LocalPrism(aTop, new Vector3D(0, 0, 5));
        Shape aRes = aPrism.Shape();
        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(1000, Math.Abs(Volume(aRes)), 6);
        Assert.Equal(6, Count(aRes, global::Kerf.Topology.ShapeType.Face));
        // a planar prism stores no pcurves on its walls: the face's edges and the face are the caller's
        Assert.True(Shares(aRes, aTop));
        Assert.True(aPrism.FirstShape().IsSame(aTop));
        for (var anExp = new Explorer(aTop, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.Equal(1, aPrism.Shapes(anExp.Current()).Extent());
        }
        var aCurves = new NCollection_Sequence<Curve3D>();
        aPrism.Curves(aCurves);
        Assert.Equal(4 * 9 + 4, aCurves.Length());
        Assert.NotNull(aPrism.BarycCurve());

        var aTrans = new global::Kerf.Engine.Features.Local.LocalPrism(aTop, new Vector3D(0, 0, 5), new Vector3D(0, 0, 1));
        Assert.Equal(1000, Math.Abs(Volume(aTrans.Shape())), 6);
        Assert.Equal(1, aTrans.Shapes(new Explorer(aTop, ShapeType.Edge).Current()).Extent());
    }

    [Fact]
    public void Prism_OfACylinderFace_CopiesTheEdgesItWritesInto()
    {
        Shape aCyl = new ApiMakeCylinder(5, 10).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        Face aLateral = new();
        for (var anExp = new Explorer(aCyl, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            if (Tool.Surface(ShapeCasts.Face(anExp.Current())) is CylindricalSurface)
            {
                aLateral = ShapeCasts.Face(anExp.Current());
            }
        }
        var aPrism = new global::Kerf.Engine.Features.Local.LocalPrism(aLateral, new Vector3D(1, 0, 0));
        aSnap.AssertUnchanged(aCyl);
        Assert.True(AllFrozen(aPrism.Shape()));
        Assert.True(Count(aPrism.Shape(), global::Kerf.Topology.ShapeType.Face) > 3);
    }

    [Fact]
    public void Revol_OfAFrozenFace()
    {
        Shape aBox = Box();
        Face aTop = TopFace(aBox);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var anAxis = new Axis3D(new Point3D(-10, 0, 0), new Direction3D(0, 1, 0));
        var aRevol = new global::Kerf.Engine.Features.Local.LocalRevol();
        aRevol.Perform(aTop, anAxis, Math.PI / 6);
        Shape aRes = aRevol.Shape();
        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(6, Count(aRes, global::Kerf.Topology.ShapeType.Face));
        var aCurves = new NCollection_Sequence<Curve3D>();
        aRevol.Curves(aCurves);
        Assert.Equal(4 * 9 + 4, aCurves.Length());
        Assert.NotNull(aRevol.BarycCurve());

        var aRevol2 = new global::Kerf.Engine.Features.Local.LocalRevol();
        aRevol2.Perform(aTop, anAxis, Math.PI / 6, Math.PI / 12);
        Assert.Equal(6, Count(aRevol2.Shape(), global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(1, aRevol2.Shapes(new Explorer(aTop, ShapeType.Edge).Current()).Extent());

        var aForm = new global::Kerf.Engine.Features.Local.RevolutionForm();
        aForm.Perform(aTop, anAxis, Math.PI / 6);
        Assert.Equal(6, Count(aForm.Shape(), global::Kerf.Topology.ShapeType.Face));
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void LinearForm_OfAFrozenFace()
    {
        Shape aBox = Box();
        Face aTop = TopFace(aBox);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aForm = new global::Kerf.Engine.Features.Local.LinearForm(aTop, new Vector3D(0, 0, 5), new Point3D(), new Point3D(1, 0, 0));
        Assert.Equal(1000, Math.Abs(Volume(aForm.Shape())), 6);
        var aForm2 = new global::Kerf.Engine.Features.Local.LinearForm(aTop, new Vector3D(0, 0, 5), new Vector3D(0, 0, 2), new Point3D(), new Point3D(1, 0, 0));
        Assert.Equal(1000, Math.Abs(Volume(aForm2.Shape())), 6);
        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aForm.Shape()));
    }

    [Fact]
    public void DPrism_OfAFrozenRectangle()
    {
        Face aFace = new MakeFace(new Plane3D(), 0, 10, 0, 20).Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aDPrism = new global::Kerf.Engine.Features.Local.DPrism(aFace, 5, 5 * Math.PI / 180);
        Assert.True(aDPrism.IsDone);
        Shape aRes = aDPrism.Shape();
        aSnap.AssertUnchanged(aFace);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(6, Count(aRes, global::Kerf.Topology.ShapeType.Face)); // as natively (tools/locoperec, PRIM_BOX DPRISM)
        Assert.True(Math.Abs(Volume(aRes)) < 1000 && Math.Abs(Volume(aRes)) > 800);
        var aCurves = new NCollection_Sequence<Curve3D>();
        aDPrism.Curves(aCurves);
        Assert.Equal(4 * 8, aCurves.Length());
        Assert.NotNull(aDPrism.BarycCurve());
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.Equal(1, aDPrism.Shapes(anExp.Current()).Extent());
        }

        var aDPrism2 = new global::Kerf.Engine.Features.Local.DPrism(aFace, 5, 2, 5 * Math.PI / 180);
        Assert.True(aDPrism2.IsDone);
        Assert.Equal(10, Count(aDPrism2.Shape(), global::Kerf.Topology.ShapeType.Face)); // the arcs at the corners below the plane (PRIM_BOX DPRISM2)
    }

    [Fact]
    public void Gluer_GluesABoxOnABox_TheInputsAreUnchanged()
    {
        Shape aBase = Box();
        Shape aNew = new MakeBox(new Point3D(2, 2, 30), 4, 4, 5).Shape();
        ShapeSnapshot aSnapBase = ShapeSnapshot.Of(aBase);
        ShapeSnapshot aSnapNew = ShapeSnapshot.Of(aNew);
        Face aTop = TopFace(aBase);
        Face aBottom = new();
        for (var anExp = new Explorer(aNew, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z() - 30) < 1e-9)
            {
                aBottom = ShapeCasts.Face(anExp.Current());
            }
        }

        var aGluer = new global::Kerf.Engine.Features.Local.LocalGluer(aBase, aNew);
        aGluer.Bind(aBottom, aTop);
        Assert.Equal(global::Kerf.Engine.Features.Local.FeatureOperation.Fuse, aGluer.OpeType());
        aGluer.Perform();
        Assert.True(aGluer.IsDone);
        Shape aRes = aGluer.ResultingShape();
        aSnapBase.AssertUnchanged(aBase);
        aSnapNew.AssertUnchanged(aNew);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(6000 + 80, Math.Abs(Volume(aRes)), 6);
        Assert.Equal(11, Count(aRes, global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(1, aGluer.DescendantFaces(aTop).Extent()); // the top face with the new box's footprint
        Assert.Equal(0, aGluer.DescendantFaces(aBottom).Extent()); // glued
        Assert.Equal(4, aGluer.Edges().Extent());
        Assert.True(new Analyzer(aRes).IsValid());
    }

    [Fact]
    public void Generator_OnAGluedShape()
    {
        Shape aBase = Box();
        Shape aNew = new MakeBox(new Point3D(0, 0, 30), 10, 20, 5).Shape();
        // glue the new box by its bottom face, whose edges are the base's top edges in shape (not shared): the
        // generator's own test is through the gluer; here the generated shape is glued on itself
        var aGlued = new global::Kerf.Engine.Features.Local.GluedShape(aNew);
        Face aBottom = new();
        for (var anExp = new Explorer(aNew, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z() - 30) < 1e-9)
            {
                aBottom = ShapeCasts.Face(anExp.Current());
            }
        }
        aGlued.GlueOnFace(aBottom);
        Assert.Equal(4, aGlued.GeneratingEdges().Extent());
        Assert.Equal(5, aGlued.OrientedFaces().Extent());
        _ = aBase;
    }

    [Fact]
    public void FindEdges_FindsTheSharedEdges()
    {
        Shape aBox = Box();
        Face aTop = TopFace(aBox);
        var aFind = new global::Kerf.Engine.Features.Local.FindEdges(aTop, aBox);
        int aNb = 0;
        for (aFind.InitIterator(); aFind.More(); aFind.Next())
        {
            Assert.True(aFind.EdgeFrom().IsSame(aFind.EdgeTo()));
            aNb++;
        }
        Assert.Equal(4, aNb);

        var aFindIn = new global::Kerf.Engine.Features.Local.FindEdgesInFace(aBox, aTop);
        int aNbIn = 0;
        for (aFindIn.Init(); aFindIn.More(); aFindIn.Next())
        {
            aNbIn++;
        }
        Assert.Equal(4, aNbIn);
    }

    [Fact]
    public void CSIntersector_And_CurveShapeIntersector()
    {
        Shape aBox = Box();
        var aLines = new NCollection_Sequence<Line3D>();
        aLines.Append(new Line3D(new Point3D(5, 10, -10), new Direction3D(0, 0, 1)));
        var anInt = new global::Kerf.Engine.Features.Local.CSIntersector(aBox);
        anInt.Perform(aLines);
        Assert.True(anInt.IsDone);
        Assert.Equal(2, anInt.NbPoints(1));
        Assert.Equal(10, anInt.Point(1, 1).Parameter(), 9);
        Assert.Equal(40, anInt.Point(1, 2).Parameter(), 9);
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.True(anInt.LocalizeAfter(1, 0.0, 1e-7, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(1, aFrom);
        Assert.Equal(1, aTo);

        var aCurve = new global::Kerf.Engine.Features.Local.LocalCurveShapeIntersector(new Axis3D(new Point3D(5, 10, -10), new Direction3D(0, 0, 1)), aBox);
        Assert.True(aCurve.IsDone);
        Assert.Equal(2, aCurve.PointCount);
        Assert.True(aCurve.LocalizeAfter(0.0, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(Forward, anOr);
        Assert.True(aCurve.LocalizeBefore(100.0, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(Reversed, anOr);

        var aCircles = new NCollection_Sequence<Circle3D>();
        aCircles.Append(new Circle3D(new Frame3D(new Point3D(5, 10, 15), new Direction3D(0, 0, 1)), 8));
        anInt.Perform(aCircles);
        Assert.Equal(4, anInt.NbPoints(1));
    }

    [Fact]
    public void Pipe_OfAFrozenFace()
    {
        Face aFace = new MakeFace(new Plane3D(new Point3D(), new Direction3D(0, 0, 1)), -1, 1, -1, 1).Face();
        Wire aSpine = new MakeWire(Segment(new Point3D(0, 0, 0), new Point3D(0, 0, 10))).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aPipe = new global::Kerf.Engine.Features.Local.LocalPipe(aSpine, aFace);
        Shape aRes = aPipe.Shape();
        aSnap.AssertUnchanged(aFace);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(6, Count(aRes, global::Kerf.Topology.ShapeType.Face));
        Assert.Equal(40, Math.Abs(Volume(aRes)), 6);
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.Equal(1, aPipe.Shapes(anExp.Current()).Extent());
        }
        Assert.Equal(1, aPipe.Shapes(new Explorer(aFace, ShapeType.Vertex).Current()).Extent());
        // as in OCCT, the pipe lines of a straight spine are infinite lines, which GeomConvert can't convert
        var aPoints = new NCollection_Sequence<Point3D>();
        aPoints.Append(new Point3D(0.5, 0.5, 0));
        Assert.Throws<DomainErrorException>(() => aPipe.Curves(aPoints));
    }

    [Fact]
    public void SplitDrafts_OnAFrozenBox()
    {
        Shape aBox = Box();
        Face aSide = new();
        for (var anExp = new Explorer(aBox, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().X() - 10) < 1e-9)
            {
                aSide = ShapeCasts.Face(anExp.Current());
            }
        }
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        // the side x = 10 split by a vertical segment, drafted by 5 degrees on its left part from the plane z = 0
        Wire aWire = new MakeWire(Segment(new Point3D(10, 10, 0), new Point3D(10, 10, 30))).Wire();
        ShapeSnapshot aWireSnap = ShapeSnapshot.Of(aWire);
        var aDrafts = new global::Kerf.Engine.Features.Local.SplitDrafts(aBox);
        // as natively (tools/locoperec's DRAFT records, occt-issues.md): the drafted planes meet along a line, and
        // GeomFill_Pipe can't convert the infinite line to a B-spline
        Assert.Throws<DomainErrorException>(() => aDrafts.Perform(aSide, aWire, new Direction3D(0, 0, 1),
                                                                   new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5 * Math.PI / 180));
        aSnap.AssertUnchanged(aBox);
        aWireSnap.AssertUnchanged(aWire);
        Assert.False(aDrafts.IsDone);
        Assert.True(aDrafts.OriginalShape().IsSame(aBox));
    }
}
