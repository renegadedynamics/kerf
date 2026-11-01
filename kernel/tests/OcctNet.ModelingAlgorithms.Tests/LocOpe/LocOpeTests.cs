// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.LocOpe_Operation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// LocOpe's classes (unit 7H) on frozen inputs, as callers hold them (decision 8901): the inputs are never written, the
/// results are frozen and share the caller's sub-shapes where nothing changed. The counts are OCCT's on the same cases
/// (checked against native through tools/locoperec where it records them).
/// </summary>
public class LocOpeTests
{
    internal static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    internal static bool AllFrozen(in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            if (!aMap[i].TShape()!.IsFrozen())
            {
                return false;
            }
        }
        return true;
    }

    // True if the result holds the caller's own TShape of this sub-shape
    internal static bool Shares(in TopoDS_Shape theResult, in TopoDS_Shape theSub)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theResult, theSub.ShapeType(), aMap);
        return aMap.Contains(theSub);
    }

    internal static double Area(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    internal static double Volume(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    internal static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10, 20, 30).Shape();

    // The face of the box at z = 30 (its 6th face)
    internal static TopoDS_Face Face(in TopoDS_Shape theShape, int theIndex)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, TopAbs_FACE, aMap);
        return TopoDS.Face(aMap[theIndex]);
    }

    internal static TopoDS_Face TopFace(in TopoDS_Shape theBox)
    {
        for (var anExp = new TopExp_Explorer(theBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z() - 30) < 1e-9)
            {
                return TopoDS.Face(anExp.Current());
            }
        }
        throw new InvalidOperationException("no top face");
    }

    internal static TopoDS_Edge Segment(in gp_Pnt theP1, in gp_Pnt theP2) => new BRepBuilderAPI_MakeEdge(theP1, theP2).Edge();

    [Fact]
    public void Closed_TgtFaces_SampleEdges()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Face aTop = TopFace(aBox);
        TopoDS_Wire aWire = BRepTools.OuterWire(aTop);
        Assert.True(LocOpe.Closed(aWire, aTop));
        var anEdges = new TopExp_Explorer(aTop, TopAbs_EDGE);
        Assert.False(LocOpe.Closed(TopoDS.Edge(anEdges.Current()), aTop));

        // two faces of a box meet at a right angle
        var aMap = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aBox, TopAbs_EDGE, TopAbs_FACE, aMap);
        Assert.False(LocOpe.TgtFaces(TopoDS.Edge(aMap.FindKey(1)), TopoDS.Face(aMap[1].First()), TopoDS.Face(aMap[1].Last())));

        var aPoints = new NCollection_Sequence<gp_Pnt>();
        LocOpe.SampleEdges(aBox, aPoints);
        Assert.Equal(12 * 9 + 8, aPoints.Length());
    }

    [Fact]
    public void SplitShape_VertexOnEdge_SplitsTheEdge_TheBoxIsUnchanged()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Face aTop = TopFace(aBox);
        TopoDS_Edge anEdge = TopoDS.Edge(new TopExp_Explorer(aTop, TopAbs_EDGE).Current());
        BRep_Tool.Range(anEdge, out double f, out double l);
        Geom_Curve aC = BRep_Tool.Curve(anEdge, out _, out _)!;
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(aC.Value((f + l) / 2)).Vertex();

        ShapeSnapshot aVSnap = ShapeSnapshot.Of(aV);
        var aSplit = new LocOpe_SplitShape(aBox);
        Assert.True(aSplit.CanSplit(anEdge));
        aSplit.Add(aV, (f + l) / 2, anEdge);
        NCollection_List<TopoDS_Shape> anEdges = aSplit.DescendantShapes(anEdge);
        Assert.Equal(2, anEdges.Extent());
        TopoDS_Shape aRes = aSplit.DescendantShapes(aBox).First();

        aSnap.AssertUnchanged(aBox);
        aVSnap.AssertUnchanged(aV); // OCCT adds its point representations on the new edges: the port's are on a copy
        Assert.True(AllFrozen(aRes));
        Assert.Equal(13, Count(aRes, TopAbs_EDGE));
        Assert.Equal(9, Count(aRes, TopAbs_VERTEX));
        Assert.True(aSplit.Shape().IsSame(aBox));
        Assert.False(aSplit.CanSplit(anEdge)); // the splitting is over
        // the faces away from the edge are the caller's own
        int aShared = 0;
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aShared += Shares(aRes, anExp.Current()) ? 1 : 0;
        }
        Assert.Equal(4, aShared);
        Assert.Equal(Area(aBox), Area(aRes), 9);
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
    }

    [Fact]
    public void SplitShape_OnPortedCodesUnfrozenShape_WritesInPlace_AsOCCT()
    {
        // ported code's own shape (BRepFeat's intermediate results) isn't copied: the vertices get their point
        // representations in place, and the faces away from the split are the shape's own
        TopoDS_Shape aBox = BRepTools.ExactCopyForWrite(Box(), out _);
        Assert.False(aBox.TShape()!.IsFrozen());
        TopoDS_Edge anEdge = TopoDS.Edge(new TopExp_Explorer(TopFace(aBox), TopAbs_EDGE).Current());
        BRep_Tool.Range(anEdge, out double f, out double l);
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(BRep_Tool.Curve(anEdge, out _, out _)!.Value((f + l) / 2)).Vertex();
        var aSplit = new LocOpe_SplitShape(aBox);
        aSplit.Add(aV, (f + l) / 2, anEdge);
        TopoDS_Shape aRes = aSplit.DescendantShapes(aBox).First();
        Assert.Equal(13, Count(aRes, TopAbs_EDGE));
        TopExp.Vertices(anEdge, out TopoDS_Vertex aV1, out _);
        Assert.True(Shares(aRes, aV1)); // the end vertex, written in place, is the shape's own
    }

    [Fact]
    public void WorkingShapes_CopySharedSubShapesOnce_AndGiveBackTheUnchangedOnes()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Face aTop = TopFace(aBox);
        var aWork = new LocOpe_WorkingShapes();
        TopoDS_Shape aCopy = aWork.Working(aBox);
        Assert.False(aCopy.IsSame(aBox));
        TopoDS_Shape aTopCopy = aWork.Working(aTop); // a face of the box: its copy within the box's
        Assert.True(aTopCopy.IsSame(aWork.Copied(aTop)));
        Assert.True(Shares(aCopy, aTopCopy));
        Assert.True(aWork.Original(aTopCopy).IsSame(aTop));
        TopoDS_Shape anOther = aWork.Working(MidEdge()); // another frozen shape: copied too
        Assert.True(aWork.IsCopied());
        aWork.Finish([aCopy, anOther]);
        // nothing was written: everything comes back
        Assert.True(aWork.Final(aCopy).IsSame(aBox));
        Assert.True(aWork.Final(aTopCopy).IsSame(aTop));
    }

    // Splits the top face of the box by a segment between the middles of two opposite edges
    private static (LocOpe_SplitShape Split, TopoDS_Wire Wire) SplitTop(in TopoDS_Shape theBox)
    {
        TopoDS_Face aTop = TopFace(theBox);
        var aSplit = new LocOpe_SplitShape(theBox);
        var anEnds = new List<(TopoDS_Edge, double, gp_Pnt)>();
        for (var anExp = new TopExp_Explorer(aTop, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
            Geom_Curve aC = BRep_Tool.Curve(anEdge, out double f, out double l)!;
            gp_Pnt aMid = aC.Value((f + l) / 2);
            if (Math.Abs(aMid.Y()) < 1e-9 || Math.Abs(aMid.Y() - 20) < 1e-9)
            {
                anEnds.Add((anEdge, (f + l) / 2, aMid));
            }
        }
        Assert.Equal(2, anEnds.Count);
        TopoDS_Vertex aV1 = new BRepBuilderAPI_MakeVertex(anEnds[0].Item3).Vertex();
        TopoDS_Vertex aV2 = new BRepBuilderAPI_MakeVertex(anEnds[1].Item3).Vertex();
        aSplit.Add(aV1, anEnds[0].Item2, anEnds[0].Item1);
        aSplit.Add(aV2, anEnds[1].Item2, anEnds[1].Item1);
        TopoDS_Wire aWire = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aV1, aV2).Edge()).Wire();
        Assert.True(aSplit.Add(aWire, aTop));
        return (aSplit, aWire);
    }

    [Fact]
    public void SplitShape_OpenWire_SplitsTheFaceInTwo_TheInputsAreUnchanged()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        (LocOpe_SplitShape aSplit, TopoDS_Wire aWire) = SplitTop(aBox);
        ShapeSnapshot aWireSnap = ShapeSnapshot.Of(aWire);
        TopoDS_Face aTop = TopFace(aBox);

        NCollection_List<TopoDS_Shape> aFaces = aSplit.DescendantShapes(aTop);
        Assert.Equal(2, aFaces.Extent());
        TopoDS_Shape aRes = aSplit.DescendantShapes(aBox).First();
        Assert.Equal(1, aSplit.LeftOf(aWire, aTop).Extent());

        aSnap.AssertUnchanged(aBox);
        aWireSnap.AssertUnchanged(aWire);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(7, Count(aRes, TopAbs_FACE));
        Assert.Equal(15, Count(aRes, TopAbs_EDGE));
        Assert.Equal(Area(aBox), Area(aRes), 9);
        // the bottom face is the caller's
        Assert.True(Shares(aRes, Face(aBox, 5)) || Shares(aRes, Face(aBox, 6)));
    }

    [Fact]
    public void SplitShape_ClosedWire_MakesAHole_AndItsInside()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Face aTop = TopFace(aBox);
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(5, 10, 30), new gp_Dir(0, 0, 1)), 2);
        TopoDS_Wire aWire = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aCirc).Edge()).Wire();
        var aSplit = new LocOpe_SplitShape(aBox);
        Assert.True(aSplit.Add(aWire, aTop));
        Assert.Equal(2, aSplit.DescendantShapes(aTop).Extent());
        TopoDS_Shape aRes = aSplit.DescendantShapes(aBox).First();
        Assert.Equal(7, Count(aRes, TopAbs_FACE));
        Assert.Equal(Area(aBox), Area(aRes), 6);
    }

    // The edge along the middle of the top face of the box, from y = 0 to y = 20
    private static TopoDS_Edge MidEdge() => Segment(new gp_Pnt(5, 0, 30), new gp_Pnt(5, 20, 30));

    [Fact]
    public void WiresOnShape_Spliter_SplitTheBox_TheInputsAreUnchanged()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Face aTop = TopFace(aBox);
        TopoDS_Edge anEdge = MidEdge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        ShapeSnapshot anEdgeSnap = ShapeSnapshot.Of(anEdge);

        var aWOnS = new LocOpe_WiresOnShape(aBox);
        aWOnS.Bind(anEdge, aTop);
        var aSpliter = new LocOpe_Spliter(aBox);
        aSpliter.Perform(aWOnS);
        Assert.True(aSpliter.IsDone());
        TopoDS_Shape aRes = aSpliter.ResultingShape();

        aSnap.AssertUnchanged(aBox);
        anEdgeSnap.AssertUnchanged(anEdge);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(7, Count(aRes, TopAbs_FACE));
        Assert.Equal(15, Count(aRes, TopAbs_EDGE));
        Assert.Equal(10, Count(aRes, TopAbs_VERTEX));
        Assert.Equal(2, aSpliter.DescendantShapes(aTop).Extent());
        Assert.Equal(1, aSpliter.DirectLeft().Extent());
        Assert.True(aSpliter.Left().Extent() >= 1);
        Assert.True(aSpliter.Shape().IsSame(aBox));
        // the faces that weren't split nor touched are the caller's
        Assert.True(Shares(aRes, Face(aBox, 1)) || Shares(aRes, Face(aBox, 2)));
        Assert.Equal(Area(aBox), Area(aRes), 9);
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());

        // the wires-on-shape answers with the caller's shapes
        aWOnS.InitEdgeIterator();
        Assert.True(aWOnS.MoreEdge());
        Assert.True(aWOnS.Edge().IsSame(anEdge));
        Assert.True(aWOnS.OnFace().IsSame(aTop));
    }

    [Fact]
    public void WiresOnShape_Add_BindsTheEdgesToTheFaces()
    {
        TopoDS_Shape aBox = Box();
        var aWOnS = new LocOpe_WiresOnShape(aBox);
        var anEdges = new NCollection_Sequence<TopoDS_Shape>();
        anEdges.Append(MidEdge());
        Assert.True(aWOnS.Add(anEdges));
        var aSpliter = new LocOpe_Spliter(aBox);
        aSpliter.Perform(aWOnS);
        Assert.True(aSpliter.IsDone());
        Assert.Equal(7, Count(aSpliter.ResultingShape(), TopAbs_FACE));
    }

    [Fact]
    public void BuildWires_ChainsEdges()
    {
        TopoDS_Shape aBox = Box();
        var aWOnS = new LocOpe_WiresOnShape(aBox);
        var anEdges = new NCollection_List<TopoDS_Shape>();
        var aP = new[] { new gp_Pnt(1, 1, 30), new gp_Pnt(2, 1, 30), new gp_Pnt(2, 2, 30) };
        TopoDS_Vertex aV0 = new BRepBuilderAPI_MakeVertex(aP[0]).Vertex();
        TopoDS_Vertex aV1 = new BRepBuilderAPI_MakeVertex(aP[1]).Vertex();
        TopoDS_Vertex aV2 = new BRepBuilderAPI_MakeVertex(aP[2]).Vertex();
        anEdges.Append(new BRepBuilderAPI_MakeEdge(aV0, aV1).Edge());
        anEdges.Append(new BRepBuilderAPI_MakeEdge(aV1, aV2).Edge());
        var aBuild = new LocOpe_BuildWires(anEdges, aWOnS);
        Assert.True(aBuild.IsDone());
        Assert.Equal(1, aBuild.Result().Extent());
        Assert.Equal(2, Count(aBuild.Result().First(), TopAbs_EDGE));
    }

    [Fact]
    public void BuildShape_MakesASolidFromTheFaces()
    {
        TopoDS_Shape aBox = Box();
        var aFaces = new NCollection_List<TopoDS_Shape>();
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aFaces.Append(anExp.Current());
        }
        var aBuild = new LocOpe_BuildShape(aFaces);
        TopoDS_Shape aRes = aBuild.Shape();
        Assert.Equal(TopAbs_SOLID, aRes.ShapeType());
        Assert.Equal(6000, Volume(aRes), 6);
    }

    [Fact]
    public void Prism_OfAFrozenFace_SharesItsUnchangedEdges()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Face aTop = TopFace(aBox);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aPrism = new LocOpe_Prism(aTop, new gp_Vec(0, 0, 5));
        TopoDS_Shape aRes = aPrism.Shape();
        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(1000, Math.Abs(Volume(aRes)), 6);
        Assert.Equal(6, Count(aRes, TopAbs_FACE));
        // a planar prism stores no pcurves on its walls: the face's edges and the face are the caller's
        Assert.True(Shares(aRes, aTop));
        Assert.True(aPrism.FirstShape().IsSame(aTop));
        for (var anExp = new TopExp_Explorer(aTop, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.Equal(1, aPrism.Shapes(anExp.Current()).Extent());
        }
        var aCurves = new NCollection_Sequence<Geom_Curve>();
        aPrism.Curves(aCurves);
        Assert.Equal(4 * 9 + 4, aCurves.Length());
        Assert.NotNull(aPrism.BarycCurve());

        var aTrans = new LocOpe_Prism(aTop, new gp_Vec(0, 0, 5), new gp_Vec(0, 0, 1));
        Assert.Equal(1000, Math.Abs(Volume(aTrans.Shape())), 6);
        Assert.Equal(1, aTrans.Shapes(new TopExp_Explorer(aTop, TopAbs_EDGE).Current()).Extent());
    }

    [Fact]
    public void Prism_OfACylinderFace_CopiesTheEdgesItWritesInto()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(5, 10).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        TopoDS_Face aLateral = new();
        for (var anExp = new TopExp_Explorer(aCyl, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.Surface(TopoDS.Face(anExp.Current())) is Geom_CylindricalSurface)
            {
                aLateral = TopoDS.Face(anExp.Current());
            }
        }
        var aPrism = new LocOpe_Prism(aLateral, new gp_Vec(1, 0, 0));
        aSnap.AssertUnchanged(aCyl);
        Assert.True(AllFrozen(aPrism.Shape()));
        Assert.True(Count(aPrism.Shape(), TopAbs_FACE) > 3);
    }

    [Fact]
    public void Revol_OfAFrozenFace()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Face aTop = TopFace(aBox);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var anAxis = new gp_Ax1(new gp_Pnt(-10, 0, 0), new gp_Dir(0, 1, 0));
        var aRevol = new LocOpe_Revol();
        aRevol.Perform(aTop, anAxis, Math.PI / 6);
        TopoDS_Shape aRes = aRevol.Shape();
        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(6, Count(aRes, TopAbs_FACE));
        var aCurves = new NCollection_Sequence<Geom_Curve>();
        aRevol.Curves(aCurves);
        Assert.Equal(4 * 9 + 4, aCurves.Length());
        Assert.NotNull(aRevol.BarycCurve());

        var aRevol2 = new LocOpe_Revol();
        aRevol2.Perform(aTop, anAxis, Math.PI / 6, Math.PI / 12);
        Assert.Equal(6, Count(aRevol2.Shape(), TopAbs_FACE));
        Assert.Equal(1, aRevol2.Shapes(new TopExp_Explorer(aTop, TopAbs_EDGE).Current()).Extent());

        var aForm = new LocOpe_RevolutionForm();
        aForm.Perform(aTop, anAxis, Math.PI / 6);
        Assert.Equal(6, Count(aForm.Shape(), TopAbs_FACE));
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void LinearForm_OfAFrozenFace()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Face aTop = TopFace(aBox);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aForm = new LocOpe_LinearForm(aTop, new gp_Vec(0, 0, 5), new gp_Pnt(), new gp_Pnt(1, 0, 0));
        Assert.Equal(1000, Math.Abs(Volume(aForm.Shape())), 6);
        var aForm2 = new LocOpe_LinearForm(aTop, new gp_Vec(0, 0, 5), new gp_Vec(0, 0, 2), new gp_Pnt(), new gp_Pnt(1, 0, 0));
        Assert.Equal(1000, Math.Abs(Volume(aForm2.Shape())), 6);
        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aForm.Shape()));
    }

    [Fact]
    public void DPrism_OfAFrozenRectangle()
    {
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(new gp_Pln(), 0, 10, 0, 20).Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aDPrism = new LocOpe_DPrism(aFace, 5, 5 * Math.PI / 180);
        Assert.True(aDPrism.IsDone());
        TopoDS_Shape aRes = aDPrism.Shape();
        aSnap.AssertUnchanged(aFace);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(6, Count(aRes, TopAbs_FACE)); // as natively (tools/locoperec, PRIM_BOX DPRISM)
        Assert.True(Math.Abs(Volume(aRes)) < 1000 && Math.Abs(Volume(aRes)) > 800);
        var aCurves = new NCollection_Sequence<Geom_Curve>();
        aDPrism.Curves(aCurves);
        Assert.Equal(4 * 8, aCurves.Length());
        Assert.NotNull(aDPrism.BarycCurve());
        for (var anExp = new TopExp_Explorer(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.Equal(1, aDPrism.Shapes(anExp.Current()).Extent());
        }

        var aDPrism2 = new LocOpe_DPrism(aFace, 5, 2, 5 * Math.PI / 180);
        Assert.True(aDPrism2.IsDone());
        Assert.Equal(10, Count(aDPrism2.Shape(), TopAbs_FACE)); // the arcs at the corners below the plane (PRIM_BOX DPRISM2)
    }

    [Fact]
    public void Gluer_GluesABoxOnABox_TheInputsAreUnchanged()
    {
        TopoDS_Shape aBase = Box();
        TopoDS_Shape aNew = new BRepPrimAPI_MakeBox(new gp_Pnt(2, 2, 30), 4, 4, 5).Shape();
        ShapeSnapshot aSnapBase = ShapeSnapshot.Of(aBase);
        ShapeSnapshot aSnapNew = ShapeSnapshot.Of(aNew);
        TopoDS_Face aTop = TopFace(aBase);
        TopoDS_Face aBottom = new();
        for (var anExp = new TopExp_Explorer(aNew, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z() - 30) < 1e-9)
            {
                aBottom = TopoDS.Face(anExp.Current());
            }
        }

        var aGluer = new LocOpe_Gluer(aBase, aNew);
        aGluer.Bind(aBottom, aTop);
        Assert.Equal(LocOpe_FUSE, aGluer.OpeType());
        aGluer.Perform();
        Assert.True(aGluer.IsDone());
        TopoDS_Shape aRes = aGluer.ResultingShape();
        aSnapBase.AssertUnchanged(aBase);
        aSnapNew.AssertUnchanged(aNew);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(6000 + 80, Math.Abs(Volume(aRes)), 6);
        Assert.Equal(11, Count(aRes, TopAbs_FACE));
        Assert.Equal(1, aGluer.DescendantFaces(aTop).Extent()); // the top face with the new box's footprint
        Assert.Equal(0, aGluer.DescendantFaces(aBottom).Extent()); // glued
        Assert.Equal(4, aGluer.Edges().Extent());
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
    }

    [Fact]
    public void Generator_OnAGluedShape()
    {
        TopoDS_Shape aBase = Box();
        TopoDS_Shape aNew = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 30), 10, 20, 5).Shape();
        // glue the new box by its bottom face, whose edges are the base's top edges in shape (not shared): the
        // generator's own test is through the gluer; here the generated shape is glued on itself
        var aGlued = new LocOpe_GluedShape(aNew);
        TopoDS_Face aBottom = new();
        for (var anExp = new TopExp_Explorer(aNew, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z() - 30) < 1e-9)
            {
                aBottom = TopoDS.Face(anExp.Current());
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
        TopoDS_Shape aBox = Box();
        TopoDS_Face aTop = TopFace(aBox);
        var aFind = new LocOpe_FindEdges(aTop, aBox);
        int aNb = 0;
        for (aFind.InitIterator(); aFind.More(); aFind.Next())
        {
            Assert.True(aFind.EdgeFrom().IsSame(aFind.EdgeTo()));
            aNb++;
        }
        Assert.Equal(4, aNb);

        var aFindIn = new LocOpe_FindEdgesInFace(aBox, aTop);
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
        TopoDS_Shape aBox = Box();
        var aLines = new NCollection_Sequence<gp_Lin>();
        aLines.Append(new gp_Lin(new gp_Pnt(5, 10, -10), new gp_Dir(0, 0, 1)));
        var anInt = new LocOpe_CSIntersector(aBox);
        anInt.Perform(aLines);
        Assert.True(anInt.IsDone());
        Assert.Equal(2, anInt.NbPoints(1));
        Assert.Equal(10, anInt.Point(1, 1).Parameter(), 9);
        Assert.Equal(40, anInt.Point(1, 2).Parameter(), 9);
        TopAbs_Orientation anOr = TopAbs_INTERNAL;
        int aFrom = 0, aTo = 0;
        Assert.True(anInt.LocalizeAfter(1, 0.0, 1e-7, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(1, aFrom);
        Assert.Equal(1, aTo);

        var aCurve = new LocOpe_CurveShapeIntersector(new gp_Ax1(new gp_Pnt(5, 10, -10), new gp_Dir(0, 0, 1)), aBox);
        Assert.True(aCurve.IsDone());
        Assert.Equal(2, aCurve.NbPoints());
        Assert.True(aCurve.LocalizeAfter(0.0, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(TopAbs_FORWARD, anOr);
        Assert.True(aCurve.LocalizeBefore(100.0, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(TopAbs_REVERSED, anOr);

        var aCircles = new NCollection_Sequence<gp_Circ>();
        aCircles.Append(new gp_Circ(new gp_Ax2(new gp_Pnt(5, 10, 15), new gp_Dir(0, 0, 1)), 8));
        anInt.Perform(aCircles);
        Assert.Equal(4, anInt.NbPoints(1));
    }

    [Fact]
    public void Pipe_OfAFrozenFace()
    {
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(new gp_Pln(new gp_Pnt(), new gp_Dir(0, 0, 1)), -1, 1, -1, 1).Face();
        TopoDS_Wire aSpine = new BRepBuilderAPI_MakeWire(Segment(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 0, 10))).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aPipe = new LocOpe_Pipe(aSpine, aFace);
        TopoDS_Shape aRes = aPipe.Shape();
        aSnap.AssertUnchanged(aFace);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(6, Count(aRes, TopAbs_FACE));
        Assert.Equal(40, Math.Abs(Volume(aRes)), 6);
        for (var anExp = new TopExp_Explorer(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.Equal(1, aPipe.Shapes(anExp.Current()).Extent());
        }
        Assert.Equal(1, aPipe.Shapes(new TopExp_Explorer(aFace, TopAbs_VERTEX).Current()).Extent());
        // as in OCCT, the pipe lines of a straight spine are infinite lines, which GeomConvert can't convert
        var aPoints = new NCollection_Sequence<gp_Pnt>();
        aPoints.Append(new gp_Pnt(0.5, 0.5, 0));
        Assert.Throws<Standard_DomainError>(() => aPipe.Curves(aPoints));
    }

    [Fact]
    public void SplitDrafts_OnAFrozenBox()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Face aSide = new();
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().X() - 10) < 1e-9)
            {
                aSide = TopoDS.Face(anExp.Current());
            }
        }
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        // the side x = 10 split by a vertical segment, drafted by 5 degrees on its left part from the plane z = 0
        TopoDS_Wire aWire = new BRepBuilderAPI_MakeWire(Segment(new gp_Pnt(10, 10, 0), new gp_Pnt(10, 10, 30))).Wire();
        ShapeSnapshot aWireSnap = ShapeSnapshot.Of(aWire);
        var aDrafts = new LocOpe_SplitDrafts(aBox);
        // as natively (tools/locoperec's DRAFT records, occt-issues.md): the drafted planes meet along a line, and
        // GeomFill_Pipe can't convert the infinite line to a B-spline
        Assert.Throws<Standard_DomainError>(() => aDrafts.Perform(aSide, aWire, new gp_Dir(0, 0, 1),
                                                                   new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 5 * Math.PI / 180));
        aSnap.AssertUnchanged(aBox);
        aWireSnap.AssertUnchanged(aWire);
        Assert.False(aDrafts.IsDone());
        Assert.True(aDrafts.OriginalShape().IsSame(aBox));
    }
}
