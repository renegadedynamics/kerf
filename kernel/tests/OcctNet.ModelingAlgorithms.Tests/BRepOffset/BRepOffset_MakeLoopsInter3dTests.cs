// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Smoke tests of BRepOffset_MakeLoops and BRepOffset_Inter3d (unit 7E) on the paths that run without
/// BRepOffset_Tool and BRepOffset_Offset: loops rebuilt from the edges stored in an AsDes, and the context faces'
/// free edges stored (or extended) by ContextIntByArc. The shapes are unfrozen copies, as the offset's own are.
/// </summary>
public class BRepOffset_MakeLoopsInter3dTests
{
    private static TopoDS_Face Square(double theZ)
    {
        BRepBuilderAPI_MakePolygon aPoly = new(new gp_Pnt(0, 0, theZ), new gp_Pnt(10, 0, theZ),
                                               new gp_Pnt(10, 10, theZ), new gp_Pnt(0, 10, theZ), true);
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(aPoly.Wire(), true).Face();
        return TopoDS.Face(BRepTools.ExactCopyForWrite(aFace, out _));
    }

    private static List<TopoDS_Shape> SubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        NCollection_IndexedMap<TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        List<TopoDS_Shape> aList = [];
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    [Fact]
    public void MakeLoops_Build_RebuildsTheFaceFromItsEdges()
    {
        TopoDS_Face aF = Square(0);
        BRepAlgo_AsDes anAsDes = new();
        for (TopExp_Explorer anExp = new(aF.Oriented(TopAbs_FORWARD), TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            anAsDes.Add(aF, anE);
            TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            anAsDes.Add(anE, aV1.Oriented(TopAbs_FORWARD));
            anAsDes.Add(anE, aV2.Oriented(TopAbs_REVERSED));
        }

        NCollection_List<TopoDS_Shape> aLF = new();
        aLF.Append(aF);
        BRepAlgo_Image anImage = new();
        BRepOffset_MakeLoops aMakeLoops = new();
        aMakeLoops.Build(aLF, anAsDes, anImage, new BRepAlgo_Image(), new Message_ProgressRange());

        Assert.True(anImage.HasImage(aF));
        NCollection_List<TopoDS_Shape> aNF = anImage.Image(aF);
        Assert.Equal(1, aNF.Extent());
        Assert.Equal(4, SubShapes(aNF.First(), TopAbs_EDGE).Count);
        foreach (TopoDS_Shape anE in SubShapes(aF, TopAbs_EDGE))
        {
            Assert.True(anImage.HasImage(anE));
            Assert.Equal(1, anImage.Image(anE).Extent());
        }

        GProp_GProps aProps = new();
        BRepGProp.SurfaceProperties(aNF.First(), aProps);
        Assert.Equal(100.0, aProps.Mass(), 1.0e-9);
    }

    [Fact]
    public void Inter3d_SetDoneIsDone()
    {
        TopoDS_Face aF1 = Square(0), aF2 = Square(1), aF3 = Square(2);
        BRepOffset_Inter3d anInter = new(new BRepAlgo_AsDes(), TopAbs_State.TopAbs_IN, 1.0e-7);
        Assert.False(anInter.IsDone(aF1, aF2));
        anInter.SetDone(aF1, aF2);
        Assert.True(anInter.IsDone(aF1, aF2));
        Assert.True(anInter.IsDone(aF2, aF1));
        Assert.False(anInter.IsDone(aF1, aF3));
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Inter3d_ContextIntByArc_StoresOrExtendsTheFreeEdges(bool theInSide)
    {
        // The analysed shape doesn't hold the context face: none of its edges has ancestors.
        BRepOffset_Analyse anAnalyse = new(new BRepPrimAPI_MakeBox(1, 1, 1).Shape(), 0.01);
        TopoDS_Face aCF = Square(5);
        NCollection_IndexedMap<TopoDS_Shape> aContext = new(TopTools_ShapeMapHasher.Instance);
        aContext.Add(aCF);

        BRepAlgo_AsDes anAsDes = new();
        BRepAlgo_Image anInitOffsetEdge = new();
        BRepOffset_Inter3d anInter = new(anAsDes, TopAbs_State.TopAbs_IN, 1.0e-7);
        anInter.ContextIntByArc(aContext, theInSide, anAnalyse, new BRepAlgo_Image(), anInitOffsetEdge,
                                new Message_ProgressRange());

        Assert.True(anInter.TouchedFaces().Contains(aCF));
        NCollection_List<TopoDS_Shape> aLE = anAsDes.Descendant(aCF);
        Assert.Equal(4, aLE.Extent());
        foreach (TopoDS_Shape anE in SubShapes(aCF, TopAbs_EDGE))
        {
            Assert.Equal(!theInSide, anInitOffsetEdge.HasImage(anE));
            if (theInSide)
            {
                Assert.Contains(anE, aLE);
                continue;
            }
            // The extended edge: the range grown by 100 lengths at each end, with the old vertices stored on it.
            TopoDS_Edge aNE = TopoDS.Edge(anInitOffsetEdge.Image(anE).First());
            BRep_Tool.Range(TopoDS.Edge(anE), out double f, out double l);
            BRep_Tool.Range(aNE, out double nf, out double nl);
            Assert.Equal(f - 100 * (l - f), nf, 1.0e-9);
            Assert.Equal(l + 100 * (l - f), nl, 1.0e-9);
            Assert.Equal(2, anAsDes.Descendant(aNE.Oriented(TopAbs_FORWARD)).Extent());
            TopExp.Vertices(TopoDS.Edge(anE), out TopoDS_Vertex aV1, out _);
            Assert.Equal(f, BRep_Tool.Parameter(aV1, aNE), 1.0e-9);
        }
    }
}
