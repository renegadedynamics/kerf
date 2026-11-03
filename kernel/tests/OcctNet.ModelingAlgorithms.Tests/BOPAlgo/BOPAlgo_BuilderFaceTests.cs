// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_BuilderFace and BOPAlgo_WireSplitter (unit 6F; TKBO has no GTests for them): faces built from the edges of a
/// face and new edges given twice, as BOPAlgo_Builder gives them.
/// </summary>
public class BOPAlgo_BuilderFaceTests
{
    private static TopoDS_Vertex V(double x, double y) => new BRepBuilderAPI_MakeVertex(new Point3D(x, y, 0)).Vertex();

    private static TopoDS_Edge E(in TopoDS_Vertex a, in TopoDS_Vertex b) => new BRepBuilderAPI_MakeEdge(a, b).Edge();

    // A 10 x 10 square on z = 0 whose bottom and top sides are split at x = 5, and the edge between the split points.
    private static (TopoDS_Face Face, TopoDS_Edge Middle) SplitSquare()
    {
        TopoDS_Vertex v1 = V(0, 0), v2 = V(5, 0), v3 = V(10, 0), v4 = V(10, 10), v5 = V(5, 10), v6 = V(0, 10);
        BRepBuilderAPI_MakeWire aMW = new();
        aMW.Add(E(v1, v2));
        aMW.Add(E(v2, v3));
        aMW.Add(E(v3, v4));
        aMW.Add(E(v4, v5));
        aMW.Add(E(v5, v6));
        aMW.Add(E(v6, v1));
        TopoDS_Face aF = new BRepBuilderAPI_MakeFace(aMW.Wire(), true).Face();
        return (aF, E(v2, v5));
    }

    private static NCollection_List<TopoDS_Shape> EdgesOf(in TopoDS_Face theF)
    {
        NCollection_List<TopoDS_Shape> aLE = new();
        for (TopExp_Explorer anExp = new(theF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aLE.Append(anExp.Current());
        }
        return aLE;
    }

    private static void AddTwice(NCollection_List<TopoDS_Shape> theLE, in TopoDS_Edge theE)
    {
        theLE.Append(theE.Oriented(TopAbs_FORWARD));
        theLE.Append(theE.Oriented(TopAbs_REVERSED));
    }

    private static BOPAlgo_BuilderFace Build(in TopoDS_Face theF, NCollection_List<TopoDS_Shape> theLE, bool theParallel = false)
    {
        BOPAlgo_BuilderFace aBF = new();
        aBF.SetFace(theF);
        aBF.SetShapes(theLE);
        aBF.SetRunParallel(theParallel);
        aBF.Perform();
        Assert.False(aBF.HasErrors());
        return aBF;
    }

    private static double Area(in TopoDS_Shape theS)
    {
        GProp_GProps aProps = new();
        BRepGProp.SurfaceProperties(theS, aProps);
        return aProps.Mass();
    }

    private static string Text(NCollection_List<TopoDS_Shape> theLS)
    {
        TopoDS_Compound aC = new();
        BRep_Builder aBB = new();
        aBB.MakeCompound(ref aC);
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLS); aIt.More(); aIt.Next())
        {
            aBB.Add(aC, aIt.Value());
        }
        StringWriter aW = new();
        BRepTools.Write(aC, aW);
        return aW.ToString();
    }

    private static void AssertValid(NCollection_List<TopoDS_Shape> theLS)
    {
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLS); aIt.More(); aIt.Next())
        {
            Assert.True(new BRepCheck_Analyzer(aIt.Value()).IsValid());
        }
    }

    [Fact]
    public void SquareSplitByAnEdge_GivesTwoHalves()
    {
        (TopoDS_Face aF, TopoDS_Edge aMiddle) = SplitSquare();
        NCollection_List<TopoDS_Shape> aLE = EdgesOf(aF);
        AddTwice(aLE, aMiddle);

        BOPAlgo_BuilderFace aBF = Build(TopoDS.Face(aF.Oriented(TopAbs_REVERSED)), aLE);
        Assert.Equal(TopAbs_REVERSED, aBF.Orientation());
        Assert.Equal(TopAbs_FORWARD, aBF.Face().Orientation());
        Assert.Equal(2, aBF.Loops().Extent());
        Assert.Equal(2, aBF.Areas().Extent());
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(aBF.Areas()); aIt.More(); aIt.Next())
        {
            Assert.Equal(50.0, Area(aIt.Value()), 1e-9);
        }
        AssertValid(aBF.Areas());
        Assert.False(aBF.HasWarnings());
    }

    [Fact]
    public void SquareWithACircle_GivesAFaceWithAHoleAndADisc()
    {
        (TopoDS_Face aF, _) = SplitSquare();
        NCollection_List<TopoDS_Shape> aLE = EdgesOf(aF);
        TopoDS_Edge aCircle = new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(new Point3D(5, 5, 0), new Direction3D(0, 0, 1)), 2)).Edge();
        AddTwice(aLE, aCircle);

        BOPAlgo_BuilderFace aBF = Build(aF, aLE);
        Assert.Equal(2, aBF.Areas().Extent());
        double aTotal = 0;
        bool hasHole = false, hasDisc = false;
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(aBF.Areas()); aIt.More(); aIt.Next())
        {
            double anArea = Area(aIt.Value());
            aTotal += anArea;
            int aNbW = 0;
            for (TopoDS_Iterator aItW = new(aIt.Value()); aItW.More(); aItW.Next())
            {
                aNbW++;
            }
            if (aNbW == 2)
            {
                hasHole = true;
                Assert.Equal(100 - 4 * Math.PI, anArea, 1e-9);
            }
            else
            {
                hasDisc = true;
                Assert.Equal(4 * Math.PI, anArea, 1e-9);
            }
        }
        Assert.True(hasHole && hasDisc);
        Assert.Equal(100.0, aTotal, 1e-9);
        AssertValid(aBF.Areas());
    }

    [Fact]
    public void DanglingEdge_IsAddedAsAnInternalWire_UnlessAvoided()
    {
        (TopoDS_Face aF, _) = SplitSquare();
        NCollection_List<TopoDS_Shape> aLE = EdgesOf(aF);
        AddTwice(aLE, E(V(2, 2), V(3, 3)));

        BOPAlgo_BuilderFace aBF = Build(aF, aLE);
        Assert.Equal(1, aBF.Areas().Extent());
        TopoDS_Shape aR = aBF.Areas().First();
        int aNbInternal = 0, aNbW = 0;
        for (TopoDS_Iterator aItW = new(aR); aItW.More(); aItW.Next())
        {
            aNbW++;
            for (TopoDS_Iterator aItE = new(aItW.Value()); aItE.More(); aItE.Next())
            {
                aNbInternal += aItE.Value().Orientation() == TopAbs_INTERNAL ? 1 : 0;
            }
        }
        Assert.Equal(2, aNbW);
        Assert.Equal(1, aNbInternal);
        Assert.Equal(100.0, Area(aR), 1e-9);

        BOPAlgo_BuilderFace aBF2 = new();
        aBF2.SetFace(aF);
        aBF2.SetShapes(aLE);
        aBF2.SetAvoidInternalShapes(true);
        aBF2.Perform();
        Assert.True(aBF2.IsAvoidInternalShapes());
        Assert.Equal(1, aBF2.Areas().Extent());
        int aNbW2 = 0;
        for (TopoDS_Iterator aItW = new(aBF2.Areas().First()); aItW.More(); aItW.Next())
        {
            aNbW2++;
        }
        Assert.Equal(1, aNbW2);
    }

    [Fact]
    public void DanglingEdgeOutside_IsReportedUnused()
    {
        (TopoDS_Face aF, _) = SplitSquare();
        NCollection_List<TopoDS_Shape> aLE = EdgesOf(aF);
        AddTwice(aLE, E(V(20, 20), V(21, 21)));

        BOPAlgo_BuilderFace aBF = Build(aF, aLE);
        Assert.Equal(1, aBF.Areas().Extent());
        Assert.True(aBF.HasWarning(typeof(BOPAlgo_AlertFaceBuilderUnusedEdges)));
    }

    // The lateral face of a cylinder rebuilt from its own edges: the seam is given twice, so the block is irregular
    // and the wire splitter follows the angles on the closed surface.
    [Fact]
    public void CylinderLateralFace_WithItsSeam_IsRebuilt()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(5, 10).Shape();
        TopoDS_Face aLateral = new();
        for (TopExp_Explorer anExp = new(aCyl, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aFx = TopoDS.Face(anExp.Current());
            if (BRep_Tool.Surface(aFx) is Geom_CylindricalSurface)
            {
                aLateral = aFx;
            }
        }
        Assert.False(aLateral.IsNull());

        BOPAlgo_BuilderFace aBF = Build(aLateral, EdgesOf(aLateral));
        Assert.Equal(1, aBF.Areas().Extent());
        Assert.Equal(Area(aLateral), Area(aBF.Areas().First()), 1e-9);
        AssertValid(aBF.Areas());
    }

    [Fact]
    public void SphereFace_WithDegeneratedEdges_IsRebuilt()
    {
        TopoDS_Shape aSph = new BRepPrimAPI_MakeSphere(3).Shape();
        TopoDS_Face aF = TopoDS.Face(new TopExp_Explorer(aSph, TopAbs_FACE).Current());

        BOPAlgo_BuilderFace aBF = Build(aF, EdgesOf(aF));
        Assert.Equal(1, aBF.Areas().Extent());
        Assert.Equal(Area(aF), Area(aBF.Areas().First()), 1e-9);
    }

    [Fact]
    public void SerialEqualsParallel()
    {
        (TopoDS_Face aF, TopoDS_Edge aMiddle) = SplitSquare();
        NCollection_List<TopoDS_Shape> aLE = EdgesOf(aF);
        AddTwice(aLE, aMiddle);
        TopoDS_Edge aCircle = new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(new Point3D(2.5, 5, 0), new Direction3D(0, 0, 1)), 1)).Edge();
        AddTwice(aLE, aCircle);

        BOPAlgo_BuilderFace aSerial = Build(aF, aLE);
        BOPAlgo_BuilderFace aParallel = Build(aF, aLE, true);
        Assert.Equal(3, aSerial.Areas().Extent());
        Assert.Equal(Text(aSerial.Areas()), Text(aParallel.Areas()));
        double aTotal = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(aSerial.Areas()); aIt.More(); aIt.Next())
        {
            aTotal += Area(aIt.Value());
        }
        Assert.Equal(100.0, aTotal, 1e-9);
    }

    [Fact]
    public void NullFace_IsAnError()
    {
        BOPAlgo_BuilderFace aBF = new();
        aBF.Perform();
        Assert.True(aBF.HasError(typeof(BOPAlgo_AlertNullInputShapes)));

        BOPAlgo_WireSplitter aWS = new();
        aWS.Perform();
        Assert.True(aWS.HasError(typeof(BOPAlgo_AlertNullInputShapes)));
    }
}
