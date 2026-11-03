// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_BuilderFace and BOPAlgo_WireSplitter (unit 6F; TKBO has no GTests for them): faces built from the edges of a
/// face and new edges given twice, as BOPAlgo_Builder gives them.
/// </summary>
public class BOPAlgo_BuilderFaceTests
{
    private static Vertex V(double x, double y) => new MakeVertex(new Point3D(x, y, 0)).Vertex();

    private static Edge E(in Vertex a, in Vertex b) => new MakeEdge(a, b).Edge();

    // A 10 x 10 square on z = 0 whose bottom and top sides are split at x = 5, and the edge between the split points.
    private static (Face Face, Edge Middle) SplitSquare()
    {
        Vertex v1 = V(0, 0), v2 = V(5, 0), v3 = V(10, 0), v4 = V(10, 10), v5 = V(5, 10), v6 = V(0, 10);
        MakeWire aMW = new();
        aMW.Add(E(v1, v2));
        aMW.Add(E(v2, v3));
        aMW.Add(E(v3, v4));
        aMW.Add(E(v4, v5));
        aMW.Add(E(v5, v6));
        aMW.Add(E(v6, v1));
        Face aF = new MakeFace(aMW.Wire(), true).Face();
        return (aF, E(v2, v5));
    }

    private static NCollection_List<Shape> EdgesOf(in Face theF)
    {
        NCollection_List<Shape> aLE = new();
        for (global::Kerf.Engine.Topology.Explorer anExp = new(theF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aLE.Append(anExp.Current());
        }
        return aLE;
    }

    private static void AddTwice(NCollection_List<Shape> theLE, in Edge theE)
    {
        theLE.Append(theE.Oriented(Forward));
        theLE.Append(theE.Oriented(Reversed));
    }

    private static BuilderFace Build(in Face theF, NCollection_List<Shape> theLE, bool theParallel = false)
    {
        BuilderFace aBF = new();
        aBF.SetFace(theF);
        aBF.SetShapes(theLE);
        aBF.SetRunParallel(theParallel);
        aBF.Perform();
        Assert.False(aBF.HasErrors);
        return aBF;
    }

    private static double Area(in Shape theS)
    {
        Properties aProps = new();
        MassPropertiesBuilder.SurfaceProperties(theS, aProps);
        return aProps.Mass();
    }

    private static string Text(NCollection_List<Shape> theLS)
    {
        Compound aC = new();
        BrepBuilder aBB = new();
        aBB.MakeCompound(ref aC);
        for (NCollection_List<Shape>.Iterator aIt = new(theLS); aIt.More(); aIt.Next())
        {
            aBB.Add(aC, aIt.Value());
        }
        StringWriter aW = new();
        BrepTools.Write(aC, aW);
        return aW.ToString();
    }

    private static void AssertValid(NCollection_List<Shape> theLS)
    {
        for (NCollection_List<Shape>.Iterator aIt = new(theLS); aIt.More(); aIt.Next())
        {
            Assert.True(new Analyzer(aIt.Value()).IsValid());
        }
    }

    [Fact]
    public void SquareSplitByAnEdge_GivesTwoHalves()
    {
        (Face aF, Edge aMiddle) = SplitSquare();
        NCollection_List<Shape> aLE = EdgesOf(aF);
        AddTwice(aLE, aMiddle);

        BuilderFace aBF = Build(global::Kerf.Engine.Topology.ShapeCasts.Face(aF.Oriented(Reversed)), aLE);
        Assert.Equal(Reversed, aBF.Orientation());
        Assert.Equal(Forward, aBF.Face().Orientation());
        Assert.Equal(2, aBF.Loops().Extent());
        Assert.Equal(2, aBF.Areas().Extent());
        for (NCollection_List<Shape>.Iterator aIt = new(aBF.Areas()); aIt.More(); aIt.Next())
        {
            Assert.Equal(50.0, Area(aIt.Value()), 1e-9);
        }
        AssertValid(aBF.Areas());
        Assert.False(aBF.HasWarnings);
    }

    [Fact]
    public void SquareWithACircle_GivesAFaceWithAHoleAndADisc()
    {
        (Face aF, _) = SplitSquare();
        NCollection_List<Shape> aLE = EdgesOf(aF);
        Edge aCircle = new MakeEdge(new Circle3D(new Frame3D(new Point3D(5, 5, 0), new Direction3D(0, 0, 1)), 2)).Edge();
        AddTwice(aLE, aCircle);

        BuilderFace aBF = Build(aF, aLE);
        Assert.Equal(2, aBF.Areas().Extent());
        double aTotal = 0;
        bool hasHole = false, hasDisc = false;
        for (NCollection_List<Shape>.Iterator aIt = new(aBF.Areas()); aIt.More(); aIt.Next())
        {
            double anArea = Area(aIt.Value());
            aTotal += anArea;
            int aNbW = 0;
            for (global::Kerf.Engine.Topology.Iterator aItW = new(aIt.Value()); aItW.More(); aItW.Next())
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
        (Face aF, _) = SplitSquare();
        NCollection_List<Shape> aLE = EdgesOf(aF);
        AddTwice(aLE, E(V(2, 2), V(3, 3)));

        BuilderFace aBF = Build(aF, aLE);
        Assert.Equal(1, aBF.Areas().Extent());
        Shape aR = aBF.Areas().First();
        int aNbInternal = 0, aNbW = 0;
        for (global::Kerf.Engine.Topology.Iterator aItW = new(aR); aItW.More(); aItW.Next())
        {
            aNbW++;
            for (global::Kerf.Engine.Topology.Iterator aItE = new(aItW.Value()); aItE.More(); aItE.Next())
            {
                aNbInternal += aItE.Value().Orientation() == Internal ? 1 : 0;
            }
        }
        Assert.Equal(2, aNbW);
        Assert.Equal(1, aNbInternal);
        Assert.Equal(100.0, Area(aR), 1e-9);

        BuilderFace aBF2 = new();
        aBF2.SetFace(aF);
        aBF2.SetShapes(aLE);
        aBF2.SetAvoidInternalShapes(true);
        aBF2.Perform();
        Assert.True(aBF2.IsAvoidInternalShapes);
        Assert.Equal(1, aBF2.Areas().Extent());
        int aNbW2 = 0;
        for (global::Kerf.Engine.Topology.Iterator aItW = new(aBF2.Areas().First()); aItW.More(); aItW.Next())
        {
            aNbW2++;
        }
        Assert.Equal(1, aNbW2);
    }

    [Fact]
    public void DanglingEdgeOutside_IsReportedUnused()
    {
        (Face aF, _) = SplitSquare();
        NCollection_List<Shape> aLE = EdgesOf(aF);
        AddTwice(aLE, E(V(20, 20), V(21, 21)));

        BuilderFace aBF = Build(aF, aLE);
        Assert.Equal(1, aBF.Areas().Extent());
        Assert.True(aBF.HasWarning(typeof(AlertFaceBuilderUnusedEdges)));
    }

    // The lateral face of a cylinder rebuilt from its own edges: the seam is given twice, so the block is irregular
    // and the wire splitter follows the angles on the closed surface.
    [Fact]
    public void CylinderLateralFace_WithItsSeam_IsRebuilt()
    {
        Shape aCyl = new ApiMakeCylinder(5, 10).Shape();
        Face aLateral = new();
        for (global::Kerf.Engine.Topology.Explorer anExp = new(aCyl, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Face aFx = global::Kerf.Engine.Topology.ShapeCasts.Face(anExp.Current());
            if (Tool.Surface(aFx) is CylindricalSurface)
            {
                aLateral = aFx;
            }
        }
        Assert.False(aLateral.IsNull);

        BuilderFace aBF = Build(aLateral, EdgesOf(aLateral));
        Assert.Equal(1, aBF.Areas().Extent());
        Assert.Equal(Area(aLateral), Area(aBF.Areas().First()), 1e-9);
        AssertValid(aBF.Areas());
    }

    [Fact]
    public void SphereFace_WithDegeneratedEdges_IsRebuilt()
    {
        Shape aSph = new MakeSphere(3).Shape();
        Face aF = global::Kerf.Engine.Topology.ShapeCasts.Face(new global::Kerf.Engine.Topology.Explorer(aSph, ShapeType.Face).Current());

        BuilderFace aBF = Build(aF, EdgesOf(aF));
        Assert.Equal(1, aBF.Areas().Extent());
        Assert.Equal(Area(aF), Area(aBF.Areas().First()), 1e-9);
    }

    [Fact]
    public void SerialEqualsParallel()
    {
        (Face aF, Edge aMiddle) = SplitSquare();
        NCollection_List<Shape> aLE = EdgesOf(aF);
        AddTwice(aLE, aMiddle);
        Edge aCircle = new MakeEdge(new Circle3D(new Frame3D(new Point3D(2.5, 5, 0), new Direction3D(0, 0, 1)), 1)).Edge();
        AddTwice(aLE, aCircle);

        BuilderFace aSerial = Build(aF, aLE);
        BuilderFace aParallel = Build(aF, aLE, true);
        Assert.Equal(3, aSerial.Areas().Extent());
        Assert.Equal(Text(aSerial.Areas()), Text(aParallel.Areas()));
        double aTotal = 0;
        for (NCollection_List<Shape>.Iterator aIt = new(aSerial.Areas()); aIt.More(); aIt.Next())
        {
            aTotal += Area(aIt.Value());
        }
        Assert.Equal(100.0, aTotal, 1e-9);
    }

    [Fact]
    public void NullFace_IsAnError()
    {
        BuilderFace aBF = new();
        aBF.Perform();
        Assert.True(aBF.HasError(typeof(AlertNullInputShapes)));

        WireSplitter aWS = new();
        aWS.Perform();
        Assert.True(aWS.HasError(typeof(AlertNullInputShapes)));
    }
}
