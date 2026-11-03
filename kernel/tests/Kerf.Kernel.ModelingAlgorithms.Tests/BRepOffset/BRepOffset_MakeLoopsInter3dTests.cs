// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Offsets;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Smoke tests of BRepOffset_MakeLoops and BRepOffset_Inter3d (unit 7E) on the paths that run without
/// BRepOffset_Tool and BRepOffset_Offset: loops rebuilt from the edges stored in an AsDes, and the context faces'
/// free edges stored (or extended) by ContextIntByArc. The shapes are unfrozen copies, as the offset's own are.
/// </summary>
public class BRepOffset_MakeLoopsInter3dTests
{
    private static Face Square(double theZ)
    {
        MakePolygon aPoly = new(new Point3D(0, 0, theZ), new Point3D(10, 0, theZ),
                                               new Point3D(10, 10, theZ), new Point3D(0, 10, theZ), true);
        Face aFace = new MakeFace(aPoly.Wire(), true).Face();
        return ShapeCasts.Face(TestCopies.ExactCopyForWrite(aFace, out _));
    }

    private static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
    {
        NCollection_IndexedMap<Shape> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        List<Shape> aList = [];
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    [Fact]
    public void MakeLoops_Build_RebuildsTheFaceFromItsEdges()
    {
        Face aF = Square(0);
        AsDes anAsDes = new();
        for (Explorer anExp = new(aF.Oriented(Forward), ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            anAsDes.Add(aF, anE);
            Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
            anAsDes.Add(anE, aV1.Oriented(Forward));
            anAsDes.Add(anE, aV2.Oriented(Reversed));
        }

        NCollection_List<Shape> aLF = new();
        aLF.Append(aF);
        ShapeImage anImage = new();
        OffsetMakeLoops aMakeLoops = new();
        aMakeLoops.Build(aLF, anAsDes, anImage, new ShapeImage(), new ProgressRange());

        Assert.True(anImage.HasImage(aF));
        NCollection_List<Shape> aNF = anImage.Image(aF);
        Assert.Equal(1, aNF.Extent());
        Assert.Equal(4, SubShapes(aNF.First(), ShapeType.Edge).Count);
        foreach (Shape anE in SubShapes(aF, ShapeType.Edge))
        {
            Assert.True(anImage.HasImage(anE));
            Assert.Equal(1, anImage.Image(anE).Extent());
        }

        Properties aProps = new();
        MassPropertiesBuilder.SurfaceProperties(aNF.First(), aProps);
        Assert.Equal(100.0, aProps.Mass(), 1.0e-9);
    }

    [Fact]
    public void Inter3d_SetDoneIsDone()
    {
        Face aF1 = Square(0), aF2 = Square(1), aF3 = Square(2);
        Inter3d anInter = new(new AsDes(), Containment.In, 1.0e-7);
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
        Analyse anAnalyse = new(new MakeBox(1, 1, 1).Shape(), 0.01);
        Face aCF = Square(5);
        NCollection_IndexedMap<Shape> aContext = new(ShapeHasher.Instance);
        aContext.Add(aCF);

        AsDes anAsDes = new();
        ShapeImage anInitOffsetEdge = new();
        Inter3d anInter = new(anAsDes, Containment.In, 1.0e-7);
        anInter.ContextIntByArc(aContext, theInSide, anAnalyse, new ShapeImage(), anInitOffsetEdge,
                                new ProgressRange());

        Assert.True(anInter.TouchedFaces().Contains(aCF));
        NCollection_List<Shape> aLE = anAsDes.Descendant(aCF);
        Assert.Equal(4, aLE.Extent());
        foreach (Shape anE in SubShapes(aCF, ShapeType.Edge))
        {
            Assert.Equal(!theInSide, anInitOffsetEdge.HasImage(anE));
            if (theInSide)
            {
                Assert.Contains(anE, aLE);
                continue;
            }
            // The extended edge: the range grown by 100 lengths at each end, with the old vertices stored on it.
            Edge aNE = ShapeCasts.Edge(anInitOffsetEdge.Image(anE).First());
            Tool.Range(ShapeCasts.Edge(anE), out double f, out double l);
            Tool.Range(aNE, out double nf, out double nl);
            Assert.Equal(f - 100 * (l - f), nf, 1.0e-9);
            Assert.Equal(l + 100 * (l - f), nl, 1.0e-9);
            Assert.Equal(2, anAsDes.Descendant(aNE.Oriented(Forward)).Extent());
            Exploration.Vertices(ShapeCasts.Edge(anE), out Vertex aV1, out _);
            Assert.Equal(f, Tool.Parameter(aV1, aNE), 1.0e-9);
        }
    }
}
