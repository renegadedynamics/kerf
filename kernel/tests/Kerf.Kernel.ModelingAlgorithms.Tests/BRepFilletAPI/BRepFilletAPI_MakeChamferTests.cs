// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingAlgorithms/TKFillet/GTests/BRepFilletAPI_MakeChamfer_Test.cxx (OCCT V8_0_1)

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>BRepFilletAPI_MakeChamfer (unit 7N3): OCCT's GTest.</summary>
public class BRepFilletAPI_MakeChamferTests
{
    [Fact]
    public void SymmetricChamfer()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        Explorer anExp = new(aBox, ShapeType.Edge);
        Assert.True(anExp.More());
        Edge anEdge = ShapeCasts.Edge(anExp.Current());

        MakeChamfer aChamfer = new(aBox);
        aChamfer.Add(2.0, anEdge);
        Shape aResult = aChamfer.Shape();
        Assert.True(aChamfer.IsDone);

        Assert.True(new Analyzer(aResult).IsValid());
    }

    [Fact]
    public void AsymmetricChamfer()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        IndexedMap<Shape, NCollection_List<Shape>> anEdgeFaceMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aBox, ShapeType.Edge, ShapeType.Face, anEdgeFaceMap);

        Explorer anExp = new(aBox, ShapeType.Edge);
        Assert.True(anExp.More());
        Edge anEdge = ShapeCasts.Edge(anExp.Current());
        Face aFace = ShapeCasts.Face(anEdgeFaceMap.GetValue(anEdge).First());

        MakeChamfer aChamfer = new(aBox);
        aChamfer.Add(1.0, 3.0, anEdge, aFace);
        Shape aResult = aChamfer.Shape();
        Assert.True(aChamfer.IsDone);

        Assert.True(new Analyzer(aResult).IsValid());
    }

    [Fact]
    public void ChamferMoreFaces()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        Explorer anExp = new(aBox, ShapeType.Edge);
        Assert.True(anExp.More());
        Edge anEdge = ShapeCasts.Edge(anExp.Current());

        MakeChamfer aChamfer = new(aBox);
        aChamfer.Add(2.0, anEdge);
        Shape aResult = aChamfer.Shape();
        Assert.True(aChamfer.IsDone);

        int aFaceCount = 0;
        for (Explorer aFaceExp = new(aResult, ShapeType.Face); aFaceExp.More(); aFaceExp.Next())
        {
            aFaceCount++;
        }
        Assert.True(aFaceCount > 6);
    }

    // Regression test for issue #1163: chamfer after boolean fusion must not crash due to null shapes in
    // IntersectMoreCorner when topology maps carry stale data.
    [Fact]
    public void ChamferAfterBooleanFusion()
    {
        MakeBox aBoxMaker = new(10.0, 10.0, 10.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        Frame3D anAxis = new(new Point3D(5.0, 0.0, 5.0), new Direction3D(0.0, 1.0, 0.0));
        ApiMakeCylinder aCylMaker = new(anAxis, 3.0, 10.0);
        Shape aCyl = aCylMaker.Shape();
        Assert.True(aCylMaker.IsDone);

        Fuse aFuse = new(aBox, aCyl);
        Assert.True(aFuse.IsDone);
        Shape aFused = aFuse.Shape();

        IndexedMap<Shape, NCollection_List<Shape>> aVtxFaceMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aFused, ShapeType.Vertex, ShapeType.Face, aVtxFaceMap);
        IndexedMap<Shape, NCollection_List<Shape>> anEdgeFaceMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aFused, ShapeType.Edge, ShapeType.Face, anEdgeFaceMap);

        // Select edges at vertices with 3+ adjacent faces (complex topology).
        MakeChamfer aChamfer = new(aFused);
        int anEdgeCount = 0;
        for (Explorer anEdgeExp = new(aFused, ShapeType.Edge); anEdgeExp.More() && anEdgeCount < 4; anEdgeExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anEdgeExp.Current());
            if (!anEdgeFaceMap.ContainsKey(anEdge) || anEdgeFaceMap.GetValue(anEdge).IsEmpty())
            {
                continue;
            }
            bool hasComplexVertex = false;
            for (Explorer aVtxExp = new(anEdge, ShapeType.Vertex); aVtxExp.More(); aVtxExp.Next())
            {
                if (aVtxFaceMap.ContainsKey(aVtxExp.Current()) && aVtxFaceMap.GetValue(aVtxExp.Current()).Size() >= 3)
                {
                    hasComplexVertex = true;
                    break;
                }
            }
            if (!hasComplexVertex)
            {
                continue;
            }
            Face aFace = ShapeCasts.Face(anEdgeFaceMap.GetValue(anEdge).First());
            aChamfer.Add(0.5, 0.5, anEdge, aFace);
            anEdgeCount++;
        }
        Assert.True(anEdgeCount > 0);

        // Must not crash; may succeed or fail gracefully with IsDone() == false.
        try
        {
            aChamfer.Build();
            if (aChamfer.IsDone)
            {
                Assert.True(new Analyzer(aChamfer.Shape()).IsValid());
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            // Exception instead of crash is acceptable.
        }
    }

    // Regression test for issue #1163: sequential chamfers on the same shape must not crash due to stale TShape
    // pointers in internal topology maps.
    [Fact]
    public void SequentialChamferNoCrash()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        Shape aShape = aBox;
        int aPrevEdges = 0;
        int aSuccessCount = 0;

        for (int i = 0; i < 3; ++i)
        {
            IndexedSet<Shape> anEdgeMap = new(ShapeHasher.Instance);
            Exploration.MapShapes(aShape, ShapeType.Edge, anEdgeMap);
            if (anEdgeMap.Count == 0)
            {
                break;
            }
            // Verify topology actually changed after each successful chamfer.
            if (i > 0 && aPrevEdges > 0)
            {
                Assert.NotEqual(aPrevEdges, anEdgeMap.Count);
            }
            aPrevEdges = anEdgeMap.Count;

            int anIdx = (i * 3 + 1) % anEdgeMap.Count + 1;
            Edge anEdge = ShapeCasts.Edge(anEdgeMap[anIdx - 1]);

            MakeChamfer aChamfer = new(aShape);
            aChamfer.Add(1.0, anEdge);

            try
            {
                aChamfer.Build();
                if (aChamfer.IsDone)
                {
                    aShape = aChamfer.Shape();
                    aSuccessCount++;
                }
                else
                {
                    break;
                }
            }
            catch (Exception caught) when (Refusal.Of(caught) is global::Kerf.Engine.Foundation.ConstructionErrorException)
            {
                break;
            }
            catch (NotDoneException)
            {
                break;
            }
        }
        Assert.True(aSuccessCount >= 1);
    }
}
