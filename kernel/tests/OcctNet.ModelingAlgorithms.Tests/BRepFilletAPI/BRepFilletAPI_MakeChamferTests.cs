// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingAlgorithms/TKFillet/GTests/BRepFilletAPI_MakeChamfer_Test.cxx (OCCT V8_0_1)

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepFilletAPI_MakeChamfer (unit 7N3): OCCT's GTest.</summary>
public class BRepFilletAPI_MakeChamferTests
{
    [Fact]
    public void SymmetricChamfer()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        TopExp_Explorer anExp = new(aBox, TopAbs_EDGE);
        Assert.True(anExp.More());
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());

        BRepFilletAPI_MakeChamfer aChamfer = new(aBox);
        aChamfer.Add(2.0, anEdge);
        TopoDS_Shape aResult = aChamfer.Shape();
        Assert.True(aChamfer.IsDone());

        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    [Fact]
    public void AsymmetricChamfer()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEdgeFaceMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aBox, TopAbs_EDGE, TopAbs_FACE, anEdgeFaceMap);

        TopExp_Explorer anExp = new(aBox, TopAbs_EDGE);
        Assert.True(anExp.More());
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
        TopoDS_Face aFace = TopoDS.Face(anEdgeFaceMap.FindFromKey(anEdge).First());

        BRepFilletAPI_MakeChamfer aChamfer = new(aBox);
        aChamfer.Add(1.0, 3.0, anEdge, aFace);
        TopoDS_Shape aResult = aChamfer.Shape();
        Assert.True(aChamfer.IsDone());

        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    [Fact]
    public void ChamferMoreFaces()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        TopExp_Explorer anExp = new(aBox, TopAbs_EDGE);
        Assert.True(anExp.More());
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());

        BRepFilletAPI_MakeChamfer aChamfer = new(aBox);
        aChamfer.Add(2.0, anEdge);
        TopoDS_Shape aResult = aChamfer.Shape();
        Assert.True(aChamfer.IsDone());

        int aFaceCount = 0;
        for (TopExp_Explorer aFaceExp = new(aResult, TopAbs_FACE); aFaceExp.More(); aFaceExp.Next())
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
        BRepPrimAPI_MakeBox aBoxMaker = new(10.0, 10.0, 10.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        gp_Ax2 anAxis = new(new gp_Pnt(5.0, 0.0, 5.0), new gp_Dir(0.0, 1.0, 0.0));
        BRepPrimAPI_MakeCylinder aCylMaker = new(anAxis, 3.0, 10.0);
        TopoDS_Shape aCyl = aCylMaker.Shape();
        Assert.True(aCylMaker.IsDone());

        BRepAlgoAPI_Fuse aFuse = new(aBox, aCyl);
        Assert.True(aFuse.IsDone());
        TopoDS_Shape aFused = aFuse.Shape();

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVtxFaceMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aFused, TopAbs_VERTEX, TopAbs_FACE, aVtxFaceMap);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEdgeFaceMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aFused, TopAbs_EDGE, TopAbs_FACE, anEdgeFaceMap);

        // Select edges at vertices with 3+ adjacent faces (complex topology).
        BRepFilletAPI_MakeChamfer aChamfer = new(aFused);
        int anEdgeCount = 0;
        for (TopExp_Explorer anEdgeExp = new(aFused, TopAbs_EDGE); anEdgeExp.More() && anEdgeCount < 4; anEdgeExp.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());
            if (!anEdgeFaceMap.Contains(anEdge) || anEdgeFaceMap.FindFromKey(anEdge).IsEmpty())
            {
                continue;
            }
            bool hasComplexVertex = false;
            for (TopExp_Explorer aVtxExp = new(anEdge, TopAbs_VERTEX); aVtxExp.More(); aVtxExp.Next())
            {
                if (aVtxFaceMap.Contains(aVtxExp.Current()) && aVtxFaceMap.FindFromKey(aVtxExp.Current()).Size() >= 3)
                {
                    hasComplexVertex = true;
                    break;
                }
            }
            if (!hasComplexVertex)
            {
                continue;
            }
            TopoDS_Face aFace = TopoDS.Face(anEdgeFaceMap.FindFromKey(anEdge).First());
            aChamfer.Add(0.5, 0.5, anEdge, aFace);
            anEdgeCount++;
        }
        Assert.True(anEdgeCount > 0);

        // Must not crash; may succeed or fail gracefully with IsDone() == false.
        try
        {
            aChamfer.Build();
            if (aChamfer.IsDone())
            {
                Assert.True(new BRepCheck_Analyzer(aChamfer.Shape()).IsValid());
            }
        }
        catch (Standard_Failure)
        {
            // Exception instead of crash is acceptable.
        }
    }

    // Regression test for issue #1163: sequential chamfers on the same shape must not crash due to stale TShape
    // pointers in internal topology maps.
    [Fact]
    public void SequentialChamferNoCrash()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        TopoDS_Shape aShape = aBox;
        int aPrevEdges = 0;
        int aSuccessCount = 0;

        for (int i = 0; i < 3; ++i)
        {
            NCollection_IndexedMap<TopoDS_Shape> anEdgeMap = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aShape, TopAbs_EDGE, anEdgeMap);
            if (anEdgeMap.IsEmpty())
            {
                break;
            }
            // Verify topology actually changed after each successful chamfer.
            if (i > 0 && aPrevEdges > 0)
            {
                Assert.NotEqual(aPrevEdges, anEdgeMap.Extent());
            }
            aPrevEdges = anEdgeMap.Extent();

            int anIdx = (i * 3 + 1) % anEdgeMap.Extent() + 1;
            TopoDS_Edge anEdge = TopoDS.Edge(anEdgeMap[anIdx]);

            BRepFilletAPI_MakeChamfer aChamfer = new(aShape);
            aChamfer.Add(1.0, anEdge);

            try
            {
                aChamfer.Build();
                if (aChamfer.IsDone())
                {
                    aShape = aChamfer.Shape();
                    aSuccessCount++;
                }
                else
                {
                    break;
                }
            }
            catch (Standard_ConstructionError)
            {
                break;
            }
            catch (StdFail_NotDone)
            {
                break;
            }
        }
        Assert.True(aSuccessCount >= 1);
    }
}
