// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.TestHarness;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_BuilderSolid, BOPAlgo_ShellSplitter and BOPAlgo_Tools' ClassifyFaces and FillInternals (unit 6F) on the
/// faces of boxes: solids, holes and internal shells, serially and in parallel. TKBO has no GTests of these classes.
/// </summary>
public class BOPAlgo_BuilderSolidTests
{
    private static TopoDS_Shape Box(double x, double y, double z, double dx, double dy, double dz) =>
        new BRepPrimAPI_MakeBox(new Point3D(x, y, z), dx, dy, dz).Shape();

    private static NCollection_List<TopoDS_Shape> Faces(params TopoDS_Shape[] theShapes)
    {
        NCollection_List<TopoDS_Shape> aLF = new();
        foreach (TopoDS_Shape aS in theShapes)
        {
            for (TopExp_Explorer anExp = new(aS, TopAbs_FACE); anExp.More(); anExp.Next())
            {
                aLF.Append(anExp.Current());
            }
        }
        return aLF;
    }

    private static BOPAlgo_BuilderSolid BuildSolids(NCollection_List<TopoDS_Shape> theFaces, bool theParallel = false)
    {
        BOPAlgo_BuilderSolid aBS = new();
        aBS.SetShapes(theFaces);
        aBS.SetRunParallel(theParallel);
        aBS.Perform();
        Assert.False(aBS.HasErrors());
        return aBS;
    }

    private static double Volume(in TopoDS_Shape theS) => BRepGProp.VolumeProperties(theS).Mass();

    private static int Count(in TopoDS_Shape theS, TopAbs_ShapeEnum theType)
    {
        int aNb = 0;
        for (TopoDS_Iterator anIt = new(theS); anIt.More(); anIt.Next())
        {
            aNb += anIt.Value().ShapeType() == theType ? 1 : 0;
        }
        return aNb;
    }

    [Fact]
    public void FacesOfABox_MakeOneValidSolid()
    {
        TopoDS_Shape aBox = Box(0, 0, 0, 1, 2, 3);
        BOPAlgo_BuilderSolid aBS = BuildSolids(Faces(aBox));
        Assert.Equal(1, aBS.Areas().Extent());
        TopoDS_Shape aSolid = aBS.Areas().First();
        Assert.Equal(TopAbs_SOLID, aSolid.ShapeType());
        Assert.Equal(1, Count(aSolid, TopAbs_SHELL));
        Assert.Equal(6.0, Volume(aSolid), 1e-9);
        Assert.True(new BRepCheck_Analyzer(aSolid).IsValid());
        Assert.Equal(1, aBS.GetBoxesMap().Extent());
        Assert.True(aBS.GetBoxesMap().IsBound(aSolid));
        Assert.False(aBS.HasWarnings());
    }

    [Fact]
    public void FacesOfTwoDisjointBoxes_MakeTwoSolids()
    {
        BOPAlgo_BuilderSolid aBS = BuildSolids(Faces(Box(0, 0, 0, 1, 1, 1), Box(3, 0, 0, 2, 1, 1)));
        Assert.Equal(2, aBS.Areas().Extent());
        double aSum = 0;
        foreach (TopoDS_Shape aS in aBS.Areas())
        {
            Assert.True(new BRepCheck_Analyzer(aS).IsValid());
            aSum += Volume(aS);
        }
        Assert.Equal(3.0, aSum, 1e-9);
    }

    [Fact]
    public void ReversedFacesOfABox_MakeAHole_InTheOuterBox()
    {
        // the reversed faces of an inner box form a hole: it goes into the outer box's solid
        TopoDS_Shape anOuter = Box(0, 0, 0, 4, 4, 4);
        TopoDS_Shape anInner = Box(1, 1, 1, 2, 2, 2).Reversed();
        BOPAlgo_BuilderSolid aBS = BuildSolids(Faces(anOuter, anInner));
        Assert.Equal(1, aBS.Areas().Extent());
        TopoDS_Shape aSolid = aBS.Areas().First();
        Assert.Equal(2, Count(aSolid, TopAbs_SHELL));
        Assert.Equal(64.0 - 8.0, Volume(aSolid), 1e-9);
        Assert.True(new BRepCheck_Analyzer(aSolid).IsValid());
    }

    [Fact]
    public void AHoleAlone_MakesASolidOfItsOwn_WithAnInfiniteBox()
    {
        BOPAlgo_BuilderSolid aBS = BuildSolids(Faces(Box(1, 1, 1, 2, 2, 2).Reversed()));
        Assert.Equal(1, aBS.Areas().Extent());
        TopoDS_Shape aSolid = aBS.Areas().First();
        Assert.True(aBS.GetBoxesMap().Find(aSolid).IsWhole);
    }

    [Fact]
    public void AFaceInsideTheBox_IsAddedAsAnInternalShell()
    {
        TopoDS_Shape aBox = Box(0, 0, 0, 4, 4, 4);
        TopoDS_Shape aFace = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        NCollection_List<TopoDS_Shape> aLF = Faces(aBox);
        aLF.Append(aFace);
        BOPAlgo_BuilderSolid aBS = BuildSolids(aLF);
        Assert.Equal(1, aBS.Areas().Extent());
        TopoDS_Shape aSolid = aBS.Areas().First();
        Assert.Equal(2, Count(aSolid, TopAbs_SHELL));
        int aNbInternal = 0;
        for (TopExp_Explorer anExp = new(aSolid, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            if (anExp.Current().Orientation() == TopAbs_INTERNAL)
            {
                Assert.True(anExp.Current().IsSame(aFace));
                aNbInternal++;
            }
        }
        Assert.Equal(1, aNbInternal);
        Assert.Equal(64.0, Volume(aSolid), 1e-9);
        Assert.False(aBS.HasWarnings());

        // with the internal shapes avoided, the face is left out
        BOPAlgo_BuilderSolid aBS2 = new();
        aBS2.SetShapes(aLF);
        aBS2.SetAvoidInternalShapes(true);
        aBS2.Perform();
        Assert.Equal(1, Count(aBS2.Areas().First(), TopAbs_SHELL));
    }

    [Fact]
    public void AFaceOutsideTheBox_IsReportedUnused()
    {
        TopoDS_Shape aFace = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 10), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        NCollection_List<TopoDS_Shape> aLF = Faces(Box(0, 0, 0, 4, 4, 4));
        aLF.Append(aFace);
        BOPAlgo_BuilderSolid aBS = BuildSolids(aLF);
        Assert.Equal(1, aBS.Areas().Extent());
        Assert.Equal(1, Count(aBS.Areas().First(), TopAbs_SHELL));
        Assert.True(aBS.HasWarning(typeof(BOPAlgo_AlertSolidBuilderUnusedFaces)));
    }

    [Fact]
    public void OnlyOpenFaces_MakeOneSolidOfInternalShells()
    {
        TopoDS_Shape aFace = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 1, 0, 1).Shape();
        NCollection_List<TopoDS_Shape> aLF = new();
        aLF.Append(aFace);
        BOPAlgo_BuilderSolid aBS = BuildSolids(aLF);
        Assert.Equal(1, aBS.Areas().Extent());
        Assert.Equal(1, Count(aBS.Areas().First(), TopAbs_SHELL));
    }

    [Fact]
    public void ShellSplitter_FacesOfABox_OneClosedShell()
    {
        BOPAlgo_ShellSplitter aSS = new();
        foreach (TopoDS_Shape aF in Faces(Box(0, 0, 0, 1, 1, 1)))
        {
            aSS.AddStartElement(aF);
        }
        aSS.Perform();
        Assert.False(aSS.HasErrors());
        Assert.Equal(6, aSS.StartElements().Extent());
        Assert.Equal(1, aSS.Shells().Extent());
        TopoDS_Shape aShell = aSS.Shells().First();
        Assert.True(aShell.Closed());
        Assert.True(BRep_Tool.IsClosed(aShell));
        Assert.True(new BRepCheck_Analyzer(aShell).IsValid());
    }

    // Two boxes touching along an edge, sewn in non-manifold mode so that the edge is shared by four faces: the
    // faces make one irregular connexity block, which SplitBlock splits into the two boxes' shells.
    private static NCollection_List<TopoDS_Shape> FacesOfTwoBoxesSharingAnEdge()
    {
        BRepBuilderAPI_Sewing aSewing = new();
        aSewing.SetNonManifoldMode(true);
        aSewing.Add(Box(0, 0, 0, 1, 1, 1));
        aSewing.Add(Box(1, 1, 0, 1, 1, 1));
        aSewing.Perform();
        TopoDS_Shape aSewn = aSewing.SewedShape();
        NCollection_List<TopoDS_Shape> aLF = Faces(aSewn);
        Assert.Equal(12, aLF.Extent());
        // the common edge is shared by four faces
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aSewn, TopAbs_EDGE, TopAbs_FACE, aMEF);
        bool isShared = false;
        for (int i = 1; i <= aMEF.Extent(); i++)
        {
            isShared |= aMEF[i].Extent() == 4;
        }
        Assert.True(isShared);
        return aLF;
    }

    [Fact]
    public void ShellSplitter_TwoBoxesSharingAnEdge_TwoClosedShells()
    {
        NCollection_List<TopoDS_Shape> aLF = FacesOfTwoBoxesSharingAnEdge();
        foreach (bool aParallel in new[] { false, true })
        {
            BOPAlgo_ShellSplitter aSS = new();
            aSS.SetRunParallel(aParallel);
            foreach (TopoDS_Shape aF in aLF)
            {
                aSS.AddStartElement(aF);
            }
            aSS.Perform();
            Assert.False(aSS.HasErrors());
            Assert.Equal(2, aSS.Shells().Extent());
            foreach (TopoDS_Shape aShell in aSS.Shells())
            {
                Assert.True(aShell.Closed());
                Assert.Equal(6, Faces(aShell).Extent());
            }
        }

        BOPAlgo_BuilderSolid aBS = BuildSolids(aLF);
        Assert.Equal(2, aBS.Areas().Extent());
        foreach (TopoDS_Shape aS in aBS.Areas())
        {
            Assert.Equal(1.0, Volume(aS), 1e-9);
        }
    }

    [Fact]
    public void ClassifyFaces_InsideAndOutside()
    {
        TopoDS_Shape aSolid = BuildSolids(Faces(Box(0, 0, 0, 4, 4, 4))).Areas().First();
        TopoDS_Shape aFIn = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        TopoDS_Shape aFOut = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 9), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        TopoDS_Shape aFFar = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 10, 13, 10, 13).Shape();
        NCollection_List<TopoDS_Shape> aLF = new();
        aLF.Append(aFOut);
        aLF.Append(aFIn);
        aLF.Append(aFFar);
        NCollection_List<TopoDS_Shape> aLS = new();
        aLS.Append(aSolid);
        foreach (bool aParallel in new[] { false, true })
        {
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anInParts = new(TopTools_ShapeMapHasher.Instance);
            BOPAlgo_Tools.ClassifyFaces(aLF, aLS, aParallel, new IntTools_Context(), anInParts);
            Assert.Equal(1, anInParts.Extent());
            Assert.True(anInParts.FindKey(1).IsSame(aSolid));
            NCollection_List<TopoDS_Shape> aLIn = anInParts[1];
            Assert.Equal(1, aLIn.Extent());
            Assert.True(aLIn.First().IsSame(aFIn));
        }

        // a face given as the solid's own internal face isn't classified
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aSolidsIF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> anOwn = new();
        anOwn.Append(aFIn);
        aSolidsIF.Bind(aSolid, anOwn);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anInParts2 = new(TopTools_ShapeMapHasher.Instance);
        BOPAlgo_Tools.ClassifyFaces(aLF, aLS, false, new IntTools_Context(), anInParts2, null, aSolidsIF);
        Assert.True(anInParts2[1].IsEmpty());
    }

    [Fact]
    public void FillInternals_AddsThePartsInside()
    {
        TopoDS_Shape aSolid = BuildSolids(Faces(Box(0, 0, 0, 4, 4, 4))).Areas().First();
        TopoDS_Shape aVIn = new BRepBuilderAPI_MakeVertex(new Point3D(2, 2, 2)).Shape();
        TopoDS_Shape aVOut = new BRepBuilderAPI_MakeVertex(new Point3D(9, 2, 2)).Shape();
        TopoDS_Shape anEIn = new BRepBuilderAPI_MakeEdge(new Point3D(1, 1, 1), new Point3D(1, 1, 3)).Shape();
        TopoDS_Shape aFIn = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        // a compound of parts is explored
        BRep_Builder aBB = new();
        TopoDS_Compound aC = new();
        aBB.MakeCompound(ref aC);
        aBB.Add(aC, aVOut);
        aBB.Add(aC, anEIn);
        NCollection_List<TopoDS_Shape> aLParts = new();
        aLParts.Append(aVIn);
        aLParts.Append(aC);
        aLParts.Append(aFIn);
        NCollection_List<TopoDS_Shape> aLS = new();
        aLS.Append(aSolid);
        BOPAlgo_Tools.FillInternals(aLS, aLParts, new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance),
                                    new IntTools_Context());
        // the solid holds its shell, the internal vertex, the internal edge and an internal shell of the face
        Assert.Equal(2, Count(aSolid, TopAbs_SHELL));
        Assert.Equal(1, Count(aSolid, TopAbs_VERTEX));
        Assert.Equal(1, Count(aSolid, TopAbs_EDGE));
        for (TopoDS_Iterator anIt = new(aSolid); anIt.More(); anIt.Next())
        {
            TopoDS_Shape aS = anIt.Value();
            if (aS.ShapeType() == TopAbs_VERTEX)
            {
                Assert.True(aS.IsSame(aVIn));
                Assert.Equal(TopAbs_INTERNAL, aS.Orientation());
            }
            else if (aS.ShapeType() == TopAbs_EDGE)
            {
                Assert.True(aS.IsSame(anEIn));
                Assert.Equal(TopAbs_INTERNAL, aS.Orientation());
            }
        }
        Assert.Equal(64.0, Volume(aSolid), 1e-9);
    }

    [Fact]
    public void EdgesToWires_SharedEdges()
    {
        // no edge, one edge, then the twelve edges of a box (already sharing their vertices)
        TopoDS_Shape aWires = new();
        Assert.Equal(1, BOPAlgo_Tools.EdgesToWires(new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 0)).Shape(), ref aWires, true));
        Assert.True(aWires.IsNull());

        TopoDS_Shape anEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Shape();
        Assert.Equal(0, BOPAlgo_Tools.EdgesToWires(anEdge, ref aWires, true));
        Assert.Equal(1, Count(aWires, TopAbs_WIRE));

        TopoDS_Shape aBox = Box(0, 0, 0, 1, 2, 3);
        Assert.Equal(0, BOPAlgo_Tools.EdgesToWires(aBox, ref aWires, true));
        NCollection_IndexedMap<TopoDS_Shape> aMEIn = new(TopTools_ShapeMapHasher.Instance), aMEOut = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aBox, TopAbs_EDGE, aMEIn);
        TopExp.MapShapes(aWires, TopAbs_EDGE, aMEOut);
        Assert.Equal(12, aMEIn.Extent());
        for (int i = 1; i <= 12; i++)
        {
            Assert.True(aMEOut.Contains(aMEIn[i]));
        }
        Assert.True(Count(aWires, TopAbs_WIRE) >= 1);
    }

    [Fact]
    public void SerialEqualsParallel()
    {
        // many solids, holes and internal faces
        NCollection_List<TopoDS_Shape> aLF = new();
        for (int i = 0; i < 6; i++)
        {
            foreach (TopoDS_Shape aF in Faces(Box(10 * i, 0, 0, 6, 6, 6), Box(10 * i + 1, 1, 1, 2, 2, 2).Reversed()))
            {
                aLF.Append(aF);
            }
            aLF.Append(new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 5), new Direction3D(0, 0, 1)), 10 * i + 4, 10 * i + 5, 4, 5).Shape());
        }
        foreach (TopoDS_Shape aF in FacesOfTwoBoxesSharingAnEdge())
        {
            aLF.Append(aF.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(0, 20, 0)))));
        }
        DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            BOPAlgo_BuilderSolid aBS = BuildSolids(aLF, true);
            BRep_Builder aBB = new();
            TopoDS_Compound aC = new();
            aBB.MakeCompound(ref aC);
            foreach (TopoDS_Shape aS in aBS.Areas())
            {
                aBB.Add(aC, aS);
            }
            return BRepTextComparison.Write(aC);
        }, 2);
    }
}
