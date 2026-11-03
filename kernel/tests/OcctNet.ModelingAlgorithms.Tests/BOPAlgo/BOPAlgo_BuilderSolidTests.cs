// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.TestHarness;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_BuilderSolid, BOPAlgo_ShellSplitter and BOPAlgo_Tools' ClassifyFaces and FillInternals (unit 6F) on the
/// faces of boxes: solids, holes and internal shells, serially and in parallel. TKBO has no GTests of these classes.
/// </summary>
public class BOPAlgo_BuilderSolidTests
{
    private static Shape Box(double x, double y, double z, double dx, double dy, double dz) =>
        new BRepPrimAPI_MakeBox(new Point3D(x, y, z), dx, dy, dz).Shape();

    private static NCollection_List<Shape> Faces(params Shape[] theShapes)
    {
        NCollection_List<Shape> aLF = new();
        foreach (Shape aS in theShapes)
        {
            for (Explorer anExp = new(aS, ShapeType.Face); anExp.More(); anExp.Next())
            {
                aLF.Append(anExp.Current());
            }
        }
        return aLF;
    }

    private static BOPAlgo_BuilderSolid BuildSolids(NCollection_List<Shape> theFaces, bool theParallel = false)
    {
        BOPAlgo_BuilderSolid aBS = new();
        aBS.SetShapes(theFaces);
        aBS.SetRunParallel(theParallel);
        aBS.Perform();
        Assert.False(aBS.HasErrors());
        return aBS;
    }

    private static double Volume(in Shape theS) => BRepGProp.VolumeProperties(theS).Mass();

    private static int Count(in Shape theS, ShapeType theType)
    {
        int aNb = 0;
        for (Iterator anIt = new(theS); anIt.More(); anIt.Next())
        {
            aNb += anIt.Value().ShapeType() == theType ? 1 : 0;
        }
        return aNb;
    }

    [Fact]
    public void FacesOfABox_MakeOneValidSolid()
    {
        Shape aBox = Box(0, 0, 0, 1, 2, 3);
        BOPAlgo_BuilderSolid aBS = BuildSolids(Faces(aBox));
        Assert.Equal(1, aBS.Areas().Extent());
        Shape aSolid = aBS.Areas().First();
        Assert.Equal(ShapeType.Solid, aSolid.ShapeType());
        Assert.Equal(1, Count(aSolid, ShapeType.Shell));
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
        foreach (Shape aS in aBS.Areas())
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
        Shape anOuter = Box(0, 0, 0, 4, 4, 4);
        Shape anInner = Box(1, 1, 1, 2, 2, 2).Reversed();
        BOPAlgo_BuilderSolid aBS = BuildSolids(Faces(anOuter, anInner));
        Assert.Equal(1, aBS.Areas().Extent());
        Shape aSolid = aBS.Areas().First();
        Assert.Equal(2, Count(aSolid, ShapeType.Shell));
        Assert.Equal(64.0 - 8.0, Volume(aSolid), 1e-9);
        Assert.True(new BRepCheck_Analyzer(aSolid).IsValid());
    }

    [Fact]
    public void AHoleAlone_MakesASolidOfItsOwn_WithAnInfiniteBox()
    {
        BOPAlgo_BuilderSolid aBS = BuildSolids(Faces(Box(1, 1, 1, 2, 2, 2).Reversed()));
        Assert.Equal(1, aBS.Areas().Extent());
        Shape aSolid = aBS.Areas().First();
        Assert.True(aBS.GetBoxesMap().Find(aSolid).IsWhole);
    }

    [Fact]
    public void AFaceInsideTheBox_IsAddedAsAnInternalShell()
    {
        Shape aBox = Box(0, 0, 0, 4, 4, 4);
        Shape aFace = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        NCollection_List<Shape> aLF = Faces(aBox);
        aLF.Append(aFace);
        BOPAlgo_BuilderSolid aBS = BuildSolids(aLF);
        Assert.Equal(1, aBS.Areas().Extent());
        Shape aSolid = aBS.Areas().First();
        Assert.Equal(2, Count(aSolid, ShapeType.Shell));
        int aNbInternal = 0;
        for (Explorer anExp = new(aSolid, ShapeType.Face); anExp.More(); anExp.Next())
        {
            if (anExp.Current().Orientation() == Internal)
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
        Assert.Equal(1, Count(aBS2.Areas().First(), ShapeType.Shell));
    }

    [Fact]
    public void AFaceOutsideTheBox_IsReportedUnused()
    {
        Shape aFace = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 10), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        NCollection_List<Shape> aLF = Faces(Box(0, 0, 0, 4, 4, 4));
        aLF.Append(aFace);
        BOPAlgo_BuilderSolid aBS = BuildSolids(aLF);
        Assert.Equal(1, aBS.Areas().Extent());
        Assert.Equal(1, Count(aBS.Areas().First(), ShapeType.Shell));
        Assert.True(aBS.HasWarning(typeof(BOPAlgo_AlertSolidBuilderUnusedFaces)));
    }

    [Fact]
    public void OnlyOpenFaces_MakeOneSolidOfInternalShells()
    {
        Shape aFace = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 1, 0, 1).Shape();
        NCollection_List<Shape> aLF = new();
        aLF.Append(aFace);
        BOPAlgo_BuilderSolid aBS = BuildSolids(aLF);
        Assert.Equal(1, aBS.Areas().Extent());
        Assert.Equal(1, Count(aBS.Areas().First(), ShapeType.Shell));
    }

    [Fact]
    public void ShellSplitter_FacesOfABox_OneClosedShell()
    {
        BOPAlgo_ShellSplitter aSS = new();
        foreach (Shape aF in Faces(Box(0, 0, 0, 1, 1, 1)))
        {
            aSS.AddStartElement(aF);
        }
        aSS.Perform();
        Assert.False(aSS.HasErrors());
        Assert.Equal(6, aSS.StartElements().Extent());
        Assert.Equal(1, aSS.Shells().Extent());
        Shape aShell = aSS.Shells().First();
        Assert.True(aShell.Closed());
        Assert.True(Tool.IsClosed(aShell));
        Assert.True(new BRepCheck_Analyzer(aShell).IsValid());
    }

    // Two boxes touching along an edge, sewn in non-manifold mode so that the edge is shared by four faces: the
    // faces make one irregular connexity block, which SplitBlock splits into the two boxes' shells.
    private static NCollection_List<Shape> FacesOfTwoBoxesSharingAnEdge()
    {
        BRepBuilderAPI_Sewing aSewing = new();
        aSewing.SetNonManifoldMode(true);
        aSewing.Add(Box(0, 0, 0, 1, 1, 1));
        aSewing.Add(Box(1, 1, 0, 1, 1, 1));
        aSewing.Perform();
        Shape aSewn = aSewing.SewedShape();
        NCollection_List<Shape> aLF = Faces(aSewn);
        Assert.Equal(12, aLF.Extent());
        // the common edge is shared by four faces
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aSewn, ShapeType.Edge, ShapeType.Face, aMEF);
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
        NCollection_List<Shape> aLF = FacesOfTwoBoxesSharingAnEdge();
        foreach (bool aParallel in new[] { false, true })
        {
            BOPAlgo_ShellSplitter aSS = new();
            aSS.SetRunParallel(aParallel);
            foreach (Shape aF in aLF)
            {
                aSS.AddStartElement(aF);
            }
            aSS.Perform();
            Assert.False(aSS.HasErrors());
            Assert.Equal(2, aSS.Shells().Extent());
            foreach (Shape aShell in aSS.Shells())
            {
                Assert.True(aShell.Closed());
                Assert.Equal(6, Faces(aShell).Extent());
            }
        }

        BOPAlgo_BuilderSolid aBS = BuildSolids(aLF);
        Assert.Equal(2, aBS.Areas().Extent());
        foreach (Shape aS in aBS.Areas())
        {
            Assert.Equal(1.0, Volume(aS), 1e-9);
        }
    }

    [Fact]
    public void ClassifyFaces_InsideAndOutside()
    {
        Shape aSolid = BuildSolids(Faces(Box(0, 0, 0, 4, 4, 4))).Areas().First();
        Shape aFIn = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        Shape aFOut = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 9), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        Shape aFFar = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 10, 13, 10, 13).Shape();
        NCollection_List<Shape> aLF = new();
        aLF.Append(aFOut);
        aLF.Append(aFIn);
        aLF.Append(aFFar);
        NCollection_List<Shape> aLS = new();
        aLS.Append(aSolid);
        foreach (bool aParallel in new[] { false, true })
        {
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anInParts = new(ShapeHasher.Instance);
            BOPAlgo_Tools.ClassifyFaces(aLF, aLS, aParallel, new IntTools_Context(), anInParts);
            Assert.Equal(1, anInParts.Extent());
            Assert.True(anInParts.FindKey(1).IsSame(aSolid));
            NCollection_List<Shape> aLIn = anInParts[1];
            Assert.Equal(1, aLIn.Extent());
            Assert.True(aLIn.First().IsSame(aFIn));
        }

        // a face given as the solid's own internal face isn't classified
        NCollection_DataMap<Shape, NCollection_List<Shape>> aSolidsIF = new(ShapeHasher.Instance);
        NCollection_List<Shape> anOwn = new();
        anOwn.Append(aFIn);
        aSolidsIF.Bind(aSolid, anOwn);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anInParts2 = new(ShapeHasher.Instance);
        BOPAlgo_Tools.ClassifyFaces(aLF, aLS, false, new IntTools_Context(), anInParts2, null, aSolidsIF);
        Assert.True(anInParts2[1].IsEmpty());
    }

    [Fact]
    public void FillInternals_AddsThePartsInside()
    {
        Shape aSolid = BuildSolids(Faces(Box(0, 0, 0, 4, 4, 4))).Areas().First();
        Shape aVIn = new BRepBuilderAPI_MakeVertex(new Point3D(2, 2, 2)).Shape();
        Shape aVOut = new BRepBuilderAPI_MakeVertex(new Point3D(9, 2, 2)).Shape();
        Shape anEIn = new BRepBuilderAPI_MakeEdge(new Point3D(1, 1, 1), new Point3D(1, 1, 3)).Shape();
        Shape aFIn = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 1, 3, 1, 3).Shape();
        // a compound of parts is explored
        BrepBuilder aBB = new();
        Compound aC = new();
        aBB.MakeCompound(ref aC);
        aBB.Add(aC, aVOut);
        aBB.Add(aC, anEIn);
        NCollection_List<Shape> aLParts = new();
        aLParts.Append(aVIn);
        aLParts.Append(aC);
        aLParts.Append(aFIn);
        NCollection_List<Shape> aLS = new();
        aLS.Append(aSolid);
        BOPAlgo_Tools.FillInternals(aLS, aLParts, new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance),
                                    new IntTools_Context());
        // the solid holds its shell, the internal vertex, the internal edge and an internal shell of the face
        Assert.Equal(2, Count(aSolid, ShapeType.Shell));
        Assert.Equal(1, Count(aSolid, ShapeType.Vertex));
        Assert.Equal(1, Count(aSolid, ShapeType.Edge));
        for (Iterator anIt = new(aSolid); anIt.More(); anIt.Next())
        {
            Shape aS = anIt.Value();
            if (aS.ShapeType() == ShapeType.Vertex)
            {
                Assert.True(aS.IsSame(aVIn));
                Assert.Equal(Internal, aS.Orientation());
            }
            else if (aS.ShapeType() == ShapeType.Edge)
            {
                Assert.True(aS.IsSame(anEIn));
                Assert.Equal(Internal, aS.Orientation());
            }
        }
        Assert.Equal(64.0, Volume(aSolid), 1e-9);
    }

    [Fact]
    public void EdgesToWires_SharedEdges()
    {
        // no edge, one edge, then the twelve edges of a box (already sharing their vertices)
        Shape aWires = new();
        Assert.Equal(1, BOPAlgo_Tools.EdgesToWires(new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 0)).Shape(), ref aWires, true));
        Assert.True(aWires.IsNull);

        Shape anEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Shape();
        Assert.Equal(0, BOPAlgo_Tools.EdgesToWires(anEdge, ref aWires, true));
        Assert.Equal(1, Count(aWires, ShapeType.Wire));

        Shape aBox = Box(0, 0, 0, 1, 2, 3);
        Assert.Equal(0, BOPAlgo_Tools.EdgesToWires(aBox, ref aWires, true));
        NCollection_IndexedMap<Shape> aMEIn = new(ShapeHasher.Instance), aMEOut = new(ShapeHasher.Instance);
        Exploration.MapShapes(aBox, ShapeType.Edge, aMEIn);
        Exploration.MapShapes(aWires, ShapeType.Edge, aMEOut);
        Assert.Equal(12, aMEIn.Extent());
        for (int i = 1; i <= 12; i++)
        {
            Assert.True(aMEOut.Contains(aMEIn[i]));
        }
        Assert.True(Count(aWires, ShapeType.Wire) >= 1);
    }

    [Fact]
    public void SerialEqualsParallel()
    {
        // many solids, holes and internal faces
        NCollection_List<Shape> aLF = new();
        for (int i = 0; i < 6; i++)
        {
            foreach (Shape aF in Faces(Box(10 * i, 0, 0, 6, 6, 6), Box(10 * i + 1, 1, 1, 2, 2, 2).Reversed()))
            {
                aLF.Append(aF);
            }
            aLF.Append(new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 5), new Direction3D(0, 0, 1)), 10 * i + 4, 10 * i + 5, 4, 5).Shape());
        }
        foreach (Shape aF in FacesOfTwoBoxesSharingAnEdge())
        {
            aLF.Append(aF.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(0, 20, 0)))));
        }
        DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            BOPAlgo_BuilderSolid aBS = BuildSolids(aLF, true);
            BrepBuilder aBB = new();
            Compound aC = new();
            aBB.MakeCompound(ref aC);
            foreach (Shape aS in aBS.Areas())
            {
                aBB.Add(aC, aS);
            }
            return BRepTextComparison.Write(aC);
        }, 2);
    }
}
