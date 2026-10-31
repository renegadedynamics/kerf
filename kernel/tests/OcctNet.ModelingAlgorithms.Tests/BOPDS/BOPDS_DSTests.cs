// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests.BOPDS;

/// <summary>
/// BOPDS_DS built by hand on boxes: the shape information and ranges after Init, the pave blocks of the edges and
/// their splitting, common blocks, the interference tables and the face information.
/// </summary>
public class BOPDS_DSTests
{
    internal static TopoDS_Shape Box(double theX, double theY, double theZ, double theSize = 10.0) =>
        new BRepPrimAPI_MakeBox(new gp_Pnt(theX, theY, theZ), theSize, theSize, theSize).Shape();

    internal static BOPDS_DS MakeDS(params TopoDS_Shape[] theShapes)
    {
        NCollection_List<TopoDS_Shape> anArgs = new();
        foreach (TopoDS_Shape aShape in theShapes)
        {
            anArgs.Append(aShape);
        }
        BOPDS_DS aDS = new();
        aDS.SetArguments(anArgs);
        aDS.Init();
        return aDS;
    }

    private static List<int> IndicesOfType(BOPDS_DS theDS, TopAbs_ShapeEnum theType, int theRank = -1)
    {
        List<int> anIndices = [];
        for (int i = 0; i < theDS.NbSourceShapes(); ++i)
        {
            if (theDS.ShapeInfo(i).ShapeType() == theType && (theRank < 0 || theDS.Rank(i) == theRank))
            {
                anIndices.Add(i);
            }
        }
        return anIndices;
    }

    [Fact]
    public void Init_TwoBoxes_ShapeInfosAndRanges()
    {
        TopoDS_Shape aBox1 = Box(0, 0, 0), aBox2 = Box(20, 0, 0);
        BOPDS_DS aDS = MakeDS(aBox1, aBox2);

        // A box has 1 solid, 1 shell, 6 faces, 6 wires, 12 edges and 8 vertices
        Assert.Equal(68, aDS.NbSourceShapes());
        Assert.Equal(68, aDS.NbShapes());
        Assert.Equal(2, aDS.NbRanges());
        Assert.Equal(new BOPDS_IndexRange(0, 33), aDS.Range(0));
        Assert.Equal(new BOPDS_IndexRange(34, 67), aDS.Range(1));
        Assert.Equal(0, aDS.Rank(5));
        Assert.Equal(1, aDS.Rank(40));
        Assert.Equal(-1, aDS.Rank(68));
        Assert.True(aDS.IsNewShape(68));
        Assert.False(aDS.IsNewShape(67));

        Assert.True(aDS.Shape(0).IsSame(aBox1));
        Assert.Equal(0, aDS.Index(aBox1));
        Assert.Equal(34, aDS.Index(aBox2));
        Assert.Equal(-1, aDS.Index(Box(0, 0, 0)));
        for (int i = 0; i < aDS.NbShapes(); ++i)
        {
            Assert.Equal(i, aDS.Index(aDS.Shape(i)));
        }

        Assert.Equal(TopAbs_SOLID, aDS.ShapeInfo(0).ShapeType());
        Assert.Equal(6, IndicesOfType(aDS, TopAbs_FACE, 0).Count);
        Assert.Equal(12, IndicesOfType(aDS, TopAbs_EDGE, 1).Count);
        Assert.Equal(8, IndicesOfType(aDS, TopAbs_VERTEX, 1).Count);

        // With two arguments the solids keep their shells as sub-shapes, and no box
        BOPDS_ShapeInfo aSolid = aDS.ShapeInfo(0);
        Assert.Equal(1, aSolid.SubShapes().Extent());
        Assert.Equal(TopAbs_SHELL, aDS.ShapeInfo(aSolid.SubShapes().First()).ShapeType());
        Assert.True(aSolid.Box().IsVoid());

        // A face's sub-shapes become its edges and vertices
        foreach (int nF in IndicesOfType(aDS, TopAbs_FACE))
        {
            BOPDS_ShapeInfo aFace = aDS.ShapeInfo(nF);
            Assert.Equal(8, aFace.SubShapes().Extent());
            Assert.Equal(4, aFace.SubShapes().Count(i => aDS.ShapeInfo(i).ShapeType() == TopAbs_EDGE));
            Assert.Equal(4, aFace.SubShapes().Count(i => aDS.ShapeInfo(i).ShapeType() == TopAbs_VERTEX));
            Assert.False(aFace.Box().IsVoid());
            Assert.True(aFace.HasBRep());
            Assert.True(aFace.IsInterfering());
        }

        // An edge's box holds its vertices' boxes, with the half fuzzy gap added
        foreach (int nE in IndicesOfType(aDS, TopAbs_EDGE))
        {
            BOPDS_ShapeInfo anEdge = aDS.ShapeInfo(nE);
            Assert.Equal(2, anEdge.SubShapes().Extent());
            Assert.False(anEdge.HasFlag());
            foreach (int nV in anEdge.SubShapes())
            {
                Assert.False(anEdge.Box().IsOut(aDS.ShapeInfo(nV).Box()));
            }
        }
        foreach (int nV in IndicesOfType(aDS, TopAbs_VERTEX))
        {
            Bnd_Box aBox = aDS.ShapeInfo(nV).Box();
            Assert.Equal(BRep_Tool.Tolerance(TopoDS.Vertex(aDS.Shape(nV))) + Precision.Confusion() * 0.5, aBox.GetGap());
        }

        Assert.True(aDS.IsSubShape(aSolid.SubShapes().First(), 0));
        Assert.False(aDS.IsSubShape(40, 0));
    }

    [Fact]
    public void Init_OneSolid_HasFacesAndEdgesAndABox()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0));
        BOPDS_ShapeInfo aSolid = aDS.ShapeInfo(0);
        // the faces, and the faces' sub-shapes: edges and vertices
        Assert.Equal(26, aSolid.SubShapes().Extent());
        Assert.Equal(6, aSolid.SubShapes().Count(i => aDS.ShapeInfo(i).ShapeType() == TopAbs_FACE));
        Assert.False(aSolid.Box().IsVoid());
        Assert.False(aSolid.Box().IsOpen());

        // Init has replaced the solid's shells by their faces, edges and vertices: building the solid's box again
        // finds no shell and gives a void box, as in OCCT
        Bnd_Box aBox = new();
        aDS.BuildBndBoxSolid(0, ref aBox);
        Assert.True(aBox.IsVoid());

        // with two arguments the shells stay, and the box is built from the faces' boxes
        BOPDS_DS aDS2 = MakeDS(Box(0, 0, 0), Box(20, 0, 0));
        Bnd_Box aBox2 = new();
        aDS2.BuildBndBoxSolid(0, ref aBox2);
        Assert.False(aBox2.IsVoid());
        Assert.False(aBox2.IsWhole());
    }

    [Fact]
    public void Init_OneInvertedSolid_HasTheWholeBox()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0).Reversed());
        Assert.True(aDS.ShapeInfo(0).Box().IsWhole());
    }

    [Fact]
    public void Init_SameArgumentTwice_IsOneRange()
    {
        TopoDS_Shape aBox = Box(0, 0, 0);
        BOPDS_DS aDS = MakeDS(aBox, aBox);
        Assert.Equal(1, aDS.NbRanges());
        Assert.Equal(34, aDS.NbSourceShapes());

        BOPDS_DS anEmpty = MakeDS();
        Assert.Equal(0, anEmpty.NbShapes());
        Assert.Equal(0, anEmpty.NbRanges());

        aDS.Clear();
        Assert.Equal(0, aDS.NbShapes());
        Assert.True(aDS.Arguments().IsEmpty());
    }

    [Fact]
    public void PaveBlocks_OfAnEdge()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0));
        int nE = IndicesOfType(aDS, TopAbs_EDGE)[0];
        Assert.False(aDS.HasPaveBlocks(nE));
        Assert.True(aDS.PaveBlocks(nE).IsEmpty());

        NCollection_List<BOPDS_PaveBlock> aLPB = aDS.ChangePaveBlocks(nE);
        Assert.True(aDS.HasPaveBlocks(nE));
        Assert.Same(aLPB, aDS.PaveBlocks(nE));
        Assert.Equal(1, aLPB.Extent());
        Assert.Equal(1, aDS.PaveBlocksPool().Length());

        BOPDS_PaveBlock aPB = aLPB.First();
        Assert.Equal(nE, aPB.OriginalEdge());
        Assert.Equal(-1, aPB.Edge());
        Assert.False(aPB.HasEdge());
        Assert.True(aPB.IsSplitEdge());
        Assert.False(aPB.IsToUpdate());
        Assert.False(aPB.HasShrunkData());
        aPB.Range(out double aT1, out double aT2);
        Assert.True(aT1 < aT2);
        BRep_Tool.Range(TopoDS.Edge(aDS.Shape(nE)), out double aF, out double aL);
        Assert.Equal((aF, aL), (aT1, aT2));
        aPB.Indices(out int nV1, out int nV2);
        Assert.Contains(nV1, aDS.ShapeInfo(nE).SubShapes());
        Assert.Contains(nV2, aDS.ShapeInfo(nE).SubShapes());
        Assert.NotEqual(nV1, nV2);

        NCollection_List<BOPDS_Pave> aPaves = new();
        aDS.Paves(nE, aPaves);
        Assert.Equal(new[] { aPB.Pave1(), aPB.Pave2() }, aPaves.ToArray());

        // The vertex/edge map makes the pave blocks of every edge of a vertex
        aDS.InitPaveBlocksForVertex(nV1);
        int aNbEdgesOfV1 = IndicesOfType(aDS, TopAbs_EDGE).Count(e => aDS.ShapeInfo(e).HasSubShape(nV1));
        Assert.Equal(3, aNbEdgesOfV1);
        Assert.Equal(3, aDS.PaveBlocksPool().Length());
    }

    [Fact]
    public void PaveBlock_ExtraPavesAndUpdate()
    {
        BOPDS_PaveBlock aPB = new();
        aPB.SetOriginalEdge(5);
        aPB.SetPave1(new BOPDS_Pave(1, 0.0));
        aPB.SetPave2(new BOPDS_Pave(2, 10.0));

        aPB.AppendExtPave(new BOPDS_Pave(7, 6.0));
        aPB.AppendExtPave(new BOPDS_Pave(7, 6.5)); // the same vertex again: ignored
        aPB.AppendExtPave(new BOPDS_Pave(8, 3.0));
        Assert.Equal(2, aPB.ExtPaves().Extent());
        Assert.True(aPB.IsToUpdate());

        int anInd = -5;
        Assert.True(aPB.ContainsParameter(3.05, 0.1, ref anInd));
        Assert.Equal(8, anInd);
        anInd = -5;
        Assert.False(aPB.ContainsParameter(4.0, 0.1, ref anInd));
        Assert.Equal(-5, anInd);

        aPB.RemoveExtPave(7);
        Assert.Equal(1, aPB.ExtPaves().Extent());
        aPB.AppendExtPave(new BOPDS_Pave(7, 6.0));

        NCollection_List<BOPDS_PaveBlock> aLPB = new();
        aPB.Update(aLPB);
        Assert.False(aPB.IsToUpdate());
        Assert.Equal(3, aLPB.Extent());
        double[] aExpected = [0.0, 3.0, 6.0, 10.0];
        int i = 0;
        foreach (BOPDS_PaveBlock aNew in aLPB)
        {
            aNew.Range(out double aT1, out double aT2);
            Assert.Equal((aExpected[i], aExpected[i + 1]), (aT1, aT2));
            Assert.Equal(5, aNew.OriginalEdge());
            ++i;
        }
        Assert.True(aLPB.First().HasSameBounds(aLPB.First()));
        Assert.False(aLPB.First().HasSameBounds(aLPB.Last()));

        // Without the flag, only the extra paves make blocks; fewer than two make none
        BOPDS_PaveBlock aPB2 = new();
        aPB2.AppendExtPave1(new BOPDS_Pave(1, 1.0));
        NCollection_List<BOPDS_PaveBlock> aLPB2 = new();
        aPB2.Update(aLPB2, false);
        Assert.True(aLPB2.IsEmpty());
        Assert.False(aPB2.IsToUpdate());

        // Shrunk data
        Bnd_Box aBox = new();
        aBox.Add(new gp_Pnt(1, 2, 3));
        aPB.SetShrunkData(0.1, 9.9, aBox, true);
        Assert.True(aPB.HasShrunkData());
        Assert.True(aPB.IsSplittable());
        aPB.ShrunkData(out double aTS1, out double aTS2, out Bnd_Box aShrunkBox, out bool isSplittable);
        Assert.Equal((0.1, 9.9, true), (aTS1, aTS2, isSplittable));
        Assert.Equal(aBox, aShrunkBox);

        using StringWriter aWriter = new();
        aPB.Dump(aWriter);
        Assert.StartsWith(" PB:{ E:-1 orE:5 Pave1: { 1 0.000  } Pave2:", aWriter.ToString());
    }

    [Fact]
    public void UpdatePaveBlocks_SplitsTheEdgesWithExtraPaves()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0));
        int nE = IndicesOfType(aDS, TopAbs_EDGE)[0];
        BOPDS_PaveBlock aPB = aDS.ChangePaveBlocks(nE).First();
        aPB.Range(out double aT1, out double aT2);
        int nV = IndicesOfType(aDS, TopAbs_VERTEX).First(v => !aDS.ShapeInfo(nE).HasSubShape(v));
        aPB.AppendExtPave(new BOPDS_Pave(nV, 0.5 * (aT1 + aT2)));

        aDS.UpdatePaveBlocks();
        NCollection_List<BOPDS_PaveBlock> aLPB = aDS.PaveBlocks(nE);
        Assert.Equal(2, aLPB.Extent());
        Assert.DoesNotContain(aPB, aLPB);
        aLPB.First().Indices(out _, out int n2);
        Assert.Equal(nV, n2);

        NCollection_List<BOPDS_Pave> aPaves = new();
        aDS.Paves(nE, aPaves);
        Assert.Equal(3, aPaves.Extent());
        Assert.Equal(nV, aPaves.ToArray()[1].Index());

        // UpdatePaveBlock splits one pave block in its edge's list
        BOPDS_PaveBlock aPBFirst = aLPB.First();
        aPBFirst.Range(out double aF, out double aL);
        aPBFirst.AppendExtPave(new BOPDS_Pave(IndicesOfType(aDS, TopAbs_VERTEX).Last(), 0.5 * (aF + aL)));
        aDS.UpdatePaveBlock(aPBFirst);
        Assert.Equal(3, aDS.PaveBlocks(nE).Extent());
        Assert.DoesNotContain(aPBFirst, aDS.PaveBlocks(nE));
    }

    [Fact]
    public void ReleasePaveBlocks_ForgetsUntouchedEdges()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0));
        List<int> anEdges = IndicesOfType(aDS, TopAbs_EDGE);
        aDS.ChangePaveBlocks(anEdges[0]);
        aDS.ChangePaveBlocks(anEdges[1]);
        aDS.ReleasePaveBlocks();
        Assert.False(aDS.HasPaveBlocks(anEdges[0]));
        Assert.False(aDS.HasPaveBlocks(anEdges[1]));
        Assert.True(aDS.PaveBlocksPool().Value(0).IsEmpty());
    }

    [Fact]
    public void CommonBlocks()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0), Box(20, 0, 0));
        int nE1 = IndicesOfType(aDS, TopAbs_EDGE, 0)[0];
        int nE2 = IndicesOfType(aDS, TopAbs_EDGE, 1)[0];
        BOPDS_PaveBlock aPB1 = aDS.ChangePaveBlocks(nE1).First();
        BOPDS_PaveBlock aPB2 = aDS.ChangePaveBlocks(nE2).First();

        BOPDS_CommonBlock aCB = new();
        aCB.AddPaveBlock(aPB2);
        aCB.AddPaveBlock(aPB1); // the smaller original edge goes first
        Assert.Same(aPB1, aCB.PaveBlock1());
        Assert.True(aCB.Contains(aPB2));
        Assert.False(aCB.Contains(new BOPDS_PaveBlock()));
        Assert.True(aCB.IsPaveBlockOnEdge(nE2));
        Assert.Same(aPB2, aCB.PaveBlockOnEdge(nE2));
        Assert.Null(aCB.PaveBlockOnEdge(1000));

        aCB.SetRealPaveBlock(aPB2);
        Assert.Same(aPB2, aCB.PaveBlock1());
        Assert.Equal(2, aCB.PaveBlocks().Extent());
        Assert.Same(aPB1, aCB.PaveBlocks().Last());

        NCollection_List<int> aFaces = new();
        aFaces.Append(3);
        aCB.SetFaces(aFaces);
        aCB.AddFace(4);
        Assert.Equal(1, aFaces.Extent()); // SetFaces copies
        Assert.True(aCB.Contains(4));
        Assert.True(aCB.IsPaveBlockOnFace(3));
        Assert.False(aCB.IsPaveBlockOnFace(5));
        NCollection_List<int> aMore = new();
        aMore.Append(9);
        aCB.AppendFaces(aMore);
        Assert.True(aMore.IsEmpty()); // OCCT's list Append moves the items
        Assert.Equal(3, aCB.Faces().Extent());

        aCB.SetEdge(70);
        Assert.Equal(70, aCB.Edge());
        Assert.Equal(70, aPB1.Edge());
        aCB.SetTolerance(1e-4);
        Assert.Equal(1e-4, aCB.Tolerance());

        Assert.False(aDS.IsCommonBlock(aPB1));
        Assert.Null(aDS.CommonBlock(aPB1));
        Assert.Same(aPB1, aDS.RealPaveBlock(aPB1));
        aDS.SetCommonBlock(aPB1, aCB);
        aDS.SetCommonBlock(aPB2, aCB);
        Assert.True(aDS.IsCommonBlock(aPB1));
        Assert.Same(aCB, aDS.CommonBlock(aPB1));
        Assert.Same(aPB2, aDS.RealPaveBlock(aPB1));
        Assert.True(aDS.IsCommonBlockOnEdge(aPB1));

        using StringWriter aWriter = new();
        aCB.Dump(aWriter);
        Assert.StartsWith(" -- CB:\n PB:{ E:70", aWriter.ToString());
        Assert.Contains(" Faces: 3 4 9\n", aWriter.ToString());
    }

    [Fact]
    public void UpdateCommonBlock_MakesNewCommonBlocksOfTheSplits()
    {
        // Two boxes sharing a face: the edges of the shared square coincide pairwise
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0), Box(10, 0, 0));
        int nE1 = -1, nE2 = -1;
        foreach (int e1 in IndicesOfType(aDS, TopAbs_EDGE, 0))
        {
            foreach (int e2 in IndicesOfType(aDS, TopAbs_EDGE, 1))
            {
                BRepAdaptor_Curve aC1 = new(TopoDS.Edge(aDS.Shape(e1)));
                BRepAdaptor_Curve aC2 = new(TopoDS.Edge(aDS.Shape(e2)));
                gp_Pnt a1F = aC1.Value(aC1.FirstParameter()), a1L = aC1.Value(aC1.LastParameter());
                gp_Pnt a2F = aC2.Value(aC2.FirstParameter()), a2L = aC2.Value(aC2.LastParameter());
                if (a1F.Distance(a2F) < 1e-9 && a1L.Distance(a2L) < 1e-9)
                {
                    (nE1, nE2) = (e1, e2);
                }
            }
        }
        Assert.True(nE1 >= 0);

        BOPDS_PaveBlock aPB1 = aDS.ChangePaveBlocks(nE1).First();
        BOPDS_PaveBlock aPB2 = aDS.ChangePaveBlocks(nE2).First();
        aPB1.Indices(out int n11, out int n12);
        aPB2.Indices(out int n21, out int n22);
        // The vertices coincide: same-domain them as the pave filler would
        aDS.AddShapeSD(n21, n11);
        aDS.AddShapeSD(n22, n12);
        aDS.UpdatePaveBlockWithSDVertices(aPB2);
        aPB2.Indices(out n21, out n22);
        Assert.Equal((n11, n12), (n21, n22));

        BOPDS_CommonBlock aCB = new();
        aCB.AddPaveBlock(aPB1);
        aCB.AddPaveBlock(aPB2);
        aCB.AddFace(1);
        aDS.SetCommonBlock(aPB1, aCB);
        aDS.SetCommonBlock(aPB2, aCB);

        // A new vertex in the middle of both
        aPB1.Range(out double aT1, out double aT2);
        int nVNew = aDS.Append(new BRepBuilderAPI_MakeVertex(new gp_Pnt(50, 50, 50)).Vertex());
        aDS.ChangeShapeInfo(nVNew).SetShapeType(TopAbs_VERTEX);
        Assert.True(aDS.IsNewShape(nVNew));
        aPB1.AppendExtPave(new BOPDS_Pave(nVNew, 0.5 * (aT1 + aT2)));
        aPB2.Range(out aT1, out aT2);
        aPB2.AppendExtPave(new BOPDS_Pave(nVNew, 0.5 * (aT1 + aT2)));

        aDS.UpdateCommonBlock(aCB, Precision.Confusion());
        Assert.Equal(2, aDS.PaveBlocks(nE1).Extent());
        Assert.Equal(2, aDS.PaveBlocks(nE2).Extent());
        foreach (BOPDS_PaveBlock aSplit in aDS.PaveBlocks(nE1))
        {
            BOPDS_CommonBlock? aNewCB = aDS.CommonBlock(aSplit);
            Assert.NotNull(aNewCB);
            Assert.NotSame(aCB, aNewCB);
            Assert.Equal(2, aNewCB!.PaveBlocks().Extent());
            Assert.True(aNewCB.Contains(1));
        }
    }

    [Fact]
    public void SameDomainChains()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0));
        aDS.AddShapeSD(5, 5); // ignored
        Assert.Equal(5, aDS.GetSameDomainIndex(5));
        aDS.AddShapeSD(5, 6);
        aDS.AddShapeSD(6, 7);
        Assert.Equal(7, aDS.GetSameDomainIndex(5));
        int aSD = -1;
        Assert.True(aDS.HasShapeSD(5, ref aSD));
        Assert.Equal(7, aSD);
        aSD = -1;
        Assert.False(aDS.HasShapeSD(7, ref aSD));
        Assert.Equal(-1, aSD);
        Assert.Equal(2, aDS.ShapesSD().Extent());
    }

    [Fact]
    public void Interferences()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0), Box(5, 5, 5));
        Assert.Equal(10, BOPDS_DS.NbInterfTypes());
        Assert.True(aDS.AddInterf(3, 40));
        Assert.False(aDS.AddInterf(40, 3));
        Assert.True(aDS.HasInterf(3));
        Assert.True(aDS.HasInterf(40));
        Assert.False(aDS.HasInterf(4));
        Assert.True(aDS.HasInterf(40, 3));
        Assert.False(aDS.HasInterf(3, 41));
        Assert.Equal(1, aDS.Interferences().Extent());

        BOPDS_InterfVV aVV = aDS.InterfVV().Append(new BOPDS_InterfVV());
        aVV.SetIndices(3, 40);
        aVV.SetIndexNew(70);
        BOPDS_InterfVV aRead = aDS.InterfVV().Value(0);
        Assert.Same(aVV, aRead);
        Assert.Equal(40, aRead.OppositeIndex(3));
        Assert.Equal(3, aRead.OppositeIndex(40));
        Assert.Equal(-1, aRead.OppositeIndex(1));
        Assert.True(aRead.Contains(40));
        Assert.True(aRead.HasIndexNew());
        Assert.True(aRead.HasIndexNew(out int nNew));
        Assert.Equal(70, nNew);
        Assert.Equal(70, aRead.GetIndexNew());

        BOPDS_InterfVE aVE = aDS.InterfVE().Append(new BOPDS_InterfVE());
        aVE.SetParameter(2.5);
        Assert.Equal(2.5, aDS.InterfVE().Value(0).Parameter());
        Assert.Null(aVE.GetIndexNew());
        Assert.False(aVE.HasIndexNew());

        BOPDS_InterfVF aVF = aDS.InterfVF().Append(new BOPDS_InterfVF());
        aVF.SetUV(0.25, 0.75);
        aVF.UV(out double aU, out double aV);
        Assert.Equal((0.25, 0.75), (aU, aV));

        BOPDS_InterfFF aFF = aDS.InterfFF().Append(new BOPDS_InterfFF());
        aFF.Init(2, 1);
        aFF.SetTangentFaces(true);
        BOPDS_Curve aCurve = aFF.ChangeCurves().Append(new BOPDS_Curve());
        aCurve.SetTolerance(1e-5);
        aCurve.InitPaveBlock1();
        aCurve.InitPaveBlock1(); // only once
        Assert.Equal(1, aCurve.PaveBlocks().Extent());
        ref BOPDS_PaveBlock aPB1Ref = ref aCurve.ChangePaveBlock1();
        BOPDS_PaveBlock aReplacement = new();
        aReplacement.SetEdge(12);
        aPB1Ref = aReplacement;
        Assert.Same(aReplacement, aCurve.PaveBlocks().First());
        Assert.True(aCurve.HasEdge());
        aFF.ChangePoints().Appended().SetIndex(9);
        Assert.True(aDS.InterfFF().Value(0).TangentFaces());
        Assert.Equal(1e-5, aDS.InterfFF().Value(0).Curves().Value(0).Tolerance());
        Assert.Equal(9, aDS.InterfFF().Value(0).Points().Value(0).Index());

        aDS.InterfEE().Append(new BOPDS_InterfEE());
        aDS.InterfEF().Append(new BOPDS_InterfEF());
        aDS.InterfVZ().Append(new BOPDS_InterfVZ());
        aDS.InterfEZ().Append(new BOPDS_InterfEZ());
        aDS.InterfFZ().Append(new BOPDS_InterfFZ());
        aDS.InterfZZ().Append(new BOPDS_InterfZZ());
        Assert.Equal(1, aDS.InterfZZ().Length());

        // HasInterfShapeSubShapes / HasInterfSubShapes on a face of the first box
        int nF = IndicesOfType(aDS, TopAbs_FACE, 0)[0];
        List<int> aFaceSubs = [.. aDS.ShapeInfo(nF).SubShapes()];
        Assert.False(aDS.HasInterfShapeSubShapes(50, nF));
        aDS.AddInterf(50, aFaceSubs[0]);
        Assert.True(aDS.HasInterfShapeSubShapes(50, nF));
        Assert.False(aDS.HasInterfShapeSubShapes(50, nF, false));
        foreach (int nS in aFaceSubs)
        {
            aDS.AddInterf(50, nS);
        }
        Assert.True(aDS.HasInterfShapeSubShapes(50, nF, false));

        aDS.Clear();
        Assert.Equal(0, aDS.InterfVV().Length());
        Assert.Equal(0, aDS.Interferences().Extent());
        Assert.False(aDS.HasInterf(3));
    }

    [Fact]
    public void InterfEE_SetCommonPartTwice_AccumulatesTheSecondEdgeRanges()
    {
        // OCCT: SetCommonPart is IntTools_CommonPrt's assignment, which appends the second edge's ranges to the
        // ones already there (occt-issues.md, unit 6A)
        IntTools_CommonPrt aCP = new();
        aCP.SetType(TopAbs_EDGE);
        aCP.AppendRange2(0.0, 1.0);
        BOPDS_InterfEE anEE = new();
        anEE.SetCommonPart(aCP);
        Assert.Equal(1, anEE.CommonPart().Ranges2().Length());
        anEE.SetCommonPart(aCP);
        Assert.Equal(2, anEE.CommonPart().Ranges2().Length());
        Assert.Equal(TopAbs_EDGE, anEE.CommonPart().Type());
    }

    [Fact]
    public void FaceInfo_OnInAndAloneVertices()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0), Box(20, 0, 0));
        int nF = IndicesOfType(aDS, TopAbs_FACE, 0)[0];
        foreach (int nE in aDS.ShapeInfo(nF).SubShapes().Where(i => aDS.ShapeInfo(i).ShapeType() == TopAbs_EDGE))
        {
            aDS.ChangePaveBlocks(nE);
        }

        Assert.False(aDS.HasFaceInfo(nF));
        Assert.Equal(-1, aDS.FaceInfo(nF).Index());
        BOPDS_FaceInfo aFI = aDS.ChangeFaceInfo(nF);
        Assert.True(aDS.HasFaceInfo(nF));
        Assert.Same(aFI, aDS.FaceInfo(nF));
        Assert.Equal(nF, aFI.Index());
        Assert.Equal(4, aFI.PaveBlocksOn().Extent());
        Assert.Equal(4, aFI.VerticesOn().Extent());
        Assert.True(aFI.PaveBlocksIn().IsEmpty());
        Assert.True(aFI.VerticesIn().IsEmpty());

        // A vertex/face interference puts the vertex In the face
        int nV = IndicesOfType(aDS, TopAbs_VERTEX, 1)[0];
        BOPDS_InterfVF aVF = aDS.InterfVF().Append(new BOPDS_InterfVF());
        aVF.SetIndices(nV, nF);
        aDS.UpdateFaceInfoIn(nF);
        Assert.True(aFI.VerticesIn().Contains(nV));
        NCollection_IntMap aFaces = new();
        aFaces.Add(nF);
        aDS.UpdateFaceInfoIn(aFaces);
        Assert.True(aFI.VerticesIn().Contains(nV));
        aDS.UpdateFaceInfoOn(aFaces);
        Assert.Equal(4, aFI.PaveBlocksOn().Extent());

        // the vertex In the face is alone: no In or Sc pave block ends there
        NCollection_List<int> anAlone = new();
        aDS.AloneVertices(nF, anAlone);
        Assert.Equal(new[] { nV }, anAlone.ToArray());

        // RefineFaceInfoOn keeps only pave blocks with edges
        aDS.RefineFaceInfoOn();
        Assert.True(aFI.PaveBlocksOn().IsEmpty());

        // RefineFaceInfoIn removes from In what is On
        aDS.UpdateFaceInfoOn(nF);
        BOPDS_PaveBlock anOn = aFI.PaveBlocksOn().FindKey(1);
        aFI.ChangePaveBlocksIn().Add(anOn);
        aFI.ChangePaveBlocksIn().Add(new BOPDS_PaveBlock());
        aDS.RefineFaceInfoIn();
        Assert.Equal(1, aFI.PaveBlocksIn().Extent());
        Assert.False(aFI.PaveBlocksIn().Contains(anOn));

        aFI.ChangePaveBlocksSc().Add(new BOPDS_PaveBlock());
        aFI.ChangeVerticesSc().Add(3);
        aFI.Clear(); // OCCT leaves the Sc maps
        Assert.True(aFI.PaveBlocksOn().IsEmpty());
        Assert.Equal(1, aFI.PaveBlocksSc().Extent());
        Assert.Equal(1, aFI.VerticesSc().Extent());
        Assert.Equal(1, aDS.FaceInfoPool().Length());
    }

    [Fact]
    public void SubShapesOnIn_AndSharedEdges()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0));
        List<int> aFaces = IndicesOfType(aDS, TopAbs_FACE);
        foreach (int nE in IndicesOfType(aDS, TopAbs_EDGE))
        {
            aDS.ChangePaveBlocks(nE).First().SetEdge(nE);
        }
        // Two adjacent faces share one edge
        int nF1 = aFaces[0];
        int nF2 = aFaces.First(f => f != nF1 && aDS.ShapeInfo(f).SubShapes().Any(s =>
            aDS.ShapeInfo(s).ShapeType() == TopAbs_EDGE && aDS.ShapeInfo(nF1).HasSubShape(s)));
        foreach (int nF in aFaces)
        {
            aDS.ChangeFaceInfo(nF);
        }
        NCollection_List<int> aShared = new();
        aDS.SharedEdges(nF1, nF2, aShared);
        Assert.Equal(1, aShared.Extent());
        Assert.True(aDS.ShapeInfo(nF1).HasSubShape(aShared.First()));
        Assert.True(aDS.ShapeInfo(nF2).HasSubShape(aShared.First()));

        NCollection_IntMap aMVOnIn = new(), aMVCommon = new();
        NCollection_IndexedMap<BOPDS_PaveBlock> aPBOnIn = new();
        NCollection_Map<BOPDS_PaveBlock> aCommonPB = new();
        aDS.SubShapesOnIn(nF1, nF2, aMVOnIn, aMVCommon, aPBOnIn, aCommonPB);
        Assert.Equal(7, aPBOnIn.Extent());
        Assert.Equal(1, aCommonPB.Extent());
        Assert.Equal(6, aMVOnIn.Extent());
        Assert.Equal(2, aMVCommon.Extent());
    }

    [Fact]
    public void IsValidShrunkData()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0));
        int nE = IndicesOfType(aDS, TopAbs_EDGE)[0];
        BOPDS_PaveBlock aPB = aDS.ChangePaveBlocks(nE).First();
        Assert.False(aDS.IsValidShrunkData(aPB));
        aPB.Range(out double aT1, out double aT2);
        Bnd_Box aBox = new();
        aBox.Add(new gp_Pnt(0, 0, 0));
        // the shrunk range ends well outside the vertices' tolerances
        aPB.SetShrunkData(aT1 + 1.0, aT2 - 1.0, aBox, true);
        Assert.True(aDS.IsValidShrunkData(aPB));
        // ends inside a vertex's tolerance: no longer valid
        aPB.SetShrunkData(aT1 + 1e-9, aT2 - 1.0, aBox, true);
        Assert.False(aDS.IsValidShrunkData(aPB));
    }

    [Fact]
    public void Dump_WritesTheRangesAndShapes()
    {
        BOPDS_DS aDS = MakeDS(Box(0, 0, 0));
        using StringWriter aWriter = new();
        aDS.Dump(aWriter);
        string aText = aWriter.ToString();
        Assert.StartsWith(" *** DS ***\n Ranges:1\n range: 0 33\n Shapes:34\n 0 : SOLID {", aText);
        Assert.Contains(" ****** adds\n", aText);
        Assert.EndsWith(" ******\n", aText);

        using StringWriter aShapeWriter = new();
        int nV = Enumerable.Range(0, aDS.NbShapes()).First(i => aDS.ShapeInfo(i).ShapeType() == TopAbs_VERTEX);
        aDS.ShapeInfo(nV).Dump(aShapeWriter);
        Assert.Equal(" VERTEX { }", aShapeWriter.ToString());
    }
}
