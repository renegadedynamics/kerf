// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// A text dump of the data structure after the pave filler, written the same way by tools/pfrec's native recorder
/// (decision 785): the shapes (types, sub-shapes, tolerances, points, ranges and three curve points of new edges, the
/// faces on which new edges have p-curves), the same-domain vertices, every source edge's pave blocks with their
/// common blocks, the face information, and every interference (VV, VE, VF, EE, EF and FF with its curves, their pave
/// blocks and points). Every double is its 64 bits in hex.
/// </summary>
/// <remarks>
/// Lists the DS keeps in an order are written in that order (sub-shape lists of new shapes, pave blocks, the face
/// information's pave-block maps, which are indexed maps, curves, points). Sets OCCT keeps in hashed maps
/// (<c>NCollection_Map&lt;int&gt;</c>: a face's vertices) and the sub-shapes of source shapes are written sorted, so
/// the dump doesn't depend on hash order. Common blocks are numbered in the order the edges' pave blocks meet them.
/// </remarks>
internal static class PaveFillerDump
{
    private static string H(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16");

    private static string P(gp_Pnt theP) => H(theP.X()) + " " + H(theP.Y()) + " " + H(theP.Z());

    private static string P2(gp_Pnt2d theP) => H(theP.X()) + " " + H(theP.Y());

    private static string Sorted(IEnumerable<int> theValues) => string.Join(",", theValues.OrderBy(x => x));

    private static string PB(BOPDS_PaveBlock thePB)
    {
        thePB.Indices(out int nV1, out int nV2);
        return string.Format("[{0} {1} {2} {3} {4} {5}]", thePB.OriginalEdge(), thePB.Edge(), nV1, H(thePB.Pave1().Parameter()),
                             nV2, H(thePB.Pave2().Parameter()));
    }

    private static string CP(IntTools_CommonPrt theCP)
    {
        StringBuilder aB = new();
        aB.Append((int)theCP.Type());
        aB.Append(' ').Append(H(theCP.Range1().First())).Append(' ').Append(H(theCP.Range1().Last()));
        NCollection_Sequence<IntTools_Range> aR2 = theCP.Ranges2();
        aB.Append(" r2:");
        for (int i = 1; i <= aR2.Length(); ++i)
        {
            aB.Append(' ').Append(H(aR2.Value(i).First())).Append(' ').Append(H(aR2.Value(i).Last()));
        }
        aB.Append(" vp ").Append(H(theCP.VertexParameter1())).Append(' ').Append(H(theCP.VertexParameter2()));
        return aB.ToString();
    }

    public static string Write(BOPDS_DS theDS)
    {
        StringBuilder aB = new();
        int aNb = theDS.NbShapes(), aNbSrc = theDS.NbSourceShapes();
        aB.Append("ds ").Append(aNb).Append(' ').Append(aNbSrc).Append('\n');
        List<int> aFaces = [];
        for (int i = 0; i < aNb; ++i)
        {
            BOPDS_ShapeInfo aSI = theDS.ShapeInfo(i);
            TopoDS_Shape aS = aSI.Shape();
            TopAbs_ShapeEnum aType = aSI.ShapeType();
            aB.Append("s ").Append(i).Append(' ').Append((int)aType).Append(' ').Append((int)aS.Orientation());
            List<int> aSub = [.. aSI.SubShapes()];
            aB.Append(" sub ").Append(i < aNbSrc ? Sorted(aSub) : string.Join(",", aSub));
            if (aType == TopAbs_VERTEX)
            {
                TopoDS_Vertex aV = TopoDS.Vertex(aS);
                aB.Append(" tol ").Append(H(BRep_Tool.Tolerance(aV))).Append(" p ").Append(P(BRep_Tool.Pnt(aV)));
            }
            else if (aType == TopAbs_EDGE)
            {
                TopoDS_Edge aE = TopoDS.Edge(aS);
                aB.Append(" tol ").Append(H(BRep_Tool.Tolerance(aE))).Append(" deg ").Append(BRep_Tool.Degenerated(aE) ? 1 : 0);
                if (i >= aNbSrc)
                {
                    Geom_Curve? aC = BRep_Tool.Curve(aE, out double aT1, out double aT2);
                    aB.Append(" r ").Append(H(aT1)).Append(' ').Append(H(aT2));
                    if (aC is not null)
                    {
                        aB.Append(" c ").Append(P(aC.Value(aT1))).Append(' ').Append(P(aC.Value((aT1 + aT2) * 0.5)))
                          .Append(' ').Append(P(aC.Value(aT2)));
                    }
                }
            }
            else if (aType == TopAbs_FACE)
            {
                aB.Append(" tol ").Append(H(BRep_Tool.Tolerance(TopoDS.Face(aS))));
                aFaces.Add(i);
            }
            aB.Append('\n');
        }
        // p-curves of the new edges on the faces
        for (int i = aNbSrc; i < aNb; ++i)
        {
            if (theDS.ShapeInfo(i).ShapeType() != TopAbs_EDGE)
            {
                continue;
            }
            TopoDS_Edge aE = TopoDS.Edge(theDS.Shape(i));
            if (BRep_Tool.Degenerated(aE))
            {
                continue;
            }
            foreach (int nF in aFaces)
            {
                TopoDS_Face aF = TopoDS.Face(theDS.Shape(nF));
                Geom2d_Curve? aC2 = BRep_Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2, out bool anIsStored);
                if (aC2 is not null && anIsStored)
                {
                    aB.Append("pc ").Append(i).Append(' ').Append(nF).Append(' ').Append(H(aT1)).Append(' ').Append(H(aT2))
                      .Append(' ').Append(P2(aC2.Value((aT1 + aT2) * 0.5))).Append('\n');
                }
            }
        }
        // same domain shapes
        for (int i = 0; i < aNb; ++i)
        {
            int nSD = -1;
            if (theDS.HasShapeSD(i, ref nSD))
            {
                aB.Append("sd ").Append(i).Append(' ').Append(nSD).Append('\n');
            }
        }
        // pave blocks and common blocks
        Dictionary<BOPDS_CommonBlock, int> aCBs = new(ReferenceEqualityComparer.Instance);
        List<BOPDS_CommonBlock> aCBList = [];
        for (int i = 0; i < aNb; ++i)
        {
            BOPDS_ShapeInfo aSI = theDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_EDGE || !theDS.HasPaveBlocks(i))
            {
                continue;
            }
            foreach (BOPDS_PaveBlock aPB in theDS.PaveBlocks(i))
            {
                aB.Append("pb ").Append(i).Append(' ').Append(PB(aPB));
                if (aPB.HasShrunkData())
                {
                    aPB.ShrunkData(out double aTS1, out double aTS2, out Bnd_Box _, out bool aSplittable);
                    aB.Append(" sh ").Append(H(aTS1)).Append(' ').Append(H(aTS2)).Append(' ').Append(aSplittable ? 1 : 0);
                }
                BOPDS_CommonBlock? aCB = theDS.CommonBlock(aPB);
                if (aCB is not null)
                {
                    if (!aCBs.TryGetValue(aCB, out int anId))
                    {
                        anId = aCBs.Count;
                        aCBs.Add(aCB, anId);
                        aCBList.Add(aCB);
                    }
                    aB.Append(" cb ").Append(anId);
                }
                aB.Append('\n');
            }
        }
        for (int k = 0; k < aCBList.Count; ++k)
        {
            BOPDS_CommonBlock aCB = aCBList[k];
            aB.Append("cb ").Append(k).Append(' ').Append(H(aCB.Tolerance())).Append(" pbs");
            foreach (BOPDS_PaveBlock aPB in aCB.PaveBlocks())
            {
                aB.Append(' ').Append(PB(aPB));
            }
            aB.Append(" faces ").Append(Sorted(aCB.Faces())).Append('\n');
        }
        // face information
        foreach (int nF in aFaces)
        {
            if (!theDS.HasFaceInfo(nF))
            {
                continue;
            }
            BOPDS_FaceInfo aFI = theDS.FaceInfo(nF);
            aB.Append("fi ").Append(nF);
            foreach ((string aTag, NCollection_IndexedMap<BOPDS_PaveBlock> aMPB) in
                     new[] { ("on", aFI.PaveBlocksOn()), ("in", aFI.PaveBlocksIn()), ("sc", aFI.PaveBlocksSc()) })
            {
                aB.Append(' ').Append(aTag);
                for (int j = 1; j <= aMPB.Extent(); ++j)
                {
                    aB.Append(' ').Append(PB(aMPB[j]));
                }
            }
            aB.Append(" von ").Append(Sorted(aFI.VerticesOn())).Append(" vin ").Append(Sorted(aFI.VerticesIn()))
              .Append(" vsc ").Append(Sorted(aFI.VerticesSc())).Append('\n');
        }
        // interferences
        foreach (BOPDS_InterfVV aI in theDS.InterfVV())
        {
            aB.Append("vv ").Append(aI.Index1()).Append(' ').Append(aI.Index2()).Append(' ').Append(aI.IndexNew()).Append('\n');
        }
        foreach (BOPDS_InterfVE aI in theDS.InterfVE())
        {
            aB.Append("ve ").Append(aI.Index1()).Append(' ').Append(aI.Index2()).Append(' ').Append(aI.IndexNew()).Append(' ')
              .Append(H(aI.Parameter())).Append('\n');
        }
        foreach (BOPDS_InterfVF aI in theDS.InterfVF())
        {
            aI.UV(out double aU, out double aV);
            aB.Append("vf ").Append(aI.Index1()).Append(' ').Append(aI.Index2()).Append(' ').Append(aI.IndexNew()).Append(' ')
              .Append(H(aU)).Append(' ').Append(H(aV)).Append('\n');
        }
        foreach (BOPDS_InterfEE aI in theDS.InterfEE())
        {
            aB.Append("ee ").Append(aI.Index1()).Append(' ').Append(aI.Index2()).Append(' ').Append(aI.IndexNew()).Append(' ')
              .Append(CP(aI.CommonPart())).Append('\n');
        }
        foreach (BOPDS_InterfEF aI in theDS.InterfEF())
        {
            aB.Append("ef ").Append(aI.Index1()).Append(' ').Append(aI.Index2()).Append(' ').Append(aI.IndexNew()).Append(' ')
              .Append(CP(aI.CommonPart())).Append('\n');
        }
        foreach (BOPDS_InterfFF aI in theDS.InterfFF())
        {
            NCollection_DynamicArray<BOPDS_Curve> aVC = aI.Curves();
            NCollection_DynamicArray<BOPDS_Point> aVP = aI.Points();
            aB.Append("ff ").Append(aI.Index1()).Append(' ').Append(aI.Index2()).Append(' ').Append(aI.TangentFaces() ? 1 : 0)
              .Append(' ').Append(aVC.Length()).Append(' ').Append(aVP.Length()).Append('\n');
            foreach (BOPDS_Curve aNC in aVC)
            {
                aB.Append(" c ").Append(H(aNC.Tolerance())).Append(' ').Append(H(aNC.TangentialTolerance()))
                  .Append(" tv ").Append(string.Join(",", aNC.TechnoVertices()));
                IntTools_Curve anIC = aNC.Curve();
                double aT1 = 0, aT2 = 0;
                gp_Pnt aP1 = new(), aP2 = new();
                if (anIC.Bounds(ref aT1, ref aT2, ref aP1, ref aP2))
                {
                    aB.Append(" b ").Append(H(aT1)).Append(' ').Append(H(aT2)).Append(' ').Append(P(aP1)).Append(' ').Append(P(aP2));
                }
                aB.Append(" pbs");
                foreach (BOPDS_PaveBlock aPB in aNC.PaveBlocks())
                {
                    aB.Append(' ').Append(PB(aPB));
                }
                aB.Append('\n');
            }
            foreach (BOPDS_Point aP in aVP)
            {
                aB.Append(" p ").Append(aP.Index()).Append(' ').Append(P(aP.Pnt())).Append('\n');
            }
        }
        return aB.ToString();
    }
}
