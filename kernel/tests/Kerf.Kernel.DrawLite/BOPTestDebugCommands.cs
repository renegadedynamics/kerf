// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using System.Globalization;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BOPTest_DebugCommands.cxx: the commands that read the data structure of <c>bfillds</c> and the session's builder
/// (<c>bopds</c>, <c>bopiterator</c>, <c>bopinterf</c>, <c>bopnews</c>, <c>bopwho</c>, <c>bopindex</c>, <c>bopsd</c>,
/// <c>bopsc</c>, <c>boppb</c>, <c>bopcb</c>, <c>bopsp</c>, <c>bopfon</c>, <c>bopfin</c>, <c>bopfsc</c>, <c>bopfav</c>,
/// <c>bopimage</c>, <c>boporigin</c>, <c>bopfsd</c>) and the area builders' <c>bopbface</c> and <c>bopbsolid</c>
/// (unit 6L).
/// </summary>
/// <remarks>
/// BOPTest_DrawableShape (a shape drawn with its name as a label) is a plain shape variable: Draw-lite has no viewer.
/// What OCCT prints with printf (the pave and common block dumps, <c>bopf*</c>) goes to the interpreter's output, as
/// the other commands' std::cout.
/// </remarks>
internal static class BOPTestDebugCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bopds", BopDS);
        DrawCxx.Add(theInterpreter, "bopiterator", BopIterator);
        DrawCxx.Add(theInterpreter, "bopinterf", BopInterf);
        DrawCxx.Add(theInterpreter, "bopnews", BopNews);
        DrawCxx.Add(theInterpreter, "bopwho", BopWho);
        DrawCxx.Add(theInterpreter, "bopindex", BopIndex);
        DrawCxx.Add(theInterpreter, "bopsd", BopSD);
        DrawCxx.Add(theInterpreter, "bopsc", BopSC);
        DrawCxx.Add(theInterpreter, "boppb", BopPB);
        DrawCxx.Add(theInterpreter, "bopcb", BopCB);
        DrawCxx.Add(theInterpreter, "bopsp", BopSP);
        DrawCxx.Add(theInterpreter, "bopfon", (i, di, a) => BopFInfo(i, di, a, 0));
        DrawCxx.Add(theInterpreter, "bopfin", (i, di, a) => BopFInfo(i, di, a, 1));
        DrawCxx.Add(theInterpreter, "bopfsc", (i, di, a) => BopFInfo(i, di, a, 2));
        DrawCxx.Add(theInterpreter, "bopfav", BopFAV);
        DrawCxx.Add(theInterpreter, "bopimage", BopImage);
        DrawCxx.Add(theInterpreter, "boporigin", BopOrigin);
        DrawCxx.Add(theInterpreter, "bopfsd", BopFSD);
        DrawCxx.Add(theInterpreter, "bopbface", BopBFace);
        DrawCxx.Add(theInterpreter, "bopbsolid", BopBSolid);
    }

    private static string Z(int theIndex) => "z" + theIndex.ToString(CultureInfo.InvariantCulture);

    private static int BopDS(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n > 2)
        {
            di.Append("Shows the shapes from DS. Use: bopds [v/e/w/f/sh/s/cs/c]\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        int aNbS = pDS.SourceShapeCount;
        ShapeType aType = ShapeType.Shape;
        if (n == 2)
        {
            aType = GetTypeByName(a[1]);
        }
        for (int i = 0; i < aNbS; ++i)
        {
            ShapeInfo aSI = pDS.ShapeInfo(i);
            Shape aS = aSI.Shape();
            ShapeType aTypeShape = aSI.ShapeType();
            if (n == 1)
            {
                if (aTypeShape is ShapeType.Compound or ShapeType.CompSolid
                    or ShapeType.Solid or ShapeType.Shell or ShapeType.Wire)
                {
                    continue;
                }
            }
            else
            {
                if (aTypeShape != aType)
                {
                    continue;
                }
            }
            theDI.SetShape(Z(i), aS);
        }
        return 0;
    }

    private static int BopIterator(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 1 && n != 3)
        {
            di.Append("Shows the pairs of interfered shapes. Use: bopiterator [type1 type2]\n");
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        DS? pDS = aSession.PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        DataIterator aIt = new();
        IntersectionContext aCtx = new();
        aIt.SetDS(pDS);
        aIt.Prepare(aCtx, aSession.UseOBB, aSession.FuzzyValue);
        if (n == 1)
        {
            ShapeType[] aT =
                [ShapeType.Vertex, ShapeType.Edge, ShapeType.Face, ShapeType.Solid];
            for (int i = 0; i < 4; ++i)
            {
                string aST1 = GetNameByType(aT[i]);
                for (int j = i; j < 4; ++j)
                {
                    string aST2 = GetNameByType(aT[j]);
                    aIt.Initialize(aT[i], aT[j]);
                    for (; aIt.More(); aIt.Next())
                    {
                        aIt.Value(out int n1, out int n2);
                        di.Append($"{aST1}/{aST2}: (z{n1} z{n2})\n");
                    }
                }
            }
        }
        else if (n == 3)
        {
            ShapeType aT1 = GetTypeByName(a[1]);
            ShapeType aT2 = GetTypeByName(a[2]);
            string aST1 = GetNameByType(aT1);
            string aST2 = GetNameByType(aT2);
            aIt.Initialize(aT1, aT2);
            for (; aIt.More(); aIt.Next())
            {
                aIt.Value(out int n1, out int n2);
                di.Append($"{aST1}/{aST2}: (z{n1} z{n2})\n");
            }
        }
        return 0;
    }

    private static int BopInterf(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            di.Append("Shows interferences of given type. Use: bopinterf type1 type2\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        ShapeType aT1 = GetTypeByName(a[1]);
        ShapeType aT2 = GetTypeByName(a[2]);
        bool Is(ShapeType theA, ShapeType theB) => (aT1 == theA && aT2 == theB) || (aT2 == theA && aT1 == theB);
        const ShapeType V = ShapeType.Vertex, E = ShapeType.Edge,
                               F = ShapeType.Face, S = ShapeType.Solid;
        if (aT1 == V && aT2 == V)
        {
            di.Append("V/V: ");
            DumpInterfs(pDS.InterfVV(), di);
        }
        else if (Is(V, E))
        {
            di.Append("V/E: ");
            DumpInterfs(pDS.InterfVE(), di);
        }
        else if (aT1 == E && aT2 == E)
        {
            di.Append("E/E: ");
            DumpInterfs(pDS.InterfEE(), di);
        }
        else if (Is(V, F))
        {
            di.Append("V/F: ");
            DumpInterfs(pDS.InterfVF(), di);
        }
        else if (Is(E, F))
        {
            di.Append("E/F: ");
            DumpInterfs(pDS.InterfEF(), di);
        }
        else if (aT1 == F && aT2 == F)
        {
            di.Append("F/F: ");
            DumpInterfs(pDS.InterfFF(), di);
        }
        else if (Is(V, S))
        {
            di.Append("V/S: ");
            DumpInterfs(pDS.InterfVZ(), di);
        }
        else if (Is(E, S))
        {
            di.Append("E/S: ");
            DumpInterfs(pDS.InterfEZ(), di);
        }
        else if (Is(F, S))
        {
            di.Append("F/S: ");
            DumpInterfs(pDS.InterfFZ(), di);
        }
        else if (aT1 == S && aT2 == S)
        {
            di.Append("S/S: ");
            DumpInterfs(pDS.InterfZZ(), di);
        }
        return 0;
    }

    private static int BopWho(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Shows where the new shape was created. Use: bopwho #\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 0;
        }
        int ind = theDI.Atoi(a[1]);
        if (ind <= 0)
        {
            di.Append(" Index must be grater than 0\n");
            return 1;
        }
        int i1 = 0;
        int i2 = pDS.ShapeCount;
        // OCCT: ind == NbShapes() passes the check and ShapeInfo reads past the end
        if (ind < i1 || ind > i2)
        {
            di.Append(" DS does not contain the shape\n");
            return 1;
        }
        if (!pDS.IsNewShape(ind))
        {
            int iRank = pDS.Rank(ind);
            di.Append(" Rank: ").Append(iRank).Append("\n");
            return 0;
        }
        di.Append("the shape is new\n");
        ShapeInfo aSI = pDS.ShapeInfo(ind);
        if (aSI.ShapeType() != ShapeType.Vertex)
        {
            return 0;
        }
        SearchNewIndex("V/V: ", ind, pDS.InterfVV(), di);
        SearchNewIndex("V/E: ", ind, pDS.InterfVE(), di);
        SearchNewIndex("E/E: ", ind, pDS.InterfEE(), di);
        SearchNewIndex("V/F: ", ind, pDS.InterfVF(), di);
        SearchNewIndex("E/F: ", ind, pDS.InterfEF(), di);
        bool bFound = false;
        NCollection_DynamicArray<InterfFF> aFFs = pDS.InterfFF();
        int aNb = aFFs.Length();
        for (int i = 0; i < aNb; ++i)
        {
            InterfFF anInt = aFFs[i];
            anInt.Indices(out int n1, out int n2);
            NCollection_DynamicArray<SectionCurve> aVNC = anInt.Curves();
            int aNbC = aVNC.Length();
            for (int k = 0; k < aNbC; ++k)
            {
                SectionCurve aNC = aVNC[k];
                foreach (PaveBlock aPB in aNC.PaveBlocks())
                {
                    aPB.Indices(out int nV1, out int nV2);
                    if (ind == nV1 || ind == nV2)
                    {
                        if (!bFound)
                        {
                            di.Append(" FF curves: ");
                            bFound = true;
                        }
                        di.Append($"({n1}, {n2}) ");
                        break;
                    }
                }
            }
            if (bFound)
            {
                di.Append("\n");
            }
            bFound = false;
            NCollection_DynamicArray<DataPoint> aVNP = anInt.Points();
            int aNbP = aVNP.Length();
            for (int k = 0; k < aNbP; ++k)
            {
                DataPoint aNP = aVNP[k];
                int nV1 = aNP.Index();
                if (ind == nV1)
                {
                    if (!bFound)
                    {
                        di.Append(" FF points: ");
                        bFound = true;
                    }
                    di.Append($"({n1}, {n2}) ");
                }
            }
            if (bFound)
            {
                di.Append("\n");
            }
        }
        return 0;
    }

    private static int BopNews(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Shows the newly created shapes. Use: bopnews v/e/f\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        ShapeType aType = GetTypeByName(a[1]);
        if (aType != ShapeType.Vertex && aType != ShapeType.Edge && aType != ShapeType.Face)
        {
            di.Append("Use: bopnews v/e/f\n");
            return 1;
        }
        bool bFound = false;
        int i1 = pDS.SourceShapeCount;
        int i2 = pDS.ShapeCount;
        for (int i = i1; i < i2; ++i)
        {
            ShapeInfo aSI = pDS.ShapeInfo(i);
            if (aSI.ShapeType() == aType)
            {
                theDI.SetShape(Z(i), aSI.Shape());
                di.Append(Z(i) + " ");
                bFound = true;
            }
        }
        if (bFound)
        {
            di.Append("\n");
        }
        else
        {
            di.Append(" No new shapes found\n");
        }
        return 0;
    }

    private static int BopIndex(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Gets the index of the shape in the DS. Use: bopindex s\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            di.Append(a[1]).Append(" is a null shape\n");
            return 1;
        }
        int ind = pDS.Index(aS);
        // OCCT: index 0 (the first argument) is reported as not found
        bool bFound = ind > 0;
        if (bFound)
        {
            di.Append(" Index: ").Append(ind).Append("\n");
        }
        else
        {
            di.Append(" DS does not contain the shape\n");
        }
        return 0;
    }

    private static int BopSD(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Gets the Same domain shape. Use: bopsd #\n");
            return 0;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 0;
        }
        int ind = theDI.Atoi(a[1]);
        int i1 = 0;
        int i2 = pDS.ShapeCount;
        if (ind < i1 || ind > i2)
        {
            di.Append(" DS does not contain the shape\n");
            return 0;
        }
        int iSD = 0;
        bool bHasSD = pDS.HasShapeSD(ind, ref iSD);
        if (bHasSD)
        {
            di.Append($" Shape {ind} has SD shape {iSD}\n");
        }
        else
        {
            di.Append($" Shape: {ind} has no SD shape\n");
        }
        return 0;
    }

    private static int BopSC(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n > 3)
        {
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 0;
        }
        int nSF1 = -1, nSF2 = -1;
        if (n > 1)
        {
            nSF1 = theDI.Atoi(a[1]);
        }
        if (n > 2)
        {
            nSF2 = theDI.Atoi(a[2]);
        }
        NCollection_DynamicArray<InterfFF> aFFs = pDS.InterfFF();
        int iCnt = 0;
        int iPriz = 0;
        int aNb = aFFs.Length();
        for (int j = 0; j < aNb; ++j)
        {
            InterfFF aFF = aFFs[j];
            if (n == 3)
            {
                if (!aFF.Contains(nSF1) || !aFF.Contains(nSF2))
                {
                    continue;
                }
                iPriz = 1;
            }
            else if (n == 2)
            {
                if (!aFF.Contains(nSF1))
                {
                    continue;
                }
            }
            aFF.Indices(out int nF1, out int nF2);
            int iX = 0;
            NCollection_DynamicArray<SectionCurve> aVNC = aFF.Curves();
            int aNbC = aVNC.Length();
            for (int k = 0; k < aNbC; ++k)
            {
                SectionCurve aNC = aVNC[k];
                foreach (PaveBlock aPB in aNC.PaveBlocks())
                {
                    if (!aPB.HasEdge(out int nSp))
                    {
                        continue;
                    }
                    if (iX == 0)
                    {
                        di.Append($"[{nF1} {nF2}] section edges: ");
                        iX = 1;
                    }
                    string aText = $"t_{k}_{nSp}";
                    di.Append(aText);
                    theDI.SetShape(aText, pDS.Shape(nSp));
                    di.Append(" ");
                    ++iCnt;
                }
            }
            if (iX != 0)
            {
                di.Append("\n");
            }
            iX = 0;
            NCollection_DynamicArray<DataPoint> aVNP = aFF.Points();
            int aNbP = aVNP.Length();
            for (int k = 0; k < aNbP; ++k)
            {
                DataPoint aNP = aVNP[k];
                int nSp = aNP.Index();
                if (nSp < 0)
                {
                    continue;
                }
                if (iX == 0)
                {
                    di.Append($"[{nF1} {nF2}] section vertices: ");
                    iX = 1;
                }
                string aText = $"p_{k}_{nSp}";
                di.Append(aText);
                theDI.SetShape(aText, pDS.Shape(nSp));
                di.Append(" ");
                ++iCnt;
            }
            if (iX != 0)
            {
                di.Append("\n");
            }
            if (iPriz != 0)
            {
                break;
            }
        }
        if (iCnt != 0)
        {
            di.Append("\n");
        }
        else
        {
            di.Append(" no sections found\n");
        }
        return 0;
    }

    /// <summary>The edges between i1 and i2 (one edge when an index is given) that have pave blocks (boppb, bopcb, bopsp).</summary>
    private static IEnumerable<int> EdgesWithPaveBlocks(DS pDS, IReadOnlyList<string> a, DrawInterpreter theDI)
    {
        int i1 = 0;
        int i2 = pDS.SourceShapeCount;
        if (a.Count == 2)
        {
            int ind = theDI.Atoi(a[1]);
            i1 = ind;
            i2 = ind + 1;
        }
        for (int ind = i1; ind < i2; ++ind)
        {
            ShapeInfo aSI = pDS.ShapeInfo(ind);
            if (aSI.ShapeType() != ShapeType.Edge)
            {
                continue;
            }
            if (!pDS.HasPaveBlocks(ind))
            {
                continue;
            }
            yield return ind;
        }
    }

    private static int BopPB(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count > 2)
        {
            di.Append("Shows information about pave blocks. Use: boppb [#e]\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        foreach (int ind in EdgesWithPaveBlocks(pDS, a, theDI))
        {
            foreach (PaveBlock aPB in pDS.PaveBlocks(ind))
            {
                aPB.Dump(theDI.Output);
                theDI.Output.Write("\n");
            }
        }
        return 0;
    }

    private static int BopCB(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count > 2)
        {
            di.Append("Shows information about common blocks. Use: bopcb [#e]\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        HashSet<CommonBlock> aMCB = new(ReferenceEqualityComparer.Instance);
        foreach (int ind in EdgesWithPaveBlocks(pDS, a, theDI))
        {
            foreach (PaveBlock aPB in pDS.PaveBlocks(ind))
            {
                if (pDS.IsCommonBlock(aPB))
                {
                    CommonBlock aCB = pDS.CommonBlock(aPB)!;
                    if (aMCB.Add(aCB))
                    {
                        aCB.Dump(theDI.Output);
                        theDI.Output.Write("\n");
                    }
                }
            }
        }
        return 0;
    }

    private static int BopSP(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count > 2)
        {
            di.Append("Shows the splits of edges. Use: bopsp [#e]\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        foreach (int ind in EdgesWithPaveBlocks(pDS, a, theDI))
        {
            di.Append("Edge ").Append(ind).Append(": ");
            foreach (PaveBlock aPB in pDS.PaveBlocks(ind))
            {
                int nSp = aPB.Edge();
                string aText = $"z{ind}_{nSp}";
                theDI.SetShape(aText, pDS.Shape(nSp));
                di.Append(aText).Append(" ");
            }
            di.Append("\n");
        }
        return 0;
    }

    private static int BopFInfo(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a, int iPriz)
    {
        if (a.Count != 2)
        {
            di.Append("Shows ").Append(iPriz == 0 ? "ON" : (iPriz == 1 ? "IN" : "SC"))
              .Append(" information for the face. Use: bopf* #f\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        int nF = theDI.Atoi(a[1]);
        int i1 = 0;
        int i2 = pDS.SourceShapeCount;
        // OCCT: nF == NbSourceShapes() passes the check
        if (nF < i1 || nF > i2)
        {
            di.Append(" DS does not contain the shape\n");
            return 1;
        }
        if (pDS.ShapeInfo(nF).ShapeType() != ShapeType.Face)
        {
            di.Append(" The shape is not a face\n");
            return 1;
        }
        if (!pDS.HasFaceInfo(nF))
        {
            di.Append(" The face has no face information\n");
            return 0;
        }
        FaceInfo aFI = pDS.ChangeFaceInfo(nF);
        (string aText, NCollection_IndexedMap<PaveBlock> aMPB, NCollection_IntMap aMI) = iPriz switch
        {
            0 => ("On", aFI.PaveBlocksOn(), aFI.VerticesOn()),
            1 => ("In", aFI.PaveBlocksIn(), aFI.VerticesIn()),
            _ => ("Sc", aFI.PaveBlocksSc(), aFI.VerticesSc()),
        };
        TextWriter anOut = theDI.Output;
        if (aMPB.Extent() != 0)
        {
            anOut.Write($" pave blocks {aText}:\n");
            int aNb = aMPB.Extent();
            for (int i = 1; i <= aNb; ++i)
            {
                aMPB[i].Dump(anOut);
                anOut.Write(" \n");
            }
        }
        else
        {
            anOut.Write($" no pave blocks {aText} found\n");
        }
        if (aMI.Extent() != 0)
        {
            anOut.Write($" vertices {aText}:\n");
            foreach (int nV in aMI)
            {
                anOut.Write($" {nV}");
            }
            anOut.Write(" \n");
        }
        else
        {
            anOut.Write($" no verts {aText} found\n");
        }
        return 0;
    }

    private static int BopFAV(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Shows information about alone vertices for the face. Use: bopfav #f\n");
            return 1;
        }
        DS? pDS = BOPTestObjects.Of(theDI).PDS();
        if (pDS is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 0;
        }
        int nF = theDI.Atoi(a[1]);
        int i1 = 0;
        int i2 = pDS.SourceShapeCount;
        // OCCT: nF == NbSourceShapes() passes the check
        if (nF < i1 || nF > i2)
        {
            di.Append("DS does not contain the shape\n");
            return 1;
        }
        if (pDS.ShapeInfo(nF).ShapeType() != ShapeType.Face)
        {
            di.Append(" The shape is not a face\n");
            return 1;
        }
        if (!pDS.HasFaceInfo(nF))
        {
            di.Append(" The face has no face information\n");
            return 0;
        }
        NCollection_List<int> aLI = new();
        pDS.AloneVertices(nF, aLI);
        if (aLI.Extent() == 0)
        {
            di.Append(" no alone vertices found\n");
            return 0;
        }
        di.Append(" alone vertices: \n");
        foreach (int nV in aLI)
        {
            di.Append(nV).Append(" ");
        }
        di.Append("\n");
        return 0;
    }

    private static int BopImage(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Shows split parts of the shape. Use: bopimage s\n");
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        if (aSession.PDS() is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            di.Append(a[1]).Append(" is a null shape\n");
            return 1;
        }
        Dictionary<Shape, NCollection_List<Shape>> anImages = aSession.Builder().Images();
        if (!anImages.ContainsKey(aS))
        {
            di.Append(" no images found\n");
            return 0;
        }
        BrepBuilder aBB = new();
        Compound aC = new();
        aBB.MakeCompound(ref aC);
        int i = 0;
        foreach (Shape aSIm in anImages.GetValue(aS))
        {
            aBB.Add(aC, aSIm);
            ++i;
        }
        di.Append(i).Append(" images found\n");
        string buf = a[1] + "_im";
        theDI.SetShape(buf, aC);
        di.Append(buf).Append("\n");
        return 0;
    }

    private static int BopOrigin(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Shows the original shape for the shape. Use: boporigin s\n");
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        if (aSession.PDS() is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            di.Append(a[1]).Append(" is a null shape\n");
            return 0;
        }
        Dictionary<Shape, NCollection_List<Shape>> aDMI = aSession.Builder().Origins();
        if (!aDMI.ContainsKey(aS))
        {
            di.Append(" no origins found\n");
            return 0;
        }
        string buf = a[1] + "_or";
        NCollection_List<Shape> aLSx = aDMI.GetValue(aS);
        if (aLSx.Extent() == 1)
        {
            theDI.SetShape(buf, aLSx.First());
            di.Append("1 origin found\n").Append(buf).Append("\n");
            return 0;
        }
        Compound aCOr = new();
        new BrepBuilder().MakeCompound(ref aCOr);
        foreach (Shape aSx in aLSx)
        {
            new BrepBuilder().Add(aCOr, aSx);
        }
        theDI.SetShape(buf, aCOr);
        di.Append(aLSx.Extent()).Append(" origins found\n");
        di.Append(buf).Append("\n");
        return 0;
    }

    private static int BopFSD(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Shows SD faces for the face: Use: bopfsd f\n");
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        if (aSession.PDS() is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            di.Append(a[1]).Append(" is a null shape\n");
            return 1;
        }
        Dictionary<Shape, Shape> aDMSD = aSession.Builder().ShapesSD();
        if (!aDMSD.ContainsKey(aS))
        {
            di.Append(" shape has no sd shape\n");
            return 0;
        }
        BrepBuilder aBB = new();
        Compound aC = new();
        aBB.MakeCompound(ref aC);
        int i = 0;
        foreach ((Shape aSK, Shape aSV) in aDMSD)
        {
            // OCCT adds the face itself, not its SD face
            if (aSK.IsEqual(aS))
            {
                if (!aSV.IsEqual(aS))
                {
                    aBB.Add(aC, aS);
                    ++i;
                }
            }
            else if (aSV.IsEqual(aS))
            {
                if (!aSK.IsEqual(aS))
                {
                    aBB.Add(aC, aS);
                    ++i;
                }
            }
        }
        di.Append(i).Append(" SD shapes found\n");
        string buf = a[1] + "_sd";
        theDI.SetShape(buf, aC);
        di.Append(buf).Append("\n");
        return 0;
    }

    private static int BopBFace(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            di.Append("Build faces from surface and set of shared edges. Use: bopbface fr cx\n");
            return 1;
        }
        Shape aS = theDI.GetShape(a[2]);
        if (aS.IsNull)
        {
            di.Append(a[1]).Append(" is a null shape\n");
            return 1;
        }
        Face aF = new();
        NCollection_List<Shape> aLE = new();
        int i = 0;
        for (Iterator aItS = new(aS); aItS.More(); aItS.Next(), ++i)
        {
            Shape aSx = aItS.Value();
            if (i == 0)
            {
                if (aSx.ShapeType() != ShapeType.Face)
                {
                    di.Append(" shape ").Append(i).Append(" is not a face\n");
                    return 1;
                }
                aF = ShapeCasts.Face(aSx);
            }
            else
            {
                if (aSx.ShapeType() != ShapeType.Edge)
                {
                    di.Append(" shape ").Append(i).Append(" is not an edge\n");
                    return 1;
                }
                aLE.Append(aSx);
            }
        }
        BuilderFace aBF = new();
        aBF.SetFace(aF);
        aBF.SetShapes(aLE);
        aBF.Perform();
        BOPTestObjects.ReportAlerts(theDI, di, aBF.GetReport());
        if (aBF.HasErrors)
        {
            return 0;
        }
        NCollection_List<Shape> aLFR = aBF.Areas();
        i = 1;
        foreach (Shape aFR in aLFR)
        {
            string buf = $"{a[1]}_{i++}";
            theDI.SetShape(buf, aFR);
            di.Append(" ").Append(buf);
        }
        i = aLFR.Extent();
        if (i != 0)
        {
            di.Append("\n ").Append(i).Append(" faces were built\n");
        }
        else
        {
            di.Append(" No faces were built\n");
        }
        return 0;
    }

    private static int BopBSolid(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            di.Append("Build solids from set of shared faces. Use: bopbsolid sr cx\n");
            return 1;
        }
        Shape aS = theDI.GetShape(a[2]);
        if (aS.IsNull)
        {
            di.Append(a[1]).Append(" is a null shape\n");
            return 1;
        }
        NCollection_List<Shape> aLF = new();
        for (Explorer aExp = new(aS, ShapeType.Face); aExp.More(); aExp.Next())
        {
            aLF.Append(aExp.Current());
        }
        if (aLF.IsEmpty())
        {
            di.Append(" No faces to build solids\n");
            return 1;
        }
        BuilderSolid aBS = new();
        aBS.SetShapes(aLF);
        aBS.Perform();
        BOPTestObjects.ReportAlerts(theDI, di, aBS.GetReport());
        if (aBS.HasErrors)
        {
            return 0;
        }
        NCollection_List<Shape> aLSR = aBS.Areas();
        int i = 1;
        foreach (Shape aSR in aLSR)
        {
            string buf = $"{a[1]}_{i++}";
            theDI.SetShape(buf, aSR);
            di.Append(" ").Append(buf);
        }
        i = aLSR.Extent();
        if (i != 0)
        {
            di.Append("\n ").Append(i).Append(" solids were built\n");
        }
        else
        {
            di.Append(" No solids were built\n");
        }
        return 0;
    }

    private static ShapeType GetTypeByName(string theName) => theName switch
    {
        "v" or "V" => ShapeType.Vertex,
        "e" or "E" => ShapeType.Edge,
        "w" or "W" => ShapeType.Wire,
        "f" or "F" => ShapeType.Face,
        "sh" or "Sh" or "SH" => ShapeType.Shell,
        "s" or "S" => ShapeType.Solid,
        "cs" or "Cs" or "CS" => ShapeType.CompSolid,
        "c" or "C" => ShapeType.Compound,
        _ => ShapeType.Shape,
    };

    private static string GetNameByType(ShapeType theType) => theType switch
    {
        ShapeType.Vertex => "V",
        ShapeType.Edge => "E",
        ShapeType.Wire => "w",
        ShapeType.Face => "F",
        ShapeType.Shell => "Sh",
        ShapeType.Solid => "S",
        ShapeType.CompSolid => "Cs",
        ShapeType.Compound => "c",
        _ => "Shape",
    };

    private static void DumpInterfs<TInterf>(NCollection_DynamicArray<TInterf> theVInterf, DrawResult di) where TInterf : Interf
    {
        int aNb = theVInterf.Length();
        if (aNb == 0)
        {
            di.Append("Not found\n");
            return;
        }
        di.Append(aNb).Append(" interference(s) found\n");
        for (int i = 0; i < aNb; ++i)
        {
            TInterf anInt = theVInterf[i];
            anInt.Indices(out int n1, out int n2);
            if (anInt.HasIndexNew())
            {
                di.Append($" ({n1}, {n2}, {anInt.IndexNew()})\n");
            }
            else
            {
                di.Append($" ({n1}, {n2})\n");
            }
        }
    }

    private static void SearchNewIndex<TInterf>(string theCType, int theInd, NCollection_DynamicArray<TInterf> theVInterf, DrawResult di)
        where TInterf : Interf
    {
        bool bFound = false;
        int aNb = theVInterf.Length();
        for (int i = 0; i < aNb; ++i)
        {
            TInterf anInt = theVInterf[i];
            int nNew = anInt.IndexNew();
            if (theInd == nNew)
            {
                if (!bFound)
                {
                    di.Append(theCType);
                    bFound = true;
                }
                anInt.Indices(out int n1, out int n2);
                di.Append($"({n1}, {n2}) ");
            }
        }
        if (bFound)
        {
            di.Append("\n");
        }
    }
}
