// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using System.Globalization;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Runs the reconstruction (<see cref="ChFi3d_BOPReconstruction"/>) on native's data structure as
/// <c>ChFi3d_Builder::Compute</c> does after <c>CompleteDS</c> (ChFi3d_Builder.cxx:471–574), and compares its result
/// and history with native's geometrically (decision 992): volume and area, counts of faces, edges and vertices,
/// validity, bounding box, and the history as sets of (surface type, area).
/// </summary>
internal static class ReconComparison
{
    /// <summary>What the reconstruction gave for a case.</summary>
    internal sealed record Run(TopoDS_Shape Input, TopOpeBRepDS_HDataStructure HDS, ChFi3d_BOPReconstruction Builder,
                               TopoDS_Shape Result);

    /// <summary>Compute from <c>myCoup->Perform</c> to the result and the same-parameter (not SetRegul).</summary>
    /// <remarks>
    /// With <paramref name="theFreezeInput"/>, the input is frozen and only <c>Perform</c> and <c>MergeSolid</c> run:
    /// the reconstruction must not write into it (Compute's own steps after them write into the working copy's
    /// vertices, as natively).
    /// </remarks>
    internal static Run Reconstruct(ReconCase theCase, bool theFreezeInput = false)
    {
        TopoDS_Shape anInput = theCase.Input;
        if (theFreezeInput)
        {
            anInput.TShape()?.FreezeGraph(); // the reconstruction writes only into its own shapes
        }
        else
        {
            // the fillet builder's working copy (decision 1087): Compute's steps after MergeSolid write into it
            anInput = BRepTools.ExactCopyForWrite(anInput, out BRepTools_History _);
        }
        TopOpeBRepDS_HDataStructure aHDS = TopOpeBRepDS_TextDump.Read(theCase.DsText, anInput);
        TopOpeBRepDS_DataStructure aDS = aHDS.DS();
        ChFi3d_BOPReconstruction aCoup = new(new TopOpeBRepDS_BuildTool());
        aCoup.Perform(aHDS);

        // the stripes' solids (MapIndSo): the solids, or shells, of the data structure with surfaces
        List<int> aSolids = [];
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            TopoDS_Shape S = aDS.Shape(i);
            if (S.ShapeType() == TopAbs_SOLID
                || (S.ShapeType() == TopAbs_SHELL && aDS.ShapeInterferences(i).Extent() > 0))
            {
                aSolids.Add(i);
            }
        }
        foreach (int i in aSolids)
        {
            aCoup.MergeSolid(aDS.Shape(i), TopAbs_IN);
        }

        BRep_Builder B1 = new();
        if (theFreezeInput)
        {
            TopoDS_Compound aMergedOnly = new();
            B1.MakeCompound(ref aMergedOnly);
            foreach (int i in aSolids)
            {
                foreach (TopoDS_Shape M in aCoup.Merged(aDS.Shape(i), TopAbs_IN))
                {
                    B1.Add(aMergedOnly, M);
                }
            }
            return new Run(anInput, aHDS, aCoup, aMergedOnly);
        }
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            TopoDS_Shape S = aDS.Shape(i);
            if (S.ShapeType() != TopAbs_EDGE || !aCoup.IsSplit(S, TopAbs_IN))
            {
                continue;
            }
            foreach (TopoDS_Shape aNewE in aCoup.Splits(S, TopAbs_IN))
            {
                double aTolE = BRep_Tool.Tolerance(TopoDS.Edge(aNewE));
                for (TopExp_Explorer exv = new(aNewE, TopAbs_VERTEX); exv.More(); exv.Next())
                {
                    TopoDS_Vertex v = TopoDS.Vertex(exv.Current());
                    if (aTolE > BRep_Tool.Tolerance(v) && !v.Locked())
                    {
                        B1.UpdateVertex(v, aTolE);
                    }
                }
            }
        }
        TopoDS_Compound aResult = new();
        B1.MakeCompound(ref aResult);
        foreach (int i in aSolids)
        {
            TopoDS_Shape S = aDS.Shape(i);
            NCollection_List<TopoDS_Shape> aMerged = aCoup.Merged(S, TopAbs_IN);
            if (aMerged.IsEmpty())
            {
                B1.Add(aResult, S);
            }
            foreach (TopoDS_Shape M in aMerged)
            {
                B1.Add(aResult, M);
            }
        }
        // Compute's last step: the new faces made same-parameter (ChFi3d_Builder.cxx:654-676)
        for (int iF = 1; iF <= aDS.NbSurfaces(); iF++)
        {
            foreach (TopoDS_Shape aF in aCoup.NewFaces(iF))
            {
                BRepLib.SameParameter(aF, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                ShapeFix.SameParameter(aF, false, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
        return new Run(anInput, aHDS, aCoup, aResult);
    }

    internal static double Volume(in TopoDS_Shape S)
    {
        GProp_GProps p = new();
        BRepGProp.VolumeProperties(S, p);
        return p.Mass();
    }

    internal static double Area(in TopoDS_Shape S)
    {
        GProp_GProps p = new();
        BRepGProp.SurfaceProperties(S, p);
        return p.Mass();
    }

    internal static int Count(in TopoDS_Shape S, TopAbs_ShapeEnum T)
    {
        NCollection_IndexedMap<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, T, M);
        return M.Extent();
    }

    private static string FaceKey(in TopoDS_Shape F) =>
        new BRepAdaptor_Surface(TopoDS.Face(F)).GetType().ToString().Replace("GeomAbs_", "") + " "
        + Area(F).ToString("G6", CultureInfo.InvariantCulture);

    private static string FaceSet(IEnumerable<TopoDS_Shape> theFaces)
    {
        List<string> aKeys = theFaces.Select(f => FaceKey(f)).ToList();
        aKeys.Sort(StringComparer.Ordinal);
        return "[" + string.Join(", ", aKeys) + "]";
    }

    internal static double EdgeLength(in TopoDS_Shape E) =>
        GCPnts_AbscissaPoint.Length(new BRepAdaptor_Curve(TopoDS.Edge(E)));

    private static string Lengths(IEnumerable<double> theLengths) =>
        "[" + string.Join(", ", theLengths.Select(x => x.ToString("G6", CultureInfo.InvariantCulture))
                                          .Order(StringComparer.Ordinal)) + "]";

    private static bool Close(double a, double b, double theRel) =>
        Math.Abs(a - b) <= theRel * Math.Max(1.0, Math.Max(Math.Abs(a), Math.Abs(b)));

    /// <summary>The faults BRepCheck finds, by sub-shape type.</summary>
    internal static string Faults(in TopoDS_Shape S)
    {
        BRepCheck_Analyzer an = new(S);
        SortedSet<string> aFaults = [];
        TopAbs_ShapeEnum[] aTypes = [TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE, TopAbs_WIRE, TopAbs_EDGE, TopAbs_VERTEX];
        foreach (TopAbs_ShapeEnum T in aTypes)
        {
            for (TopExp_Explorer exp = new(S, T); exp.More(); exp.Next())
            {
                if (an.Result(exp.Current()) is BRepCheck_Result r)
                {
                    foreach (BRepCheck_Status st in r.Status())
                    {
                        if (st != BRepCheck_Status.BRepCheck_NoError)
                        {
                            aFaults.Add(T.ToString().Replace("TopAbs_", "") + ":"
                                        + st.ToString().Replace("BRepCheck_", ""));
                        }
                    }
                }
            }
        }
        return "{" + string.Join(" ", aFaults) + "}";
    }

    /// <summary>
    /// The differences between the reconstruction's result and history and native's (empty: geometrically equal).
    /// </summary>
    internal static List<string> Differences(ReconCase theCase, Run theRun, double theRelTol = 1.0e-6)
    {
        List<string> d = [];
        TopoDS_Shape aNative = theCase.Result ?? throw new InvalidOperationException("native was not done");
        TopoDS_Shape aMine = theRun.Result;

        double v1 = Volume(aNative), v2 = Volume(aMine), a1 = Area(aNative), a2 = Area(aMine);
        if (!Close(v1, v2, theRelTol))
        {
            d.Add($"volume {v1:G9} | {v2:G9}");
        }
        if (!Close(a1, a2, theRelTol))
        {
            d.Add($"area {a1:G9} | {a2:G9}");
        }
        foreach (TopAbs_ShapeEnum T in new[] { TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX })
        {
            int n1 = Count(aNative, T), n2 = Count(aMine, T);
            if (n1 != n2)
            {
                d.Add($"{T} {n1} | {n2}");
            }
        }
        bool ok1 = new BRepCheck_Analyzer(aNative).IsValid(), ok2 = new BRepCheck_Analyzer(aMine).IsValid();
        if (ok1 != ok2)
        {
            d.Add($"valid {ok1} | {ok2}" + (ok2 ? "" : " " + Faults(aMine))
                  + (ok1 ? "" : " native " + Faults(aNative)));
        }
        Box b1 = new(), b2 = new();
        BRepBndLib.AddOptimal(aNative, ref b1, false, false);
        BRepBndLib.AddOptimal(aMine, ref b2, false, false);
        if (!b1.IsVoid && !b2.IsVoid)
        {
            b1.Get(out double x1, out double y1, out double z1, out double X1, out double Y1, out double Z1);
            b2.Get(out double x2, out double y2, out double z2, out double X2, out double Y2, out double Z2);
            double aDiag = Math.Sqrt(b1.SquareExtent());
            double[] e = [x1 - x2, y1 - y2, z1 - z2, X1 - X2, Y1 - Y2, Z1 - Z2];
            if (e.Any(x => Math.Abs(x) > 1.0e-4 * aDiag))
            {
                d.Add($"box ({x1:G6} {y1:G6} {z1:G6} {X1:G6} {Y1:G6} {Z1:G6})"
                      + $" | ({x2:G6} {y2:G6} {z2:G6} {X2:G6} {Y2:G6} {Z2:G6})");
            }
        }

        // the history, as sets of (surface type, area): the faces of each surface and the images of each input face
        NCollection_IndexedMap<TopoDS_Shape> aNF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aNative, TopAbs_FACE, aNF);
        NCollection_IndexedMap<TopoDS_Shape> aMF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aMine, TopAbs_FACE, aMF);
        IEnumerable<TopoDS_Shape> NativeFaces(int[] theIdx) => theIdx.Where(i => i > 0).Select(i => aNF[i]);
        for (int i = 1; i <= theCase.NewFaces.Count; i++)
        {
            string s1 = FaceSet(NativeFaces(theCase.NewFaces[i - 1]));
            string s2 = FaceSet(theRun.Builder.NewFaces(i));
            if (s1 != s2)
            {
                d.Add($"NewFaces({i}) {s1} | {s2}");
            }
        }
        for (int i = 1; i <= theCase.NewEdges.Count; i++)
        {
            double[] l1 = [.. theCase.NewEdges[i - 1].Order()];
            double[] l2 = [.. theRun.Builder.NewEdges(i).Select(e => EdgeLength(e)).Order()];
            if (l1.Length != l2.Length || l1.Zip(l2).Any(p => !Close(p.First, p.Second, 1.0e-5)))
            {
                d.Add($"NewEdges({i}) {Lengths(l1)} | {Lengths(l2)}");
            }
        }
        NCollection_IndexedMap<TopoDS_Shape> aIF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theRun.Input, TopAbs_FACE, aIF);
        for (int i = 1; i <= theCase.Modified.Count; i++)
        {
            TopoDS_Shape F = aIF[i];
            List<TopoDS_Shape> aMod = [];
            foreach (TopAbs_State st in new[] { TopAbs_OUT, TopAbs_IN, TopAbs_ON })
            {
                if (theRun.Builder.IsSplit(F, st))
                {
                    aMod.AddRange(theRun.Builder.Splits(F, st));
                }
            }
            bool aDeleted = !(aMF.Contains(F) || aMod.Count > 0);
            string s1 = (theCase.Deleted[i - 1] ? "deleted " : "") + FaceSet(NativeFaces(theCase.Modified[i - 1]));
            string s2 = (aDeleted ? "deleted " : "") + FaceSet(aMod);
            if (s1 != s2)
            {
                d.Add($"Modified(F{i}) {s1} | {s2}");
            }
        }
        // the input's solids: BRepFilletAPI's IsDeleted and Modified read the reconstruction's splits (unit 7R3)
        NCollection_IndexedMap<TopoDS_Shape> aIS = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theRun.Input, TopAbs_SOLID, aIS);
        NCollection_IndexedMap<TopoDS_Shape> aNS = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aNative, TopAbs_SOLID, aNS);
        NCollection_IndexedMap<TopoDS_Shape> aMS = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aMine, TopAbs_SOLID, aMS);
        for (int i = 1; i <= Math.Min(theCase.Solids.Count, aIS.Extent()); i++)
        {
            TopoDS_Shape S = aIS[i];
            List<TopoDS_Shape> aMod = [];
            foreach (TopAbs_State st in new[] { TopAbs_OUT, TopAbs_IN, TopAbs_ON })
            {
                if (theRun.Builder.IsSplit(S, st))
                {
                    aMod.AddRange(theRun.Builder.Splits(S, st));
                }
            }
            bool aDeleted = !(aMS.Contains(S) || aMod.Count > 0);
            static string Volumes(IEnumerable<TopoDS_Shape> L) =>
                "[" + string.Join(", ", L.Select(x => Volume(x).ToString("G6", CultureInfo.InvariantCulture))) + "]";
            (bool nDel, int[] nMod) = theCase.Solids[i - 1];
            string s1 = (nDel ? "deleted " : "") + Volumes(nMod.Where(j => j > 0).Select(j => aNS[j]));
            string s2 = (aDeleted ? "deleted " : "") + Volumes(aMod);
            if (s1 != s2)
            {
                d.Add($"Modified(So{i}) {s1} | {s2}");
            }
        }
        return d;
    }
}
