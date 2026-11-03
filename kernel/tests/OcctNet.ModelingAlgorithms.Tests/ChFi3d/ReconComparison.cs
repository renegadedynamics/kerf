// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Fillets;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Runs the reconstruction (<see cref="BOPReconstruction"/>) on native's data structure as
/// <c>ChFi3d_Builder::Compute</c> does after <c>CompleteDS</c> (ChFi3d_Builder.cxx:471–574), and compares its result
/// and history with native's geometrically (decision 992): volume and area, counts of faces, edges and vertices,
/// validity, bounding box, and the history as sets of (surface type, area).
/// </summary>
internal static class ReconComparison
{
    /// <summary>What the reconstruction gave for a case.</summary>
    internal sealed record Run(Shape Input, DsHDataStructure HDS, BOPReconstruction Builder,
                               Shape Result);

    /// <summary>Compute from <c>myCoup->Perform</c> to the result and the same-parameter (not SetRegul).</summary>
    /// <remarks>
    /// With <paramref name="theFreezeInput"/>, the input is frozen and only <c>Perform</c> and <c>MergeSolid</c> run:
    /// the reconstruction must not write into it (Compute's own steps after them write into the working copy's
    /// vertices, as natively).
    /// </remarks>
    internal static Run Reconstruct(ReconCase theCase, bool theFreezeInput = false)
    {
        Shape anInput = theCase.Input;
        if (theFreezeInput)
        {
            anInput.TShape()?.FreezeGraph(); // the reconstruction writes only into its own shapes
        }
        else
        {
            // the fillet builder's working copy (decision 1087): Compute's steps after MergeSolid write into it
            anInput = BrepTools.ExactCopyForWrite(anInput, out History _);
        }
        DsHDataStructure aHDS = TextDump.Read(theCase.DsText, anInput);
        DsDataStructure aDS = aHDS.DS();
        BOPReconstruction aCoup = new(new DsBuildTool());
        aCoup.Perform(aHDS);

        // the stripes' solids (MapIndSo): the solids, or shells, of the data structure with surfaces
        List<int> aSolids = [];
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            Shape S = aDS.Shape(i);
            if (S.ShapeType() == ShapeType.Solid
                || (S.ShapeType() == ShapeType.Shell && aDS.ShapeInterferences(i).Extent() > 0))
            {
                aSolids.Add(i);
            }
        }
        foreach (int i in aSolids)
        {
            aCoup.MergeSolid(aDS.Shape(i), In);
        }

        BrepBuilder B1 = new();
        if (theFreezeInput)
        {
            Compound aMergedOnly = new();
            B1.MakeCompound(ref aMergedOnly);
            foreach (int i in aSolids)
            {
                foreach (Shape M in aCoup.Merged(aDS.Shape(i), In))
                {
                    B1.Add(aMergedOnly, M);
                }
            }
            return new Run(anInput, aHDS, aCoup, aMergedOnly);
        }
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            Shape S = aDS.Shape(i);
            if (S.ShapeType() != ShapeType.Edge || !aCoup.IsSplit(S, In))
            {
                continue;
            }
            foreach (Shape aNewE in aCoup.Splits(S, In))
            {
                double aTolE = Tool.Tolerance(ShapeCasts.Edge(aNewE));
                for (Explorer exv = new(aNewE, ShapeType.Vertex); exv.More(); exv.Next())
                {
                    Vertex v = ShapeCasts.Vertex(exv.Current());
                    if (aTolE > Tool.Tolerance(v) && !v.Locked())
                    {
                        B1.UpdateVertex(v, aTolE);
                    }
                }
            }
        }
        Compound aResult = new();
        B1.MakeCompound(ref aResult);
        foreach (int i in aSolids)
        {
            Shape S = aDS.Shape(i);
            NCollection_List<Shape> aMerged = aCoup.Merged(S, In);
            if (aMerged.IsEmpty())
            {
                B1.Add(aResult, S);
            }
            foreach (Shape M in aMerged)
            {
                B1.Add(aResult, M);
            }
        }
        // Compute's last step: the new faces made same-parameter (ChFi3d_Builder.cxx:654-676)
        for (int iF = 1; iF <= aDS.NbSurfaces(); iF++)
        {
            foreach (Shape aF in aCoup.NewFaces(iF))
            {
                BrepLibrary.SameParameter(aF, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                ShapeFixFixer.SameParameter(aF, false, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
        return new Run(anInput, aHDS, aCoup, aResult);
    }

    internal static double Volume(in Shape S)
    {
        Properties p = new();
        MassPropertiesBuilder.VolumeProperties(S, p);
        return p.Mass();
    }

    internal static double Area(in Shape S)
    {
        Properties p = new();
        MassPropertiesBuilder.SurfaceProperties(S, p);
        return p.Mass();
    }

    internal static int Count(in Shape S, ShapeType T)
    {
        NCollection_IndexedMap<Shape> M = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, T, M);
        return M.Extent();
    }

    private static string FaceKey(in Shape F) =>
        Kerf.Engine.Foundation.OcctTypeNames.Of(new FaceSurfaceAdaptor(ShapeCasts.Face(F)).GetType()).Replace("GeomAbs_", "") + " "
        + Area(F).ToString("G6", CultureInfo.InvariantCulture);

    private static string FaceSet(IEnumerable<Shape> theFaces)
    {
        List<string> aKeys = theFaces.Select(f => FaceKey(f)).ToList();
        aKeys.Sort(StringComparer.Ordinal);
        return "[" + string.Join(", ", aKeys) + "]";
    }

    internal static double EdgeLength(in Shape E) =>
        AbscissaPoint.Length(new EdgeCurveAdaptor(ShapeCasts.Edge(E)));

    private static string Lengths(IEnumerable<double> theLengths) =>
        "[" + string.Join(", ", theLengths.Select(x => x.ToString("G6", CultureInfo.InvariantCulture))
                                          .Order(StringComparer.Ordinal)) + "]";

    private static bool Close(double a, double b, double theRel) =>
        Math.Abs(a - b) <= theRel * Math.Max(1.0, Math.Max(Math.Abs(a), Math.Abs(b)));

    /// <summary>The faults BRepCheck finds, by sub-shape type.</summary>
    internal static string Faults(in Shape S)
    {
        Analyzer an = new(S);
        SortedSet<string> aFaults = [];
        ShapeType[] aTypes = [ShapeType.Solid, ShapeType.Shell, ShapeType.Face, ShapeType.Wire, ShapeType.Edge, ShapeType.Vertex];
        foreach (ShapeType T in aTypes)
        {
            for (Explorer exp = new(S, T); exp.More(); exp.Next())
            {
                if (an.Result(exp.Current()) is ResultCheck r)
                {
                    foreach (StatusCheck st in r.Status())
                    {
                        if (st != StatusCheck.NoError)
                        {
                            aFaults.Add(Kerf.Engine.Foundation.OcctTypeNames.Of(T).Replace("TopAbs_", "") + ":"
                                        + Kerf.Engine.Foundation.OcctTypeNames.Of(st).Replace("BRepCheck_", ""));
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
        Shape aNative = theCase.Result ?? throw new InvalidOperationException("native was not done");
        Shape aMine = theRun.Result;

        double v1 = Volume(aNative), v2 = Volume(aMine), a1 = Area(aNative), a2 = Area(aMine);
        if (!Close(v1, v2, theRelTol))
        {
            d.Add($"volume {v1:G9} | {v2:G9}");
        }
        if (!Close(a1, a2, theRelTol))
        {
            d.Add($"area {a1:G9} | {a2:G9}");
        }
        foreach (ShapeType T in new[] { ShapeType.Solid, ShapeType.Shell, ShapeType.Face, ShapeType.Edge, ShapeType.Vertex })
        {
            int n1 = Count(aNative, T), n2 = Count(aMine, T);
            if (n1 != n2)
            {
                d.Add($"{Kerf.Engine.Foundation.OcctTypeNames.Of(T)} {n1} | {n2}");
            }
        }
        bool ok1 = new Analyzer(aNative).IsValid(), ok2 = new Analyzer(aMine).IsValid();
        if (ok1 != ok2)
        {
            d.Add($"valid {ok1} | {ok2}" + (ok2 ? "" : " " + Faults(aMine))
                  + (ok1 ? "" : " native " + Faults(aNative)));
        }
        Box b1 = new(), b2 = new();
        Bounds.AddOptimal(aNative, ref b1, false, false);
        Bounds.AddOptimal(aMine, ref b2, false, false);
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
        NCollection_IndexedMap<Shape> aNF = new(ShapeHasher.Instance);
        Exploration.MapShapes(aNative, ShapeType.Face, aNF);
        NCollection_IndexedMap<Shape> aMF = new(ShapeHasher.Instance);
        Exploration.MapShapes(aMine, ShapeType.Face, aMF);
        IEnumerable<Shape> NativeFaces(int[] theIdx) => theIdx.Where(i => i > 0).Select(i => aNF[i]);
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
        NCollection_IndexedMap<Shape> aIF = new(ShapeHasher.Instance);
        Exploration.MapShapes(theRun.Input, ShapeType.Face, aIF);
        for (int i = 1; i <= theCase.Modified.Count; i++)
        {
            Shape F = aIF[i];
            List<Shape> aMod = [];
            foreach (Containment st in new[] { Out, In, On })
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
        NCollection_IndexedMap<Shape> aIS = new(ShapeHasher.Instance);
        Exploration.MapShapes(theRun.Input, ShapeType.Solid, aIS);
        NCollection_IndexedMap<Shape> aNS = new(ShapeHasher.Instance);
        Exploration.MapShapes(aNative, ShapeType.Solid, aNS);
        NCollection_IndexedMap<Shape> aMS = new(ShapeHasher.Instance);
        Exploration.MapShapes(aMine, ShapeType.Solid, aMS);
        for (int i = 1; i <= Math.Min(theCase.Solids.Count, aIS.Extent()); i++)
        {
            Shape S = aIS[i];
            List<Shape> aMod = [];
            foreach (Containment st in new[] { Out, In, On })
            {
                if (theRun.Builder.IsSplit(S, st))
                {
                    aMod.AddRange(theRun.Builder.Splits(S, st));
                }
            }
            bool aDeleted = !(aMS.Contains(S) || aMod.Count > 0);
            static string Volumes(IEnumerable<Shape> L) =>
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
