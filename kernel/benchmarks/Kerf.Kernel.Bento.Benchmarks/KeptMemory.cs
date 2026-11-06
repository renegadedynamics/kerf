// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;
using Kerf.Geometry;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

// What stays in memory (unit 11Q, sixth slice; performance.md, "The sixth slice"). A workbook keeps a shape and a mesh in each cell for hours,
// so what limits it is what stays alive and the peak, not what's allocated in passing:
//   -- kept                 each example's kept shape and mesh, by group of type, and checked against the collector's count
//   -- kept-types [name]    the same for one example, by type and by holding field, with identical data
//   -- chain [cells]        chains of features where each cell keeps its step
//   -- step file...         a STEP import kept: bytes, by group; and what's left after the import when nothing is kept
//   -- peak file [mesh]     one STEP import (and mesh), in a process of its own: peak working set against live data
internal static class KeptMemory
{
    private static long Live()
    {
        for (int i = 0; i < 3; i++)
        {
            GC.Collect(GC.MaxGeneration, GCCollectionMode.Forced, true, true);
            GC.WaitForPendingFinalizers();
        }
        return GC.GetTotalMemory(true);
    }

    private static string Group(string type)
    {
        string plain = type.TrimEnd('[', ']');
        if (Regex.IsMatch(type, @"ModelHistory|ShapeHistory|ShapeTrace"))
            return "history";
        if (Regex.IsMatch(type, @"^(ShapeMesh|FaceMesh|EdgeMesh|Single\[\]|Int32\[\])"))
            return "mesh";
        if (Regex.IsMatch(type, @"Triangulation|Polygon(3D|2D|OnTriangulation)|Poly"))
            return "shape's own mesh";
        if (Regex.IsMatch(plain, @"^(Brep)?T(Shape|Vertex|Edge|Wire|Face|Shell|Solid|CompSolid|Compound)$") || Regex.IsMatch(type, @"NCollection_List<Shape>|Shape\[\]|^(Shape|Vertex|Edge|Wire|Face|Shell|Solid|Compound)$"))
            return "topology";
        if (Regex.IsMatch(type, @"Representation|CurveOn|PointOn|CurveRep|PointRep|Rep\b"))
            return "representations";
        if (Regex.IsMatch(type, @"Placement|Location|Datum|Trsf|Transform"))
            return "placements";
        if (Regex.IsMatch(type, @"Curve|Surface|Plane|Line|Circle|Cylinder|Sphere|Cone|Torus|Ellipse|Hyperbola|Parabola|BSpline|Bezier|Double\[\]|Point[23]D|Vector[23]D|Direction|Axis|Ax[123]"))
            return "geometry";
        if (Regex.IsMatch(type, @"Part|Sketch|ModelShape|Assembly|^String$|Box|Cylinder"))
            return "model object";
        return "other";
    }

    private static readonly string[] Groups = ["model object", "topology", "representations", "geometry", "placements", "mesh", "shape's own mesh", "history", "other"];

    private static string Row(HeapWalk walk)
    {
        var sums = walk.ByType.GroupBy(t => Group(t.Key)).ToDictionary(g => g.Key, g => g.Sum(t => t.Value.Bytes));
        return string.Join(" ", Groups.Select(g => string.Create(CultureInfo.InvariantCulture, $"{sums.GetValueOrDefault(g) / 1024.0,8:F1}")));
    }

    private static string Header => string.Join(" ", Groups.Select(g => $"{(g.Length > 8 ? g[..8] : g),8}"));

    private static bool IsTShape(Type type)
    {
        for (Type? t = type; t is not null; t = t.BaseType)
        {
            if (t.Name == "TShape")
                return true;
        }
        return false;
    }

    private static long CountOf(HeapWalk walk, params string[] types) =>
        types.Sum(t => walk.ByType.GetValueOrDefault(t).Count);

    private static (long Used, long Held) MeshSlack(ShapeMesh mesh)
    {
        long used = 0, held = 0;
        var arrays = new HashSet<object>(ReferenceEqualityComparer.Instance);
        void Add<T>(ReadOnlyMemory<T> memory) where T : unmanaged
        {
            used += memory.Length * Marshal.SizeOf<T>();
            if (MemoryMarshal.TryGetArray(memory, out ArraySegment<T> whole) && whole.Array is { } array)
                held += arrays.Add(array) ? array.Length * Marshal.SizeOf<T>() : 0;
        }
        foreach (FaceMesh f in mesh.Faces)
        {
            Add(f.Positions);
            Add(f.Normals);
            Add(f.Triangles);
        }
        foreach (EdgeMesh e in mesh.Edges)
            Add(e.Points);
        return (used, held);
    }

    /// <summary>`-- kept`: each example's kept object and mesh, grouped, against the collector's own count.</summary>
    public static void Kept((string Name, Func<ModelShape> Build)[] examples, string? which = null, int held = 8)
    {
        Console.WriteLine($"{"example",-24} {"faces",5} {"tris",6} {"KB",8} {"GC KB",8} {"no mesh",8} {"B/face",7} {"B/tri",6} slack% | {Header}");
        foreach (var (name, build) in examples)
        {
            if (which is not null && !name.Contains(which, StringComparison.Ordinal))
                continue;
            for (int warm = 0; warm < 2; warm++)
            {
                ModelShape warmed = build();
                GC.KeepAlive(warmed.AsShape().Triangulate());
                GC.KeepAlive(warmed.AsShape().Volume);
            }
            var keep = new (ModelShape Model, ShapeMesh Mesh)[held];
            long live = 0;
            // The second half only: the first copies can still grow a thread's work sets
            for (int i = 0; i < held; i++)
            {
                if (i == held / 2)
                    live = Live();
                ModelShape model = build();
                _ = model.AsShape().Volume;
                keep[i] = (model, model.AsShape().Triangulate());
            }
            double gc = (Live() - live) / (double)(held - held / 2);
            var walk = new HeapWalk();
            walk.Add(keep[0].Model, "model");
            long noMesh = walk.Bytes;
            walk.Add(keep[0].Mesh, "mesh");
            int faces = keep[0].Model.AsShape().Faces().Count, tris = keep[0].Mesh.TriangleCount;
            (long used, long heldBytes) = MeshSlack(keep[0].Mesh);
            if (which is not null)
            {
                var everything = new HeapWalk();
                for (int i = held / 2; i < held; i++)
                {
                    everything.Add(keep[i].Model, "model");
                    everything.Add(keep[i].Mesh, "mesh");
                }
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  the second half walked, each: {everything.Bytes / 1024.0 / (held - held / 2):F1} KB"));
            }
            GC.KeepAlive(keep);
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{name,-24} {faces,5} {tris,6} {walk.Bytes / 1024.0,8:F1} {gc / 1024.0,8:F1} {noMesh / 1024.0,8:F1} {(double)noMesh / faces,7:F0} {(double)(walk.Bytes - noMesh) / tris,6:F1} {100.0 * (heldBytes - used) / Math.Max(1, heldBytes),6:F1} | {Row(walk)}"));
        }
    }

    /// <summary>`-- kept-types`: one example, by type and by holder, with identical data.</summary>
    public static void KeptTypes((string Name, Func<ModelShape> Build)[] examples, string? which)
    {
        foreach (var (name, build) in examples)
        {
            if (which is not null && !name.Contains(which, StringComparison.Ordinal))
                continue;
            ModelShape model = build();
            _ = model.AsShape().Volume;
            ShapeMesh mesh = model.AsShape().Triangulate();
            var walk = new HeapWalk();
            walk.Add(model, "model");
            walk.Add(mesh, "mesh");
            Console.WriteLine($"== {name}: {walk.Bytes / 1024.0:F1} KB in {walk.Objects} objects");
            Print(walk, 40);
            PrintSame(walk, 12);
        }
    }

    private static void Print(HeapWalk walk, int top)
    {
        Console.WriteLine("  by type");
        foreach (var t in walk.ByType.OrderByDescending(t => t.Value.Bytes).Take(top))
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    {t.Value.Bytes / 1024.0,10:F1} KB {100.0 * t.Value.Bytes / walk.Bytes,5:F1}% {t.Value.Count,8}  {Group(t.Key),-16} {t.Key}"));
        Console.WriteLine("  by the field that reached it first");
        foreach (var t in walk.ByHolder.OrderByDescending(t => t.Value.Bytes).Take(top))
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    {t.Value.Bytes / 1024.0,10:F1} KB {100.0 * t.Value.Bytes / walk.Bytes,5:F1}% {t.Value.Count,8}  {t.Key}"));
    }

    private static void PrintSame(HeapWalk walk, int top)
    {
        var same = new SameData(IsTShape).Over(walk.Seen);
        Console.WriteLine($"  identical data: {same.Sum(s => s.Wasted) / 1024.0:F1} KB would go if each were held once");
        foreach (var s in same.Where(s => s.Wasted > 0).Take(top))
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    {s.Wasted / 1024.0,10:F1} KB {s.Count,8} objects, {s.Distinct,8} distinct  {s.Type}"));
    }

    private static readonly Func<FieldInfo, bool> HistoryField = f => f.DeclaringType?.Name == "ModelShape" && f.Name.Contains("History", StringComparison.Ordinal);

    /// <summary>`-- chain`: long chains of features, each cell keeping its step; what each cell adds to the ones before.</summary>
    public static void Chain(int cells)
    {
        var chains = new (string Name, Func<int, Part, Part> Step, Part Start)[]
        {
            ("hole in a plate", (i, p) => p - new Pos(-90 + 12 * (i % 16), -90 + 12 * (i / 16), 0.0) * new Cylinder(3, 40), new Box(220, 220, 20)),
            ("boss on a plate", (i, p) => p + new Pos(-90 + 12 * (i % 16), -90 + 12 * (i / 16), 12.0) * new Box(5, 5, 8), new Box(220, 220, 20)),
            ("fillet each new hole", (i, p) =>
            {
                Part cut = p - new Pos(-90 + 20 * (i % 10), -90 + 20 * (i / 10), 0.0) * new Cylinder(3, 40);
                return Fillet(cut.Edges().Except(p.Edges()).GroupBy(Axis.Z)[^1], 1).Value;
            }, new Box(220, 220, 20)),
        };
        Console.WriteLine($"{"chain",-22} {"cells",5} | {"held, MB",9} {"no mesh",8} {"alone",8} {"no hist",8} | last cell KB: {"adds",7} {"alone",7} {"history",7} | faces: {"last",5} {"sum",6} {"TFaces",6} {"share%",6}");
        foreach (var (name, step, start) in chains)
        {
            long live = Live();
            var models = new List<Part>(cells);
            var meshes = new List<ShapeMesh>(cells);
            Part current = start;
            for (int i = 0; i < cells; i++)
            {
                current = step(i, current);
                _ = current.AsShape().Volume;
                models.Add(current);
                meshes.Add(current.AsShape().Triangulate());
            }
            double gc = (Live() - live) / 1048576.0;

            var all = new HeapWalk();
            long lastAdds = 0, lastHistoryAdds = 0;
            for (int i = 0; i < cells; i++)
            {
                lastAdds = all.Add(models[i], "model");
            }
            long noMesh = all.Bytes;
            foreach (ShapeMesh mesh in meshes)
                all.Add(mesh, "mesh");

            var noHistory = new HeapWalk(HistoryField);
            foreach (Part model in models)
                lastHistoryAdds = noHistory.Add(model, "model");

            long alone = 0;
            foreach (Part model in models)
                alone += new HeapWalk(HistoryField).Add(model, "model");
            long lastAlone = new HeapWalk(HistoryField).Add(models[^1], "model");

            long faceSum = models.Sum(m => (long)m.AsShape().Faces().Count), lastFaces = models[^1].AsShape().Faces().Count;
            long tfaces = all.ByType.Where(t => t.Key is "BrepTFace").Sum(t => t.Value.Count);
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{name,-22} {cells,5} | {all.Bytes / 1048576.0,9:F2} {noMesh / 1048576.0,8:F2} {alone / 1048576.0,8:F2} {noHistory.Bytes / 1048576.0,8:F2} |               {lastAdds / 1024.0,7:F1} {lastAlone / 1024.0,7:F1} {(lastAdds - lastHistoryAdds) / 1024.0,7:F1} |        {lastFaces,5} {faceSum,6} {tfaces,6} {100.0 * (1 - (double)tfaces / faceSum),6:F1}  (GC {gc:F2} MB; meshes {(all.Bytes - noMesh) / 1048576.0:F2} MB)"));
            Print(all, 24);
            PrintSame(all, 10);
            GC.KeepAlive(models);
            GC.KeepAlive(meshes);
        }
    }

    /// <summary>`-- step`: a STEP import kept, grouped; then what's left once nothing is kept.</summary>
    public static void Step(string[] files, bool same)
    {
        Console.WriteLine($"{"file",-28} {"MB file",7} {"faces",6} {"KB",9} {"GC KB",9} {"left KB",8} {"B/face",7} | {Header}");
        foreach (string file in files)
        {
            long live = Live();
            ModelShape model = Importers.ImportStep(file).Value;
            double gc = (Live() - live) / 1024.0;
            var walk = new HeapWalk();
            walk.Add(model, "model");
            int faces = model.AsShape().Faces().Count;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{Path.GetFileName(file),-28} {new FileInfo(file).Length / 1048576.0,7:F1} {faces,6} {walk.Bytes / 1024.0,9:F0} {gc,9:F0} {"",8} {(double)walk.Bytes / Math.Max(1, faces),7:F0} | {Row(walk)}"));
            if (same)
            {
                Print(walk, 30);
                PrintSame(walk, 15);
            }
            GC.KeepAlive(model);
            model = null!;
            walk = null!;
            double left = (Live() - live) / 1024.0;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    left after dropping it: {left:F0} KB"));
        }
    }

    /// <summary>`-- peak`: one import (and mesh) in a fresh process: the peak working set, what the heap held then, and the largest arrays.</summary>
    public static void Peak(string? file, Func<ShapeMesh?>? work, bool mesh)
    {
        using var large = new LargeAllocations();
        using var stop = new CancellationTokenSource();
        var process = Process.GetCurrentProcess();
        (long Ws, long Heap, long Committed, long Loh, long Fragmented, long Promoted, int Generation) atPeak = default;
        long maxHeapAfter = 0, maxPromotedGen2 = 0;
        var sampler = new Thread(() =>
        {
            while (!stop.IsCancellationRequested)
            {
                process.Refresh();
                long ws = process.WorkingSet64;
                GCMemoryInfo info = GC.GetGCMemoryInfo(GCKind.Any);
                if (info.Index != 0)
                {
                    maxHeapAfter = Math.Max(maxHeapAfter, info.HeapSizeBytes);
                    GCMemoryInfo full = GC.GetGCMemoryInfo(GCKind.FullBlocking);
                    GCMemoryInfo background = GC.GetGCMemoryInfo(GCKind.Background);
                    maxPromotedGen2 = Math.Max(maxPromotedGen2, Math.Max(full.Index != 0 ? full.PromotedBytes : 0, background.Index != 0 ? background.PromotedBytes : 0));
                }
                if (ws > atPeak.Ws)
                {
                    var gens = info.GenerationInfo;
                    atPeak = (ws, info.HeapSizeBytes, info.TotalCommittedBytes, gens.Length > 3 ? gens[3].SizeAfterBytes : 0, info.FragmentedBytes, info.PromotedBytes, info.Generation);
                }
                Thread.Sleep(5);
            }
        }) { IsBackground = true };
        sampler.Start();

        long start = GC.GetTotalAllocatedBytes(true);
        object? kept = null;
        ShapeMesh? keptMesh = null;
        var clock = Stopwatch.StartNew();
        if (file is not null)
        {
            ModelShape model = Importers.ImportStep(file).Value;
            kept = model;
            if (mesh)
                keptMesh = model.AsShape().Triangulate();
        }
        else if (work is not null)
        {
            keptMesh = work();
        }
        double seconds = clock.Elapsed.TotalSeconds;
        long allocated = GC.GetTotalAllocatedBytes(true) - start;
        stop.Cancel();
        sampler.Join();
        long peak = process.PeakWorkingSet64;
        long live = Live();
        var info2 = GC.GetGCMemoryInfo(GCKind.Any);
        long gen0 = GC.CollectionCount(0), gen1 = GC.CollectionCount(1), gen2 = GC.CollectionCount(2);
        var walk = new HeapWalk();
        walk.Add(kept, "model");
        long modelBytes = walk.Bytes;
        walk.Add(keptMesh, "mesh");
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{(file is null ? "work" : Path.GetFileName(file))} {(mesh ? "with mesh " : "")}server={GCSettings.IsServerGC} heaps={GC.GetGCMemoryInfo().Concurrent}: {seconds:F1} s (loaded machine), allocated {allocated / 1048576.0:F0} MB, collections {gen0}/{gen1}/{gen2}"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  peak working set {peak / 1048576.0:F1} MB; sampled peak {atPeak.Ws / 1048576.0:F1} MB with heap after last GC {atPeak.Heap / 1048576.0:F1}, committed {atPeak.Committed / 1048576.0:F1}, LOH {atPeak.Loh / 1048576.0:F1}, fragmented {atPeak.Fragmented / 1048576.0:F1}, promoted {atPeak.Promoted / 1048576.0:F1}"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  largest heap after any GC {maxHeapAfter / 1048576.0:F1} MB; most promoted by a full GC {maxPromotedGen2 / 1048576.0:F1} MB; live at the end {live / 1048576.0:F1} MB (walked: model {modelBytes / 1048576.0:F1}, mesh {(walk.Bytes - modelBytes) / 1048576.0:F1}); working set at the end {GetWs(process) / 1048576.0:F1} MB"));
        Console.WriteLine("  largest arrays allocated (type, count, largest, sum, MB)");
        Thread.Sleep(500);
        foreach (var a in large.Report().Take(12))
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    {a.Type,-50} {a.Count,6} {a.Largest / 1048576.0,8:F2} {a.Sum / 1048576.0,8:F1}"));
        GC.KeepAlive(kept);
        GC.KeepAlive(keptMesh);
        GC.KeepAlive(info2);
    }

    private static long GetWs(Process process)
    {
        process.Refresh();
        return process.WorkingSet64;
    }
}
