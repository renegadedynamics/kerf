// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features;
using Kerf.Engine.Features.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Offsets;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Offsets;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using Kerf.Engine;

namespace Kerf.Kernel.Benchmarks;

/// <summary>
/// Offsets, fillets and features (Kerf unit 10H5) on frozen parts, as callers hold them: boxes, cylinders and Boolean
/// results of several sizes, each offset (arc and intersection joins), hollowed (a thick solid open at the top),
/// filleted and chamfered on every edge, drafted on its side faces, and given a boss (BRepFeat_MakePrism). Per
/// operation the total of each part's fastest of three runs, and the bytes allocated by that run, serially.
/// </summary>
internal static class Features
{
    private const int Rounds = 3;

    private static IEnumerable<(string Name, Shape Shape)> Parts()
    {
        for (int k = 1; k <= 4; k++)
        {
            double s = 10.0 * k;
            Shape box = new MakeBox(s, 1.5 * s, 2 * s).Shape();
            Shape cylinder = new ApiMakeCylinder(s / 2, 2 * s).Shape();
            yield return ($"box {k}", box);
            yield return ($"cylinder {k}", cylinder);
            yield return ($"box with a boss {k}", new Fuse(box, new MakeBox(new Point3D(s / 4, s / 4, 2 * s - 1), s / 2, s / 2, s / 2).Shape()).Shape());
            yield return ($"box with a hole {k}", new Cut(box, new ApiMakeCylinder(new Frame3D(new Point3D(s / 2, 0.75 * s, -1), new Direction3D(0, 0, 1)), s / 5, 3 * s).Shape()).Shape());
        }
    }

    private static List<Shape> Sub(in Shape shape, ShapeType type)
    {
        var map = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(shape, type, map);
        return [.. Enumerable.Range(1, map.Count).Select(i => map[i - 1])];
    }

    // The face with the highest centre
    private static Face Top(in Shape shape) =>
        ShapeCasts.Face(Sub(shape, ShapeType.Face).MaxBy(f =>
        {
            var b = new Kerf.Engine.Geometry.Bounds.Box();
            Kerf.Engine.Measurement.Bounds.Add(f, ref b);
            b.Get(out _, out _, out double z1, out _, out _, out double z2);
            return z1 + z2;
        })!);

    public static void Run(string? theFilter)
    {
        var operations = new (string Name, Action<Shape> Run)[]
        {
            ("offset, arc join", s =>
            {
                var o = new MakeOffsetShape();
                o.PerformByJoin(s, 1.0, 1.0e-7);
                _ = o.IsDone ? o.Shape() : s;
            }),
            ("offset, intersection", s =>
            {
                var o = new MakeOffsetShape();
                o.PerformByJoin(s, 1.0, 1.0e-7, OffsetMode.Skin, true, false, JoinKind.Intersection);
                _ = o.IsDone ? o.Shape() : s;
            }),
            ("thick solid", s =>
            {
                var closing = new NCollection_List<Shape>();
                closing.Append(Top(s));
                var t = new MakeThickSolid();
                t.MakeThickSolidByJoin(s, closing, -1.0, 1.0e-3);
                _ = t.IsDone ? t.Shape() : s;
            }),
            ("fillet every edge", s =>
            {
                var f = new MakeFillet(s);
                foreach (Shape e in Sub(s, ShapeType.Edge))
                {
                    f.Add(0.5, ShapeCasts.Edge(e));
                }
                f.Build();
                _ = f.IsDone ? f.Shape() : s;
            }),
            ("chamfer every edge", s =>
            {
                var c = new MakeChamfer(s);
                foreach (Shape e in Sub(s, ShapeType.Edge))
                {
                    c.Add(0.5, ShapeCasts.Edge(e));
                }
                c.Build();
                _ = c.IsDone ? c.Shape() : s;
            }),
            ("draft the sides", s =>
            {
                var d = new DraftAngle(s);
                foreach (Shape f in Sub(s, ShapeType.Face))
                {
                    if (Tool.Surface(ShapeCasts.Face(f)) is not PlaneSurface p || Math.Abs(p.Plane.Axis.Direction.Z) < 1.0e-9)
                    {
                        d.Add(ShapeCasts.Face(f), new Direction3D(0, 0, 1), 3.0 * Math.PI / 180.0, new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
                        if (!d.AddDone())
                        {
                            d.Remove(ShapeCasts.Face(f));
                        }
                    }
                }
                d.Build();
                _ = d.IsDone ? d.Shape() : s;
            }),
            ("boss (BRepFeat)", s =>
            {
                Face top = Top(s);
                var b = new Kerf.Engine.Geometry.Bounds.Box();
                Kerf.Engine.Measurement.Bounds.Add(top, ref b);
                b.Get(out double x1, out double y1, out _, out double x2, out double y2, out double z);
                double dx = (x2 - x1) / 8, dy = (y2 - y1) / 8;
                var polygon = new MakePolygon(new Point3D(x1 + dx, y1 + dy, z), new Point3D(x1 + 2 * dx, y1 + dy, z),
                                              new Point3D(x1 + 2 * dx, y1 + 2 * dy, z), new Point3D(x1 + dx, y1 + 2 * dy, z), true);
                Face profile = new MakeFace(polygon.Wire(), true).Face();
                var prism = new MakePrism(s, profile, top, new Direction3D(0, 0, 1), 1, true);
                prism.Perform(2.0);
                _ = prism.IsDone ? prism.Shape() : s;
            }),
        };

        var parts = Parts().Where(p => theFilter is null || p.Name.Contains(theFilter, StringComparison.OrdinalIgnoreCase)).ToList();
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{parts.Count} frozen parts, the fastest of {Rounds} runs each, serial:"));
        foreach ((string name, Action<Shape> run) in operations)
        {
            double ms = 0;
            long bytes = 0;
            foreach ((_, Shape part) in parts)
            {
                (double m, long b) = Best(() => run(part));
                ms += m;
                bytes += b;
            }
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"  {name,-22} {ms,9:F1} ms  {bytes / 1048576.0,9:F1} MB"));
        }
    }

    // The fastest of the runs, with the bytes the thread allocated in it (failures count as runs)
    private static (double Ms, long Bytes) Best(Action theRun)
    {
        double aBest = double.MaxValue;
        long aBytes = 0;
        for (int r = 0; r < Rounds; r++)
        {
            long a0 = GC.GetAllocatedBytesForCurrentThread();
            long t0 = Stopwatch.GetTimestamp();
            try
            {
                theRun();
            }
            catch (FailureException)
            {
            }
            double aMs = Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
            if (aMs < aBest)
            {
                aBest = aMs;
                aBytes = GC.GetAllocatedBytesForCurrentThread() - a0;
            }
        }
        return (aBest, aBytes);
    }
}
