// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;

namespace Kerf.Kernel.Benchmarks;

/// <summary>
/// The construction benchmark (unit 10H3): the operations that copied their input at the entry until 10H3 (prisms,
/// revolutions, pipes, lofts, ruled faces and shells, evolved shapes, projections, the makers of faces and wires, and
/// BRepLib's functional twins), each on frozen inputs as a caller holds them. Per operation, the median time of a
/// round of repetitions after a warm-up, and the bytes allocated per operation. Run in Release:
/// <c>-- construction [filter]</c>.
/// </summary>
public static class Construction
{
    private sealed record Case(string Name, int Repeats, Func<object> Run);

    private static Point3D P(double x, double y, double z) => new(x, y, z);

    private static Wire Polygon(bool close, params Point3D[] points)
    {
        MakePolygon aMaker = new();
        foreach (Point3D aPoint in points)
        {
            aMaker.Add(aPoint);
        }
        if (close)
        {
            aMaker.Close();
        }
        return aMaker.Wire();
    }

    private static Wire Circle(double z, double r)
        => new MakeWire(new MakeEdge(new Circle3D(new Frame3D(P(0, 0, z), new Direction3D(0, 0, 1)), r)).Edge()).Wire();

    private static List<Case> Cases()
    {
        Wire aSquare = Polygon(true, P(0, 0, 0), P(10, 0, 0), P(10, 10, 0), P(0, 10, 0));
        Face aSquareFace = new MakeFace(aSquare).Face();
        Face aCircleFace = new MakeFace(Circle(0, 5)).Face();
        Face anOffAxisFace = new MakeFace(Polygon(true, P(5, 0, 0), P(10, 0, 0), P(10, 0, 4), P(5, 0, 4))).Face();
        Wire aSpine = Polygon(false, P(0, 0, 0), P(0, 0, 20), P(0, 15, 35));
        Wire aBend = new MakeWire(new MakeEdge(new Circle3D(new Frame3D(P(30, 0, 0), new Direction3D(0, 1, 0)), 30),
                                                0, Math.PI / 2).Edge()).Wire();
        Wire aC1 = Circle(0, 5), aC2 = Circle(10, 8), aC3 = Circle(20, 4);
        Edge anE1 = new MakeEdge(P(0, 0, 0), P(10, 0, 0)).Edge();
        Edge anE2 = new MakeEdge(P(0, 5, 3), P(10, 5, 3)).Edge();
        Wire aWire1 = Polygon(false, P(0, 0, 0), P(5, 0, 0), P(10, 0, 0));
        Wire aWire2 = Polygon(false, P(0, 5, 3), P(5, 5, 3), P(10, 5, 3));
        Wire aProfile = Polygon(false, P(0, 0, 0), P(0, 0, 2));
        Edge[] aSides =
        [
            new MakeEdge(P(0, 0, 0), P(10, 0, 0)).Edge(), new MakeEdge(P(10, 0, 0), P(10, 10, 0)).Edge(),
            new MakeEdge(P(10, 10, 0), P(0, 10, 0)).Edge(), new MakeEdge(P(0, 10, 0), P(0, 0, 0)).Edge(),
        ];
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Shape aCylinder = new ApiMakeCylinder(20, 30).Shape();
        Wire aSmallSquare = Polygon(true, P(-3, -3, 40), P(3, -3, 40), P(3, 3, 40), P(-3, 3, 40));

        return
        [
            new("prism of a square face", 2000, () => new ApiMakePrism(aSquareFace, new Vector3D(0, 0, 10)).Shape()),
            new("prism of a circle face", 2000, () => new ApiMakePrism(aCircleFace, new Vector3D(0, 0, 10)).Shape()),
            new("prism, history of every edge", 1000, () =>
            {
                ApiMakePrism aPrism = new(aSquareFace, new Vector3D(0, 0, 10));
                int aCount = 0;
                for (Explorer anExp = new(aSquareFace, ShapeType.Edge); anExp.More(); anExp.Next())
                {
                    aCount += aPrism.Generated(anExp.Current()).Count;
                }
                return aCount;
            }),
            new("revolution of a face", 1000, () => new ApiMakeRevol(anOffAxisFace, new Axis3D(P(0, 0, 0), new Direction3D(0, 0, 1))).Shape()),
            new("pipe of a circle face", 200, () => new ApiMakePipe(aBend, aCircleFace).Shape()),
            new("loft of three circles", 200, () =>
            {
                ThruSections aLoft = new(true);
                aLoft.AddWire(aC1);
                aLoft.AddWire(aC2);
                aLoft.AddWire(aC3);
                aLoft.Build();
                return aLoft.Shape();
            }),
            new("ruled face of two edges", 2000, () => Fill.Face(anE1, anE2)),
            new("ruled shell of two wires", 1000, () => Fill.Shell(aWire1, aWire2)),
            new("evolved", 50, () => new MakeEvolved(aSquare, aProfile).Shape()),
            new("projection on a cylinder", 200, () => new Projection(aSmallSquare, aCylinder, new Direction3D(0, 0, 1)).Shape()),
            new("face of a frozen wire", 2000, () => new MakeFace(aSquare).Face()),
            new("face of a frozen circle", 2000, () => new MakeFace(Circle(0, 5)).Face()),
            new("wire of four frozen edges", 2000, () => new MakeWire(aSides[0], aSides[1], aSides[2], aSides[3]).Wire()),
            new("pipe along a polyline", 200, () => new ApiMakePipe(aSpine, aSmallSquare).Shape()),
            new("tolerances updated (twin)", 2000, () => BrepLibrary.TolerancesUpdated(aBox)),
        ];
    }

    public static void Run(string? filter)
    {
        Console.WriteLine($".NET {Environment.Version}, {Environment.ProcessorCount} logical processors; medians of 9 rounds after a warm-up");
        Console.WriteLine();
        Console.WriteLine("| operation | repeats | µs per operation | KB allocated per operation |");
        Console.WriteLine("|---|---:|---:|---:|");
        foreach (Case aCase in Cases())
        {
            if (filter is not null && !aCase.Name.Contains(filter, StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }
            for (int i = 0; i < Math.Max(10, aCase.Repeats / 10); i++)
            {
                aCase.Run();
            }
            var aTimes = new List<double>();
            var aBytes = new List<double>();
            for (int aRound = 0; aRound < 9; aRound++)
            {
                long anAllocated = GC.GetAllocatedBytesForCurrentThread();
                long t0 = Stopwatch.GetTimestamp();
                for (int i = 0; i < aCase.Repeats; i++)
                {
                    aCase.Run();
                }
                long t1 = Stopwatch.GetTimestamp();
                aBytes.Add((GC.GetAllocatedBytesForCurrentThread() - anAllocated) / (double)aCase.Repeats);
                aTimes.Add((t1 - t0) * 1e6 / Stopwatch.Frequency / aCase.Repeats);
            }
            aTimes.Sort();
            aBytes.Sort();
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"| {aCase.Name} | {aCase.Repeats} | {aTimes[4]:F1} | {aBytes[4] / 1024:F1} |"));
            BenchRows.Add("construction", aCase.Name, "us", aTimes[4]);
            BenchRows.Add("construction", aCase.Name, "alloc KB", aBytes[4] / 1024);
        }
    }
}
