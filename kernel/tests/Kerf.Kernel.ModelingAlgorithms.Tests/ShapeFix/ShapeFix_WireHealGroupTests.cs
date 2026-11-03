// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Diagnostics;
using System.Reflection;
using System.Text.RegularExpressions;
using Kerf.Kernel.ModelingData.Tests;
using Xunit.Abstractions;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_Wire with its face on the wires of the heal group's inputs (the test dataset's files the heal group
/// restores), frozen: every fix runs (the 2d ones too, which native ShapeFix_Shape on a wire doesn't reach; ShapeFix_Face
/// is unit 6H3's), the caller's shape never changes, the results are frozen, and nothing throws but OCCT's exceptions.
/// </summary>
public class ShapeFix_WireHealGroupTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(ShapeFix_WireHealGroupTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static List<string> HealFiles()
    {
        var aFiles = new List<string>();
        string aRoot = Metadata("OcctTestDataDir");
        if (string.IsNullOrEmpty(aRoot) || !Directory.Exists(aRoot))
        {
            return aFiles;
        }
        var aNames = new SortedSet<string>(StringComparer.Ordinal);
        var aLocate = new Regex(@"locate_data_file\s+([^\]\s]+)");
        foreach (string aCase in Directory.EnumerateFiles(Path.Combine(Metadata("OcctSourceDir"), "tests", "heal"), "*", SearchOption.AllDirectories))
        {
            foreach (Match aMatch in aLocate.Matches(File.ReadAllText(aCase)))
            {
                aNames.Add(aMatch.Groups[1].Value);
            }
        }
        var aByName = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (string aFile in Directory.EnumerateFiles(aRoot, "*", SearchOption.AllDirectories))
        {
            aByName.TryAdd(Path.GetFileName(aFile), aFile);
        }
        foreach (string aName in aNames)
        {
            if (aByName.TryGetValue(aName, out string? aFile)
                && (aName.EndsWith(".brep", StringComparison.OrdinalIgnoreCase) || aName.EndsWith(".rle", StringComparison.OrdinalIgnoreCase))
                && new FileInfo(aFile).Length <= 1_000_000)
            {
                aFiles.Add(aFile);
            }
        }
        return aFiles;
    }

    [Fact]
    public void Perform_OnTheWiresOfTheHealGroupsFaces_LeavesTheInputsUnchanged()
    {
        List<string> aFiles = HealFiles();
        if (aFiles.Count == 0)
        {
            output.WriteLine("OcctTestDataDir isn't set: nothing to do");
            return;
        }

        var aWatch = Stopwatch.StartNew();
        int aShapes = 0, aWires = 0, aFixed = 0, aOcctExceptions = 0;
        var aStatuses = new SortedDictionary<string, int>(StringComparer.Ordinal);
        var aFailures = new List<string>();
        foreach (string aFile in aFiles)
        {
            if (!BrepTools.Read(out Shape aShape, aFile) || aShape.IsNull)
            {
                continue;
            }
            aShape.TShape()!.FreezeGraph();
            var aSnap = ShapeSnapshot.Of(aShape);
            aShapes++;

            var aFaces = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(aShape, ShapeType.Face, aFaces);
            int aStep = Math.Max(1, aFaces.Extent() / 40);
            for (int f = 1; f <= aFaces.Extent(); f += aStep)
            {
                Face aFace = ShapeCasts.Face(aFaces[f]);
                for (var anIt = new Iterator(aFace); anIt.More(); anIt.Next())
                {
                    if (anIt.Value().ShapeType() != ShapeType.Wire)
                    {
                        continue;
                    }
                    foreach (bool isTopo in new[] { false, true })
                    {
                        aWires++;
                        var sfw = new WireFixer();
                        try
                        {
                            sfw.SetContext(new ReShapeBuilder());
                            sfw.Init(ShapeCasts.Wire(anIt.Value()), aFace, isTopo ? 1e-3 : Tool.Tolerance(aFace));
                            sfw.ModifyTopologyMode() = isTopo;
                            if (sfw.Perform())
                            {
                                aFixed++;
                            }
                            Wire aRes = sfw.Wire();
                            Assert.True(aRes.IsNull || aRes.TShape()!.IsFrozen);
                        }
                        catch (FailureException)
                        {
                            aOcctExceptions++;
                        }
                        catch (Exception e)
                        {
                            aFailures.Add($"{Path.GetFileName(aFile)} face {f} (topology {isTopo}): {Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType())}: {e.Message} {e.StackTrace?.Split('\n').FirstOrDefault()?.Trim()}");
                        }
                        foreach ((string aName, Func<ExtendStatus, bool> aStatus) in new (string, Func<ExtendStatus, bool>)[]
                                 {
                                     ("reorder", sfw.StatusReorder), ("small", sfw.StatusSmall), ("connected", sfw.StatusConnected),
                                     ("edge curves", sfw.StatusEdgeCurves), ("degenerated", sfw.StatusDegenerated),
                                     ("self-intersection", sfw.StatusSelfIntersection), ("lacking", sfw.StatusLacking),
                                     ("closed", sfw.StatusClosed), ("gaps 3d", sfw.StatusGaps3d), ("gaps 2d", sfw.StatusGaps2d),
                                     ("notches", sfw.StatusNotches),
                                 })
                        {
                            if (aStatus(ExtendStatus.Done))
                            {
                                aStatuses[aName] = aStatuses.GetValueOrDefault(aName) + 1;
                            }
                        }
                    }
                }
            }
            aSnap.AssertUnchanged(aShape);
        }

        output.WriteLine($"{aShapes} shapes, {aWires} wire fixes, {aFixed} fixed, {aOcctExceptions} OCCT exceptions, {aWatch.Elapsed.TotalSeconds:F0} s");
        output.WriteLine("fixes done: " + string.Join(", ", aStatuses.Select(p => $"{p.Key} {p.Value}")));
        foreach (string aF in aFailures.Take(30))
        {
            output.WriteLine(aF);
        }
        Assert.Empty(aFailures);
    }
}
