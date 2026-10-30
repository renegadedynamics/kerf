// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Reflection;
using System.Text.RegularExpressions;
using OcctNet.ModelingData.Tests;
using Xunit.Abstractions;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

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
            if (!BRepTools.Read(out TopoDS_Shape aShape, aFile) || aShape.IsNull())
            {
                continue;
            }
            aShape.TShape()!.FreezeGraph();
            var aSnap = ShapeSnapshot.Of(aShape);
            aShapes++;

            var aFaces = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aShape, TopAbs_FACE, aFaces);
            int aStep = Math.Max(1, aFaces.Extent() / 40);
            for (int f = 1; f <= aFaces.Extent(); f += aStep)
            {
                TopoDS_Face aFace = TopoDS.Face(aFaces[f]);
                for (var anIt = new TopoDS_Iterator(aFace); anIt.More(); anIt.Next())
                {
                    if (anIt.Value().ShapeType() != TopAbs_WIRE)
                    {
                        continue;
                    }
                    foreach (bool isTopo in new[] { false, true })
                    {
                        aWires++;
                        var sfw = new ShapeFix_Wire();
                        try
                        {
                            sfw.SetContext(new ShapeBuild_ReShape());
                            sfw.Init(TopoDS.Wire(anIt.Value()), aFace, isTopo ? 1e-3 : BRep_Tool.Tolerance(aFace));
                            sfw.ModifyTopologyMode() = isTopo;
                            if (sfw.Perform())
                            {
                                aFixed++;
                            }
                            TopoDS_Wire aRes = sfw.Wire();
                            Assert.True(aRes.IsNull() || aRes.TShape()!.IsFrozen());
                        }
                        catch (Standard_Failure)
                        {
                            aOcctExceptions++;
                        }
                        catch (Exception e)
                        {
                            aFailures.Add($"{Path.GetFileName(aFile)} face {f} (topology {isTopo}): {e.GetType().Name}: {e.Message} {e.StackTrace?.Split('\n').FirstOrDefault()?.Trim()}");
                        }
                        foreach ((string aName, Func<ShapeExtend_Status, bool> aStatus) in new (string, Func<ShapeExtend_Status, bool>)[]
                                 {
                                     ("reorder", sfw.StatusReorder), ("small", sfw.StatusSmall), ("connected", sfw.StatusConnected),
                                     ("edge curves", sfw.StatusEdgeCurves), ("degenerated", sfw.StatusDegenerated),
                                     ("self-intersection", sfw.StatusSelfIntersection), ("lacking", sfw.StatusLacking),
                                     ("closed", sfw.StatusClosed), ("gaps 3d", sfw.StatusGaps3d), ("gaps 2d", sfw.StatusGaps2d),
                                     ("notches", sfw.StatusNotches),
                                 })
                        {
                            if (aStatus(ShapeExtend_DONE))
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
