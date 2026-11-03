// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using OcctNet.ModelingData.Tests;
using Xunit.Abstractions;
using static Kerf.Geometry.Continuity;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeUpgrade's divide and convert tools against native OCCT's recordings (tools/shupgraderec, unit 7B), with the
/// heal group's Draw parameters, on data/occ's shapes and the heal group's public inputs up to 60 kB. The port reads each
/// input frozen, as a caller's, runs every tool on it and writes the recorder's records: the statuses, the hash of the
/// result's BRep text with the Free and Checked columns masked (decision 347: native writes into its input's flags), and
/// the numbers of faces, edges and vertices. Every input is checked unchanged.
/// </summary>
public class ShapeUpgradeNativeTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(ShapeUpgradeNativeTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static List<(string Name, List<string> Lines)> Sections()
    {
        using Stream aRes = typeof(ShapeUpgradeNativeTests).Assembly.GetManifestResourceStream("ShapeUpgrade.shupgraderec.txt.gz")
                            ?? throw new InvalidOperationException("ShapeUpgrade.shupgraderec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        var aSections = new List<(string, List<string>)>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            if (aLine.StartsWith("FILE ", StringComparison.Ordinal))
            {
                aSections.Add((aLine[5..], []));
            }
            else
            {
                aSections[^1].Item2.Add(aLine);
            }
        }
        return aSections;
    }

    // The recorder's hash: FNV-1a 64 of the BRep text, the Free and Checked columns of each line of flags masked.
    internal static ulong Hash(string theText)
    {
        ulong aHash = 14695981039346656037UL;
        string[] aLines = theText.Split('\n');
        // std::getline's lines: no empty last line after a final newline
        int aNbLines = theText.EndsWith('\n') ? aLines.Length - 1 : aLines.Length;
        for (int i = 0; i < aNbLines; i++)
        {
            string aRaw = aLines[i];
            char[] aLine = aRaw.ToCharArray();
            if (aLine.Length == 7 && aLine.All(c => c is '0' or '1'))
            {
                aLine[0] = 'x';
                aLine[2] = 'x';
            }
            foreach (char c in aLine)
            {
                aHash ^= (byte)c;
                aHash *= 1099511628211UL;
            }
            aHash ^= (byte)'\n';
            aHash *= 1099511628211UL;
        }
        return aHash;
    }

    private static string Text(in TopoDS_Shape theShape)
    {
        var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BRepTools.Write(theShape, aS, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3);
        return aS.ToString();
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        if (theShape.IsNull())
        {
            return 0;
        }
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static string Record(string theTool, in TopoDS_Shape theRes, int theStatus)
    {
        if (!theRes.IsNull() && !theRes.TShape()!.IsFrozen)
        {
            throw new InvalidOperationException($"{theTool}: the result isn't frozen");
        }
        string aText = Text(theRes);
        if (s_dump is not null && s_name is not null)
        {
            File.WriteAllText(Path.Combine(s_dump, $"{s_name}.{theTool}.brep"), aText);
        }
        return string.Create(CultureInfo.InvariantCulture,
                             $"{theTool} {theStatus:x5} {Hash(aText):x16} {Count(theRes, TopAbs_FACE)} {Count(theRes, TopAbs_EDGE)} {Count(theRes, TopAbs_VERTEX)}");
    }

    private static readonly ShapeExtend_Status[] s_flags =
    [
        ShapeExtend_DONE1, ShapeExtend_DONE2, ShapeExtend_DONE3, ShapeExtend_DONE4, ShapeExtend_DONE5, ShapeExtend_DONE6,
        ShapeExtend_DONE7, ShapeExtend_DONE8, ShapeExtend_FAIL1, ShapeExtend_FAIL2, ShapeExtend_FAIL3, ShapeExtend_FAIL4,
        ShapeExtend_FAIL5, ShapeExtend_FAIL6, ShapeExtend_FAIL7, ShapeExtend_FAIL8,
    ];

    private static int Status(Func<ShapeExtend_Status, bool> theStatus, bool theResult)
    {
        int aStatus = 0;
        for (int i = 0; i < 16; i++)
        {
            if (theStatus(s_flags[i]))
            {
                aStatus |= 1 << i;
            }
        }
        return aStatus | (theResult ? 1 << 16 : 0);
    }

    private static string Run(string theTool, Func<string> theRun)
    {
        try
        {
            Trace("  " + theTool);
            return theRun();
        }
        catch (Exception e) when (e is FailureException or NullReferenceException && e is not (ImmutableObjectException or TopoDS_LockedShape))
        {
            // an access violation natively (decision 977)
            return $"{theTool} THROW {(e is NullReferenceException ? "OSD_Exception_ACCESS_VIOLATION" : global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()))}";
        }
    }

    // OCCTNET_SHUPGRADE_TRACE: a file the runs are logged to, inputs one at a time (to find a slow one)
    private static readonly string? s_trace = Environment.GetEnvironmentVariable("OCCTNET_SHUPGRADE_TRACE");

    // OCCTNET_SHUPGRADE_DUMP (with the trace): a directory the results' texts are written to, as the recorder's
    private static readonly string? s_dump = Environment.GetEnvironmentVariable("OCCTNET_SHUPGRADE_DUMP");

    [ThreadStatic]
    private static string? s_name;

    private static void Trace(string theLine)
    {
        if (s_trace is not null)
        {
            File.AppendAllText(s_trace, theLine + "\n");
        }
    }

    private static string Divide(string theTool, ShapeUpgrade_ShapeDivide theDivider) =>
        Run(theTool, () =>
        {
            bool isDone = theDivider.Perform();
            return Record(theTool, theDivider.Result(), Status(theDivider.Status, isDone));
        });

    /// <summary>The recorder's runs on a frozen shape.</summary>
    internal static List<string> Tools(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aShape = theShape;
        var aRecords = new List<string>
        {
            Divide("ANGLE", new ShapeUpgrade_ShapeDivideAngle(95 * Math.PI / 180, aShape)),
            Divide("CLOSED", new ShapeUpgrade_ShapeDivideClosed(aShape)),
            Divide("CLEDGES", new ShapeUpgrade_ShapeDivideClosedEdges(aShape)),
        };

        var aCont = new ShapeUpgrade_ShapeDivideContinuity(aShape);
        aCont.SetTolerance(0.01);
        aRecords.Add(Divide("CONT", aCont));
        aRecords.Add(Divide("CONTD", new ShapeUpgrade_ShapeDivideContinuity(aShape)));
        var aCont2 = new ShapeUpgrade_ShapeDivideContinuity(aShape);
        aCont2.SetBoundaryCriterion(C2);
        aCont2.SetPCurveCriterion(C2);
        aCont2.SetSurfaceCriterion(C2);
        aCont2.SetTolerance(1e-3);
        aCont2.SetTolerance2d(1e-5);
        aRecords.Add(Divide("CONT2", aCont2));

        var aNum4 = new ShapeUpgrade_ShapeDivideArea(aShape);
        aNum4.SetSplittingByNumber(true);
        aNum4.NbParts() = 4;
        aRecords.Add(Divide("NUM4", aNum4));
        var aNum23 = new ShapeUpgrade_ShapeDivideArea(aShape);
        aNum23.SetSplittingByNumber(true);
        aNum23.SetNumbersUVSplits(2, 3);
        aRecords.Add(Divide("NUM23", aNum23));

        var aBez = new ShapeUpgrade_ShapeConvertToBezier(aShape);
        aBez.SetSurfaceConversion(true);
        aBez.Set2dConversion(true);
        aBez.Set3dConversion(true);
        aRecords.Add(Divide("BEZ", aBez));
        var aBezC = new ShapeUpgrade_ShapeConvertToBezier(aShape);
        aBezC.Set2dConversion(true);
        aBezC.Set3dConversion(true);
        aBezC.Set3dLineConversion(false);
        aBezC.Set3dCircleConversion(false);
        aRecords.Add(Divide("BEZC", aBezC));

        aRecords.Add(Run("RIW", () =>
        {
            var aTool = new ShapeUpgrade_RemoveInternalWires(aShape);
            aTool.MinArea() = 10;
            bool isDone = aTool.Perform();
            return Record("RIW", aTool.GetResult(), Status(aTool.Status, isDone));
        }));
        aRecords.Add(Run("SEW", () => Record("SEW", new ShapeUpgrade_ShellSewing().ApplySewing(aShape), 0)));
        return aRecords;
    }

    // The records natively different, attributed. BUC60173.rle CONT2: SplitSurfaceContinuity removes knots from the
    // basis curve of a surface of linear extrusion and, natively, sets it into the surface in place, the surface of the
    // face being divided (here the basis of a rectangular trimmed surface, shared by two faces): native's result keeps
    // the input's changed surface; the port changes a copy and leaves the caller's surface (decision 998,
    // occt-issues.md).
    private static readonly HashSet<string> s_attributed = new(StringComparer.Ordinal) { "BUC60173.rle CONT2" };

    [Fact]
    public void DivideAndConvertTools_AsNativeOCCT()
    {
        var aByName = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (string aFile in Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep"))
        {
            aByName[Path.GetFileName(aFile)] = aFile;
        }
        string aData = Metadata("OcctTestDataDir");
        if (!string.IsNullOrEmpty(aData) && Directory.Exists(aData))
        {
            foreach (string aFile in Directory.EnumerateFiles(aData, "*", SearchOption.AllDirectories))
            {
                aByName.TryAdd(Path.GetFileName(aFile), aFile);
            }
        }

        List<(string Name, List<string> Lines)> aSections = Sections();
        var aPorted = new ConcurrentDictionary<int, List<string>>();
        int aMissing = 0;
        Parallel.For(0, aSections.Count, new ParallelOptions { MaxDegreeOfParallelism = s_trace is null ? -1 : 1 }, i =>
        {
            if (!aByName.TryGetValue(aSections[i].Name, out string? aPath))
            {
                Interlocked.Increment(ref aMissing);
                return;
            }
            Assert.True(BRepTools.Read(out TopoDS_Shape aShape, aPath), aPath);
            aShape.TShape()!.FreezeGraph(); // a caller's shape
            Trace(aSections[i].Name);
            s_name = aSections[i].Name;
            ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);
            List<string> aRecords = Tools(aShape);
            aSnap.AssertUnchanged(aShape);
            aPorted[i] = aRecords;
        });

        int aFiles = 0, aRecordsCount = 0, anAttributed = 0;
        var aCounts = new SortedDictionary<string, int>(StringComparer.Ordinal);
        var aDifferences = new List<string>();
        for (int i = 0; i < aSections.Count; i++)
        {
            if (!aPorted.TryGetValue(i, out List<string>? aPort))
            {
                continue;
            }
            aFiles++;
            List<string> aNative = aSections[i].Lines;
            Assert.Equal(aNative.Count, aPort.Count);
            for (int k = 0; k < aNative.Count; k++)
            {
                string aTool = aNative[k].Split(' ')[0];
                aCounts[aTool] = aCounts.GetValueOrDefault(aTool) + 1;
                aRecordsCount++;
                if (aNative[k] == aPort[k])
                {
                    continue;
                }
                if (s_attributed.Contains($"{aSections[i].Name} {aTool}"))
                {
                    anAttributed++;
                    continue;
                }
                aDifferences.Add($"{aSections[i].Name}: '{aNative[k]}' natively, '{aPort[k]}' in the port");
            }
        }
        output.WriteLine($"{aFiles} inputs ({aMissing} not found), {aRecordsCount} records ({string.Join(", ", aCounts.Select(c => $"{c.Key} {c.Value}"))}): "
                         + $"{aRecordsCount - aDifferences.Count - anAttributed} identical, {anAttributed} attributed, {aDifferences.Count} different");
        foreach (string aD in aDifferences.Take(200))
        {
            output.WriteLine("  " + aD);
        }
        Assert.Empty(aDifferences);
        Assert.True(aFiles > 500);
    }
}
