// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using Kerf.Kernel.ModelingData.Tests;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeProcess against native OCCT's recordings (tools/shprocrec, unit 7C): OCCT's default sequences (IGES's and
/// STEP's, compiled into the port), the heal group's (data1, ToV4, remove1v, merge1v) and each operator alone, on
/// data/occ's shapes and the heal group's public inputs up to 60 kB. The port reads each input frozen, as a caller's,
/// and writes the recorder's records: Perform's result, the hash of the result's BRep text with the Free and Checked
/// columns masked (decision 347), the numbers of faces, edges and vertices, the history map (its size, and what each
/// of the input's root, solids, shells and faces became), the messages attached to the shapes (sorted texts) and what
/// the context's messenger printed. Every input is checked unchanged.
/// </summary>
public class ShapeProcessNativeTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(ShapeProcessNativeTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static List<(string Name, List<string> Lines)> Sections()
    {
        using Stream aRes = typeof(ShapeProcessNativeTests).Assembly.GetManifestResourceStream("ShapeProcess.shprocrec.txt.gz")
                            ?? throw new InvalidOperationException("ShapeProcess.shprocrec.txt.gz");
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

    private static ulong Fnv(string theText)
    {
        ulong aHash = 14695981039346656037UL;
        foreach (char c in theText)
        {
            aHash ^= (byte)c;
            aHash *= 1099511628211UL;
        }
        return aHash;
    }

    private static string Text(in Shape theShape)
    {
        var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(theShape, aS, true, false, FormatVersion.FormatVersion_VERSION_3);
        return aS.ToString();
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        if (theShape.IsNull)
        {
            return 0;
        }
        var aMap = new NCollection_IndexedMap<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static char TypeChar(in Shape theShape) => theShape.IsNull ? 'n' : theShape.ShapeType() switch
    {
        ShapeType.Compound => 'C',
        ShapeType.CompSolid => 'c',
        ShapeType.Solid => 'S',
        ShapeType.Shell => 's',
        ShapeType.Face => 'F',
        ShapeType.Wire => 'W',
        ShapeType.Edge => 'E',
        ShapeType.Vertex => 'V',
        _ => '?',
    };

    /// <summary>A printer keeping what the messenger prints (every gravity), as the recorder's.</summary>
    internal sealed class StringPrinter : Printer
    {
        public readonly StringBuilder Text = new();

        public StringPrinter() => _traceLevel = Gravity.Trace;

        protected override void SendCore(string theString, Gravity theGravity)
        {
            Text.Append(theString).Append('\n');
        }
    }

    private static string Record(string theRun, in Shape theInput, global::Kerf.Engine.Healing.Process.ShapeContext theContext, bool theDone,
                                 StringPrinter thePrinter)
    {
        Shape aRes = theContext.Result();
        if (!aRes.IsNull && !aRes.TShape()!.IsFrozen)
        {
            throw new InvalidOperationException($"{theRun}: the result isn't frozen");
        }
        string aBRep = Text(aRes);
        ulong aHash = ShapeUpgradeNativeTests.Hash(aBRep);
        if (s_dump is not null && s_name is not null)
        {
            File.WriteAllText(Path.Combine(s_dump, $"{s_name}.{(theRun.StartsWith("OP:", StringComparison.Ordinal) ? theRun[3..] : theRun)}.brep"), aBRep);
        }

        NCollection_DataMap<Shape, Shape> aMap = theContext.Map();
        var aSig = new StringBuilder();
        void Sign(in Shape theKey)
        {
            Shape aKey = theKey.Located(new Placement());
            aSig.Append(aMap.Find(aKey, out Shape aValue) ? TypeChar(aValue) : '-');
        }
        Sign(theInput);
        foreach (ShapeType aType in new[] { ShapeType.Solid, ShapeType.Shell, ShapeType.Face })
        {
            var aKeys = new NCollection_IndexedMap<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
            global::Kerf.Engine.Topology.Exploration.MapShapes(theInput, aType, aKeys);
            for (int i = 1; i <= aKeys.Extent(); i++)
            {
                Sign(aKeys[i]);
            }
        }

        var aTexts = new List<string>();
        if (theContext.Messages() is MsgRegistrator aMessages)
        {
            foreach (KeyValuePair<Shape, NCollection_List<Msg>> anItem in aMessages.MapShape())
            {
                for (var aMsg = new NCollection_List<Msg>.Iterator(anItem.Value); aMsg.More(); aMsg.Next())
                {
                    aTexts.Add(aMsg.Value().Value());
                }
            }
        }
        // std::sort of std::string: byte order
        aTexts.Sort(StringComparer.Ordinal);
        var aJoined = new StringBuilder();
        foreach (string aText in aTexts)
        {
            aJoined.Append(aText).Append('\n');
        }

        return string.Create(CultureInfo.InvariantCulture,
                             $"{theRun} {(theDone ? 1 : 0)} {aHash:x16} {Count(aRes, ShapeType.Face)} {Count(aRes, ShapeType.Edge)} {Count(aRes, ShapeType.Vertex)} "
                             + $"{aMap.Extent()} {Fnv(aSig.ToString()):x16} {aTexts.Count} {Fnv(aJoined.ToString()):x16} {Fnv(thePrinter.Text.ToString()):x16}");
    }

    // OCCTNET_SHPROC_DUMP: a directory the results' texts are written to, as the recorder's SHPROCREC_DUMP
    private static readonly string? s_dump = Environment.GetEnvironmentVariable("OCCTNET_SHPROC_DUMP");

    [ThreadStatic]
    private static string? s_name;

    private static string Run(string theRun, Func<string> theBody)
    {
        try
        {
            return theBody();
        }
        catch (Exception e) when (e is FailureException or NullReferenceException && e is not (ImmutableObjectException or global::Kerf.Engine.Topology.LockedShapeException))
        {
            // an access violation natively (decision 977)
            return $"{theRun} THROW {(e is NullReferenceException ? "OSD_Exception_ACCESS_VIOLATION" : global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()))}";
        }
    }

    private static string HealData => Path.Combine(Metadata("OcctSourceDir"), "tests", "heal", "data");

    private static string Sequence(in Shape theShape, string theRsc, string theSeq)
    {
        Shape aShape = theShape;
        // DRAWEXE's CSF_<name>Defaults: the heal group's files in its data directory, res in the dataset's brep directory;
        // IGES and STEP: OCCT's defaults, compiled into the port
        string? aDir = theRsc switch
        {
            "data" or "MDTV" => HealData,
            "res" => Path.Combine(Metadata("OcctTestDataDir"), "brep"),
            _ => null,
        };
        return Run(theSeq, () =>
        {
            var aPrinter = new StringPrinter();
            var aContext = new global::Kerf.Engine.Healing.Process.ShapeContext(aShape, theRsc, "", aDir);
            aContext.SetMessenger(new Messenger(aPrinter));
            bool isDone = global::Kerf.Engine.Healing.Process.ShapeProcess.Perform(aContext, theSeq);
            return Record(theSeq, aShape, aContext, isDone, aPrinter);
        });
    }

    private static string Operation(in Shape theShape, global::Kerf.Engine.Healing.Process.ShapeProcess.Operation theOperation)
    {
        Shape aShape = theShape;
        string aRun = "OP:" + s_operations[(int)theOperation];
        return Run(aRun, () =>
        {
            var aPrinter = new StringPrinter();
            var aContext = new global::Kerf.Engine.Healing.Process.ShapeContext(aShape, "");
            aContext.SetMessenger(new Messenger(aPrinter));
            bool isDone = global::Kerf.Engine.Healing.Process.ShapeProcess.Perform(aContext, new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(theOperation));
            return Record(aRun, aShape, aContext, isDone, aPrinter);
        });
    }

    private static readonly string[] s_operations =
    [
        "DirectFaces", "SameParameter", "SetTolerance", "SplitAngle", "BSplineRestriction", "ElementaryToRevolution",
        "SweptToElementary", "SurfaceToBSpline", "ToBezier", "SplitContinuity", "SplitClosedFaces", "FixWireGaps",
        "FixFaceSize", "DropSmallSolids", "DropSmallEdges", "FixShape", "SplitClosedEdges", "SplitCommonVertex",
    ];

    /// <summary>The recorder's runs on a frozen shape.</summary>
    internal static List<string> Runs(in Shape theShape)
    {
        var aRecords = new List<string>
        {
            Sequence(theShape, "data", "data1"),
            Sequence(theShape, "MDTV", "ToV4"),
            Sequence(theShape, "res", "remove1v"),
            Sequence(theShape, "res", "merge1v"),
            Sequence(theShape, "IGES", "ToIGES"),
            Sequence(theShape, "IGES", "FromIGES"),
            Sequence(theShape, "STEP", "ToSTEP"),
            Sequence(theShape, "STEP", "FromSTEP"),
        };
        for (global::Kerf.Engine.Healing.Process.ShapeProcess.Operation anOperation = global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.First; anOperation <= global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.Last; anOperation++)
        {
            aRecords.Add(Operation(theShape, anOperation));
        }
        return aRecords;
    }

    // The records natively different, attributed (input and run). test.brep merge1v: ShapeFix_FixSmallSolid's Merge
    // builds the merged shell from OCCT's map of new free faces in hash order (TShape addresses), the port's in a
    // deterministic order (occt-issues.md, ShapeFix): the same faces, in another order.
    private static readonly HashSet<string> s_attributed = new(StringComparer.Ordinal)
    {
        "test.brep merge1v",
    };

    // OP:SetTolerance runs BRepLib::UpdateTolerances in place, whose last step sets the tolerances it computed for each
    // (sub-shape, location) by iterating a map keyed by shape in hash order (TShape addresses): a TShape the input holds
    // at two locations (an instance), whose tolerances differ at each, ends with the last one OCCT iterates; the port
    // iterates in insertion order (occt-issues.md, unit 7C). Attributed only for inputs holding such a TShape.
    private static bool HoldsInstances(in Shape theShape)
    {
        var aSubShapes = new NCollection_IndexedMap<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(theShape, aSubShapes);
        var aTShapes = new HashSet<global::Kerf.Engine.Topology.TShape>(ReferenceEqualityComparer.Instance);
        for (int i = 1; i <= aSubShapes.Extent(); i++)
        {
            if (!aTShapes.Add(aSubShapes[i].TShape()!))
            {
                return true;
            }
        }
        return false;
    }

    // The records equal but for the result's hash
    private static bool SameButHash(string theNative, string thePort)
    {
        string[] aN = theNative.Split(' '), aP = thePort.Split(' ');
        return aN.Length == aP.Length && Enumerable.Range(0, aN.Length).All(j => j == 2 || aN[j] == aP[j]);
    }

    [Fact]
    public void Sequences_and_operators_AsNativeOCCT()
    {
        global::Kerf.Engine.Healing.Process.OperLibrary.Init();
        var aByName = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (string aFile in Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep"))
        {
            aByName[Path.GetFileName(aFile)] = aFile;
        }
        string aData = Metadata("OcctTestDataDir");
        Assert.True(!string.IsNullOrEmpty(aData) && Directory.Exists(aData), "OCCT's test data (tools/get-occt-testdata.ps1)");
        foreach (string aFile in Directory.EnumerateFiles(aData, "*", SearchOption.AllDirectories))
        {
            aByName.TryAdd(Path.GetFileName(aFile), aFile);
        }

        List<(string Name, List<string> Lines)> aSections = Sections();
        string? aOnly = Environment.GetEnvironmentVariable("OCCTNET_SHPROC_ONLY");
        var aPorted = new ConcurrentDictionary<int, List<string>>();
        var aInstances = new ConcurrentDictionary<int, bool>();
        int aMissing = 0;
        Parallel.For(0, aSections.Count, i =>
        {
            if (aOnly is not null && aSections[i].Name != aOnly)
            {
                return;
            }
            if (!aByName.TryGetValue(aSections[i].Name, out string? aPath))
            {
                Interlocked.Increment(ref aMissing);
                return;
            }
            Assert.True(BrepTools.Read(out Shape aShape, aPath), aPath);
            aShape.TShape()!.FreezeGraph(); // a caller's shape
            ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);
            if (HoldsInstances(aShape))
            {
                aInstances.TryAdd(i, true);
            }
            s_name = aSections[i].Name;
            List<string> aRecords = Runs(aShape);
            aSnap.AssertUnchanged(aShape);
            aPorted[i] = aRecords;
        });

        int aFiles = 0, aRecordsCount = 0, anAttributed = 0;
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
                aRecordsCount++;
                if (aNative[k] == aPort[k])
                {
                    continue;
                }
                string aRun = aNative[k].Split(' ')[0];
                if (s_attributed.Contains($"{aSections[i].Name} {aRun}")
                    || (aRun == "OP:SetTolerance" && aInstances.ContainsKey(i) && SameButHash(aNative[k], aPort[k])))
                {
                    anAttributed++;
                    continue;
                }
                aDifferences.Add($"{aSections[i].Name}:\n    native '{aNative[k]}'\n    port   '{aPort[k]}'");
            }
        }
        output.WriteLine($"{aFiles} inputs ({aMissing} not found), {aRecordsCount} records: "
                         + $"{aRecordsCount - aDifferences.Count - anAttributed} identical, {anAttributed} attributed, {aDifferences.Count} different");
        foreach (string aD in aDifferences.Take(300))
        {
            output.WriteLine("  " + aD);
        }
        Assert.Empty(aDifferences);
        if (aOnly is null)
        {
            Assert.True(aFiles > 500);
        }
    }
}
