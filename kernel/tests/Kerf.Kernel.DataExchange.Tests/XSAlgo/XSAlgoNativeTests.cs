// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DataExchange.Tests.XSAlgo;

/// <summary>
/// XSAlgo against native OCCT's recording (tools/xsalgorec, unit 8A2), on data/occ's shapes up to 600 kB: the shape
/// processor with STEP's default sequences (FromSTEP and ToSTEP read from the STEP resource file by
/// ReadProcessingData, DESTEP's write operations with the default DE_ShapeFixParameters), the merge of its history into
/// a transient process and a finder process, the AlgoContainer's ProcessShape (STEP's sequence, the default ShapeFix,
/// the default DirectFaces) and CheckPCurve on every edge of every face. The port reads each input frozen, as a
/// caller's (but for CheckPCurve, which writes into the edges it checks, as a reader's edges under construction), and
/// checks it unchanged.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class XSAlgoNativeTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(XSAlgoNativeTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static List<string> Lines()
    {
        using Stream aRes = typeof(XSAlgoNativeTests).Assembly.GetManifestResourceStream("XSAlgo.xsalgorec.txt.gz")
                            ?? throw new InvalidOperationException("XSAlgo.xsalgorec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        var aLines = new List<string>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            aLines.Add(aLine);
        }
        return aLines;
    }

    internal static ulong Fnv(string theText)
    {
        ulong aHash = 14695981039346656037UL;
        foreach (char c in theText)
        {
            aHash ^= (byte)c;
            aHash *= 1099511628211UL;
        }
        return aHash;
    }

    // The BRep text with the Free and Checked columns of the flags masked (decision 347)
    internal static ulong BRepHash(in Shape theShape)
    {
        var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(theShape, aS, true, false, FormatVersion.FormatVersion_VERSION_3);
        string aText = aS.ToString();
        string[] aLines = aText.Split('\n');
        // std::getline's lines: no empty last line after a final newline; each line ends with one
        int aNbLines = aText.EndsWith('\n') ? aLines.Length - 1 : aLines.Length;
        var aRes = new StringBuilder();
        for (int i = 0; i < aNbLines; i++)
        {
            string aLine = aLines[i];
            if (aLine.Length == 7 && aLine.All(c => c is '0' or '1'))
            {
                aLine = "x" + aLine[1] + "x" + aLine[3..];
            }
            aRes.Append(aLine).Append('\n');
        }
        return Fnv(aRes.ToString());
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

    private static List<Shape> Keys(in Shape theShape)
    {
        var aKeys = new List<Shape> { theShape };
        foreach (ShapeType aType in new[] { ShapeType.Solid, ShapeType.Shell, ShapeType.Face, ShapeType.Edge })
        {
            var aMap = new NCollection_IndexedMap<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
            global::Kerf.Engine.Topology.Exploration.MapShapes(theShape, aType, aMap);
            for (int i = 1; i <= aMap.Extent(); i++)
            {
                aKeys.Add(aMap[i]);
            }
        }
        return aKeys;
    }

    private static string Messages(MsgRegistrator? theMessages, out int theNb)
    {
        var aTexts = new List<string>();
        if (theMessages is not null)
        {
            foreach (KeyValuePair<Shape, NCollection_List<Msg>> anItem in theMessages.MapShape())
            {
                for (var aMsg = new NCollection_List<Msg>.Iterator(anItem.Value); aMsg.More(); aMsg.Next())
                {
                    aTexts.Add(aMsg.Value().Value());
                }
            }
        }
        aTexts.Sort(StringComparer.Ordinal);
        theNb = aTexts.Count;
        return string.Concat(aTexts.Select(t => t + "\n"));
    }

    private static ulong MergeTP(List<Shape> theKeys, Action<global::Kerf.Engine.Exchange.Framework.Transfer.TransientProcess> theMerge)
    {
        var aTP = new global::Kerf.Engine.Exchange.Framework.Transfer.TransientProcess();
        for (int i = 0; i < theKeys.Count; i++)
        {
            Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetShapeResult(aTP, new TCollection_HAsciiString(i), theKeys[i]);
        }
        theMerge(aTP);
        var aLines = new StringBuilder();
        for (int i = 1; i <= aTP.MappedCount; i++)
        {
            var aB = (global::Kerf.Engine.Exchange.Framework.Transfer.ShapeBinder)aTP.MapItem(i)!;
            aLines.Append(CultureInfo.InvariantCulture,
                          $"{TypeChar(aB.Result())} {(aB.Result().IsSame(theKeys[i - 1]) ? 1 : 0)} {aB.Check().WarningCount}\n");
        }
        return Fnv(aLines.ToString());
    }

    private static ulong MergeFP(in Shape theInput, Action<global::Kerf.Engine.Exchange.Framework.Transfer.FinderProcess> theMerge)
    {
        var aFP = new global::Kerf.Engine.Exchange.Framework.Transfer.FinderProcess();
        var aFaces = new NCollection_IndexedMap<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(theInput, ShapeType.Face, aFaces);
        for (int i = 1; i <= aFaces.Extent(); i++)
        {
            Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetTransientFromShape(aFP, aFaces[i], new TCollection_HAsciiString(i));
        }
        theMerge(aFP);
        var aLines = new List<string>();
        for (int i = 1; i <= aFP.MappedCount; i++)
        {
            var aLine = new StringBuilder(aFP.Mapped(i)!.ValueTypeName());
            int aNbW = 0;
            for (global::Kerf.Engine.Exchange.Framework.Transfer.Binder? aN = aFP.MapItem(i); aN is not null; aN = aN.NextResult())
            {
                aLine.Append(' ').Append(aN.ResultTypeName());
                aNbW += aN.Check().WarningCount;
            }
            aLine.Append(' ').Append(aNbW.ToString(CultureInfo.InvariantCulture));
            aLines.Add(aLine.ToString());
        }
        aLines.Sort(StringComparer.Ordinal);
        return Fnv(string.Concat(aLines.Select(l => l + "\n")));
    }

    private static string Record(string theRun, in Shape theInput, in Shape theResult,
                                 global::Kerf.Engine.Healing.Process.ShapeContext? theContext, ulong theMerge)
    {
        if (!theResult.IsNull && !theResult.IsSame(theInput) && !theResult.TShape()!.IsFrozen)
        {
            throw new InvalidOperationException($"{theRun}: the result isn't frozen");
        }
        int aNbMsg = 0;
        string aMsgs = theContext is null ? "" : Messages(theContext.Messages(), out aNbMsg);
        return string.Create(CultureInfo.InvariantCulture,
                             $"{theRun} {(theResult.IsSame(theInput) ? 1 : 0)} {BRepHash(theResult):x16} {Count(theResult, ShapeType.Face)} "
                             + $"{Count(theResult, ShapeType.Edge)} {Count(theResult, ShapeType.Vertex)} {(theContext is null ? 0 : theContext.Map().Extent())} "
                             + $"{aNbMsg} {Fnv(aMsgs):x16} {theMerge:x16}");
    }

    private static string Run(string theRun, Func<string> theBody)
    {
        try
        {
            return theBody();
        }
        catch (FailureException e)
        {
            return $"{theRun} THROW {Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType())}";
        }
    }

    private static string FlagsText(global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags theFlags)
    {
        var aText = new StringBuilder();
        for (global::Kerf.Engine.Healing.Process.ShapeProcess.Operation i = global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.First; i <= global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.Last; i++)
        {
            aText.Append(theFlags.Test(i) ? '1' : '0');
        }
        return aText.ToString();
    }

    private static ulong ParamsHash(NCollection_DataMap<string, string> theParams)
    {
        var aLines = theParams.Select(p => p.Key + "=" + p.Value).ToList();
        aLines.Sort(StringComparer.Ordinal);
        return Fnv(string.Concat(aLines.Select(l => l + "\n")));
    }

    /// <summary>The statics of a STEP reader: XSAlgo's, and STEPControl_Controller's naming STEP's resource file and sequences.</summary>
    internal static global::Kerf.Engine.Exchange.Framework.StaticSet StepStatics()
    {
        var aStatics = new global::Kerf.Engine.Exchange.Framework.StaticSet();
        Kerf.Engine.Exchange.Framework.XSAlgo.Init(aStatics);
        aStatics.Init("step", "read.step.resource.name", 't', "STEP");
        aStatics.Init("step", "read.step.sequence", 't', "FromSTEP");
        aStatics.Init("step", "write.step.resource.name", 't', "STEP");
        aStatics.Init("step", "write.step.sequence", 't', "ToSTEP");
        return aStatics;
    }

    private sealed class Setup
    {
        public readonly global::Kerf.Engine.Exchange.Framework.StaticSet Statics = StepStatics();
        public readonly (NCollection_DataMap<string, string>, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags) Read, ToStep;
        public readonly NCollection_DataMap<string, string> WriteParams = new();
        public readonly global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags WriteFlags =
            new(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.SplitCommonVertex, global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.DirectFaces);

        public Setup()
        {
            Read = global::Kerf.Engine.Exchange.Framework.ShapeProcessor.ReadProcessingData("read.step.resource.name", "read.step.sequence", Statics);
            global::Kerf.Engine.Exchange.Framework.ShapeProcessor.SetParameter("FixShape.Tolerance3d", 1.0e-3, true, Read.Item1);
            global::Kerf.Engine.Exchange.Framework.ShapeProcessor.SetParameter("FixShape.MaxTolerance3d", 1.0, true, Read.Item1);
            ToStep = global::Kerf.Engine.Exchange.Framework.ShapeProcessor.ReadProcessingData("write.step.resource.name", "write.step.sequence", Statics);
            global::Kerf.Engine.Exchange.Framework.ShapeProcessor.FillParameterMap(new ShapeFixParameters(), true, WriteParams);
        }

        public List<string> Params() =>
        [
            $"PARAMS FromSTEP {FlagsText(Read.Item2)} {ParamsHash(Read.Item1):x16}",
            $"PARAMS ToSTEP {FlagsText(ToStep.Item2)} {ParamsHash(ToStep.Item1):x16}",
            $"PARAMS DE {FlagsText(WriteFlags)} {ParamsHash(WriteParams):x16}",
        ];
    }

    private static string Processor(in Shape theInput, string theRun, NCollection_DataMap<string, string> theParams,
                                    global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags theFlags, bool theToFinder)
    {
        Shape anInput = theInput;
        return Run(theRun, () =>
        {
            var aProcessor = new global::Kerf.Engine.Exchange.Framework.ShapeProcessor(theParams);
            Shape aResult = aProcessor.ProcessShape(anInput, theFlags);
            ulong aMerge = theToFinder
                ? MergeFP(anInput, aProcessor.MergeTransferInfo)
                : MergeTP(Keys(anInput), theTP => aProcessor.MergeTransferInfo(theTP, 1));
            return Record(theRun, anInput, aResult, aProcessor.GetContext(), aMerge);
        });
    }

    private static string Container(in Shape theInput, string theRun, string theSeq, global::Kerf.Engine.Exchange.Framework.StaticSet theStatics)
    {
        Shape anInput = theInput;
        return Run(theRun, () =>
        {
            Transient? anInfo = null;
            Shape aResult = Kerf.Engine.Exchange.Framework.XSAlgo.AlgoContainer().ProcessShape(anInput, 1.0e-3, 1.0, "read.step.resource.name",
                                                                           theSeq, ref anInfo, theStatics);
            ulong aMerge = MergeTP(Keys(anInput), theTP => Kerf.Engine.Exchange.Framework.XSAlgo.AlgoContainer().MergeTransferInfo(theTP, anInfo, 1));
            return Record(theRun, anInput, aResult, anInfo as global::Kerf.Engine.Healing.Process.ShapeContext, aMerge);
        });
    }

    private static string PCurves(string thePath, global::Kerf.Engine.Exchange.Framework.StaticSet theStatics)
    {
        return Run("PCURVE", () =>
        {
            Assert.True(BrepTools.Read(out Shape aRead, thePath));
            Shape anInput = BrepTools.ExactCopyForWrite(aRead, out _); // a reader's shape, not frozen
            var aFaces = new NCollection_IndexedMap<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
            global::Kerf.Engine.Topology.Exploration.MapShapes(anInput, ShapeType.Face, aFaces);
            int aNb = 0, aNbTrue = 0;
            var aLines = new StringBuilder();
            for (int i = 1; i <= aFaces.Extent(); i++)
            {
                Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(aFaces[i]);
                for (var anExp = new global::Kerf.Engine.Topology.Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
                {
                    Edge anEdge = global::Kerf.Engine.Topology.ShapeCasts.Edge(anExp.Current());
                    bool isSeam = Tool.IsClosed(anEdge, aFace);
                    bool isOk = global::Kerf.Engine.Exchange.Framework.ShapeProcessor.CheckPCurve(anEdge, aFace, 1.0e-3, isSeam, theStatics);
                    aLines.Append(isOk ? '1' : '0').Append(' ')
                          .Append(G17(Tool.Tolerance(anEdge)))
                          .Append('\n');
                    aNb++;
                    aNbTrue += isOk ? 1 : 0;
                }
            }
            return string.Create(CultureInfo.InvariantCulture, $"PCURVE {aNb} {aNbTrue} {Fnv(aLines.ToString()):x16} {BRepHash(anInput):x16}");
        });
    }

    // The records natively different, attributed (input and run): only the finder merge's hash differs. ShapeProcess
    // works on a copy of a frozen input (decision 1022), so a face OCCT's operators change in place (a history entry
    // F -> F) is a changed copy in the port (F -> F'): the merge then binds the input's pre-bound faces differently.
    // On an unfrozen input, which the port's operators change in place too, the merge is native's
    // (Finder_merge_of_an_unfrozen_shape_is_natives).
    private static readonly HashSet<string> s_attributed = new(StringComparer.Ordinal)
    {
        "CrankArm.brep WRITE", "CrankArm.brep TOSTEP", "CrankArm.brep AC-WRITE",
    };

    // The records equal but for the merge's hash
    private static bool SameButMerge(string theNative, string thePort)
    {
        string[] aN = theNative.Split(' '), aP = thePort.Split(' ');
        return aN.Length == aP.Length && Enumerable.Range(0, aN.Length).All(j => j == aN.Length - 1 || aN[j] == aP[j]);
    }

    /// <summary>
    /// The attribution's ground: on an unfrozen CrankArm.brep, which ShapeProcess changes in place as OCCT does, the
    /// write processing's merge into a finder process is native's.
    /// </summary>
    [Fact]
    public void Finder_merge_of_an_unfrozen_shape_is_natives()
    {
        string aPath = Path.Combine(Metadata("OcctSourceDir"), "data", "occ", "CrankArm.brep");
        Assert.True(BrepTools.Read(out Shape aRead, aPath), aPath);
        Shape aShape = BrepTools.ExactCopyForWrite(aRead, out _);
        var aSetup = new Setup();
        var aProcessor = new global::Kerf.Engine.Exchange.Framework.ShapeProcessor(aSetup.WriteParams);
        aProcessor.ProcessShape(aShape, aSetup.WriteFlags);
        ulong aMerge = MergeFP(aShape, aProcessor.MergeTransferInfo);
        string aNative = Lines().SkipWhile(l => l != "FILE CrankArm.brep").First(l => l.StartsWith("WRITE ", StringComparison.Ordinal));
        Assert.Equal(aNative.Split(' ')[^1], aMerge.ToString("x16", CultureInfo.InvariantCulture));
    }

    // C's %.17g
    private static string G17(double theValue) => StreamUtilities.FormatReal(theValue, 17);

    [Fact]
    public void ShapeProcessor_AlgoContainer_and_CheckPCurve_AsNativeOCCT()
    {
        string aDir = Path.Combine(Metadata("OcctSourceDir"), "data", "occ");
        Assert.True(Directory.Exists(aDir), "OCCT's source (tools/get-occt.ps1)");
        List<string> aNative = Lines();
        var aSetup = new Setup();
        Assert.Equal(aNative.Take(3), aSetup.Params());

        // the files' sections
        var aSections = new List<(string Name, List<string> Lines)>();
        foreach (string aLine in aNative.Skip(3))
        {
            if (aLine.StartsWith("FILE ", StringComparison.Ordinal))
            {
                aSections.Add((aLine[5..], []));
            }
            else
            {
                aSections[^1].Lines.Add(aLine);
            }
        }
        Assert.True(aSections.Count >= 25, $"{aSections.Count} inputs recorded");

        var aPorted = new ConcurrentDictionary<int, List<string>>();
        Parallel.For(0, aSections.Count, i =>
        {
            string aPath = Path.Combine(aDir, aSections[i].Name);
            Assert.True(BrepTools.Read(out Shape aShape, aPath), aPath);
            aShape.TShape()!.FreezeGraph(); // a caller's shape
            ulong aBefore = BRepHash(aShape);
            // each worker its own statics (a static set is a working object)
            global::Kerf.Engine.Exchange.Framework.StaticSet aStatics = StepStatics();
            var aRecords = new List<string>
            {
                Processor(aShape, "READ", aSetup.Read.Item1, aSetup.Read.Item2, false),
                Processor(aShape, "WRITE", aSetup.WriteParams, aSetup.WriteFlags, true),
                Processor(aShape, "TOSTEP", aSetup.ToStep.Item1, aSetup.ToStep.Item2, true),
                Container(aShape, "AC-READ", "read.step.sequence", aStatics),
                Container(aShape, "AC-FIX", "read.none", aStatics),
                Container(aShape, "AC-WRITE", "write.none", aStatics),
                PCurves(aPath, aStatics),
            };
            Assert.Equal(aBefore, BRepHash(aShape)); // the input unchanged
            aPorted[i] = aRecords;
        });

        int aRecordsCount = 0, anIdentical = 0, anAttributed = 0;
        var aDifferences = new List<string>();
        for (int i = 0; i < aSections.Count; i++)
        {
            List<string> aPort = aPorted[i];
            Assert.Equal(aSections[i].Lines.Count, aPort.Count);
            for (int j = 0; j < aPort.Count; j++)
            {
                aRecordsCount++;
                string aNat = aSections[i].Lines[j];
                if (aNat == aPort[j])
                {
                    anIdentical++;
                }
                else if (s_attributed.Contains(aSections[i].Name + " " + aNat.Split(' ')[0]) && SameButMerge(aNat, aPort[j]))
                {
                    anAttributed++;
                }
                else
                {
                    aDifferences.Add($"{aSections[i].Name}\n  native {aNat}\n  port   {aPort[j]}");
                }
            }
        }
        output.WriteLine($"{aSections.Count} inputs, {aRecordsCount} records: {anIdentical} identical, {anAttributed} attributed, "
                         + $"{aDifferences.Count} different");
        foreach (string aDifference in aDifferences)
        {
            output.WriteLine(aDifference);
        }
        Assert.Empty(aDifferences);
        Assert.Equal(s_attributed.Count, anAttributed);
    }
}
