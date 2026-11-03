// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// STEPControl's reader and writer against native OCCT's (build123d.NET's bound STEPControl_Reader and
/// STEPControl_Writer, unit 8F): the corpus's STEP files read by both (status, roots, shapes, the shape's BRep text),
/// data/occ's shapes and assemblies of located instances written by both in each model type (the STEP text, the
/// header's FILE_NAME and the assembly occurrences' identifiers masked), and the round trips (the port's files read
/// natively, native files read by the port). The native side runs with OCCT's default statics (AP214IS); the other
/// schemas and the transfer's checks are compared through tools/stepctlrec (DataExchange.Tests).
/// </summary>
public class STEPControlOracleTests(ITestOutputHelper theOutput) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-stepcontrol-").FullName;
    private int myFileNumber;
    private static int theDumpNumber;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private static string Metadata(string theKey) =>
        typeof(STEPControlOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    internal static List<string> CorpusFiles()
    {
        var aFiles = new List<string>();
        foreach (string aDir in new[] { Path.Combine(Metadata("OcctTestDataDir"), "step"), Path.Combine(Metadata("OcctSourceDir"), "data", "step") })
        {
            if (Directory.Exists(aDir))
            {
                aFiles.AddRange(Directory.GetFiles(aDir).Where(f => f.EndsWith(".stp", StringComparison.OrdinalIgnoreCase)
                                                                    || f.EndsWith(".step", StringComparison.OrdinalIgnoreCase))
                                         .Order(StringComparer.Ordinal));
            }
        }
        return aFiles;
    }

    private string TempFile(string theExtension) => Path.Combine(myDirectory, $"f{myFileNumber++}{theExtension}");

    private string NativeBRep(N.TopoDS_Shape theShape)
    {
        string aFile = TempFile(".brep");
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile);
    }

    private static N.TopoDS_Shape NativeReadBRep(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return aShape;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Reading

    /// <summary>What a reader gives: the read status, the roots, the shapes, the shape's BRep text.</summary>
    private sealed record ReadResult(int Status, int NbRoots, int NbTransferred, int NbShapes, string BRep);

    private ReadResult NativeRead(string thePath)
    {
        using var aReader = new N.STEPControl_Reader();
        int aStatus = (int)aReader.ReadFile(thePath);
        if (aStatus != (int)N.IFSelect_ReturnStatus.IFSelect_RetDone)
        {
            return new ReadResult(aStatus, 0, 0, 0, "");
        }
        int aNbRoots = aReader.NbRootsForTransfer();
        int aNbTransferred = aReader.TransferRoots();
        int aNbShapes = aReader.NbShapes();
        string aBRep = aNbShapes == 0 ? "" : NativeBRep(aReader.OneShape());
        return new ReadResult(aStatus, aNbRoots, aNbTransferred, aNbShapes, aBRep);
    }

    private ReadResult PortRead(string thePath)
    {
        var aReader = new StepReader();
        int aStatus = (int)aReader.ReadFile(thePath);
        if (aStatus != (int)ReturnStatus.RetDone)
        {
            return new ReadResult(aStatus, 0, 0, 0, "");
        }
        int aNbRoots = aReader.NbRootsForTransfer();
        int aNbTransferred = aReader.TransferRoots();
        int aNbShapes = aReader.ShapeCount;
        string aBRep = aNbShapes == 0 ? "" : PortBRep(aReader.OneShape());
        return new ReadResult(aStatus, aNbRoots, aNbTransferred, aNbShapes, aBRep);
    }

    // the BRep file's text, as native's (with Draw's header line)
    private string PortBRep(Shape theShape)
    {
        string aFile = TempFile(".brep");
        Assert.True(BrepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile);
    }

    /// <summary>
    /// How two reads compare: "identical" (BRep bytes), "free" (identical but for TShapes' Free flags),
    /// "canonical" (the same sub-shapes, CanonicalShapeComparer), or what differs.
    /// </summary>
    private string CompareReads(ReadResult theNative, ReadResult thePort)
    {
        if (theNative.Status != thePort.Status || theNative.NbRoots != thePort.NbRoots
            || theNative.NbTransferred != thePort.NbTransferred || theNative.NbShapes != thePort.NbShapes)
        {
            return $"counts: native {theNative.Status}/{theNative.NbRoots}/{theNative.NbTransferred}/{theNative.NbShapes}, "
                   + $"port {thePort.Status}/{thePort.NbRoots}/{thePort.NbTransferred}/{thePort.NbShapes}";
        }
        BRepTextDiff aDiff = BRepTextComparison.Compare(theNative.BRep, thePort.BRep);
        if (!aDiff.IsIdenticalMaskingFree && Environment.GetEnvironmentVariable("OCCTNET_STEP_DUMP") is { } aDump)
        {
            int aNumber = Interlocked.Increment(ref theDumpNumber);
            File.WriteAllText(Path.Combine(aDump, $"d{aNumber}-native.brep"), theNative.BRep);
            File.WriteAllText(Path.Combine(aDump, $"d{aNumber}-port.brep"), thePort.BRep);
        }
        if (aDiff.IsIdentical)
        {
            return "identical";
        }
        if (aDiff.IsIdenticalMaskingFree)
        {
            return "free";
        }
        // the canonical comparison of the two shapes, both read by the port from the texts
        Shape aNative = ReadPortBRep(theNative.BRep);
        Shape aPort = ReadPortBRep(thePort.BRep);
        CanonicalComparison aCanonical = CanonicalShapeComparer.Compare(aNative, aPort,
                                                                        new CanonicalOptions { CheckValidity = false });
        return aCanonical.IsIdentical ? $"canonical: {aDiff}" : $"differs: {aDiff}; {aCanonical}";
    }

    private static Shape ReadPortBRep(string theText)
    {
        using var aReader = new StringReader(theText);
        BrepTools.Read(out Shape aShape, aReader);
        return aShape;
    }

    [Fact]
    public void ReadsTheCorpusAsNative()
    {
        List<string> aFiles = CorpusFiles();
        if (aFiles.Count == 0)
        {
            return; // OCCT's dataset not fetched
        }
        var aCounts = new SortedDictionary<string, int>(StringComparer.Ordinal);
        var aDifferent = new List<string>();
        foreach (string aPath in Tape.Budget(aFiles, isSpread: true, theWeight: Tape.FileSize, theMaxWeight: 200_000))
        {
            ReadResult aNative = NativeRead(aPath);
            ReadResult aPort = PortRead(aPath);
            string aVerdict = CompareReads(aNative, aPort);
            string aKind = aVerdict.Split(':')[0];
            aCounts[aKind] = aCounts.GetValueOrDefault(aKind) + 1;
            if (aKind is not ("identical" or "free" or "canonical"))
            {
                aDifferent.Add($"{Path.GetFileName(aPath)}: {aVerdict}");
            }
        }
        theOutput.WriteLine($"{aFiles.Count} files: " + string.Join(", ", aCounts.Select(c => $"{c.Key} {c.Value}")));
        foreach (string aLine in aDifferent)
        {
            theOutput.WriteLine(aLine);
        }
        Assert.DoesNotContain(aDifferent, d => !KnownReadDifferences.Contains(d.Split(":")[0]));
    }

    // Files whose reading differs from native's, each attributed (decision 1272)
    private static readonly HashSet<string> KnownReadDifferences = new(StringComparer.Ordinal)
    {
    };

    // -----------------------------------------------------------------------------------------------------------------
    // Writing

    private static readonly (ModelType Port, N.STEPControl_StepModelType Native)[] ModelTypes =
    [
        (ModelType.AsIs, N.STEPControl_StepModelType.STEPControl_AsIs),
        (ModelType.ManifoldSolidBrep, N.STEPControl_StepModelType.STEPControl_ManifoldSolidBrep),
        (ModelType.BrepWithVoids, N.STEPControl_StepModelType.STEPControl_BrepWithVoids),
        (ModelType.FacetedBrep, N.STEPControl_StepModelType.STEPControl_FacetedBrep),
        (ModelType.ShellBasedSurfaceModel, N.STEPControl_StepModelType.STEPControl_ShellBasedSurfaceModel),
        (ModelType.GeometricCurveSet, N.STEPControl_StepModelType.STEPControl_GeometricCurveSet),
    ];

    private static readonly Regex FileNameRecord = new(@"FILE_NAME\(.*?\);\r?\n", RegexOptions.Singleline | RegexOptions.Compiled);
    private static readonly Regex NauoId = new(@"NEXT_ASSEMBLY_USAGE_OCCURRENCE\('[^']*'", RegexOptions.Compiled);
    private static readonly Regex ProductIndex = new(@"(Open CASCADE STEP translator [0-9]+\.[0-9]+) [0-9.]+", RegexOptions.Compiled);

    /// <summary>The STEP text with the header's FILE_NAME record (time, author, file name) and the NAUO identifiers masked.</summary>
    internal static string Masked(string theText) =>
        ProductIndex.Replace(NauoId.Replace(FileNameRecord.Replace(theText.Replace("\r\n", "\n"), "FILE_NAME(...);\n"),
                                            "NEXT_ASSEMBLY_USAGE_OCCURRENCE('#'"), "$1 #");

    private (int Status, string Text, string File) NativeWrite(string theBRepFile, N.STEPControl_StepModelType theMode)
    {
        N.TopoDS_Shape aShape = NativeReadBRep(theBRepFile);
        using var aWriter = new N.STEPControl_Writer();
        int aStatus = (int)aWriter.Transfer(aShape, theMode);
        string aFile = TempFile(".stp");
        if (aStatus != (int)N.IFSelect_ReturnStatus.IFSelect_RetDone)
        {
            return (aStatus, "", aFile);
        }
        Assert.Equal(N.IFSelect_ReturnStatus.IFSelect_RetDone, aWriter.Write(aFile));
        return (aStatus, File.ReadAllText(aFile), aFile);
    }

    private (int Status, string Text, string File) PortWrite(string theBRepFile, ModelType theMode)
    {
        Assert.True(BrepTools.Read(out Shape aShape, theBRepFile));
        string aBefore = BRepTextComparison.Write(aShape);
        var aWriter = new Writer();
        int aStatus = (int)aWriter.Transfer(aShape, theMode);
        Assert.Equal(aBefore, BRepTextComparison.Write(aShape)); // the writer never changes its input
        string aFile = TempFile(".stp");
        if (aStatus != (int)ReturnStatus.RetDone)
        {
            return (aStatus, "", aFile);
        }
        Assert.Equal(ReturnStatus.RetDone, aWriter.Write(aFile));
        return (aStatus, File.ReadAllText(aFile), aFile);
    }

    /// <summary>The shapes written: data/occ's .brep files, and assemblies of located instances made from some of them.</summary>
    private List<(string Name, string File)> WriteInputs()
    {
        var anInputs = new List<(string, string)>();
        string aDir = Path.Combine(Metadata("OcctSourceDir"), "data", "occ");
        if (!Directory.Exists(aDir))
        {
            return anInputs;
        }
        string[] aBReps = Directory.GetFiles(aDir, "*.brep").Order(StringComparer.Ordinal).ToArray();
        foreach (string aFile in aBReps)
        {
            anInputs.Add((Path.GetFileName(aFile), aFile));
        }

        // assemblies: two instances of a shape (one translated and rotated), and a nested assembly of two shapes
        var aBuilder = new BrepBuilder();
        foreach (string aName in new[] { "Ball.brep", "Pump_Nut.brep", "bottle.brep" })
        {
            string aFile = Path.Combine(aDir, aName);
            if (!File.Exists(aFile))
            {
                continue;
            }
            Assert.True(BrepTools.Read(out Shape aShape, aFile));
            var aTrsf = new Transform3D();
            aTrsf.SetRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.5);
            aTrsf.SetTranslationPart(new Vector3D(100.0, 50.0, 0.0));
            var anAssembly = new Compound();
            aBuilder.MakeCompound(ref anAssembly);
            aBuilder.Add(anAssembly, aShape);
            aBuilder.Add(anAssembly, aShape.Moved(new Placement(aTrsf)));
            var aTop = new Compound();
            aBuilder.MakeCompound(ref aTop);
            var aTrsf2 = new Transform3D();
            aTrsf2.SetTranslation(new Vector3D(0.0, 0.0, 300.0));
            aBuilder.Add(aTop, anAssembly);
            aBuilder.Add(aTop, anAssembly.Moved(new Placement(aTrsf2)));
            string anAssemblyFile = TempFile(".brep");
            Assert.True(BrepTools.Write(aTop, anAssemblyFile));
            anInputs.Add(($"assembly of {aName}", anAssemblyFile));
        }
        return anInputs;
    }

    [Fact]
    public void WritesAsNativeAndRoundTrips()
    {
        List<(string Name, string File)> anInputs = WriteInputs();
        if (anInputs.Count == 0)
        {
            return;
        }
        var aCounts = new SortedDictionary<string, int>(StringComparer.Ordinal);
        var aDifferent = new List<string>();
        void Count(string theKind, string theCase, string theVerdict)
        {
            string aKind = theVerdict.Split(':')[0];
            aCounts[$"{theKind} {aKind}"] = aCounts.GetValueOrDefault($"{theKind} {aKind}") + 1;
            if (aKind is not ("identical" or "free" or "canonical"))
            {
                aDifferent.Add($"{theKind} {theCase}: {theVerdict}");
            }
        }

        foreach ((string aName, string aFile) in Tape.Budget(anInputs, isSpread: true))
        {
            foreach ((ModelType aPortMode, N.STEPControl_StepModelType aNativeMode) in ModelTypes)
            {
                string aCase = $"{aName} {aPortMode}";
                (int Status, string Text, string File) aNative = NativeWrite(aFile, aNativeMode);
                (int Status, string Text, string File) aPort = PortWrite(aFile, aPortMode);
                if (aNative.Status != aPort.Status)
                {
                    Count("write", aCase, $"status: native {aNative.Status}, port {aPort.Status}");
                    continue;
                }
                if (aNative.Status != (int)ReturnStatus.RetDone)
                {
                    Count("write", aCase, "identical"); // both refuse
                    continue;
                }
                string aNativeText = Masked(aNative.Text);
                string aPortText = Masked(aPort.Text);
                if (aNativeText == aPortText)
                {
                    Count("write", aCase, "identical");
                }
                else
                {
                    string[] aN = aNativeText.Split('\n');
                    string[] aP = aPortText.Split('\n');
                    int aLine = 0;
                    while (aLine < Math.Min(aN.Length, aP.Length) && aN[aLine] == aP[aLine])
                    {
                        aLine++;
                    }
                    Count("write", aCase, $"differs: line {aLine + 1}: native '{(aLine < aN.Length ? aN[aLine] : "")}' port '{(aLine < aP.Length ? aP[aLine] : "")}'");
                }

                // round trips: the port's file read natively and by the port; native's file read by the port and natively
                Count("port->native", aCase, CompareReads(NativeRead(aPort.File), PortRead(aPort.File)));
                Count("native->port", aCase, CompareReads(NativeRead(aNative.File), PortRead(aNative.File)));
            }
        }
        theOutput.WriteLine(string.Join("\n", aCounts.Select(c => $"{c.Key}: {c.Value}")));
        foreach (string aLine in aDifferent)
        {
            theOutput.WriteLine(aLine);
        }
        Assert.DoesNotContain(aDifferent, d => !KnownWriteDifferences.Contains(d.Split(":")[0]));
    }

    // Cases whose writing or round trip differs from native's, each attributed (decision 1272)
    private static readonly HashSet<string> KnownWriteDifferences = new(StringComparer.Ordinal)
    {
    };
}
