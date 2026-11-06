// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Exchange.Stl.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Buffers.Binary;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using Xunit.Abstractions;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// RWStl and StlAPI against native OCCT's recordings (tools/stlrec, unit 8H1). Writing: data/occ's shapes meshed with
/// the same parameters (the port's BRepMesh is native's) and written as ASCII and binary STL by StlAPI_Writer, and a
/// hand-made triangulation written by RWStl, compared byte for byte (binary STL's 80-byte header included), then read
/// back. Reading: the STL files of data/stl, of the test dataset and of the tests' own inputs, and data derived from
/// some of them, read every way (RWStl::ReadFile at three merge angles and per domain, an RWStl_Reader's calls, the
/// stream readers, StlAPI_Reader): triangulations compared node for node (bits) and triangle for triangle, and shapes
/// as BRep text.
/// </summary>
/// <remarks>
/// One difference is attributed: native reads ASCII data as binary (<c>ReadBinaryStream</c> on text) with the facet
/// count taken from the text, and throws Standard_OutOfMemory sizing its node map for it; the port sizes the map by the
/// data (decision 1170) and reads nothing (NULL).
/// </remarks>
public class RWStlNativeTests(ITestOutputHelper output)
{
    internal static string Metadata(string theKey) =>
        typeof(RWStlNativeTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    internal static string OcctSourceDir => Metadata("OcctSourceDir");

    internal static string OcctTestDataDir => Metadata("OcctTestDataDir");

    internal static string InputsDir => Path.Combine(AppContext.BaseDirectory, "RWStl", "Inputs");

    private static List<string> Recording()
    {
        using Stream aRes = typeof(RWStlNativeTests).Assembly.GetManifestResourceStream("RWStl.stlrec.txt.gz")
                            ?? throw new InvalidOperationException("RWStl.stlrec.txt.gz");
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

    // The recording's blocks: a head line (W, M, R, D) and the lines up to the next head
    private static List<(string Head, List<string> Lines)> Blocks(List<string> theLines)
    {
        var aBlocks = new List<(string, List<string>)>();
        foreach (string aLine in theLines)
        {
            if (aLine == "M" || aLine.StartsWith("W ", StringComparison.Ordinal) || aLine.StartsWith("R ", StringComparison.Ordinal)
                || aLine.StartsWith("D ", StringComparison.Ordinal))
            {
                aBlocks.Add((aLine, []));
            }
            else
            {
                aBlocks[^1].Item2.Add(aLine);
            }
        }
        return aBlocks;
    }

    internal static string Sha256(ReadOnlySpan<byte> theData) => Convert.ToHexStringLower(SHA256.HashData(theData));

    private static string Bytes(byte[] theData) => theData.Length.ToString(CultureInfo.InvariantCulture) + " " + Sha256(theData);

    // Binary STL as native wrote it, for its frozen hashes: Kerf's 80-byte header names Kerf (unit 11L), native's named OCCT
    private static string BinaryBytes(byte[] theData)
    {
        if (theData.Length < 80) // nothing was written
        {
            return Bytes(theData);
        }
        byte[] aNative = (byte[])theData.Clone();
        aNative.AsSpan(0, 80).Clear();
        Encoding.ASCII.GetBytes("STL Exported by Open CASCADE Technology [dev.opencascade.org]", aNative);
        return Bytes(aNative);
    }

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture);

    /// <summary>"nodes triangles sha" of the triangulation's dump (stlrec's), or NULL.</summary>
    internal static string Tri(Triangulation? theTri)
    {
        if (theTri is null)
        {
            return "NULL";
        }
        var aDump = new StringBuilder();
        for (int i = 1; i <= theTri.NodeCount; ++i)
        {
            Point3D aP = theTri.Node(i);
            aDump.Append(Hex(aP.X)).Append(' ').Append(Hex(aP.Y)).Append(' ').Append(Hex(aP.Z)).Append('\n');
        }
        for (int i = 1; i <= theTri.TriangleCount; ++i)
        {
            theTri.Triangle(i).Get(out int n1, out int n2, out int n3);
            aDump.Append(CultureInfo.InvariantCulture, $"{n1} {n2} {n3}\n");
        }
        return $"{theTri.NodeCount} {theTri.TriangleCount} {Sha256(Encoding.ASCII.GetBytes(aDump.ToString()))}";
    }

    private static string ShapeText(in Shape theShape)
    {
        var aWriter = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    // Runs the call: "<head> <its line>", or "THROW class <type>" if it throws
    private static string Line(string theHead, Func<string> theCall)
    {
        try
        {
            return theHead + " " + theCall();
        }
        catch (Exception e)
        {
            return "THROW class " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(e).GetType());
        }
    }

    private sealed class CountingReader : global::Kerf.Engine.Exchange.Stl.Reader
    {
        public int myNbNodes, myNbTris, myNbSolids;

        public override int AddNode(in Xyz thePnt) => ++myNbNodes;

        public override void AddTriangle(int theN1, int theN2, int theN3) => ++myNbTris;

        public override void AddSolid() => ++myNbSolids;
    }

    /// <summary>The data of a recipe of stlrec (main.cpp's Derive).</summary>
    internal static byte[] Derive(string theRecipe, byte[] theData, string theArg)
    {
        switch (theRecipe)
        {
            case "cat2":
                return [.. theData, .. theData];
            case "ws":
                return [.. theData, .. "\n \t\r\n"u8, .. theData];
            case "trunc":
                return theData[..Math.Min(theData.Length, int.Parse(theArg, CultureInfo.InvariantCulture))];
            case "solidhdr":
            {
                byte[] aData = [.. theData];
                "solid "u8.CopyTo(aData);
                return aData;
            }
            case "lie":
            {
                byte[] aData = [.. theData];
                int aCount = BinaryPrimitives.ReadInt32LittleEndian(aData.AsSpan(80));
                BinaryPrimitives.WriteInt32LittleEndian(aData.AsSpan(80), aCount + int.Parse(theArg, CultureInfo.InvariantCulture));
                return aData;
            }
            case "crlf":
            {
                var aData = new List<byte>();
                foreach (byte c in theData)
                {
                    if (c == '\n')
                    {
                        aData.Add((byte)'\r');
                    }
                    aData.Add(c);
                }
                return [.. aData];
            }
            default:
                throw new ArgumentException(theRecipe);
        }
    }

    /// <summary>An input file by name: the tests' inputs, the dataset's others/, data/stl, data/occ; null if absent.</summary>
    internal static string? Find(string theName)
    {
        foreach (string aDir in new[]
                 {
                     InputsDir, Path.Combine(OcctTestDataDir, "others"), Path.Combine(OcctSourceDir, "data", "stl"),
                     Path.Combine(OcctSourceDir, "data", "occ"),
                 })
        {
            string aPath = Path.Combine(aDir, theName);
            if (File.Exists(aPath))
            {
                return aPath;
            }
        }
        return null;
    }

    // stlrec's ReadCase
    private static List<string> ReadCase(string thePath, byte[] theData)
    {
        var aLines = new List<string>();
        double[] anAngles = [Math.PI / 2.0, Math.PI / 4.0, 0.0];
        string[] aTags = ["90", "45", "0"];
        for (int i = 0; i < 3; ++i)
        {
            aLines.Add(Line("FILE " + aTags[i], () => Tri(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(thePath, anAngles[i]))));
        }
        aLines.AddRange(Line("MULTI", () =>
        {
            var aList = new NCollection_Sequence<Triangulation>();
            global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(thePath, Math.PI / 2.0, aList);
            var aText = new StringBuilder(aList.Length().ToString(CultureInfo.InvariantCulture));
            for (int i = 1; i <= aList.Length(); ++i)
            {
                aText.Append('\n').Append(Tri(aList.Value(i)));
            }
            return aText.ToString();
        }).Split('\n'));
        aLines.Add(Line("READER", () =>
        {
            var aReader = new CountingReader();
            bool isOk = aReader.Read(thePath);
            return $"{(isOk ? 1 : 0)} {aReader.myNbNodes} {aReader.myNbTris} {aReader.myNbSolids}";
        }));
        aLines.Add(Line("STREAM", () => Tri(global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(new MemoryStream(theData)))));
        aLines.Add(Line("ASCIIS", () => Tri(global::Kerf.Engine.Exchange.Stl.RWStl.ReadAsciiStream(new MemoryStream(theData)))));
        aLines.Add(Line("BINS", () => Tri(global::Kerf.Engine.Exchange.Stl.RWStl.ReadBinaryStream(new MemoryStream(theData)))));
        if (theData.Length <= 300000)
        {
            aLines.Add(Line("SHAPE", () =>
            {
                Shape aShape = new();
                return new StlApiReader().Read(ref aShape, thePath) ? Bytes(Encoding.ASCII.GetBytes(ShapeText(aShape))) : "FAIL";
            }));
            aLines.Add(Line("SHAPES", () =>
            {
                Shape aShape = new();
                return new StlApiReader().Read(ref aShape, new MemoryStream(theData)) ? Bytes(Encoding.ASCII.GetBytes(ShapeText(aShape))) : "FAIL";
            }));
        }
        return aLines;
    }

    /// <summary>stlrec's hand-made triangulation.</summary>
    internal static Triangulation Mesh()
    {
        var aMesh = new Triangulation(7, 5, false);
        aMesh.SetNode(1, new Point3D(0.0, 0.0, 0.0));
        aMesh.SetNode(2, new Point3D(1.0, 0.0, 0.0));
        aMesh.SetNode(3, new Point3D(0.0, 1.0, 0.0));
        aMesh.SetNode(4, new Point3D(-0.0, 1e-310, 1e300));
        aMesh.SetNode(5, new Point3D(1.0 / 3.0, 2.0 / 3.0, -1e-5));
        aMesh.SetNode(6, new Point3D(0.5, 0.5, 0.0));
        aMesh.SetNode(7, new Point3D(123456.789, -9.87654321e-7, 5e-324));
        aMesh.SetTriangle(1, new Triangle1(1, 2, 3));
        aMesh.SetTriangle(2, new Triangle1(1, 2, 2));
        aMesh.SetTriangle(3, new Triangle1(4, 5, 6));
        aMesh.SetTriangle(4, new Triangle1(3, 2, 1));
        aMesh.SetTriangle(5, new Triangle1(5, 6, 7));
        return aMesh;
    }

    // The port's lines for a block of the recording; null if its input is absent
    private static List<string>? PortBlock(string theHead, string theScratch, ref int theDerived)
    {
        string[] aWords = theHead.Split(' ');
        var aLines = new List<string>();
        switch (aWords[0])
        {
            case "W":
            {
                BrepTools.Read(out Shape aShape, Path.Combine(OcctSourceDir, "data", "occ", aWords[1]));
                Shape aMeshed = new IncrementalMesh(aShape, 0.1, true, 0.5, false).Shape();
                byte[] anAscii = [], aBinary = [];
                aLines.Add(Line("ASCII", () =>
                {
                    var aStream = new MemoryStream();
                    new ApiWriter().Write(aMeshed, aStream);
                    anAscii = aStream.ToArray();
                    return Bytes(anAscii);
                }));
                aLines.Add(Line("BINARY", () =>
                {
                    var aStream = new MemoryStream();
                    var aWriter = new ApiWriter();
                    aWriter.SetASCIIMode(false);
                    aWriter.Write(aMeshed, aStream);
                    aBinary = aStream.ToArray();
                    return BinaryBytes(aBinary);
                }));
                aLines.Add(Line("BACKA", () => Tri(global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(new MemoryStream(anAscii)))));
                aLines.Add(Line("BACKB", () => Tri(global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(new MemoryStream(aBinary)))));
                return aLines;
            }
            case "M":
            {
                Triangulation aMesh = Mesh();
                aLines.Add(Line("ASCII", () =>
                {
                    var aStream = new MemoryStream();
                    global::Kerf.Engine.Exchange.Stl.RWStl.WriteAscii(aMesh, aStream);
                    return Bytes(aStream.ToArray());
                }));
                aLines.Add(Line("BINARY", () =>
                {
                    var aStream = new MemoryStream();
                    global::Kerf.Engine.Exchange.Stl.RWStl.WriteBinary(aMesh, aStream);
                    return BinaryBytes(aStream.ToArray());
                }));
                return aLines;
            }
            case "R":
            {
                string? aPath = Find(aWords[1]);
                return aPath is null ? null : ReadCase(aPath, File.ReadAllBytes(aPath));
            }
            default: // D
            {
                string? aPath = Find(aWords[2]);
                if (aPath is null)
                {
                    return null;
                }
                byte[] aData = Derive(aWords[1], File.ReadAllBytes(aPath), aWords.Length > 3 ? aWords[3] : "");
                string aDerivedPath = Path.Combine(theScratch, "derived" + (++theDerived).ToString(CultureInfo.InvariantCulture) + ".stl");
                File.WriteAllBytes(aDerivedPath, aData);
                return ReadCase(aDerivedPath, aData);
            }
        }
    }

    // Plan 3.2's third level (occt-issues.md, RWStl::ReadStream; Kerf decision 1498): native read a stream as ASCII
    // only if it started "solid", so a binary header starting "solid " (and ASCII data starting otherwise) gave NULL; the
    // port recognises them as the file reader does, so its stream read is native's file read of the same data.
    private static bool StreamRecognised(string thePort, string theNative, List<string> theNativeBlock)
    {
        if (theNative == "STREAM NULL" && thePort.StartsWith("STREAM ", StringComparison.Ordinal))
        {
            return theNativeBlock.Contains("FILE 90 " + thePort["STREAM ".Length..]);
        }
        if (theNative == "SHAPES FAIL" && thePort.StartsWith("SHAPES ", StringComparison.Ordinal))
        {
            return theNativeBlock.Contains("SHAPE " + thePort["SHAPES ".Length..]);
        }
        return false;
    }

    // The domains a block lists after MULTI, with the line that follows them
    private static (List<string> Domains, int Next) Multi(List<string> theBlock)
    {
        int i = theBlock.FindIndex(l => l.StartsWith("MULTI ", StringComparison.Ordinal));
        int n = int.Parse(theBlock[i]["MULTI ".Length..], CultureInfo.InvariantCulture);
        return (theBlock.GetRange(i + 1, n), i + 1 + n);
    }

    private static int Triangles(string theTri) => int.Parse(theTri.Split(' ')[1], CultureInfo.InvariantCulture);

    // Plan 3.2's third level (occt-issues.md, RWStl_Reader::Read, multi-domain ASCII; Kerf decision 1499): native lost
    // the domains after the first in files under its 1 KB block, and after ws's blank run. The port reads more: native's
    // domains are its first ones; the whole-file reads (FILE, READER, SHAPE) hold the domains' triangles together; the
    // stream readers (one domain) are native's.
    private static bool MoreDomains(List<string> thePort, List<string> theNative)
    {
        if (!thePort.Exists(l => l.StartsWith("MULTI ", StringComparison.Ordinal)) || !theNative.Exists(l => l.StartsWith("MULTI ", StringComparison.Ordinal)))
        {
            return false;
        }
        (List<string> aPD, int aPNext) = Multi(thePort);
        (List<string> aND, int aNNext) = Multi(theNative);
        if (aPD.Count <= aND.Count || !aPD.GetRange(0, aND.Count).SequenceEqual(aND))
        {
            return false;
        }
        int aTris = aPD.Sum(Triangles);
        foreach (string aLine in thePort.Take(3))
        {
            // FILE 90/45/0: one triangulation of every domain
            if (!aLine.StartsWith("FILE ", StringComparison.Ordinal) || int.Parse(aLine.Split(' ')[3], CultureInfo.InvariantCulture) != aTris)
            {
                return false;
            }
        }
        string[] aReader = thePort[aPNext].Split(' ');
        if (aReader[0] != "READER" || int.Parse(aReader[3], CultureInfo.InvariantCulture) != aTris
            || int.Parse(aReader[4], CultureInfo.InvariantCulture) != aPD.Count)
        {
            return false;
        }
        // the stream readers, as native's (SHAPE reads the whole file: its triangles differ)
        List<string> aPRest = thePort.Skip(aPNext + 1).Where(l => !l.StartsWith("SHAPE ", StringComparison.Ordinal)).ToList();
        List<string> aNRest = theNative.Skip(aNNext + 1).Where(l => !l.StartsWith("SHAPE ", StringComparison.Ordinal)).ToList();
        return aPRest.Count == aNRest.Count
               && aPRest.Zip(aNRest).All(p => p.First == p.Second || (p.Second == "THROW class Standard_OutOfMemory" && p.First == "BINS NULL"));
    }

    [Fact]
    public void The_port_writes_and_reads_STL_as_native_OCCT()
    {
        List<(string Head, List<string> Lines)> aBlocks = Blocks(Recording());
        string aScratch = Directory.CreateTempSubdirectory("p8h1-stl").FullName;
        try
        {
            int aDerived = 0, aCompared = 0, anAbsent = 0, anAttributed = 0, anIntendedStream = 0, anIntendedDomains = 0;
            var aDifferent = new List<string>();
            foreach ((string aHead, List<string> aNative) in aBlocks)
            {
                List<string>? aPort = PortBlock(aHead, aScratch, ref aDerived);
                if (aPort is null)
                {
                    anAbsent++;
                    continue;
                }
                aCompared++;
                if (MoreDomains(aPort, aNative))
                {
                    anIntendedDomains++;
                    continue;
                }
                if (aPort.Count != aNative.Count)
                {
                    aDifferent.Add($"{aHead}: {aPort.Count} lines, native {aNative.Count}");
                    continue;
                }
                for (int i = 0; i < aPort.Count; ++i)
                {
                    if (aPort[i] == aNative[i])
                    {
                        continue;
                    }
                    // decision 1170: native's node map for a facet count read from text runs out of memory
                    if (aNative[i] == "THROW class Standard_OutOfMemory" && aPort[i] == "BINS NULL")
                    {
                        anAttributed++;
                        continue;
                    }
                    // decision 20208: a file with NaN or infinite numbers fails at the direction, not later: a read fails either way
                    if (aHead == "R numbers.stl" && aNative[i] == "THROW class StdFail_NotDone" && aPort[i] == "THROW class Standard_ConstructionError")
                    {
                        anAttributed++;
                        continue;
                    }
                    if (StreamRecognised(aPort[i], aNative[i], aNative))
                    {
                        anIntendedStream++;
                        continue;
                    }
                    aDifferent.Add($"{aHead}: port '{aPort[i]}', native '{aNative[i]}'");
                }
            }
            output.WriteLine($"{aCompared} blocks compared, {anAbsent} absent (no dataset), {anAttributed} attributed lines; "
                             + $"intended differences: {anIntendedStream} lines read from streams, {anIntendedDomains} blocks with more domains");
            foreach (string aLine in aDifferent)
            {
                output.WriteLine(aLine);
            }
            Assert.Empty(aDifferent);
            Assert.Equal(aBlocks.Count, aCompared + anAbsent);
            if (Directory.Exists(OcctTestDataDir))
            {
                Assert.Equal(0, anAbsent);
                // the second of decision 1170's two lines is in ws OMF6391_box.stl, a block with more domains now
                Assert.Equal(2, anAttributed);
                Assert.Equal(8, anIntendedStream);
                Assert.Equal(5, anIntendedDomains);
            }
        }
        finally
        {
            Directory.Delete(aScratch, true);
        }
    }
}
