// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using System.Text;

namespace OcctNet.DataExchange.Tests.STEPCAFControl;

/// <summary>
/// STEPCAFControl_Reader's external references read safely (unit 8Z): OCCT's disk reads for the file API, none for the
/// stream API without a resolver, a resolver confined to a folder, the limits, the budget, cycles and cancellation; and
/// the writer's stream form for the multifile mode.
/// </summary>
public sealed class STEPCAFControlExternTests : IDisposable
{
    private readonly string myDir;

    public STEPCAFControlExternTests()
    {
        myDir = Path.Combine(Path.GetTempPath(), "occtnet-p8z-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(myDir);
    }

    public void Dispose()
    {
        try
        {
            Directory.Delete(myDir, true);
        }
        catch (IOException)
        {
        }
    }

    private static TopoDS_Shape Box(double theSize) => new BRepPrimAPI_MakeBox(theSize, theSize, theSize).Shape();

    private static XdeDoc_Document Assembly()
    {
        var aDoc = new XdeDoc_Document();
        XdeDoc_Label anAsm = aDoc.NewShape();
        aDoc.SetName(anAsm, "asm");
        XdeDoc_Label aPartA = aDoc.AddShape(Box(10.0), false);
        aDoc.SetName(aPartA, "partA");
        XdeDoc_Label aPartB = aDoc.AddShape(Box(20.0), false);
        aDoc.SetName(aPartB, "partB");
        aDoc.AddComponent(anAsm, aPartA, new Placement());
        aDoc.AddComponent(anAsm, aPartB, new Placement());
        return aDoc;
    }

    // The three-file assembly (main.stp, partA.stp, partB.stp) written by the file API into theDir
    private static string WriteAssembly(string theDir)
    {
        Directory.CreateDirectory(theDir);
        var aWriter = new STEPCAFControl_Writer();
        Assert.True(aWriter.Transfer(Assembly(), STEPControl_StepModelType.STEPControl_AsIs, ""));
        string aMain = Path.Combine(theDir, "main.stp");
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aWriter.Write(aMain));
        Assert.True(File.Exists(Path.Combine(theDir, "partA.stp")));
        Assert.True(File.Exists(Path.Combine(theDir, "partB.stp")));
        return aMain;
    }

    private static int NbResolved(STEPCAFControl_Reader theReader) =>
        theReader.ExternFiles().Count(f => f.Value.GetLoadStatus() == IFSelect_ReturnStatus.IFSelect_RetDone && !f.Value.GetLabel().IsNull());

    private static int NbExternLabels(XdeDoc_Document theDoc)
    {
        int aNb = 0;
        foreach (XdeDoc_Label aLabel in theDoc.Descendants(theDoc.ShapesLabel()))
        {
            if (theDoc.IsExternRef(aLabel))
            {
                aNb++;
            }
        }
        return aNb;
    }

    // The warnings and fails of the main file's transfer and of the external files'
    private static List<string> Messages(STEPCAFControl_Reader theReader)
    {
        var aList = new List<string>();
        var aSessions = new List<XSControl_WorkSession> { theReader.Reader().WS()! };
        aSessions.AddRange(theReader.ExternFiles().Select(f => f.Value.GetWS()).OfType<XSControl_WorkSession>());
        foreach (XSControl_WorkSession aWS in aSessions)
        {
            Interface_CheckIterator anIter = aWS.TransferReader()!.TransientProcess()!.CheckList(false);
            for (anIter.Start(); anIter.More(); anIter.Next())
            {
                Interface_Check aCheck = anIter.Value()!;
                for (int i = 1; i <= aCheck.NbWarnings(); i++)
                {
                    aList.Add(aCheck.CWarning(i));
                }
                for (int i = 1; i <= aCheck.NbFails(); i++)
                {
                    aList.Add(aCheck.CFail(i));
                }
            }
        }
        return aList;
    }

    private static (STEPCAFControl_Reader Reader, XdeDoc_Document Doc) ReadStream(string theName, byte[] theBytes,
        Action<STEPCAFControl_Reader>? theSetup = null, ProgressRange theProgress = default, StepFile_ReadLimits? theLimits = null)
    {
        var aReader = new STEPCAFControl_Reader();
        theSetup?.Invoke(aReader);
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone,
                     aReader.ReadStream(theName, new MemoryStream(theBytes), default, theLimits!));
        var aDoc = new XdeDoc_Document();
        aReader.Transfer(aDoc, theProgress);
        return (aReader, aDoc);
    }

    // ---------------------------------------------------------------------------------------------------- defaults

    [Fact]
    public void FileApiReadsTheExternalFilesFromTheDiskAsOcct()
    {
        string aMain = WriteAssembly(myDir);
        var aReader = new STEPCAFControl_Reader();
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aReader.ReadFile(aMain));
        var aDoc = new XdeDoc_Document();
        Assert.True(aReader.Transfer(aDoc));
        Assert.Equal(2, NbResolved(aReader));
        Assert.Equal(2, NbExternLabels(aDoc));
        Assert.True(aReader.ExternFile("partA.stp", out STEPCAFControl_ExternFile? anEF));
        Assert.True(anEF!.GetTransferStatus());
    }

    // The issue build123d.NET found: ReadStream under the file's name read its siblings from the disk
    [Fact]
    public void StreamApiReadsNoExternalFileWithoutAResolver()
    {
        string aMain = WriteAssembly(myDir);
        (STEPCAFControl_Reader aReader, XdeDoc_Document aDoc) = ReadStream(aMain, File.ReadAllBytes(aMain));
        Assert.Equal(0, NbResolved(aReader));
        Assert.Equal(2, aReader.ExternFiles().Count());
        Assert.All(aReader.ExternFiles(), f => Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetError, f.Value.GetLoadStatus()));
        // unresolved as OCCT leaves a missing file: the names recorded, OCCT's fail
        Assert.Equal(2, NbExternLabels(aDoc));
        Assert.Contains(Messages(aReader), m => m.StartsWith("Can not read external file", StringComparison.Ordinal));
    }

    [Fact]
    public void DisabledReadsLeaveTheReferencesUnresolvedAsAMissingFile()
    {
        string aMain = WriteAssembly(myDir);
        var aReader = new STEPCAFControl_Reader();
        aReader.SetExternFilesMode(false);
        Assert.False(aReader.GetExternFilesMode());
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aReader.ReadFile(aMain));
        var aDoc = new XdeDoc_Document();
        Assert.True(aReader.Transfer(aDoc));
        Assert.Equal(0, NbResolved(aReader));

        // the same document as OCCT's with the external files missing
        string anOther = Path.Combine(myDir, "alone");
        Directory.CreateDirectory(anOther);
        File.Copy(aMain, Path.Combine(anOther, "main.stp"));
        var aMissing = new STEPCAFControl_Reader();
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aMissing.ReadFile(Path.Combine(anOther, "main.stp")));
        var aMissingDoc = new XdeDoc_Document();
        Assert.True(aMissing.Transfer(aMissingDoc));
        Assert.Equal(XdeDocDump.Dump(aMissingDoc), XdeDocDump.Dump(aDoc));
        Assert.Equal(Messages(aMissing), Messages(aReader));
    }

    // ---------------------------------------------------------------------------------------------------- resolvers

    [Fact]
    public void StreamApiReadsThroughAResolver()
    {
        string aMain = WriteAssembly(myDir);
        var aCalls = new List<(string, string)>();
        STEPCAFControl_ExternFileResolver aFolder = STEPCAFControl_Reader.FolderResolver(myDir);
        (STEPCAFControl_Reader aReader, XdeDoc_Document aDoc) = ReadStream("main.stp", File.ReadAllBytes(aMain), r =>
            r.SetExternFileResolver((n, f) =>
            {
                aCalls.Add((n, f));
                return aFolder(n, f);
            }));
        Assert.Equal(2, NbResolved(aReader));
        Assert.Contains(("partA.stp", "main.stp"), aCalls);
        Assert.Contains(("partB.stp", "main.stp"), aCalls);

        // the same document as the file API's
        var aFileReader = new STEPCAFControl_Reader();
        aFileReader.ReadFile(aMain);
        var aFileDoc = new XdeDoc_Document();
        Assert.True(aFileReader.Transfer(aFileDoc));
        Assert.Equal(XdeDocDump.Dump(aFileDoc), XdeDocDump.Dump(aDoc));
    }

    [Fact]
    public void ResolverFromMemory()
    {
        string aMain = WriteAssembly(myDir);
        var aFiles = new Dictionary<string, byte[]>
        {
            ["partA.stp"] = File.ReadAllBytes(Path.Combine(myDir, "partA.stp")),
            ["partB.stp"] = File.ReadAllBytes(Path.Combine(myDir, "partB.stp")),
        };
        (STEPCAFControl_Reader aReader, _) = ReadStream("in-memory/main.stp", File.ReadAllBytes(aMain), r =>
            r.SetExternFileResolver((n, f) => aFiles.TryGetValue(n, out byte[]? b) ? new MemoryStream(b) : null));
        Assert.Equal(2, NbResolved(aReader));
    }

    [Fact]
    public void FolderResolverStaysInsideItsFolder()
    {
        string aRoot = Path.Combine(myDir, "root");
        Directory.CreateDirectory(Path.Combine(aRoot, "sub"));
        File.WriteAllText(Path.Combine(aRoot, "a.stp"), "a");
        File.WriteAllText(Path.Combine(aRoot, "sub", "b.stp"), "b");
        File.WriteAllText(Path.Combine(myDir, "outside.stp"), "o");
        STEPCAFControl_ExternFileResolver aResolver = STEPCAFControl_Reader.FolderResolver(aRoot);

        string? Read(string theName, string theReferencing)
        {
            using Stream? aStream = aResolver(theName, theReferencing);
            return aStream is null ? null : new StreamReader(aStream).ReadToEnd();
        }

        Assert.Equal("a", Read("a.stp", "main.stp"));
        Assert.Equal("b", Read("sub/b.stp", "main.stp"));
        Assert.Equal("b", Read("b.stp", "sub/main.stp")); // beside the referencing file
        Assert.Equal("a", Read("../a.stp", "sub/main.stp")); // up, still inside
        Assert.Equal("a", Read("sub/../a.stp", "main.stp"));
        Assert.Equal("a", Read("a.stp", Path.Combine(myDir, "elsewhere", "main.stp"))); // an outside referencing name: the root
        Assert.Null(Read("../outside.stp", "main.stp"));
        Assert.Null(Read("../../outside.stp", "sub/main.stp"));
        Assert.Null(Read("sub/../../outside.stp", "main.stp"));
        Assert.Null(Read("../outside.stp", "../main.stp")); // a referencing name above the root: the root
        Assert.Null(Read(Path.Combine(myDir, "outside.stp"), "main.stp")); // absolute
        Assert.Null(Read(Path.Combine(aRoot, "a.stp"), "main.stp")); // absolute, even inside
        Assert.Null(Read("\\\\server\\share\\a.stp", "main.stp")); // UNC
        Assert.Null(Read("//server/share/a.stp", "main.stp"));
        Assert.Null(Read("\\\\?\\" + Path.Combine(aRoot, "a.stp"), "main.stp")); // device path
        Assert.Null(Read("/a.stp", "main.stp"));
        Assert.Null(Read("C:a.stp", "main.stp")); // drive-relative
        Assert.Null(Read("a.stp:stream", "main.stp")); // alternate data stream
        Assert.Null(Read("missing.stp", "main.stp"));
        Assert.Null(Read("sub", "main.stp")); // a folder
        Assert.Null(Read("", "main.stp"));
        Assert.Null(Read("a.stp\0x", "main.stp"));
    }

    // A file naming an absolute path and a path above its folder: OCCT reads both, a folder resolver neither
    [Fact]
    public void HostileReferencesAreRefusedByAFolderResolver()
    {
        string aMain = WriteAssembly(Path.Combine(myDir, "src"));
        string aSecret = Path.Combine(myDir, "secret");
        Directory.CreateDirectory(aSecret);
        File.Copy(Path.Combine(myDir, "src", "partA.stp"), Path.Combine(aSecret, "a.stp"));
        File.Copy(Path.Combine(myDir, "src", "partB.stp"), Path.Combine(aSecret, "b.stp"));
        string aText = File.ReadAllText(aMain)
            .Replace("'partA.stp'", "'" + Path.Combine(aSecret, "a.stp") + "'", StringComparison.Ordinal)
            .Replace("'partB.stp'", "'../secret/b.stp'", StringComparison.Ordinal);
        string aBox = Path.Combine(myDir, "box");
        Directory.CreateDirectory(aBox);
        string aHostile = Path.Combine(aBox, "main.stp");
        File.WriteAllText(aHostile, aText);

        // OCCT's way (the file API) reads them both
        var anOcct = new STEPCAFControl_Reader();
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, anOcct.ReadFile(aHostile));
        Assert.True(anOcct.Transfer(new XdeDoc_Document()));
        Assert.Equal(2, NbResolved(anOcct));

        // confined to the box, neither
        var aConfined = new STEPCAFControl_Reader();
        aConfined.SetExternFileResolver(STEPCAFControl_Reader.FolderResolver(aBox));
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aConfined.ReadFile(aHostile));
        Assert.True(aConfined.Transfer(new XdeDoc_Document()));
        Assert.Equal(0, NbResolved(aConfined));

        // nor through the stream API
        (STEPCAFControl_Reader aStream, _) = ReadStream(aHostile, Encoding.UTF8.GetBytes(aText),
                                                        r => r.SetExternFileResolver(STEPCAFControl_Reader.FolderResolver(aBox)));
        Assert.Equal(0, NbResolved(aStream));
    }

    // ---------------------------------------------------------------------------------------------------- limits

    [Fact]
    public void TheBudgetCoversAllTheExternalFiles()
    {
        string aMain = WriteAssembly(myDir);
        long aSizeA = new FileInfo(Path.Combine(myDir, "partA.stp")).Length;

        var aOne = new STEPCAFControl_Reader();
        aOne.SetExternFileLimits(new STEPCAFControl_ExternFileLimits { MaxFiles = 1 });
        aOne.ReadFile(aMain);
        Assert.True(aOne.Transfer(new XdeDoc_Document()));
        Assert.Equal(1, NbResolved(aOne));
        Assert.Contains(Messages(aOne), m => m.Contains("more than 1 files", StringComparison.Ordinal));

        // the bytes: the first file fits, nothing is left for the second
        var aBytes = new STEPCAFControl_Reader();
        aBytes.SetExternFileLimits(new STEPCAFControl_ExternFileLimits { MaxTotalSize = aSizeA });
        aBytes.ReadFile(aMain);
        Assert.True(aBytes.Transfer(new XdeDoc_Document()));
        Assert.Equal(1, NbResolved(aBytes));
        Assert.Contains(Messages(aBytes), m => m.Contains("bytes are read", StringComparison.Ordinal));

        // a budget smaller than one file: the parse stops at it
        var aSmall = new STEPCAFControl_Reader();
        aSmall.SetExternFileLimits(new STEPCAFControl_ExternFileLimits { MaxTotalSize = aSizeA / 2 });
        aSmall.ReadFile(aMain);
        aSmall.Transfer(new XdeDoc_Document());
        Assert.Equal(0, NbResolved(aSmall));
        Assert.Contains(aSmall.ExternFiles(), f => f.Value.GetLoadStatus() == IFSelect_ReturnStatus.IFSelect_RetFail);
    }

    // The caller's StepFile_ReadLimits apply to each external file: an endless one is read no further than them
    [Fact]
    public void TheCallersLimitsApplyToTheExternalFiles()
    {
        string aMain = WriteAssembly(myDir);
        byte[] aMainBytes = File.ReadAllBytes(aMain);
        var aStreams = new List<EndlessStream>();
        var aLimits = new StepFile_ReadLimits { MaxFileSize = aMainBytes.Length + 1000 };
        (STEPCAFControl_Reader aReader, _) = ReadStream("main.stp", aMainBytes, r => r.SetExternFileResolver((n, f) =>
        {
            var aStream = new EndlessStream();
            aStreams.Add(aStream);
            return aStream;
        }), default, aLimits);
        Assert.Equal(0, NbResolved(aReader));
        Assert.Contains(aReader.ExternFiles(), f => f.Value.GetLoadStatus() == IFSelect_ReturnStatus.IFSelect_RetFail);
        Assert.All(aStreams, s => Assert.True(s.BytesRead <= aLimits.MaxFileSize + 1, s.BytesRead.ToString()));
        Assert.All(aStreams, s => Assert.True(s.IsDisposed));
    }

    // ---------------------------------------------------------------------------------------------------- cycles

    // main.stp refers to partA.stp and partB.stp; partA.stp refers to itself and back to main.stp. OCCT recurses
    // without end (a stack overflow)
    private string WriteCycle()
    {
        string aMain = WriteAssembly(myDir);
        string aText = File.ReadAllText(aMain);
        File.WriteAllText(Path.Combine(myDir, "partA.stp"), aText.Replace("'partB.stp'", "'main.stp'", StringComparison.Ordinal));
        return aMain;
    }

    [Fact]
    public void ACycleOnTheDiskStops()
    {
        string aMain = WriteCycle();
        var aReader = new STEPCAFControl_Reader();
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aReader.ReadFile(aMain));
        Assert.True(aReader.Transfer(new XdeDoc_Document()));
        Assert.Equal(2, NbResolved(aReader)); // partA (its own references unresolved), partB
        List<string> aMessages = Messages(aReader);
        Assert.Contains(aMessages, m => m.StartsWith("External reference file is being read already", StringComparison.Ordinal));
        Assert.Contains(aMessages, m => m == "External reference file is the same main file");
    }

    [Fact]
    public void ACycleThroughAResolverStops()
    {
        string aMain = WriteCycle();
        (STEPCAFControl_Reader aReader, _) = ReadStream("main.stp", File.ReadAllBytes(aMain),
                                                        r => r.SetExternFileResolver(STEPCAFControl_Reader.FolderResolver(myDir)));
        Assert.Equal(2, NbResolved(aReader));
        Assert.Contains(Messages(aReader), m => m.StartsWith("External reference file is being read already", StringComparison.Ordinal));
    }

    [Fact]
    public void DepthIsLimited()
    {
        // each file refers to the next by another name: a.stp -> d1/a.stp -> d1/d1/a.stp ...
        string aMain = WriteAssembly(myDir);
        string aText = File.ReadAllText(aMain).Replace("'partA.stp'", "'d1/main.stp'", StringComparison.Ordinal);
        var aStreams = new List<TrackedStream>();
        (STEPCAFControl_Reader aReader, _) = ReadStream("main.stp", Encoding.UTF8.GetBytes(aText), r =>
        {
            r.SetExternFileLimits(new STEPCAFControl_ExternFileLimits { MaxDepth = 3 });
            r.SetExternFileResolver((n, f) =>
            {
                if (n != "d1/main.stp")
                {
                    return null;
                }
                var aStream = new TrackedStream(Encoding.UTF8.GetBytes(aText));
                aStreams.Add(aStream);
                return aStream;
            });
        });
        // d1/main.stp, d1/d1/main.stp and d1/d1/d1/main.stp read, the next refused (its warning is in the last one's
        // transfer, whose session the reader's map doesn't keep: the three share the name the reference gives)
        Assert.Equal(3, aStreams.Count(s => s.WasRead));
        Assert.NotEmpty(aReader.ExternFiles());
    }

    private sealed class TrackedStream(byte[] theBytes) : MemoryStream(theBytes)
    {
        public bool WasRead { get; private set; }

        public override int Read(byte[] buffer, int offset, int count)
        {
            WasRead = true;
            return base.Read(buffer, offset, count);
        }

        public override int Read(Span<byte> buffer)
        {
            WasRead = true;
            return base.Read(buffer);
        }
    }

    // ---------------------------------------------------------------------------------------------------- cancellation

    [Fact]
    public void CancellationStopsTheExternalReads()
    {
        string aMain = WriteAssembly(myDir);
        using var aCancel = new CancellationTokenSource();
        var aNames = new HashSet<string>();
        STEPCAFControl_ExternFileResolver aFolder = STEPCAFControl_Reader.FolderResolver(myDir);
        (STEPCAFControl_Reader aReader, _) = ReadStream("main.stp", File.ReadAllBytes(aMain), r => r.SetExternFileResolver((n, f) =>
        {
            aNames.Add(n);
            aCancel.Cancel(); // the user cancels as the first external file is looked for
            return aFolder(n, f);
        }), new ProgressIndicator(aCancel.Token).Start());
        Assert.Equal(0, NbResolved(aReader));
        Assert.Single(aNames);
    }

    // ---------------------------------------------------------------------------------------------------- the writer

    [Fact]
    public void WriterStreamFormRefusesTheMultifileModeWithoutStreams()
    {
        var aWriter = new STEPCAFControl_Writer();
        Assert.True(aWriter.Transfer(Assembly(), STEPControl_StepModelType.STEPControl_AsIs, ""));
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetError, aWriter.WriteStream(new MemoryStream()));
    }

    [Fact]
    public void WriterStreamFormHandsTheExternalFilesToTheCaller()
    {
        string aMain = WriteAssembly(Path.Combine(myDir, "disk"));
        var aWriter = new STEPCAFControl_Writer();
        Assert.True(aWriter.Transfer(Assembly(), STEPControl_StepModelType.STEPControl_AsIs, ""));
        var aFiles = new Dictionary<string, MemoryStream>();
        var aMainStream = new MemoryStream();
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aWriter.WriteStream(aMainStream, n => aFiles[n] = new MemoryStream()));
        Assert.Equal(["partA.stp", "partB.stp"], aFiles.Keys.Order());
        Assert.True(aFiles.Values.All(s => s.ToArray().Length > 0));

        // what the file API writes, but for the header's time stamp
        static string Masked(string theText) =>
            string.Join("\n", theText.Split('\n').Where(l => !l.StartsWith("FILE_NAME", StringComparison.Ordinal) && !l.StartsWith("  '20", StringComparison.Ordinal)));
        Assert.Equal(Masked(File.ReadAllText(Path.Combine(myDir, "disk", "partA.stp"))).Length,
                     Masked(Encoding.UTF8.GetString(aFiles["partA.stp"].ToArray())).Length);

        // all read back from memory
        (STEPCAFControl_Reader aReader, _) = ReadStream("main.stp", aMainStream.ToArray(), r =>
            r.SetExternFileResolver((n, f) => aFiles.TryGetValue(n, out MemoryStream? s) ? new MemoryStream(s.ToArray()) : null));
        Assert.Equal(2, NbResolved(aReader));
    }

    [Fact]
    public void WriterStreamFormLetsTheCallerRefuseAName()
    {
        var aDoc = Assembly();
        var aFree = new NCollection_Sequence<XdeDoc_Label>();
        aDoc.GetFreeShapes(aFree);
        var aComps = new NCollection_Sequence<XdeDoc_Label>();
        aDoc.GetComponents(aFree.Value(1), aComps);
        Assert.True(aDoc.GetReferredShape(aComps.Value(1), out XdeDoc_Label aPart));
        aDoc.SetName(aPart, "../../escape"); // a hostile name read from another file
        var aWriter = new STEPCAFControl_Writer();
        Assert.True(aWriter.Transfer(aDoc, STEPControl_StepModelType.STEPControl_AsIs, ""));
        var aNames = new List<string>();
        IFSelect_ReturnStatus aStatus = aWriter.WriteStream(new MemoryStream(), n =>
        {
            aNames.Add(n);
            return n.Contains("..", StringComparison.Ordinal) ? null : new MemoryStream();
        });
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetError, aStatus);
        Assert.Contains("../../escape.stp", aNames);
        Assert.Contains("partB.stp", aNames);
    }

    private sealed class EndlessStream : Stream
    {
        private static readonly byte[] Pattern = Encoding.ASCII.GetBytes("ISO-10303-21;\nHEADER;\n/* endless */");
        private static readonly byte[] Filler = Encoding.ASCII.GetBytes("#1=CARTESIAN_POINT('',(0.,0.,0.));\n");

        public long BytesRead { get; private set; }
        public bool IsDisposed { get; private set; }

        public override bool CanRead => true;
        public override bool CanSeek => false;
        public override bool CanWrite => false;
        public override long Length => throw new NotSupportedException();
        public override long Position { get => BytesRead; set => throw new NotSupportedException(); }
        public override void Flush() { }
        public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
        public override void SetLength(long value) => throw new NotSupportedException();
        public override void Write(byte[] buffer, int offset, int count) => throw new NotSupportedException();

        public override int Read(byte[] buffer, int offset, int count)
        {
            for (int i = 0; i < count; i++)
            {
                long aPos = BytesRead + i;
                buffer[offset + i] = aPos < Pattern.Length ? Pattern[aPos] : Filler[(aPos - Pattern.Length) % Filler.Length];
            }
            BytesRead += count;
            return count;
        }

        protected override void Dispose(bool disposing)
        {
            IsDisposed = true;
            base.Dispose(disposing);
        }
    }
}
