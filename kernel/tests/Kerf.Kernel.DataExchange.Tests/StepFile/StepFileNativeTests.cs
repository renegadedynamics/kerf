// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// The STEP parser against native OCCT's (tools/steprec, unit 8P): what each leaves in StepFile_ReadData (records,
/// arguments, scopes, errors, the parse's result) on every STEP file of OCCT's test dataset and source data, and on the
/// hostile and mutated inputs of tools/steprec/gen-cases.py. Identical, but for the inputs native crashes on.
/// </summary>
public class StepFileNativeTests(ITestOutputHelper output)
{
    /// <summary>
    /// Inputs native OCCT crashes on: an empty header and a scope ended before any list was closed, where
    /// StepFile_ReadData::FinalOfScope reads through its null mySubArg (unit 8P's OCCT issue). The port takes it for
    /// a scope without an export list.
    /// </summary>
    private static readonly HashSet<string> KnownNativeCrashes = ["struct-nohead/034", "struct-nohead/044"];

    [Fact]
    public void The_corpus_parses_as_native()
    {
        List<StepFileDump.Entry> anEntries = StepFileDump.Recording("StepFile.steprec-corpus.txt.gz");
        Assert.Equal(340, anEntries.Count);
        int aRun = 0;
        long aBytes = 0;
        var aDiffs = new List<string>();
        foreach (StepFileDump.Entry anEntry in anEntries)
        {
            string? aPath = StepFileDump.CorpusPath(anEntry.Name);
            if (aPath is null)
            {
                continue;
            }

            using FileStream aStream = File.OpenRead(aPath);
            Assert.Equal(anEntry.Size, aStream.Length);
            string? aDiff = StepFileDump.Compare(anEntry, StepFileDump.Dump(aStream));
            if (aDiff is not null)
            {
                aDiffs.Add(aDiff);
            }

            aRun++;
            aBytes += anEntry.Size;
        }

        output.WriteLine($"{aRun} of {anEntries.Count} files compared ({aBytes / 1e6:F1} MB), {aDiffs.Count} different");
        Assert.Empty(aDiffs);
    }

    [Fact]
    public void The_hostile_and_mutated_inputs_parse_as_native()
    {
        List<(string Name, byte[] Data)> anInputs = StepFileDump.Pack("StepFile.cases.pack.gz");
        List<StepFileDump.Entry> anEntries = StepFileDump.Recording("StepFile.steprec-cases.txt.gz");
        Assert.Equal(anInputs.Count, anEntries.Count);
        var aDiffs = new List<string>();
        var aResults = new Dictionary<string, int>();
        for (int i = 0; i < anInputs.Count; i++)
        {
            StepFileDump.Entry anEntry = anEntries[i];
            Assert.Equal(anInputs[i].Name, anEntry.Name);
            List<string> aDump = StepFileDump.Dump(new MemoryStream(anInputs[i].Data));
            aResults[aDump[0]] = aResults.GetValueOrDefault(aDump[0]) + 1;
            if (KnownNativeCrashes.Contains(anEntry.Name))
            {
                Assert.Equal("RESULT CRASH", anEntry.Lines[0]);
                Assert.Equal("RESULT 0", aDump[0]);
                continue;
            }

            string? aDiff = StepFileDump.Compare(anEntry, aDump);
            if (aDiff is not null)
            {
                aDiffs.Add(aDiff);
            }
        }

        output.WriteLine($"{anInputs.Count} inputs: " + string.Join(", ", aResults.OrderBy(p => p.Key).Select(p => $"{p.Key}: {p.Value}")));
        Assert.Equal(1329, anInputs.Count);
        Assert.Empty(aDiffs);
    }

    [Fact]
    public void The_corpus_parses_as_native_through_a_stream_reading_a_few_bytes_at_a_time()
    {
        // The scanner's refills at every block boundary: the dataset's smaller files through a stream that returns at
        // most 7 bytes per read and can't seek
        List<StepFileDump.Entry> anEntries = StepFileDump.Recording("StepFile.steprec-corpus.txt.gz");
        foreach (StepFileDump.Entry anEntry in anEntries.Where(e => e.Size < 200_000))
        {
            string? aPath = StepFileDump.CorpusPath(anEntry.Name);
            if (aPath is null)
            {
                continue;
            }

            using var aStream = new TrickleStream(File.ReadAllBytes(aPath), 7);
            Assert.Null(StepFileDump.Compare(anEntry, StepFileDump.Dump(aStream)));
        }
    }

    [Fact]
    public void The_long_inputs_parse_as_native_through_a_stream_reading_a_few_bytes_at_a_time()
    {
        List<(string Name, byte[] Data)> anInputs = StepFileDump.Pack("StepFile.cases.pack.gz");
        List<StepFileDump.Entry> anEntries = StepFileDump.Recording("StepFile.steprec-cases.txt.gz");
        for (int i = 0; i < anInputs.Count; i++)
        {
            if (anInputs[i].Name.StartsWith("long/", StringComparison.Ordinal))
            {
                using var aStream = new TrickleStream(anInputs[i].Data, 997);
                Assert.Null(StepFileDump.Compare(anEntries[i], StepFileDump.Dump(aStream)));
            }
        }
    }

    /// <summary>A forward-only stream returning at most a few bytes per read.</summary>
    internal sealed class TrickleStream(byte[] theData, int theChunk) : Stream
    {
        private int myPos;

        public override int Read(byte[] buffer, int offset, int count)
        {
            int aCount = Math.Min(Math.Min(count, theChunk), theData.Length - myPos);
            Array.Copy(theData, myPos, buffer, offset, aCount);
            myPos += aCount;
            return aCount;
        }

        public override bool CanRead => true;
        public override bool CanSeek => false;
        public override bool CanWrite => false;
        public override long Length => throw new NotSupportedException();
        public override long Position { get => myPos; set => throw new NotSupportedException(); }
        public override void Flush() { }
        public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
        public override void SetLength(long value) => throw new NotSupportedException();
        public override void Write(byte[] buffer, int offset, int count) => throw new NotSupportedException();
    }
}
