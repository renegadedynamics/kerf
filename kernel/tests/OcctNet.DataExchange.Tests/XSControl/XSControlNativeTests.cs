// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.IO.Compression;
using System.Text;
using Xunit.Abstractions;

namespace OcctNet.DataExchange.Tests.XSControl;

/// <summary>
/// The work session on the corpus's STEP files (up to 3 MB) compared with native OCCT (tools/xscontrolrec, unit 8A3):
/// each file read as STEPControl_Reader reads it, in "lite" and "full" modes; the roots, the shareds of the roots and
/// the sharings of entity 1 (selections evaluated through GiveList), the type, validity and category signatures
/// (counters), the model's check list counted per message, a label looked up, a selection by signature.
/// </summary>
[Collection(ProcessWideMessages.Name)]
public class XSControlNativeTests(ITestOutputHelper theOutput)
{
    private static List<(string Name, List<string> Lines)> Recording()
    {
        using Stream aRes = typeof(XSControlNativeTests).Assembly.GetManifestResourceStream("XSControl.xscontrolrec.txt.gz")!;
        using var aReader = new StreamReader(new GZipStream(aRes, CompressionMode.Decompress), Encoding.UTF8);
        var aList = new List<(string, List<string>)>();
        for (string? aLine = aReader.ReadLine(); aLine is not null; aLine = aReader.ReadLine())
        {
            if (aLine.StartsWith("F ", StringComparison.Ordinal))
            {
                aList.Add((aLine[2..], new List<string>()));
            }
            else
            {
                aList[^1].Item2.Add(aLine);
            }
        }
        return aList;
    }

    [Fact]
    public void CorpusAsNative()
    {
        List<(string Name, List<string> Lines)> aRec = Recording();
        int aFiles = 0, aSame = 0, aLines = 0;
        var aDiffs = new List<string>();
        foreach ((string aName, List<string> aNative) in aRec)
        {
            string? aPath = StepFileDump.CorpusPath(aName);
            if (aPath is null)
            {
                continue;
            }
            aFiles++;
            List<string> aPort = XSControlDump.Dump(aPath);
            aLines += aNative.Count;
            int aFirst = Enumerable.Range(0, Math.Max(aPort.Count, aNative.Count))
                                   .FirstOrDefault(i => i >= aPort.Count || i >= aNative.Count || aPort[i] != aNative[i], -1);
            if (aFirst < 0)
            {
                aSame++;
                continue;
            }
            aDiffs.Add($"{aName}: line {aFirst + 1}: native '{(aFirst < aNative.Count ? aNative[aFirst] : "<end>")}', " +
                       $"port '{(aFirst < aPort.Count ? aPort[aFirst] : "<end>")}'");
        }
        if (aFiles == 0)
        {
            theOutput.WriteLine("The dataset is absent: skipped");
            return;
        }
        theOutput.WriteLine($"{aFiles} files ({aLines} native lines): {aSame} identical");
        foreach (string aDiff in aDiffs.Take(40))
        {
            theOutput.WriteLine(aDiff);
        }
        Assert.Empty(aDiffs);
    }
}
