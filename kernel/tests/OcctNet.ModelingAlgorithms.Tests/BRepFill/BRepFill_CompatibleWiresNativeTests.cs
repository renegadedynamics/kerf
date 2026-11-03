// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The port's BRepFill_CompatibleWires against native OCCT's, recorded by tools/cwrec (unit 5S1). Each recorded case
/// holds its input sections as BRep text, which native OCCT read back before running, so both sides run on the same
/// shapes; the port reads them, freezes them (as a caller's published shapes are) and runs. Compared: the status, the
/// degenerated-section flags, the result wires as the BRep text of a compound (the Free column masked, open question 1),
/// and the generated edges of every input edge (indices and orientations in the result's map of edges). The input's
/// text is also checked unchanged after the run (no write into the caller's shapes). Native OCCT isn't repeatable on
/// looping sections (the first wire repeated last): those cases are reported, and the port checked deterministic.
/// </summary>
public class BRepFill_CompatibleWiresNativeTests(ITestOutputHelper output)
{
    private sealed record Case(string Name, bool WithRotation, double? Percent, bool DefaultPercent, string Input, string Expected);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    private static List<Case> Load()
    {
        using Stream aRes = typeof(BRepFill_CompatibleWiresNativeTests).Assembly.GetManifestResourceStream("BRepFill.cwrec.txt.gz")
                            ?? throw new InvalidOperationException("BRepFill.cwrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        List<string> aLines = [];
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            aLines.Add(aLine);
        }

        List<Case> aCases = [];
        int i = 0;
        while (i < aLines.Count)
        {
            string aName = aLines[i++][5..];
            string[] anOpt = aLines[i++].Split(' ');
            bool aRot = anOpt[1] == "1";
            double? aPct = null;
            bool aDefault = false;
            if (anOpt[2] == "d")
            {
                aDefault = true;
            }
            else if (anOpt[2] != "-")
            {
                aPct = BitConverter.Int64BitsToDouble(long.Parse(anOpt[2], NumberStyles.HexNumber, CultureInfo.InvariantCulture));
            }
            string anIn = ReadBlock(aLines, ref i);
            var anExpected = new StringBuilder();
            while (i < aLines.Count && !aLines[i].StartsWith("CASE ", StringComparison.Ordinal))
            {
                anExpected.Append(aLines[i++]).Append('\n');
            }
            aCases.Add(new Case(aName, aRot, aPct, aDefault, anIn, anExpected.ToString()));
        }
        return aCases;
    }

    private static string ReadBlock(List<string> theLines, ref int theI)
    {
        int aCount = int.Parse(theLines[theI++].Split(' ')[1], CultureInfo.InvariantCulture);
        var aText = new StringBuilder();
        for (int k = 0; k < aCount; k++)
        {
            aText.Append(theLines[theI++]).Append('\n');
        }
        return aText.ToString();
    }

    private static string Text(in TopoDS_Shape theShape)
    {
        using StringWriter aW = new(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BRepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static string Block(string theTag, string theText)
    {
        int aCount = theText.Count(c => c == '\n');
        if (theText.Length > 0 && theText[^1] != '\n')
        {
            aCount++;
            theText += "\n";
        }
        return $"{theTag} {aCount}\n{theText}";
    }

    private static string Orient(TopAbs_Orientation theO) => theO switch
    {
        TopAbs_Orientation.TopAbs_FORWARD => "F",
        TopAbs_Orientation.TopAbs_REVERSED => "R",
        TopAbs_Orientation.TopAbs_INTERNAL => "I",
        _ => "E",
    };

    // The Free flag is the first of the seven flag digits on their own line.
    private static string MaskFree(string theText) => Regex.Replace(theText, "(?m)^[01]([01]{6})$", "x$1");

    /// <summary>Runs one recorded case on the port and writes what cwrec writes after the input block.</summary>
    private static string Run(Case theCase, out string theInputBefore, out string theInputAfter)
    {
        BRepTools.Read(out TopoDS_Shape anIn, new StringReader(theCase.Input));
        anIn.TShape()!.FreezeGraph();
        theInputBefore = Text(anIn);
        NCollection_Sequence<TopoDS_Shape> aSeq = new();
        for (TopoDS_Iterator anIt = new(anIn); anIt.More(); anIt.Next())
        {
            aSeq.Append(anIt.Value());
        }

        var aOut = new StringBuilder();
        try
        {
            BRepFill_CompatibleWires aCW = new(aSeq);
            if (theCase.Percent is double aPct)
            {
                aCW.SetPercent(aPct);
            }
            else if (theCase.DefaultPercent)
            {
                aCW.SetPercent();
            }
            aCW.Perform(theCase.WithRotation);
            aOut.Append($"STATUS {(int)aCW.GetStatus()} {(aCW.IsDone() ? 1 : 0)} {(aCW.IsDegeneratedFirstSection() ? 1 : 0)} {(aCW.IsDegeneratedLastSection() ? 1 : 0)}\n");
            BRep_Builder B = new();
            TopoDS_Compound anOut = new();
            B.MakeCompound(ref anOut);
            foreach (TopoDS_Shape aW in aCW.Shape())
            {
                B.Add(anOut, aW);
            }
            aOut.Append(Block("OUT", Text(anOut)));
            NCollection_IndexedMap<TopoDS_Shape> anInEdges = new(TopTools_ShapeMapHasher.Instance), anOutEdges = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(anIn, TopAbs_ShapeEnum.TopAbs_EDGE, anInEdges);
            TopExp.MapShapes(anOut, TopAbs_ShapeEnum.TopAbs_EDGE, anOutEdges);
            aOut.Append("GEN ");
            for (int i = 1; i <= anInEdges.Extent(); i++)
            {
                aOut.Append(string.Join(",", aCW.GeneratedShapes(TopoDS.Edge(anInEdges.FindKey(i)))
                                              .Select(s => anOutEdges.FindIndex(s) + Orient(s.Orientation()))));
                aOut.Append(';');
            }
            aOut.Append('\n');
        }
        catch (FailureException anE)
        {
            aOut.Append($"THROW {Kerf.Engine.Foundation.OcctTypeNames.Of(anE.GetType())}\n");
        }
        theInputAfter = Text(anIn);
        return aOut.ToString();
    }

    public static IEnumerable<object[]> Kinds() =>
        theCases.Value.Select(c => c.Name.Split(' ')[0]).Distinct().Select(k => new object[] { k });

    [Theory]
    [MemberData(nameof(Kinds))]
    public void Port_matches_native(string theKind)
    {
        int aCount = 0, anIdentical = 0, aMasked = 0;
        List<string> aDiffs = [];
        foreach (Case aCase in theCases.Value.Where(c => c.Name.Split(' ')[0] == theKind))
        {
            aCount++;
            string aPort = Run(aCase, out string aBefore, out string anAfter);
            Assert.True(aBefore == anAfter, $"{aCase.Name}: the input was written");
            if (theKind == "closed-loop")
            {
                // Native isn't repeatable here: with the first section repeated last, the polar method's vertex maps
                // hold a vertex in several lists, and SearchRoot takes the first list in the map's address order
                // (decision 908; native's status and wires change from run to run). The port's order is the binding
                // order: deterministic, and compared with native's only for the record.
                Assert.Equal(aPort, Run(aCase, out _, out _));
            }
            if (aPort == aCase.Expected)
            {
                anIdentical++;
            }
            else if (MaskFree(aPort) == MaskFree(aCase.Expected))
            {
                aMasked++;
            }
            else
            {
                string[] aN = MaskFree(aCase.Expected).Split('\n'), aP = MaskFree(aPort).Split('\n');
                var aD = new StringBuilder($"{aCase.Name}:\n");
                for (int i = 0, n = 0; i < Math.Max(aN.Length, aP.Length) && n < 6; i++)
                {
                    string a = i < aN.Length ? aN[i] : "", b = i < aP.Length ? aP[i] : "";
                    if (a != b)
                    {
                        aD.Append($"  line {i + 1}: native [{a}] port [{b}]\n");
                        n++;
                    }
                }
                aDiffs.Add(aD.ToString());
            }
        }
        output.WriteLine($"{theKind}: {aCount} cases, {anIdentical} identical, {aMasked} equal with the Free column masked, {aDiffs.Count} different");
        foreach (string aD in aDiffs.Take(10))
        {
            output.WriteLine(aD);
        }
        if (theKind != "closed-loop")
        {
            Assert.Empty(aDiffs);
        }
    }
}
