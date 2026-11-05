// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The port's BRepOffsetAPI_MakePipe (BRepFill_Pipe, BRepFill_Sweep) and BRepOffsetAPI_MiddlePath against native OCCT's,
/// recorded by tools/piperec (unit 5S2; build123d.NET binds neither). Each recorded case holds its inputs as BRep text,
/// which native OCCT read back before running, so both sides run on the same shapes; the port reads them, freezes them
/// (as a caller's published shapes are) and runs. Compared: the exception, IsDone, ErrorOnSurface's bits, the result,
/// FirstShape and LastShape as BRep text (then with the Free column masked), Generated for every sub-shape of the
/// profile and the spine, and Generated(spine edge, profile edge or vertex), as indices in the result. The inputs'
/// text is also checked unchanged after the run.
/// </summary>
public class BRepOffsetAPI_MakePipeNativeTests(ITestOutputHelper output)
{
    private sealed record Case(string Name, int Mode, bool ForceC1, string Input, string Expected);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    private static List<Case> Load()
    {
        using Stream aRes = typeof(BRepOffsetAPI_MakePipeNativeTests).Assembly.GetManifestResourceStream("BRepOffsetAPI.piperec.txt.gz")
                            ?? throw new InvalidOperationException("BRepOffsetAPI.piperec.txt.gz");
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
            int aMode = -1;
            bool aForce = false;
            if (aLines[i].StartsWith("OPT ", StringComparison.Ordinal))
            {
                string[] anOpt = aLines[i++].Split(' ');
                aMode = int.Parse(anOpt[1], CultureInfo.InvariantCulture);
                aForce = anOpt[2] == "1";
            }
            string anIn = aLines[i].StartsWith("IN ", StringComparison.Ordinal) ? ReadBlock(aLines, ref i) : "";
            var anExpected = new StringBuilder();
            while (i < aLines.Count && !aLines[i].StartsWith("CASE ", StringComparison.Ordinal))
            {
                anExpected.Append(aLines[i++]).Append('\n');
            }
            aCases.Add(new Case(aName, aMode, aForce, anIn, anExpected.ToString()));
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

    private static string Text(in Shape theShape)
    {
        using StringWriter aW = new(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(theShape, aW);
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

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture);

    // The Free flag is the first of the seven flag digits on their own line.
    private static string MaskFree(string theText) => Regex.Replace(theText, "(?m)^[01]([01]{6})$", "x$1");

    private static string Indices(IEnumerable<Shape> theList, IndexedSet<Shape> theMap)
        => string.Join(",", theList.Select(s => (theMap.IndexOf(s) + 1)));

    /// <summary>Runs one recorded case on the port and writes what piperec writes after the input block.</summary>
    private static string Run(Case theCase, out string theInputBefore, out string theInputAfter)
    {
        BrepTools.Read(out Shape anIn, new StringReader(theCase.Input));
        anIn.TShape()!.FreezeGraph();
        theInputBefore = Text(anIn);
        List<Shape> aParts = [];
        for (Iterator anIt = new(anIn); anIt.More(); anIt.Next())
        {
            aParts.Add(anIt.Value());
        }

        var aOut = new StringBuilder();
        try
        {
            if (theCase.Mode < 0)
            {
                // a middle path
                MiddlePath aMP = new(aParts[0], aParts[1], aParts[2]);
                aMP.Build();
                aOut.Append($"STATUS {(aMP.IsDone ? 1 : 0)} -\n");
                if (aMP.IsDone)
                {
                    aOut.Append(Block("OUT", Text(aMP.Shape())));
                }
            }
            else
            {
                Wire aSpine = ShapeCasts.Wire(aParts[0]);
                Shape aProfile = aParts[1];
                Trihedron aTrihedron = theCase.Mode switch
                {
                    0 => Trihedron.IsCorrectedFrenet,
                    1 => Trihedron.IsFrenet,
                    _ => Trihedron.IsDiscreteTrihedron,
                };
                ApiMakePipe aPipe = new(aSpine, aProfile, aTrihedron, theCase.ForceC1);
                aOut.Append($"STATUS {(aPipe.IsDone ? 1 : 0)} {Hex(aPipe.ErrorOnSurface())}\n");
                if (aPipe.IsDone)
                {
                    Shape aRes = aPipe.Shape();
                    aOut.Append(Block("OUT", Text(aRes)));
                    aOut.Append(Block("FIRST", Text(aPipe.FirstShape())));
                    aOut.Append(Block("LAST", Text(aPipe.LastShape())));
                    IndexedSet<Shape> aResMap = new(ShapeHasher.Instance);
                    Exploration.MapShapes(aRes, aResMap);
                    // (piperec builds each GEN line before writing it: an exception leaves no partial line)
                    var aGen = new StringBuilder("GEN ");
                    foreach (Shape anIn2 in new[] { aProfile, aSpine })
                    {
                        IndexedSet<Shape> aMap = new(ShapeHasher.Instance);
                        Exploration.MapShapes(anIn2, aMap);
                        for (int i = 0; i < aMap.Count; i++)
                        {
                            aGen.Append(Indices(aPipe.Generated(aMap[i]), aResMap)).Append(';');
                        }
                    }
                    aOut.Append(aGen).Append('\n');
                    var aGen2 = new StringBuilder("GEN2 ");
                    IndexedSet<Shape> aSpineEdges = new(ShapeHasher.Instance);
                    IndexedSet<Shape> aProfShapes = new(ShapeHasher.Instance);
                    Exploration.MapShapes(aSpine, ShapeType.Edge, aSpineEdges);
                    Exploration.MapShapes(aProfile, ShapeType.Edge, aProfShapes);
                    Exploration.MapShapes(aProfile, ShapeType.Vertex, aProfShapes);
                    for (int i = 0; i < aSpineEdges.Count; i++)
                    {
                        for (int j = 0; j < aProfShapes.Count; j++)
                        {
                            Shape aG = aPipe.Generated(aSpineEdges[i], aProfShapes[j]);
                            aGen2.Append(aG.IsNull ? -1 : (aResMap.IndexOf(aG) + 1)).Append(';');
                        }
                    }
                    aOut.Append(aGen2).Append('\n');
                }
            }
        }
        catch (Exception anE) when (Refusal.IsFailure(anE))
        {
            aOut.Append($"THROW {Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anE).GetType())}\n");
        }
        catch (Exception anE) when (anE is NullReferenceException or IndexOutOfRangeException)
        {
            // native's access violation, which OSD::SetSignal turns into OSD_Exception_ACCESS_VIOLATION (decision 931)
            aOut.Append("THROW OSD_Exception_ACCESS_VIOLATION\n");
        }
        theInputAfter = Text(anIn);
        return aOut.ToString();
    }

    public static IEnumerable<object[]> Kinds() =>
        theCases.Value.Select(c => c.Name.Split(' ')[0]).Distinct().Select(k => new object[] { k });

    /// <summary>
    /// Plan 3.2's third level, an intended difference (occt-issues.md, unit 5S1's BRepFill_Pipe::Edge row; Kerf unit
    /// 10K1, decision 1435): native threw an access violation at Generated(spine edge, profile vertex) of a vertex
    /// profile, where the port gives the edge the vertex swept. Everything before agrees; the port's GEN2 line holds an
    /// edge of the result for each pair.
    /// </summary>
    private static bool IsVertexProfileEdge(string theExpected, string thePort)
    {
        const string aThrow = "THROW OSD_Exception_ACCESS_VIOLATION\n";
        if (!theExpected.EndsWith(aThrow, StringComparison.Ordinal))
        {
            return false;
        }
        string aBefore = theExpected[..^aThrow.Length];
        if (!thePort.StartsWith(aBefore, StringComparison.Ordinal))
        {
            return false;
        }
        string aRest = thePort[aBefore.Length..];
        return Regex.IsMatch(aRest, "^GEN2 ([1-9][0-9]*;)+\n$");
    }

    [Theory]
    [MemberData(nameof(Kinds))]
    public void Port_matches_native(string theKind)
    {
        int aCount = 0, anIdentical = 0, aMasked = 0, anIntended = 0;
        List<string> aDiffs = [];
        foreach (Case aCase in theCases.Value.Where(c => c.Name.Split(' ')[0] == theKind))
        {
            aCount++;
            string aPort = Run(aCase, out string aBefore, out string anAfter);
            Assert.True(aBefore == anAfter, $"{aCase.Name}: the input was written");
            if (aPort == aCase.Expected)
            {
                anIdentical++;
            }
            else if (MaskFree(aPort) == MaskFree(aCase.Expected))
            {
                aMasked++;
            }
            else if (IsVertexProfileEdge(aCase.Expected, aPort))
            {
                anIntended++;
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
        output.WriteLine($"{theKind}: {aCount} cases, {anIdentical} identical, {aMasked} equal with the Free column masked, "
                         + $"{anIntended} with the vertex profile's edges, {aDiffs.Count} different");
        foreach (string aD in aDiffs.Take(10))
        {
            output.WriteLine(aD);
        }
        Assert.Empty(aDiffs);
    }
}
