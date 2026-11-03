// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Text;
using OcctNet.DrawLite;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Captures the inputs of chosen commands while Draw-lite runs corpus cases, for tools/borec, the native recorder of
/// unit 6G: native OCCT then runs the same commands on the same BRep text, without a Tcl interpreter of its own.
/// </summary>
/// <remarks>
/// Each chosen command is renamed and wrapped: a call records its words and the BRep text (version 3, as
/// BRepTextComparison writes it) of every word naming a shape at the time of the call, then runs the real command.
/// A command Draw-lite lacks is captured without being run (its result variables are then missing). The capture
/// file is what tools/borec/common.hxx's ReadCaptures reads.
/// </remarks>
internal static class DrawCapture
{
    /// <summary>A captured call: its words, and the BRep text of each word naming a shape.</summary>
    internal sealed record Call(IReadOnlyList<string> Words, IReadOnlyDictionary<string, string> Shapes);

    /// <summary>Runs a case with the commands wrapped, and returns their calls in order.</summary>
    public static List<Call> Capture(DrawCorpus theCorpus, DrawTestCase theCase, IReadOnlyCollection<string> theCommands,
                                     TimeSpan theTimeLimit)
    {
        List<Call> aCalls = [];
        theCorpus.RunAndCheck(theCase, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath, theTimeLimit, i =>
        {
            foreach (string aCommand in theCommands)
            {
                string aReal = "__capture_" + aCommand;
                bool anExists = i.HasCommand(aCommand);
                if (anExists)
                {
                    i.Eval($"rename {aCommand} {aReal}");
                }
                i.Register(aCommand, (theDI, a) =>
                {
                    Dictionary<string, string> aShapes = new(StringComparer.Ordinal);
                    foreach (string aWord in a.Skip(1))
                    {
                        if (!aShapes.ContainsKey(aWord) && theDI.GetDrawable(aWord) is DrawShape { Shape: var aShape } && !aShape.IsNull)
                        {
                            aShapes[aWord] = Write(aShape);
                        }
                    }
                    aCalls.Add(new Call([.. a], aShapes));
                    return anExists ? theDI.Invoke([aReal, .. a.Skip(1)]) : "";
                });
            }
        });
        return aCalls;
    }

    private static string Write(Shape theShape)
    {
        using var aWriter = new StringWriter(System.Globalization.CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    /// <summary>Writes a case's calls as tools/borec reads them.</summary>
    public static void Write(TextWriter theWriter, string theCaseName, IEnumerable<Call> theCalls)
    {
        StringBuilder aB = new();
        aB.Append("case ").Append(theCaseName).Append('\n');
        foreach (Call aCall in theCalls)
        {
            aB.Append("call ").Append(aCall.Words.Count).Append('\n');
            foreach (string aWord in aCall.Words)
            {
                aB.Append("w ").Append(aWord.Replace('\n', ' ')).Append('\n');
            }
            foreach ((string aName, string aText) in aCall.Shapes)
            {
                aB.Append("shape ").Append(aName).Append(' ').Append(Encoding.UTF8.GetByteCount(aText)).Append('\n').Append(aText).Append('\n');
            }
        }
        theWriter.Write(aB.ToString());
    }

    /// <summary>
    /// Captures the calls of the given commands over every listed case of the given grids of the boolean group (and of
    /// any other group/grid pairs given as "group/grid") into a file, for tools/borec.
    /// </summary>
    public static int CaptureGrids(string theFile, IReadOnlyCollection<string> theCommands, params string[] theGrids)
    {
        var aCorpus = new DrawCorpus(Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests"));
        HashSet<string> aGrids = [.. theGrids.Select(g => g.Contains('/') ? g : "boolean/" + g)];
        List<DrawTestCase> aCases = [.. aCorpus.Cases().Where(c => aGrids.Contains(c.Group + "/" + c.Grid))];
        var aResults = aCases.AsParallel().AsOrdered().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => (Case: c, Calls: Capture(aCorpus, c, theCommands, TimeSpan.FromSeconds(300)))).ToList();
        using var aWriter = new StreamWriter(theFile, false, new UTF8Encoding(false)) { NewLine = "\n" };
        int aNb = 0;
        foreach ((DrawTestCase aCase, List<Call> aCalls) in aResults)
        {
            if (aCalls.Count > 0)
            {
                Write(aWriter, $"{aCase.Group}/{aCase.Grid}/{aCase.Name}", aCalls);
                aNb += aCalls.Count;
            }
        }
        return aNb;
    }
}
