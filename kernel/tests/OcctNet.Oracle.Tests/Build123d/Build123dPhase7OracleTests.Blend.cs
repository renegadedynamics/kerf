// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using OcctNet.TestHarness;
using static Kerf.Topology.ShapeType;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// build123d.NET's fillets and chamfers (unit 7X, its second part): compared geometrically (option C, decision 992: the
/// reconstruction's faces come in another order than native's HBuilder's), as unit 7N3's oracle compares them: volume,
/// area, the counts of solids, shells, faces, edges and vertices, validity (build123d.NET's finishing step: an invalid
/// result is its ArgumentException), the optimal box; the history build123d.NET records (Generated and Modified of every
/// vertex, edge, face and solid, its presence in the result) and IsDeleted, as sets of the result's sub-shapes' measures;
/// the exception, IsDone and the faulty contours (their stripe statuses and edges, as FilletFailureDetails reads them)
/// exactly; MaxFillet's probes and radius exactly.
/// </summary>
public sealed partial class Build123dPhase7OracleTests
{
    private void CompareBlend(P7Call theCall)
    {
        string aLabel = Build123dPhase7Capture.Label(theCall);
        string aWhat = $"{theCall.Test}: {theCall.Name} [{aLabel}]";
        Tally aTally = TallyOf(aLabel);
        aTally.Calls++;
        string anOperands = Build123dPhase7Capture.OperandsText(theCall.Operands);
        P7Native aN = Build123dPhase7Capture.RunNative(theCall, Build123dPhase7Capture.NativeOperands(anOperands));
        List<Shape> aPOps = Build123dPhase7Capture.PortOperands(anOperands);
        string aPOpsText = string.Concat(aPOps.Select(BRepTextComparison.Write));
        P7Port aP = Build123dPhase7Capture.RunPort(theCall, aPOps);
        if (string.Concat(aPOps.Select(BRepTextComparison.Write)) != aPOpsText)
        {
            aTally.Differences.Add($"{aWhat}: the port changed its frozen operands");
        }
        if (aP.Result is Shape aPR && !aPR.IsNull && !aPR.TShape()!.IsFrozen)
        {
            aTally.Differences.Add($"{aWhat}: the port's result isn't frozen");
        }
        CheckBlendTranscription(theCall, aN, aWhat);
        if (aN.Error is not null || aP.Error is not null)
        {
            if (aN.Error == aP.Error && aN.Status == aP.Status)
            {
                aTally.SameError++;
                aTally.Failures.Add($"{aWhat}: {aN.Error} {aN.Status}");
            }
            else
            {
                aTally.Differences.Add($"{aWhat}: native {aN.Error} {aN.Status} | port {aP.Error} {aP.Status}");
            }
            return;
        }
        if (aN.Status != aP.Status)
        {
            aTally.Differences.Add($"{aWhat}: status native {aN.Status} | port {aP.Status}");
            return;
        }
        if (theCall.Op == P7Op.MaxFillet)
        {
            aTally.Identical++; // every probe's outcome and the radius
            return;
        }
        if (aN.Result is null || aN.Result.IsNull() || aP.Result is not Shape aPRes || aPRes.IsNull)
        {
            aTally.Differences.Add($"{aWhat}: a result on one side only");
            return;
        }
        Shape aNRes = BooleanOracleTests.ReadPort(Text(aN.Result));
        var aNOut = new BRepFilletAPIMakeFilletOracleTests.Outcome(null, "", aNRes, []);
        var aPOut = new BRepFilletAPIMakeFilletOracleTests.Outcome(null, "", aPRes, []);
        List<string> aDiffs = BRepFilletAPIMakeFilletOracleTests.Geometric(aNOut, aPOut);
        aTally.Histories++;
        List<string> aHistoryDiffs = HistoryDifferences(aN.History, aNRes, aP.History, aPRes);
        if (aHistoryDiffs.RemoveAll(d => d.StartsWith(SolidSplit, StringComparison.Ordinal)) > 0)
        {
            aTally.SolidSplit++;
        }
        if (aHistoryDiffs.Count == 0)
        {
            aTally.HistoriesGeometric++;
        }
        aDiffs.AddRange(aHistoryDiffs);
        if (aDiffs.Count == 0)
        {
            aTally.Geometric++;
            return;
        }
        bool isNValid = new BRepCheck_Analyzer(aNRes).IsValid(), isPValid = new BRepCheck_Analyzer(aPRes).IsValid();
        string aSummary = string.Join("; ", aDiffs.Take(4));
        if (!isNValid)
        {
            aTally.NativeInvalid++;
            aTally.AttributedCases.Add($"{aWhat}: native's result is invalid {BRepFilletAPIMakeFilletOracleTests.ReconFaults(aNRes)}, the port's "
                                       + $"{(isPValid ? "valid" : "invalid " + BRepFilletAPIMakeFilletOracleTests.ReconFaults(aPRes))}; {aSummary}");
            return;
        }
        aTally.Differences.Add($"{aWhat}: {(isPValid ? "" : "the port's result is invalid " + BRepFilletAPIMakeFilletOracleTests.ReconFaults(aPRes) + "; ")}"
                               + string.Join("; ", aDiffs.Take(Environment.GetEnvironmentVariable("P7X_DETAIL") is null ? 4 : 100)));
    }

    // The transcription: native's replay gives build123d.NET's own result, its failure (with its message's details) or radius
    private void CheckBlendTranscription(P7Call theCall, P7Native theN, string theWhat)
    {
        myTranscribed++;
        if (theCall.Op == P7Op.MaxFillet)
        {
            string anOwn = theCall.Error ?? "max=" + theCall.Message;
            string aReplay = theN.Error ?? theN.Status.Split(' ')[0];
            if (anOwn != aReplay)
            {
                myTranscription.Add($"{theWhat}: build123d.NET's {anOwn}, the replay's {aReplay}");
            }
            return;
        }
        bool isReplayFailing = theN.Error is not null || theN.Result is null || theN.Result.IsNull() || !NativeValid(theN.Result);
        if (theCall.Error is not null || theCall.Result is null)
        {
            if (!isReplayFailing)
            {
                myTranscription.Add($"{theWhat}: build123d.NET threw {theCall.Error}, the replay succeeded");
            }
            else if (theCall.Op == P7Op.Fillet && theN.Details is string aDetails
                     && theCall.Message?.Contains(aDetails + ", try a smaller value", StringComparison.Ordinal) != true)
            {
                myTranscription.Add($"{theWhat}: build123d.NET's message \"{theCall.Message}\", the replay's details \"{aDetails}\"");
            }
            return;
        }
        if (isReplayFailing)
        {
            myTranscription.Add($"{theWhat}: build123d.NET succeeded, the replay failed ({theN.Error ?? theN.Status})");
            return;
        }
        string anExpected = Signature(theCall.Result), aReplayed = Signature(theN.Result!);
        if (anExpected != aReplayed)
        {
            myTranscription.Add($"{theWhat}: build123d.NET's {anExpected}, the replay's {aReplayed}");
        }
    }

    private static bool NativeValid(N.TopoDS_Shape theShape)
    {
        N.TopoDS_Shape aShape = theShape;
        while (aShape.ShapeType() == N.TopAbs_ShapeEnum.TopAbs_COMPOUND && aShape.NbChildren() == 1)
        {
            using var anIt = new N.TopoDS_Iterator(aShape);
            N.TopoDS_Shape aChild = anIt.Value();
            GC.KeepAlive(aShape);
            aShape = aChild;
        }
        using var aCheck = new N.BRepCheck_Analyzer(aShape);
        return aCheck.IsValid();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The history as sets of measures of the result's sub-shapes

    private static (string Type, double[] Values) Measure(in Shape theShape)
    {
        var aProps = new Properties();
        switch (theShape.ShapeType())
        {
            case ShapeType.Face:
                BRepGProp.SurfaceProperties(theShape, aProps);
                return ("F " + new FaceSurfaceAdaptor(ShapeCasts.Face(theShape)).GetType(), [aProps.Mass()]);
            case ShapeType.Edge:
                BRepGProp.LinearProperties(theShape, aProps);
                return ("E " + new EdgeCurveAdaptor(ShapeCasts.Edge(theShape)).GetType(), [aProps.Mass()]);
            case ShapeType.Vertex:
            {
                Point3D aP = Tool.Pnt(ShapeCasts.Vertex(theShape));
                return ("V", [aP.X(), aP.Y(), aP.Z()]);
            }
            default:
                BRepGProp.VolumeProperties(theShape, aProps);
                return (theShape.ShapeType().ToString(), [aProps.Mass()]);
        }
    }

    private static bool Close(double a, double b) => Math.Abs(a - b) <= 1.0e-5 * Math.Max(1.0, Math.Max(Math.Abs(a), Math.Abs(b)));

    private static List<(string Type, double[] Values)> Measures(string theList, NCollection_IndexedMap<Shape> theMap) =>
        [.. theList.Split(' ', StringSplitOptions.RemoveEmptyEntries)
                   .Select(s => int.Parse(s.TrimEnd('r'), CultureInfo.InvariantCulture))
                   .Select(i => i > 0 ? Measure(theMap[i]) : ("out", Array.Empty<double>()))
                   .OrderBy(m => m.Item1, StringComparer.Ordinal).ThenBy(m => m.Item2.FirstOrDefault()).ThenBy(m => m.Item2.Skip(1).FirstOrDefault())];

    private static bool SameMeasures(List<(string Type, double[] Values)> a, List<(string Type, double[] Values)> b) =>
        a.Count == b.Count && a.Zip(b).All(p => p.First.Type == p.Second.Type && p.First.Values.Length == p.Second.Values.Length
                                                && p.First.Values.Zip(p.Second.Values).All(v => Close(v.First, v.Second)));

    private static string MeasuresText(List<(string Type, double[] Values)> theList) =>
        "[" + string.Join(", ", theList.Select(m => m.Type + " " + string.Join(" ", m.Values.Select(v => v.ToString("G6", CultureInfo.InvariantCulture))))) + "]";

    /// <summary>
    /// The difference attributed to the reconstruction (for unit 7R3): native's HBuilder splits the input solid (state IN)
    /// into the result's, so <c>Modified</c> gives the result's solid and <c>IsDeleted</c> is false; the reconstruction
    /// records the splits of edges and faces only, so the port's <c>Modified</c> of the solid is empty and
    /// <c>IsDeleted</c> true. build123d.NET's history then records the solid removed rather than modified.
    /// </summary>
    private const string SolidSplit = "the input solid's split: ";

    private static readonly System.Text.RegularExpressions.Regex s_historyLine = new(@"^(\d+):G\[([^\]]*)\]M\[([^\]]*)\](D?)(P?)$");

    private static List<string> HistoryDifferences(string theNHistory, in Shape theNResult, string thePHistory, in Shape thePResult)
    {
        var aDiffs = new List<string>();
        string[] aNLines = theNHistory.Split('\n', StringSplitOptions.RemoveEmptyEntries);
        string[] aPLines = thePHistory.Split('\n', StringSplitOptions.RemoveEmptyEntries);
        if (aNLines.Length != aPLines.Length)
        {
            aDiffs.Add($"history of {aNLines.Length} | {aPLines.Length} sub-shapes");
            return aDiffs;
        }
        var aNMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theNResult, aNMap);
        var aPMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(thePResult, aPMap);
        for (int i = 0; i < aNLines.Length; i++)
        {
            System.Text.RegularExpressions.Match aN = s_historyLine.Match(aNLines[i]), aP = s_historyLine.Match(aPLines[i]);
            if (!aN.Success || !aP.Success || aN.Groups[1].Value != aP.Groups[1].Value)
            {
                aDiffs.Add($"history line {aNLines[i]} | {aPLines[i]}");
                continue;
            }
            string aKey = aN.Groups[1].Value;
            if (aN.Groups[3].Value.Length > 0 && aP.Groups[3].Value.Length == 0 && aN.Groups[4].Value.Length == 0 && aP.Groups[4].Value == "D"
                && aN.Groups[2].Value == aP.Groups[2].Value && aN.Groups[5].Value == aP.Groups[5].Value
                && Measures(aN.Groups[3].Value, aNMap) is [(string aType, _)] && aType == "TopAbs_SOLID")
            {
                aDiffs.Add(SolidSplit + aKey);
                continue;
            }
            if (aN.Groups[4].Value != aP.Groups[4].Value)
            {
                aDiffs.Add($"history {aKey} IsDeleted {aN.Groups[4].Value == "D"} | {aP.Groups[4].Value == "D"}");
            }
            if (aN.Groups[5].Value != aP.Groups[5].Value)
            {
                aDiffs.Add($"history {aKey} in the result {aN.Groups[5].Value == "P"} | {aP.Groups[5].Value == "P"}");
            }
            foreach ((int aGroup, string aName) in new[] { (2, "Generated"), (3, "Modified") })
            {
                List<(string, double[])> aNM = Measures(aN.Groups[aGroup].Value, aNMap), aPM = Measures(aP.Groups[aGroup].Value, aPMap);
                if (!SameMeasures(aNM, aPM))
                {
                    aDiffs.Add($"history {aKey} {aName} {MeasuresText(aNM)} | {MeasuresText(aPM)}");
                }
            }
        }
        return aDiffs;
    }
}
