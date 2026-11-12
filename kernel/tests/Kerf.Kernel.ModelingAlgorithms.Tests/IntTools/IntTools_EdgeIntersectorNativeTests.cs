// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.ModelingAlgorithms.Tests.EdgeIntersectorRecordings;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntTools_EdgeEdge, IntTools_EdgeFace and IntTools_BeanFaceIntersector against native OCCT's results recorded by
/// tools/edgerec (decision 754): the common parts (types, ranges, vertex parameters, bounding points) bit for bit, on
/// pairs of edges and faces of data/occ shapes moved against themselves and crossed by probe edges. Runs on a circular
/// edge may differ in the last bits, where .NET's and MSVC's sine and cosine differ (decision 754): they must give the
/// same common parts (counts, types, flags) with every value within 1e-8 relative; the others must be identical, but for
/// the runs a deliberate change of Kerf's moves off native's in the last bits, each pinned to what it gives now
/// (<see cref="Pinned"/>).
/// </summary>
public class IntTools_EdgeIntersectorNativeTests(ITestOutputHelper theOutput)
{
    /// <summary>
    /// The edge/face runs that a deliberate change moves off native's values, by case and the run's edge and face: what the
    /// port gives now, to the bit, which must also be within 1e-12 relative of native's.
    /// </summary>
    private static readonly Dictionary<(string Case, int Edge, int Face), string> Pinned = new()
    {
        // Unit 15Y2, decision 2282: the samples of this line beside the B-spline face are told apart along their own
        // directions, so the extremum that finds the crossing is given a smaller sample of the face than the boxes left
        // it, and ends 11 to 13 units in the last place from native's (under 2e-15 of the parameter)
        [("mat-probes", 28, 1)] = "1 0 7fefffffffffffff 1 | 7 1 401dabddc4d2c26c 401dabde0138a448 0 401dabdde305b35a 0000000000000000 0 "
                                  + "408aceeee708916b 408a84cade07b59b 401c84fd4a14d940 408aceeee6e50880 408a84cade34d272 401c84fd7f37b183",
    };

    /// <summary>
    /// Same tokens, except that 16-digit hex doubles may differ by 1e-8 relative (to max(1, |a|, |b|)) when
    /// <paramref name="isCircle"/>.
    /// </summary>
    private static bool Agrees(string theNative, string thePort, bool isCircle, ref int theNbNear) =>
        Agrees(theNative, thePort, isCircle ? 1.0e-8 : 0.0, ref theNbNear);

    private static bool Agrees(string theNative, string thePort, double theTolerance, ref int theNbNear)
    {
        if (theNative == thePort)
        {
            return true;
        }
        if (theTolerance == 0.0)
        {
            return false;
        }
        string[] n = theNative.Split(' '), p = thePort.Split(' ');
        if (n.Length != p.Length)
        {
            return false;
        }
        for (int i = 0; i < n.Length; i++)
        {
            if (n[i] == p[i])
            {
                continue;
            }
            if (n[i].Length != 16 || p[i].Length != 16)
            {
                return false;
            }
            double a = BitConverter.Int64BitsToDouble(Convert.ToInt64(n[i], 16));
            double b = BitConverter.Int64BitsToDouble(Convert.ToInt64(p[i], 16));
            if (!(Math.Abs(a - b) <= theTolerance * Math.Max(1.0, Math.Max(Math.Abs(a), Math.Abs(b)))))
            {
                return false;
            }
        }
        theNbNear++;
        return true;
    }

    private static bool IsCircle(in global::Kerf.Topology.Edge theE) => new EdgeCurveAdaptor(theE).GetType() == CurveKind.Circle;

    private void Report(string theCase, List<string> theMismatches, int theTotal, string theWhat)
    {
        foreach (string aLine in theMismatches.Take(40))
        {
            theOutput.WriteLine(aLine);
        }
        Assert.True(theMismatches.Count == 0, $"{theCase}: {theMismatches.Count} of {theTotal} {theWhat} differ from native");
    }

    [Theory]
    [MemberData(nameof(CaseNames), MemberType = typeof(EdgeIntersectorRecordings))]
    public void EdgeEdge_GivesNativeCommonParts(string theCase)
    {
        Case aCase = Cases[theCase];
        List<string> aMismatches = [];
        int aNbNear = 0;
        foreach (EERun aRun in aCase.EE)
        {
            global::Kerf.Topology.Edge aE1 = Edge(aCase, aCase.EA, aRun.I), aE2 = Edge(aCase, aCase.EB, aRun.J);
            string aPort = RunEE(aE1, aE2, aRun.Variant);
            if (!Agrees(aRun.Native, aPort, IsCircle(aE1) || IsCircle(aE2), ref aNbNear))
            {
                aMismatches.Add($"EE {aRun.I} {aRun.J} {aRun.Variant}\n  native {aRun.Native}\n  port   {aPort}");
            }
        }
        theOutput.WriteLine($"{theCase}: {aCase.EE.Count} edge/edge runs, {aNbNear} within 1e-8 on a circle, the rest identical");
        Report(theCase, aMismatches, aCase.EE.Count, "edge/edge runs");
    }

    /// <summary>
    /// Each edge/face pair with a fresh context, then all the case's pairs with one context in the recorded order (as a
    /// serial pave filler runs them), in reverse order and shuffled: every run gives native's fresh-context result
    /// (decision 753).
    /// </summary>
    [Theory]
    [MemberData(nameof(CaseNames), MemberType = typeof(EdgeIntersectorRecordings))]
    public void EdgeFace_GivesNativeCommonParts_WithFreshAndReusedContexts(string theCase)
    {
        Case aCase = Cases[theCase];
        List<string> aMismatches = [];
        List<int> anOrder = [.. Enumerable.Range(0, aCase.EF.Count)];
        string[] aFresh = new string[aCase.EF.Count];
        int aNbNear = 0;
        foreach (int k in anOrder)
        {
            EFRun aRun = aCase.EF[k];
            aFresh[k] = RunEF(Edge(aCase, aCase.EA, aRun.I), Face(aCase, aRun.J), new IntersectionContext());
            int aNbPinned = 0;
            bool isPinned = Pinned.TryGetValue((theCase, aRun.I, aRun.J), out string? aPinned) && aFresh[k] == aPinned
                            && Agrees(aRun.Native, aFresh[k], 1.0e-12, ref aNbPinned);
            if (!isPinned && !Agrees(aRun.Native, aFresh[k], IsCircle(Edge(aCase, aCase.EA, aRun.I)), ref aNbNear))
            {
                aMismatches.Add($"EF {aRun.I} {aRun.J} L{(aRun.Localized ? 1 : 0)} {new EdgeCurveAdaptor(Edge(aCase, aCase.EA, aRun.I)).GetType()} {new FaceSurfaceAdaptor(Face(aCase, aRun.J)).GetType()}\n  native {aRun.Native}\n  port   {aFresh[k]}");
            }
        }
        theOutput.WriteLine($"{theCase}: {aCase.EF.Count} edge/face runs, {aNbNear} within 1e-8 on a circle, the rest identical");
        Report(theCase, aMismatches, aCase.EF.Count, "edge/face runs (fresh context)");

        Random aRnd = new(6601);
        // The reverse and shuffled orders where the intersector caches boxes in the context (localised faces).
        List<int>[] anOrders = aCase.EF.Any(r => r.Localized)
                                 ? [anOrder, [.. Enumerable.Reverse(anOrder)], [.. anOrder.OrderBy(_ => aRnd.Next())]]
                                 : [anOrder];
        foreach (List<int> aSequence in anOrders)
        {
            IntersectionContext aCtx = new();
            foreach (int k in aSequence)
            {
                EFRun aRun = aCase.EF[k];
                string aShared = RunEF(Edge(aCase, aCase.EA, aRun.I), Face(aCase, aRun.J), aCtx);
                if (aShared != aFresh[k])
                {
                    aMismatches.Add($"EF {aRun.I} {aRun.J} reused context\n  fresh  {aFresh[k]}\n  reused {aShared}");
                }
            }
        }
        Report(theCase, aMismatches, anOrders.Length * aCase.EF.Count, "edge/face runs with a reused context");
    }
}
