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
/// same common parts (counts, types, flags) with every value within 1e-8 relative; the others must be identical.
/// </summary>
public class IntTools_EdgeIntersectorNativeTests(ITestOutputHelper theOutput)
{
    /// <summary>
    /// Same tokens, except that 16-digit hex doubles may differ by 1e-8 relative (to max(1, |a|, |b|)) when
    /// <paramref name="isCircle"/>.
    /// </summary>
    private static bool Agrees(string theNative, string thePort, bool isCircle, ref int theNbNear)
    {
        if (theNative == thePort)
        {
            return true;
        }
        if (!isCircle)
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
            if (!(Math.Abs(a - b) <= 1.0e-8 * Math.Max(1.0, Math.Max(Math.Abs(a), Math.Abs(b)))))
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
            if (!Agrees(aRun.Native, aFresh[k], IsCircle(Edge(aCase, aCase.EA, aRun.I)), ref aNbNear))
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
