// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Healing.Analysis;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of ShapeAnalysis_WireOrder on shuffled and reversed chains, in 3D, 2D and both modes.</summary>
public class ShapeAnalysisWireOrderTests
{
    private static readonly Xyz P1 = new(0, 0, 0), P2 = new(1, 0, 0), P3 = new(1, 1, 0), P4 = new(0, 1, 0);

    // the square P1 P2 P3 P4: e1 = P1->P2, e2 = P2->P3, e3 = P3->P4, e4 = P4->P1
    private static (Xyz, Xyz)[] Square(in Xyz theShift)
    {
        Xyz s = theShift;
        return [(P1 + s, P2 + s), (P2 + s, P3 + s), (P3 + s, P4 + s), (P4 + s, P1 + s)];
    }

    private static int[] Order(WireOrderAnalysis theWO)
    {
        int[] anOrder = new int[theWO.EdgeCount];
        for (int i = 1; i <= anOrder.Length; i++)
        {
            anOrder[i - 1] = theWO.Ordered(i);
        }
        return anOrder;
    }

    // the edges in the proposed order chain end to start (and the last to the first if closed)
    private static void AssertChained(WireOrderAnalysis theWO, bool theClosed, double theGap = 0)
    {
        int nb = theWO.EdgeCount;
        for (int i = 1; i <= (theClosed ? nb : nb - 1); i++)
        {
            theWO.XYZ(theWO.Ordered(i), out _, out Xyz anEnd);
            theWO.XYZ(theWO.Ordered(i == nb ? 1 : i + 1), out Xyz aStart, out _);
            Assert.True(anEnd.IsEqual(aStart, theGap + 1e-12), $"gap after position {i}");
        }
    }

    private static WireOrderAnalysis Loaded(params (Xyz S, Xyz E)[] theEdges)
    {
        var aWO = new WireOrderAnalysis(true, 1e-7);
        foreach ((Xyz aS, Xyz anE) in theEdges)
        {
            aWO.Add(aS, anE);
        }
        return aWO;
    }

    [Fact]
    public void Ordered_chain_is_kept()
    {
        var aSq = Square(new Xyz());
        WireOrderAnalysis aWO = Loaded(aSq);
        Assert.False(aWO.IsDone);
        Assert.Equal(3, aWO.Ordered(3)); // before Perform
        aWO.Perform();
        Assert.True(aWO.IsDone);
        Assert.Equal(0, aWO.Status());
        Assert.Equal([1, 2, 3, 4], Order(aWO));
        Assert.Equal(0.0, aWO.Gap());
        Assert.Equal(0.0, aWO.Gap(1));
    }

    [Fact]
    public void Shuffled_chain_is_reordered()
    {
        var aSq = Square(new Xyz());
        WireOrderAnalysis aWO = Loaded(aSq[2], aSq[0], aSq[3], aSq[1]);
        aWO.Perform();
        Assert.Equal(1, aWO.Status());
        Assert.Equal([1, 3, 2, 4], Order(aWO));
        AssertChained(aWO, true);
    }

    [Fact]
    public void Reversed_chain_is_walked_backwards()
    {
        var aSq = Square(new Xyz());
        WireOrderAnalysis aWO = Loaded((aSq[0].Item2, aSq[0].Item1), (aSq[1].Item2, aSq[1].Item1),
                                            (aSq[2].Item2, aSq[2].Item1), (aSq[3].Item2, aSq[3].Item1));
        aWO.Perform();
        // only shifted backwards, nothing reversed
        Assert.Equal(3, aWO.Status());
        Assert.Equal([1, 4, 3, 2], Order(aWO));
        AssertChained(aWO, true);
    }

    [Fact]
    public void Reversed_edge_gets_a_negative_index()
    {
        var aSq = Square(new Xyz());
        WireOrderAnalysis aWO = Loaded(aSq[0], (aSq[1].Item2, aSq[1].Item1), aSq[2], aSq[3]);
        aWO.Perform();
        Assert.Equal(-1, aWO.Status());
        Assert.Equal([3, 4, 1, -2], Order(aWO));
        AssertChained(aWO, true);
        aWO.XYZ(-2, out Xyz aStart, out Xyz anEnd);
        Assert.Equal(P2, aStart);
        Assert.Equal(P3, anEnd);
    }

    [Fact]
    public void Shuffled_and_reversed_chain_of_ten()
    {
        // a zigzag polyline of 10 segments, loaded in a scrambled order, half of them reversed
        var aPts = new Xyz[11];
        for (int i = 0; i <= 10; i++)
        {
            aPts[i] = new Xyz(i, (i % 2) * 0.5, 0);
        }
        int[] aPerm = [7, 2, 9, 0, 5, 3, 8, 1, 6, 4];
        var aWO = new WireOrderAnalysis(true, 1e-7);
        for (int k = 0; k < 10; k++)
        {
            int i = aPerm[k];
            if (k % 2 == 0)
            {
                aWO.Add(aPts[i], aPts[i + 1]);
            }
            else
            {
                aWO.Add(aPts[i + 1], aPts[i]);
            }
        }
        aWO.Perform(false);
        Assert.Equal(-1, aWO.Status());
        AssertChained(aWO, false);
        // every edge once
        Assert.Equal(Enumerable.Range(1, 10), Order(aWO).Select(Math.Abs).Order());
    }

    [Fact]
    public void Gaps_are_measured_between_ordered_edges()
    {
        WireOrderAnalysis aWO = Loaded((new Xyz(0, 0, 0), new Xyz(1, 0, 0)),
                                             (new Xyz(1.1, 0, 0), new Xyz(2, 0, 0)),
                                             (new Xyz(2, 0.2, 0.1), new Xyz(3, 0, 0)));
        aWO.Perform(false);
        Assert.Equal(0, aWO.Status());
        Assert.Equal(0.1, aWO.Gap(2), 1e-15);
        Assert.Equal(0.30000000000000004, aWO.Gap(3), 1e-15);
        Assert.Equal(3.0, aWO.Gap(1), 1e-15); // from the end of the last to the start of the first
        Assert.Equal(0.0, aWO.Gap()); // OCCT never computes the largest gap
    }

    [Fact]
    public void Two_loops_kept_or_merged()
    {
        var aA = Square(new Xyz());
        var aB = Square(new Xyz(10, 0, 0));
        (Xyz, Xyz)[] anEdges = [aA[0], aB[0], aA[1], aB[1], aA[2], aB[2], aA[3], aB[3]];

        WireOrderAnalysis aKeep = Loaded(anEdges);
        aKeep.KeepLoopsMode() = true;
        Assert.True(aKeep.KeepLoopsMode());
        aKeep.Perform();
        Assert.Equal([1, 3, 5, 7, 2, 4, 6, 8], Order(aKeep));
        Assert.Equal(1, aKeep.Status());

        WireOrderAnalysis aMerge = Loaded(anEdges);
        aMerge.Perform();
        int[] anOrder = Order(aMerge);
        Assert.Equal(Enumerable.Range(1, 8), anOrder.Select(Math.Abs).Order());
        Assert.Equal(1, aMerge.Status());
        Assert.Equal([1, 2, 4, 6, 8, 3, 5, 7], anOrder);
    }

    [Fact]
    public void Chains_of_an_open_and_a_closed_wire()
    {
        // closed: one chain over all the edges
        WireOrderAnalysis aWO = Loaded(Square(new Xyz()));
        aWO.Perform();
        aWO.SetChains(1e-3);
        Assert.Equal(1, aWO.ChainCount);
        aWO.Chain(1, out int n1, out int n2);
        Assert.Equal((1, 4), (n1, n2));
        aWO.Chain(2, out n1, out n2);
        Assert.Equal((0, 0), (n1, n2));

        // open with a gap before the third edge
        aWO = Loaded((new Xyz(0, 0, 0), new Xyz(1, 0, 0)), (new Xyz(1, 0, 0), new Xyz(2, 0, 0)),
                     (new Xyz(2.5, 0, 0), new Xyz(3, 0, 0)));
        aWO.Perform(false);
        Assert.Equal([1, 2, 3], Order(aWO));
        aWO.SetChains(1e-3);
        // OCCT also checks the wrap-around from the last edge to the start of its own chain, and appends the last
        // chain's start again: three chains, the second empty (3, 2)
        Assert.Equal(3, aWO.ChainCount);
        aWO.Chain(1, out n1, out n2);
        Assert.Equal((1, 2), (n1, n2));
        aWO.Chain(2, out n1, out n2);
        Assert.Equal((3, 2), (n1, n2));
        aWO.Chain(3, out n1, out n2);
        Assert.Equal((3, 3), (n1, n2));
    }

    [Fact]
    public void Couples_are_not_implemented()
    {
        WireOrderAnalysis aWO = Loaded(Square(new Xyz()));
        aWO.Perform();
        aWO.SetCouples(1e-3);
        Assert.Equal(0, aWO.CoupleCount);
        aWO.Couple(1, out int n1, out int n2);
        Assert.Equal((0, 0), (n1, n2));
    }

    [Fact]
    public void Modes_accept_only_their_points()
    {
        var aWO = new WireOrderAnalysis(false, 1e-7);
        aWO.Add(new Xyz(0, 0, 0), new Xyz(1, 0, 0)); // ignored in 2D mode
        Assert.Equal(0, aWO.EdgeCount);
        aWO.Add(new Xy(1, 1), new Xy(0, 1));
        aWO.Add(new Xy(0, 0), new Xy(1, 0));
        aWO.Add(new Xy(1, 0), new Xy(1, 1));
        Assert.Equal(3, aWO.EdgeCount);
        aWO.Perform(false);
        Assert.Equal([2, 3, 1], Order(aWO));
        Assert.Equal(3, aWO.Status()); // a forward shift
        aWO.XY(-1, out Xy aS, out Xy anE);
        Assert.Equal(new Xy(0, 1), aS);
        Assert.Equal(new Xy(1, 1), anE);

        // a new mode clears; a tolerance that isn't positive becomes 1e-8
        aWO.SetMode(true, 0);
        Assert.Equal(0, aWO.EdgeCount);
        Assert.Equal(1e-8, aWO.Tolerance());
        Assert.False(aWO.IsDone);
        aWO.SetMode(true, 1e-3);
        Assert.Equal(1e-3, aWO.Tolerance());

        var aDefault = new WireOrderAnalysis();
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aDefault.Tolerance());
        aDefault.Perform();
        Assert.False(aDefault.IsDone); // no edges: nothing done
        Assert.Equal(0, aDefault.Status());
    }

    [Fact]
    public void Both_modes_order_a_seam_wire()
    {
        // the wire of a cylinder's lateral face: bottom circle, seam up, top circle backwards, seam down; the two seam
        // edges have the same 3D ends, so only 2D tells where each goes
        Xyz aQ = new(1, 0, 0), aR = new(1, 0, 1);
        double aP = 2 * Math.PI;
        (Xyz, Xyz, Xy, Xy)[] anEdges =
        [
            (aQ, aQ, new Xy(0, 0), new Xy(aP, 0)),
            (aQ, aR, new Xy(aP, 0), new Xy(aP, 1)),
            (aR, aR, new Xy(aP, 1), new Xy(0, 1)),
            (aR, aQ, new Xy(0, 1), new Xy(0, 0)),
        ];
        var aWO = new WireOrderAnalysis(true, 1e-7, true);
        foreach (int i in new[] { 0, 3, 2, 1 })
        {
            (Xyz a, Xyz b, Xy c, Xy d) = anEdges[i];
            aWO.Add(a, b, c, d);
        }
        aWO.Add(aQ, aR); // ignored in both mode
        aWO.Add(new Xy(0, 0), new Xy(1, 1)); // ignored too
        Assert.Equal(4, aWO.EdgeCount);
        aWO.Perform();
        int[] anOrder = Order(aWO);
        Assert.Equal([1, 4, 3, 2], anOrder);
        Assert.Equal(3, aWO.Status());
        for (int i = 1; i <= 4; i++)
        {
            aWO.XY(aWO.Ordered(i), out _, out Xy anEnd);
            aWO.XY(aWO.Ordered(i == 4 ? 1 : i + 1), out Xy aStart, out _);
            Assert.True(anEnd.IsEqual(aStart, 1e-12));
        }
    }

    [Fact]
    public void Clear_keeps_the_mode_and_forgets_the_edges()
    {
        WireOrderAnalysis aWO = Loaded(Square(new Xyz()));
        aWO.Perform();
        aWO.Clear();
        Assert.Equal(0, aWO.EdgeCount);
        aWO.Add(P1, P2);
        Assert.Equal(1, aWO.EdgeCount);
    }
}
