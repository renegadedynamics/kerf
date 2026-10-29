// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of ShapeAnalysis_WireOrder on shuffled and reversed chains, in 3D, 2D and both modes.</summary>
public class ShapeAnalysisWireOrderTests
{
    private static readonly gp_XYZ P1 = new(0, 0, 0), P2 = new(1, 0, 0), P3 = new(1, 1, 0), P4 = new(0, 1, 0);

    // the square P1 P2 P3 P4: e1 = P1->P2, e2 = P2->P3, e3 = P3->P4, e4 = P4->P1
    private static (gp_XYZ, gp_XYZ)[] Square(in gp_XYZ theShift)
    {
        gp_XYZ s = theShift;
        return [(P1 + s, P2 + s), (P2 + s, P3 + s), (P3 + s, P4 + s), (P4 + s, P1 + s)];
    }

    private static int[] Order(ShapeAnalysis_WireOrder theWO)
    {
        int[] anOrder = new int[theWO.NbEdges()];
        for (int i = 1; i <= anOrder.Length; i++)
        {
            anOrder[i - 1] = theWO.Ordered(i);
        }
        return anOrder;
    }

    // the edges in the proposed order chain end to start (and the last to the first if closed)
    private static void AssertChained(ShapeAnalysis_WireOrder theWO, bool theClosed, double theGap = 0)
    {
        int nb = theWO.NbEdges();
        for (int i = 1; i <= (theClosed ? nb : nb - 1); i++)
        {
            theWO.XYZ(theWO.Ordered(i), out _, out gp_XYZ anEnd);
            theWO.XYZ(theWO.Ordered(i == nb ? 1 : i + 1), out gp_XYZ aStart, out _);
            Assert.True(anEnd.IsEqual(aStart, theGap + 1e-12), $"gap after position {i}");
        }
    }

    private static ShapeAnalysis_WireOrder Loaded(params (gp_XYZ S, gp_XYZ E)[] theEdges)
    {
        var aWO = new ShapeAnalysis_WireOrder(true, 1e-7);
        foreach ((gp_XYZ aS, gp_XYZ anE) in theEdges)
        {
            aWO.Add(aS, anE);
        }
        return aWO;
    }

    [Fact]
    public void Ordered_chain_is_kept()
    {
        var aSq = Square(new gp_XYZ());
        ShapeAnalysis_WireOrder aWO = Loaded(aSq);
        Assert.False(aWO.IsDone());
        Assert.Equal(3, aWO.Ordered(3)); // before Perform
        aWO.Perform();
        Assert.True(aWO.IsDone());
        Assert.Equal(0, aWO.Status());
        Assert.Equal([1, 2, 3, 4], Order(aWO));
        Assert.Equal(0.0, aWO.Gap());
        Assert.Equal(0.0, aWO.Gap(1));
    }

    [Fact]
    public void Shuffled_chain_is_reordered()
    {
        var aSq = Square(new gp_XYZ());
        ShapeAnalysis_WireOrder aWO = Loaded(aSq[2], aSq[0], aSq[3], aSq[1]);
        aWO.Perform();
        Assert.Equal(1, aWO.Status());
        Assert.Equal([1, 3, 2, 4], Order(aWO));
        AssertChained(aWO, true);
    }

    [Fact]
    public void Reversed_chain_is_walked_backwards()
    {
        var aSq = Square(new gp_XYZ());
        ShapeAnalysis_WireOrder aWO = Loaded((aSq[0].Item2, aSq[0].Item1), (aSq[1].Item2, aSq[1].Item1),
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
        var aSq = Square(new gp_XYZ());
        ShapeAnalysis_WireOrder aWO = Loaded(aSq[0], (aSq[1].Item2, aSq[1].Item1), aSq[2], aSq[3]);
        aWO.Perform();
        Assert.Equal(-1, aWO.Status());
        Assert.Equal([3, 4, 1, -2], Order(aWO));
        AssertChained(aWO, true);
        aWO.XYZ(-2, out gp_XYZ aStart, out gp_XYZ anEnd);
        Assert.Equal(P2, aStart);
        Assert.Equal(P3, anEnd);
    }

    [Fact]
    public void Shuffled_and_reversed_chain_of_ten()
    {
        // a zigzag polyline of 10 segments, loaded in a scrambled order, half of them reversed
        var aPts = new gp_XYZ[11];
        for (int i = 0; i <= 10; i++)
        {
            aPts[i] = new gp_XYZ(i, (i % 2) * 0.5, 0);
        }
        int[] aPerm = [7, 2, 9, 0, 5, 3, 8, 1, 6, 4];
        var aWO = new ShapeAnalysis_WireOrder(true, 1e-7);
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
        ShapeAnalysis_WireOrder aWO = Loaded((new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0)),
                                             (new gp_XYZ(1.1, 0, 0), new gp_XYZ(2, 0, 0)),
                                             (new gp_XYZ(2, 0.2, 0.1), new gp_XYZ(3, 0, 0)));
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
        var aA = Square(new gp_XYZ());
        var aB = Square(new gp_XYZ(10, 0, 0));
        (gp_XYZ, gp_XYZ)[] anEdges = [aA[0], aB[0], aA[1], aB[1], aA[2], aB[2], aA[3], aB[3]];

        ShapeAnalysis_WireOrder aKeep = Loaded(anEdges);
        aKeep.KeepLoopsMode() = true;
        Assert.True(aKeep.KeepLoopsMode());
        aKeep.Perform();
        Assert.Equal([1, 3, 5, 7, 2, 4, 6, 8], Order(aKeep));
        Assert.Equal(1, aKeep.Status());

        ShapeAnalysis_WireOrder aMerge = Loaded(anEdges);
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
        ShapeAnalysis_WireOrder aWO = Loaded(Square(new gp_XYZ()));
        aWO.Perform();
        aWO.SetChains(1e-3);
        Assert.Equal(1, aWO.NbChains());
        aWO.Chain(1, out int n1, out int n2);
        Assert.Equal((1, 4), (n1, n2));
        aWO.Chain(2, out n1, out n2);
        Assert.Equal((0, 0), (n1, n2));

        // open with a gap before the third edge
        aWO = Loaded((new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0)), (new gp_XYZ(1, 0, 0), new gp_XYZ(2, 0, 0)),
                     (new gp_XYZ(2.5, 0, 0), new gp_XYZ(3, 0, 0)));
        aWO.Perform(false);
        Assert.Equal([1, 2, 3], Order(aWO));
        aWO.SetChains(1e-3);
        // OCCT also checks the wrap-around from the last edge to the start of its own chain, and appends the last
        // chain's start again: three chains, the second empty (3, 2)
        Assert.Equal(3, aWO.NbChains());
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
        ShapeAnalysis_WireOrder aWO = Loaded(Square(new gp_XYZ()));
        aWO.Perform();
        aWO.SetCouples(1e-3);
        Assert.Equal(0, aWO.NbCouples());
        aWO.Couple(1, out int n1, out int n2);
        Assert.Equal((0, 0), (n1, n2));
    }

    [Fact]
    public void Modes_accept_only_their_points()
    {
        var aWO = new ShapeAnalysis_WireOrder(false, 1e-7);
        aWO.Add(new gp_XYZ(0, 0, 0), new gp_XYZ(1, 0, 0)); // ignored in 2D mode
        Assert.Equal(0, aWO.NbEdges());
        aWO.Add(new gp_XY(1, 1), new gp_XY(0, 1));
        aWO.Add(new gp_XY(0, 0), new gp_XY(1, 0));
        aWO.Add(new gp_XY(1, 0), new gp_XY(1, 1));
        Assert.Equal(3, aWO.NbEdges());
        aWO.Perform(false);
        Assert.Equal([2, 3, 1], Order(aWO));
        Assert.Equal(3, aWO.Status()); // a forward shift
        aWO.XY(-1, out gp_XY aS, out gp_XY anE);
        Assert.Equal(new gp_XY(0, 1), aS);
        Assert.Equal(new gp_XY(1, 1), anE);

        // a new mode clears; a tolerance that isn't positive becomes 1e-8
        aWO.SetMode(true, 0);
        Assert.Equal(0, aWO.NbEdges());
        Assert.Equal(1e-8, aWO.Tolerance());
        Assert.False(aWO.IsDone());
        aWO.SetMode(true, 1e-3);
        Assert.Equal(1e-3, aWO.Tolerance());

        var aDefault = new ShapeAnalysis_WireOrder();
        Assert.Equal(Precision.Confusion(), aDefault.Tolerance());
        aDefault.Perform();
        Assert.False(aDefault.IsDone()); // no edges: nothing done
        Assert.Equal(0, aDefault.Status());
    }

    [Fact]
    public void Both_modes_order_a_seam_wire()
    {
        // the wire of a cylinder's lateral face: bottom circle, seam up, top circle backwards, seam down; the two seam
        // edges have the same 3D ends, so only 2D tells where each goes
        gp_XYZ aQ = new(1, 0, 0), aR = new(1, 0, 1);
        double aP = 2 * Math.PI;
        (gp_XYZ, gp_XYZ, gp_XY, gp_XY)[] anEdges =
        [
            (aQ, aQ, new gp_XY(0, 0), new gp_XY(aP, 0)),
            (aQ, aR, new gp_XY(aP, 0), new gp_XY(aP, 1)),
            (aR, aR, new gp_XY(aP, 1), new gp_XY(0, 1)),
            (aR, aQ, new gp_XY(0, 1), new gp_XY(0, 0)),
        ];
        var aWO = new ShapeAnalysis_WireOrder(true, 1e-7, true);
        foreach (int i in new[] { 0, 3, 2, 1 })
        {
            (gp_XYZ a, gp_XYZ b, gp_XY c, gp_XY d) = anEdges[i];
            aWO.Add(a, b, c, d);
        }
        aWO.Add(aQ, aR); // ignored in both mode
        aWO.Add(new gp_XY(0, 0), new gp_XY(1, 1)); // ignored too
        Assert.Equal(4, aWO.NbEdges());
        aWO.Perform();
        int[] anOrder = Order(aWO);
        Assert.Equal([1, 4, 3, 2], anOrder);
        Assert.Equal(3, aWO.Status());
        for (int i = 1; i <= 4; i++)
        {
            aWO.XY(aWO.Ordered(i), out _, out gp_XY anEnd);
            aWO.XY(aWO.Ordered(i == 4 ? 1 : i + 1), out gp_XY aStart, out _);
            Assert.True(anEnd.IsEqual(aStart, 1e-12));
        }
    }

    [Fact]
    public void Clear_keeps_the_mode_and_forgets_the_edges()
    {
        ShapeAnalysis_WireOrder aWO = Loaded(Square(new gp_XYZ()));
        aWO.Perform();
        aWO.Clear();
        Assert.Equal(0, aWO.NbEdges());
        aWO.Add(P1, P2);
        Assert.Equal(1, aWO.NbEdges());
    }
}
