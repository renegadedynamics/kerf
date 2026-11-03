// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// TopTrans_CurveTransition on known configurations (OCCT has no GTest for it). Directions are in the plane z = 0,
/// as the 2D classifiers use them; the boundary's matter is on the left of its edges.
/// </summary>
public class TopTrans_CurveTransition_PortTests
{
    private const double Tol = 1.0e-9;

    private static Direction3D D(double x, double y) => new(x, y, 0.0);

    [Fact]
    public void Empty_and_reset_transitions()
    {
        // OCCT's constructor leaves Init false and both transitions FORWARD
        var aTransition = new TopTrans_CurveTransition();
        Assert.Equal(TopAbs_OUT, aTransition.StateBefore());
        Assert.Equal(TopAbs_IN, aTransition.StateAfter());

        aTransition.Reset(D(1.0, 0.0));
        Assert.Equal(TopAbs_UNKNOWN, aTransition.StateBefore());
        Assert.Equal(TopAbs_UNKNOWN, aTransition.StateAfter());

        aTransition.Reset(D(1.0, 0.0), D(0.0, 1.0), 2.0);
        Assert.Equal(TopAbs_UNKNOWN, aTransition.StateBefore());
        Assert.Equal(TopAbs_UNKNOWN, aTransition.StateAfter());
    }

    [Theory]
    [InlineData(TopAbs_FORWARD, TopAbs_OUT, TopAbs_IN)]
    [InlineData(TopAbs_REVERSED, TopAbs_IN, TopAbs_OUT)]
    [InlineData(TopAbs_EXTERNAL, TopAbs_OUT, TopAbs_OUT)]
    public void Line_crossing_the_middle_of_an_edge(TopAbs_Orientation theTrans,
                                                    TopAbs_State theBefore,
                                                    TopAbs_State theAfter)
    {
        // The X axis crosses an edge along Y in its middle (O = INTERNAL)
        var aTransition = new TopTrans_CurveTransition();
        aTransition.Reset(D(1.0, 0.0));
        aTransition.Compare(Tol, D(0.0, 1.0), D(1.0, 0.0), 0.0, theTrans, TopAbs_INTERNAL);
        Assert.Equal(theBefore, aTransition.StateBefore());
        Assert.Equal(theAfter, aTransition.StateAfter());
    }

    [Fact]
    public void Internal_transition_takes_the_orientation()
    {
        // An INTERNAL transition becomes the orientation, reversed when the edge goes against the curve
        var aTransition = new TopTrans_CurveTransition();
        aTransition.Reset(D(1.0, 0.0));
        aTransition.Compare(Tol, D(0.0, 1.0), D(1.0, 0.0), 0.0, TopAbs_INTERNAL, TopAbs_INTERNAL);
        Assert.Equal(TopAbs_IN, aTransition.StateBefore());
        Assert.Equal(TopAbs_IN, aTransition.StateAfter());

        aTransition.Reset(D(1.0, 0.0));
        aTransition.Compare(Tol, D(-1.0, 1.0), D(1.0, 1.0), 0.0, TopAbs_INTERNAL, TopAbs_FORWARD);
        // T * myTgt < 0: S = Reverse(FORWARD) = REVERSED
        Assert.Equal(TopAbs_IN, aTransition.StateBefore());
        Assert.Equal(TopAbs_OUT, aTransition.StateAfter());
    }

    [Fact]
    public void Line_through_a_convex_corner()
    {
        // Square [-1, 1]^2, counter-clockwise. At the corner (-1, -1) the left edge ends (tangent (0, -1),
        // O = REVERSED) and the bottom edge starts (tangent (1, 0), O = FORWARD).

        // Direction (1, 1): enters the square through the corner. Both edges are crossed inwards.
        var aThrough = new TopTrans_CurveTransition();
        aThrough.Reset(D(1.0, 1.0));
        aThrough.Compare(Tol, D(0.0, -1.0), D(1.0, 0.0), 0.0, TopAbs_FORWARD, TopAbs_REVERSED);
        aThrough.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, TopAbs_FORWARD, TopAbs_FORWARD);
        Assert.Equal(TopAbs_OUT, aThrough.StateBefore());
        Assert.Equal(TopAbs_IN, aThrough.StateAfter());

        // Direction (1, -1): only touches the corner. The left edge is crossed inwards, the bottom one outwards;
        // the left edge is nearest before the corner and the bottom one nearest after it.
        var aTouch = new TopTrans_CurveTransition();
        aTouch.Reset(D(1.0, -1.0));
        aTouch.Compare(Tol, D(0.0, -1.0), D(1.0, 0.0), 0.0, TopAbs_FORWARD, TopAbs_REVERSED);
        aTouch.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, TopAbs_REVERSED, TopAbs_FORWARD);
        Assert.Equal(TopAbs_OUT, aTouch.StateBefore());
        Assert.Equal(TopAbs_OUT, aTouch.StateAfter());

        // The same edges given in the other order
        var aTouch2 = new TopTrans_CurveTransition();
        aTouch2.Reset(D(1.0, -1.0));
        aTouch2.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, TopAbs_REVERSED, TopAbs_FORWARD);
        aTouch2.Compare(Tol, D(0.0, -1.0), D(1.0, 0.0), 0.0, TopAbs_FORWARD, TopAbs_REVERSED);
        Assert.Equal(TopAbs_OUT, aTouch2.StateBefore());
        Assert.Equal(TopAbs_OUT, aTouch2.StateAfter());
    }

    [Fact]
    public void Line_through_a_concave_corner()
    {
        // The boundary (0, -1) -> (0, 0) -> (1, 0), matter on its left: everything but the quadrant x > 0, y < 0.
        // At the reflex corner (0, 0) the first edge ends (tangent (0, 1), O = REVERSED) and the second starts
        // (tangent (1, 0), O = FORWARD).

        // Direction (1, -1): from the matter into the missing quadrant. Both edges are crossed outwards.
        var aLeaving = new TopTrans_CurveTransition();
        aLeaving.Reset(D(1.0, -1.0));
        aLeaving.Compare(Tol, D(0.0, 1.0), D(-1.0, 0.0), 0.0, TopAbs_REVERSED, TopAbs_REVERSED);
        aLeaving.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, TopAbs_REVERSED, TopAbs_FORWARD);
        Assert.Equal(TopAbs_IN, aLeaving.StateBefore());
        Assert.Equal(TopAbs_OUT, aLeaving.StateAfter());

        // Direction (1, 1): stays in the matter. The first edge is crossed outwards (x = 0 towards x > 0), the
        // second inwards (y = 0 towards y > 0); the first is nearest before the corner, the second after it.
        var aInside = new TopTrans_CurveTransition();
        aInside.Reset(D(1.0, 1.0));
        aInside.Compare(Tol, D(0.0, 1.0), D(-1.0, 0.0), 0.0, TopAbs_REVERSED, TopAbs_REVERSED);
        aInside.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, TopAbs_FORWARD, TopAbs_FORWARD);
        Assert.Equal(TopAbs_IN, aInside.StateBefore());
        Assert.Equal(TopAbs_IN, aInside.StateAfter());
    }

    [Fact]
    public void Tangent_elements_are_ordered_by_curvature()
    {
        // Two boundary arcs leave the intersection tangent to the straight reference (same angle): the
        // curvatures decide which is nearest. Reference straight (myCurv = 0) and cos > 0: the larger curvature
        // comes first (OCCT's 2000 modification reverses the lower-curvature rule).
        var aTransition = new TopTrans_CurveTransition();
        aTransition.Reset(D(1.0, 0.0));
        aTransition.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 1.0, TopAbs_FORWARD, TopAbs_FORWARD);
        aTransition.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 2.0, TopAbs_REVERSED, TopAbs_FORWARD);
        Assert.Equal(TopAbs_IN, aTransition.StateBefore()); // first: the second arc (REVERSED)
        Assert.Equal(TopAbs_IN, aTransition.StateAfter()); // last: unchanged, the first arc (FORWARD)

        // In the other order the second arc doesn't replace the first
        var aOther = new TopTrans_CurveTransition();
        aOther.Reset(D(1.0, 0.0));
        aOther.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 2.0, TopAbs_REVERSED, TopAbs_FORWARD);
        aOther.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 1.0, TopAbs_FORWARD, TopAbs_FORWARD);
        Assert.Equal(TopAbs_IN, aOther.StateBefore()); // first: still the REVERSED arc
        Assert.Equal(TopAbs_IN, aOther.StateAfter()); // last: the FORWARD arc replaces it
    }

    [Fact]
    public void Curved_reference_compares_curvature_differences()
    {
        // Reference of curvature 1 towards +Y; two tangent boundary arcs. The one whose curvature is nearer
        // below the reference's (deltaC = (C - 1) * (N . myNorm)) comes first, reversed because cos > 0.
        var aTransition = new TopTrans_CurveTransition();
        aTransition.Reset(D(1.0, 0.0), D(0.0, 1.0), 1.0);
        aTransition.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.5, TopAbs_FORWARD, TopAbs_FORWARD);
        aTransition.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 3.0, TopAbs_REVERSED, TopAbs_FORWARD);
        // IsBefore(new (deltaC 2), first (deltaC -0.5)): 2 < -0.5 false, flipped to true: the new arc is first
        Assert.Equal(TopAbs_IN, aTransition.StateBefore());
        Assert.Equal(TopAbs_IN, aTransition.StateAfter());
    }
}
