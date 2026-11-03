// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

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
        Assert.Equal(Out, aTransition.StateBefore());
        Assert.Equal(In, aTransition.StateAfter());

        aTransition.Reset(D(1.0, 0.0));
        Assert.Equal(Unknown, aTransition.StateBefore());
        Assert.Equal(Unknown, aTransition.StateAfter());

        aTransition.Reset(D(1.0, 0.0), D(0.0, 1.0), 2.0);
        Assert.Equal(Unknown, aTransition.StateBefore());
        Assert.Equal(Unknown, aTransition.StateAfter());
    }

    [Theory]
    [InlineData(Forward, Out, In)]
    [InlineData(Reversed, In, Out)]
    [InlineData(External, Out, Out)]
    public void Line_crossing_the_middle_of_an_edge(Orientation theTrans,
                                                    Containment theBefore,
                                                    Containment theAfter)
    {
        // The X axis crosses an edge along Y in its middle (O = INTERNAL)
        var aTransition = new TopTrans_CurveTransition();
        aTransition.Reset(D(1.0, 0.0));
        aTransition.Compare(Tol, D(0.0, 1.0), D(1.0, 0.0), 0.0, theTrans, Internal);
        Assert.Equal(theBefore, aTransition.StateBefore());
        Assert.Equal(theAfter, aTransition.StateAfter());
    }

    [Fact]
    public void Internal_transition_takes_the_orientation()
    {
        // An INTERNAL transition becomes the orientation, reversed when the edge goes against the curve
        var aTransition = new TopTrans_CurveTransition();
        aTransition.Reset(D(1.0, 0.0));
        aTransition.Compare(Tol, D(0.0, 1.0), D(1.0, 0.0), 0.0, Internal, Internal);
        Assert.Equal(In, aTransition.StateBefore());
        Assert.Equal(In, aTransition.StateAfter());

        aTransition.Reset(D(1.0, 0.0));
        aTransition.Compare(Tol, D(-1.0, 1.0), D(1.0, 1.0), 0.0, Internal, Forward);
        // T * myTgt < 0: S = Reverse(FORWARD) = REVERSED
        Assert.Equal(In, aTransition.StateBefore());
        Assert.Equal(Out, aTransition.StateAfter());
    }

    [Fact]
    public void Line_through_a_convex_corner()
    {
        // Square [-1, 1]^2, counter-clockwise. At the corner (-1, -1) the left edge ends (tangent (0, -1),
        // O = REVERSED) and the bottom edge starts (tangent (1, 0), O = FORWARD).

        // Direction (1, 1): enters the square through the corner. Both edges are crossed inwards.
        var aThrough = new TopTrans_CurveTransition();
        aThrough.Reset(D(1.0, 1.0));
        aThrough.Compare(Tol, D(0.0, -1.0), D(1.0, 0.0), 0.0, Forward, Reversed);
        aThrough.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, Forward, Forward);
        Assert.Equal(Out, aThrough.StateBefore());
        Assert.Equal(In, aThrough.StateAfter());

        // Direction (1, -1): only touches the corner. The left edge is crossed inwards, the bottom one outwards;
        // the left edge is nearest before the corner and the bottom one nearest after it.
        var aTouch = new TopTrans_CurveTransition();
        aTouch.Reset(D(1.0, -1.0));
        aTouch.Compare(Tol, D(0.0, -1.0), D(1.0, 0.0), 0.0, Forward, Reversed);
        aTouch.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, Reversed, Forward);
        Assert.Equal(Out, aTouch.StateBefore());
        Assert.Equal(Out, aTouch.StateAfter());

        // The same edges given in the other order
        var aTouch2 = new TopTrans_CurveTransition();
        aTouch2.Reset(D(1.0, -1.0));
        aTouch2.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, Reversed, Forward);
        aTouch2.Compare(Tol, D(0.0, -1.0), D(1.0, 0.0), 0.0, Forward, Reversed);
        Assert.Equal(Out, aTouch2.StateBefore());
        Assert.Equal(Out, aTouch2.StateAfter());
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
        aLeaving.Compare(Tol, D(0.0, 1.0), D(-1.0, 0.0), 0.0, Reversed, Reversed);
        aLeaving.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, Reversed, Forward);
        Assert.Equal(In, aLeaving.StateBefore());
        Assert.Equal(Out, aLeaving.StateAfter());

        // Direction (1, 1): stays in the matter. The first edge is crossed outwards (x = 0 towards x > 0), the
        // second inwards (y = 0 towards y > 0); the first is nearest before the corner, the second after it.
        var aInside = new TopTrans_CurveTransition();
        aInside.Reset(D(1.0, 1.0));
        aInside.Compare(Tol, D(0.0, 1.0), D(-1.0, 0.0), 0.0, Reversed, Reversed);
        aInside.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.0, Forward, Forward);
        Assert.Equal(In, aInside.StateBefore());
        Assert.Equal(In, aInside.StateAfter());
    }

    [Fact]
    public void Tangent_elements_are_ordered_by_curvature()
    {
        // Two boundary arcs leave the intersection tangent to the straight reference (same angle): the
        // curvatures decide which is nearest. Reference straight (myCurv = 0) and cos > 0: the larger curvature
        // comes first (OCCT's 2000 modification reverses the lower-curvature rule).
        var aTransition = new TopTrans_CurveTransition();
        aTransition.Reset(D(1.0, 0.0));
        aTransition.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 1.0, Forward, Forward);
        aTransition.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 2.0, Reversed, Forward);
        Assert.Equal(In, aTransition.StateBefore()); // first: the second arc (REVERSED)
        Assert.Equal(In, aTransition.StateAfter()); // last: unchanged, the first arc (FORWARD)

        // In the other order the second arc doesn't replace the first
        var aOther = new TopTrans_CurveTransition();
        aOther.Reset(D(1.0, 0.0));
        aOther.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 2.0, Reversed, Forward);
        aOther.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 1.0, Forward, Forward);
        Assert.Equal(In, aOther.StateBefore()); // first: still the REVERSED arc
        Assert.Equal(In, aOther.StateAfter()); // last: the FORWARD arc replaces it
    }

    [Fact]
    public void Curved_reference_compares_curvature_differences()
    {
        // Reference of curvature 1 towards +Y; two tangent boundary arcs. The one whose curvature is nearer
        // below the reference's (deltaC = (C - 1) * (N . myNorm)) comes first, reversed because cos > 0.
        var aTransition = new TopTrans_CurveTransition();
        aTransition.Reset(D(1.0, 0.0), D(0.0, 1.0), 1.0);
        aTransition.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 0.5, Forward, Forward);
        aTransition.Compare(Tol, D(1.0, 0.0), D(0.0, 1.0), 3.0, Reversed, Forward);
        // IsBefore(new (deltaC 2), first (deltaC -0.5)): 2 < -0.5 false, flipped to true: the new arc is first
        Assert.Equal(In, aTransition.StateBefore());
        Assert.Equal(In, aTransition.StateAfter());
    }
}
