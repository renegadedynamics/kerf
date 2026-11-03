// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/TopTrans_SurfaceTransition_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests;

public class TopTrans_SurfaceTransition_Test
{
    [Fact]
    public void UndefinedStateStaysInstanceLocal()
    {
        var aReferenceTransition = new TopTrans_SurfaceTransition();
        aReferenceTransition.Reset(new Direction3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        aReferenceTransition.Compare(global::Kerf.Engine.Foundation.Precision.Angular(), new Direction3D(0.0, 0.0, 1.0), Forward, Forward);

        Containment aBefore = aReferenceTransition.StateBefore();
        Containment anAfter = aReferenceTransition.StateAfter();
        Assert.NotEqual(Unknown, aBefore);
        Assert.NotEqual(Unknown, anAfter);

        var anInvalidTransition = new TopTrans_SurfaceTransition();
        anInvalidTransition.Reset(new Direction3D(1.0, 0.0, 0.0),
                                  new Direction3D(0.0, 0.0, 1.0),
                                  new Direction3D(1.0, 0.0, 0.0),
                                  new Direction3D(1.0, 0.0, 0.0),
                                  1.0,
                                  1.0);

        Assert.Equal(Unknown, anInvalidTransition.StateBefore());
        Assert.Equal(Unknown, anInvalidTransition.StateAfter());
        Assert.Equal(aBefore, aReferenceTransition.StateBefore());
        Assert.Equal(anAfter, aReferenceTransition.StateAfter());
    }

    [Fact]
    public void CompareUsesProvidedTolerance()
    {
        var aTransition = new TopTrans_SurfaceTransition();
        aTransition.Reset(new Direction3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        aTransition.Compare(1.0,
                            new Direction3D(0.0, 0.0, 1.0),
                            new Direction3D(0.6, 0.8, 0.0),
                            new Direction3D(0.6, -0.8, 0.0),
                            1.0,
                            0.5,
                            Forward,
                            Forward);

        Assert.NotEqual(Unknown, aTransition.StateBefore());
        Assert.NotEqual(Unknown, aTransition.StateAfter());
    }
}

/// <summary>Known configurations of TopTrans_SurfaceTransition, beyond OCCT's GTests.</summary>
public class TopTrans_SurfaceTransition_PortTests
{
    [Fact]
    public void Empty_transition_is_unknown()
    {
        var aTransition = new TopTrans_SurfaceTransition();
        Assert.Equal(Unknown, aTransition.StateBefore());
        Assert.Equal(Unknown, aTransition.StateAfter());

        // Compare before Reset is ignored
        aTransition.Compare(0.0, new Direction3D(0.0, 0.0, 1.0), Forward, Forward);
        Assert.Equal(Unknown, aTransition.StateBefore());
    }

    [Fact]
    public void Plane_face_gives_the_states_of_its_orientation()
    {
        // Reference: the plane z = 0 crossed along X. The face's dironF is +Y, at angle 0 from beafter (+Y):
        // only the "after" row is filled, and the "before" state is read from it.
        var aForward = new TopTrans_SurfaceTransition();
        aForward.Reset(new Direction3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        aForward.Compare(0.0, new Direction3D(0.0, 0.0, 1.0), Forward, Forward);
        Assert.Equal(Out, aForward.StateBefore());
        Assert.Equal(In, aForward.StateAfter());

        var aReversed = new TopTrans_SurfaceTransition();
        aReversed.Reset(new Direction3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        aReversed.Compare(0.0, new Direction3D(0.0, 0.0, 1.0), Reversed, Forward);
        Assert.Equal(In, aReversed.StateBefore());
        Assert.Equal(Out, aReversed.StateAfter());

        // Reset forgets the previous faces
        aReversed.Reset(new Direction3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Assert.Equal(Unknown, aReversed.StateBefore());
        Assert.Equal(Unknown, aReversed.StateAfter());
    }

    [Fact]
    public void Tangent_faces_with_the_same_orientation_are_undecided()
    {
        // Two coincident faces with the same orientation: FUN_refnearest can't choose (OCCT's "nyi FUN_RAISE").
        var aTransition = new TopTrans_SurfaceTransition();
        aTransition.Reset(new Direction3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        aTransition.Compare(0.0, new Direction3D(0.0, 0.0, 1.0), Forward, Forward);
        aTransition.Compare(0.0, new Direction3D(0.0, 0.0, 1.0), Forward, Forward);
        Assert.Equal(Unknown, aTransition.StateBefore());
        Assert.Equal(Unknown, aTransition.StateAfter());
    }

    [Fact]
    public void GetBefore_GetAfter()
    {
        Assert.Equal(Out, TopTrans_SurfaceTransition.GetBefore(Forward));
        Assert.Equal(In, TopTrans_SurfaceTransition.GetBefore(Reversed));
        Assert.Equal(In, TopTrans_SurfaceTransition.GetBefore(Internal));
        Assert.Equal(Out, TopTrans_SurfaceTransition.GetBefore(External));
        Assert.Equal(In, TopTrans_SurfaceTransition.GetAfter(Forward));
        Assert.Equal(Out, TopTrans_SurfaceTransition.GetAfter(Reversed));
        Assert.Equal(In, TopTrans_SurfaceTransition.GetAfter(Internal));
        Assert.Equal(Out, TopTrans_SurfaceTransition.GetAfter(External));
    }
}
