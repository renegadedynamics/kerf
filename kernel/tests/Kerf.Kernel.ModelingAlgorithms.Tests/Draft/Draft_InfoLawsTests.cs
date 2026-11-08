// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features.Api;
using Kerf.Engine.Features.Draft;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// What the draft keeps for each face and edge, and the angle of a face to the direction, by what they must be
/// (decision 1753): a face's angle is the angle of its outward normal, whichever way its surface was made, and a failed
/// face blocks the draft until it's removed.
/// </summary>
public class Draft_InfoLawsTests
{
    private static readonly Direction3D Up = new(0, 0, 1);
    private static readonly Plane3D Floor = new(new Point3D(0, 0, 0), Up);

    private static IEnumerable<Face> Faces(Shape theShape)
    {
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            yield return ShapeCasts.Face(anExp.Current());
        }
    }

    private static Face Lateral(in Shape theShape) => Faces(theShape).First(f => Tool.Surface(f) is not PlaneSurface);

    private static Face Bare(Surface theSurface, double theTolerance = 1.0e-7)
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, theSurface, theTolerance);
        return aF;
    }

    [Fact]
    public void An_edge_keeps_its_first_two_faces_their_largest_tolerance_and_where_it_is_tangent()
    {
        var anInfo = new DraftEdgeInfo();
        Assert.False(anInfo.NewGeometry());
        Assert.False(anInfo.IsTangent(out _));
        Assert.Equal(0, anInfo.Tolerance());
        Assert.True(anInfo.FirstFace().IsNull);

        Face[] aBox = [.. Faces(new MakeBox(10, 10, 10).Shape())];
        Face aLoose = Bare(new PlaneSurface(new Point3D(0, 0, 0), Up), 1.0e-3);
        anInfo.Add(aBox[0]);
        anInfo.Add(aBox[0]); // the same face again is not a second
        Assert.True(anInfo.SecondFace().IsNull);
        anInfo.Add(aLoose);
        anInfo.Add(aBox[1]); // a third is not kept
        Assert.True(anInfo.FirstFace().IsSame(aBox[0]));
        Assert.True(anInfo.SecondFace().IsSame(aLoose));
        Assert.Equal(1.0e-3, anInfo.Tolerance(), 15);

        anInfo.Tangent(new Point3D(1, 2, 3));
        Assert.True(anInfo.IsTangent(out Point3D aPoint));
        Assert.Equal(0, aPoint.DistanceTo(new Point3D(1, 2, 3)), 15);
        Assert.True(new DraftEdgeInfo(true).NewGeometry());
    }

    [Fact]
    public void A_face_keeps_its_first_two_neighbours_and_the_basis_of_a_trimmed_surface()
    {
        var anInfo = new DraftFaceInfo();
        Assert.False(anInfo.NewGeometry());
        Assert.Null(anInfo.Geometry());
        Face[] aBox = [.. Faces(new MakeBox(10, 10, 10).Shape())];
        anInfo.Add(aBox[0]);
        anInfo.Add(aBox[1]);
        anInfo.Add(aBox[2]);
        Assert.True(anInfo.FirstFace().IsSame(aBox[0]));
        Assert.True(anInfo.SecondFace().IsSame(aBox[1]));

        var aPlane = new PlaneSurface(new Point3D(0, 0, 0), Up);
        var aTrimmed = new DraftFaceInfo(new TrimmedSurface(aPlane, 0, 1, 0, 1), true);
        Assert.True(aTrimmed.NewGeometry());
        Assert.IsType<PlaneSurface>(aTrimmed.Geometry()); // the trimmed surface's copy of its basis, as OCCT keeps
        Assert.True(((PlaneSurface)aTrimmed.Geometry()!).Plane.Axis.Direction.IsEqual(Up, 1.0e-15));
    }

    [Fact]
    public void A_face_is_at_the_angle_of_its_outward_normal_however_its_plane_was_made()
    {
        // a plane facing up is at a right angle to Up; its face reversed, or made on a left-handed frame (whose normal is
        // down), at minus that; on a trimmed plane, as on the plane
        var aDirect = new CoordinateSystem(new Point3D(0, 0, 0), Up, new Direction3D(1, 0, 0));
        Face anUp = Bare(new PlaneSurface(aDirect));
        Assert.Equal(Math.PI / 2, DraftTools.Angle(anUp, Up), 12);
        Assert.Equal(-Math.PI / 2, DraftTools.Angle(ShapeCasts.Face(anUp.Reversed()), Up), 12);
        Face aLeft = Bare(new PlaneSurface(aDirect.YReversed()));
        Assert.Equal(-Math.PI / 2, DraftTools.Angle(aLeft, Up), 12);
        Face aTrimmed = Bare(new TrimmedSurface(new PlaneSurface(aDirect), 0, 1, 0, 1));
        Assert.Equal(Math.PI / 2, DraftTools.Angle(aTrimmed, Up), 12);

        // a slanting plane, at the angle its normal makes with the floor
        double a = 0.3;
        Face aSlant = Bare(new PlaneSurface(new Point3D(0, 0, 0), new Direction3D(Math.Cos(a), 0, Math.Sin(a))));
        Assert.Equal(a, DraftTools.Angle(aSlant, Up), 12);

        // a cone's side, reversed, leans the other way by its half angle, atan(3/10)
        Face aCone = Lateral(new ApiMakeCone(5, 2, 10).Shape());
        double anAngle = DraftTools.Angle(aCone, Up);
        Assert.Equal(Math.Atan(0.3), Math.Abs(anAngle), 9);
        Assert.Equal(-anAngle, DraftTools.Angle(ShapeCasts.Face(aCone.Reversed()), Up), 12);
    }

    [Fact]
    public void A_face_that_failed_blocks_the_draft_until_it_is_cleared()
    {
        // a box's side is drafted, then a sphere's face beside it can't be: until the draft is cleared, the faces drafted
        // and their neighbours aren't known, and the sphere's face, never taken, can't be removed (as in OCCT)
        var aB = new BrepBuilder();
        var aBoth = new Compound();
        aB.MakeCompound(ref aBoth);
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Shape aSphere = new MakeSphere(new Point3D(30, 0, 0), 5).Shape();
        aB.Add(aBoth, aBox);
        aB.Add(aBoth, aSphere);
        Face aSide = Faces(aBox).First(f => Tool.Surface(f) is PlaneSurface aP && Math.Abs(aP.Plane.Axis.Direction.X) > 0.5);
        Face aRound = Lateral(aSphere);
        var aDraft = new DraftAngle(aBoth);
        aDraft.Add(aSide, Up, 0.05, Floor);
        Assert.True(aDraft.AddDone());
        aDraft.Add(aRound, Up, 0.05, Floor);
        Assert.False(aDraft.AddDone());
        Assert.True(aDraft.ProblematicShape().IsSame(aRound));
        Assert.Throws<NotDoneException>(() => aDraft.ModifiedFaces());
        Assert.Throws<NotDoneException>(() => aDraft.ConnectedFaces(aSide));
        Assert.Throws<NoSuchObjectException>(() => aDraft.Remove(aRound));
        aDraft.Clear();
        aDraft.Init(aBoth);
        Assert.True(aDraft.AddDone());
        Assert.Empty(aDraft.ModifiedFaces());
    }
}
