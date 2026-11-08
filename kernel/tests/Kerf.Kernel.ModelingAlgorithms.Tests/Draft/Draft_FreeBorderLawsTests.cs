// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features.Api;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Drafts of open shells (unit 12Q, decision 1753): a face with a free border has no neighbour to meet there, so its
/// border is where the drafted face meets the plane or cone the border lay in. A drafted side keeps its place at the
/// neutral plane and moves in by tan(a) per unit above it; a cylinder becomes a cone of the draft's angle.
/// </summary>
public class Draft_FreeBorderLawsTests
{
    private static readonly Direction3D Up = new(0, 0, 1);

    private static List<Face> Faces(in Shape theShape) =>
        BRepOffset_MakeOffsetTests.SubShapes(theShape, ShapeType.Face).Select(f => ShapeCasts.Face(f)).ToList();

    private static List<Point3D> Points(in Shape theShape) =>
        BRepOffset_MakeOffsetTests.SubShapes(theShape, ShapeType.Vertex).Select(v => Tool.Pnt(ShapeCasts.Vertex(v))).ToList();

    private static bool AllAt(in Shape theShape, double theZ) => Points(theShape).All(p => Math.Abs(p.Z - theZ) < 1e-9);

    // The faces of a shape but those dropped, sewn into a shell
    private static Shape Without(in Shape theShape, Func<Face, bool> theDrop)
    {
        var aSew = new Sewing(1.0e-6);
        foreach (Face aFace in Faces(theShape).Where(f => !theDrop(f)))
        {
            aSew.Add(aFace);
        }
        aSew.Perform();
        return aSew.SewedShape();
    }

    private static Shape Drafted(in Shape theShape, IEnumerable<Face> theFaces, double theDegrees, double theNeutral)
    {
        ShapeSnapshot aSnap = ShapeSnapshot.Of(theShape);
        var aDraft = new DraftAngle(theShape);
        foreach (Face aFace in theFaces)
        {
            aDraft.Add(aFace, Up, theDegrees * Math.PI / 180, new Plane3D(new Point3D(0, 0, theNeutral), Up));
            Assert.True(aDraft.AddDone(), aDraft.Status().ToString());
        }
        aDraft.Build();
        Assert.True(aDraft.IsDone, aDraft.Status().ToString());
        aSnap.AssertUnchanged(theShape);
        Shape aResult = aDraft.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        return aResult;
    }

    [Theory]
    [InlineData(5.0)]
    [InlineData(-5.0)]
    public void An_open_box_drafted_on_all_sides_keeps_its_floor_and_narrows_its_rim(double theDegrees)
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Shape anOpen = Without(aBox, f => AllAt(f, 10));
        Shape aResult = Drafted(anOpen, Faces(anOpen).Where(f => !AllAt(f, 0)), theDegrees, 0);
        Assert.Equal(5, Faces(aResult).Count);
        // the floor is on the neutral plane: its corners stay; the rim, a free border, is the square 10 less 2 h tan(a)
        double aIn = 10 * Math.Tan(theDegrees * Math.PI / 180);
        List<Point3D> aPoints = Points(aResult);
        Assert.Equal(8, aPoints.Count);
        foreach (Point3D aP in aPoints)
        {
            if (Math.Abs(aP.Z) < 1e-9)
            {
                Assert.True((Math.Abs(aP.X) < 1e-9 || Math.Abs(aP.X - 10) < 1e-9) && (Math.Abs(aP.Y) < 1e-9 || Math.Abs(aP.Y - 10) < 1e-9), aP.ToString());
            }
            else
            {
                Assert.Equal(10, aP.Z, 1e-9);
                Assert.True(Math.Abs(aP.X - aIn) < 1e-9 || Math.Abs(aP.X - (10 - aIn)) < 1e-9, aP.ToString());
                Assert.True(Math.Abs(aP.Y - aIn) < 1e-9 || Math.Abs(aP.Y - (10 - aIn)) < 1e-9, aP.ToString());
            }
        }
    }

    [Fact]
    public void One_side_of_an_open_box_tilts_and_its_neighbours_follow_it()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Shape anOpen = Without(aBox, f => AllAt(f, 10));
        Face aSide = Faces(anOpen).Single(f => Points(f).All(p => Math.Abs(p.X) < 1e-9));
        Shape aResult = Drafted(anOpen, [aSide], 5, 0);
        // the side at x = 0 now leans in: at the rim it is at x = 10 tan(5 degrees); the others keep their planes
        double aIn = 10 * Math.Tan(5 * Math.PI / 180);
        List<Point3D> aRim = Points(aResult).Where(p => Math.Abs(p.Z - 10) < 1e-9).ToList();
        Assert.Equal(4, aRim.Count);
        Assert.Equal(2, aRim.Count(p => Math.Abs(p.X - aIn) < 1e-9));
        Assert.Equal(2, aRim.Count(p => Math.Abs(p.X - 10) < 1e-9));
        Face aTilted = Faces(aResult).Single(f => Points(f).Any(p => Math.Abs(p.X - aIn) < 1e-9 && Math.Abs(p.Z - 10) < 1e-9)
                                                  && Points(f).All(p => p.X < aIn + 1e-9));
        Direction3D aNormal = ((PlaneSurface)Tool.Surface(aTilted)!).Plane.Axis.Direction;
        Assert.Equal(Math.Sin(5 * Math.PI / 180), Math.Abs(aNormal.Z), 1e-12);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(5.0)]
    public void A_cylinder_with_an_open_end_becomes_a_cone_of_the_angle(double theNeutral)
    {
        // a cup: the cylinder of radius 5 and its floor, open at z = 10
        Shape aCylinder = new ApiMakeCylinder(5, 10).Shape();
        Shape aCup = Without(aCylinder, f => Tool.Surface(f) is PlaneSurface && AllAt(f, 10));
        Shape aResult = Drafted(aCup, Faces(aCup).Where(f => Tool.Surface(f) is CylindricalSurface), 5, theNeutral);
        ConicalSurface aCone = Assert.IsType<ConicalSurface>(Faces(aResult).Select(f => Tool.Surface(f)).Single(s => s is not PlaneSurface));
        Assert.Equal(5 * Math.PI / 180, Math.Abs(aCone.SemiAngle), 1e-12);
        // radius 5 at the neutral plane, smaller by tan(a) per unit above it, larger below
        double Radius(double z) => 5 - ((z - theNeutral) * Math.Tan(5 * Math.PI / 180));
        foreach (Shape anE in BRepOffset_MakeOffsetTests.SubShapes(aResult, ShapeType.Edge))
        {
            if (Tool.Curve(ShapeCasts.Edge(anE), out _, out _) is CircleCurve3D aCircle)
            {
                Assert.Equal(Radius(aCircle.Circle.Location.Z), aCircle.Radius, 1e-9);
            }
        }
        Assert.Equal(2, BRepOffset_MakeOffsetTests.SubShapes(aResult, ShapeType.Edge).Count(e => Tool.Curve(ShapeCasts.Edge(e), out _, out _) is CircleCurve3D));
    }

    [Fact]
    public void A_tube_open_at_both_ends_becomes_a_cone()
    {
        Shape aCylinder = new ApiMakeCylinder(5, 10).Shape();
        Shape aTube = Without(aCylinder, f => Tool.Surface(f) is PlaneSurface);
        Shape aResult = Drafted(aTube, Faces(aTube), 5, 0);
        ConicalSurface aCone = Assert.IsType<ConicalSurface>(Tool.Surface(Faces(aResult).Single()));
        Assert.Equal(5, aCone.RefRadius, 1e-9);
        Assert.Equal(0, aCone.Position.Location.Z, 1e-9);
        Assert.Equal(5 * Math.PI / 180, Math.Abs(aCone.SemiAngle), 1e-12);
    }

    [Fact]
    public void A_cone_with_an_open_end_is_redrafted_from_its_foot()
    {
        // a cone of radius 5 at its foot and 2 at its top, open there: drafted at 5 degrees about its foot
        Shape aCone = new ApiMakeCone(5, 2, 10).Shape();
        Shape aCup = Without(aCone, f => Tool.Surface(f) is PlaneSurface && AllAt(f, 10));
        Shape aResult = Drafted(aCup, Faces(aCup).Where(f => Tool.Surface(f) is ConicalSurface), 5, 0);
        ConicalSurface aNew = Assert.IsType<ConicalSurface>(Faces(aResult).Select(f => Tool.Surface(f)).Single(s => s is not PlaneSurface));
        Assert.Equal(5 * Math.PI / 180, Math.Abs(aNew.SemiAngle), 1e-12);
        Assert.Equal(5, aNew.RefRadius, 1e-9);
        Assert.Equal(0, aNew.Position.Location.Z, 1e-9);
    }
}
