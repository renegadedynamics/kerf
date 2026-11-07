// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A volume alone takes a planar face's part in closed form (<see cref="PlanarFaceMass"/>): it must be what the integration
/// gives, face by face and for the whole, wherever the shape is and however it's held together.
/// </summary>
public class PlanarFaceMassTests
{
    private const double Eps = 1e-6;

    private static double Exact(in Shape shape, bool onlyClosed = false, bool skipShared = false)
    {
        var props = new Properties();
        MassPropertiesBuilder.VolumeProperties(shape, props, Eps, onlyClosed, skipShared, massOnly: true);
        return props.Mass();
    }

    private static double Integrated(in Shape shape, bool onlyClosed = false, bool skipShared = false)
    {
        var props = new Properties();
        MassPropertiesBuilder.VolumeProperties(shape, props, Eps, onlyClosed, skipShared);
        return props.Mass();
    }

    private static void AssertSame(in Shape shape, double expected)
    {
        Assert.Equal(expected, Exact(shape), Math.Abs(expected) * 1e-12);
        Assert.Equal(Integrated(shape), Exact(shape), Math.Max(Math.Abs(expected), 1.0) * 1e-12);
    }

    [Fact]
    public void A_box_alone_and_placed()
    {
        Shape box = TestShapes.Frozen(TestShapes.Box(3, 5, 7));
        AssertSame(box, 105.0);
        AssertSame(box.Reversed(), -105.0);

        var turned = Transform3D.Rotation(new Axis3D(new Point3D(10, -20, 30), new Direction3D(1, 2, 3)), 0.7);
        AssertSame(box.Moved(new Placement(turned)), 105.0);
    }

    [Fact]
    public void A_scaled_placement_scales_the_volume()
    {
        Shape box = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        AssertSame(box.Moved(new Placement(Transform3D.Scale(new Point3D(1, 1, 1), 2.0))), 48.0);
    }

    [Fact]
    public void Shared_solids_and_what_is_skipped()
    {
        Shape box = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Shape moved = box.Moved(new Placement(Transform3D.Translation(new Vector3D(10, 0, 0))));
        Shape both = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(box, moved));
        AssertSame(both, 12.0);
        Shape cancelled = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(box, box.Reversed()));
        Assert.Equal(0.0, Exact(cancelled), 1e-12);
        Shape twice = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(box, box));
        Assert.Equal(Integrated(twice, skipShared: true), Exact(twice, skipShared: true), 1e-12);
        Assert.Equal(Integrated(twice), Exact(twice), 1e-12);
    }

    [Fact]
    public void A_cylinder_mixes_exact_discs_with_an_integrated_side()
    {
        Shape cylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2, 5));
        AssertSame(cylinder, Math.PI * 4 * 5);
    }

    [Theory]
    [InlineData(1.0, -2.0, 0.5, false)]
    [InlineData(1.0, -2.0, 0.5, true)]
    [InlineData(-30.0, 4.0, 2.0, false)]
    public void A_face_alone_counts_from_the_reference_point_as_the_integration_does(double x, double y, double z, bool reversed)
    {
        Face face = BRepGPropTestShapes.Faces(TestShapes.Frozen(TestShapes.Box(4, 6, 2)))[1];
        Shape placed = face.Moved(new Placement(Transform3D.Translation(new Vector3D(x, y, z))));
        placed = reversed ? placed.Reversed() : placed;
        // The reference is where the shape is held, so a face off it bounds a pyramid from there
        Assert.Equal(Integrated(placed), Exact(placed), 1e-11);
    }

    [Fact]
    public void Areas_are_the_integrations_whichever_way_a_face_is_turned()
    {
        static double Area(in Shape shape, bool massOnly)
        {
            var props = new Properties();
            MassPropertiesBuilder.SurfaceProperties(shape, props, Eps, massOnly: massOnly);
            return props.Mass();
        }

        Shape box = TestShapes.Frozen(TestShapes.Box(3, 5, 7));
        Assert.Equal(142.0, Area(box, true), 1e-10);
        Assert.Equal(142.0, Area(box.Reversed(), true), 1e-10);
        Shape scaled = box.Moved(new Placement(Transform3D.Scale(new Point3D(1, 1, 1), 2.0)));
        Assert.Equal(568.0, Area(scaled, true), 1e-10);
        Assert.Equal(Area(scaled, false), Area(scaled, true), 1e-10);
        Shape cylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2, 5));
        Assert.Equal(Area(cylinder, false), Area(cylinder, true), 1e-9);
    }

    private static Point3D IntegratedCentre(in Shape shape)
    {
        var props = new Properties();
        MassPropertiesBuilder.SurfaceProperties(shape, props, 1e-9);
        return props.CentreOfMass();
    }

    private static void AssertCentre(in Face face, Point3D expected, double tolerance = 1e-11)
    {
        Assert.True(PlanarFaceMass.TryCentre(face, new Domain(), out Point3D centre));
        Assert.True(new Vector3D(centre, expected).Magnitude <= tolerance, $"{centre.X} {centre.Y} {centre.Z} against {expected.X} {expected.Y} {expected.Z}");
        Point3D integrated = IntegratedCentre(face);
        Assert.True(new Vector3D(centre, integrated).Magnitude <= 1e-7, $"{centre.X} {centre.Y} {centre.Z} against the integration's {integrated.X} {integrated.Y} {integrated.Z}");
    }

    [Fact]
    public void A_boxes_faces_have_their_centres_wherever_they_are_and_whichever_way_turned()
    {
        Shape box = TestShapes.Frozen(TestShapes.Box(4, 6, 2));
        var turned = Transform3D.Rotation(new Axis3D(new Point3D(10, -20, 30), new Direction3D(1, 2, 3)), 0.7);
        var far = Transform3D.Translation(new Vector3D(1000, -2000, 500));
        var scaled = Transform3D.Scale(new Point3D(1, 1, 1), 2.0);
        foreach (Face face in BRepGPropTestShapes.Faces(box))
        {
            Point3D centre = IntegratedCentre(face);
            AssertCentre(face, centre, 1e-7);
            AssertCentre(ShapeCasts.Face(face.Reversed()), centre, 1e-7);
            AssertCentre(ShapeCasts.Face(face.Moved(new Placement(turned))), centre.Transformed(turned), 1e-7);
            AssertCentre(ShapeCasts.Face(face.Moved(new Placement(far))), centre.Transformed(far), 1e-7);
            AssertCentre(ShapeCasts.Face(face.Moved(new Placement(scaled))), centre.Transformed(scaled), 1e-7);
        }
    }

    [Fact]
    public void A_cylinders_discs_are_at_its_axis()
    {
        Shape cylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2, 5));
        var discs = BRepGPropTestShapes.Faces(cylinder).Where(f => Tool.Surface(f) is PlaneSurface).ToList();
        Assert.Equal(2, discs.Count);
        foreach (Face disc in discs)
        {
            Assert.True(PlanarFaceMass.TryCentre(disc, new Domain(), out Point3D centre));
            Assert.Equal(0.0, centre.X, 1e-14);
            Assert.Equal(0.0, centre.Y, 1e-14);
        }
    }

    [Fact]
    public void A_face_of_another_kind_or_an_open_boundary_has_no_exact_centre()
    {
        Shape cylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2, 5));
        Face side = BRepGPropTestShapes.Faces(cylinder).First(f => Tool.Surface(f) is not PlaneSurface);
        Assert.False(PlanarFaceMass.TryCentre(side, new Domain(), out _));
    }

    [Fact]
    public void A_planar_face_is_answered()
    {
        foreach (Face face in BRepGPropTestShapes.Faces(TestShapes.Frozen(TestShapes.Box(4, 6, 2))))
        {
            Assert.True(PlanarFaceMass.TryVolume(face, new Domain(), new Point3D(0, 0, 0), out _));
        }

        Shape cylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2, 5));
        Assert.Equal(2, BRepGPropTestShapes.Faces(cylinder).Count(f => PlanarFaceMass.TryVolume(f, new Domain(), new Point3D(0, 0, 0), out _)));
    }

    [Fact]
    public void A_face_of_another_kind_is_not_answered()
    {
        Shape cylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2, 5));
        Face side = BRepGPropTestShapes.Faces(cylinder).First(f => Tool.Surface(f) is not PlaneSurface);
        Assert.False(PlanarFaceMass.TryVolume(side, new Domain(), new Point3D(0, 0, 0), out _));
    }

    [Fact]
    public void An_open_boundary_is_left_to_the_integration()
    {
        var builder = new BrepBuilder();
        var plane = new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face face = new();
        builder.MakeFace(ref face, plane, 1e-7);
        (double X, double Y)[] corners = [(0, 0), (10, 0), (10, 10)];
        Wire wire = new();
        builder.MakeWire(ref wire);
        for (int i = 0; i + 1 < corners.Length; i++)
        {
            var (a, b) = (corners[i], corners[i + 1]);
            Edge edge = TestShapes.Edge(TestShapes.Vertex(new Point3D(a.X, a.Y, 0)), TestShapes.Vertex(new Point3D(b.X, b.Y, 0)));
            builder.UpdateEdge(edge, new LineCurve2D(new Point2D(a.X, a.Y), new Direction2D(b.X - a.X, b.Y - a.Y)), face, 1e-7);
            builder.Add(wire, edge);
        }

        builder.Add(face, wire);
        Assert.False(PlanarFaceMass.TryVolume(TestShapes.Frozen(face), new Domain(), new Point3D(0, 0, -1), out _));
    }
}
