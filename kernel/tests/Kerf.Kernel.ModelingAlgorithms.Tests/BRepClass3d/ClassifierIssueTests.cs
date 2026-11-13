// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Exchange.Brep.Binary;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepClass3d;

/// <summary>Points that OCCT's issues report the solid classifier puts on the wrong side, and Kerf now classifies right.</summary>
public class ClassifierIssueTests
{
    private static Shape Read(string name)
    {
        using Stream stream = typeof(ClassifierIssueTests).Assembly.GetManifestResourceStream($"BRepClass3d.Issues.{name}.bbrep")
                              ?? throw new InvalidOperationException(name);
        BinTools.Read(out Shape shape, stream);
        return shape;
    }

    // OCCT #1700's "cheek", a valid solid of 31 faces, and a cube of 1 whose corner P lies 6.4 below the cheek's box. The
    // classifier's ray from P meets a B-spline face by its bound, where the face's intersector misses the crossing, so P read
    // IN: the distance was 0, an inner solution and a success. A point outside a finite solid's box is now OUT
    // (decision 2447), and the distance is the shells', 6.480287, as the issue expects.
    [Fact]
    public void Issue_1700_a_point_outside_the_solids_box_is_outside_it_and_the_distance_is_right()
    {
        Solid cheek = Read("1700-cheek").AllSolids().Single();
        Point3D p = new(142.98277910146606, -1.303540193564, 8.904633106342246);
        Assert.True(cheek.BoundingBox().Min.Y > p.Y + 6);

        Assert.Equal(Containment.Out, cheek.Classify(p, 1e-7));
        Assert.Equal(Containment.Out, cheek.Classify(p, 1e-3));

        Solid cube = Primitives.Box(new Point3D(p.X - 1, p.Y - 1, p.Z - 1), p).Value.Shape;
        foreach (bool parallel in new[] { false, true })
        {
            DistanceResult distance = cube.Distance(cheek, new DistanceOptions { Parallel = parallel }).Value;
            Assert.False(distance.IsInnerSolution);
            Assert.Equal(6.480287, distance.Value, 6);
        }
    }

    // A point outside the box of a hole in space, a solid turned inside out, is still IN it
    [Fact]
    public void A_point_outside_the_box_of_a_solid_turned_inside_out_is_in_it()
    {
        Solid box = Primitives.Box(1, 1, 1).Value.Shape;
        Shape reversed = ((Shape)box).Reversed();
        Assert.Equal(Containment.In, reversed.ClassifyPoint(new Point3D(5, 5, 5)));
        Assert.Equal(Containment.Out, reversed.ClassifyPoint(new Point3D(0.5, 0.5, 0.5)));
        Assert.Equal(Containment.Out, box.ClassifyPoint(new Point3D(5, 5, 5)));
    }

    // A box placed with a scale, which the public layer no longer gives (decision 2452) but the kernel's own code and
    // Draw-lite can: points by its corners inside and outside, and far outside, against the box's own geometry. The ray
    // read a point inside by a corner OUT, as natively
    [Theory]
    [InlineData(1.7, false)]
    [InlineData(1.7, true)]
    [InlineData(0.4, true)]
    public void A_solid_placed_with_a_scale_classifies_as_its_geometry(double factor, bool rotate)
    {
        Shape box = Primitives.Box(3, 4, 5).Value.Shape;
        Transform3D t = Transform3D.Scale(new Point3D(2, -3, 1), factor);
        if (rotate)
        {
            t = t.Multiplied(Transform3D.Identity.WithRotation(new Axis3D(new Point3D(1, 2, 3), new Direction3D(1, 1, 0)), 0.7));
        }

        KernelAssert.Throws<ArgumentException>(() => box.Moved(new Placement(t)));
        KernelAssert.Throws<ArgumentException>(() => box.Located(new Placement(t)));
        Shape placed = box;
        placed.Move(new Placement(t));
        foreach ((Point3D local, Containment expected) in new (Point3D, Containment)[]
                 {
                     (new(2.9, 3.9, 4.9), Containment.In), (new(0.1, 0.1, 0.1), Containment.In), (new(1.5, 2, 2.5), Containment.In),
                     (new(3.1, 3.9, 4.9), Containment.Out), (new(-0.1, 2, 2.5), Containment.Out), (new(30, 40, 50), Containment.Out),
                 })
        {
            Assert.Equal(expected, placed.ClassifyPoint(t * local));
        }
    }

    // OCCT #1703's sleeve with a helical thread and 24 teeth, its file's location records rewritten to 8.0.1's layout (they
    // held a scale and a form before each matrix). The classifier's ray from the issue's point, 0.787 outside the solid
    // behind a collar, enters a thread flank, a B-spline with two rows of poles over 32 spans, between the facets of its
    // intersector's polyhedron: the point read IN. The flank's samples now count its boundary rows, and a long face that
    // asks for more than 20 gets a cap of 60 (decision 2453): OUT
    [Fact]
    public void Issue_1703_a_point_outside_a_helical_thread_is_outside_it()
    {
        Solid sleeve = Read("1703-teeth").AllSolids().Single();
        Point3D p = new(106.667, 6.60084, 0.465566);
        Assert.Equal(Containment.Out, sleeve.Classify(p, 1e-6));
        Assert.Equal(Containment.Out, sleeve.Classify(p, 1e-7));
        Assert.True(Vertex.At(p).Distance(sleeve).Value.Value > 0.5);
    }
}
