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
    // (decision 35003), and the distance is the shells', 6.480287, as the issue expects.
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
}
