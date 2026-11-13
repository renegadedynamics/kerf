// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The check of a Boolean's result takes each face's bounds from the adaptor it makes of the face (unit 15AR), where it
/// worked them out a second time with <c>BrepTools.UVBounds</c>: they must be the same bits, for every kind of face and
/// placement the check samples.
/// </summary>
public class BooleanCheckBoundsTests
{
    private static Shape Placed(Shape shape) =>
        shape.Moved(new Placement(new Transform3D().WithRotation(new Axis3D(new Point3D(1, 2, 3), new Direction3D(1, 1, 0.3)), 0.7)
                                                   .WithTranslation(new Vector3D(17, -4, 9))));

    public static TheoryData<string> Shapes() => ["box", "cylinder", "cone", "sphere", "torus", "rounded box", "cut", "fuse"];

    private static Shape Made(string name) => name switch
    {
        "box" => Solid.Box(10, 20, 30).Value.Shape,
        "cylinder" => Solid.Cylinder(4, 9).Value.Shape,
        "cone" => Solid.Cone(5, 2, 7).Value.Shape,
        "sphere" => Solid.Sphere(6).Value.Shape,
        "torus" => Solid.Torus(8, 2).Value.Shape,
        "rounded box" => Rounded(),
        "cut" => Solid.Box(20, 20, 20).Value.Shape.Cut(Solid.Cylinder(4, 30).Value.Shape.Moved(
            new Placement(new Transform3D().WithTranslation(new Vector3D(10, 10, -5))))).Value.Shape,
        _ => Solid.Sphere(10).Value.Shape.Fuse(Solid.Cylinder(3, 20).Value.Shape).Value.Shape,
    };

    private static Shape Rounded()
    {
        Solid box = Solid.Box(20, 20, 20).Value;
        return box.Fillet([.. box.AllEdges()], 3).Value.Shape;
    }

    [Theory]
    [MemberData(nameof(Shapes))]
    public void A_faces_bounds_from_its_adaptor_are_UVBounds_own(string name)
    {
        foreach (Shape shape in new[] { Made(name), Placed(Made(name)) })
        {
            // Each face as the solid holds it, in its orientation and placement
            foreach (Face face in shape.AllFaces())
            {
                BooleanVolumes.Bounded(face, out double fromU, out double toU, out double fromV, out double toV);
                BrepTools.UVBounds(face, out double u0, out double u1, out double v0, out double v1);
                Assert.Equal(BitConverter.DoubleToInt64Bits(u0), BitConverter.DoubleToInt64Bits(fromU));
                Assert.Equal(BitConverter.DoubleToInt64Bits(u1), BitConverter.DoubleToInt64Bits(toU));
                Assert.Equal(BitConverter.DoubleToInt64Bits(v0), BitConverter.DoubleToInt64Bits(fromV));
                Assert.Equal(BitConverter.DoubleToInt64Bits(v1), BitConverter.DoubleToInt64Bits(toV));
            }
        }
    }
}
