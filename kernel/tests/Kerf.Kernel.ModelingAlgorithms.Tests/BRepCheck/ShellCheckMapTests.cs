// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Healing.Checks.Full;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A shell's check keeps the map of its edges to its faces from its first walk for its closure and orientation checks when
/// every face and edge is oriented, and builds it again, leaving the unoriented out, when one isn't (unit 15B,
/// <c>ShellCheck</c>). Either way it must find what the check as it was finds (tests/Shared/FullCheck), on every sub-shape
/// in every context: here on shells whole and not, closed and open, and with a face or an edge the second walk leaves out.
/// </summary>
public class ShellCheckMapTests
{
    private static void Same(in Shape shape)
    {
        foreach (bool parallel in new[] { false, true })
        {
            string? difference = CheckComparison.Compare(shape, true, parallel);
            Assert.True(difference is null, $"parallel {parallel}: {difference}");
        }
    }

    private static Shape Box(double x, double y, double z) => Solid.Box(x, y, z).Value.Shape;

    private static Shape[] Faces(in Shape shape) => shape.AllFaces().Select(static f => f.AsShape()).ToArray();

    private static Shape ShellOf(IEnumerable<Shape> faces)
    {
        var builder = new Builder();
        Shell shell = new();
        builder.MakeShell(ref shell);
        foreach (Shape face in faces)
        {
            builder.Add(shell, face);
        }

        return shell;
    }

    [Fact]
    public void A_whole_shell_closed_and_oriented_checks_as_the_full_check_does()
    {
        Shape box = Box(10, 20, 30);
        Same(box);
        Same(ShellOf(Faces(box)));
        Shape fused = box.Fuse(Box(5, 5, 50).Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(2, 2, -5)))),
                               new BooleanOptions { CheckResult = false }).Value.Shape;
        Same(fused);
    }

    [Fact]
    public void An_open_shell_and_a_shell_with_a_reversed_face_check_as_the_full_check_does()
    {
        Shape[] faces = Faces(Box(10, 10, 10));
        Same(ShellOf(faces.Take(5)));
        Same(ShellOf(faces.Select((f, i) => i == 2 ? f.Reversed() : f)));
    }

    [Fact]
    public void A_shell_with_an_internal_face_or_an_internal_edge_checks_as_the_full_check_does()
    {
        Shape[] faces = Faces(Box(10, 10, 10));
        // A face the second walk leaves out, and a face it keeps whose edge it leaves out
        Same(ShellOf(faces.Select((f, i) => i == 3 ? f.Oriented(Orientation.Internal) : f)));
        // The first face again, on its surface with its wires, and an internal wire of one of the box's other edges
        var builder = new BrepBuilder();
        Face first = ShapeCasts.Face(faces[0]);
        Face withInternalEdge = new();
        builder.MakeFace(ref withInternalEdge, Tool.Surface(first)!, Tool.Tolerance(first));
        foreach (Shape wire in faces[0].Children)
        {
            builder.Add(withInternalEdge, wire);
        }

        Wire internalWire = new();
        builder.MakeWire(ref internalWire);
        builder.Add(internalWire, faces[1].AllEdges()[0].AsShape().Oriented(Orientation.Internal));
        builder.Add(withInternalEdge, internalWire);
        Same(ShellOf(faces.Select((f, i) => i == 0 ? withInternalEdge.AsShape().Oriented(faces[0].Orientation) : f)));
    }
}
