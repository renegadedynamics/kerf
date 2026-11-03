// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Process;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;
using System.Reflection;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// No copy at entry in healing (Kerf unit 10H4, phase10-plan.md 2.4): each healing tool, sewing and each ShapeProcess
/// operator runs on the caller's frozen shape in a working overlay. After each, the input is untouched, TShape for
/// TShape and value for value (<see cref="ShapeSnapshot"/>), the result is frozen, and it holds the input's own
/// sub-shapes wherever the tool didn't change them. The inputs are OCCT's healing corpus files, which need fixing.
/// </summary>
public class HealingInputsUnchangedTests
{
    private static string Metadata(string key) =>
        typeof(HealingInputsUnchangedTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                           .Single(a => a.Key == key).Value!;

    // A file of OCCT's test data (the heal group's inputs), frozen as a caller's shape
    private static Shape Read(string name)
    {
        string path = Path.Combine(Metadata("OcctTestDataDir"), "brep", name);
        Assert.True(BrepTools.Read(out Shape shape, path), path);
        shape.TShape()!.FreezeGraph();
        return shape;
    }

    private static List<Shape> SubShapes(in Shape shape, ShapeType type)
    {
        var map = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(shape, type, map);
        var list = new List<Shape>(map.Count);
        for (int i = 0; i < map.Count; i++)
        {
            list.Add(map[i]);
        }
        return list;
    }

    // The number of the input's sub-shapes of the type whose TShapes are in the result
    private static int Shared(in Shape result, in Shape input, ShapeType type)
    {
        var tShapes = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        foreach (Shape s in SubShapes(result, type))
        {
            tShapes.Add(s.TShape()!);
        }
        return SubShapes(input, type).Count(s => tShapes.Contains(s.TShape()!));
    }

    // True if the result holds some of the input's vertices, edges or faces, or shares their geometry (a tool that
    // changed every edge, as sewing, keeps the faces' surfaces and the edges' curves)
    private static bool SharesSome(in Shape result, in Shape input)
    {
        if (Shared(result, input, ShapeType.Vertex) + Shared(result, input, ShapeType.Edge)
            + Shared(result, input, ShapeType.Face) > 0)
        {
            return true;
        }
        var geometry = new HashSet<object>(ReferenceEqualityComparer.Instance);
        foreach (Shape f in SubShapes(input, ShapeType.Face))
        {
            if (Tool.Surface(ShapeCasts.Face(f)) is { } surface)
            {
                geometry.Add(surface);
            }
        }
        foreach (Shape e in SubShapes(input, ShapeType.Edge))
        {
            if (Tool.Curve(ShapeCasts.Edge(e), out _, out _) is { } curve)
            {
                geometry.Add(curve);
            }
        }
        return SubShapes(result, ShapeType.Face).Exists(f => Tool.Surface(ShapeCasts.Face(f)) is { } s && geometry.Contains(s))
               || SubShapes(result, ShapeType.Edge).Exists(e => Tool.Curve(ShapeCasts.Edge(e), out _, out _) is { } c
                                                                 && geometry.Contains(c));
    }

    // Runs the tool on the frozen input; checks the input untouched, the result frozen and, when the tool changed
    // something, a new shape sharing some of the input's sub-shapes or geometry
    private static Shape Check(in Shape input, Func<Shape, Shape> tool, bool isChanged = true)
    {
        ShapeSnapshot snapshot = ShapeSnapshot.Of(input);
        Shape result = tool(input);
        snapshot.AssertUnchanged(input);
        Assert.False(result.IsNull);
        Assert.True(result.TShape()!.IsFrozen);
        if (isChanged)
        {
            Assert.False(ReferenceEquals(result.TShape(), input.TShape()));
            Assert.True(SharesSome(result, input), "the result shares nothing with the input");
        }
        return result;
    }

    [Theory]
    [InlineData("BoxAndConeFS.rle")]
    [InlineData("newFS.rle")]
    [InlineData("aaa.rle")]
    [InlineData("CTO900_buc60127-part.rle")]
    public void ShapeFix_Shape_leaves_its_input_untouched(string name)
    {
        Check(Read(name), s =>
        {
            var fixer = new ShapeFixer(s);
            fixer.Perform();
            return fixer.Shape();
        });
    }

    [Fact]
    public void The_shell_solid_face_wire_and_vertex_fixers_leave_their_inputs_untouched()
    {
        Shape shape = Read("aaa.rle");
        foreach (Shape shell in SubShapes(shape, ShapeType.Shell))
        {
            Check(shell, s =>
            {
                var fixer = new ShellFixer(ShapeCasts.Shell(s));
                fixer.Perform();
                return fixer.Shape();
            }, isChanged: false);
        }
        foreach (Shape solid in SubShapes(shape, ShapeType.Solid))
        {
            Check(solid, s =>
            {
                var fixer = new SolidFixer(ShapeCasts.Solid(s));
                fixer.Perform();
                return fixer.Shape();
            }, isChanged: false);
        }
        int changedFaces = 0, changedWires = 0;
        foreach (Shape face in SubShapes(shape, ShapeType.Face))
        {
            Shape fixedFace = Check(face, f =>
            {
                var fixer = new FaceFixer(ShapeCasts.Face(f));
                fixer.Perform();
                return fixer.Face();
            }, isChanged: false);
            changedFaces += ReferenceEquals(fixedFace.TShape(), face.TShape()) ? 0 : 1;

            foreach (Shape wire in SubShapes(face, ShapeType.Wire))
            {
                Shape fixedWire = Check(wire, w =>
                {
                    var fixer = new WireFixer(ShapeCasts.Wire(w), ShapeCasts.Face(face), 1e-3);
                    fixer.Perform();
                    return fixer.Wire();
                }, isChanged: false);
                changedWires += ReferenceEquals(fixedWire.TShape(), wire.TShape()) ? 0 : 1;

                Check(wire, w =>
                {
                    var fixer = new WireVertexFixer();
                    fixer.Init(ShapeCasts.Wire(w), 1e-3);
                    fixer.Fix();
                    return fixer.Wire();
                }, isChanged: false);
            }
        }
        // the fixers wrote somewhere: their writes went to the overlays
        Assert.True(changedFaces > 0);
        Assert.True(changedWires > 0);
    }

    [Fact]
    public void ShapeFix_Wireframe_leaves_its_input_untouched()
    {
        Check(Read("boxFG.rle"), s =>
        {
            var fixer = new WireframeFixer(s);
            fixer.FixWireGaps();
            fixer.FixSmallEdges();
            return fixer.Shape();
        });
    }

    [Fact]
    public void ShapeFix_FixSmallFace_leaves_its_input_untouched()
    {
        Check(Read("CIN001_b62.rle"), s =>
        {
            var fixer = new FixSmallFaceFixer();
            fixer.Init(s);
            fixer.Perform();
            return fixer.Shape();
        }, isChanged: false);
    }

    // Six faces of a box, each with edges of its own: a frozen compound to sew
    private static Shape SeparateFaces()
    {
        Shape box = new MakeBox(10, 10, 10).Shape();
        var b = new BrepBuilder();
        var compound = new Compound();
        b.MakeCompound(ref compound);
        foreach (Shape face in SubShapes(box, ShapeType.Face))
        {
            b.Add(compound, new Copy(face, false).Shape());
        }
        compound.TShape()!.FreezeGraph();
        return compound;
    }

    [Fact]
    public void Sewing_and_the_free_bounds_fixer_leave_their_inputs_untouched()
    {
        Shape faces = SeparateFaces();
        Shape sewn = Check(faces, s =>
        {
            var sewing = new Sewing(1e-6);
            foreach (Shape face in SubShapes(s, ShapeType.Face))
            {
                sewing.Add(face);
            }
            sewing.Perform();
            return sewing.SewedShape();
        });
        Assert.Equal(12, SubShapes(sewn, ShapeType.Edge).Count);
        // the faces keep their surfaces
        Assert.All(SubShapes(sewn, ShapeType.Face),
                   f => Assert.Contains(SubShapes(faces, ShapeType.Face),
                                        g => ReferenceEquals(Tool.Surface(ShapeCasts.Face(f)), Tool.Surface(ShapeCasts.Face(g)))));

        Check(faces, s => new FreeBoundsFixer(s, 1e-6, 1e-6, true, true).GetShape(), isChanged: false);
    }

    // Two unit squares side by side, each with its own edges, in a frozen shell (ShapeFix_FaceConnectTests')
    private static Shell TwoSquares(out Face f1, out Face f2)
    {
        f1 = new MakeFace(new Plane3D(), 0, 1, 0, 1).Face();
        f2 = new MakeFace(new Plane3D(), 1 + 1e-7, 2, 0, 1).Face();
        var b = new BrepBuilder();
        var shell = new Shell();
        b.MakeShell(ref shell);
        b.Add(shell, f1);
        b.Add(shell, f2);
        shell.TShape()!.FreezeGraph();
        return shell;
    }

    [Fact]
    public void The_connect_fixers_leave_their_inputs_untouched()
    {
        Shell shell = TwoSquares(out Face f1, out Face f2);
        Check(shell, s =>
        {
            var connect = new FaceConnectFixer();
            connect.Add(f1, f2);
            return connect.Build(ShapeCasts.Shell(s), 1e-4, 1e-4);
        });

        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Edge e2 = new MakeEdge(new Point3D(1.0002, 0, 0), new Point3D(2, 0, 0)).Edge();
        var b = new BrepBuilder();
        var edges = new Compound();
        b.MakeCompound(ref edges);
        b.Add(edges, e1);
        b.Add(edges, e2);
        edges.TShape()!.FreezeGraph();
        Check(edges, s =>
        {
            var connect = new EdgeConnectFixer();
            connect.Add(e1, e2);
            return connect.Built(s);
        });
    }

    // Two boxes side by side, fused: their coplanar faces to unify
    private static Shape FusedBoxes() =>
        new Fuse(new MakeBox(new Point3D(0, 0, 0), 2, 1, 1).Shape(), new MakeBox(new Point3D(1, 0, 0), 2, 1, 1).Shape())
            .Shape();

    [Fact]
    public void UnifySameDomain_leaves_its_input_untouched_and_keeps_the_surfaces()
    {
        Shape fused = FusedBoxes();
        Shape unified = Check(fused, s =>
        {
            var unify = new UnifySameDomain(s, true, true, false);
            unify.Build();
            return unify.Shape();
        });
        Assert.Equal(6, SubShapes(unified, ShapeType.Face).Count);
        // the end faces keep their surfaces (the faces themselves are new: their edges gain the merged faces'
        // p-curves, so a face holding one changes)
        int kept = SubShapes(unified, ShapeType.Face).Count(f => SubShapes(fused, ShapeType.Face).Exists(
            g => ReferenceEquals(Tool.Surface(ShapeCasts.Face(f)), Tool.Surface(ShapeCasts.Face(g)))));
        Assert.True(kept >= 2);
    }

    [Fact]
    public void The_dividers_leave_their_inputs_untouched()
    {
        Shape cylinder = new ApiMakeCylinder(5, 10).Shape();
        Check(cylinder, s =>
        {
            var divide = new ShapeDivideAngle(Math.PI / 4, s);
            divide.Perform();
            return divide.Result();
        });
        Check(cylinder, s =>
        {
            var divide = new ShapeDivideClosed(s);
            divide.Perform();
            return divide.Result();
        });
    }

    [Fact]
    public void The_twins_leave_their_inputs_untouched()
    {
        Shape shape = Read("BoxAndConeFS.rle");
        Check(shape, s => ShapeFixFixer.SameParametered(s, true, out _));
        Check(shape, s => ShapeFixFixer.RegularityEncoded(s), isChanged: false);
        Check(shape, s => new ShapeToleranceFixer().WithTolerance(s, 1e-3));
        Edge edge = ShapeCasts.Edge(SubShapes(shape, ShapeType.Edge)[0]);
        Check(edge, e => new BuildEdgeBuilder().WithRange3d(ShapeCasts.Edge(e), 0.1, 0.2), isChanged: false);
    }

    [Theory]
    [InlineData("BoxAndConeFS.rle")]
    [InlineData("aaa.rle")]
    public void Each_ShapeProcess_operator_leaves_its_input_untouched(string name)
    {
        OperLibrary.Init();
        Shape shape = Read(name);
        ShapeSnapshot snapshot = ShapeSnapshot.Of(shape);
        int changed = 0;
        for (ShapeProcess.Operation operation = ShapeProcess.Operation.First;
             operation <= ShapeProcess.Operation.Last;
             operation++)
        {
            var context = new ShapeContext(shape, "");
            ShapeProcess.Perform(context, new ShapeProcess.OperationsFlags(operation));
            Shape result = context.Result();
            snapshot.AssertUnchanged(shape);
            if (result.IsNull)
            {
                continue; // ToBezier with its default modes gives a null result, as natively
            }
            Assert.True(result.TShape()!.IsFrozen);
            if (!ReferenceEquals(result.TShape(), shape.TShape()))
            {
                changed++;
                Assert.True(SharesSome(result, shape), $"{operation}: the result shares nothing with the input");
            }
        }
        Assert.True(changed > 0);
    }

    [Fact]
    public void No_overlay_is_left_open()
    {
        Shape shape = Read("aaa.rle");
        var fixer = new FaceFixer(ShapeCasts.Face(SubShapes(shape, ShapeType.Face)[0]));
        fixer.Perform(); // the tool's overlay is entered by each fix only
        Assert.Null(WorkingOverlay.Current);
        fixer.Face();
        Assert.Null(WorkingOverlay.Current);
    }
}
