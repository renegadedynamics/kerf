// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.IO.Compression;
using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Exchange;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.Reliability.Tests;

/// <summary>
/// What runs in the child process: <c>Main</c> takes a scenario's name and prints what it returned, so a test reads the
/// result, or the exit code of a process that died.
/// </summary>
internal static class Scenarios
{
    /// <summary>The deepest nesting that's allowed, and more than that.</summary>
    private const int Limit = ModelNode.MaxNesting;

    private static Shape Box() => Solid.Box(1, 1, 1).Value.Shape;

    private static Shape Nested(int depth)
    {
        Shape shape = Box();
        for (int i = 0; i < depth; i++)
        {
            shape = Compound.Create([shape]);
        }

        return shape;
    }

    private static ModelNode Tree(int depth)
    {
        ModelNode node = ModelNode.Part(Box(), "part");
        for (int i = 0; i < depth; i++)
        {
            node = ModelNode.Assembly([node], "assembly");
        }

        return node;
    }

    private static string Outcome(Result result) => result.IsSuccess ? "ok" : result.Error!.Code.Value;

    private static string Outcome<T>(Result<T> result) => result.IsSuccess ? "ok" : result.Error!.Code.Value;

    private static Stream Fixture(string name)
    {
        Stream stream = typeof(Scenarios).Assembly.GetManifestResourceStream("Fixtures." + name)!;
        return name.EndsWith(".gz", StringComparison.Ordinal) ? new GZipStream(stream, CompressionMode.Decompress) : stream;
    }

    private static MemoryStream Bytes(string name)
    {
        var bytes = new MemoryStream();
        using Stream fixture = Fixture(name);
        fixture.CopyTo(bytes);
        bytes.Position = 0;
        return bytes;
    }

    private static string RefusedToNest(int depth)
    {
        try
        {
            Nested(depth);
            return "built";
        }
        catch (ArgumentException)
        {
            return "refused";
        }
    }

    private static string RefusedTree(int depth)
    {
        try
        {
            Tree(depth);
            return "built";
        }
        catch (ArgumentException)
        {
            return "refused";
        }
    }

    public static readonly Dictionary<string, Func<string>> All = new()
    {
        ["compound-over-the-limit"] = () => RefusedToNest(Limit + 2),
        ["compound-far-over-the-limit"] = () => RefusedToNest(1_000_000),
        ["compound-at-the-limit"] = () => RefusedToNest(Limit + 1),
        ["faces"] = () => Nested(Limit).AllFaces().Length.ToString(System.Globalization.CultureInfo.InvariantCulture),
        ["volume"] = () => Nested(Limit).IntegrateVolume().ToString("F6", System.Globalization.CultureInfo.InvariantCulture),
        ["mesh"] = () => Outcome(Nested(Limit).Mesh()),
        ["validate"] = () => Nested(Limit).Validate().IsValid ? "ok" : "invalid",
        ["transform"] = () => Outcome(Nested(Limit).Transformed(Transform3D.Translation(new Vector3D(1, 0, 0)))),
        ["transform-copy"] = () => Outcome(Nested(Limit).Transformed(Transform3D.Translation(new Vector3D(1, 0, 0)), true)),
        ["mirror"] = () => Outcome(Nested(Limit).Mirrored(Frame3D.World)),
        ["scale"] = () => Outcome(Nested(Limit).Scaled(new Point3D(0, 0, 0), 2)),
        ["fuse"] = () => Outcome(BooleanOperations.Fuse(Nested(Limit), Solid.Box(1, 1, 1, new Frame3D(new Point3D(.5, 0, 0), Direction3D.UnitZ)).Value.Shape)),
        ["cut"] = () => Outcome(Nested(Limit).Cut(Solid.Box(1, 1, 1, new Frame3D(new Point3D(.5, 0, 0), Direction3D.UnitZ)).Value.Shape)),
        ["fix"] = () => Outcome(Nested(Limit).Fix()),
        ["unify"] = () => Outcome(Nested(Limit).Unify()),
        ["brep"] = () => Outcome(Brep.Write(Nested(Limit), new MemoryStream())),
        ["step"] = () => Outcome(Step.Write(ModelNode.Part(Nested(Limit)), new MemoryStream())),
        ["tree-over-the-limit"] = () => RefusedTree(Limit + 1),
        ["tree-far-over-the-limit"] = () => RefusedTree(1_000_000),
        ["tree-step"] = () => Outcome(Step.Write(Tree(Limit), new MemoryStream())),
        ["tree-gltf"] = () => Outcome(Gltf.Write(Tree(Limit), new MemoryStream())),
        ["tree-3mf"] = () => Outcome(ThreeMf.Write(Tree(Limit), new MemoryStream())),
        ["step-file-too-deep"] = () => Outcome(Step.Read(Bytes("deep-assembly-990.step.gz"))),
        ["step-curves-too-deep"] = () => Outcome(Step.Read(Bytes("deep-composite-curve-3000.step.gz"))),
        ["step-trimmed-curves-too-deep"] = () => Outcome(Step.Read(Bytes("deep-trimmed-curve-30000.step.gz"))),
        ["step-surfaces-too-deep"] = () => Outcome(Step.Read(Bytes("deep-offset-surface-30000.step.gz"))),
        ["step-curves-deep-but-fine"] = () =>
        {
            Result<ModelNode> read = Step.Read(Bytes("deep-composite-curve-300.step.gz"));
            return read.IsSuccess ? read.Value.ToShape().AllEdges().Length.ToString(System.Globalization.CultureInfo.InvariantCulture) : read.Error!.Code.Value;
        },
        ["gltf-file-too-deep"] = () => Outcome(Gltf.Read(Bytes("deep-assembly-300.glb"))),
        ["brep-file-too-deep"] = () => Outcome(Brep.Read(Bytes("deep-compound-500.brep"))),
        ["moved-many-times"] = MovedManyTimes,
        ["shared-twice-each-level"] = () =>
        {
            Shape shape = Box();
            try
            {
                for (int i = 0; i < 40; i++)
                {
                    shape = Compound.Create([shape, shape]);
                }
            }
            catch (ArgumentException)
            {
                return "refused";
            }

            return shape.AllFaces().Length.ToString(System.Globalization.CultureInfo.InvariantCulture);
        },
        ["mesh-too-fine"] = () => Outcome(Solid.Sphere(10).Value.Shape.Mesh(new MeshOptions { LinearDeflection = 1e-6, Parallel = false })),
        ["mesh-sensible"] = () => Outcome(Solid.Sphere(10).Value.Shape.Mesh(new MeshOptions { LinearDeflection = 1e-2, Parallel = false })),
        ["placement-chain"] = PlacementChain,
    };

    // A chain of placements was a recursion for each item, and a copy of the chain for each move
    private static string MovedManyTimes()
    {
        Shape shape = Box();
        var along = new Placement(Transform3D.Translation(new Vector3D(1, 0, 0)));
        var turn = new Placement(Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), Direction3D.UnitZ), 0.001));
        for (int i = 0; i < 100_000; i++)
        {
            shape = shape.Moved(i % 2 == 0 ? along : turn);
        }

        return shape.IntegrateVolume().ToString("F6", System.Globalization.CultureInfo.InvariantCulture);
    }

    private static string PlacementChain()
    {
        var placement = new Placement();
        for (int i = 0; i < 100_000; i++)
        {
            placement = placement.Multiplied(new Placement(Transform3D.Translation(new Vector3D(1, 0, 0))));
        }

        var two = new Placement(Transform3D.Translation(new Vector3D(1, 0, 0))).Multiplied(new Placement(Transform3D.Translation(new Vector3D(0, 1, 0))));
        Point3D far = new Point3D(0, 0, 0).Transformed(placement.Transformation);
        Point3D powered = new Point3D(0, 0, 0).Transformed(two.Powered(100_000).Transformation);
        return FormattableString.Invariant($"{far.X:F3} {powered.X:F3} {powered.Y:F3}");
    }
}
