// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Topology;
using Kerf.Fillets;
using Kerf.Healing;
using Kerf.Kernel.DrawLite;
using Kerf.Measurement;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// The corpus's <c>blend/complex</c> C1, C5, F1 and F2 (unit 15X): each has a cylinder of exactly the fillet's radius with the ball
/// inside it, so the ball stands still there and the fillet is a piece of a sphere, which ChFi3d can't build (decision 2159). The
/// cases' shapes and edges as their scripts make them, filleted through the public <c>Fillet</c>: where ChFi3d fails, stage 4c builds
/// them, proved by the ball and the volume from the exact sections. Draw's <c>blend</c> calls the engine alone, so the corpus's
/// statuses don't move.
/// </summary>
public class DrawFilletSphereTests(ITestOutputHelper output)
{
    private static readonly MassOptions Precise = new() { Tolerance = 1e-9 };

    // The script up to its blend, and the shape and edges it fillets
    private static (Shape Shape, Edge[] Edges, double Radius) Load(string name)
    {
        var interpreter = new DrawInterpreter();
        interpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        string data = DrawResourcesTests.OcctTestDataDirectory.Replace('\\', '/').TrimEnd('/') + "/brep";
        string[] lines = [.. File.ReadAllLines(Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests", "blend", "complex", name))
                                 .Select(l => l.Trim()).Where(l => l.Length > 0 && !l.StartsWith('#') && !l.StartsWith("checkprops", StringComparison.Ordinal))];
        foreach (string line in lines.Where(l => !l.StartsWith("blend ", StringComparison.Ordinal)))
        {
            string[] parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            interpreter.Eval(parts[0] == "restore"
                ? $"restore {data}/{(parts[1].StartsWith('[') ? parts[2].TrimEnd(']') : Path.GetFileName(parts[1]))} {parts[^1]}"
                : line);
        }
        string[] blend = lines.Single(l => l.StartsWith("blend ", StringComparison.Ordinal)).Split(' ', StringSplitOptions.RemoveEmptyEntries);
        var edges = new List<Edge>();
        double radius = 0;
        for (int i = 3; i + 1 < blend.Length; i += 2)
        {
            radius = double.Parse(blend[i], CultureInfo.InvariantCulture);
            edges.Add(ShapeCasts.Edge(interpreter.GetShape(blend[i + 1])));
        }
        return (interpreter.GetShape(blend[2]), [.. edges], radius);
    }

    [Theory]
    [InlineData("F1", 18, 796640.4254, 50534.2309)]
    [InlineData("F2", 18, 791489.1380, 49421.8940)]
    [InlineData("C5", 17, 4876176.7540, 180859.1874)]
    public void The_sphere_cases_are_built_where_ChFi3d_fails(string name, int faces, double volume, double area)
    {
        (Shape shape, Edge[] edges, double radius) = Load(name);
        var chfi3d = new MakeFillet(shape);
        foreach (Edge edge in edges)
        {
            chfi3d.Add(radius, edge);
        }
        chfi3d.Build();
        Assert.False(chfi3d.IsDone);

        Result<Built<Shape>> result = shape.Fillet(edges, radius);
        Assert.True(result.IsSuccess, result.Error?.Message);
        Shape made = result.Value.Shape;
        Assert.True(made.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(shape, made));
        output.WriteLine(FormattableString.Invariant($"{name}: {made.AllFaces().Length} faces, volume {made.IntegrateVolume(Precise):R}, area {made.IntegrateArea(Precise):R}"));
        Assert.Equal(faces, made.AllFaces().Length);
        Assert.Equal(volume, made.IntegrateVolume(Precise), 1e-8 * volume);
        Assert.Equal(area, made.IntegrateArea(Precise), 1e-8 * area);
    }

    [Fact]
    public void C1_still_fails_at_its_slanted_end()
    {
        // The pin's fillet runs on to a face that meets the common face at a concave corner, slanted to the contour: the fillet
        // there would have to be cut by that face, carried on past its edge, which this stage doesn't do (decision 30205)
        (Shape shape, Edge[] edges, double radius) = Load("C1");
        Result<Built<Shape>> result = shape.Fillet(edges, radius);
        Assert.True(result.IsFailure);
    }
}
