// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Drawing;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.Drawing;

/// <summary>
/// Unit 14E: the renderer's loops over pixels by vectors of four give the scalar loops' pictures byte for byte: shapes
/// flat and curved, cameras orthographic and perspective, every supersampling, smooth and flat shading, edges, widths
/// that leave rows a few pixels short of a vector, and a see-through colour (whose faces stay on the scalar path).
/// </summary>
public class RendererVectorTests
{
    private static Shape Part()
    {
        Solid box = Primitives.Box(20, 14, 8).Value.Shape;
        Solid cylinder = Primitives.Cylinder(4, 30, new Frame3D(new Point3D(10, 7, -5), Direction3D.UnitZ)).Value.Shape;
        return box.Cut(cylinder).Value.Shape;
    }

    public static TheoryData<string> Shapes() => ["part", "sphere", "torus"];

    [Theory]
    [MemberData(nameof(Shapes))]
    public void VectorAndScalarPicturesAreTheSame(string name)
    {
        Shape shape = name switch
        {
            "part" => Part(),
            "sphere" => Primitives.Sphere(6).Value.Shape,
            _ => Primitives.Torus(8, 2.5).Value.Shape,
        };
        ShapeMesh mesh = shape.Mesh(Renderer.DefaultMeshOptions(shape)).Value;
        Camera[] cameras = [Camera.Isometric, Camera.Top, Camera.LookAt(new Point3D(40, -30, 25), new Point3D(0, 0, 0), projection: Projection.Perspective)];
        RenderOptions[] options =
        [
            RenderOptions.Default with { Width = 203, Height = 151 },
            new() { Width = 97, Height = 64, Supersampling = 1, Shading = Shading.Flat },
            new() { Width = 130, Height = 77, Supersampling = 3, Edges = EdgeDisplay.All, Ambient = 0.1 },
            new() { Width = 61, Height = 45, Supersampling = 4 },
        ];
        var scenes = new[]
        {
            new Scene(new SceneItem(mesh)),
            new Scene(new SceneItem(mesh, new Color(0.2, 0.6, 0.9, 0.5))),
        };
        int compared = 0;
        foreach (Scene scene in scenes)
        {
            foreach (Camera camera in cameras)
            {
                foreach (RenderOptions option in options)
                {
                    RgbaImage scalar = Renderer.Render(scene, camera, option, vectors: false, CancellationToken.None).Value;
                    RgbaImage vector = Renderer.Render(scene, camera, option, vectors: true, CancellationToken.None).Value;
                    Assert.True(scalar.Pixels.Span.SequenceEqual(vector.Pixels.Span), $"{name}, {camera}, {option}");
                    Assert.Contains(scalar.Pixels.ToArray(), b => b is > 0 and < 255);
                    compared++;
                }
            }
        }

        Assert.Equal(24, compared);
    }
}
