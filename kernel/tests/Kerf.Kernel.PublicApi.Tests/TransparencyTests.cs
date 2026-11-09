// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Drawing;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// See-through items in the renderer (decision 1998), by pixels worked by hand. From the top, a face facing the eye is
/// lit 0.98017 (as in <see cref="RenderingTests"/>): light grey is 200, pure blue 250. A see-through box shows both its
/// top and its bottom (lit as if it faced the eye), each laid over what's behind it.
/// </summary>
public class TransparencyTests
{
    private static readonly RenderOptions Small = new() { Width = 200, Height = 150, Supersampling = 1, Edges = EdgeDisplay.None };

    private static readonly ShapeMesh Grey = Fixtures.Box().Mesh().Value;

    // A plate 4 × 4 × 1 over the box's top (the box is 10 × 20 × 30)
    private static readonly ShapeMesh Plate = Solid.Box(4, 4, 1, new Frame3D(new Point3D(3, 8, 40), Direction3D.UnitZ)).Value.Shape.Mesh().Value;

    private static (byte R, byte G, byte B, byte A) Centre(Scene scene, RenderOptions? options = null) =>
        Renderer.Render(scene, Camera.Top, options ?? Small).Value.GetPixel(100, 75);

    [Fact]
    public void A_see_through_plate_shows_the_box_behind_it()
    {
        // Grey 200 under the plate's bottom (blue 250, half) and then its top: 100, 100, 225; then 50, 50, 237.5
        var scene = new Scene(new SceneItem(Grey), new SceneItem(Plate, new Color(0, 0, 1, 0.5)));
        (byte r, byte g, byte b, byte a) = Centre(scene);
        Assert.InRange(r, 49, 51);
        Assert.Equal(r, g);
        Assert.InRange(b, 236, 239);
        Assert.Equal(255, a);
        // Opaque, the plate hides the box: as before
        Assert.Equal(0, Centre(new Scene(new SceneItem(Grey), new SceneItem(Plate, new Color(0, 0, 1)))).R);
    }

    [Fact]
    public void The_order_of_the_items_doesnt_matter()
    {
        // Two see-through items and an opaque one, overlapping, in each of the six orders: the same pixels
        var items = new[]
        {
            new SceneItem(Grey, new Color(0.8, 0.8, 0.8, 0.4)),
            new SceneItem(Plate, new Color(0, 0, 1, 0.5)),
            new SceneItem(Plate, new Color(1, 0, 0)) with { Transform = Transform3D.Translation(new Vector3D(2, 3, -50)) },
        };
        int[][] orders = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
        var options = RenderOptions.Default with { Width = 200, Height = 150 };
        byte[] first = Renderer.Render(new Scene(orders[0].Select(i => items[i])), Camera.Isometric, options).Value.Pixels.ToArray();
        foreach (int[] order in orders.Skip(1))
        {
            Assert.Equal(first, Renderer.Render(new Scene(order.Select(i => items[i])), Camera.Isometric, options).Value.Pixels.ToArray());
        }

        // And serial or parallel
        Assert.Equal(first, Renderer.Render(new Scene(items), Camera.Isometric, options with { Parallel = false }).Value.Pixels.ToArray());
    }

    [Fact]
    public void A_face_of_many_triangles_is_laid_once_at_each_pixel()
    {
        // A cylinder's end is a fan of triangles: see-through over white, every pixel well inside it is the same colour,
        // where a pixel on two triangles' shared edge laid twice would be darker
        Solid cylinder = Fixtures.Cylinder();
        var options = Small with { Shading = Shading.Flat };
        RgbaImage image = Renderer.Render(new Scene(new SceneItem(cylinder.Mesh().Value, new Color(0, 0, 0, 0.3))), Camera.Top, options).Value;
        var inside = new HashSet<(byte, byte, byte, byte)>();
        for (int y = 55; y < 95; y++)
        {
            for (int x = 80; x < 120; x++)
            {
                inside.Add(image.GetPixel(x, y));
            }
        }

        // Black at 0.3 over the bottom (also black at 0.3) over white: 255 × 0.7 × 0.7 = 125
        Assert.Single(inside);
        Assert.InRange(inside.Single().Item1, 124, 126);
    }

    [Fact]
    public void A_see_through_items_edges_are_see_through_and_hidden_edges_show_through()
    {
        var options = Small with { Edges = EdgeDisplay.Feature };
        // The box's left edge at column 66 (as in RenderingTests), in the edge colour 0.1 (26) when opaque
        RgbaImage opaque = Renderer.Render(new Scene(new SceneItem(Grey)), Camera.Top, options).Value;
        RgbaImage glass = Renderer.Render(new Scene(new SceneItem(Grey, new Color(0.8, 0.8, 0.8, 0.5))), Camera.Top, options).Value;
        Assert.InRange(opaque.GetPixel(66, 75).R, 24, 27);
        // See-through, at column 66: the bottom (grey 200, half) over white, the top (half), then the item's nearest edge
        // (half; its bottom edge there is the same item's, so not laid again): 227.5, 213.75, then 119.9
        Assert.InRange(glass.GetPixel(66, 75).R, 117, 123);
        // An opaque red cube inside the see-through box: its edges and top show through the box's top, half grey. Its left
        // edge, x = 3, is at column 66.25 + 3 × 6.75 = 86.5: 26 under grey at half, 113; its top, red 250, 225, 100, 100
        var cube = Solid.Box(4, 4, 1, new Frame3D(new Point3D(3, 8, 10), Direction3D.UnitZ)).Value.Shape.Mesh().Value;
        RgbaImage inside = Renderer.Render(new Scene(new SceneItem(Grey, new Color(0.8, 0.8, 0.8, 0.5)), new SceneItem(cube, new Color(1, 0, 0))),
                                           Camera.Top, options).Value;
        Assert.InRange(inside.GetPixel(86, 75).R, 110, 116);
        (byte r, byte g, byte b, _) = inside.GetPixel(100, 75);
        Assert.InRange(r, 223, 227);
        Assert.InRange(g, 98, 102);
        Assert.Equal(g, b);
    }

    [Fact]
    public void A_see_through_face_over_a_clear_background_is_partly_clear()
    {
        var options = Small with { Background = new Color(1, 1, 1, 0) };
        (_, _, _, byte a) = Centre(new Scene(new SceneItem(Plate, new Color(0, 0, 1, 0.5))), options);
        Assert.InRange(a, 190, 192);   // 1 − 0.5 × 0.5 of 255: two faces at half each
    }

    [Fact]
    public void Golden_picture()
    {
        Solid cylinder = Fixtures.Cylinder();
        var scene = new Scene(new SceneItem(Grey, new Color(0.2, 0.5, 0.9, 0.35)),
                              new SceneItem(cylinder.Mesh().Value, new Color(0.9, 0.4, 0.1)) with { Transform = Transform3D.Translation(new Vector3D(2, 6, 4)) });
        RgbaImage image = Renderer.Render(scene, Camera.Isometric, new RenderOptions { Width = 160, Height = 120 }).Value;
        // To make it again after an intended change: KERF_GOLDEN_DIR=<this project's Fixtures folder>
        if (Environment.GetEnvironmentVariable("KERF_GOLDEN_DIR") is { Length: > 0 } directory)
        {
            File.WriteAllBytes(Path.Combine(directory, "golden-glass-iso.png"), Png.Encode(image));
        }

        PngDecoder.Decoded golden = PngDecoder.Decode(Fixtures.Resource("golden-glass-iso.png"));
        ReadOnlySpan<byte> pixels = image.Pixels.Span;
        int worst = 0;
        for (int i = 0; i < pixels.Length; i++)
        {
            worst = Math.Max(worst, Math.Abs(pixels[i] - golden.Rgba[i]));
        }

        Assert.True(worst <= 3, $"A channel differs by {worst}");
    }
}
