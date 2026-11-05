// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Drawing;
using Kerf.Exchange;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Kerf.Drawing's renderer (docs/api/drawing.md): pixel facts worked by hand, depth, colours, edges, determinism, and
/// golden pictures. The fixtures' box (10 × 20 × 30) seen from the top in 200 × 150 pixels fits at 6.75 pixels a unit
/// (the height decides: 150 × 0.9 / 20), centred: its top covers columns 66.25 to 133.75 and rows 7.5 to 142.5.
/// </summary>
public class RenderingTests
{
    private static readonly RenderOptions Small = new() { Width = 200, Height = 150, Supersampling = 1, Edges = EdgeDisplay.None };

    private static bool IsBackground((byte R, byte G, byte B, byte A) p) => p == (255, 255, 255, 255);

    private static int Coverage(RgbaImage image)
    {
        int count = 0;
        for (int y = 0; y < image.Height; y++)
        {
            for (int x = 0; x < image.Width; x++)
            {
                count += IsBackground(image.GetPixel(x, y)) ? 0 : 1;
            }
        }

        return count;
    }

    [Fact]
    public void A_box_from_the_top_is_a_lit_rectangle()
    {
        RgbaImage image = Fixtures.Box().Render(Camera.Top, Small).Value;
        Assert.Equal((200, 150), (image.Width, image.Height));
        foreach ((int x, int y) in new[] { (0, 0), (199, 0), (0, 149), (199, 149) })
        {
            Assert.True(IsBackground(image.GetPixel(x, y)));
        }

        // The top faces the eye: ambient 0.3, and the lights' 0.65 and 0.2 times their directions' Z (0.83771 and
        // 0.67828): 0.98017 of light grey's 0.8, so 0.78413 or 200 of 255
        (byte r, byte g, byte b, byte a) = image.GetPixel(100, 75);
        Assert.InRange(r, 199, 201);
        Assert.Equal(r, g);
        Assert.Equal(r, b);
        Assert.Equal(255, a);
        // 67.5 × 135 pixels; the pixels whose centres are in it, 68 × 136
        Assert.Equal(68 * 136, Coverage(image), 68 * 136 * 0.01);
        Assert.True(IsBackground(image.GetPixel(65, 75)));
        Assert.False(IsBackground(image.GetPixel(66, 75)));
    }

    [Fact]
    public void Edges_are_drawn_over_the_faces_and_seams_left_out()
    {
        RgbaImage plain = Fixtures.Box().Render(Camera.Top, Small).Value;
        RgbaImage edged = Fixtures.Box().Render(Camera.Top, Small with { Edges = EdgeDisplay.Feature }).Value;
        Assert.InRange(edged.GetPixel(66, 75).R, 24, 27); // the left edge, at 66.25, in the edge colour (0.1)
        Assert.Equal(plain.GetPixel(100, 75), edged.GetPixel(100, 75));

        // A cylinder's seam is drawn only when all edges are
        Solid cylinder = Fixtures.Cylinder();
        int Dark(RgbaImage image) => Enumerable.Range(0, image.Height).Sum(y => Enumerable.Range(0, image.Width).Count(x => image.GetPixel(x, y).R is >= 24 and <= 27));
        int feature = Dark(cylinder.Render(Camera.Isometric, Small with { Edges = EdgeDisplay.Feature }).Value);
        int all = Dark(cylinder.Render(Camera.Isometric, Small with { Edges = EdgeDisplay.All }).Value);
        Assert.True(feature > 300, $"{feature}");
        Assert.True(all > feature + 50, $"{all} > {feature}");
    }

    [Fact]
    public void A_face_can_have_its_own_colour()
    {
        Solid box = Fixtures.Box();
        ShapeMesh mesh = box.Mesh().Value;
        Face top = box.AllFaces().Single(f => f.AllVertices().All(v => v.Point.Z == 30));
        var item = new SceneItem(mesh).WithFaceColor(top, new Color(1, 0, 0));
        (byte r, byte g, byte b, _) = Renderer.Render(new Scene(item), Camera.Top, Small).Value.GetPixel(100, 75);
        Assert.InRange(r, 240, 255);
        Assert.Equal(0, g);
        Assert.Equal(0, b);
        Assert.Throws<ArgumentException>(() => item.WithFaceColor(Fixtures.Cylinder().AllFaces()[0], new Color(0, 1, 0)));
    }

    [Fact]
    public void Nearer_faces_hide_farther_ones_whatever_the_order()
    {
        ShapeMesh box = Fixtures.Box().Mesh().Value;
        ShapeMesh plate = Solid.Box(4, 4, 1, new Frame3D(new Point3D(3, 8, 40), Direction3D.UnitZ)).Value.Shape.Mesh().Value;
        var grey = new SceneItem(box);
        var blue = new SceneItem(plate, new Color(0, 0, 1));
        foreach (Scene scene in new[] { new Scene(grey, blue), new Scene(blue, grey) })
        {
            RgbaImage image = Renderer.Render(scene, Camera.Top, Small).Value;
            (byte r, _, byte b, _) = image.GetPixel(100, 75);
            Assert.Equal(0, r);
            Assert.True(b > 200);
        }

        // Moved below the box, the plate is hidden
        RgbaImage below = Renderer.Render(new Scene(grey, blue with { Transform = Transform3D.Translation(new Vector3D(0, 0, -45)) }), Camera.Top,
                                          Small).Value;
        Assert.InRange(below.GetPixel(100, 75).R, 199, 201);
    }

    [Fact]
    public void The_background_can_be_transparent()
    {
        RgbaImage image = Fixtures.Box().Render(Camera.Isometric, RenderOptions.Default with { Background = new Color(1, 1, 1, 0) }).Value;
        Assert.Equal(0, image.GetPixel(0, 0).A);
        Assert.Equal(255, image.GetPixel(400, 300).A);
    }

    [Fact]
    public void A_perspective_camera_shows_more_of_the_near_end()
    {
        Solid box = Fixtures.Box();
        Camera orthographic = Camera.LookAt(new Point3D(5, 10, 80), new Point3D(5, 10, 15), Direction3D.UnitY);
        RgbaImage flat = box.Render(orthographic, Small with { Scale = 4 }).Value;
        RgbaImage perspective = box.Render(orthographic with { Projection = Projection.Perspective }, Small with { Scale = 4 }).Value;
        // At 4 pixels a unit both are centred on the target; in perspective, the top (nearer than the target) is larger
        Assert.True(Coverage(perspective) > 1.3 * Coverage(flat), $"{Coverage(perspective)} > {Coverage(flat)}");
        Assert.InRange(perspective.GetPixel(100, 75).R, 199, 201);
    }

    [Fact]
    public void The_same_scene_gives_the_same_pixels_serial_or_parallel()
    {
        Solid cylinder = Fixtures.Cylinder();
        byte[] parallel = cylinder.Render(Camera.Isometric).Value.Pixels.ToArray();
        byte[] serial = cylinder.Render(Camera.Isometric, RenderOptions.Default with { Parallel = false }).Value.Pixels.ToArray();
        Assert.Equal(serial, parallel);
        Assert.Equal(Png.Encode(cylinder.Render(Camera.Isometric).Value), Png.Encode(cylinder.Render(Camera.Isometric).Value));
    }

    [Theory]
    [InlineData("box")]
    [InlineData("cylinder")]
    public void Golden_pictures(string name)
    {
        Solid solid = name == "box" ? Fixtures.Box() : Fixtures.Cylinder();
        RgbaImage image = solid.Render(Camera.Isometric, new RenderOptions { Width = 160, Height = 120 }).Value;
        // To make them again after an intended change: KERF_GOLDEN_DIR=<this project's Fixtures folder>
        if (Environment.GetEnvironmentVariable("KERF_GOLDEN_DIR") is { Length: > 0 } directory)
        {
            File.WriteAllBytes(Path.Combine(directory, $"golden-{name}-iso.png"), Png.Encode(image));
        }

        PngDecoder.Decoded golden = PngDecoder.Decode(Fixtures.Resource($"golden-{name}-iso.png"));
        Assert.Equal((golden.Width, golden.Height), (image.Width, image.Height));
        ReadOnlySpan<byte> pixels = image.Pixels.Span;
        int worst = 0;
        for (int i = 0; i < pixels.Length; i++)
        {
            worst = Math.Max(worst, Math.Abs(pixels[i] - golden.Rgba[i]));
        }

        Assert.True(worst <= 3, $"A channel differs by {worst}");
    }

    [Fact]
    public void A_model_renders_with_its_colours()
    {
        ModelNode model = ModelNode.Assembly(
        [
            ModelNode.Part(Fixtures.Box(), "box", new Color(0, 0.5, 0)),
            ModelNode.Assembly([ModelNode.Part(Fixtures.Cylinder(), "cylinder")], "moved",
                               new Placement(Transform3D.Translation(new Vector3D(30, 0, 0))), new Color(0.5, 0, 0)),
        ]);
        Scene scene = model.ToScene().Value;
        Assert.Equal(2, scene.Items.Length);
        Assert.Equal(new Color(0, 0.5, 0), scene.Items[0].Color);
        Assert.Equal(new Color(0.5, 0, 0), scene.Items[1].Color); // inherited
        Assert.Equal(35, scene.Items[1].Mesh.Positions.ToArray().Where((_, i) => i % 3 == 0).Max(), 1e-4); // placed
        RgbaImage image = Renderer.Render(scene, Camera.Top, Small).Value;
        Assert.Contains(Enumerable.Range(0, 200 * 150), i => image.GetPixel(i % 200, i / 200) is { R: > 50, G: 0 });
    }

    [Fact]
    public void Misuse_throws_and_an_empty_scene_is_the_background()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new RenderOptions { Supersampling = 5 });
        Assert.Throws<ArgumentOutOfRangeException>(() => new RenderOptions { Width = 0 });
        Assert.Throws<ArgumentNullException>(() => Renderer.Render((Scene)null!, Camera.Top));
        Assert.Throws<ArgumentException>(() => Renderer.Render(new Scene(), Camera.Top with { Up = Direction3D.UnitZ }));
        RgbaImage empty = Renderer.Render(new Scene(), Camera.Top, Small).Value;
        Assert.Equal(0, Coverage(empty));
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => Renderer.Render(new Scene(), Camera.Top, cancellationToken: cancelled.Token));
    }
}
