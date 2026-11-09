// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Drawing;

/// <summary>How faces are shaded.</summary>
public enum Shading
{
    /// <summary>Each triangle in one shade, from its own normal: facets show.</summary>
    Flat,

    /// <summary>The mesh's normals interpolated across each triangle: curved faces look smooth.</summary>
    Smooth,
}

/// <summary>Which of the meshes' edges <see cref="Renderer"/> draws over the faces.</summary>
public enum EdgeDisplay
{
    /// <summary>No edges: the faces alone.</summary>
    None,

    /// <summary>The shape's edges where its faces meet at more than <see cref="RenderOptions.FeatureAngle"/>, and free
    /// edges: not seams, nor the tangent edges of fillets.</summary>
    Feature,

    /// <summary>All the shape's edges, seams included.</summary>
    All,
}

/// <summary>A light shining from a direction, given in the view's frame (X right, Y up, Z towards the eye), so it moves
/// with the camera.</summary>
/// <param name="Direction">The direction towards the light.</param>
/// <param name="Intensity">Its strength: 1 lights a face facing it fully.</param>
public sealed record DirectionalLight(Vector3D Direction, double Intensity);

/// <summary>A mesh to render, with its colours and its placement.</summary>
public sealed record SceneItem
{
    /// <summary>A mesh in one colour (light grey by default), unmoved.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="mesh"/> is null.</exception>
    public SceneItem(ShapeMesh mesh, Color? color = null)
    {
        Mesh = mesh ?? throw new KernelArgumentNullException(nameof(mesh));
        Color = color ?? DefaultColor;
    }

    /// <summary>Light grey.</summary>
    public static Color DefaultColor { get; } = new(0.8, 0.8, 0.8);

    /// <summary>The mesh.</summary>
    public ShapeMesh Mesh { get; init; }

    /// <summary>The colour of its faces; an opacity under 1 makes them, and its edges, see-through (decision 1998).</summary>
    public Color Color { get; init; }

    /// <summary>Colours of single faces, by their index in <see cref="ShapeMesh.Faces"/>.</summary>
    public ImmutableDictionary<int, Color> FaceColors { get; init; } = ImmutableDictionary<int, Color>.Empty;

    /// <summary>Where the mesh is moved to.</summary>
    public Transform3D Transform { get; init; } = Transform3D.Identity;

    /// <summary>A copy with the face's colour (the face found in the mesh as itself, either orientation).</summary>
    /// <exception cref="ArgumentException">The face isn't one of the mesh's.</exception>
    public SceneItem WithFaceColor(Face face, Color color)
    {
        for (int i = 0; i < Mesh.Faces.Count; i++)
        {
            if (!Mesh.Faces[i].Face.AsShape().IsNull && Mesh.Faces[i].Face.IsSame(face))
            {
                return this with { FaceColors = FaceColors.SetItem(i, color) };
            }
        }

        throw new KernelArgumentException("The face isn't one of the mesh's.", nameof(face));
    }
}

/// <summary>What <see cref="Renderer"/> draws: meshes, each with its colours and placement.</summary>
public sealed record Scene
{
    /// <summary>A scene of the items.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="items"/> or an item is null.</exception>
    public Scene(IEnumerable<SceneItem> items)
    {
        KernelArgumentNullException.ThrowIfNull(items);
        Items = [.. items];
        if (Items.Any(i => i is null))
        {
            throw new KernelArgumentNullException(nameof(items), "An item is null.");
        }
    }

    /// <summary>A scene of the items.</summary>
    public Scene(params SceneItem[] items)
        : this((IEnumerable<SceneItem>)items)
    {
    }

    /// <summary>The items, drawn in order (a nearer face hides a farther one whatever the order).</summary>
    public ImmutableArray<SceneItem> Items { get; }
}

/// <summary>How <see cref="Renderer"/> draws.</summary>
public sealed record RenderOptions
{
    /// <summary>800 × 600, white, smooth shading, feature edges, two lights, supersampled 2 × 2.</summary>
    public static RenderOptions Default { get; } = new();

    /// <summary>The picture's width in pixels (1 to 16,384).</summary>
    public int Width { get; init => field = Size(value); } = 800;

    /// <summary>The picture's height in pixels (1 to 16,384).</summary>
    public int Height { get; init => field = Size(value); } = 600;

    /// <summary>The background; its opacity is the picture's where nothing is drawn (zero: transparent).</summary>
    public Color Background { get; init; } = new(1, 1, 1);

    /// <summary>Flat or smooth.</summary>
    public Shading Shading { get; init; } = Shading.Smooth;

    /// <summary>The lights, in the view's frame.</summary>
    public ImmutableArray<DirectionalLight> Lights { get; init; } =
        [new(new Vector3D(-0.35, 0.55, 1.0), 0.65), new(new Vector3D(0.6, -0.25, 0.6), 0.2)];

    /// <summary>The light everywhere, whatever a face's direction: 0 to 1.</summary>
    public double Ambient
    {
        get;
        init => field = value is >= 0 and <= 1 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The ambient light must be from 0 to 1.");
    } = 0.3;

    /// <summary>Which edges are drawn over the faces.</summary>
    public EdgeDisplay Edges { get; init; } = EdgeDisplay.Feature;

    /// <summary>The edges' colour.</summary>
    public Color EdgeColor { get; init; } = new(0.1, 0.1, 0.1);

    /// <summary>The edges' width in pixels.</summary>
    public double EdgeWidth
    {
        get;
        init => field = value > 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The width must be more than zero.");
    } = 1.0;

    /// <summary>The angle in degrees between faces above which their edge is a feature edge.</summary>
    public double FeatureAngle
    {
        get;
        init => field = value is >= 0 and < 180 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The angle must be from 0 to 180 degrees.");
    } = 20;

    /// <summary>Anti-aliasing: each pixel is the mean of this many samples squared (1, none, to 4).</summary>
    public int Supersampling
    {
        get;
        init => field = value is >= 1 and <= 4 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The supersampling must be from 1 to 4.");
    } = 2;

    /// <summary>Pixels per model unit at the target; null to fit the scene in the picture, inside the margin.</summary>
    public double? Scale
    {
        get;
        init => field = value is null or > 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The scale must be more than zero.");
    }

    /// <summary>The margin left when fitting, as a fraction of the picture's size (0 to 0.45).</summary>
    public double Margin
    {
        get;
        init => field = value is >= 0 and <= 0.45 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The margin must be from 0 to 0.45.");
    } = 0.05;

    /// <summary>Rasterise bands of rows in parallel: the picture is the serial one, bit for bit.</summary>
    public bool Parallel { get; init; } = true;

    private static int Size(int value)
        => value is >= 1 and <= 16384 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The size must be from 1 to 16,384 pixels.");
}
