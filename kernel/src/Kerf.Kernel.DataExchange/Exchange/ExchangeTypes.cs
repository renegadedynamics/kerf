// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Drawing;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Exchange;

/// <summary>
/// A node of a model's assembly tree, as the exchange formats read and write it: a part (a <see cref="Shape"/>, or a
/// <see cref="Mesh"/> for the mesh formats) or an assembly of <see cref="Children"/>, with a name and a colour.
/// </summary>
/// <remarks>
/// A part's shape carries its own placement (a part read from STEP is placed where its instance puts it); an
/// assembly's <see cref="Placement"/> places its children in its parent. A null <see cref="Color"/> inherits the
/// parent's.
/// </remarks>
public sealed record ModelNode
{
    /// <summary>The name, or null.</summary>
    public string? Name { get; init; }

    /// <summary>The colour, or null to inherit the parent's.</summary>
    public Color? Color { get; init; }

    /// <summary>An assembly's placement in its parent; the identity for a part, whose shape is placed itself.</summary>
    public Placement Placement { get; init; } = new();

    /// <summary>A part's shape; null for an assembly or a mesh.</summary>
    public Shape? Shape { get; init; }

    /// <summary>A part's mesh, as the mesh formats (STL, glTF) read one; null for an assembly or a shape.</summary>
    public ShapeMesh? Mesh { get; init; }

    /// <summary>An assembly's children; empty for a part.</summary>
    /// <exception cref="ArgumentException">The tree would be nested more than <see cref="MaxNesting"/> deep.</exception>
    public ImmutableArray<ModelNode> Children
    {
        get;
        init
        {
            field = value;
            int nesting = 0;
            foreach (ModelNode child in value)
            {
                nesting = Math.Max(nesting, child._nesting + 1);
            }

            if (nesting > MaxNesting)
            {
                throw new KernelArgumentException($"Assemblies are nested more than {MaxNesting} deep.", nameof(value));
            }

            _nesting = nesting;
        }
    } = [];

    /// <summary>
    /// The most levels of assembly a model can have (decision 1854): the writers recurse once a level, a stack overflow
    /// ends the process, and on a 1 MB stack they fail at about 120. No real model comes near it.
    /// </summary>
    public const int MaxNesting = 64;

    private readonly int _nesting;

    /// <summary>True for a part: a shape or a mesh.</summary>
    public bool IsPart => Shape is not null || Mesh is not null;

    /// <summary>A part of a shape (placed by its own placement).</summary>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    public static ModelNode Part(Shape shape, string? name = null, Color? color = null)
        => shape.IsNull ? throw new KernelArgumentException("The shape is null.", nameof(shape)) : new() { Shape = shape, Name = name, Color = color };

    /// <summary>A part of a mesh.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="mesh"/> is null.</exception>
    public static ModelNode Part(ShapeMesh mesh, string? name = null, Color? color = null)
        => new() { Mesh = mesh ?? throw new KernelArgumentNullException(nameof(mesh)), Name = name, Color = color };

    /// <summary>An assembly of children at a placement.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="children"/> is null.</exception>
    public static ModelNode Assembly(IEnumerable<ModelNode> children, string? name = null, Placement? placement = null, Color? color = null)
    {
        KernelArgumentNullException.ThrowIfNull(children);
        ImmutableArray<ModelNode> list = [.. children];
        if (list.Any(c => c is null))
        {
            throw new KernelArgumentException("A child is null.", nameof(children));
        }

        return new() { Children = list, Name = name, Placement = placement ?? new Placement(), Color = color };
    }

    /// <summary>The parts' shapes placed, in one compound (a part's shape itself); meshes are left out.</summary>
    public Shape ToShape()
    {
        if (Shape is { } shape)
        {
            return shape;
        }

        var shapes = new List<Shape>();
        Collect(this, new Placement(), shapes);
        return Compound.Create(shapes);
    }

    /// <summary>
    /// The model as a scene for <see cref="Renderer"/>: each part meshed (a shape with <paramref name="meshOptions"/>,
    /// or a deflection of a thousandth of its size) at its place, in its colour or its nearest ancestor's
    /// (<paramref name="defaultColor"/>, light grey by default, where none has one).
    /// </summary>
    /// <returns>The scene, or the first part's meshing failure.</returns>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public Result<Scene> ToScene(MeshOptions? meshOptions = null, Color? defaultColor = null, CancellationToken cancellationToken = default)
    {
        var items = new List<SceneItem>();
        var warnings = new List<Diagnostic>();
        Diagnostic? failure = Collect(this, new Placement(), defaultColor ?? SceneItem.DefaultColor, meshOptions, items, warnings, cancellationToken);
        return failure is not null ? Result<Scene>.Failure(failure, [.. warnings]) : Result<Scene>.Success(new Scene(items), [.. warnings]);
    }

    private static Diagnostic? Collect(ModelNode node, Placement placement, Color color, MeshOptions? meshOptions, List<SceneItem> items,
                                       List<Diagnostic> warnings, CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        color = node.Color ?? color;
        if (node.Shape is { } shape)
        {
            Shape placed = placement.IsIdentity ? shape : global::Kerf.Engine.Construction.Api.Transform.Placed(shape, placement);
            Result<ShapeMesh> mesh = Mesher.Mesh(placed, meshOptions ?? Renderer.DefaultMeshOptions(placed), null, cancellationToken);
            warnings.AddRange(mesh.Diagnostics.Where(d => !d.IsError));
            if (!mesh.TryGetValue(out ShapeMesh? value))
            {
                return mesh.Error;
            }

            items.Add(new SceneItem(value, color));
            return null;
        }

        if (node.Mesh is { } part)
        {
            items.Add(new SceneItem(part, color) { Transform = placement.Transformation });
            return null;
        }

        Placement inner = placement * node.Placement;
        foreach (ModelNode child in node.Children)
        {
            if (Collect(child, inner, color, meshOptions, items, warnings, cancellationToken) is { } failure)
            {
                return failure;
            }
        }

        return null;
    }

    private static void Collect(ModelNode node, Placement placement, List<Shape> shapes)
    {
        if (node.Shape is { } shape)
        {
            // a placement with a scale made part of the geometry (decision 2452)
            shapes.Add(placement.IsIdentity ? shape : global::Kerf.Engine.Construction.Api.Transform.Placed(shape, placement));
            return;
        }

        Placement inner = placement * node.Placement;
        foreach (ModelNode child in node.Children)
        {
            Collect(child, inner, shapes);
        }
    }
}

/// <summary>A length unit of an exchange format.</summary>
public enum LengthUnit
{
    /// <summary>The millimetre.</summary>
    Millimetre,

    /// <summary>The micrometre.</summary>
    Micrometre,

    /// <summary>The centimetre.</summary>
    Centimetre,

    /// <summary>The metre.</summary>
    Metre,

    /// <summary>The inch.</summary>
    Inch,

    /// <summary>The foot.</summary>
    Foot,
}

/// <summary>The conversions of <see cref="LengthUnit"/>.</summary>
public static class LengthUnits
{
    /// <summary>The unit's size in millimetres.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Not one of the enum's units.</exception>
    public static double Millimetres(this LengthUnit unit) => unit switch
    {
        LengthUnit.Millimetre => 1.0,
        LengthUnit.Micrometre => 0.001,
        LengthUnit.Centimetre => 10.0,
        LengthUnit.Metre => 1000.0,
        LengthUnit.Inch => 25.4,
        LengthUnit.Foot => 304.8,
        _ => throw new KernelArgumentOutOfRangeException(nameof(unit), unit, "Not a length unit."),
    };
}

/// <summary>
/// What a reader refuses rather than allocate for, reading data that may be hostile (bento-integration.md, section 7).
/// Every reader takes the limits that apply to its format.
/// </summary>
public sealed record ReadLimits
{
    /// <summary>A gigabyte, 20 million STEP records, 50 million triangles: any reasonable model.</summary>
    public static ReadLimits Default { get; } = new();

    /// <summary>The most bytes read from the stream (every format).</summary>
    public long MaxBytes { get; init; } = 1L << 30;

    /// <summary>The most STEP records.</summary>
    public int MaxEntities { get; init; } = 20_000_000;

    /// <summary>The deepest nesting: STEP's parameter lists, glTF's node hierarchy.</summary>
    public int MaxDepth { get; init; } = 1_000;

    /// <summary>The most triangles (STL, glTF).</summary>
    public long MaxTriangles { get; init; } = 50_000_000;

    /// <summary>The most glTF nodes.</summary>
    public int MaxNodes { get; init; } = 1_000_000;
}

/// <summary>How STEP is read.</summary>
public sealed record StepReadOptions
{
    /// <summary>The default limits; product names; no external references.</summary>
    public static StepReadOptions Default { get; } = new();

    /// <summary>What this read refuses rather than allocate for.</summary>
    public ReadLimits Limits { get; init; } = ReadLimits.Default;

    /// <summary>
    /// Heal the shapes read (OCCT's shape processing after translation, <c>ShapeFix_Shape</c> with STEP's parameters);
    /// false gives the shapes as the file describes them, which may be invalid.
    /// </summary>
    public bool Heal { get; init; } = true;

    /// <summary>Name the nodes by their products (else their instances), as build123d names STEP's components.</summary>
    public bool ProductNames { get; init; } = true;

    /// <summary>
    /// Read the other STEP files the data refers to, from the file's folder and only from inside it (with
    /// <c>Step.Read(path)</c>); otherwise data that refers to other files is refused.
    /// </summary>
    public bool ExternalReferences { get; init; }
}

/// <summary>The precision STEP records in its uncertainty (OCCT's <c>write.precision.mode</c>).</summary>
public enum StepPrecision
{
    /// <summary>The least tolerance of the shapes.</summary>
    Least = -1,

    /// <summary>The average tolerance (OCCT's default).</summary>
    Average = 0,

    /// <summary>The greatest tolerance.</summary>
    Greatest = 1,

    /// <summary>OCCT's session value (<c>write.precision.val</c>).</summary>
    Session = 2,
}

/// <summary>How STEP is written (AP214).</summary>
public sealed record StepWriteOptions
{
    /// <summary>Millimetres, with pcurves, average precision.</summary>
    public static StepWriteOptions Default { get; } = new();

    /// <summary>The shapes' unit, which the file is written in.</summary>
    public LengthUnit Unit { get; init; } = LengthUnit.Millimetre;

    /// <summary>Write the edges' curves on their faces (OCCT's <c>write.surfacecurve.mode</c>).</summary>
    public bool WritePcurves { get; init; } = true;

    /// <summary>The precision recorded.</summary>
    public StepPrecision Precision { get; init; } = StepPrecision.Average;

    /// <summary>The application protocol written (OCCT's <c>write.step.schema</c>).</summary>
    public StepSchema Schema { get; init; } = StepSchema.Ap214;

    /// <summary>
    /// Merge equal points, directions, lines, planes, circles and placements after the transfer, and remove the duplicates,
    /// for a smaller file holding the same shapes (OCCT's <c>write.cleanduplicates</c>). Off by default, as in OCCT.
    /// </summary>
    public bool CleanDuplicates { get; init; }
}

/// <summary>The STEP application protocols written.</summary>
public enum StepSchema
{
    /// <summary>AP214 (automotive design, the international standard): OCCT's default, colours and layers.</summary>
    Ap214,

    /// <summary>AP203 (configuration-controlled design): shapes only, no colours.</summary>
    Ap203,

    /// <summary>AP242 (managed model-based 3D engineering).</summary>
    Ap242,
}

/// <summary>How STL is read.</summary>
public sealed record StlReadOptions
{
    /// <summary>The default limits, millimetres.</summary>
    public static StlReadOptions Default { get; } = new();

    /// <summary>The limits (bytes, triangles).</summary>
    public ReadLimits Limits { get; init; } = ReadLimits.Default;

    /// <summary>The data's unit, scaled to millimetres.</summary>
    public LengthUnit Unit { get; init; } = LengthUnit.Millimetre;
}

/// <summary>How STL is written.</summary>
public sealed record StlWriteOptions
{
    /// <summary>Binary STL, the compact form.</summary>
    public static StlWriteOptions Default { get; } = new();

    /// <summary>ASCII (with LF line ends) rather than binary.</summary>
    public bool Ascii { get; init; }
}

/// <summary>How glTF is read.</summary>
public sealed record GltfReadOptions
{
    /// <summary>The default limits, millimetres.</summary>
    public static GltfReadOptions Default { get; } = new();

    /// <summary>The limits (bytes, nodes, depth, triangles).</summary>
    public ReadLimits Limits { get; init; } = ReadLimits.Default;

    /// <summary>The unit the model is read into (glTF is in metres, Y up; the reader turns it Z up).</summary>
    public LengthUnit Unit { get; init; } = LengthUnit.Millimetre;
}

/// <summary>How glTF is written.</summary>
public sealed record GltfWriteOptions
{
    /// <summary>Binary (<c>.glb</c>), millimetres, the default mesh.</summary>
    public static GltfWriteOptions Default { get; } = new();

    /// <summary>Binary (<c>.glb</c>); otherwise JSON with the buffer embedded as a <c>data:</c> URI, since one stream
    /// can't carry the <c>.bin</c> file beside it.</summary>
    public bool Binary { get; init; } = true;

    /// <summary>The shapes' unit (glTF is in metres).</summary>
    public LengthUnit Unit { get; init; } = LengthUnit.Millimetre;

    /// <summary>How the shapes of the parts are meshed (meshes are written as they are).</summary>
    public MeshOptions Mesh { get; init; } = MeshOptions.Default;

    /// <summary>The generator named in the file's asset.</summary>
    public string Generator { get; init; } = "Kerf";
}

/// <summary>How 3MF is written (3MF Core).</summary>
public sealed record ThreeMfWriteOptions
{
    /// <summary>Millimetres, the default mesh.</summary>
    public static ThreeMfWriteOptions Default { get; } = new();

    /// <summary>The shapes' unit.</summary>
    public LengthUnit Unit { get; init; } = LengthUnit.Millimetre;

    /// <summary>How the shapes of the parts are meshed.</summary>
    public MeshOptions Mesh { get; init; } = MeshOptions.Default;
}

/// <summary>How Wavefront OBJ is written.</summary>
public sealed record ObjWriteOptions
{
    /// <summary>Model units, Z up, with normals, the default mesh.</summary>
    public static ObjWriteOptions Default { get; } = new();

    /// <summary>Scale the coordinates by this.</summary>
    public double Scale { get; init; } = 1.0;

    /// <summary>Turn Z up into Y up.</summary>
    public bool YUp { get; init; }

    /// <summary>Write the nodes' normals.</summary>
    public bool Normals { get; init; } = true;

    /// <summary>How the shapes of the parts are meshed.</summary>
    public MeshOptions Mesh { get; init; } = MeshOptions.Default;
}

/// <summary>How BREP (OCCT's own format) is written.</summary>
public sealed record BrepWriteOptions
{
    /// <summary>Text, with any stored meshes, without normals.</summary>
    public static BrepWriteOptions Default { get; } = new();

    /// <summary>OCCT's binary format rather than its text.</summary>
    public bool Binary { get; init; }

    /// <summary>Write the meshes stored on the faces (a BREP file read with them).</summary>
    public bool WithTriangles { get; init; } = true;

    /// <summary>Write the meshes' normals too.</summary>
    public bool WithNormals { get; init; }
}
