// New in OCCT.NET (unit 8T): glTF read fresh into the assembly tree (decisions 1166, 1236). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Limits on the glTF data <see cref="RWGltf_TreeReader"/> accepts, for hostile input (phase8-plan.md 2.7). Past a
/// limit the reading stops with an error and gives no tree. <see cref="None"/> accepts anything;
/// <see cref="Default"/> is generous for real models.
/// </summary>
public sealed record RWGltf_ReadLimits
{
    /// <summary>No limits (but the JSON nesting, which System.Text.Json always bounds).</summary>
    public static RWGltf_ReadLimits None { get; } = new()
    {
        MaxBytes = long.MaxValue,
        MaxBufferBytes = long.MaxValue,
        MaxNodes = int.MaxValue,
        MaxDepth = int.MaxValue,
        MaxTriangles = long.MaxValue,
    };

    /// <summary>The defaults.</summary>
    public static RWGltf_ReadLimits Default { get; } = new();

    /// <summary>The largest file (the JSON or the .glb), in bytes. 1 GiB by default.</summary>
    public long MaxBytes { get; init; } = 1L << 30;

    /// <summary>The largest buffer, and the largest total of buffers, in bytes. 2 GiB by default.</summary>
    public long MaxBufferBytes { get; init; } = 2L << 30;

    /// <summary>The largest number of nodes. 1,000,000 by default.</summary>
    public int MaxNodes { get; init; } = 1_000_000;

    /// <summary>The deepest node hierarchy. 256 by default.</summary>
    public int MaxDepth { get; init; } = 256;

    /// <summary>The largest number of triangles, of all meshes together (each mesh once). 100 million by default.</summary>
    public long MaxTriangles { get; init; } = 100_000_000;

    /// <summary>The deepest JSON nesting. 64 by default.</summary>
    public int MaxJsonDepth { get; init; } = 64;
}

/// <summary>The options of <see cref="RWGltf_TreeReader"/>.</summary>
public sealed record RWGltf_ReadOptions
{
    /// <summary>The defaults.</summary>
    public static RWGltf_ReadOptions Default { get; } = new();

    /// <summary>The size in metres of the tree's length unit (glTF's is the metre): millimetres by default.</summary>
    public double OutputLengthUnit { get; init; } = 0.001;

    /// <summary>Turn glTF's Y-up system into OCCT's Z-up one (OCCT's default too).</summary>
    public bool YUpToZUp { get; init; } = true;

    /// <summary>The limits for hostile input.</summary>
    public RWGltf_ReadLimits Limits { get; init; } = RWGltf_ReadLimits.Default;
}
