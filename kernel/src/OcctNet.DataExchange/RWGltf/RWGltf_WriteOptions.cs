// New in OCCT.NET (unit 8T): glTF written fresh over the assembly tree (decisions 1166, 1233). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Tools;

namespace Occt;

/// <summary>The options of <see cref="RWGltf_TreeWriter"/>.</summary>
public sealed record RWGltf_WriteOptions
{
    /// <summary>The defaults.</summary>
    public static RWGltf_WriteOptions Default { get; } = new();

    /// <summary>
    /// The size of the model's length unit in metres (glTF's unit); null to take the tree's
    /// <see cref="XdeDoc_Tree.LengthUnit"/>, or millimetres if it has none.
    /// </summary>
    public double? InputLengthUnit { get; init; }

    /// <summary>Turn OCCT's Z-up system into glTF's Y-up one (OCCT's default too).</summary>
    public bool ZUpToYUp { get; init; } = true;

    /// <summary>Write normals: the triangulations', or computed from the faces' surfaces where they have none.</summary>
    public bool WriteNormals { get; init; } = true;

    /// <summary>
    /// Write a part's faces of a colour as one primitive (the default); false writes a primitive per face, as OCCT's
    /// RWGltf_CafWriter does without its merge option, so that a reader gives the faces back (unit 8G1).
    /// </summary>
    public bool MergeFaces { get; init; } = true;

    /// <summary>Write hidden parts too (by default they are left out).</summary>
    public bool IncludeHidden { get; init; }

    /// <summary>
    /// If set, the tree is meshed first with these parameters (<see cref="XdeDoc_Tree.Meshed"/>; faces already meshed
    /// are kept); otherwise only the faces' existing triangulations are written.
    /// </summary>
    public ToolParameters? MeshParameters { get; init; }

    /// <summary>The asset's generator.</summary>
    public string Generator { get; init; } = "OCCT.NET";
}
