// New in Kerf (unit 8T): Wavefront OBJ written fresh over the assembly tree (decisions 1166, 1234). Not a port of
// OCCT's RWObj_CafWriter, which needs XCAF; its conventions are kept where they show: a group per placed part named by
// its path, MTL diffuse colours in sRGB, transparency as "Tr".
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Mesh;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using System.Globalization;
using System.Text;

namespace Kerf.Engine.Exchange.Obj;

/// <summary>The options of <see cref="ObjTreeWriter"/>.</summary>
internal sealed record ObjWriterOptions
{
    /// <summary>The defaults.</summary>
    public static ObjWriterOptions Default { get; } = new();

    /// <summary>The factor applied to coordinates (1: the model's unit).</summary>
    public double Scale { get; init; } = 1.0;

    /// <summary>Turn OCCT's Z-up system into the Y-up one most OBJ readers expect (off by default).</summary>
    public bool ZUpToYUp { get; init; }

    /// <summary>Write normals (<c>vn</c>): the triangulations', or computed from the surfaces.</summary>
    public bool WriteNormals { get; init; } = true;

    /// <summary>Write hidden parts too.</summary>
    public bool IncludeHidden { get; init; }

    /// <summary>If set, the tree is meshed first (<see cref="Tree.Meshed"/>).</summary>
    public ToolParameters? MeshParameters { get; init; }
}

/// <summary>
/// Writes an assembly tree as Wavefront OBJ, with its colours as an MTL material library.
/// </summary>
/// <remarks>
/// OBJ has no instances: each placed part (<see cref="Tree.Flatten"/>) is written in world coordinates as an
/// object (<c>o</c>) named by its path ("assembly/part"), with a <c>usemtl</c> per colour group. Materials are named
/// <c>mat_N</c>: <c>Kd</c> the colour in sRGB, <c>d</c> and <c>Tr</c> its opacity and transparency; faces without colour
/// after faces with one take a white material, <c>default</c>. Streams first; the
/// file form writes the MTL beside the OBJ. Numbers in the invariant culture, LF line ends. The tree isn't changed.
/// </remarks>
internal sealed class ObjTreeWriter
{
    /// <summary>A writer with the options (the defaults if null).</summary>
    public ObjTreeWriter(ObjWriterOptions? options = null) => Options = options ?? ObjWriterOptions.Default;

    /// <summary>The options.</summary>
    public ObjWriterOptions Options { get; }

    /// <summary>
    /// Writes the tree's OBJ to <paramref name="obj"/> and its materials to <paramref name="mtl"/> (if not null),
    /// which the OBJ names as <paramref name="mtlName"/>. False if cancelled or a stream fails.
    /// </summary>
    public bool Write(Tree tree, Stream obj, Stream? mtl, string mtlName = "model.mtl", ProgressRange progress = default)
    {
        ArgumentNullException.ThrowIfNull(tree);
        ArgumentNullException.ThrowIfNull(obj);
        using var ps = new ProgressScope(progress, "Writing OBJ", Options.MeshParameters is null ? 1 : 2);
        Tree aTree = tree;
        if (Options.MeshParameters is ToolParameters aParams)
        {
            Tree? meshed = tree.Meshed(aParams, ps.Next());
            if (meshed is null)
            {
                return false;
            }
            aTree = meshed;
        }

        var converter = new CoordinateConverter(Options.Scale, Options.ZUpToYUp);
        IReadOnlyList<FlatItem> items = aTree.Flatten(Options.IncludeHidden);
        var meshes = new Dictionary<DocumentProduct, ProductMesh>(ReferenceEqualityComparer.Instance);
        var materials = new List<ColorWithAlpha>();
        var materialIndex = new Dictionary<ColorWithAlpha, int>();
        var anObj = new StringBuilder();
        anObj.Append("# Kerf\n");
        if (mtl is not null)
        {
            anObj.Append("mtllib ").Append(mtlName).Append('\n');
        }

        int nodeBase = 1, normalBase = 1;
        string? currentMaterial = null;
        bool hasDefault = false;
        using var itemsPS = new ProgressScope(ps.Next(), "Parts", Math.Max(1, items.Count));
        foreach (FlatItem item in items)
        {
            if (!itemsPS.More())
            {
                return false;
            }
            itemsPS.Next();
            if (!meshes.TryGetValue(item.Product, out ProductMesh? mesh))
            {
                mesh = ProductMesh.Make(item.Product, Options.WriteNormals);
                meshes.Add(item.Product, mesh);
            }
            if (mesh.Groups.Count == 0)
            {
                continue;
            }
            Transform3D trsf = item.Location.Transformation;
            bool isIdentity = item.Location.IsIdentity;
            bool isMirrored = trsf.VectorialPart.Determinant < 0.0;
            anObj.Append("o ").Append(ObjName(item)).Append('\n');
            ColorWithAlpha? nodeColor = item.Style.SurfaceColor ?? item.Style.CurveColor;
            foreach (ProductMesh.Group aGroup in mesh.Groups)
            {
                for (int i = 0; i < aGroup.NbNodes; i++)
                {
                    var p = new Point3D(aGroup.Positions[3 * i], aGroup.Positions[3 * i + 1], aGroup.Positions[3 * i + 2]);
                    if (!isIdentity)
                    {
                        p = p.Transformed(trsf);
                    }
                    double x = p.X, y = p.Y, z = p.Z;
                    converter.Map(ref x, ref y, ref z);
                    anObj.Append("v ").Append(Num(x)).Append(' ').Append(Num(y)).Append(' ').Append(Num(z)).Append('\n');
                }
                bool hasNormals = aGroup.Normals.Count == aGroup.Positions.Count && aGroup.NbNodes > 0;
                if (hasNormals)
                {
                    for (int i = 0; i < aGroup.NbNodes; i++)
                    {
                        var n = new Direction3D(aGroup.Normals[3 * i], aGroup.Normals[3 * i + 1], aGroup.Normals[3 * i + 2]);
                        if (!isIdentity)
                        {
                            n = n.Transformed(trsf);
                        }
                        double x = n.X, y = n.Y, z = n.Z;
                        converter.Map(ref x, ref y, ref z, true);
                        anObj.Append("vn ").Append(Num((float)x)).Append(' ').Append(Num((float)y)).Append(' ').Append(Num((float)z)).Append('\n');
                    }
                }
                if (mtl is not null && (aGroup.Color ?? nodeColor) is ColorWithAlpha color)
                {
                    if (!materialIndex.TryGetValue(color, out int mat))
                    {
                        mat = materials.Count;
                        materials.Add(color);
                        materialIndex.Add(color, mat);
                    }
                    string name = "mat_" + mat.ToString(CultureInfo.InvariantCulture);
                    if (name != currentMaterial)
                    {
                        anObj.Append("usemtl ").Append(name).Append('\n');
                        currentMaterial = name;
                    }
                }
                else if (mtl is not null && currentMaterial is not null && currentMaterial != "default")
                {
                    // a group without colour after one with: OBJ keeps the material, so a neutral one is named
                    anObj.Append("usemtl default\n");
                    currentMaterial = "default";
                    hasDefault = true;
                }
                for (int t = 0; t < aGroup.NbTriangles; t++)
                {
                    int a = aGroup.Indices[3 * t], b = aGroup.Indices[3 * t + 1], c = aGroup.Indices[3 * t + 2];
                    if (isMirrored)
                    {
                        (b, c) = (c, b);
                    }
                    anObj.Append('f');
                    foreach (int node in new[] { a, b, c })
                    {
                        anObj.Append(' ').Append((nodeBase + node).ToString(CultureInfo.InvariantCulture));
                        if (hasNormals)
                        {
                            anObj.Append("//").Append((normalBase + node).ToString(CultureInfo.InvariantCulture));
                        }
                    }
                    anObj.Append('\n');
                }
                nodeBase += aGroup.NbNodes;
                if (hasNormals)
                {
                    normalBase += aGroup.NbNodes;
                }
            }
        }

        try
        {
            obj.Write(Encoding.UTF8.GetBytes(anObj.ToString()));
            obj.Flush();
            if (mtl is not null)
            {
                var aMtl = new StringBuilder("# Kerf\n");
                for (int i = 0; i < materials.Count; i++)
                {
                    LinearColor rgb = materials[i].GetRGB();
                    rgb.Values(out double r, out double g, out double aB, ColorSpace.TOC_sRGB);
                    float alpha = materials[i].Alpha();
                    aMtl.Append("newmtl mat_").Append(i.ToString(CultureInfo.InvariantCulture)).Append('\n');
                    aMtl.Append("Kd ").Append(Num((float)r)).Append(' ').Append(Num((float)g)).Append(' ').Append(Num((float)aB)).Append('\n');
                    aMtl.Append("d ").Append(Num(alpha)).Append('\n');
                    if (alpha < 1.0f)
                    {
                        aMtl.Append("Tr ").Append(Num(1.0f - alpha)).Append('\n');
                    }
                }
                if (hasDefault)
                {
                    aMtl.Append("newmtl default\nKd 1 1 1\nd 1\n");
                }
                mtl.Write(Encoding.UTF8.GetBytes(aMtl.ToString()));
                mtl.Flush();
            }
        }
        catch (IOException)
        {
            return false;
        }
        return true;
    }

    /// <summary>Writes the tree to an OBJ file, with its MTL file of the same name beside it.</summary>
    public bool Perform(Tree tree, string filePath, ProgressRange progress = default)
    {
        string mtlPath = Path.ChangeExtension(filePath, ".mtl");
        using FileStream obj = File.Create(filePath);
        using FileStream mtl = File.Create(mtlPath);
        return Write(tree, obj, mtl, Path.GetFileName(mtlPath), progress);
    }

    private static string ObjName(FlatItem item)
    {
        string name = string.Join("/", item.NamePath.Select(n => n ?? ""));
        var builder = new StringBuilder(name.Length);
        foreach (char aChar in name)
        {
            builder.Append(char.IsWhiteSpace(aChar) || char.IsControl(aChar) ? '_' : aChar);
        }
        return builder.Length == 0 ? "part" : builder.ToString();
    }

    private static string Num(double value) => value.ToString("R", CultureInfo.InvariantCulture);

    private static string Num(float value) => value.ToString("R", CultureInfo.InvariantCulture);
}
