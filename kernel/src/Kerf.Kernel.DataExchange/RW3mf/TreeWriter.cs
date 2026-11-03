// New in OCCT.NET (unit 8T): 3MF written fresh over the assembly tree (decisions 1166, 1235); OCCT has no 3MF.
// The 3MF Core Specification 1.3: an OPC package (a zip) of the content types, the root relationship and the model
// part; objects with meshes or components, build items with transforms, colours as core base materials.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Mesh;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Occt;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using System.Xml;

namespace Kerf.Engine.Exchange.ThreeMf;

/// <summary>The options of <see cref="TreeWriter"/>.</summary>
public sealed record WriteOptions
{
    /// <summary>The defaults.</summary>
    public static WriteOptions Default { get; } = new();

    /// <summary>
    /// The size of the model's length unit in metres; null to take the tree's <see cref="Tree.LengthUnit"/>, or
    /// millimetres if it has none. A unit 3MF names (micron, millimetre, centimetre, inch, foot, metre) is written as
    /// is; another is converted to millimetres.
    /// </summary>
    public double? InputLengthUnit { get; init; }

    /// <summary>Write hidden parts too.</summary>
    public bool IncludeHidden { get; init; }

    /// <summary>If set, the tree is meshed first (<see cref="Tree.Meshed"/>).</summary>
    public ToolParameters? MeshParameters { get; init; }
}

/// <summary>
/// Writes an assembly tree as a 3MF package.
/// </summary>
/// <remarks>
/// <para>
/// Each part becomes a mesh object (its faces' triangulations, in the part's coordinates), each assembly an object of
/// components with their transforms, each root a build item; objects are written before the objects using them, as
/// the specification requires. Colours are core base materials (sRGB with alpha): an object's default material is its
/// instance's colour, and triangles of faces with a sub-shape colour name theirs. A product placed with different
/// colours becomes an object per colour; parts without triangles are left out.
/// </para>
/// <para>
/// The package is deterministic (fixed entry times). Streams first; the stream must be writable (and is left open).
/// The tree isn't changed.
/// </para>
/// </remarks>
public sealed class TreeWriter
{
    /// <summary>The namespace of the 3MF core model.</summary>
    public const string CoreNamespace = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";

    private const string ContentTypesNamespace = "http://schemas.openxmlformats.org/package/2006/content-types";
    private const string RelationshipsNamespace = "http://schemas.openxmlformats.org/package/2006/relationships";
    private const string ModelRelationshipType = "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel";
    private static readonly DateTimeOffset EntryTime = new(1980, 1, 1, 0, 0, 0, TimeSpan.Zero);

    /// <summary>A writer with the options (the defaults if null).</summary>
    public TreeWriter(WriteOptions? options = null) => Options = options ?? WriteOptions.Default;

    /// <summary>The options.</summary>
    public WriteOptions Options { get; }

    /// <summary>Writes the tree as a 3MF package to the stream. False if cancelled or the stream fails.</summary>
    public bool Write(Tree tree, Stream stream, ProgressRange progress = default)
    {
        ArgumentNullException.ThrowIfNull(tree);
        ArgumentNullException.ThrowIfNull(stream);
        using var ps = new ProgressScope(progress, "Writing 3MF", Options.MeshParameters is null ? 1 : 2);
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
        byte[]? model = BuildModel(aTree, ps.Next());
        if (model is null)
        {
            return false;
        }

        try
        {
            using var zip = new ZipArchive(stream, ZipArchiveMode.Create, true);
            WriteEntry(zip, "[Content_Types].xml", ContentTypes());
            WriteEntry(zip, "_rels/.rels", Relationships());
            WriteEntry(zip, "3D/3dmodel.model", model);
        }
        catch (IOException)
        {
            return false;
        }
        return true;
    }

    /// <summary>Writes the tree to a 3MF file.</summary>
    public bool Perform(Tree tree, string filePath, ProgressRange progress = default)
    {
        using FileStream file = File.Create(filePath);
        return Write(tree, file, progress);
    }

    private static void WriteEntry(ZipArchive zip, string name, byte[] data)
    {
        ZipArchiveEntry entry = zip.CreateEntry(name, CompressionLevel.Optimal);
        entry.LastWriteTime = EntryTime;
        using Stream stream = entry.Open();
        stream.Write(data);
    }

    private static XmlWriterSettings Settings() => new()
    {
        Encoding = new UTF8Encoding(false),
        Indent = false,
        NewLineChars = "\n",
    };

    private static byte[] ContentTypes()
    {
        using var stream = new MemoryStream();
        using (var w = XmlWriter.Create(stream, Settings()))
        {
            w.WriteStartDocument();
            w.WriteStartElement("Types", ContentTypesNamespace);
            w.WriteStartElement("Default", ContentTypesNamespace);
            w.WriteAttributeString("Extension", "rels");
            w.WriteAttributeString("ContentType", "application/vnd.openxmlformats-package.relationships+xml");
            w.WriteEndElement();
            w.WriteStartElement("Default", ContentTypesNamespace);
            w.WriteAttributeString("Extension", "model");
            w.WriteAttributeString("ContentType", "application/vnd.ms-package.3dmanufacturing-3dmodel+xml");
            w.WriteEndElement();
            w.WriteEndElement();
            w.WriteEndDocument();
        }
        return stream.ToArray();
    }

    private static byte[] Relationships()
    {
        using var stream = new MemoryStream();
        using (var w = XmlWriter.Create(stream, Settings()))
        {
            w.WriteStartDocument();
            w.WriteStartElement("Relationships", RelationshipsNamespace);
            w.WriteStartElement("Relationship", RelationshipsNamespace);
            w.WriteAttributeString("Target", "/3D/3dmodel.model");
            w.WriteAttributeString("Id", "rel0");
            w.WriteAttributeString("Type", ModelRelationshipType);
            w.WriteEndElement();
            w.WriteEndElement();
            w.WriteEndDocument();
        }
        return stream.ToArray();
    }

    /// <summary>The 3MF unit name of a unit size in metres, or null if 3MF has none.</summary>
    internal static string? UnitName(double unitValue)
    {
        static bool near(double a, double b) => Math.Abs(a - b) <= 1e-9 * b;
        return near(unitValue, 1e-6) ? "micron"
             : near(unitValue, 0.001) ? "millimeter"
             : near(unitValue, 0.01) ? "centimeter"
             : near(unitValue, 0.0254) ? "inch"
             : near(unitValue, 0.3048) ? "foot"
             : near(unitValue, 1.0) ? "meter"
             : null;
    }

    private sealed class ObjectDef
    {
        public int Id;
        public string? Name;
        public ProductMesh? Mesh;
        public int DefaultMaterial = -1;
        public List<int>? GroupMaterials;
        public List<(int ObjectId, Transform3D? Trsf)>? Components;
    }

    private byte[]? BuildModel(Tree tree, ProgressRange progress)
    {
        double inputUnit = Options.InputLengthUnit ?? tree.LengthUnit?.UnitValue ?? 0.001;
        string? unitName = UnitName(inputUnit);
        double scale = 1.0;
        if (unitName is null)
        {
            unitName = "millimeter";
            scale = inputUnit / 0.001;
        }

        var materials = new List<ColorWithAlpha>();
        var materialIndex = new Dictionary<ColorWithAlpha, int>();
        int materialOf(ColorWithAlpha color)
        {
            if (!materialIndex.TryGetValue(color, out int index))
            {
                index = materials.Count;
                materials.Add(color);
                materialIndex.Add(color, index);
            }
            return index;
        }

        var objects = new List<ObjectDef>();
        var known = new Dictionary<(DocumentProduct, ColorWithAlpha?, ColorWithAlpha?), ObjectDef?>();
        var meshes = new Dictionary<DocumentProduct, ProductMesh>(ReferenceEqualityComparer.Instance);
        var build = new List<(int ObjectId, Transform3D? Trsf)>();
        int nextId = 2; // 1 is the base materials
        int nbParts = tree.Products().Count(p => !p.IsAssembly);
        using var ps = new ProgressScope(progress, "Objects", Math.Max(1, nbParts));
        bool isCancelled = false;

        ObjectDef? objectOf(DocumentProduct product, Style style)
        {
            ColorWithAlpha? aColor = style.SurfaceColor ?? style.CurveColor;
            if (known.TryGetValue((product, style.SurfaceColor, style.CurveColor), out ObjectDef? def))
            {
                return def;
            }
            if (product.IsAssembly)
            {
                var components = new List<(int, Transform3D?)>();
                foreach (Instance component in product.Components)
                {
                    Style aStyle = ComponentStyle(tree, component, style);
                    if (!aStyle.IsVisible && !Options.IncludeHidden)
                    {
                        continue;
                    }
                    ObjectDef? child = objectOf(component.Product, aStyle);
                    if (isCancelled)
                    {
                        return null;
                    }
                    if (child is not null)
                    {
                        components.Add((child.Id, component.Location.IsIdentity ? null : component.Location.Transformation()));
                    }
                }
                def = components.Count == 0 ? null : new ObjectDef { Name = product.Name, Components = components };
            }
            else
            {
                if (!meshes.TryGetValue(product, out ProductMesh? mesh))
                {
                    if (!ps.More())
                    {
                        isCancelled = true;
                        return null;
                    }
                    ps.Next();
                    mesh = ProductMesh.Make(product, false);
                    meshes.Add(product, mesh);
                }
                if (mesh.NbTriangles == 0)
                {
                    def = null;
                }
                else
                {
                    def = new ObjectDef { Name = product.Name, Mesh = mesh };
                    bool hasColor = aColor is not null || mesh.Groups.Any(g => g.Color is not null);
                    if (hasColor)
                    {
                        // a part with colours but none of its own takes white where its faces have none
                        def.DefaultMaterial = materialOf(aColor ?? new ColorWithAlpha(1.0f, 1.0f, 1.0f, 1.0f));
                        def.GroupMaterials = mesh.Groups.Select(g => g.Color is ColorWithAlpha c ? materialOf(c) : -1).ToList();
                    }
                }
            }
            if (def is not null)
            {
                def.Id = nextId++;
                objects.Add(def);
            }
            known[(product, style.SurfaceColor, style.CurveColor)] = def;
            return def;
        }

        foreach (Instance root in tree.Roots)
        {
            Style aStyle = ComponentStyle(tree, root, Style.Default);
            if (!aStyle.IsVisible && !Options.IncludeHidden)
            {
                continue;
            }
            ObjectDef? def = objectOf(root.Product, aStyle);
            if (isCancelled)
            {
                return null;
            }
            if (def is not null)
            {
                build.Add((def.Id, root.Location.IsIdentity ? null : root.Location.Transformation()));
            }
        }

        using var stream = new MemoryStream();
        using (var w = XmlWriter.Create(stream, Settings()))
        {
            w.WriteStartDocument();
            w.WriteStartElement("model", CoreNamespace);
            w.WriteAttributeString("unit", unitName);
            w.WriteAttributeString("xml", "lang", null, "en-US");
            w.WriteStartElement("metadata", CoreNamespace);
            w.WriteAttributeString("name", "Application");
            w.WriteString("OCCT.NET");
            w.WriteEndElement();

            w.WriteStartElement("resources", CoreNamespace);
            if (materials.Count > 0)
            {
                w.WriteStartElement("basematerials", CoreNamespace);
                w.WriteAttributeString("id", "1");
                for (int i = 0; i < materials.Count; i++)
                {
                    w.WriteStartElement("base", CoreNamespace);
                    w.WriteAttributeString("name", "mat_" + i.ToString(CultureInfo.InvariantCulture));
                    w.WriteAttributeString("displaycolor", DisplayColor(materials[i]));
                    w.WriteEndElement();
                }
                w.WriteEndElement();
            }
            foreach (ObjectDef anObject in objects)
            {
                w.WriteStartElement("object", CoreNamespace);
                w.WriteAttributeString("id", anObject.Id.ToString(CultureInfo.InvariantCulture));
                w.WriteAttributeString("type", "model");
                if (anObject.Name is not null)
                {
                    w.WriteAttributeString("name", anObject.Name);
                }
                if (anObject.DefaultMaterial >= 0)
                {
                    w.WriteAttributeString("pid", "1");
                    w.WriteAttributeString("pindex", anObject.DefaultMaterial.ToString(CultureInfo.InvariantCulture));
                }
                if (anObject.Mesh is ProductMesh mesh)
                {
                    WriteMesh(w, mesh, anObject, scale);
                }
                else
                {
                    w.WriteStartElement("components", CoreNamespace);
                    foreach ((int id, Transform3D? trsf) in anObject.Components!)
                    {
                        w.WriteStartElement("component", CoreNamespace);
                        w.WriteAttributeString("objectid", id.ToString(CultureInfo.InvariantCulture));
                        if (trsf is Transform3D t)
                        {
                            w.WriteAttributeString("transform", Transform(t, scale));
                        }
                        w.WriteEndElement();
                    }
                    w.WriteEndElement();
                }
                w.WriteEndElement();
            }
            w.WriteEndElement();

            w.WriteStartElement("build", CoreNamespace);
            foreach ((int id, Transform3D? trsf) in build)
            {
                w.WriteStartElement("item", CoreNamespace);
                w.WriteAttributeString("objectid", id.ToString(CultureInfo.InvariantCulture));
                if (trsf is Transform3D t)
                {
                    w.WriteAttributeString("transform", Transform(t, scale));
                }
                w.WriteEndElement();
            }
            w.WriteEndElement();
            w.WriteEndElement();
            w.WriteEndDocument();
        }
        return stream.ToArray();
    }

    private static Style ComponentStyle(Tree tree, Instance instance, Style parent)
    {
        Style style = parent.Merged(instance.Product.Attributes).Merged(instance.Attributes);
        if (tree.IsInHiddenLayer(instance.Product.Attributes) || tree.IsInHiddenLayer(instance.Attributes))
        {
            style = style with { IsVisible = false };
        }
        return style;
    }

    private static void WriteMesh(XmlWriter w, ProductMesh mesh, ObjectDef theObject, double scale)
    {
        w.WriteStartElement("mesh", CoreNamespace);
        w.WriteStartElement("vertices", CoreNamespace);
        foreach (ProductMesh.Group aGroup in mesh.Groups)
        {
            for (int i = 0; i < aGroup.NbNodes; i++)
            {
                w.WriteStartElement("vertex", CoreNamespace);
                w.WriteAttributeString("x", Number(aGroup.Positions[3 * i] * scale));
                w.WriteAttributeString("y", Number(aGroup.Positions[3 * i + 1] * scale));
                w.WriteAttributeString("z", Number(aGroup.Positions[3 * i + 2] * scale));
                w.WriteEndElement();
            }
        }
        w.WriteEndElement();
        w.WriteStartElement("triangles", CoreNamespace);
        int aBase = 0;
        for (int g = 0; g < mesh.Groups.Count; g++)
        {
            ProductMesh.Group aGroup = mesh.Groups[g];
            int material = theObject.GroupMaterials is null ? -1 : theObject.GroupMaterials[g];
            for (int t = 0; t < aGroup.NbTriangles; t++)
            {
                int a = aGroup.Indices[3 * t], b = aGroup.Indices[3 * t + 1], c = aGroup.Indices[3 * t + 2];
                if (a == b || b == c || a == c)
                {
                    continue; // 3MF: a triangle's three vertices must differ
                }
                w.WriteStartElement("triangle", CoreNamespace);
                w.WriteAttributeString("v1", (aBase + a).ToString(CultureInfo.InvariantCulture));
                w.WriteAttributeString("v2", (aBase + b).ToString(CultureInfo.InvariantCulture));
                w.WriteAttributeString("v3", (aBase + c).ToString(CultureInfo.InvariantCulture));
                if (material >= 0 && material != theObject.DefaultMaterial)
                {
                    w.WriteAttributeString("pid", "1");
                    w.WriteAttributeString("p1", material.ToString(CultureInfo.InvariantCulture));
                }
                w.WriteEndElement();
            }
            aBase += aGroup.NbNodes;
        }
        w.WriteEndElement();
        w.WriteEndElement();
    }

    /// <summary>A colour as 3MF writes it: "#RRGGBB" (sRGB), with "AA" if not opaque.</summary>
    internal static string DisplayColor(in ColorWithAlpha color)
    {
        color.GetRGB().Values(out double r, out double g, out double b, ColorSpace.TOC_sRGB);
        static int toByte(double v) => (int)Math.Round(Math.Clamp(v, 0.0, 1.0) * 255.0);
        string hex = string.Create(CultureInfo.InvariantCulture, $"#{toByte(r):X2}{toByte(g):X2}{toByte(b):X2}");
        float alpha = color.Alpha();
        return alpha >= 1.0f ? hex : hex + toByte(alpha).ToString("X2", CultureInfo.InvariantCulture);
    }

    /// <summary>
    /// A transformation as 3MF writes it: "m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32", applied to row vectors
    /// (the transpose of OCCT's matrix), the translation scaled.
    /// </summary>
    internal static string Transform(in Transform3D trsf, double scale)
    {
        double[] values =
        [
            trsf.Value(1, 1), trsf.Value(2, 1), trsf.Value(3, 1),
            trsf.Value(1, 2), trsf.Value(2, 2), trsf.Value(3, 2),
            trsf.Value(1, 3), trsf.Value(2, 3), trsf.Value(3, 3),
            trsf.Value(1, 4) * scale, trsf.Value(2, 4) * scale, trsf.Value(3, 4) * scale,
        ];
        return string.Join(" ", values.Select(Number));
    }

    internal static string Number(double value) => value.ToString("R", CultureInfo.InvariantCulture);
}
