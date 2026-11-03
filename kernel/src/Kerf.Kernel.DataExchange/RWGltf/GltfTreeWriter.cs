// New in OCCT.NET (unit 8T): glTF 2.0 written fresh over the assembly tree with System.Text.Json (decisions 1166, 1233).
// Not a port of OCCT's RWGltf_CafWriter, which needs XCAF and RapidJSON; its conventions are kept where they show in
// the output: a node per instance and a mesh per part shared by its instances, Z-up turned Y-up and the length unit
// turned into metres, a material per colour (PBR, metallic 0, roughness 1, the surface colour else the curve colour as
// base colour, blended if transparent).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Mesh;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Occt;
using System.Buffers.Binary;
using System.Text;
using System.Text.Json;

namespace Kerf.Engine.Exchange.Gltf;

/// <summary>
/// Writes an assembly tree as glTF 2.0: JSON with a separate binary buffer (<c>.gltf</c> and <c>.bin</c>), or a single
/// binary file (<c>.glb</c>).
/// </summary>
/// <remarks>
/// <para>
/// The scene has a node per root; a node per instance (its name the instance's or else its product's, its matrix the
/// instance's location); an assembly's node has its components' nodes as children, a part's node a mesh. Each part
/// is meshed once: its faces' triangulations (<see cref="GltfWriteOptions.MeshParameters"/> to mesh first) in a
/// primitive per sub-shape colour, the accessors shared by every instance, a mesh per part and instance colour.
/// Materials are PBR, base colour the style's surface colour (linear RGB with alpha, as glTF wants), else its curve
/// colour; parts without colour have no material (glTF's default).
/// </para>
/// <para>Streams first; the file path forms write next to each other. The tree isn't changed. Output is deterministic.</para>
/// </remarks>
public sealed class GltfTreeWriter
{
    private const int ARRAY_BUFFER = 34962;
    private const int ELEMENT_ARRAY_BUFFER = 34963;
    private const int Float = 5126;
    private const int UNSIGNED_SHORT = 5123;
    private const int UNSIGNED_INT = 5125;

    /// <summary>A writer with the options (the defaults if null).</summary>
    public GltfTreeWriter(GltfWriteOptions? options = null) => Options = options ?? GltfWriteOptions.Default;

    /// <summary>The options.</summary>
    public GltfWriteOptions Options { get; }

    /// <summary>Writes the tree as binary glTF (<c>.glb</c>). False if cancelled or the stream fails.</summary>
    public bool WriteGlb(Tree tree, Stream stream, ProgressRange progress = default)
    {
        ArgumentNullException.ThrowIfNull(tree);
        ArgumentNullException.ThrowIfNull(stream);
        Model? model = Build(tree, null, progress);
        if (model is null)
        {
            return false;
        }
        byte[] json = model.Json(false);
        int jsonLength = Pad4(json.Length);
        int binLength = Pad4((int)model.Bin.Length);
        bool hasBin = model.Bin.Length > 0;
        int total = 12 + 8 + jsonLength + (hasBin ? 8 + binLength : 0);
        try
        {
            Span<byte> header = stackalloc byte[12];
            BinaryPrimitives.WriteUInt32LittleEndian(header, 0x46546C67); // "glTF"
            BinaryPrimitives.WriteUInt32LittleEndian(header[4..], 2);
            BinaryPrimitives.WriteUInt32LittleEndian(header[8..], (uint)total);
            stream.Write(header);
            WriteChunk(stream, 0x4E4F534A, json, jsonLength, 0x20); // "JSON", padded with spaces
            if (hasBin)
            {
                WriteChunk(stream, 0x004E4942, model.Bin.ToArray(), binLength, 0); // "BIN\0"
            }
            stream.Flush();
        }
        catch (IOException)
        {
            return false;
        }
        return true;
    }

    /// <summary>
    /// Writes the tree as glTF JSON to <paramref name="json"/> and its buffer to <paramref name="bin"/>, which the
    /// JSON refers to as <paramref name="binUri"/> (a relative URI, such as "model.bin"). Without triangles nothing
    /// is written to <paramref name="bin"/> and the JSON has no buffer. False if cancelled or a stream fails.
    /// </summary>
    public bool WriteGltf(Tree tree, Stream json, Stream bin, string binUri, ProgressRange progress = default)
    {
        ArgumentNullException.ThrowIfNull(tree);
        ArgumentNullException.ThrowIfNull(json);
        ArgumentNullException.ThrowIfNull(bin);
        ArgumentNullException.ThrowIfNull(binUri);
        Model? model = Build(tree, binUri, progress);
        if (model is null)
        {
            return false;
        }
        try
        {
            json.Write(model.Json(true));
            json.Flush();
            if (model.Bin.Length > 0)
            {
                model.Bin.Position = 0;
                model.Bin.CopyTo(bin);
                bin.Flush();
            }
        }
        catch (IOException)
        {
            return false;
        }
        return true;
    }

    /// <summary>
    /// Writes the tree to a file: binary if its extension is <c>.glb</c>, otherwise JSON with the buffer in a
    /// <c>.bin</c> file of the same name beside it.
    /// </summary>
    public bool Perform(Tree tree, string filePath, ProgressRange progress = default)
    {
        if (string.Equals(Path.GetExtension(filePath), ".glb", StringComparison.OrdinalIgnoreCase))
        {
            using FileStream file = File.Create(filePath);
            return WriteGlb(tree, file, progress);
        }
        string binPath = Path.ChangeExtension(filePath, ".bin");
        using FileStream json = File.Create(filePath);
        using FileStream bin = File.Create(binPath);
        return WriteGltf(tree, json, bin, Uri.EscapeDataString(Path.GetFileName(binPath)), progress);
    }

    private static int Pad4(int length) => (length + 3) & ~3;

    private static void WriteChunk(Stream stream, uint type, byte[] data, int paddedLength, byte pad)
    {
        Span<byte> header = stackalloc byte[8];
        BinaryPrimitives.WriteUInt32LittleEndian(header, (uint)paddedLength);
        BinaryPrimitives.WriteUInt32LittleEndian(header[4..], type);
        stream.Write(header);
        stream.Write(data);
        for (int i = data.Length; i < paddedLength; i++)
        {
            stream.WriteByte(pad);
        }
    }

    // ---------------------------------------------------------------------------------------------- the model

    private sealed record Accessor(int BufferView, int ComponentType, int Count, string Type, float[]? Min, float[]? Max);

    private sealed record BufferView(int Offset, int Length, int Target);

    private sealed record Primitive(int Position, int Normal, int Indices, int Material);

    private sealed record Mesh(string? Name, List<Primitive> Primitives);

    private sealed class Node
    {
        public string? Name;
        public double[]? Matrix;
        public int Mesh = -1;
        public readonly List<int> Children = [];
    }

    private sealed class Model
    {
        public readonly MemoryStream Bin = new();
        public readonly List<BufferView> BufferViews = [];
        public readonly List<Accessor> Accessors = [];
        public readonly List<ColorWithAlpha> Materials = [];
        public readonly List<Mesh> Meshes = [];
        public readonly List<Node> Nodes = [];
        public readonly List<int> SceneNodes = [];
        public string? BinUri;
        public string Generator = "";

        public int AddView(ReadOnlySpan<byte> data, int target)
        {
            int offset = (int)Bin.Length;
            Bin.Write(data);
            while (Bin.Length % 4 != 0)
            {
                Bin.WriteByte(0);
            }
            BufferViews.Add(new BufferView(offset, data.Length, target));
            return BufferViews.Count - 1;
        }

        public byte[] Json(bool isIndented)
        {
            using var stream = new MemoryStream();
            using (var w = new Utf8JsonWriter(stream, new JsonWriterOptions { Indented = isIndented }))
            {
                w.WriteStartObject();
                w.WriteStartObject("asset");
                w.WriteString("generator", Generator);
                w.WriteString("version", "2.0");
                w.WriteEndObject();

                w.WriteNumber("scene", 0);
                w.WriteStartArray("scenes");
                w.WriteStartObject();
                w.WriteStartArray("nodes");
                foreach (int node in SceneNodes)
                {
                    w.WriteNumberValue(node);
                }
                w.WriteEndArray();
                w.WriteEndObject();
                w.WriteEndArray();

                if (Nodes.Count > 0)
                {
                    w.WriteStartArray("nodes");
                    foreach (Node node in Nodes)
                    {
                        w.WriteStartObject();
                        if (node.Name is not null)
                        {
                            w.WriteString("name", node.Name);
                        }
                        if (node.Matrix is not null)
                        {
                            w.WriteStartArray("matrix");
                            foreach (double value in node.Matrix)
                            {
                                w.WriteNumberValue(value);
                            }
                            w.WriteEndArray();
                        }
                        if (node.Mesh >= 0)
                        {
                            w.WriteNumber("mesh", node.Mesh);
                        }
                        if (node.Children.Count > 0)
                        {
                            w.WriteStartArray("children");
                            foreach (int child in node.Children)
                            {
                                w.WriteNumberValue(child);
                            }
                            w.WriteEndArray();
                        }
                        w.WriteEndObject();
                    }
                    w.WriteEndArray();
                }

                if (Meshes.Count > 0)
                {
                    w.WriteStartArray("meshes");
                    foreach (Mesh mesh in Meshes)
                    {
                        w.WriteStartObject();
                        if (mesh.Name is not null)
                        {
                            w.WriteString("name", mesh.Name);
                        }
                        w.WriteStartArray("primitives");
                        foreach (Primitive prim in mesh.Primitives)
                        {
                            w.WriteStartObject();
                            w.WriteStartObject("attributes");
                            if (prim.Normal >= 0)
                            {
                                w.WriteNumber("NORMAL", prim.Normal);
                            }
                            w.WriteNumber("POSITION", prim.Position);
                            w.WriteEndObject();
                            w.WriteNumber("indices", prim.Indices);
                            if (prim.Material >= 0)
                            {
                                w.WriteNumber("material", prim.Material);
                            }
                            w.WriteNumber("mode", 4);
                            w.WriteEndObject();
                        }
                        w.WriteEndArray();
                        w.WriteEndObject();
                    }
                    w.WriteEndArray();
                }

                if (Materials.Count > 0)
                {
                    w.WriteStartArray("materials");
                    for (int i = 0; i < Materials.Count; i++)
                    {
                        ColorWithAlpha color = Materials[i];
                        Color rgb = color.GetRGB();
                        w.WriteStartObject();
                        w.WriteString("name", "mat_" + i.ToString(System.Globalization.CultureInfo.InvariantCulture));
                        w.WriteStartObject("pbrMetallicRoughness");
                        w.WriteStartArray("baseColorFactor");
                        w.WriteNumberValue((float)rgb.Red());
                        w.WriteNumberValue((float)rgb.Green());
                        w.WriteNumberValue((float)rgb.Blue());
                        w.WriteNumberValue(color.Alpha());
                        w.WriteEndArray();
                        w.WriteNumber("metallicFactor", 0);
                        w.WriteNumber("roughnessFactor", 1);
                        w.WriteEndObject();
                        if (color.Alpha() < 1.0f)
                        {
                            w.WriteString("alphaMode", "BLEND");
                        }
                        w.WriteEndObject();
                    }
                    w.WriteEndArray();
                }

                if (Accessors.Count > 0)
                {
                    w.WriteStartArray("accessors");
                    foreach (Accessor accessor in Accessors)
                    {
                        w.WriteStartObject();
                        w.WriteNumber("bufferView", accessor.BufferView);
                        w.WriteNumber("componentType", accessor.ComponentType);
                        w.WriteNumber("count", accessor.Count);
                        w.WriteString("type", accessor.Type);
                        if (accessor.Min is not null && accessor.Max is not null)
                        {
                            w.WriteStartArray("min");
                            foreach (float value in accessor.Min)
                            {
                                w.WriteNumberValue(value);
                            }
                            w.WriteEndArray();
                            w.WriteStartArray("max");
                            foreach (float value in accessor.Max)
                            {
                                w.WriteNumberValue(value);
                            }
                            w.WriteEndArray();
                        }
                        w.WriteEndObject();
                    }
                    w.WriteEndArray();

                    w.WriteStartArray("bufferViews");
                    foreach (BufferView view in BufferViews)
                    {
                        w.WriteStartObject();
                        w.WriteNumber("buffer", 0);
                        w.WriteNumber("byteOffset", view.Offset);
                        w.WriteNumber("byteLength", view.Length);
                        w.WriteNumber("target", view.Target);
                        w.WriteEndObject();
                    }
                    w.WriteEndArray();

                    w.WriteStartArray("buffers");
                    w.WriteStartObject();
                    w.WriteNumber("byteLength", Bin.Length);
                    if (BinUri is not null)
                    {
                        w.WriteString("uri", BinUri);
                    }
                    w.WriteEndObject();
                    w.WriteEndArray();
                }
                w.WriteEndObject();
            }
            return stream.ToArray();
        }
    }

    // A part's accessors: per group, its position, normal and index accessors
    private sealed record PartAccessors(ProductMesh Mesh, List<(int Position, int Normal, int Indices)> Groups);

    private Model? Build(Tree tree, string? binUri, ProgressRange progress)
    {
        using var ps = new ProgressScope(progress, "Writing glTF", Options.MeshParameters is null ? 1 : 2);
        Tree aTree = tree;
        if (Options.MeshParameters is ToolParameters aParams)
        {
            Tree? meshed = tree.Meshed(aParams, ps.Next());
            if (meshed is null)
            {
                return null;
            }
            aTree = meshed;
        }

        double unit = Options.InputLengthUnit ?? aTree.LengthUnit?.UnitValue ?? 0.001;
        var converter = new CoordinateConverter(unit, Options.ZUpToYUp);
        var model = new Model { BinUri = binUri, Generator = Options.Generator };
        var parts = new Dictionary<DocumentProduct, PartAccessors>(ReferenceEqualityComparer.Instance);
        var meshes = new Dictionary<(DocumentProduct, ColorWithAlpha?), int>();
        var materials = new Dictionary<ColorWithAlpha, int>();
        int nbParts = aTree.Products().Count(p => !p.IsAssembly);
        using var partsPS = new ProgressScope(ps.Next(), "Meshes", Math.Max(1, nbParts));
        bool isCancelled = false;

        int materialOf(ColorWithAlpha? color)
        {
            if (color is not ColorWithAlpha aColor)
            {
                return -1;
            }
            if (!materials.TryGetValue(aColor, out int index))
            {
                index = model.Materials.Count;
                model.Materials.Add(aColor);
                materials.Add(aColor, index);
            }
            return index;
        }

        PartAccessors? AccessorsOf(DocumentProduct part)
        {
            if (parts.TryGetValue(part, out PartAccessors? known))
            {
                return known;
            }
            if (!partsPS.More())
            {
                isCancelled = true;
                return null;
            }
            partsPS.Next();
            ProductMesh mesh = ProductMesh.Make(part, Options.WriteNormals, Options.MergeFaces);
            var groups = new List<(int, int, int)>();
            foreach (ProductMesh.Group aGroup in mesh.Groups)
            {
                groups.Add(WriteGroup(model, aGroup, converter));
            }
            var result = new PartAccessors(mesh, groups);
            parts.Add(part, result);
            return result;
        }

        int MeshOf(DocumentProduct part, Style style)
        {
            ColorWithAlpha? nodeColor = style.SurfaceColor ?? style.CurveColor;
            if (meshes.TryGetValue((part, nodeColor), out int known))
            {
                return known;
            }
            PartAccessors? accessors = AccessorsOf(part);
            if (accessors is null || accessors.Groups.Count == 0)
            {
                meshes[(part, nodeColor)] = -1;
                return -1;
            }
            var primitives = new List<Primitive>();
            for (int i = 0; i < accessors.Groups.Count; i++)
            {
                (int pos, int normal, int indices) = accessors.Groups[i];
                ColorWithAlpha? aColor = accessors.Mesh.Groups[i].Color ?? nodeColor;
                primitives.Add(new Primitive(pos, normal, indices, materialOf(aColor)));
            }
            model.Meshes.Add(new Mesh(part.Name, primitives));
            int index = model.Meshes.Count - 1;
            meshes[(part, nodeColor)] = index;
            return index;
        }

        int visit(Instance instance, Style parentStyle)
        {
            DocumentProduct product = instance.Product;
            Style aStyle = parentStyle.Merged(product.Attributes).Merged(instance.Attributes);
            if (aTree.IsInHiddenLayer(product.Attributes) || aTree.IsInHiddenLayer(instance.Attributes))
            {
                aStyle = aStyle with { IsVisible = false };
            }
            if (!aStyle.IsVisible && !Options.IncludeHidden)
            {
                return -1;
            }
            var node = new Node
            {
                Name = instance.DisplayName,
                Matrix = instance.Location.IsIdentity ? null : converter.MapMatrix(instance.Location.Transformation()),
            };
            model.Nodes.Add(node);
            int index = model.Nodes.Count - 1;
            if (product.IsAssembly)
            {
                foreach (Instance component in product.Components)
                {
                    int child = visit(component, aStyle);
                    if (isCancelled)
                    {
                        return -1;
                    }
                    if (child >= 0)
                    {
                        node.Children.Add(child);
                    }
                }
            }
            else
            {
                node.Mesh = MeshOf(product, aStyle);
            }
            return index;
        }

        foreach (Instance root in aTree.Roots)
        {
            int node = visit(root, Style.Default);
            if (isCancelled)
            {
                return null;
            }
            if (node >= 0)
            {
                model.SceneNodes.Add(node);
            }
        }
        return model;
    }

    private static (int Position, int Normal, int Indices) WriteGroup(Model model, ProductMesh.Group theGroup, CoordinateConverter converter)
    {
        int nbNodes = theGroup.NbNodes;
        byte[] positions = new byte[nbNodes * 12];
        float[] min = [float.MaxValue, float.MaxValue, float.MaxValue];
        float[] max = [float.MinValue, float.MinValue, float.MinValue];
        for (int i = 0; i < nbNodes; i++)
        {
            double x = theGroup.Positions[3 * i], y = theGroup.Positions[3 * i + 1], z = theGroup.Positions[3 * i + 2];
            converter.Map(ref x, ref y, ref z);
            float[] p = [(float)x, (float)y, (float)z];
            for (int k = 0; k < 3; k++)
            {
                BinaryPrimitives.WriteSingleLittleEndian(positions.AsSpan(12 * i + 4 * k), p[k]);
                min[k] = Math.Min(min[k], p[k]);
                max[k] = Math.Max(max[k], p[k]);
            }
        }
        model.Accessors.Add(new Accessor(model.AddView(positions, ARRAY_BUFFER), Float, nbNodes, "VEC3", min, max));
        int position = model.Accessors.Count - 1;

        int normal = -1;
        if (theGroup.Normals.Count == theGroup.Positions.Count)
        {
            byte[] normals = new byte[nbNodes * 12];
            for (int i = 0; i < nbNodes; i++)
            {
                double x = theGroup.Normals[3 * i], y = theGroup.Normals[3 * i + 1], z = theGroup.Normals[3 * i + 2];
                converter.Map(ref x, ref y, ref z, true);
                double length = Math.Sqrt(x * x + y * y + z * z);
                if (length > 0.0)
                {
                    x /= length;
                    y /= length;
                    z /= length;
                }
                BinaryPrimitives.WriteSingleLittleEndian(normals.AsSpan(12 * i), (float)x);
                BinaryPrimitives.WriteSingleLittleEndian(normals.AsSpan(12 * i + 4), (float)y);
                BinaryPrimitives.WriteSingleLittleEndian(normals.AsSpan(12 * i + 8), (float)z);
            }
            model.Accessors.Add(new Accessor(model.AddView(normals, ARRAY_BUFFER), Float, nbNodes, "VEC3", null, null));
            normal = model.Accessors.Count - 1;
        }

        // glTF: an index may not be the type's largest value (primitive restart)
        bool isShort = nbNodes <= ushort.MaxValue;
        int nbIndices = theGroup.Indices.Count;
        byte[] indices = new byte[nbIndices * (isShort ? 2 : 4)];
        for (int i = 0; i < nbIndices; i++)
        {
            if (isShort)
            {
                BinaryPrimitives.WriteUInt16LittleEndian(indices.AsSpan(2 * i), (ushort)theGroup.Indices[i]);
            }
            else
            {
                BinaryPrimitives.WriteUInt32LittleEndian(indices.AsSpan(4 * i), (uint)theGroup.Indices[i]);
            }
        }
        model.Accessors.Add(new Accessor(model.AddView(indices, ELEMENT_ARRAY_BUFFER),
                                            isShort ? UNSIGNED_SHORT : UNSIGNED_INT, nbIndices, "SCALAR", null, null));
        return (position, normal, model.Accessors.Count - 1);
    }
}
