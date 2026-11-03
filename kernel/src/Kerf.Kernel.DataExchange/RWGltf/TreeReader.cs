// New in OCCT.NET (unit 8T): glTF 2.0 read fresh into the assembly tree with System.Text.Json (decisions 1166, 1236).
// Not a port of OCCT's RWGltf_CafReader, which needs XCAF and RapidJSON; its result is kept: each primitive a face
// holding only its triangulation (BRep_Builder::MakeFace with a Poly_Triangulation, as RWMesh does), the meshes' names
// and colours, the node hierarchy, glTF's Y-up turned Z-up and the metres turned into the model's unit.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Mesh;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Buffers.Binary;
using System.Text;
using System.Text.Json;

namespace Kerf.Engine.Exchange.Gltf;

/// <summary>
/// Reads glTF 2.0 (<c>.gltf</c> with its buffers, or <c>.glb</c>) into an assembly tree.
/// </summary>
/// <remarks>
/// <para>
/// Each mesh becomes a part shared by the nodes using it: a face per triangle primitive (modes 4, 5 and 6; points and
/// lines are skipped with a warning) holding only its triangulation, with the primitive's normals; the part's colour
/// (one primitive) or its faces' colours (several) the materials' base colour factor (linear RGB with alpha). A node
/// with a mesh and no children is an instance of the mesh's part (named by the node if its name differs from the
/// mesh's); a node with children an assembly named by the node. A node's transformation that isn't rigid (a scale,
/// shear or mirror) is applied to the meshes below it (copied for that node), since shape locations are rigid
/// (decision 1236). The tree's roots are the default scene's nodes (all parentless nodes if there is no scene).
/// </para>
/// <para>
/// Hostile input (<see cref="ReadLimits"/>): the file and buffer sizes, the node count and depth, the triangle
/// count and the JSON nesting are bounded; every index and accessor range is checked against what it refers to; a
/// node may have one parent; required extensions not supported, sparse accessors and data that isn't 32-bit floats for
/// positions and normals are refused. An error stops the reading (<see cref="Error"/>) and gives no tree. External
/// buffers are resolved by the caller (<see cref="Read"/>); <see cref="ReadFile"/> resolves them in the file's
/// folder only. The tree's shapes are frozen. Cancellation through the progress range.
/// </para>
/// </remarks>
public sealed class TreeReader
{
    private const int Float = 5126;
    private const int UNSIGNED_BYTE = 5121;
    private const int UNSIGNED_SHORT = 5123;
    private const int UNSIGNED_INT = 5125;
    private const int HARD_MAX_DEPTH = 4096; // the node recursion's own bound, whatever the limits say

    private readonly List<string> _warnings = [];

    /// <summary>A reader with the options (the defaults if null).</summary>
    public TreeReader(ReadOptions? options = null) => Options = options ?? ReadOptions.Default;

    /// <summary>The options.</summary>
    public ReadOptions Options { get; }

    /// <summary>The last reading's error, or null.</summary>
    public string? Error { get; private set; }

    /// <summary>The last reading's warnings (what was skipped).</summary>
    public IReadOnlyList<string> Warnings => _warnings;

    /// <summary>
    /// Reads glTF or binary glTF (recognised from the data) from the stream; external buffers are opened by
    /// <paramref name="resolver"/> from their URIs (unescaped), and refused if it is null or gives null. Null on
    /// an error (see <see cref="Error"/>) or if cancelled.
    /// </summary>
    public Tree? Read(Stream stream, Func<string, Stream?>? resolver = null, ProgressRange progress = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        Error = null;
        _warnings.Clear();
        try
        {
            byte[] data = ReadAll(stream, Options.Limits.MaxBytes, "the file");
            var context = new Context(this, resolver);
            return context.Read(data, progress);
        }
        catch (FormatError error)
        {
            Error = error.Message;
            return null;
        }
        catch (JsonException error)
        {
            Error = "Invalid JSON: " + error.Message;
            return null;
        }
        catch (IOException error)
        {
            Error = "Read error: " + error.Message;
            return null;
        }
    }

    /// <summary>
    /// Reads a glTF file; its external buffers are read from files in its folder (or below it) only.
    /// </summary>
    public Tree? ReadFile(string filePath, ProgressRange progress = default)
    {
        string folder = Path.GetFullPath(Path.GetDirectoryName(Path.GetFullPath(filePath)) ?? ".");
        string prefix = folder.EndsWith(Path.DirectorySeparatorChar) ? folder : folder + Path.DirectorySeparatorChar;
        Stream? resolve(string uri)
        {
            if (Path.IsPathRooted(uri) || uri.Contains(':', StringComparison.Ordinal))
            {
                return null;
            }
            string path = Path.GetFullPath(Path.Combine(folder, uri));
            return path.StartsWith(prefix, StringComparison.OrdinalIgnoreCase) && File.Exists(path) ? File.OpenRead(path) : null;
        }
        using FileStream file = File.OpenRead(filePath);
        return Read(file, resolve, progress);
    }

    private sealed class FormatError(string theMessage) : Exception(theMessage);

    private static byte[] ReadAll(Stream stream, long max, string what)
    {
        if (stream.CanSeek && stream.Length - stream.Position > max)
        {
            throw new FormatError($"{what} is larger than the limit ({max} bytes)");
        }
        using var buffer = new MemoryStream();
        byte[] chunk = new byte[81920];
        int read;
        while ((read = stream.Read(chunk, 0, chunk.Length)) > 0)
        {
            if (buffer.Length + read > max)
            {
                throw new FormatError($"{what} is larger than the limit ({max} bytes)");
            }
            buffer.Write(chunk, 0, read);
        }
        return buffer.ToArray();
    }

    // ---------------------------------------------------------------------------------------------- the reading

    private sealed class Context(TreeReader theReader, Func<string, Stream?>? theResolver)
    {
        private readonly ReadLimits _limits = theReader.Options.Limits;
        private readonly CoordinateConverter _toModel =
            new CoordinateConverter(theReader.Options.OutputLengthUnit, theReader.Options.YUpToZUp).Inverse();
        private JsonElement _root;
        private byte[]? _glbBin;
        private readonly Dictionary<int, byte[]> _buffers = [];
        private long _bufferTotal;
        private long _triangles;
        private readonly Dictionary<int, DocumentProduct?> _meshProducts = [];
        private ProgressScope? _ps;

        public Tree? Read(byte[] data, ProgressRange progress)
        {
            ReadOnlySpan<byte> json = data;
            if (data.Length >= 4 && BinaryPrimitives.ReadUInt32LittleEndian(data) == 0x46546C67)
            {
                json = ParseGlb(data);
            }
            var options = new JsonDocumentOptions { MaxDepth = _limits.MaxJsonDepth };
            using JsonDocument doc = JsonDocument.Parse(json.ToArray(), options);
            _root = doc.RootElement;
            if (_root.ValueKind != JsonValueKind.Object)
            {
                throw new FormatError("The glTF JSON isn't an object");
            }
            CheckAsset();

            JsonElement[] nodes = global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(_root, "nodes");
            if (nodes.Length > _limits.MaxNodes)
            {
                throw new FormatError($"{nodes.Length} nodes, more than the limit ({_limits.MaxNodes})");
            }
            int[] parents = Parents(nodes);
            int[] rootNodes = SceneRoots(nodes, parents);

            int nbMeshes = global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(_root, "meshes").Length;
            using var ps = new ProgressScope(progress, "Reading glTF", Math.Max(1, nbMeshes));
            _ps = ps;
            var roots = new List<Instance>();
            foreach (int node in rootNodes)
            {
                Instance? instance = Visit(nodes, node, null, 1);
                if (!ps.More())
                {
                    return null;
                }
                if (instance is not null)
                {
                    roots.Add(instance);
                }
            }
            var unit = new DocumentLengthUnit(theReader.Options.OutputLengthUnit == 0.001 ? "MM" : "", theReader.Options.OutputLengthUnit);
            return new Tree(roots, null, unit);
        }

        private ReadOnlySpan<byte> ParseGlb(byte[] data)
        {
            if (data.Length < 20)
            {
                throw new FormatError("GLB: truncated header");
            }
            uint version = BinaryPrimitives.ReadUInt32LittleEndian(data.AsSpan(4));
            uint length = BinaryPrimitives.ReadUInt32LittleEndian(data.AsSpan(8));
            if (version != 2)
            {
                throw new FormatError($"GLB: version {version}, not 2");
            }
            if (length > data.Length || length < 20)
            {
                throw new FormatError("GLB: the length in the header doesn't fit the data");
            }
            ReadOnlySpan<byte> json = default;
            long offset = 12;
            int chunk = 0;
            while (offset + 8 <= length)
            {
                uint chunkLength = BinaryPrimitives.ReadUInt32LittleEndian(data.AsSpan((int)offset));
                uint chunkType = BinaryPrimitives.ReadUInt32LittleEndian(data.AsSpan((int)offset + 4));
                long start = offset + 8;
                if (start + chunkLength > length)
                {
                    throw new FormatError("GLB: a chunk runs past the end of the data");
                }
                if (chunk == 0)
                {
                    if (chunkType != 0x4E4F534A)
                    {
                        throw new FormatError("GLB: the first chunk isn't JSON");
                    }
                    json = data.AsSpan((int)start, (int)chunkLength);
                }
                else if (chunk == 1 && chunkType == 0x004E4942)
                {
                    _glbBin = data.AsSpan((int)start, (int)chunkLength).ToArray();
                }
                offset = start + ((chunkLength + 3L) & ~3L);
                chunk++;
            }
            if (chunk == 0)
            {
                throw new FormatError("GLB: no JSON chunk");
            }
            return json;
        }

        private void CheckAsset()
        {
            if (!_root.TryGetProperty("asset", out JsonElement asset) || asset.ValueKind != JsonValueKind.Object
                || !asset.TryGetProperty("version", out JsonElement version) || version.ValueKind != JsonValueKind.String)
            {
                throw new FormatError("No asset version: not glTF 2.0");
            }
            string text = version.GetString() ?? "";
            if (!text.StartsWith("2.", StringComparison.Ordinal))
            {
                throw new FormatError($"glTF version {text}, not 2.x");
            }
            foreach (JsonElement ext in global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(_root, "extensionsRequired"))
            {
                string name = ext.ValueKind == JsonValueKind.String ? ext.GetString() ?? "" : "";
                // extensions of materials and textures change only the looks: the geometry reads without them
                if (name.StartsWith("KHR_materials_", StringComparison.Ordinal) || name.StartsWith("KHR_texture_", StringComparison.Ordinal))
                {
                    theReader._warnings.Add($"Required extension {name} ignored (materials only)");
                    continue;
                }
                throw new FormatError($"Required extension not supported: {name}");
            }
        }

        // ------------------------------------------------------------------------------------------ JSON access

        private static JsonElement[] Array(JsonElement theObject, string name)
        {
            if (!theObject.TryGetProperty(name, out JsonElement array))
            {
                return [];
            }
            if (array.ValueKind != JsonValueKind.Array)
            {
                throw new FormatError($"'{name}' isn't an array");
            }
            return [.. array.EnumerateArray()];
        }

        private static JsonElement Element(JsonElement[] array, int index, string what)
        {
            if (index < 0 || index >= array.Length)
            {
                throw new FormatError($"{what} {index} doesn't exist");
            }
            if (array[index].ValueKind != JsonValueKind.Object)
            {
                throw new FormatError($"{what} {index} isn't an object");
            }
            return array[index];
        }

        private static long Integer(JsonElement theObject, string name, long theDefault, bool isRequired = false)
        {
            if (!theObject.TryGetProperty(name, out JsonElement value))
            {
                return isRequired ? throw new FormatError($"'{name}' is missing") : theDefault;
            }
            if (value.ValueKind != JsonValueKind.Number || !value.TryGetInt64(out long result) || result < 0)
            {
                throw new FormatError($"'{name}' isn't a non-negative integer");
            }
            return result;
        }

        private static int Index(JsonElement theObject, string name, bool isRequired = false)
        {
            long value = Integer(theObject, name, -1, isRequired);
            return value > int.MaxValue ? throw new FormatError($"'{name}' is too large") : (int)value;
        }

        private static string? Text(JsonElement theObject, string name)
            => theObject.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.String ? value.GetString() : null;

        private static double[]? Numbers(JsonElement theObject, string name, int count)
        {
            if (!theObject.TryGetProperty(name, out JsonElement value))
            {
                return null;
            }
            if (value.ValueKind != JsonValueKind.Array || value.GetArrayLength() != count)
            {
                throw new FormatError($"'{name}' isn't an array of {count} numbers");
            }
            double[] result = new double[count];
            int i = 0;
            foreach (JsonElement number in value.EnumerateArray())
            {
                if (number.ValueKind != JsonValueKind.Number || !double.IsFinite(result[i++] = number.GetDouble()))
                {
                    throw new FormatError($"'{name}' isn't an array of {count} numbers");
                }
            }
            return result;
        }

        // ------------------------------------------------------------------------------------------ the hierarchy

        private static int[] Parents(JsonElement[] nodes)
        {
            int[] parents = new int[nodes.Length];
            System.Array.Fill(parents, -1);
            for (int i = 0; i < nodes.Length; i++)
            {
                JsonElement node = Element(nodes, i, "Node");
                foreach (JsonElement child in global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(node, "children"))
                {
                    if (child.ValueKind != JsonValueKind.Number || !child.TryGetInt32(out int c) || c < 0 || c >= nodes.Length)
                    {
                        throw new FormatError($"Node {i} has an invalid child");
                    }
                    if (parents[c] >= 0 || c == i)
                    {
                        throw new FormatError($"Node {c} has more than one parent, or is its own child");
                    }
                    parents[c] = i;
                }
            }
            return parents;
        }

        private int[] SceneRoots(JsonElement[] nodes, int[] parents)
        {
            JsonElement[] scenes = global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(_root, "scenes");
            if (scenes.Length == 0)
            {
                // no scene: every parentless node (a node in a cycle has a parent, so cycles aren't reached)
                return [.. Enumerable.Range(0, nodes.Length).Where(i => parents[i] < 0)];
            }
            int scene = _root.TryGetProperty("scene", out _) ? global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Index(_root, "scene") : 0;
            JsonElement sceneObject = Element(scenes, scene, "Scene");
            var roots = new List<int>();
            foreach (JsonElement node in global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(sceneObject, "nodes"))
            {
                if (node.ValueKind != JsonValueKind.Number || !node.TryGetInt32(out int n) || n < 0 || n >= nodes.Length)
                {
                    throw new FormatError("A scene has an invalid node");
                }
                if (parents[n] >= 0)
                {
                    throw new FormatError($"Scene node {n} isn't a root node");
                }
                roots.Add(n);
            }
            return [.. roots];
        }

        private Instance? Visit(JsonElement[] nodes, int node, double[]? pending, int depth)
        {
            if (depth > Math.Min(_limits.MaxDepth, HARD_MAX_DEPTH))
            {
                throw new FormatError($"The node hierarchy is deeper than the limit ({Math.Min(_limits.MaxDepth, HARD_MAX_DEPTH)})");
            }
            if (_ps is not null && !_ps.More())
            {
                return null;
            }
            JsonElement aNode = nodes[node];
            double[] matrix = _toModel.MapMatrix(LocalMatrix(aNode));
            if (pending is not null)
            {
                matrix = Multiply(pending, matrix);
            }
            Placement location = new();
            double[]? bake = null;
            if (Rigid(matrix, out Transform3D trsf, out bool isIdentity))
            {
                if (!isIdentity)
                {
                    location = new Placement(trsf);
                }
            }
            else
            {
                bake = matrix;
            }

            string? name = Text(aNode, "name");
            DocumentProduct? meshProduct = null;
            if (aNode.TryGetProperty("mesh", out _))
            {
                meshProduct = MeshProduct(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Index(aNode, "mesh"), bake);
            }
            var children = new List<Instance>();
            foreach (JsonElement child in global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(aNode, "children"))
            {
                Instance? instance = Visit(nodes, child.GetInt32(), bake, depth + 1);
                if (instance is not null)
                {
                    children.Add(instance);
                }
            }
            if (children.Count == 0 && meshProduct is not null)
            {
                Attributes attributes = name is not null && name != meshProduct.Name
                                                   ? Attributes.Empty.WithName(name)
                                                   : Attributes.Empty;
                return new Instance(meshProduct, location, attributes);
            }
            var components = new List<Instance>();
            if (meshProduct is not null)
            {
                components.Add(new Instance(meshProduct));
            }
            components.AddRange(children);
            return new Instance(DocumentProduct.Assembly(components, Attributes.Empty.WithName(name)), location);
        }

        // The node's matrix (column-major), from "matrix" or from translation, rotation and scale
        private static double[] LocalMatrix(JsonElement node)
        {
            if (Numbers(node, "matrix", 16) is double[] matrix)
            {
                return matrix;
            }
            double[] t = Numbers(node, "translation", 3) ?? [0, 0, 0];
            double[] q = Numbers(node, "rotation", 4) ?? [0, 0, 0, 1];
            double[] s = Numbers(node, "scale", 3) ?? [1, 1, 1];
            double norm = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (norm < 1e-12)
            {
                throw new FormatError("A node's rotation is a null quaternion");
            }
            double x = q[0] / norm, y = q[1] / norm, z = q[2] / norm, w = q[3] / norm;
            double[] r =
            [
                1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w),
                2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y),
            ];
            double[] result = new double[16];
            for (int col = 0; col < 3; col++)
            {
                for (int row = 0; row < 3; row++)
                {
                    result[4 * col + row] = r[3 * col + row] * s[col];
                }
            }
            result[12] = t[0];
            result[13] = t[1];
            result[14] = t[2];
            result[15] = 1.0;
            return result;
        }

        private static double[] Multiply(double[] a, double[] b)
        {
            double[] result = new double[16];
            for (int col = 0; col < 4; col++)
            {
                for (int row = 0; row < 4; row++)
                {
                    double sum = 0.0;
                    for (int k = 0; k < 4; k++)
                    {
                        sum += a[4 * k + row] * b[4 * col + k];
                    }
                    result[4 * col + row] = sum;
                }
            }
            return result;
        }

        // A rigid motion (orthonormal, not mirroring) as a gp_Trsf
        private static bool Rigid(double[] m, out Transform3D trsf, out bool isIdentity)
        {
            trsf = new Transform3D();
            isIdentity = false;
            const double tol = 1e-6;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    double dot = m[4 * i] * m[4 * j] + m[4 * i + 1] * m[4 * j + 1] + m[4 * i + 2] * m[4 * j + 2];
                    if (Math.Abs(dot - (i == j ? 1.0 : 0.0)) > tol)
                    {
                        return false;
                    }
                }
            }
            double det = m[0] * (m[5] * m[10] - m[9] * m[6])
                        - m[4] * (m[1] * m[10] - m[9] * m[2])
                        + m[8] * (m[1] * m[6] - m[5] * m[2]);
            if (det <= 0.0 || Math.Abs(m[3]) > tol || Math.Abs(m[7]) > tol || Math.Abs(m[11]) > tol
                || Math.Abs(m[15] - 1.0) > tol)
            {
                return false;
            }
            isIdentity = Math.Abs(m[0] - 1) <= 1e-12 && Math.Abs(m[5] - 1) <= 1e-12 && Math.Abs(m[10] - 1) <= 1e-12
                         && m[12] == 0.0 && m[13] == 0.0 && m[14] == 0.0;
            if (!isIdentity)
            {
                trsf = trsf.WithValues(m[0], m[4], m[8], m[12],
                                  m[1], m[5], m[9], m[13],
                                  m[2], m[6], m[10], m[14]);
            }
            return true;
        }

        // ------------------------------------------------------------------------------------------ meshes

        private DocumentProduct? MeshProduct(int mesh, double[]? bake)
        {
            if (bake is null && _meshProducts.TryGetValue(mesh, out DocumentProduct? known))
            {
                return known;
            }
            JsonElement aMesh = Element(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(_root, "meshes"), mesh, "Mesh");
            if (bake is null)
            {
                _ps?.Next();
            }
            string? name = Text(aMesh, "name");
            var faces = new List<(Face Face, ColorWithAlpha? Color)>();
            int primIndex = 0;
            foreach (JsonElement prim in global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(aMesh, "primitives"))
            {
                if (prim.ValueKind != JsonValueKind.Object)
                {
                    throw new FormatError($"Mesh {mesh}: a primitive isn't an object");
                }
                Triangulation? tri = this.Triangulation(mesh, primIndex++, prim, bake);
                if (tri is null)
                {
                    continue;
                }
                Face face = new();
                new BrepBuilder().MakeFace(ref face, tri);
                ColorWithAlpha? color = prim.TryGetProperty("material", out _) ? MaterialColor(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Index(prim, "material")) : null;
                faces.Add((face, color));
            }

            DocumentProduct? product = null;
            if (faces.Count == 1)
            {
                product = DocumentProduct.Part(faces[0].Face, new Attributes { Name = name, SurfaceColor = faces[0].Color });
            }
            else if (faces.Count > 1)
            {
                var builder = new BrepBuilder();
                Compound compound = new();
                builder.MakeCompound(ref compound);
                var subs = new List<SubShape>();
                foreach ((Face face, ColorWithAlpha? color) in faces)
                {
                    builder.Add(compound, face);
                    if (color is not null)
                    {
                        subs.Add(new SubShape(face, new Attributes { SurfaceColor = color }));
                    }
                }
                product = DocumentProduct.Part(compound, Attributes.Empty.WithName(name), subs);
            }
            if (bake is null)
            {
                _meshProducts[mesh] = product;
            }
            return product;
        }

        private ColorWithAlpha MaterialColor(int material)
        {
            JsonElement aMaterial = Element(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(_root, "materials"), material, "Material");
            double[] factor = [1, 1, 1, 1];
            if (aMaterial.TryGetProperty("pbrMetallicRoughness", out JsonElement pbr) && pbr.ValueKind == JsonValueKind.Object)
            {
                factor = Numbers(pbr, "baseColorFactor", 4) ?? factor;
            }
            else if (aMaterial.TryGetProperty("extensions", out JsonElement exts) && exts.ValueKind == JsonValueKind.Object
                     && exts.TryGetProperty("KHR_materials_pbrSpecularGlossiness", out JsonElement specGloss)
                     && specGloss.ValueKind == JsonValueKind.Object)
            {
                factor = Numbers(specGloss, "diffuseFactor", 4) ?? factor;
            }
            float clamp(double v) => (float)Math.Clamp(v, 0.0, 1.0);
            return new ColorWithAlpha(clamp(factor[0]), clamp(factor[1]), clamp(factor[2]), clamp(factor[3]));
        }

        private Triangulation? Triangulation(int mesh, int prim, JsonElement primitive, double[]? bake)
        {
            long mode = Integer(primitive, "mode", 4);
            if (mode is not (4 or 5 or 6))
            {
                theReader._warnings.Add($"Mesh {mesh}, primitive {prim}: mode {mode} (not triangles) skipped");
                return null;
            }
            if (!primitive.TryGetProperty("attributes", out JsonElement attributes) || attributes.ValueKind != JsonValueKind.Object
                || !attributes.TryGetProperty("POSITION", out _))
            {
                theReader._warnings.Add($"Mesh {mesh}, primitive {prim}: no POSITION, skipped");
                return null;
            }
            float[] positions = Floats(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Index(attributes, "POSITION"), "VEC3", out int nbNodes);
            float[]? normals = null;
            if (attributes.TryGetProperty("NORMAL", out _))
            {
                normals = Floats(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Index(attributes, "NORMAL"), "VEC3", out int nbNormals);
                if (nbNormals != nbNodes)
                {
                    throw new FormatError($"Mesh {mesh}, primitive {prim}: NORMAL and POSITION counts differ");
                }
            }
            if (nbNodes == 0)
            {
                return null;
            }
            int[] indices = primitive.TryGetProperty("indices", out _)
                                ? Indices(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Index(primitive, "indices"), nbNodes)
                                : [.. Enumerable.Range(0, nbNodes)];

            // the triangles of the mode
            var triangles = new List<(int, int, int)>();
            if (mode == 4)
            {
                for (int i = 0; i + 2 < indices.Length; i += 3)
                {
                    triangles.Add((indices[i], indices[i + 1], indices[i + 2]));
                }
            }
            else if (mode == 5)
            {
                for (int i = 0; i + 2 < indices.Length; i++)
                {
                    triangles.Add(i % 2 == 0 ? (indices[i], indices[i + 1], indices[i + 2])
                                              : (indices[i + 1], indices[i], indices[i + 2]));
                }
            }
            else
            {
                for (int i = 1; i + 1 < indices.Length; i++)
                {
                    triangles.Add((indices[0], indices[i], indices[i + 1]));
                }
            }
            if (triangles.Count == 0)
            {
                return null;
            }
            _triangles += triangles.Count;
            if (_triangles > _limits.MaxTriangles)
            {
                throw new FormatError($"More triangles than the limit ({_limits.MaxTriangles})");
            }

            bool isMirrored = false;
            if (bake is not null)
            {
                double det = bake[0] * (bake[5] * bake[10] - bake[9] * bake[6])
                            - bake[4] * (bake[1] * bake[10] - bake[9] * bake[2])
                            + bake[8] * (bake[1] * bake[6] - bake[5] * bake[2]);
                isMirrored = det < 0.0;
            }
            var tri = new Triangulation(nbNodes, triangles.Count, false, normals is not null);
            for (int i = 0; i < nbNodes; i++)
            {
                double x = positions[3 * i], y = positions[3 * i + 1], z = positions[3 * i + 2];
                _toModel.Map(ref x, ref y, ref z);
                if (bake is not null)
                {
                    (x, y, z) = (bake[0] * x + bake[4] * y + bake[8] * z + bake[12],
                                    bake[1] * x + bake[5] * y + bake[9] * z + bake[13],
                                    bake[2] * x + bake[6] * y + bake[10] * z + bake[14]);
                }
                tri.SetNode(i + 1, new Point3D(x, y, z));
                if (normals is not null)
                {
                    double nx = normals[3 * i], ny = normals[3 * i + 1], nz = normals[3 * i + 2];
                    _toModel.Map(ref nx, ref ny, ref nz, true);
                    if (bake is not null)
                    {
                        (nx, ny, nz) = NormalTransformed(bake, nx, ny, nz);
                    }
                    double length = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                    if (length > 0.0)
                    {
                        tri.SetNormal(i + 1, new NCollection_Vec3<float>((float)(nx / length), (float)(ny / length), (float)(nz / length)));
                    }
                    else
                    {
                        tri.SetNormal(i + 1, new NCollection_Vec3<float>(0.0f, 0.0f, 1.0f));
                    }
                }
            }
            for (int i = 0; i < triangles.Count; i++)
            {
                (int a, int b, int c) = triangles[i];
                tri.SetTriangle(i + 1, isMirrored ? new Triangle1(a + 1, c + 1, b + 1) : new Triangle1(a + 1, b + 1, c + 1));
            }
            return tri;
        }

        // A normal by the inverse transpose of the matrix's linear part (its cofactors)
        private static (double, double, double) NormalTransformed(double[] m, double x, double y, double z)
        {
            double a = m[0], b = m[4], c = m[8], d = m[1], e = m[5], f = m[9], g = m[2], h = m[6], k = m[10];
            double c00 = e * k - f * h, c01 = -(d * k - f * g), c02 = d * h - e * g;
            double c10 = -(b * k - c * h), c11 = a * k - c * g, c12 = -(a * h - b * g);
            double c20 = b * f - c * e, c21 = -(a * f - c * d), c22 = a * e - b * d;
            double det = a * c00 + b * c01 + c * c02;
            double sign = det < 0.0 ? -1.0 : 1.0; // the cofactors of a mirror flip the normal: undone, the winding is swapped instead
            return (sign * (c00 * x + c01 * y + c02 * z),
                    sign * (c10 * x + c11 * y + c12 * z),
                    sign * (c20 * x + c21 * y + c22 * z));
        }

        // ------------------------------------------------------------------------------------------ accessors

        private sealed record AccessorData(byte[] Buffer, long Offset, int Stride, int Count, int ComponentType, int NbComponents);

        private AccessorData Accessor(int accessor, string type)
        {
            JsonElement anAccessor = Element(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(_root, "accessors"), accessor, "Accessor");
            if (anAccessor.TryGetProperty("sparse", out _))
            {
                throw new FormatError($"Accessor {accessor}: sparse accessors aren't supported");
            }
            string? aType = Text(anAccessor, "type");
            if (aType != type)
            {
                throw new FormatError($"Accessor {accessor}: type {aType}, {type} expected");
            }
            int nbComponents = type == "VEC3" ? 3 : 1;
            int componentType = global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Index(anAccessor, "componentType", true);
            int componentSize = componentType switch
            {
                5120 or UNSIGNED_BYTE => 1,
                5122 or UNSIGNED_SHORT => 2,
                UNSIGNED_INT or Float => 4,
                _ => throw new FormatError($"Accessor {accessor}: unknown component type {componentType}"),
            };
            long count = Integer(anAccessor, "count", 0, true);
            if (count > int.MaxValue / 3)
            {
                throw new FormatError($"Accessor {accessor}: count too large");
            }
            if (!anAccessor.TryGetProperty("bufferView", out _))
            {
                throw new FormatError($"Accessor {accessor}: no buffer view (zero-filled accessors aren't supported)");
            }
            JsonElement view = Element(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(_root, "bufferViews"), global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Index(anAccessor, "bufferView"), "Buffer view");
            int bufferIndex = global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Index(view, "buffer", true);
            long viewOffset = Integer(view, "byteOffset", 0);
            long viewLength = Integer(view, "byteLength", 0, true);
            long elementSize = (long)componentSize * nbComponents;
            long stride = Integer(view, "byteStride", elementSize);
            if (stride < elementSize || stride > 252)
            {
                throw new FormatError($"Accessor {accessor}: invalid byte stride {stride}");
            }
            long accessorOffset = Integer(anAccessor, "byteOffset", 0);
            if (accessorOffset % componentSize != 0)
            {
                throw new FormatError($"Accessor {accessor}: misaligned");
            }
            byte[] buffer = this.Buffer(bufferIndex);
            if (viewOffset + viewLength > buffer.Length)
            {
                throw new FormatError($"Accessor {accessor}: its buffer view runs past its buffer");
            }
            if (count > 0 && accessorOffset + stride * (count - 1) + elementSize > viewLength)
            {
                throw new FormatError($"Accessor {accessor}: runs past its buffer view");
            }
            return new AccessorData(buffer, viewOffset + accessorOffset, (int)stride, (int)count, componentType, nbComponents);
        }

        private float[] Floats(int accessor, string type, out int count)
        {
            AccessorData data = Accessor(accessor, type);
            if (data.ComponentType != Float)
            {
                throw new FormatError($"Accessor {accessor}: not 32-bit floats (quantized data isn't supported)");
            }
            count = data.Count;
            float[] result = new float[data.Count * data.NbComponents];
            for (int i = 0; i < data.Count; i++)
            {
                for (int k = 0; k < data.NbComponents; k++)
                {
                    float value = BinaryPrimitives.ReadSingleLittleEndian(data.Buffer.AsSpan((int)(data.Offset + (long)data.Stride * i + 4 * k)));
                    if (!float.IsFinite(value))
                    {
                        throw new FormatError($"Accessor {accessor}: a value isn't finite");
                    }
                    result[data.NbComponents * i + k] = value;
                }
            }
            return result;
        }

        private int[] Indices(int accessor, int nbNodes)
        {
            AccessorData data = Accessor(accessor, "SCALAR");
            int[] result = new int[data.Count];
            for (int i = 0; i < data.Count; i++)
            {
                ReadOnlySpan<byte> span = data.Buffer.AsSpan((int)(data.Offset + (long)data.Stride * i));
                long value = data.ComponentType switch
                {
                    UNSIGNED_BYTE => span[0],
                    UNSIGNED_SHORT => BinaryPrimitives.ReadUInt16LittleEndian(span),
                    UNSIGNED_INT => BinaryPrimitives.ReadUInt32LittleEndian(span),
                    _ => throw new FormatError($"Accessor {accessor}: indices of component type {data.ComponentType}"),
                };
                if (value >= nbNodes)
                {
                    throw new FormatError($"Accessor {accessor}: index {value} out of the {nbNodes} vertices");
                }
                result[i] = (int)value;
            }
            return result;
        }

        private byte[] Buffer(int buffer)
        {
            if (_buffers.TryGetValue(buffer, out byte[]? known))
            {
                return known;
            }
            JsonElement aBuffer = Element(global::Kerf.Engine.Exchange.Gltf.TreeReader.Context.Array(_root, "buffers"), buffer, "Buffer");
            long length = Integer(aBuffer, "byteLength", 0, true);
            if (length > _limits.MaxBufferBytes || _bufferTotal + length > _limits.MaxBufferBytes)
            {
                throw new FormatError($"Buffer {buffer}: larger than the limit ({_limits.MaxBufferBytes} bytes)");
            }
            string? uri = Text(aBuffer, "uri");
            byte[] data;
            if (uri is null)
            {
                data = buffer == 0 && _glbBin is not null
                          ? _glbBin
                          : throw new FormatError($"Buffer {buffer}: no URI and no GLB binary chunk");
            }
            else if (uri.StartsWith("data:", StringComparison.Ordinal))
            {
                int comma = uri.IndexOf(',', StringComparison.Ordinal);
                if (comma < 0 || !uri.AsSpan(0, comma).EndsWith(";base64", StringComparison.Ordinal))
                {
                    throw new FormatError($"Buffer {buffer}: a data URI that isn't base64");
                }
                if ((uri.Length - comma - 1) / 4L * 3 > _limits.MaxBufferBytes)
                {
                    throw new FormatError($"Buffer {buffer}: larger than the limit ({_limits.MaxBufferBytes} bytes)");
                }
                try
                {
                    data = Convert.FromBase64String(uri[(comma + 1)..]);
                }
                catch (FormatException)
                {
                    throw new FormatError($"Buffer {buffer}: invalid base64 data");
                }
            }
            else
            {
                Stream? stream = theResolver?.Invoke(Uri.UnescapeDataString(uri))
                                  ?? throw new FormatError($"Buffer {buffer}: '{uri}' can't be opened");
                using (stream)
                {
                    data = ReadAll(stream, _limits.MaxBufferBytes - _bufferTotal, $"Buffer {buffer}");
                }
            }
            if (data.Length < length)
            {
                throw new FormatError($"Buffer {buffer}: {data.Length} bytes, {length} expected");
            }
            _bufferTotal += length;
            _buffers[buffer] = data;
            return data;
        }
    }
}
