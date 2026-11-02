// New in OCCT.NET (unit 8T): glTF 2.0 written fresh over the assembly tree with System.Text.Json (decisions 1166, 1233).
// Not a port of OCCT's RWGltf_CafWriter, which needs XCAF and RapidJSON; its conventions are kept where they show in
// the output: a node per instance and a mesh per part shared by its instances, Z-up turned Y-up and the length unit
// turned into metres, a material per colour (PBR, metallic 0, roughness 1, the surface colour else the curve colour as
// base colour, blended if transparent).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Buffers.Binary;
using System.Text;
using System.Text.Json;

namespace Occt;

/// <summary>
/// Writes an assembly tree as glTF 2.0: JSON with a separate binary buffer (<c>.gltf</c> and <c>.bin</c>), or a single
/// binary file (<c>.glb</c>).
/// </summary>
/// <remarks>
/// <para>
/// The scene has a node per root; a node per instance (its name the instance's or else its product's, its matrix the
/// instance's location); an assembly's node has its components' nodes as children, a part's node a mesh. Each part
/// is meshed once: its faces' triangulations (<see cref="RWGltf_WriteOptions.MeshParameters"/> to mesh first) in a
/// primitive per sub-shape colour, the accessors shared by every instance, a mesh per part and instance colour.
/// Materials are PBR, base colour the style's surface colour (linear RGB with alpha, as glTF wants), else its curve
/// colour; parts without colour have no material (glTF's default).
/// </para>
/// <para>Streams first; the file path forms write next to each other. The tree isn't changed. Output is deterministic.</para>
/// </remarks>
public sealed class RWGltf_TreeWriter
{
    private const int ARRAY_BUFFER = 34962;
    private const int ELEMENT_ARRAY_BUFFER = 34963;
    private const int FLOAT = 5126;
    private const int UNSIGNED_SHORT = 5123;
    private const int UNSIGNED_INT = 5125;

    /// <summary>A writer with the options (the defaults if null).</summary>
    public RWGltf_TreeWriter(RWGltf_WriteOptions? theOptions = null) => Options = theOptions ?? RWGltf_WriteOptions.Default;

    /// <summary>The options.</summary>
    public RWGltf_WriteOptions Options { get; }

    /// <summary>Writes the tree as binary glTF (<c>.glb</c>). False if cancelled or the stream fails.</summary>
    public bool WriteGlb(XdeDoc_Tree theTree, Stream theStream, Message_ProgressRange theProgress = default)
    {
        ArgumentNullException.ThrowIfNull(theTree);
        ArgumentNullException.ThrowIfNull(theStream);
        Model? aModel = build(theTree, null, theProgress);
        if (aModel is null)
        {
            return false;
        }
        byte[] aJson = aModel.Json(false);
        int aJsonLength = pad4(aJson.Length);
        int aBinLength = pad4((int)aModel.Bin.Length);
        bool hasBin = aModel.Bin.Length > 0;
        int aTotal = 12 + 8 + aJsonLength + (hasBin ? 8 + aBinLength : 0);
        try
        {
            Span<byte> aHeader = stackalloc byte[12];
            BinaryPrimitives.WriteUInt32LittleEndian(aHeader, 0x46546C67); // "glTF"
            BinaryPrimitives.WriteUInt32LittleEndian(aHeader[4..], 2);
            BinaryPrimitives.WriteUInt32LittleEndian(aHeader[8..], (uint)aTotal);
            theStream.Write(aHeader);
            writeChunk(theStream, 0x4E4F534A, aJson, aJsonLength, 0x20); // "JSON", padded with spaces
            if (hasBin)
            {
                writeChunk(theStream, 0x004E4942, aModel.Bin.ToArray(), aBinLength, 0); // "BIN\0"
            }
            theStream.Flush();
        }
        catch (IOException)
        {
            return false;
        }
        return true;
    }

    /// <summary>
    /// Writes the tree as glTF JSON to <paramref name="theJson"/> and its buffer to <paramref name="theBin"/>, which the
    /// JSON refers to as <paramref name="theBinUri"/> (a relative URI, such as "model.bin"). Without triangles nothing
    /// is written to <paramref name="theBin"/> and the JSON has no buffer. False if cancelled or a stream fails.
    /// </summary>
    public bool WriteGltf(XdeDoc_Tree theTree, Stream theJson, Stream theBin, string theBinUri, Message_ProgressRange theProgress = default)
    {
        ArgumentNullException.ThrowIfNull(theTree);
        ArgumentNullException.ThrowIfNull(theJson);
        ArgumentNullException.ThrowIfNull(theBin);
        ArgumentNullException.ThrowIfNull(theBinUri);
        Model? aModel = build(theTree, theBinUri, theProgress);
        if (aModel is null)
        {
            return false;
        }
        try
        {
            theJson.Write(aModel.Json(true));
            theJson.Flush();
            if (aModel.Bin.Length > 0)
            {
                aModel.Bin.Position = 0;
                aModel.Bin.CopyTo(theBin);
                theBin.Flush();
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
    public bool Perform(XdeDoc_Tree theTree, string theFilePath, Message_ProgressRange theProgress = default)
    {
        if (string.Equals(Path.GetExtension(theFilePath), ".glb", StringComparison.OrdinalIgnoreCase))
        {
            using FileStream aFile = File.Create(theFilePath);
            return WriteGlb(theTree, aFile, theProgress);
        }
        string aBinPath = Path.ChangeExtension(theFilePath, ".bin");
        using FileStream aJson = File.Create(theFilePath);
        using FileStream aBin = File.Create(aBinPath);
        return WriteGltf(theTree, aJson, aBin, Uri.EscapeDataString(Path.GetFileName(aBinPath)), theProgress);
    }

    private static int pad4(int theLength) => (theLength + 3) & ~3;

    private static void writeChunk(Stream theStream, uint theType, byte[] theData, int thePaddedLength, byte thePad)
    {
        Span<byte> aHeader = stackalloc byte[8];
        BinaryPrimitives.WriteUInt32LittleEndian(aHeader, (uint)thePaddedLength);
        BinaryPrimitives.WriteUInt32LittleEndian(aHeader[4..], theType);
        theStream.Write(aHeader);
        theStream.Write(theData);
        for (int i = theData.Length; i < thePaddedLength; i++)
        {
            theStream.WriteByte(thePad);
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
        public readonly List<Quantity_ColorRGBA> Materials = [];
        public readonly List<Mesh> Meshes = [];
        public readonly List<Node> Nodes = [];
        public readonly List<int> SceneNodes = [];
        public string? BinUri;
        public string Generator = "";

        public int AddView(ReadOnlySpan<byte> theData, int theTarget)
        {
            int anOffset = (int)Bin.Length;
            Bin.Write(theData);
            while (Bin.Length % 4 != 0)
            {
                Bin.WriteByte(0);
            }
            BufferViews.Add(new BufferView(anOffset, theData.Length, theTarget));
            return BufferViews.Count - 1;
        }

        public byte[] Json(bool isIndented)
        {
            using var aStream = new MemoryStream();
            using (var aW = new Utf8JsonWriter(aStream, new JsonWriterOptions { Indented = isIndented }))
            {
                aW.WriteStartObject();
                aW.WriteStartObject("asset");
                aW.WriteString("generator", Generator);
                aW.WriteString("version", "2.0");
                aW.WriteEndObject();

                aW.WriteNumber("scene", 0);
                aW.WriteStartArray("scenes");
                aW.WriteStartObject();
                aW.WriteStartArray("nodes");
                foreach (int aNode in SceneNodes)
                {
                    aW.WriteNumberValue(aNode);
                }
                aW.WriteEndArray();
                aW.WriteEndObject();
                aW.WriteEndArray();

                if (Nodes.Count > 0)
                {
                    aW.WriteStartArray("nodes");
                    foreach (Node aNode in Nodes)
                    {
                        aW.WriteStartObject();
                        if (aNode.Name is not null)
                        {
                            aW.WriteString("name", aNode.Name);
                        }
                        if (aNode.Matrix is not null)
                        {
                            aW.WriteStartArray("matrix");
                            foreach (double aValue in aNode.Matrix)
                            {
                                aW.WriteNumberValue(aValue);
                            }
                            aW.WriteEndArray();
                        }
                        if (aNode.Mesh >= 0)
                        {
                            aW.WriteNumber("mesh", aNode.Mesh);
                        }
                        if (aNode.Children.Count > 0)
                        {
                            aW.WriteStartArray("children");
                            foreach (int aChild in aNode.Children)
                            {
                                aW.WriteNumberValue(aChild);
                            }
                            aW.WriteEndArray();
                        }
                        aW.WriteEndObject();
                    }
                    aW.WriteEndArray();
                }

                if (Meshes.Count > 0)
                {
                    aW.WriteStartArray("meshes");
                    foreach (Mesh aMesh in Meshes)
                    {
                        aW.WriteStartObject();
                        if (aMesh.Name is not null)
                        {
                            aW.WriteString("name", aMesh.Name);
                        }
                        aW.WriteStartArray("primitives");
                        foreach (Primitive aPrim in aMesh.Primitives)
                        {
                            aW.WriteStartObject();
                            aW.WriteStartObject("attributes");
                            if (aPrim.Normal >= 0)
                            {
                                aW.WriteNumber("NORMAL", aPrim.Normal);
                            }
                            aW.WriteNumber("POSITION", aPrim.Position);
                            aW.WriteEndObject();
                            aW.WriteNumber("indices", aPrim.Indices);
                            if (aPrim.Material >= 0)
                            {
                                aW.WriteNumber("material", aPrim.Material);
                            }
                            aW.WriteNumber("mode", 4);
                            aW.WriteEndObject();
                        }
                        aW.WriteEndArray();
                        aW.WriteEndObject();
                    }
                    aW.WriteEndArray();
                }

                if (Materials.Count > 0)
                {
                    aW.WriteStartArray("materials");
                    for (int i = 0; i < Materials.Count; i++)
                    {
                        Quantity_ColorRGBA aColor = Materials[i];
                        Quantity_Color aRgb = aColor.GetRGB();
                        aW.WriteStartObject();
                        aW.WriteString("name", "mat_" + i.ToString(System.Globalization.CultureInfo.InvariantCulture));
                        aW.WriteStartObject("pbrMetallicRoughness");
                        aW.WriteStartArray("baseColorFactor");
                        aW.WriteNumberValue((float)aRgb.Red());
                        aW.WriteNumberValue((float)aRgb.Green());
                        aW.WriteNumberValue((float)aRgb.Blue());
                        aW.WriteNumberValue(aColor.Alpha());
                        aW.WriteEndArray();
                        aW.WriteNumber("metallicFactor", 0);
                        aW.WriteNumber("roughnessFactor", 1);
                        aW.WriteEndObject();
                        if (aColor.Alpha() < 1.0f)
                        {
                            aW.WriteString("alphaMode", "BLEND");
                        }
                        aW.WriteEndObject();
                    }
                    aW.WriteEndArray();
                }

                if (Accessors.Count > 0)
                {
                    aW.WriteStartArray("accessors");
                    foreach (Accessor anAccessor in Accessors)
                    {
                        aW.WriteStartObject();
                        aW.WriteNumber("bufferView", anAccessor.BufferView);
                        aW.WriteNumber("componentType", anAccessor.ComponentType);
                        aW.WriteNumber("count", anAccessor.Count);
                        aW.WriteString("type", anAccessor.Type);
                        if (anAccessor.Min is not null && anAccessor.Max is not null)
                        {
                            aW.WriteStartArray("min");
                            foreach (float aValue in anAccessor.Min)
                            {
                                aW.WriteNumberValue(aValue);
                            }
                            aW.WriteEndArray();
                            aW.WriteStartArray("max");
                            foreach (float aValue in anAccessor.Max)
                            {
                                aW.WriteNumberValue(aValue);
                            }
                            aW.WriteEndArray();
                        }
                        aW.WriteEndObject();
                    }
                    aW.WriteEndArray();

                    aW.WriteStartArray("bufferViews");
                    foreach (BufferView aView in BufferViews)
                    {
                        aW.WriteStartObject();
                        aW.WriteNumber("buffer", 0);
                        aW.WriteNumber("byteOffset", aView.Offset);
                        aW.WriteNumber("byteLength", aView.Length);
                        aW.WriteNumber("target", aView.Target);
                        aW.WriteEndObject();
                    }
                    aW.WriteEndArray();

                    aW.WriteStartArray("buffers");
                    aW.WriteStartObject();
                    aW.WriteNumber("byteLength", Bin.Length);
                    if (BinUri is not null)
                    {
                        aW.WriteString("uri", BinUri);
                    }
                    aW.WriteEndObject();
                    aW.WriteEndArray();
                }
                aW.WriteEndObject();
            }
            return aStream.ToArray();
        }
    }

    // A part's accessors: per group, its position, normal and index accessors
    private sealed record PartAccessors(RWMesh_ProductMesh Mesh, List<(int Position, int Normal, int Indices)> Groups);

    private Model? build(XdeDoc_Tree theTree, string? theBinUri, Message_ProgressRange theProgress)
    {
        using var aPS = new Message_ProgressScope(theProgress, "Writing glTF", Options.MeshParameters is null ? 1 : 2);
        XdeDoc_Tree aTree = theTree;
        if (Options.MeshParameters is IMeshTools_Parameters aParams)
        {
            XdeDoc_Tree? aMeshed = theTree.Meshed(aParams, aPS.Next());
            if (aMeshed is null)
            {
                return null;
            }
            aTree = aMeshed;
        }

        double aUnit = Options.InputLengthUnit ?? aTree.LengthUnit?.UnitValue ?? 0.001;
        var aConverter = new RWMesh_CoordinateConverter(aUnit, Options.ZUpToYUp);
        var aModel = new Model { BinUri = theBinUri, Generator = Options.Generator };
        var aParts = new Dictionary<XdeDoc_Product, PartAccessors>(ReferenceEqualityComparer.Instance);
        var aMeshes = new Dictionary<(XdeDoc_Product, Quantity_ColorRGBA?), int>();
        var aMaterials = new Dictionary<Quantity_ColorRGBA, int>();
        int aNbParts = aTree.Products().Count(p => !p.IsAssembly);
        using var aPartsPS = new Message_ProgressScope(aPS.Next(), "Meshes", Math.Max(1, aNbParts));
        bool isCancelled = false;

        int materialOf(Quantity_ColorRGBA? theColor)
        {
            if (theColor is not Quantity_ColorRGBA aColor)
            {
                return -1;
            }
            if (!aMaterials.TryGetValue(aColor, out int anIndex))
            {
                anIndex = aModel.Materials.Count;
                aModel.Materials.Add(aColor);
                aMaterials.Add(aColor, anIndex);
            }
            return anIndex;
        }

        PartAccessors? accessorsOf(XdeDoc_Product thePart)
        {
            if (aParts.TryGetValue(thePart, out PartAccessors? aKnown))
            {
                return aKnown;
            }
            if (!aPartsPS.More())
            {
                isCancelled = true;
                return null;
            }
            aPartsPS.Next();
            RWMesh_ProductMesh aMesh = RWMesh_ProductMesh.Make(thePart, Options.WriteNormals);
            var aGroups = new List<(int, int, int)>();
            foreach (RWMesh_ProductMesh.Group aGroup in aMesh.Groups)
            {
                aGroups.Add(writeGroup(aModel, aGroup, aConverter));
            }
            var aResult = new PartAccessors(aMesh, aGroups);
            aParts.Add(thePart, aResult);
            return aResult;
        }

        int meshOf(XdeDoc_Product thePart, XdeDoc_Style theStyle)
        {
            Quantity_ColorRGBA? aNodeColor = theStyle.SurfaceColor ?? theStyle.CurveColor;
            if (aMeshes.TryGetValue((thePart, aNodeColor), out int aKnown))
            {
                return aKnown;
            }
            PartAccessors? anAccessors = accessorsOf(thePart);
            if (anAccessors is null || anAccessors.Groups.Count == 0)
            {
                aMeshes[(thePart, aNodeColor)] = -1;
                return -1;
            }
            var aPrimitives = new List<Primitive>();
            for (int i = 0; i < anAccessors.Groups.Count; i++)
            {
                (int aPos, int aNormal, int anIndices) = anAccessors.Groups[i];
                Quantity_ColorRGBA? aColor = anAccessors.Mesh.Groups[i].Color ?? aNodeColor;
                aPrimitives.Add(new Primitive(aPos, aNormal, anIndices, materialOf(aColor)));
            }
            aModel.Meshes.Add(new Mesh(thePart.Name, aPrimitives));
            int anIndex = aModel.Meshes.Count - 1;
            aMeshes[(thePart, aNodeColor)] = anIndex;
            return anIndex;
        }

        int visit(XdeDoc_Instance theInstance, XdeDoc_Style theParentStyle)
        {
            XdeDoc_Product aProduct = theInstance.Product;
            XdeDoc_Style aStyle = theParentStyle.Merged(aProduct.Attributes).Merged(theInstance.Attributes);
            if (aTree.IsInHiddenLayer(aProduct.Attributes) || aTree.IsInHiddenLayer(theInstance.Attributes))
            {
                aStyle = aStyle with { IsVisible = false };
            }
            if (!aStyle.IsVisible && !Options.IncludeHidden)
            {
                return -1;
            }
            var aNode = new Node
            {
                Name = theInstance.DisplayName,
                Matrix = theInstance.Location.IsIdentity() ? null : aConverter.MapMatrix(theInstance.Location.Transformation()),
            };
            aModel.Nodes.Add(aNode);
            int anIndex = aModel.Nodes.Count - 1;
            if (aProduct.IsAssembly)
            {
                foreach (XdeDoc_Instance aComponent in aProduct.Components)
                {
                    int aChild = visit(aComponent, aStyle);
                    if (isCancelled)
                    {
                        return -1;
                    }
                    if (aChild >= 0)
                    {
                        aNode.Children.Add(aChild);
                    }
                }
            }
            else
            {
                aNode.Mesh = meshOf(aProduct, aStyle);
            }
            return anIndex;
        }

        foreach (XdeDoc_Instance aRoot in aTree.Roots)
        {
            int aNode = visit(aRoot, XdeDoc_Style.Default);
            if (isCancelled)
            {
                return null;
            }
            if (aNode >= 0)
            {
                aModel.SceneNodes.Add(aNode);
            }
        }
        return aModel;
    }

    private static (int Position, int Normal, int Indices) writeGroup(Model theModel, RWMesh_ProductMesh.Group theGroup, RWMesh_CoordinateConverter theConverter)
    {
        int aNbNodes = theGroup.NbNodes;
        byte[] aPositions = new byte[aNbNodes * 12];
        float[] aMin = [float.MaxValue, float.MaxValue, float.MaxValue];
        float[] aMax = [float.MinValue, float.MinValue, float.MinValue];
        for (int i = 0; i < aNbNodes; i++)
        {
            double aX = theGroup.Positions[3 * i], aY = theGroup.Positions[3 * i + 1], aZ = theGroup.Positions[3 * i + 2];
            theConverter.Map(ref aX, ref aY, ref aZ);
            float[] aP = [(float)aX, (float)aY, (float)aZ];
            for (int k = 0; k < 3; k++)
            {
                BinaryPrimitives.WriteSingleLittleEndian(aPositions.AsSpan(12 * i + 4 * k), aP[k]);
                aMin[k] = Math.Min(aMin[k], aP[k]);
                aMax[k] = Math.Max(aMax[k], aP[k]);
            }
        }
        theModel.Accessors.Add(new Accessor(theModel.AddView(aPositions, ARRAY_BUFFER), FLOAT, aNbNodes, "VEC3", aMin, aMax));
        int aPosition = theModel.Accessors.Count - 1;

        int aNormal = -1;
        if (theGroup.Normals.Count == theGroup.Positions.Count)
        {
            byte[] aNormals = new byte[aNbNodes * 12];
            for (int i = 0; i < aNbNodes; i++)
            {
                double aX = theGroup.Normals[3 * i], aY = theGroup.Normals[3 * i + 1], aZ = theGroup.Normals[3 * i + 2];
                theConverter.Map(ref aX, ref aY, ref aZ, true);
                double aLength = Math.Sqrt(aX * aX + aY * aY + aZ * aZ);
                if (aLength > 0.0)
                {
                    aX /= aLength;
                    aY /= aLength;
                    aZ /= aLength;
                }
                BinaryPrimitives.WriteSingleLittleEndian(aNormals.AsSpan(12 * i), (float)aX);
                BinaryPrimitives.WriteSingleLittleEndian(aNormals.AsSpan(12 * i + 4), (float)aY);
                BinaryPrimitives.WriteSingleLittleEndian(aNormals.AsSpan(12 * i + 8), (float)aZ);
            }
            theModel.Accessors.Add(new Accessor(theModel.AddView(aNormals, ARRAY_BUFFER), FLOAT, aNbNodes, "VEC3", null, null));
            aNormal = theModel.Accessors.Count - 1;
        }

        // glTF: an index may not be the type's largest value (primitive restart)
        bool isShort = aNbNodes <= ushort.MaxValue;
        int aNbIndices = theGroup.Indices.Count;
        byte[] anIndices = new byte[aNbIndices * (isShort ? 2 : 4)];
        for (int i = 0; i < aNbIndices; i++)
        {
            if (isShort)
            {
                BinaryPrimitives.WriteUInt16LittleEndian(anIndices.AsSpan(2 * i), (ushort)theGroup.Indices[i]);
            }
            else
            {
                BinaryPrimitives.WriteUInt32LittleEndian(anIndices.AsSpan(4 * i), (uint)theGroup.Indices[i]);
            }
        }
        theModel.Accessors.Add(new Accessor(theModel.AddView(anIndices, ELEMENT_ARRAY_BUFFER),
                                            isShort ? UNSIGNED_SHORT : UNSIGNED_INT, aNbIndices, "SCALAR", null, null));
        return (aPosition, aNormal, theModel.Accessors.Count - 1);
    }
}
