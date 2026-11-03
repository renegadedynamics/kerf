// New in OCCT.NET (unit 8T): glTF 2.0 read fresh into the assembly tree with System.Text.Json (decisions 1166, 1236).
// Not a port of OCCT's RWGltf_CafReader, which needs XCAF and RapidJSON; its result is kept: each primitive a face
// holding only its triangulation (BRep_Builder::MakeFace with a Poly_Triangulation, as RWMesh does), the meshes' names
// and colours, the node hierarchy, glTF's Y-up turned Z-up and the metres turned into the model's unit.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Buffers.Binary;
using System.Text;
using System.Text.Json;

namespace Occt;

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
/// Hostile input (<see cref="RWGltf_ReadLimits"/>): the file and buffer sizes, the node count and depth, the triangle
/// count and the JSON nesting are bounded; every index and accessor range is checked against what it refers to; a
/// node may have one parent; required extensions not supported, sparse accessors and data that isn't 32-bit floats for
/// positions and normals are refused. An error stops the reading (<see cref="Error"/>) and gives no tree. External
/// buffers are resolved by the caller (<see cref="Read"/>); <see cref="ReadFile"/> resolves them in the file's
/// folder only. The tree's shapes are frozen. Cancellation through the progress range.
/// </para>
/// </remarks>
public sealed class RWGltf_TreeReader
{
    private const int FLOAT = 5126;
    private const int UNSIGNED_BYTE = 5121;
    private const int UNSIGNED_SHORT = 5123;
    private const int UNSIGNED_INT = 5125;
    private const int HARD_MAX_DEPTH = 4096; // the node recursion's own bound, whatever the limits say

    private readonly List<string> myWarnings = [];

    /// <summary>A reader with the options (the defaults if null).</summary>
    public RWGltf_TreeReader(RWGltf_ReadOptions? theOptions = null) => Options = theOptions ?? RWGltf_ReadOptions.Default;

    /// <summary>The options.</summary>
    public RWGltf_ReadOptions Options { get; }

    /// <summary>The last reading's error, or null.</summary>
    public string? Error { get; private set; }

    /// <summary>The last reading's warnings (what was skipped).</summary>
    public IReadOnlyList<string> Warnings => myWarnings;

    /// <summary>
    /// Reads glTF or binary glTF (recognised from the data) from the stream; external buffers are opened by
    /// <paramref name="theResolver"/> from their URIs (unescaped), and refused if it is null or gives null. Null on
    /// an error (see <see cref="Error"/>) or if cancelled.
    /// </summary>
    public XdeDoc_Tree? Read(Stream theStream, Func<string, Stream?>? theResolver = null, ProgressRange theProgress = default)
    {
        ArgumentNullException.ThrowIfNull(theStream);
        Error = null;
        myWarnings.Clear();
        try
        {
            byte[] aData = readAll(theStream, Options.Limits.MaxBytes, "the file");
            var aContext = new Context(this, theResolver);
            return aContext.Read(aData, theProgress);
        }
        catch (FormatError anError)
        {
            Error = anError.Message;
            return null;
        }
        catch (JsonException anError)
        {
            Error = "Invalid JSON: " + anError.Message;
            return null;
        }
        catch (IOException anError)
        {
            Error = "Read error: " + anError.Message;
            return null;
        }
    }

    /// <summary>
    /// Reads a glTF file; its external buffers are read from files in its folder (or below it) only.
    /// </summary>
    public XdeDoc_Tree? ReadFile(string theFilePath, ProgressRange theProgress = default)
    {
        string aFolder = Path.GetFullPath(Path.GetDirectoryName(Path.GetFullPath(theFilePath)) ?? ".");
        string aPrefix = aFolder.EndsWith(Path.DirectorySeparatorChar) ? aFolder : aFolder + Path.DirectorySeparatorChar;
        Stream? resolve(string theUri)
        {
            if (Path.IsPathRooted(theUri) || theUri.Contains(':', StringComparison.Ordinal))
            {
                return null;
            }
            string aPath = Path.GetFullPath(Path.Combine(aFolder, theUri));
            return aPath.StartsWith(aPrefix, StringComparison.OrdinalIgnoreCase) && File.Exists(aPath) ? File.OpenRead(aPath) : null;
        }
        using FileStream aFile = File.OpenRead(theFilePath);
        return Read(aFile, resolve, theProgress);
    }

    private sealed class FormatError(string theMessage) : Exception(theMessage);

    private static byte[] readAll(Stream theStream, long theMax, string theWhat)
    {
        if (theStream.CanSeek && theStream.Length - theStream.Position > theMax)
        {
            throw new FormatError($"{theWhat} is larger than the limit ({theMax} bytes)");
        }
        using var aBuffer = new MemoryStream();
        byte[] aChunk = new byte[81920];
        int aRead;
        while ((aRead = theStream.Read(aChunk, 0, aChunk.Length)) > 0)
        {
            if (aBuffer.Length + aRead > theMax)
            {
                throw new FormatError($"{theWhat} is larger than the limit ({theMax} bytes)");
            }
            aBuffer.Write(aChunk, 0, aRead);
        }
        return aBuffer.ToArray();
    }

    // ---------------------------------------------------------------------------------------------- the reading

    private sealed class Context(RWGltf_TreeReader theReader, Func<string, Stream?>? theResolver)
    {
        private readonly RWGltf_ReadLimits myLimits = theReader.Options.Limits;
        private readonly RWMesh_CoordinateConverter myToModel =
            new RWMesh_CoordinateConverter(theReader.Options.OutputLengthUnit, theReader.Options.YUpToZUp).Inverse();
        private JsonElement myRoot;
        private byte[]? myGlbBin;
        private readonly Dictionary<int, byte[]> myBuffers = [];
        private long myBufferTotal;
        private long myTriangles;
        private readonly Dictionary<int, XdeDoc_Product?> myMeshProducts = [];
        private ProgressScope? myPS;

        public XdeDoc_Tree? Read(byte[] theData, ProgressRange theProgress)
        {
            ReadOnlySpan<byte> aJson = theData;
            if (theData.Length >= 4 && BinaryPrimitives.ReadUInt32LittleEndian(theData) == 0x46546C67)
            {
                aJson = parseGlb(theData);
            }
            var anOptions = new JsonDocumentOptions { MaxDepth = myLimits.MaxJsonDepth };
            using JsonDocument aDoc = JsonDocument.Parse(aJson.ToArray(), anOptions);
            myRoot = aDoc.RootElement;
            if (myRoot.ValueKind != JsonValueKind.Object)
            {
                throw new FormatError("The glTF JSON isn't an object");
            }
            checkAsset();

            JsonElement[] aNodes = array(myRoot, "nodes");
            if (aNodes.Length > myLimits.MaxNodes)
            {
                throw new FormatError($"{aNodes.Length} nodes, more than the limit ({myLimits.MaxNodes})");
            }
            int[] aParents = parents(aNodes);
            int[] aRootNodes = sceneRoots(aNodes, aParents);

            int aNbMeshes = array(myRoot, "meshes").Length;
            using var aPS = new ProgressScope(theProgress, "Reading glTF", Math.Max(1, aNbMeshes));
            myPS = aPS;
            var aRoots = new List<XdeDoc_Instance>();
            foreach (int aNode in aRootNodes)
            {
                XdeDoc_Instance? anInstance = visit(aNodes, aNode, null, 1);
                if (!aPS.More())
                {
                    return null;
                }
                if (anInstance is not null)
                {
                    aRoots.Add(anInstance);
                }
            }
            var aUnit = new XdeDoc_LengthUnit(theReader.Options.OutputLengthUnit == 0.001 ? "MM" : "", theReader.Options.OutputLengthUnit);
            return new XdeDoc_Tree(aRoots, null, aUnit);
        }

        private ReadOnlySpan<byte> parseGlb(byte[] theData)
        {
            if (theData.Length < 20)
            {
                throw new FormatError("GLB: truncated header");
            }
            uint aVersion = BinaryPrimitives.ReadUInt32LittleEndian(theData.AsSpan(4));
            uint aLength = BinaryPrimitives.ReadUInt32LittleEndian(theData.AsSpan(8));
            if (aVersion != 2)
            {
                throw new FormatError($"GLB: version {aVersion}, not 2");
            }
            if (aLength > theData.Length || aLength < 20)
            {
                throw new FormatError("GLB: the length in the header doesn't fit the data");
            }
            ReadOnlySpan<byte> aJson = default;
            long anOffset = 12;
            int aChunk = 0;
            while (anOffset + 8 <= aLength)
            {
                uint aChunkLength = BinaryPrimitives.ReadUInt32LittleEndian(theData.AsSpan((int)anOffset));
                uint aChunkType = BinaryPrimitives.ReadUInt32LittleEndian(theData.AsSpan((int)anOffset + 4));
                long aStart = anOffset + 8;
                if (aStart + aChunkLength > aLength)
                {
                    throw new FormatError("GLB: a chunk runs past the end of the data");
                }
                if (aChunk == 0)
                {
                    if (aChunkType != 0x4E4F534A)
                    {
                        throw new FormatError("GLB: the first chunk isn't JSON");
                    }
                    aJson = theData.AsSpan((int)aStart, (int)aChunkLength);
                }
                else if (aChunk == 1 && aChunkType == 0x004E4942)
                {
                    myGlbBin = theData.AsSpan((int)aStart, (int)aChunkLength).ToArray();
                }
                anOffset = aStart + ((aChunkLength + 3L) & ~3L);
                aChunk++;
            }
            if (aChunk == 0)
            {
                throw new FormatError("GLB: no JSON chunk");
            }
            return aJson;
        }

        private void checkAsset()
        {
            if (!myRoot.TryGetProperty("asset", out JsonElement anAsset) || anAsset.ValueKind != JsonValueKind.Object
                || !anAsset.TryGetProperty("version", out JsonElement aVersion) || aVersion.ValueKind != JsonValueKind.String)
            {
                throw new FormatError("No asset version: not glTF 2.0");
            }
            string aText = aVersion.GetString() ?? "";
            if (!aText.StartsWith("2.", StringComparison.Ordinal))
            {
                throw new FormatError($"glTF version {aText}, not 2.x");
            }
            foreach (JsonElement anExt in array(myRoot, "extensionsRequired"))
            {
                string aName = anExt.ValueKind == JsonValueKind.String ? anExt.GetString() ?? "" : "";
                // extensions of materials and textures change only the looks: the geometry reads without them
                if (aName.StartsWith("KHR_materials_", StringComparison.Ordinal) || aName.StartsWith("KHR_texture_", StringComparison.Ordinal))
                {
                    theReader.myWarnings.Add($"Required extension {aName} ignored (materials only)");
                    continue;
                }
                throw new FormatError($"Required extension not supported: {aName}");
            }
        }

        // ------------------------------------------------------------------------------------------ JSON access

        private static JsonElement[] array(JsonElement theObject, string theName)
        {
            if (!theObject.TryGetProperty(theName, out JsonElement anArray))
            {
                return [];
            }
            if (anArray.ValueKind != JsonValueKind.Array)
            {
                throw new FormatError($"'{theName}' isn't an array");
            }
            return [.. anArray.EnumerateArray()];
        }

        private static JsonElement element(JsonElement[] theArray, int theIndex, string theWhat)
        {
            if (theIndex < 0 || theIndex >= theArray.Length)
            {
                throw new FormatError($"{theWhat} {theIndex} doesn't exist");
            }
            if (theArray[theIndex].ValueKind != JsonValueKind.Object)
            {
                throw new FormatError($"{theWhat} {theIndex} isn't an object");
            }
            return theArray[theIndex];
        }

        private static long integer(JsonElement theObject, string theName, long theDefault, bool isRequired = false)
        {
            if (!theObject.TryGetProperty(theName, out JsonElement aValue))
            {
                return isRequired ? throw new FormatError($"'{theName}' is missing") : theDefault;
            }
            if (aValue.ValueKind != JsonValueKind.Number || !aValue.TryGetInt64(out long aResult) || aResult < 0)
            {
                throw new FormatError($"'{theName}' isn't a non-negative integer");
            }
            return aResult;
        }

        private static int index(JsonElement theObject, string theName, bool isRequired = false)
        {
            long aValue = integer(theObject, theName, -1, isRequired);
            return aValue > int.MaxValue ? throw new FormatError($"'{theName}' is too large") : (int)aValue;
        }

        private static string? text(JsonElement theObject, string theName)
            => theObject.TryGetProperty(theName, out JsonElement aValue) && aValue.ValueKind == JsonValueKind.String ? aValue.GetString() : null;

        private static double[]? numbers(JsonElement theObject, string theName, int theCount)
        {
            if (!theObject.TryGetProperty(theName, out JsonElement aValue))
            {
                return null;
            }
            if (aValue.ValueKind != JsonValueKind.Array || aValue.GetArrayLength() != theCount)
            {
                throw new FormatError($"'{theName}' isn't an array of {theCount} numbers");
            }
            double[] aResult = new double[theCount];
            int i = 0;
            foreach (JsonElement aNumber in aValue.EnumerateArray())
            {
                if (aNumber.ValueKind != JsonValueKind.Number || !double.IsFinite(aResult[i++] = aNumber.GetDouble()))
                {
                    throw new FormatError($"'{theName}' isn't an array of {theCount} numbers");
                }
            }
            return aResult;
        }

        // ------------------------------------------------------------------------------------------ the hierarchy

        private static int[] parents(JsonElement[] theNodes)
        {
            int[] aParents = new int[theNodes.Length];
            Array.Fill(aParents, -1);
            for (int i = 0; i < theNodes.Length; i++)
            {
                JsonElement aNode = element(theNodes, i, "Node");
                foreach (JsonElement aChild in array(aNode, "children"))
                {
                    if (aChild.ValueKind != JsonValueKind.Number || !aChild.TryGetInt32(out int c) || c < 0 || c >= theNodes.Length)
                    {
                        throw new FormatError($"Node {i} has an invalid child");
                    }
                    if (aParents[c] >= 0 || c == i)
                    {
                        throw new FormatError($"Node {c} has more than one parent, or is its own child");
                    }
                    aParents[c] = i;
                }
            }
            return aParents;
        }

        private int[] sceneRoots(JsonElement[] theNodes, int[] theParents)
        {
            JsonElement[] aScenes = array(myRoot, "scenes");
            if (aScenes.Length == 0)
            {
                // no scene: every parentless node (a node in a cycle has a parent, so cycles aren't reached)
                return [.. Enumerable.Range(0, theNodes.Length).Where(i => theParents[i] < 0)];
            }
            int aScene = myRoot.TryGetProperty("scene", out _) ? index(myRoot, "scene") : 0;
            JsonElement aSceneObject = element(aScenes, aScene, "Scene");
            var aRoots = new List<int>();
            foreach (JsonElement aNode in array(aSceneObject, "nodes"))
            {
                if (aNode.ValueKind != JsonValueKind.Number || !aNode.TryGetInt32(out int n) || n < 0 || n >= theNodes.Length)
                {
                    throw new FormatError("A scene has an invalid node");
                }
                if (theParents[n] >= 0)
                {
                    throw new FormatError($"Scene node {n} isn't a root node");
                }
                aRoots.Add(n);
            }
            return [.. aRoots];
        }

        private XdeDoc_Instance? visit(JsonElement[] theNodes, int theNode, double[]? thePending, int theDepth)
        {
            if (theDepth > Math.Min(myLimits.MaxDepth, HARD_MAX_DEPTH))
            {
                throw new FormatError($"The node hierarchy is deeper than the limit ({Math.Min(myLimits.MaxDepth, HARD_MAX_DEPTH)})");
            }
            if (myPS is not null && !myPS.More())
            {
                return null;
            }
            JsonElement aNode = theNodes[theNode];
            double[] aMatrix = myToModel.MapMatrix(localMatrix(aNode));
            if (thePending is not null)
            {
                aMatrix = multiply(thePending, aMatrix);
            }
            Placement aLocation = new();
            double[]? aBake = null;
            if (rigid(aMatrix, out Transform3D aTrsf, out bool isIdentity))
            {
                if (!isIdentity)
                {
                    aLocation = new Placement(aTrsf);
                }
            }
            else
            {
                aBake = aMatrix;
            }

            string? aName = text(aNode, "name");
            XdeDoc_Product? aMeshProduct = null;
            if (aNode.TryGetProperty("mesh", out _))
            {
                aMeshProduct = meshProduct(index(aNode, "mesh"), aBake);
            }
            var aChildren = new List<XdeDoc_Instance>();
            foreach (JsonElement aChild in array(aNode, "children"))
            {
                XdeDoc_Instance? anInstance = visit(theNodes, aChild.GetInt32(), aBake, theDepth + 1);
                if (anInstance is not null)
                {
                    aChildren.Add(anInstance);
                }
            }
            if (aChildren.Count == 0 && aMeshProduct is not null)
            {
                XdeDoc_Attributes anAttributes = aName is not null && aName != aMeshProduct.Name
                                                   ? XdeDoc_Attributes.Empty.WithName(aName)
                                                   : XdeDoc_Attributes.Empty;
                return new XdeDoc_Instance(aMeshProduct, aLocation, anAttributes);
            }
            var aComponents = new List<XdeDoc_Instance>();
            if (aMeshProduct is not null)
            {
                aComponents.Add(new XdeDoc_Instance(aMeshProduct));
            }
            aComponents.AddRange(aChildren);
            return new XdeDoc_Instance(XdeDoc_Product.Assembly(aComponents, XdeDoc_Attributes.Empty.WithName(aName)), aLocation);
        }

        // The node's matrix (column-major), from "matrix" or from translation, rotation and scale
        private static double[] localMatrix(JsonElement theNode)
        {
            if (numbers(theNode, "matrix", 16) is double[] aMatrix)
            {
                return aMatrix;
            }
            double[] aT = numbers(theNode, "translation", 3) ?? [0, 0, 0];
            double[] aQ = numbers(theNode, "rotation", 4) ?? [0, 0, 0, 1];
            double[] aS = numbers(theNode, "scale", 3) ?? [1, 1, 1];
            double aNorm = Math.Sqrt(aQ[0] * aQ[0] + aQ[1] * aQ[1] + aQ[2] * aQ[2] + aQ[3] * aQ[3]);
            if (aNorm < 1e-12)
            {
                throw new FormatError("A node's rotation is a null quaternion");
            }
            double x = aQ[0] / aNorm, y = aQ[1] / aNorm, z = aQ[2] / aNorm, w = aQ[3] / aNorm;
            double[] aR =
            [
                1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w),
                2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y),
            ];
            double[] aResult = new double[16];
            for (int aCol = 0; aCol < 3; aCol++)
            {
                for (int aRow = 0; aRow < 3; aRow++)
                {
                    aResult[4 * aCol + aRow] = aR[3 * aCol + aRow] * aS[aCol];
                }
            }
            aResult[12] = aT[0];
            aResult[13] = aT[1];
            aResult[14] = aT[2];
            aResult[15] = 1.0;
            return aResult;
        }

        private static double[] multiply(double[] theA, double[] theB)
        {
            double[] aResult = new double[16];
            for (int aCol = 0; aCol < 4; aCol++)
            {
                for (int aRow = 0; aRow < 4; aRow++)
                {
                    double aSum = 0.0;
                    for (int k = 0; k < 4; k++)
                    {
                        aSum += theA[4 * k + aRow] * theB[4 * aCol + k];
                    }
                    aResult[4 * aCol + aRow] = aSum;
                }
            }
            return aResult;
        }

        // A rigid motion (orthonormal, not mirroring) as a gp_Trsf
        private static bool rigid(double[] theM, out Transform3D theTrsf, out bool isIdentity)
        {
            theTrsf = new Transform3D();
            isIdentity = false;
            const double aTol = 1e-6;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    double aDot = theM[4 * i] * theM[4 * j] + theM[4 * i + 1] * theM[4 * j + 1] + theM[4 * i + 2] * theM[4 * j + 2];
                    if (Math.Abs(aDot - (i == j ? 1.0 : 0.0)) > aTol)
                    {
                        return false;
                    }
                }
            }
            double aDet = theM[0] * (theM[5] * theM[10] - theM[9] * theM[6])
                        - theM[4] * (theM[1] * theM[10] - theM[9] * theM[2])
                        + theM[8] * (theM[1] * theM[6] - theM[5] * theM[2]);
            if (aDet <= 0.0 || Math.Abs(theM[3]) > aTol || Math.Abs(theM[7]) > aTol || Math.Abs(theM[11]) > aTol
                || Math.Abs(theM[15] - 1.0) > aTol)
            {
                return false;
            }
            isIdentity = Math.Abs(theM[0] - 1) <= 1e-12 && Math.Abs(theM[5] - 1) <= 1e-12 && Math.Abs(theM[10] - 1) <= 1e-12
                         && theM[12] == 0.0 && theM[13] == 0.0 && theM[14] == 0.0;
            if (!isIdentity)
            {
                theTrsf.SetValues(theM[0], theM[4], theM[8], theM[12],
                                  theM[1], theM[5], theM[9], theM[13],
                                  theM[2], theM[6], theM[10], theM[14]);
            }
            return true;
        }

        // ------------------------------------------------------------------------------------------ meshes

        private XdeDoc_Product? meshProduct(int theMesh, double[]? theBake)
        {
            if (theBake is null && myMeshProducts.TryGetValue(theMesh, out XdeDoc_Product? aKnown))
            {
                return aKnown;
            }
            JsonElement aMesh = element(array(myRoot, "meshes"), theMesh, "Mesh");
            if (theBake is null)
            {
                myPS?.Next();
            }
            string? aName = text(aMesh, "name");
            var aFaces = new List<(Face Face, ColorWithAlpha? Color)>();
            int aPrimIndex = 0;
            foreach (JsonElement aPrim in array(aMesh, "primitives"))
            {
                if (aPrim.ValueKind != JsonValueKind.Object)
                {
                    throw new FormatError($"Mesh {theMesh}: a primitive isn't an object");
                }
                Triangulation? aTri = triangulation(theMesh, aPrimIndex++, aPrim, theBake);
                if (aTri is null)
                {
                    continue;
                }
                Face aFace = new();
                new BrepBuilder().MakeFace(ref aFace, aTri);
                ColorWithAlpha? aColor = aPrim.TryGetProperty("material", out _) ? materialColor(index(aPrim, "material")) : null;
                aFaces.Add((aFace, aColor));
            }

            XdeDoc_Product? aProduct = null;
            if (aFaces.Count == 1)
            {
                aProduct = XdeDoc_Product.Part(aFaces[0].Face, new XdeDoc_Attributes { Name = aName, SurfaceColor = aFaces[0].Color });
            }
            else if (aFaces.Count > 1)
            {
                var aBuilder = new BrepBuilder();
                Compound aCompound = new();
                aBuilder.MakeCompound(ref aCompound);
                var aSubs = new List<XdeDoc_SubShape>();
                foreach ((Face aFace, ColorWithAlpha? aColor) in aFaces)
                {
                    aBuilder.Add(aCompound, aFace);
                    if (aColor is not null)
                    {
                        aSubs.Add(new XdeDoc_SubShape(aFace, new XdeDoc_Attributes { SurfaceColor = aColor }));
                    }
                }
                aProduct = XdeDoc_Product.Part(aCompound, XdeDoc_Attributes.Empty.WithName(aName), aSubs);
            }
            if (theBake is null)
            {
                myMeshProducts[theMesh] = aProduct;
            }
            return aProduct;
        }

        private ColorWithAlpha materialColor(int theMaterial)
        {
            JsonElement aMaterial = element(array(myRoot, "materials"), theMaterial, "Material");
            double[] aFactor = [1, 1, 1, 1];
            if (aMaterial.TryGetProperty("pbrMetallicRoughness", out JsonElement aPbr) && aPbr.ValueKind == JsonValueKind.Object)
            {
                aFactor = numbers(aPbr, "baseColorFactor", 4) ?? aFactor;
            }
            else if (aMaterial.TryGetProperty("extensions", out JsonElement anExts) && anExts.ValueKind == JsonValueKind.Object
                     && anExts.TryGetProperty("KHR_materials_pbrSpecularGlossiness", out JsonElement aSpecGloss)
                     && aSpecGloss.ValueKind == JsonValueKind.Object)
            {
                aFactor = numbers(aSpecGloss, "diffuseFactor", 4) ?? aFactor;
            }
            float clamp(double v) => (float)Math.Clamp(v, 0.0, 1.0);
            return new ColorWithAlpha(clamp(aFactor[0]), clamp(aFactor[1]), clamp(aFactor[2]), clamp(aFactor[3]));
        }

        private Triangulation? triangulation(int theMesh, int thePrim, JsonElement thePrimitive, double[]? theBake)
        {
            long aMode = integer(thePrimitive, "mode", 4);
            if (aMode is not (4 or 5 or 6))
            {
                theReader.myWarnings.Add($"Mesh {theMesh}, primitive {thePrim}: mode {aMode} (not triangles) skipped");
                return null;
            }
            if (!thePrimitive.TryGetProperty("attributes", out JsonElement anAttributes) || anAttributes.ValueKind != JsonValueKind.Object
                || !anAttributes.TryGetProperty("POSITION", out _))
            {
                theReader.myWarnings.Add($"Mesh {theMesh}, primitive {thePrim}: no POSITION, skipped");
                return null;
            }
            float[] aPositions = floats(index(anAttributes, "POSITION"), "VEC3", out int aNbNodes);
            float[]? aNormals = null;
            if (anAttributes.TryGetProperty("NORMAL", out _))
            {
                aNormals = floats(index(anAttributes, "NORMAL"), "VEC3", out int aNbNormals);
                if (aNbNormals != aNbNodes)
                {
                    throw new FormatError($"Mesh {theMesh}, primitive {thePrim}: NORMAL and POSITION counts differ");
                }
            }
            if (aNbNodes == 0)
            {
                return null;
            }
            int[] anIndices = thePrimitive.TryGetProperty("indices", out _)
                                ? indices(index(thePrimitive, "indices"), aNbNodes)
                                : [.. Enumerable.Range(0, aNbNodes)];

            // the triangles of the mode
            var aTriangles = new List<(int, int, int)>();
            if (aMode == 4)
            {
                for (int i = 0; i + 2 < anIndices.Length; i += 3)
                {
                    aTriangles.Add((anIndices[i], anIndices[i + 1], anIndices[i + 2]));
                }
            }
            else if (aMode == 5)
            {
                for (int i = 0; i + 2 < anIndices.Length; i++)
                {
                    aTriangles.Add(i % 2 == 0 ? (anIndices[i], anIndices[i + 1], anIndices[i + 2])
                                              : (anIndices[i + 1], anIndices[i], anIndices[i + 2]));
                }
            }
            else
            {
                for (int i = 1; i + 1 < anIndices.Length; i++)
                {
                    aTriangles.Add((anIndices[0], anIndices[i], anIndices[i + 1]));
                }
            }
            if (aTriangles.Count == 0)
            {
                return null;
            }
            myTriangles += aTriangles.Count;
            if (myTriangles > myLimits.MaxTriangles)
            {
                throw new FormatError($"More triangles than the limit ({myLimits.MaxTriangles})");
            }

            bool isMirrored = false;
            if (theBake is not null)
            {
                double aDet = theBake[0] * (theBake[5] * theBake[10] - theBake[9] * theBake[6])
                            - theBake[4] * (theBake[1] * theBake[10] - theBake[9] * theBake[2])
                            + theBake[8] * (theBake[1] * theBake[6] - theBake[5] * theBake[2]);
                isMirrored = aDet < 0.0;
            }
            var aTri = new Triangulation(aNbNodes, aTriangles.Count, false, aNormals is not null);
            for (int i = 0; i < aNbNodes; i++)
            {
                double aX = aPositions[3 * i], aY = aPositions[3 * i + 1], aZ = aPositions[3 * i + 2];
                myToModel.Map(ref aX, ref aY, ref aZ);
                if (theBake is not null)
                {
                    (aX, aY, aZ) = (theBake[0] * aX + theBake[4] * aY + theBake[8] * aZ + theBake[12],
                                    theBake[1] * aX + theBake[5] * aY + theBake[9] * aZ + theBake[13],
                                    theBake[2] * aX + theBake[6] * aY + theBake[10] * aZ + theBake[14]);
                }
                aTri.SetNode(i + 1, new Point3D(aX, aY, aZ));
                if (aNormals is not null)
                {
                    double aNx = aNormals[3 * i], aNy = aNormals[3 * i + 1], aNz = aNormals[3 * i + 2];
                    myToModel.Map(ref aNx, ref aNy, ref aNz, true);
                    if (theBake is not null)
                    {
                        (aNx, aNy, aNz) = normalTransformed(theBake, aNx, aNy, aNz);
                    }
                    double aLength = Math.Sqrt(aNx * aNx + aNy * aNy + aNz * aNz);
                    if (aLength > 0.0)
                    {
                        aTri.SetNormal(i + 1, new NCollection_Vec3<float>((float)(aNx / aLength), (float)(aNy / aLength), (float)(aNz / aLength)));
                    }
                    else
                    {
                        aTri.SetNormal(i + 1, new NCollection_Vec3<float>(0.0f, 0.0f, 1.0f));
                    }
                }
            }
            for (int i = 0; i < aTriangles.Count; i++)
            {
                (int a, int b, int c) = aTriangles[i];
                aTri.SetTriangle(i + 1, isMirrored ? new Triangle1(a + 1, c + 1, b + 1) : new Triangle1(a + 1, b + 1, c + 1));
            }
            return aTri;
        }

        // A normal by the inverse transpose of the matrix's linear part (its cofactors)
        private static (double, double, double) normalTransformed(double[] theM, double theX, double theY, double theZ)
        {
            double a = theM[0], b = theM[4], c = theM[8], d = theM[1], e = theM[5], f = theM[9], g = theM[2], h = theM[6], k = theM[10];
            double c00 = e * k - f * h, c01 = -(d * k - f * g), c02 = d * h - e * g;
            double c10 = -(b * k - c * h), c11 = a * k - c * g, c12 = -(a * h - b * g);
            double c20 = b * f - c * e, c21 = -(a * f - c * d), c22 = a * e - b * d;
            double aDet = a * c00 + b * c01 + c * c02;
            double aSign = aDet < 0.0 ? -1.0 : 1.0; // the cofactors of a mirror flip the normal: undone, the winding is swapped instead
            return (aSign * (c00 * theX + c01 * theY + c02 * theZ),
                    aSign * (c10 * theX + c11 * theY + c12 * theZ),
                    aSign * (c20 * theX + c21 * theY + c22 * theZ));
        }

        // ------------------------------------------------------------------------------------------ accessors

        private sealed record AccessorData(byte[] Buffer, long Offset, int Stride, int Count, int ComponentType, int NbComponents);

        private AccessorData accessor(int theAccessor, string theType)
        {
            JsonElement anAccessor = element(array(myRoot, "accessors"), theAccessor, "Accessor");
            if (anAccessor.TryGetProperty("sparse", out _))
            {
                throw new FormatError($"Accessor {theAccessor}: sparse accessors aren't supported");
            }
            string? aType = text(anAccessor, "type");
            if (aType != theType)
            {
                throw new FormatError($"Accessor {theAccessor}: type {aType}, {theType} expected");
            }
            int aNbComponents = theType == "VEC3" ? 3 : 1;
            int aComponentType = index(anAccessor, "componentType", true);
            int aComponentSize = aComponentType switch
            {
                5120 or UNSIGNED_BYTE => 1,
                5122 or UNSIGNED_SHORT => 2,
                UNSIGNED_INT or FLOAT => 4,
                _ => throw new FormatError($"Accessor {theAccessor}: unknown component type {aComponentType}"),
            };
            long aCount = integer(anAccessor, "count", 0, true);
            if (aCount > int.MaxValue / 3)
            {
                throw new FormatError($"Accessor {theAccessor}: count too large");
            }
            if (!anAccessor.TryGetProperty("bufferView", out _))
            {
                throw new FormatError($"Accessor {theAccessor}: no buffer view (zero-filled accessors aren't supported)");
            }
            JsonElement aView = element(array(myRoot, "bufferViews"), index(anAccessor, "bufferView"), "Buffer view");
            int aBufferIndex = index(aView, "buffer", true);
            long aViewOffset = integer(aView, "byteOffset", 0);
            long aViewLength = integer(aView, "byteLength", 0, true);
            long anElementSize = (long)aComponentSize * aNbComponents;
            long aStride = integer(aView, "byteStride", anElementSize);
            if (aStride < anElementSize || aStride > 252)
            {
                throw new FormatError($"Accessor {theAccessor}: invalid byte stride {aStride}");
            }
            long anAccessorOffset = integer(anAccessor, "byteOffset", 0);
            if (anAccessorOffset % aComponentSize != 0)
            {
                throw new FormatError($"Accessor {theAccessor}: misaligned");
            }
            byte[] aBuffer = buffer(aBufferIndex);
            if (aViewOffset + aViewLength > aBuffer.Length)
            {
                throw new FormatError($"Accessor {theAccessor}: its buffer view runs past its buffer");
            }
            if (aCount > 0 && anAccessorOffset + aStride * (aCount - 1) + anElementSize > aViewLength)
            {
                throw new FormatError($"Accessor {theAccessor}: runs past its buffer view");
            }
            return new AccessorData(aBuffer, aViewOffset + anAccessorOffset, (int)aStride, (int)aCount, aComponentType, aNbComponents);
        }

        private float[] floats(int theAccessor, string theType, out int theCount)
        {
            AccessorData aData = accessor(theAccessor, theType);
            if (aData.ComponentType != FLOAT)
            {
                throw new FormatError($"Accessor {theAccessor}: not 32-bit floats (quantized data isn't supported)");
            }
            theCount = aData.Count;
            float[] aResult = new float[aData.Count * aData.NbComponents];
            for (int i = 0; i < aData.Count; i++)
            {
                for (int k = 0; k < aData.NbComponents; k++)
                {
                    float aValue = BinaryPrimitives.ReadSingleLittleEndian(aData.Buffer.AsSpan((int)(aData.Offset + (long)aData.Stride * i + 4 * k)));
                    if (!float.IsFinite(aValue))
                    {
                        throw new FormatError($"Accessor {theAccessor}: a value isn't finite");
                    }
                    aResult[aData.NbComponents * i + k] = aValue;
                }
            }
            return aResult;
        }

        private int[] indices(int theAccessor, int theNbNodes)
        {
            AccessorData aData = accessor(theAccessor, "SCALAR");
            int[] aResult = new int[aData.Count];
            for (int i = 0; i < aData.Count; i++)
            {
                ReadOnlySpan<byte> aSpan = aData.Buffer.AsSpan((int)(aData.Offset + (long)aData.Stride * i));
                long aValue = aData.ComponentType switch
                {
                    UNSIGNED_BYTE => aSpan[0],
                    UNSIGNED_SHORT => BinaryPrimitives.ReadUInt16LittleEndian(aSpan),
                    UNSIGNED_INT => BinaryPrimitives.ReadUInt32LittleEndian(aSpan),
                    _ => throw new FormatError($"Accessor {theAccessor}: indices of component type {aData.ComponentType}"),
                };
                if (aValue >= theNbNodes)
                {
                    throw new FormatError($"Accessor {theAccessor}: index {aValue} out of the {theNbNodes} vertices");
                }
                aResult[i] = (int)aValue;
            }
            return aResult;
        }

        private byte[] buffer(int theBuffer)
        {
            if (myBuffers.TryGetValue(theBuffer, out byte[]? aKnown))
            {
                return aKnown;
            }
            JsonElement aBuffer = element(array(myRoot, "buffers"), theBuffer, "Buffer");
            long aLength = integer(aBuffer, "byteLength", 0, true);
            if (aLength > myLimits.MaxBufferBytes || myBufferTotal + aLength > myLimits.MaxBufferBytes)
            {
                throw new FormatError($"Buffer {theBuffer}: larger than the limit ({myLimits.MaxBufferBytes} bytes)");
            }
            string? anUri = text(aBuffer, "uri");
            byte[] aData;
            if (anUri is null)
            {
                aData = theBuffer == 0 && myGlbBin is not null
                          ? myGlbBin
                          : throw new FormatError($"Buffer {theBuffer}: no URI and no GLB binary chunk");
            }
            else if (anUri.StartsWith("data:", StringComparison.Ordinal))
            {
                int aComma = anUri.IndexOf(',', StringComparison.Ordinal);
                if (aComma < 0 || !anUri.AsSpan(0, aComma).EndsWith(";base64", StringComparison.Ordinal))
                {
                    throw new FormatError($"Buffer {theBuffer}: a data URI that isn't base64");
                }
                if ((anUri.Length - aComma - 1) / 4L * 3 > myLimits.MaxBufferBytes)
                {
                    throw new FormatError($"Buffer {theBuffer}: larger than the limit ({myLimits.MaxBufferBytes} bytes)");
                }
                try
                {
                    aData = Convert.FromBase64String(anUri[(aComma + 1)..]);
                }
                catch (FormatException)
                {
                    throw new FormatError($"Buffer {theBuffer}: invalid base64 data");
                }
            }
            else
            {
                Stream? aStream = theResolver?.Invoke(Uri.UnescapeDataString(anUri))
                                  ?? throw new FormatError($"Buffer {theBuffer}: '{anUri}' can't be opened");
                using (aStream)
                {
                    aData = readAll(aStream, myLimits.MaxBufferBytes - myBufferTotal, $"Buffer {theBuffer}");
                }
            }
            if (aData.Length < aLength)
            {
                throw new FormatError($"Buffer {theBuffer}: {aData.Length} bytes, {aLength} expected");
            }
            myBufferTotal += aLength;
            myBuffers[theBuffer] = aData;
            return aData;
        }
    }
}
