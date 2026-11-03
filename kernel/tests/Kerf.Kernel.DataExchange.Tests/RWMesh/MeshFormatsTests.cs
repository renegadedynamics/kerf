// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Gltf;
using Kerf.Engine.Exchange.Obj;
using Kerf.Engine.Exchange.ThreeMf;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Buffers.Binary;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;
using System.Xml.Linq;
using static Kerf.Kernel.DataExchange.Tests.MeshTestKit;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// The fresh mesh writers (glTF, OBJ, 3MF) and the glTF reader over the assembly tree (unit 8T): round trips (write,
/// read, same triangles, hierarchy, colours and names), the formats' rules checked in the tests (glTF 2.0's structure
/// and accessor bounds, 3MF core's parts, relationships, units, object order and winding), OBJ read back by a parser
/// here, and OCCT's own de_mesh Draw cases on the dataset's Lantern (gltf_read/lantern, gltf_write/lanternglb,
/// obj_write/lantern: native's face, triangle and node counts and its assembly structure).
/// </summary>
public class MeshFormatsTests
{
    // ---------------------------------------------------------------------------------------------- glTF

    private static byte[] WriteGlb(Tree theTree, GltfWriteOptions? theOptions = null)
    {
        var aStream = new MemoryStream();
        Assert.True(new GltfTreeWriter(theOptions).WriteGlb(theTree, aStream));
        return aStream.ToArray();
    }

    private static Tree ReadGltf(byte[] theData, Func<string, Stream?>? theResolver = null)
    {
        var aReader = new TreeReader();
        Tree? aTree = aReader.Read(new MemoryStream(theData), theResolver);
        Assert.True(aTree is not null, aReader.Error);
        return aTree;
    }

    /// <summary>Splits a GLB into its JSON and binary chunks, checking the container's rules.</summary>
    internal static (JsonDocument Json, byte[] Bin) SplitGlb(byte[] theGlb)
    {
        Assert.Equal(0x46546C67u, BinaryPrimitives.ReadUInt32LittleEndian(theGlb));
        Assert.Equal(2u, BinaryPrimitives.ReadUInt32LittleEndian(theGlb.AsSpan(4)));
        Assert.Equal((uint)theGlb.Length, BinaryPrimitives.ReadUInt32LittleEndian(theGlb.AsSpan(8)));
        int aJsonLength = (int)BinaryPrimitives.ReadUInt32LittleEndian(theGlb.AsSpan(12));
        Assert.Equal(0x4E4F534Au, BinaryPrimitives.ReadUInt32LittleEndian(theGlb.AsSpan(16)));
        Assert.Equal(0, aJsonLength % 4);
        JsonDocument aJson = JsonDocument.Parse(theGlb.AsMemory(20, aJsonLength));
        byte[] aBin = [];
        int anOffset = 20 + aJsonLength;
        if (anOffset < theGlb.Length)
        {
            int aBinLength = (int)BinaryPrimitives.ReadUInt32LittleEndian(theGlb.AsSpan(anOffset));
            Assert.Equal(0x004E4942u, BinaryPrimitives.ReadUInt32LittleEndian(theGlb.AsSpan(anOffset + 4)));
            Assert.Equal(0, aBinLength % 4);
            Assert.Equal(theGlb.Length, anOffset + 8 + aBinLength);
            aBin = theGlb.AsSpan(anOffset + 8, aBinLength).ToArray();
        }
        return (aJson, aBin);
    }

    /// <summary>
    /// glTF 2.0's structural rules (the Khronos validator isn't available offline): indices in range, a node with one
    /// parent, scene nodes roots, accessors inside their views and views inside the buffer, alignment, POSITION's min
    /// and max exact, indices below the vertex count and the type's largest value, unit normals, colours in [0, 1].
    /// </summary>
    internal static void ValidateGltf(JsonDocument theJson, byte[] theBin)
    {
        JsonElement aRoot = theJson.RootElement;
        Assert.Equal("2.0", aRoot.GetProperty("asset").GetProperty("version").GetString());
        JsonElement[] arr(string n) => aRoot.TryGetProperty(n, out JsonElement a) ? [.. a.EnumerateArray()] : [];
        JsonElement[] aNodes = arr("nodes"), aMeshes = arr("meshes"), anAccessors = arr("accessors"), aViews = arr("bufferViews"),
                      aBuffers = arr("buffers"), aMaterials = arr("materials"), aScenes = arr("scenes");

        int[] aParents = Enumerable.Repeat(-1, aNodes.Length).ToArray();
        for (int i = 0; i < aNodes.Length; i++)
        {
            if (aNodes[i].TryGetProperty("children", out JsonElement aChildren))
            {
                foreach (JsonElement c in aChildren.EnumerateArray())
                {
                    int aChild = c.GetInt32();
                    Assert.InRange(aChild, 0, aNodes.Length - 1);
                    Assert.Equal(-1, aParents[aChild]);
                    aParents[aChild] = i;
                }
            }
            if (aNodes[i].TryGetProperty("matrix", out JsonElement aMatrix))
            {
                Assert.Equal(16, aMatrix.GetArrayLength());
            }
            if (aNodes[i].TryGetProperty("mesh", out JsonElement aMesh))
            {
                Assert.InRange(aMesh.GetInt32(), 0, aMeshes.Length - 1);
            }
        }
        int aScene = aRoot.GetProperty("scene").GetInt32();
        Assert.InRange(aScene, 0, aScenes.Length - 1);
        foreach (JsonElement n in aScenes[aScene].GetProperty("nodes").EnumerateArray())
        {
            Assert.Equal(-1, aParents[n.GetInt32()]);
        }

        long aBufferLength = 0;
        if (aBuffers.Length > 0)
        {
            Assert.Single(aBuffers);
            aBufferLength = aBuffers[0].GetProperty("byteLength").GetInt64();
            Assert.True(theBin.Length >= aBufferLength);
        }
        foreach (JsonElement aView in aViews)
        {
            long anOffset = aView.GetProperty("byteOffset").GetInt64();
            Assert.True(anOffset + aView.GetProperty("byteLength").GetInt64() <= aBufferLength);
            Assert.Equal(0, anOffset % 4);
            Assert.Contains(aView.GetProperty("target").GetInt32(), new[] { 34962, 34963 });
        }

        float readFloat(JsonElement theAccessor, int theElement, int theComponent)
        {
            JsonElement aView = aViews[theAccessor.GetProperty("bufferView").GetInt32()];
            long anOffset = aView.GetProperty("byteOffset").GetInt64() + (theAccessor.TryGetProperty("byteOffset", out JsonElement o) ? o.GetInt64() : 0);
            return BinaryPrimitives.ReadSingleLittleEndian(theBin.AsSpan((int)(anOffset + 12 * theElement + 4 * theComponent)));
        }

        foreach (JsonElement anAccessor in anAccessors)
        {
            JsonElement aView = aViews[anAccessor.GetProperty("bufferView").GetInt32()];
            int aType = anAccessor.GetProperty("componentType").GetInt32();
            int aSize = aType == 5123 ? 2 : 4;
            int aComponents = anAccessor.GetProperty("type").GetString() == "VEC3" ? 3 : 1;
            long aCount = anAccessor.GetProperty("count").GetInt64();
            Assert.True(aCount > 0);
            Assert.True(aCount * aSize * aComponents <= aView.GetProperty("byteLength").GetInt64());
            Assert.Equal(0, aView.GetProperty("byteOffset").GetInt64() % aSize);
        }

        foreach (JsonElement aMesh in aMeshes)
        {
            foreach (JsonElement aPrim in aMesh.GetProperty("primitives").EnumerateArray())
            {
                Assert.Equal(4, aPrim.GetProperty("mode").GetInt32());
                JsonElement aPos = anAccessors[aPrim.GetProperty("attributes").GetProperty("POSITION").GetInt32()];
                Assert.Equal(5126, aPos.GetProperty("componentType").GetInt32());
                Assert.Equal("VEC3", aPos.GetProperty("type").GetString());
                int aCount = aPos.GetProperty("count").GetInt32();
                float[] aMin = [.. aPos.GetProperty("min").EnumerateArray().Select(e => e.GetSingle())];
                float[] aMax = [.. aPos.GetProperty("max").EnumerateArray().Select(e => e.GetSingle())];
                for (int k = 0; k < 3; k++)
                {
                    float aLo = float.MaxValue, aHi = float.MinValue;
                    for (int i = 0; i < aCount; i++)
                    {
                        float v = readFloat(aPos, i, k);
                        aLo = Math.Min(aLo, v);
                        aHi = Math.Max(aHi, v);
                    }
                    Assert.Equal(aLo, aMin[k]);
                    Assert.Equal(aHi, aMax[k]);
                }
                if (aPrim.GetProperty("attributes").TryGetProperty("NORMAL", out JsonElement aNormalIndex))
                {
                    JsonElement aNormal = anAccessors[aNormalIndex.GetInt32()];
                    Assert.Equal(aCount, aNormal.GetProperty("count").GetInt32());
                    for (int i = 0; i < aCount; i++)
                    {
                        float x = readFloat(aNormal, i, 0), y = readFloat(aNormal, i, 1), z = readFloat(aNormal, i, 2);
                        Assert.InRange(Math.Sqrt(x * x + y * y + z * z), 0.999, 1.001);
                    }
                }
                JsonElement anIndices = anAccessors[aPrim.GetProperty("indices").GetInt32()];
                Assert.Equal("SCALAR", anIndices.GetProperty("type").GetString());
                int anIndexType = anIndices.GetProperty("componentType").GetInt32();
                Assert.Contains(anIndexType, new[] { 5123, 5125 });
                int aNbIndices = anIndices.GetProperty("count").GetInt32();
                Assert.Equal(0, aNbIndices % 3);
                JsonElement anIndexView = aViews[anIndices.GetProperty("bufferView").GetInt32()];
                long anIndexOffset = anIndexView.GetProperty("byteOffset").GetInt64();
                for (int i = 0; i < aNbIndices; i++)
                {
                    long v = anIndexType == 5123
                               ? BinaryPrimitives.ReadUInt16LittleEndian(theBin.AsSpan((int)(anIndexOffset + 2 * i)))
                               : BinaryPrimitives.ReadUInt32LittleEndian(theBin.AsSpan((int)(anIndexOffset + 4 * i)));
                    Assert.True(v < aCount);
                    Assert.True(v != (anIndexType == 5123 ? ushort.MaxValue : uint.MaxValue));
                }
                if (aPrim.TryGetProperty("material", out JsonElement aMaterial))
                {
                    Assert.InRange(aMaterial.GetInt32(), 0, aMaterials.Length - 1);
                }
            }
        }
        foreach (JsonElement aMaterial in aMaterials)
        {
            foreach (JsonElement aValue in aMaterial.GetProperty("pbrMetallicRoughness").GetProperty("baseColorFactor").EnumerateArray())
            {
                Assert.InRange(aValue.GetDouble(), 0.0, 1.0);
            }
        }
    }

    [Fact]
    public void GltfRoundTripKeepsTrianglesHierarchyColoursAndNames()
    {
        Tree aTree = SampleTree();
        byte[] aGlb = WriteGlb(aTree);
        Assert.Equal(aGlb, WriteGlb(aTree)); // deterministic
        (JsonDocument aJson, byte[] aBin) = SplitGlb(aGlb);
        ValidateGltf(aJson, aBin);

        // instancing: the red box's mesh is shared, the green instance has its own (another material)
        JsonElement aRoot = aJson.RootElement;
        Assert.Equal(4, aRoot.GetProperty("nodes").GetArrayLength());     // the assembly and its 3 components
        Assert.Equal(3, aRoot.GetProperty("meshes").GetArrayLength());    // box red, box green, plate
        Assert.Equal(3, aRoot.GetProperty("materials").GetArrayLength()); // red, green, blue
        JsonElement aBlue = aRoot.GetProperty("materials")[2];
        Assert.Equal("BLEND", aBlue.GetProperty("alphaMode").GetString());

        Tree aBack = ReadGltf(aGlb);
        Assert.Equal(WorldTriangles(aTree), WorldTriangles(aBack));
        IReadOnlyList<FlatItem> aBefore = aTree.Flatten(), anAfter = aBack.Flatten();
        Assert.Equal(aBefore.Count, anAfter.Count);
        for (int i = 0; i < aBefore.Count; i++)
        {
            Assert.Equal(aBefore[i].NamePath, anAfter[i].NamePath);
            Assert.Equal(aBefore[i].Style.SurfaceColor, anAfter[i].Style.SurfaceColor);
        }
        // the plate's blue face: a sub-shape colour again
        SubShape aSub = Assert.Single(anAfter[2].Product.SubShapes);
        Assert.Equal(Blue, aSub.Attributes.SurfaceColor);
        // the read tree is in millimetres, Z up
        Assert.Equal(0.001, aBack.LengthUnit!.UnitValue);

        // .gltf with its .bin: the same tree
        var aJsonStream = new MemoryStream();
        var aBinStream = new MemoryStream();
        Assert.True(new GltfTreeWriter().WriteGltf(aTree, aJsonStream, aBinStream, "model.bin"));
        Assert.Equal(aBin.Length, pad4((int)aBinStream.Length));
        ValidateGltf(JsonDocument.Parse(aJsonStream.ToArray()), aBinStream.ToArray());
        Tree aBack2 = ReadGltf(aJsonStream.ToArray(), u => u == "model.bin" ? new MemoryStream(aBinStream.ToArray()) : null);
        Assert.Equal(WorldTriangles(aTree), WorldTriangles(aBack2));
        Assert.Null(new TreeReader().Read(new MemoryStream(aJsonStream.ToArray())));
    }

    private static int pad4(int theLength) => (theLength + 3) & ~3;

    [Fact]
    public void GltfWriterConvertsUnitsAndAxes()
    {
        DocumentProduct aPart = DocumentProduct.Part(MeshedBox(1000, 2000, 3000), "B");
        var aTree = new Tree(aPart.Placed(Translation(0, 0, 500)));
        (JsonDocument aJson, _) = SplitGlb(WriteGlb(aTree));
        JsonElement aRoot = aJson.RootElement;
        JsonElement aPos = aRoot.GetProperty("accessors")[aRoot.GetProperty("meshes")[0].GetProperty("primitives")[0].GetProperty("attributes").GetProperty("POSITION").GetInt32()];
        // millimetres to metres, (x, y, z) to (x, z, -y)
        Assert.Equal(new[] { 0.0f, 0.0f, -2.0f }, aPos.GetProperty("min").EnumerateArray().Select(e => e.GetSingle()));
        Assert.Equal(new[] { 1.0f, 3.0f, 0.0f }, aPos.GetProperty("max").EnumerateArray().Select(e => e.GetSingle()));
        double[] aMatrix = [.. aRoot.GetProperty("nodes")[0].GetProperty("matrix").EnumerateArray().Select(e => e.GetDouble())];
        Assert.Equal(0.5, aMatrix[13], 12); // Z 500 mm is Y 0.5 m
        Assert.Equal(1.0, aMatrix[0], 12);

        // without the conversions
        (aJson, _) = SplitGlb(WriteGlb(aTree, new GltfWriteOptions { ZUpToYUp = false, InputLengthUnit = 1.0 }));
        aRoot = aJson.RootElement;
        aPos = aRoot.GetProperty("accessors")[0];
        Assert.Equal(new[] { 1000.0f, 2000.0f, 3000.0f }, aPos.GetProperty("max").EnumerateArray().Select(e => e.GetSingle()));
    }

    [Fact]
    public void GltfWriterMeshesOnRequestAndLeavesHiddenParts()
    {
        Tree aTree = SampleTree(false); // not meshed
        (JsonDocument aJson, _) = SplitGlb(WriteGlb(aTree));
        Assert.False(aJson.RootElement.TryGetProperty("meshes", out _));
        Assert.False(aJson.RootElement.TryGetProperty("buffers", out _));

        byte[] aGlb = WriteGlb(aTree, new GltfWriteOptions { MeshParameters = MeshParameters });
        Assert.Equal(WorldTriangles(aTree.Meshed(MeshParameters)!), WorldTriangles(ReadGltf(aGlb)));
        Assert.Null(Tool.Triangulation(ShapeCasts.Face(new Explorer(aTree.Roots[0].Product.Components[0].Product.Shape, ShapeType.Face).Current()), out _));

        Tree aHidden = SampleTree().WithLayers([new Layer("L1", false)]);
        Assert.Equal(2, ReadGltf(WriteGlb(aHidden)).Flatten().Count);
        Assert.Equal(3, ReadGltf(WriteGlb(aHidden, new GltfWriteOptions { IncludeHidden = true })).Flatten().Count);
    }

    [Fact]
    public void WritersStopWhenCancelled()
    {
        using var aCancel = new CancellationTokenSource();
        aCancel.Cancel();
        Tree aTree = SampleTree();
        ProgressRange aRange() => new ProgressIndicator(aCancel.Token).Start();
        Assert.False(new GltfTreeWriter().WriteGlb(aTree, new MemoryStream(), aRange()));
        Assert.False(new ObjTreeWriter().Write(aTree, new MemoryStream(), new MemoryStream(), "m.mtl", aRange()));
        Assert.False(new TreeWriter().Write(aTree, new MemoryStream(), aRange()));
        Assert.Null(new TreeReader().Read(new MemoryStream(WriteGlb(aTree)), null, aRange()));
    }

    [Fact]
    public void FilePathsWriteBesideEachOther()
    {
        string aFolder = Path.Combine(Path.GetTempPath(), "occtnet-p8t-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(aFolder);
        try
        {
            Tree aTree = SampleTree();
            string aGltf = Path.Combine(aFolder, "a model.gltf");
            Assert.True(new GltfTreeWriter().Perform(aTree, aGltf));
            Assert.True(File.Exists(Path.Combine(aFolder, "a model.bin")));
            Assert.Contains("a%20model.bin", File.ReadAllText(aGltf), StringComparison.Ordinal);
            Tree? aBack = new TreeReader().ReadFile(aGltf);
            Assert.NotNull(aBack);
            Assert.Equal(WorldTriangles(aTree), WorldTriangles(aBack));

            Assert.True(new GltfTreeWriter().Perform(aTree, Path.Combine(aFolder, "m.glb")));
            Assert.NotNull(new TreeReader().ReadFile(Path.Combine(aFolder, "m.glb")));
            Assert.True(new ObjTreeWriter().Perform(aTree, Path.Combine(aFolder, "m.obj")));
            Assert.True(File.Exists(Path.Combine(aFolder, "m.mtl")));
            Assert.True(new TreeWriter().Perform(aTree, Path.Combine(aFolder, "m.3mf")));
        }
        finally
        {
            Directory.Delete(aFolder, true);
        }
    }

    // ---------------------------------------------------------------------------------------------- OBJ

    /// <summary>OBJ read back: the world triangles of its faces (with or without normals), the materials used.</summary>
    internal static (List<string> Triangles, List<string> Materials, int NbVertices) ParseObj(string theObj)
    {
        var aVertices = new List<Point3D>();
        var aTriangles = new List<string>();
        var aMaterials = new List<string>();
        foreach (string aLine in theObj.Split('\n'))
        {
            string[] aParts = aLine.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (aParts.Length == 0)
            {
                continue;
            }
            double d(int i) => double.Parse(aParts[i], CultureInfo.InvariantCulture);
            int v(int i) => int.Parse(aParts[i].Split('/')[0], CultureInfo.InvariantCulture) - 1;
            switch (aParts[0])
            {
                case "v":
                    aVertices.Add(new Point3D(d(1), d(2), d(3)));
                    break;
                case "f":
                    Assert.Equal(4, aParts.Length);
                    aTriangles.Add(Canonical(aVertices[v(1)], aVertices[v(2)], aVertices[v(3)]));
                    break;
                case "usemtl":
                    aMaterials.Add(aParts[1]);
                    break;
            }
        }
        aTriangles.Sort(StringComparer.Ordinal);
        return (aTriangles, aMaterials, aVertices.Count);
    }

    [Fact]
    public void ObjHasTheTreesTrianglesInTheWorld()
    {
        Tree aTree = SampleTree();
        var anObj = new MemoryStream();
        var aMtl = new MemoryStream();
        Assert.True(new ObjTreeWriter().Write(aTree, anObj, aMtl, "sample.mtl"));
        string aText = Encoding.UTF8.GetString(anObj.ToArray());
        Assert.DoesNotContain('\r', aText);
        Assert.Contains("mtllib sample.mtl\n", aText, StringComparison.Ordinal);
        Assert.Contains("o Asm/Box2\n", aText, StringComparison.Ordinal);
        (List<string> aTriangles, List<string> aMaterials, _) = ParseObj(aText);
        Assert.Equal(WorldTriangles(aTree), aTriangles);
        Assert.Equal(new[] { "mat_0", "mat_1", "mat_2", "default" }, aMaterials); // red box, green box, blue face, the plate's other faces
        string aMtlText = Encoding.UTF8.GetString(aMtl.ToArray());
        Assert.Contains("newmtl mat_0\nKd 1 0 0\nd 1\n", aMtlText, StringComparison.Ordinal);
        Assert.Contains("newmtl mat_2\nKd 0 0 1\nd 0.5\nTr 0.5\n", aMtlText, StringComparison.Ordinal);

        // Y up and metres on request
        var anObj2 = new MemoryStream();
        Assert.True(new ObjTreeWriter(new ObjWriteOptions { ZUpToYUp = true, Scale = 0.001 }).Write(aTree, anObj2, null));
        Assert.DoesNotContain("mtllib", Encoding.UTF8.GetString(anObj2.ToArray()), StringComparison.Ordinal);
        Assert.Equal(aTriangles.Count, ParseObj(Encoding.UTF8.GetString(anObj2.ToArray())).Triangles.Count);
    }

    // ---------------------------------------------------------------------------------------------- 3MF

    private static readonly XNamespace theCore = TreeWriter.CoreNamespace;

    /// <summary>
    /// A 3MF package's rules (3MF Core Specification 1.3, OPC): the three parts, their content types, the root
    /// relationship, the model's namespace and unit, objects defined before use, ids unique, triangle vertices in range
    /// and distinct, colours as #RRGGBB(AA); returns the triangles in the world (build items and components applied).
    /// </summary>
    internal static List<string> Validate3mf(byte[] thePackage, out XDocument theModel)
    {
        using var aZip = new ZipArchive(new MemoryStream(thePackage), ZipArchiveMode.Read);
        Assert.Equal(new[] { "[Content_Types].xml", "_rels/.rels", "3D/3dmodel.model" }, aZip.Entries.Select(e => e.FullName));
        XDocument load(string n)
        {
            using Stream s = aZip.GetEntry(n)!.Open();
            return XDocument.Load(s);
        }
        XNamespace aCt = "http://schemas.openxmlformats.org/package/2006/content-types";
        XDocument aTypes = load("[Content_Types].xml");
        var aDefaults = aTypes.Root!.Elements(aCt + "Default").ToDictionary(e => (string)e.Attribute("Extension")!, e => (string)e.Attribute("ContentType")!);
        Assert.Equal("application/vnd.openxmlformats-package.relationships+xml", aDefaults["rels"]);
        Assert.Equal("application/vnd.ms-package.3dmanufacturing-3dmodel+xml", aDefaults["model"]);
        XNamespace aRels = "http://schemas.openxmlformats.org/package/2006/relationships";
        XElement aRel = Assert.Single(load("_rels/.rels").Root!.Elements(aRels + "Relationship"));
        Assert.Equal("/3D/3dmodel.model", (string?)aRel.Attribute("Target"));
        Assert.Equal("http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel", (string?)aRel.Attribute("Type"));

        theModel = load("3D/3dmodel.model");
        XElement aModel = theModel.Root!;
        Assert.Equal(theCore + "model", aModel.Name);
        Assert.Contains((string?)aModel.Attribute("unit"), new[] { "micron", "millimeter", "centimeter", "inch", "foot", "meter" });
        XElement aResources = aModel.Element(theCore + "resources")!;
        var aDefined = new HashSet<int>();
        var aObjects = new Dictionary<int, XElement>();
        int aMaterialCount = 0;
        foreach (XElement aResource in aResources.Elements())
        {
            int anId = (int)aResource.Attribute("id")!;
            Assert.True(aDefined.Add(anId));
            if (aResource.Name == theCore + "basematerials")
            {
                foreach (XElement aBase in aResource.Elements(theCore + "base"))
                {
                    Assert.Matches(new Regex("^#[0-9A-F]{6}([0-9A-F]{2})?$"), (string)aBase.Attribute("displaycolor")!);
                    aMaterialCount++;
                }
                continue;
            }
            Assert.Equal(theCore + "object", aResource.Name);
            aObjects[anId] = aResource;
            if (aResource.Attribute("pindex") is XAttribute aPindex)
            {
                Assert.InRange((int)aPindex, 0, aMaterialCount - 1);
            }
            if (aResource.Element(theCore + "components") is XElement aComponents)
            {
                foreach (XElement aComponent in aComponents.Elements(theCore + "component"))
                {
                    Assert.Contains((int)aComponent.Attribute("objectid")!, aObjects.Keys); // defined before
                }
                continue;
            }
            XElement aMesh = aResource.Element(theCore + "mesh")!;
            int aNbVertices = aMesh.Element(theCore + "vertices")!.Elements().Count();
            Assert.True(aNbVertices >= 3);
            foreach (XElement aTriangle in aMesh.Element(theCore + "triangles")!.Elements())
            {
                int a = (int)aTriangle.Attribute("v1")!, b = (int)aTriangle.Attribute("v2")!, c = (int)aTriangle.Attribute("v3")!;
                Assert.True(a != b && b != c && a != c);
                Assert.InRange(Math.Max(a, Math.Max(b, c)), 0, aNbVertices - 1);
                if (aTriangle.Attribute("p1") is XAttribute aP1)
                {
                    Assert.InRange((int)aP1, 0, aMaterialCount - 1);
                }
            }
        }

        var aResult = new List<string>();
        void expand(int theId, double[] theMatrix)
        {
            XElement anObject = aObjects[theId];
            if (anObject.Element(theCore + "components") is XElement aComponents)
            {
                foreach (XElement aComponent in aComponents.Elements(theCore + "component"))
                {
                    expand((int)aComponent.Attribute("objectid")!, multiply(parse(aComponent), theMatrix));
                }
                return;
            }
            XElement aMesh = anObject.Element(theCore + "mesh")!;
            var aPoints = aMesh.Element(theCore + "vertices")!.Elements().Select(v =>
            {
                double x = (double)v.Attribute("x")!, y = (double)v.Attribute("y")!, z = (double)v.Attribute("z")!;
                return new Point3D(x * theMatrix[0] + y * theMatrix[3] + z * theMatrix[6] + theMatrix[9],
                                  x * theMatrix[1] + y * theMatrix[4] + z * theMatrix[7] + theMatrix[10],
                                  x * theMatrix[2] + y * theMatrix[5] + z * theMatrix[8] + theMatrix[11]);
            }).ToList();
            foreach (XElement aTriangle in aMesh.Element(theCore + "triangles")!.Elements())
            {
                aResult.Add(Canonical(aPoints[(int)aTriangle.Attribute("v1")!], aPoints[(int)aTriangle.Attribute("v2")!], aPoints[(int)aTriangle.Attribute("v3")!]));
            }
        }
        foreach (XElement anItem in aModel.Element(theCore + "build")!.Elements(theCore + "item"))
        {
            int anId = (int)anItem.Attribute("objectid")!;
            Assert.Contains(anId, aObjects.Keys);
            expand(anId, parse(anItem));
        }
        aResult.Sort(StringComparer.Ordinal);
        return aResult;
    }

    // 3MF's transform, applied to row vectors: x' = x m00 + y m10 + z m20 + m30
    private static double[] parse(XElement theElement)
        => theElement.Attribute("transform") is XAttribute aT
             ? [.. ((string)aT).Split(' ').Select(s => double.Parse(s, CultureInfo.InvariantCulture))]
             : [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0];

    // the child's transform then the parent's (row vectors: child × parent)
    private static double[] multiply(double[] theChild, double[] theParent)
    {
        double[] r = new double[12];
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                double aSum = i == 3 ? theParent[9 + j] : 0.0;
                for (int k = 0; k < 3; k++)
                {
                    aSum += theChild[3 * i + k] * theParent[3 * k + j];
                }
                r[3 * i + j] = aSum;
            }
        }
        return r;
    }

    [Fact]
    public void ThreeMfFollowsTheCoreSpecification()
    {
        Tree aTree = SampleTree();
        var aStream = new MemoryStream();
        Assert.True(new TreeWriter().Write(aTree, aStream));
        byte[] aPackage = aStream.ToArray();
        var aStream2 = new MemoryStream();
        Assert.True(new TreeWriter().Write(aTree, aStream2));
        Assert.Equal(aPackage, aStream2.ToArray()); // deterministic

        List<string> aTriangles = Validate3mf(aPackage, out XDocument aModel);
        Assert.Equal(WorldTriangles(aTree), aTriangles);
        XElement aRoot = aModel.Root!;
        Assert.Equal("millimeter", (string?)aRoot.Attribute("unit"));
        string[] aColors = [.. aRoot.Descendants(theCore + "base").Select(b => (string)b.Attribute("displaycolor")!)];
        Assert.Equal(new[] { "#FF0000", "#00FF00", "#FFFFFF", "#0000FF80" }, aColors);
        XElement anItem = Assert.Single(aRoot.Element(theCore + "build")!.Elements());
        Assert.Equal("1 0 0 0 1 0 0 0 1 1 2 3", (string?)anItem.Attribute("transform"));
        // objects: the red box, the green box, the plate, the assembly (last, using the others)
        XElement[] anObjects = [.. aRoot.Descendants(theCore + "object")];
        Assert.Equal(new[] { "Box", "Box", "Plate", "Asm" }, anObjects.Select(o => (string?)o.Attribute("name")));

        // winding: the box's triangles turn outwards (positive volume)
        XElement aBox = anObjects[0];
        var aPoints = aBox.Descendants(theCore + "vertex").Select(v => new Xyz((double)v.Attribute("x")!, (double)v.Attribute("y")!, (double)v.Attribute("z")!)).ToList();
        double aVolume = aBox.Descendants(theCore + "triangle").Sum(t =>
            aPoints[(int)t.Attribute("v1")!].Dot(aPoints[(int)t.Attribute("v2")!].Crossed(aPoints[(int)t.Attribute("v3")!])) / 6.0);
        Assert.Equal(6000.0, aVolume, 6);

        // another unit: centimetres stay centimetres; a unit 3MF lacks becomes millimetres
        var aCm = new MemoryStream();
        Assert.True(new TreeWriter().Write(aTree.WithLengthUnit(new DocumentLengthUnit("CM", 0.01)), aCm));
        Validate3mf(aCm.ToArray(), out XDocument aCmModel);
        Assert.Equal("centimeter", (string?)aCmModel.Root!.Attribute("unit"));
        var aOdd = new MemoryStream();
        Assert.True(new TreeWriter(new WriteOptions { InputLengthUnit = 0.002 }).Write(aTree, aOdd));
        Validate3mf(aOdd.ToArray(), out XDocument anOddModel);
        Assert.Equal("millimeter", (string?)anOddModel.Root!.Attribute("unit"));
        Assert.Equal("1 0 0 0 1 0 0 0 1 2 4 6", (string?)anOddModel.Root!.Element(theCore + "build")!.Element(theCore + "item")!.Attribute("transform"));
    }

    // ---------------------------------------------------------------------------------------------- OCCT's de_mesh cases

    // de_mesh/gltf_read/lantern: checknbshapes -face 3 -compound 1; checktrinfo -tri 5394 -nod 4145; the XDE dump
    private static void CheckLantern(Tree theTree)
    {
        Instance aRoot = Assert.Single(theTree.Roots);
        Shape aShape = aRoot.Shape;
        Assert.Equal(3, CountShapes(aShape, ShapeType.Face));
        Assert.Equal(1, CountShapes(aShape, ShapeType.Compound));
        Assert.Equal((5394, 4145), TriInfo(aShape));
        // INSTANCE COMPOUND "Lantern" -> ASSEMBLY COMPOUND "Lantern" -> three INSTANCE FACEs of PART FACEs
        Assert.Equal("Lantern", aRoot.DisplayName);
        Assert.True(aRoot.Product.IsAssembly);
        Assert.Equal(new[] { "LanternPole_Body", "LanternPole_Chain", "LanternPole_Lantern" }, aRoot.Product.Components.Select(c => c.DisplayName));
        Assert.All(aRoot.Product.Components, c => Assert.Equal(ShapeType.Face, c.Product.Shape.ShapeType()));
        Assert.True(aRoot.Shape.TShape()!.IsFrozen);
    }

    [Fact]
    public void LanternReadsAsOcctReadsIt()
    {
        string? aPath = DatasetFile("bug30691_Lantern.glb");
        if (aPath is null)
        {
            return; // the dataset isn't there
        }
        var aReader = new TreeReader();
        Tree? aTree = aReader.ReadFile(aPath);
        Assert.True(aTree is not null, aReader.Error);
        CheckLantern(aTree);

        // de_mesh/gltf_write/lanternglb: written to GLB and read back, the same
        byte[] aGlb = WriteGlb(aTree);
        (JsonDocument aJson, byte[] aBin) = SplitGlb(aGlb);
        ValidateGltf(aJson, aBin);
        Tree aBack = ReadGltf(aGlb);
        CheckLantern(aBack);
        Assert.Equal(WorldTriangles(aTree), WorldTriangles(aBack));

        // de_mesh/obj_write/lantern: written to OBJ, 5394 triangles on 4145 nodes
        var anObj = new MemoryStream();
        Assert.True(new ObjTreeWriter().Write(aTree, anObj, new MemoryStream()));
        (List<string> anObjTriangles, _, int aNbVertices) = ParseObj(Encoding.UTF8.GetString(anObj.ToArray()));
        Assert.Equal(5394, anObjTriangles.Count);
        Assert.Equal(4145, aNbVertices);
        Assert.Equal(WorldTriangles(aTree), anObjTriangles);

        // and to 3MF (degenerate triangles, which 3MF forbids, left out)
        var a3mf = new MemoryStream();
        Assert.True(new TreeWriter().Write(aTree, a3mf));
        List<string> a3mfTriangles = Validate3mf(a3mf.ToArray(), out _);
        List<string> aNonDegenerate = [.. WorldTriangles(aTree).Where(t => t.Split('|').Distinct().Count() == 3)];
        Assert.True(a3mfTriangles.Count <= 5394 && a3mfTriangles.Count >= aNonDegenerate.Count);
    }

    // The dataset's other glTF files (no Draw case uses them): read without error, written again and read back alike
    [Theory]
    [InlineData("bug32867_010.glb")]
    [InlineData("bug32867_Diamond.glb")]
    [InlineData("bug32867_bull.glb")]
    [InlineData("bug32867_test.glb")]
    public void DatasetGltfFilesRead(string theName)
    {
        string? aPath = DatasetFile(theName);
        if (aPath is null)
        {
            return;
        }
        var aReader = new TreeReader();
        Tree? aTree = aReader.ReadFile(aPath);
        Assert.True(aTree is not null, aReader.Error);
        int aNbTriangles = aTree.Flatten().Sum(i => TriInfo(i.Shape).Triangles);
        Assert.True(aNbTriangles > 0);
        if (theName == "bug32867_test.glb")
        {
            Assert.Contains(aReader.Warnings, w => w.Contains("KHR_materials_pbrSpecularGlossiness", StringComparison.Ordinal));
        }
        Tree aBack = ReadGltf(WriteGlb(aTree));
        Assert.Equal(aNbTriangles, aBack.Flatten().Sum(i => TriInfo(i.Shape).Triangles));
    }
}
