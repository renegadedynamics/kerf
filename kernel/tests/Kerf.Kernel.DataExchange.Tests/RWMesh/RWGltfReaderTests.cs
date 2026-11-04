// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Gltf;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using System.Buffers.Binary;
using System.Text;
using System.Text.Json.Nodes;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// RWGltf_TreeReader on hand-made glTF (unit 8T): the conversions, the primitive modes, node transformations (rigid ones
/// as locations, others applied to the meshes), and hostile input: each limit, each index and range check, the
/// container's checks, refused extensions and data, external buffers.
/// </summary>
public class RWGltfReaderTests
{
    // A buffer of float positions then unsigned short indices (padded), as a data URI
    private static string DataUri(float[] thePositions, ushort[] theIndices, out int theIndexOffset, out int theLength)
    {
        theIndexOffset = thePositions.Length * 4;
        theLength = theIndexOffset + ((theIndices.Length * 2 + 3) & ~3);
        byte[] aData = new byte[theLength];
        for (int i = 0; i < thePositions.Length; i++)
        {
            BinaryPrimitives.WriteSingleLittleEndian(aData.AsSpan(4 * i), thePositions[i]);
        }
        for (int i = 0; i < theIndices.Length; i++)
        {
            BinaryPrimitives.WriteUInt16LittleEndian(aData.AsSpan(theIndexOffset + 2 * i), theIndices[i]);
        }
        return "data:application/octet-stream;base64," + Convert.ToBase64String(aData);
    }

    /// <summary>One node, one mesh, one primitive of the positions and indices (a single triangle by default).</summary>
    private static JsonObject Model(float[]? thePositions = null, ushort[]? theIndices = null, int theMode = 4)
    {
        float[] aPositions = thePositions ?? [0, 0, 0, 1, 0, 0, 0, 1, 0];
        ushort[] anIndices = theIndices ?? [0, 1, 2];
        string anUri = DataUri(aPositions, anIndices, out int anIndexOffset, out int aLength);
        return new JsonObject
        {
            ["asset"] = new JsonObject { ["version"] = "2.0" },
            ["scene"] = 0,
            ["scenes"] = new JsonArray(new JsonObject { ["nodes"] = new JsonArray(0) }),
            ["nodes"] = new JsonArray(new JsonObject { ["mesh"] = 0, ["name"] = "N" }),
            ["meshes"] = new JsonArray(new JsonObject
            {
                ["name"] = "M",
                ["primitives"] = new JsonArray(new JsonObject
                {
                    ["attributes"] = new JsonObject { ["POSITION"] = 0 },
                    ["indices"] = 1,
                    ["mode"] = theMode,
                }),
            }),
            ["accessors"] = new JsonArray(
                new JsonObject { ["bufferView"] = 0, ["componentType"] = 5126, ["count"] = aPositions.Length / 3, ["type"] = "VEC3" },
                new JsonObject { ["bufferView"] = 1, ["componentType"] = 5123, ["count"] = anIndices.Length, ["type"] = "SCALAR" }),
            ["bufferViews"] = new JsonArray(
                new JsonObject { ["buffer"] = 0, ["byteOffset"] = 0, ["byteLength"] = aPositions.Length * 4 },
                new JsonObject { ["buffer"] = 0, ["byteOffset"] = anIndexOffset, ["byteLength"] = anIndices.Length * 2 }),
            ["buffers"] = new JsonArray(new JsonObject { ["byteLength"] = aLength, ["uri"] = anUri }),
        };
    }

    private static Tree? Read(JsonNode theJson, out TreeReader theReader, ReadOptions? theOptions = null,
                                     Func<string, Stream?>? theResolver = null)
    {
        theReader = new TreeReader(theOptions);
        return theReader.Read(new MemoryStream(Encoding.UTF8.GetBytes(theJson.ToJsonString())), theResolver);
    }

    private static void AssertError(JsonNode theJson, string theError, ReadOptions? theOptions = null)
    {
        Tree? aTree = Read(theJson, out TreeReader aReader, theOptions);
        Assert.Null(aTree);
        Assert.NotNull(aReader.Error);
        Assert.Contains(theError, aReader.Error, StringComparison.Ordinal);
    }

    private static Triangulation SingleTriangulation(Tree theTree, out FlatItem theItem)
    {
        theItem = Assert.Single(theTree.Flatten());
        Face aFace = ShapeCasts.Face(theItem.Product.Shape);
        return Tool.Triangulation(aFace, out _)!;
    }

    [Fact]
    public void ReadsATriangleInMillimetresZUp()
    {
        Tree? aTree = Read(Model(), out TreeReader aReader);
        Assert.True(aTree is not null, aReader.Error);
        Triangulation aTri = SingleTriangulation(aTree, out FlatItem anItem);
        Assert.Equal(1, aTri.TriangleCount);
        Assert.True(aTri.IsFrozen);
        // glTF's (1, 0, 0) m is (1000, 0, 0) mm; its (0, 1, 0), Y up, is (0, 0, 1000)
        Assert.Equal(1000.0, aTri.Node(2).X, 9);
        Assert.Equal(1000.0, aTri.Node(3).Z, 9);
        Assert.Equal(0.0, aTri.Node(3).Y, 9);
        Assert.Equal("M", anItem.Product.Name);
        Assert.Equal("N", anItem.Path[0].Name);
        Assert.Empty(aReader.Warnings);

        // without the conversions
        aTree = Read(Model(), out _, new ReadOptions { YUpToZUp = false, OutputLengthUnit = 1.0 });
        aTri = SingleTriangulation(aTree!, out _);
        Assert.Equal(1.0, aTri.Node(3).Y, 9);
    }

    [Fact]
    public void ReadsStripsAndFansAndSkipsLines()
    {
        float[] aSquare = [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 2, 2, 0];
        Assert.Equal(3, SingleTriangulation(Read(Model(aSquare, [0, 1, 2, 3, 4], 5), out _)!, out _).TriangleCount);
        Assert.Equal(3, SingleTriangulation(Read(Model(aSquare, [0, 1, 2, 3, 4], 6), out _)!, out _).TriangleCount);

        Tree? aTree = Read(Model(null, [0, 1, 1, 2], 1), out TreeReader aReader);
        Assert.NotNull(aTree);
        Assert.Contains(aReader.Warnings, w => w.Contains("mode 1", StringComparison.Ordinal));
        Assert.Empty(aTree.Flatten()); // an empty node: an empty assembly
    }

    [Fact]
    public void RigidNodesBecomeLocationsOthersAreApplied()
    {
        // a quarter turn about glTF's Y: a location; the mesh is shared
        JsonObject aRotated = Model();
        double aS = Math.Sqrt(0.5);
        aRotated["nodes"]![0]!["rotation"] = new JsonArray(0.0, aS, 0.0, aS);
        aRotated["nodes"]!.AsArray().Add(new JsonObject { ["mesh"] = 0, ["translation"] = new JsonArray(0.0, 0.0, 2.0) });
        aRotated["scenes"]![0]!["nodes"]!.AsArray().Add(1);
        Tree? aTree = Read(aRotated, out TreeReader aReader);
        Assert.True(aTree is not null, aReader.Error);
        Assert.Equal(2, aTree.Roots.Length);
        Assert.Same(aTree.Roots[0].Product, aTree.Roots[1].Product);
        Assert.False(aTree.Roots[0].Location.IsIdentity);
        // glTF's Z (2 m) is OCCT's -Y
        Assert.Equal(-2000.0, aTree.Roots[1].Location.Transformation.TranslationXyz().Y(), 9);

        // a scale: applied to the mesh (a copy for the node)
        JsonObject aScaled = Model();
        aScaled["nodes"]![0]!["scale"] = new JsonArray(2.0, 1.0, 1.0);
        Triangulation aTri = SingleTriangulation(Read(aScaled, out _)!, out FlatItem anItem);
        Assert.True(anItem.Location.IsIdentity);
        Assert.Equal(2000.0, aTri.Node(2).X, 9);

        // a mirror: applied, the winding swapped so the triangle keeps facing the same way
        JsonObject aMirrored = Model();
        aMirrored["nodes"]![0]!["scale"] = new JsonArray(-1.0, 1.0, 1.0);
        aTri = SingleTriangulation(Read(aMirrored, out _)!, out _);
        aTri.Triangle(1).Get(out int a, out int b, out int c);
        Xyz aNormal = (aTri.Node(b).XYZ() - aTri.Node(a).XYZ()).Crossed(aTri.Node(c).XYZ() - aTri.Node(a).XYZ());
        Assert.True(aNormal.Y() < 0); // glTF's +Z normal is OCCT's -Y, unchanged by a mirror in X
    }

    [Fact]
    public void RefusesBadIndicesAndRanges()
    {
        AssertError(Model(null, [0, 1, 5]), "index 5 out of the 3 vertices");

        JsonObject aLong = Model();
        aLong["accessors"]![0]!["count"] = 4;
        AssertError(aLong, "runs past its buffer view");

        JsonObject aView = Model();
        aView["bufferViews"]![1]!["byteLength"] = 1000;
        aView["accessors"]![1]!["byteOffset"] = 0;
        AssertError(aView, "its buffer view runs past its buffer");

        JsonObject aStride = Model();
        aStride["bufferViews"]![0]!["byteStride"] = 4;
        AssertError(aStride, "invalid byte stride");

        JsonObject aMissing = Model();
        aMissing["meshes"]![0]!["primitives"]![0]!["indices"] = 9;
        AssertError(aMissing, "Accessor 9 doesn't exist");

        JsonObject aNegative = Model();
        aNegative["nodes"]![0]!["mesh"] = -1;
        AssertError(aNegative, "isn't a non-negative integer");

        JsonObject aShort = Model();
        aShort["accessors"]![0]!["componentType"] = 5123;
        AssertError(aShort, "not 32-bit floats");

        JsonObject aFloatIndices = Model();
        aFloatIndices["accessors"]![1]!["componentType"] = 5126;
        aFloatIndices["accessors"]![1]!["count"] = 1;
        AssertError(aFloatIndices, "indices of component type 5126");

        JsonObject aSparse = Model();
        aSparse["accessors"]![0]!["sparse"] = new JsonObject();
        AssertError(aSparse, "sparse accessors");

        JsonObject aBase64 = Model();
        aBase64["buffers"]![0]!["uri"] = "data:application/octet-stream;base64,@@@@";
        AssertError(aBase64, "invalid base64");

        JsonObject aSmall = Model();
        aSmall["buffers"]![0]!["byteLength"] = 1000;
        AssertError(aSmall, "bytes, 1000 expected");

        AssertError(new JsonArray(), "isn't an object");
        AssertError(new JsonObject { ["asset"] = new JsonObject { ["version"] = "1.0" } }, "not 2.x");
    }

    [Fact]
    public void RefusesBadHierarchies()
    {
        JsonObject aTwoParents = Model();
        aTwoParents["nodes"]!.AsArray().Add(new JsonObject { ["children"] = new JsonArray(0) });
        aTwoParents["nodes"]!.AsArray().Add(new JsonObject { ["children"] = new JsonArray(0) });
        AssertError(aTwoParents, "more than one parent");

        JsonObject aNotRoot = Model();
        aNotRoot["nodes"]!.AsArray().Add(new JsonObject { ["children"] = new JsonArray(0) });
        AssertError(aNotRoot, "isn't a root node");

        JsonObject aSelf = Model();
        aSelf["nodes"]![0]!["children"] = new JsonArray(0);
        AssertError(aSelf, "its own child");

        // a cycle without scene: no root reaches it
        JsonObject aCycle = Model();
        aCycle.Remove("scenes");
        aCycle.Remove("scene");
        aCycle["nodes"] = new JsonArray(new JsonObject { ["children"] = new JsonArray(1) }, new JsonObject { ["children"] = new JsonArray(0) });
        Tree? aTree = Read(aCycle, out _);
        Assert.NotNull(aTree);
        Assert.Empty(aTree.Roots);

        // a chain deeper than the limit
        JsonObject aDeep = Model();
        var aNodes = new JsonArray();
        for (int i = 0; i < 300; i++)
        {
            aNodes.Add(i < 299 ? new JsonObject { ["children"] = new JsonArray(i + 1) } : new JsonObject { ["mesh"] = 0 });
        }
        aDeep["nodes"] = aNodes;
        AssertError(aDeep, "deeper than the limit (256)");
        Tree? aDeepTree = Read(aDeep, out TreeReader aReader, new ReadOptions { Limits = GltfReadLimits.Default with { MaxDepth = 1000 } });
        Assert.True(aDeepTree is not null, aReader.Error);
        Assert.Single(aDeepTree.Flatten());
        AssertError(aDeep, "300 nodes, more than the limit (100)", new ReadOptions { Limits = GltfReadLimits.Default with { MaxNodes = 100 } });
    }

    [Fact]
    public void LimitsAndExtensions()
    {
        AssertError(Model(), "larger than the limit (100 bytes)", new ReadOptions { Limits = GltfReadLimits.Default with { MaxBytes = 100 } });
        AssertError(Model(), "Buffer 0: larger than the limit (10 bytes)", new ReadOptions { Limits = GltfReadLimits.Default with { MaxBufferBytes = 10 } });
        AssertError(Model(), "More triangles than the limit (0)", new ReadOptions { Limits = GltfReadLimits.Default with { MaxTriangles = 0 } });

        // JSON nested deeper than allowed
        string aNested = Model().ToJsonString().Insert(1, "\"extras\":" + new string('[', 100) + new string(']', 100) + ",");
        var aNestedReader = new TreeReader();
        Assert.Null(aNestedReader.Read(new MemoryStream(Encoding.UTF8.GetBytes(aNested))));
        Assert.StartsWith("Invalid JSON", aNestedReader.Error, StringComparison.Ordinal);

        JsonObject aDraco = Model();
        aDraco["extensionsRequired"] = new JsonArray("KHR_draco_mesh_compression");
        AssertError(aDraco, "Required extension not supported: KHR_draco_mesh_compression");

        JsonObject anUnlit = Model();
        anUnlit["extensionsRequired"] = new JsonArray("KHR_materials_unlit");
        Assert.NotNull(Read(anUnlit, out TreeReader aReader));
        Assert.Single(aReader.Warnings);
    }

    [Fact]
    public void ExternalBuffersNeedTheCaller()
    {
        JsonObject anExternal = Model();
        string aDataUri = (string)anExternal["buffers"]![0]!["uri"]!;
        byte[] aData = Convert.FromBase64String(aDataUri[(aDataUri.IndexOf(',') + 1)..]);
        anExternal["buffers"]![0]!["uri"] = "../secret%20data.bin";
        AssertError(anExternal, "'../secret%20data.bin' can't be opened");
        string? aRequested = null;
        Assert.NotNull(Read(anExternal, out _, null, u => { aRequested = u; return new MemoryStream(aData); }));
        Assert.Equal("../secret data.bin", aRequested);

        // ReadFile: only files in the glTF's folder or below
        string aFolder = Path.Combine(Path.GetTempPath(), "occtnet-p8t-" + Guid.NewGuid().ToString("N"));
        string aSub = Path.Combine(aFolder, "sub");
        Directory.CreateDirectory(aSub);
        try
        {
            File.WriteAllBytes(Path.Combine(aFolder, "secret data.bin"), aData);
            File.WriteAllText(Path.Combine(aSub, "m.gltf"), anExternal.ToJsonString());
            var aReader = new TreeReader();
            Assert.Null(aReader.ReadFile(Path.Combine(aSub, "m.gltf")));
            Assert.Contains("can't be opened", aReader.Error, StringComparison.Ordinal);

            anExternal["buffers"]![0]!["uri"] = "secret%20data.bin";
            File.WriteAllText(Path.Combine(aFolder, "m.gltf"), anExternal.ToJsonString());
            Assert.NotNull(aReader.ReadFile(Path.Combine(aFolder, "m.gltf")));

            anExternal["buffers"]![0]!["uri"] = Path.Combine(aFolder, "secret data.bin");
            File.WriteAllText(Path.Combine(aSub, "m.gltf"), anExternal.ToJsonString());
            Assert.Null(aReader.ReadFile(Path.Combine(aSub, "m.gltf"))); // absolute paths refused
        }
        finally
        {
            Directory.Delete(aFolder, true);
        }
    }

    [Fact]
    public void RefusesBrokenGlb()
    {
        var aStream = new MemoryStream();
        Assert.True(new GltfTreeWriter().WriteGlb(MeshTestKit.SampleTree(), aStream));
        byte[] aGlb = aStream.ToArray();
        Assert.NotNull(new TreeReader().Read(new MemoryStream(aGlb)));

        void assertGlbError(byte[] theData, string theError)
        {
            var aReader = new TreeReader();
            Assert.Null(aReader.Read(new MemoryStream(theData)));
            Assert.Contains(theError, aReader.Error, StringComparison.Ordinal);
        }
        assertGlbError(aGlb[..16], "truncated header");
        assertGlbError(aGlb[..(aGlb.Length - 8)], "doesn't fit the data");
        byte[] aVersion = (byte[])aGlb.Clone();
        aVersion[4] = 1;
        assertGlbError(aVersion, "version 1");
        byte[] aChunk = (byte[])aGlb.Clone();
        BinaryPrimitives.WriteUInt32LittleEndian(aChunk.AsSpan(12), 0x7FFFFFF0);
        assertGlbError(aChunk, "runs past the end");
        byte[] aType = (byte[])aGlb.Clone();
        aType[16] = (byte)'X';
        assertGlbError(aType, "first chunk isn't JSON");
        // the binary chunk cut short: the accessors find their ranges missing
        int aJsonLength = (int)BinaryPrimitives.ReadUInt32LittleEndian(aGlb.AsSpan(12));
        byte[] aShort = aGlb[..(20 + aJsonLength + 8 + 16)];
        BinaryPrimitives.WriteUInt32LittleEndian(aShort.AsSpan(8), (uint)aShort.Length);
        BinaryPrimitives.WriteUInt32LittleEndian(aShort.AsSpan(20 + aJsonLength), 16);
        assertGlbError(aShort, "expected");
    }
}
