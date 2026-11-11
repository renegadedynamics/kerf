using System.Buffers.Binary;
using System.IO.Compression;
using System.Text;
using System.Text.Json;
using System.Xml.Linq;
using Kerf.Geometry;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Exchange;

using Kerf.Modeling;

/// <summary>
/// A closed solid's 3MF mesh is closed: one vertex per mesh node, faces meeting at an edge sharing its nodes, every edge
/// used once in each direction (decision 2191). The report: a plate with a pocket cut by a moved and turned shape
/// came out open along the pocket's edges, its faces' copies of a node differing in the last digits.
/// </summary>
public class ClosedMeshTests
{
    private static readonly Location Turned = new Pos(7.3, -4.1, 4) * new Rotation(0, 0, 30);

    private static Part Plate(string pocket, Location at)
    {
        Part plate = new Pos(0, 0, 2) * new Box(40, 40, 4);
        Part cut = pocket switch
        {
            "box" => at * new Box(10, 6, 1.2),
            "cylinder" => at * new Cylinder(3, 1.2),
            _ => Extrude((Sketch)(at * (Sketch)MakeText("58S", 9, "Liberation Sans", fontStyle: Kerf.Fonts.FontStyle.Bold).Value), -0.6).Value,
        };
        return plate - cut;
    }

    // The vertices and triangles of a 3MF's one mesh object
    private static (List<(double X, double Y, double Z)> Vertices, List<int[]> Triangles) Read3mf(byte[] data)
    {
        using var zip = new ZipArchive(new MemoryStream(data));
        var doc = XDocument.Load(zip.GetEntry("3D/3dmodel.model")!.Open());
        var vertices = doc.Descendants().Where(e => e.Name.LocalName == "vertex")
                          .Select(v => ((double)v.Attribute("x")!, (double)v.Attribute("y")!, (double)v.Attribute("z")!)).ToList();
        var triangles = doc.Descendants().Where(e => e.Name.LocalName == "triangle")
                           .Select(t => new[] { (int)t.Attribute("v1")!, (int)t.Attribute("v2")!, (int)t.Attribute("v3")! }).ToList();
        return (vertices, triangles);
    }

    // The edges not used exactly once in each direction, the vertices taken by their indices
    private static int OpenEdges(IEnumerable<int[]> triangles)
    {
        var edges = new Dictionary<(int, int), int>();
        foreach (int[] t in triangles)
        {
            for (int k = 0; k < 3; k++)
            {
                var e = (t[k], t[(k + 1) % 3]);
                edges[e] = edges.GetValueOrDefault(e) + 1;
            }
        }
        return edges.Count(kv => kv.Value != 1 || edges.GetValueOrDefault((kv.Key.Item2, kv.Key.Item1)) != 1);
    }

    private static double Volume(IReadOnlyList<(double X, double Y, double Z)> v, IEnumerable<int[]> triangles) =>
        triangles.Sum(t =>
        {
            var (a, b, c) = (v[t[0]], v[t[1]], v[t[2]]);
            return (a.X * (b.Y * c.Z - b.Z * c.Y) - a.Y * (b.X * c.Z - b.Z * c.X) + a.Z * (b.X * c.Y - b.Y * c.X)) / 6;
        });

    // The triangles and volume of the same mesh written as binary STL, whose writer is unchanged: the triangles with
    // three corners apart (3MF leaves out those of no area at a degenerate edge, whose corners are one node)
    private static (int Triangles, double Volume) Stl(IShape shape)
    {
        var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportStl(shape, stream, new StlExportOptions { Ascii = false }).IsSuccess);
        byte[] data = stream.ToArray();
        int count = BinaryPrimitives.ReadInt32LittleEndian(data.AsSpan(80));
        var vertices = new List<(double, double, double)>();
        for (int i = 0; i < count; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                int at = 84 + 50 * i + 12 + 12 * k;
                vertices.Add((BinaryPrimitives.ReadSingleLittleEndian(data.AsSpan(at)), BinaryPrimitives.ReadSingleLittleEndian(data.AsSpan(at + 4)),
                              BinaryPrimitives.ReadSingleLittleEndian(data.AsSpan(at + 8))));
            }
        }
        var triangles = Enumerable.Range(0, count).Select(i => new[] { 3 * i, 3 * i + 1, 3 * i + 2 })
                                  .Where(t => vertices[t[0]] != vertices[t[1]] && vertices[t[1]] != vertices[t[2]] && vertices[t[0]] != vertices[t[2]]).ToList();
        return (triangles.Count, Volume(vertices, triangles));
    }

    private static void AssertClosed3mf(IShape shape, int euler = 2)
    {
        var stream = new MemoryStream();
        Assert.True(Exporters3D.Export3mf(shape, stream).IsSuccess);
        var (vertices, triangles) = Read3mf(stream.ToArray());
        Assert.Equal(0, OpenEdges(triangles));
        Assert.Equal(vertices.Count, vertices.Distinct().Count());              // one vertex per node
        Assert.Equal(euler, vertices.Count - 3 * triangles.Count / 2 + triangles.Count); // V - E + F: 2 a sphere's, 0 a torus's
        // The same mesh but for its vertices' sharing: the triangles and volume of the unchanged STL writer's
        var (stlTriangles, stlVolume) = Stl(shape);
        Assert.Equal(stlTriangles, triangles.Count);
        Assert.Equal(stlVolume, Volume(vertices, triangles), stlVolume * 1e-6);
        Assert.True(Volume(vertices, triangles) > 0); // facing out
    }

    [Theory]
    [InlineData("box", false)]
    [InlineData("box", true)]
    [InlineData("cylinder", false)]
    [InlineData("cylinder", true)]
    [InlineData("text", false)]
    [InlineData("text", true)]
    public void The_reports_plates_are_closed_in_3mf(string pocket, bool turned)
    {
        Part plate = Plate(pocket, turned ? Turned : new Pos(0, 0, 4));
        Assert.True(plate.IsValid);
        AssertClosed3mf(plate);
    }

    [Fact]
    public void A_part_of_several_solids_is_closed_each()
    {
        Part two = new Box(10, 10, 10) + Turned * new Pos(20, 0, 0) * new Cylinder(3, 8);
        Assert.Equal(2, two.Solids().Count);
        AssertClosed3mf(two, euler: 4);
    }

    [Fact]
    public void Seams_and_degenerate_edges_are_closed()
    {
        AssertClosed3mf(Turned * new Cylinder(5, 12));              // a seam
        AssertClosed3mf(Turned * new Cone(5, 0, 12));               // a seam and an apex
        AssertClosed3mf(Turned * new Sphere(6));                    // a seam and two poles
        AssertClosed3mf(Turned * new Torus(10, 3), euler: 0);       // two seams
    }

    // ---- STL ----

    [Theory]
    [InlineData("box")]
    [InlineData("cylinder")]
    [InlineData("text")]
    public void The_reports_plates_are_closed_in_stl_by_their_positions(string pocket)
    {
        // STL has no indices: a reader joins identical vertices, so a node's copies must round to the same floats
        Part plate = Plate(pocket, Turned);
        var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportStl(plate, stream, new StlExportOptions { Ascii = false }).IsSuccess);
        byte[] data = stream.ToArray();
        int count = BinaryPrimitives.ReadInt32LittleEndian(data.AsSpan(80));
        var ids = new Dictionary<(float, float, float), int>();
        int Id(int at) => ids.TryGetValue(Corner(at), out int id) ? id : ids[Corner(at)] = ids.Count;
        (float, float, float) Corner(int at) => (BinaryPrimitives.ReadSingleLittleEndian(data.AsSpan(at)), BinaryPrimitives.ReadSingleLittleEndian(data.AsSpan(at + 4)),
                                                 BinaryPrimitives.ReadSingleLittleEndian(data.AsSpan(at + 8)));
        var triangles = Enumerable.Range(0, count).Select(i => new[] { Id(84 + 50 * i + 12), Id(84 + 50 * i + 24), Id(84 + 50 * i + 36) }).ToList();
        Assert.Equal(0, OpenEdges(triangles));
    }

    // ---- glTF ----

    // Each primitive's positions and triangles from a .glb
    private static List<(List<(float, float, float)> Positions, List<int[]> Triangles)> ReadGlb(byte[] glb)
    {
        int jsonLength = BinaryPrimitives.ReadInt32LittleEndian(glb.AsSpan(12));
        using var json = JsonDocument.Parse(Encoding.UTF8.GetString(glb, 20, jsonLength));
        int binStart = 20 + jsonLength + 8;
        JsonElement root = json.RootElement;
        JsonElement accessors = root.GetProperty("accessors"), views = root.GetProperty("bufferViews");
        int Offset(JsonElement accessor)
        {
            JsonElement view = views[accessor.GetProperty("bufferView").GetInt32()];
            return binStart + (view.TryGetProperty("byteOffset", out JsonElement o) ? o.GetInt32() : 0)
                            + (accessor.TryGetProperty("byteOffset", out JsonElement a) ? a.GetInt32() : 0);
        }
        var result = new List<(List<(float, float, float)>, List<int[]>)>();
        foreach (JsonElement mesh in root.GetProperty("meshes").EnumerateArray())
        {
            foreach (JsonElement primitive in mesh.GetProperty("primitives").EnumerateArray())
            {
                JsonElement position = accessors[primitive.GetProperty("attributes").GetProperty("POSITION").GetInt32()];
                int at = Offset(position);
                var positions = Enumerable.Range(0, position.GetProperty("count").GetInt32())
                                          .Select(i => (BinaryPrimitives.ReadSingleLittleEndian(glb.AsSpan(at + 12 * i)),
                                                        BinaryPrimitives.ReadSingleLittleEndian(glb.AsSpan(at + 12 * i + 4)),
                                                        BinaryPrimitives.ReadSingleLittleEndian(glb.AsSpan(at + 12 * i + 8)))).ToList();
                JsonElement indices = accessors[primitive.GetProperty("indices").GetInt32()];
                int start = Offset(indices);
                bool isShort = indices.GetProperty("componentType").GetInt32() == 5123;
                int Index(int i) => isShort ? BinaryPrimitives.ReadUInt16LittleEndian(glb.AsSpan(start + 2 * i))
                                            : (int)BinaryPrimitives.ReadUInt32LittleEndian(glb.AsSpan(start + 4 * i));
                var triangles = Enumerable.Range(0, indices.GetProperty("count").GetInt32() / 3)
                                          .Select(t => new[] { Index(3 * t), Index(3 * t + 1), Index(3 * t + 2) }).ToList();
                result.Add((positions, triangles));
            }
        }
        return result;
    }

    [Theory]
    [InlineData("box")]
    [InlineData("cylinder")]
    [InlineData("text")]
    public void The_reports_plates_are_closed_in_gltf_by_their_positions(string pocket)
    {
        // glTF's vertices carry normals, so a crease keeps a vertex per face; but a node's copies are at one position
        Part plate = Plate(pocket, Turned);
        var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportGltf(plate, stream, new GltfExportOptions { Binary = true }).IsSuccess);
        var primitives = ReadGlb(stream.ToArray());
        var ids = new Dictionary<(float, float, float), int>();
        var triangles = primitives.SelectMany(p => p.Triangles.Select(t => t.Select(i => ids.TryGetValue(p.Positions[i], out int id) ? id : ids[p.Positions[i]] = ids.Count).ToArray()))
                                  .ToList();
        Assert.Equal(0, OpenEdges(triangles));
        Assert.Equal(Stl(plate).Triangles, triangles.Count);
    }
}
