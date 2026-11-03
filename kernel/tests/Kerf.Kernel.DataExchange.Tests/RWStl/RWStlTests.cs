// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Stl.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Buffers.Binary;
using System.Text;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// RWStl, RWStl_Reader and StlAPI (unit 8H1): OCCT's GTests for STL (DESTL_Provider_Test.cxx, through the calls
/// DESTL_Provider makes, since DESTL itself waits for TKDE's framework), the port's streams, limits for hostile
/// input, cancellation, frozen results and unchanged inputs. RWStlNativeTests compares the outputs with native OCCT.
/// </summary>
public class RWStlTests
{
    // DESTL_Provider_Test's CreateTriangulatedFace: a planar face with a two-triangle triangulation
    private static Shape CreateTriangulatedFace()
    {
        var aNodes = new NCollection_Array1<Point3D>(1, 4);
        aNodes.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aNodes.SetValue(2, new Point3D(10.0, 0.0, 0.0));
        aNodes.SetValue(3, new Point3D(10.0, 10.0, 0.0));
        aNodes.SetValue(4, new Point3D(0.0, 10.0, 0.0));
        var aTriangles = new NCollection_Array1<Triangle1>(1, 2);
        aTriangles.SetValue(1, new Triangle1(1, 2, 3));
        aTriangles.SetValue(2, new Triangle1(1, 3, 4));
        return FaceWith(new Triangulation(aNodes, aTriangles), 10.0);
    }

    private static Shape FaceWith(Triangulation theTriangulation, double theSize)
    {
        var aPlane = new Plane3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0, 0, 1));
        Face aFace = new MakeFace(aPlane, 0.0, theSize, 0.0, theSize).InternalFace();
        new BrepBuilder().UpdateFace(aFace, theTriangulation);
        aFace.TShape()!.FreezeGraph();
        return aFace;
    }

    private static int CountShapeElements(in Shape theShape, ShapeType theType)
    {
        int aCount = 0;
        for (var anExplorer = new Explorer(theShape, theType); anExplorer.More(); anExplorer.Next())
        {
            aCount++;
        }
        return aCount;
    }

    private static byte[] Write(in Shape theShape, bool theAscii)
    {
        var aWriter = new ApiWriter();
        aWriter.SetASCIIMode(theAscii);
        var aStream = new MemoryStream();
        Assert.True(aWriter.Write(theShape, aStream));
        return aStream.ToArray();
    }

    // DESTL_Provider::Read(stream, shape) with ReadBRep off: RWStl::ReadStream, a face holding the triangulation
    private static Shape ReadFace(byte[] theData)
    {
        Triangulation? aTriangulation = global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(new MemoryStream(theData));
        if (aTriangulation is null)
        {
            return new Shape();
        }
        Face aFace = new();
        var aB = new BrepBuilder();
        aB.MakeFace(ref aFace);
        aB.UpdateFace(aFace, aTriangulation);
        return aFace;
    }

    [Fact]
    public void StreamShapeWriteRead()
    {
        byte[] aStlContent = Write(CreateTriangulatedFace(), true);
        string aText = Encoding.ASCII.GetString(aStlContent);
        Assert.True(aText.Contains("solid", StringComparison.Ordinal) || aText.Contains("facet", StringComparison.Ordinal));
        Shape aReadShape = ReadFace(aStlContent);
        Assert.False(aReadShape.IsNull);
        Assert.True(CountShapeElements(aReadShape, ShapeType.Face) > 0);
    }

    [Fact]
    public void AsciiVsBinaryModes()
    {
        byte[] anAsciiContent = Write(CreateTriangulatedFace(), true);
        Assert.Contains("solid", Encoding.ASCII.GetString(anAsciiContent), StringComparison.Ordinal);
        byte[] aBinaryContent = Write(CreateTriangulatedFace(), false);
        Assert.NotEmpty(aBinaryContent);
        Assert.NotEqual(anAsciiContent, aBinaryContent);
        Assert.Equal(84 + 2 * 50, aBinaryContent.Length);
        Assert.StartsWith("STL Exported by Open CASCADE Technology [dev.opencascade.org]", Encoding.ASCII.GetString(aBinaryContent, 0, 80), StringComparison.Ordinal);
    }

    [Fact]
    public void TriangulatedFaceHandling()
    {
        byte[] aStlContent = Write(CreateTriangulatedFace(), true);
        Assert.True(aStlContent.Length > 100);
        Assert.False(ReadFace(aStlContent).IsNull);
        // with ReadBRep on: StlAPI_Reader, a face per triangle
        Shape aShape = new();
        Assert.True(new StlApiReader().Read(ref aShape, new MemoryStream(aStlContent)));
        Assert.Equal(2, CountShapeElements(aShape, ShapeType.Face));
    }

    [Fact]
    public void ErrorHandling()
    {
        // a null shape: nothing written
        Assert.False(new ApiWriter().Write(new Shape(), new MemoryStream()));
        // invalid content
        Assert.True(ReadFace(Encoding.ASCII.GetBytes("This is not valid STL content")).IsNull);
    }

    [Fact]
    public void StreamErrorConditions()
    {
        Assert.True(ReadFace(Encoding.ASCII.GetBytes("This is not STL data at all")).IsNull);
        // partial STL data
        Assert.True(ReadFace(Encoding.ASCII.GetBytes("solid test\n  facet normal 0 0 1\n")).IsNull);
    }

    [Fact]
    public void MultipleTriangulatedFaces()
    {
        var aNodes = new NCollection_Array1<Point3D>(1, 3);
        aNodes.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aNodes.SetValue(2, new Point3D(5.0, 0.0, 0.0));
        aNodes.SetValue(3, new Point3D(2.5, 5.0, 0.0));
        var aTriangles = new NCollection_Array1<Triangle1>(1, 1);
        aTriangles.SetValue(1, new Triangle1(1, 2, 3));
        Shape aTriangleFace = FaceWith(new Triangulation(aNodes, aTriangles), 5.0);

        byte[] aContent1 = Write(CreateTriangulatedFace(), true);
        byte[] aContent2 = Write(aTriangleFace, true);
        Assert.NotEqual(aContent1, aContent2);
        Assert.False(ReadFace(aContent1).IsNull);
        Assert.False(ReadFace(aContent2).IsNull);
    }

    [Fact]
    public void The_ASCII_writer_writes_OCCTs_text()
    {
        string aText = Encoding.ASCII.GetString(Write(CreateTriangulatedFace(), true));
        Assert.StartsWith("solid \n facet normal 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00\n   outer loop\n"
                          + "     vertex 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00\n"
                          + "     vertex 1.000000000000e+01 0.000000000000e+00 0.000000000000e+00\n", aText, StringComparison.Ordinal);
        Assert.EndsWith("   endloop\n endfacet\nendsolid\n", aText, StringComparison.Ordinal);
    }

    [Fact]
    public void Writers_leave_their_input_unchanged_and_readers_publish_frozen_results()
    {
        Shape aFace = CreateTriangulatedFace();
        Triangulation aTriangulation = Tool.Triangulation(ShapeCasts.Face(aFace), out _)!;
        string aBefore = Text(aFace);
        Write(aFace, true);
        Write(aFace, false);
        Assert.Equal(aBefore, Text(aFace));
        Assert.True(aTriangulation.IsFrozen);

        byte[] aData = Write(aFace, false);
        Triangulation? aRead = global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aData));
        Assert.NotNull(aRead);
        Assert.True(aRead.IsFrozen);
        Assert.Throws<ImmutableObjectException>(() => aRead.SetNode(1, new Point3D()));
        var aList = new NCollection_Sequence<Triangulation>();
        global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aData), Math.PI / 2, aList);
        Assert.True(aList.Value(1).IsFrozen);
        Shape aShape = new();
        Assert.True(new StlApiReader().Read(ref aShape, new MemoryStream(aData)));
        Assert.True(aShape.TShape()!.IsFrozen);
    }

    private static string Text(in Shape theShape)
    {
        var aWriter = new StringWriter { NewLine = "\n" };
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private sealed class NonSeekableStream(byte[] theData) : MemoryStream(theData)
    {
        public override bool CanSeek => false;

        public override long Position { get => base.Position; set => throw new NotSupportedException(); }

        public override long Seek(long offset, SeekOrigin loc) => throw new NotSupportedException();
    }

    [Fact]
    public void Streams_that_cant_seek_and_streams_read_from_their_position()
    {
        byte[] aData = Write(CreateTriangulatedFace(), false);
        Triangulation? aRead = global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new NonSeekableStream(aData));
        Assert.NotNull(aRead);
        Assert.Equal(4, aRead.NodeCount);
        Assert.Equal(2, aRead.TriangleCount);
        Assert.NotNull(global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(new NonSeekableStream(Write(CreateTriangulatedFace(), true))));

        // data after a prefix, read from the stream's position: binary recognised by its content
        var aPrefixed = new MemoryStream([.. "prefix"u8, .. aData]) { Position = 6 };
        Triangulation? aFromPosition = global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(aPrefixed);
        Assert.NotNull(aFromPosition);
        Assert.Equal(2, aFromPosition.TriangleCount);
    }

    [Fact]
    public void Limits_stop_hostile_input()
    {
        byte[] aBinary = Write(CreateTriangulatedFace(), false);
        byte[] anAscii = Write(CreateTriangulatedFace(), true);
        var aSmall = new global::Kerf.Engine.Exchange.Stl.StlReadLimits { MaxBytes = 100 };
        var aOneTriangle = new global::Kerf.Engine.Exchange.Stl.StlReadLimits { MaxTriangles = 1 };
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aBinary), Math.PI / 2, default, aSmall));
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new NonSeekableStream(aBinary), Math.PI / 2, default, aSmall));
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(new MemoryStream(anAscii), Math.PI / 2, default, aSmall));
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aBinary), Math.PI / 2, default, aOneTriangle));
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(anAscii), Math.PI / 2, default, aOneTriangle));
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadBinaryStream(new MemoryStream(aBinary), Math.PI / 2, default, aOneTriangle));
        Assert.NotNull(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aBinary), Math.PI / 2, default, new global::Kerf.Engine.Exchange.Stl.StlReadLimits { MaxBytes = aBinary.Length, MaxTriangles = 2 }));
        Shape aShape = new();
        Assert.False(new StlApiReader().Read(ref aShape, new MemoryStream(aBinary), default, aOneTriangle));

        var aReader = new CountingReader();
        aReader.SetLimits(aOneTriangle);
        Assert.False(aReader.Read(new MemoryStream(anAscii)));
        Assert.True(aReader.LimitExceeded());
        Assert.Equal(1, aReader.myNbTris);
    }

    [Fact]
    public void A_binary_header_claiming_too_many_facets_fails_without_allocating_for_them()
    {
        // OCCT sizes its node map for the header's count (and runs out of memory): the port for the facets the data
        // can hold. The first block (80 facets) can't be read: nothing is
        byte[] aData = Write(CreateTriangulatedFace(), false);
        BinaryPrimitives.WriteInt32LittleEndian(aData.AsSpan(80), int.MaxValue);
        var aReader = new CountingReader();
        Assert.False(aReader.Read(new MemoryStream(aData)));
        Assert.Equal(0, aReader.myNbTris);
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aData)));
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadBinaryStream(new MemoryStream(aData)));
        // a count of 3 for 2 facets: the block of 3 can't be read either
        BinaryPrimitives.WriteInt32LittleEndian(aData.AsSpan(80), 3);
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadBinaryStream(new MemoryStream(aData)));
        BinaryPrimitives.WriteInt32LittleEndian(aData.AsSpan(80), int.MinValue);
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadBinaryStream(new MemoryStream(aData)));
    }

    private sealed class CountingReader : global::Kerf.Engine.Exchange.Stl.Reader
    {
        public int myNbNodes, myNbTris, myNbSolids;

        public override int AddNode(in Xyz thePnt) => ++myNbNodes;

        public override void AddTriangle(int theN1, int theN2, int theN3) => ++myNbTris;

        public override void AddSolid() => ++myNbSolids;
    }

    private static Triangulation Grid(int theN)
    {
        var aMesh = new Triangulation((theN + 1) * (theN + 1), 2 * theN * theN, false);
        for (int i = 0; i <= theN; ++i)
        {
            for (int j = 0; j <= theN; ++j)
            {
                aMesh.SetNode(1 + i * (theN + 1) + j, new Point3D(i, j, 0));
            }
        }
        int k = 0;
        for (int i = 0; i < theN; ++i)
        {
            for (int j = 0; j < theN; ++j)
            {
                int a = 1 + i * (theN + 1) + j, b = a + theN + 1;
                aMesh.SetTriangle(++k, new Triangle1(a, b, b + 1));
                aMesh.SetTriangle(++k, new Triangle1(a, b + 1, a + 1));
            }
        }
        return aMesh;
    }

    [Fact]
    public void Cancellation_stops_reading_and_writing()
    {
        Triangulation aMesh = Grid(30); // 1,800 triangles: the writers check every 1,000
        var aBinary = new MemoryStream();
        Assert.True(global::Kerf.Engine.Exchange.Stl.RWStl.WriteBinary(aMesh, aBinary));
        var anAscii = new MemoryStream();
        Assert.True(global::Kerf.Engine.Exchange.Stl.RWStl.WriteAscii(aMesh, anAscii));
        // merged nodes: the grid's
        Triangulation? aRead = global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aBinary.ToArray()));
        Assert.Equal(31 * 31, aRead!.NodeCount);

        using var aSource = new CancellationTokenSource();
        aSource.Cancel();
        ProgressRange Cancelled() => new ProgressIndicator(aSource.Token).Start();
        Assert.False(global::Kerf.Engine.Exchange.Stl.RWStl.WriteBinary(aMesh, new MemoryStream(), Cancelled()));
        Assert.False(global::Kerf.Engine.Exchange.Stl.RWStl.WriteAscii(aMesh, new MemoryStream(), Cancelled()));
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aBinary.ToArray()), Math.PI / 2, Cancelled()));
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(anAscii.ToArray()), Math.PI / 2, Cancelled()));
        Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(new MemoryStream(anAscii.ToArray()), Math.PI / 2, Cancelled()));
        var aList = new NCollection_Sequence<Triangulation>();
        global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aBinary.ToArray()), Math.PI / 2, aList, Cancelled());
        Assert.Equal(0, aList.Length());
    }

    [Fact]
    public void Files_are_written_and_read_by_path()
    {
        string aDir = Directory.CreateTempSubdirectory("p8h1-rwstl").FullName;
        try
        {
            Triangulation aMesh = Grid(3);
            string anAscii = Path.Combine(aDir, "a.stl"), aBinary = Path.Combine(aDir, "b.stl");
            Assert.True(global::Kerf.Engine.Exchange.Stl.RWStl.WriteAscii(aMesh, anAscii));
            Assert.True(global::Kerf.Engine.Exchange.Stl.RWStl.WriteBinary(aMesh, aBinary));
            // the file form writes the stream form's bytes (LF line ends)
            var aStream = new MemoryStream();
            global::Kerf.Engine.Exchange.Stl.RWStl.WriteAscii(aMesh, aStream);
            Assert.Equal(aStream.ToArray(), File.ReadAllBytes(anAscii));
            Assert.Equal(18, global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(anAscii)!.TriangleCount);
            Assert.Equal(18, global::Kerf.Engine.Exchange.Stl.RWStl.ReadAscii(anAscii)!.TriangleCount);
            Assert.Equal(18, global::Kerf.Engine.Exchange.Stl.RWStl.ReadBinary(aBinary)!.TriangleCount);
            Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadAscii(aBinary));
            Assert.Null(global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(Path.Combine(aDir, "none.stl")));
            Assert.False(global::Kerf.Engine.Exchange.Stl.RWStl.WriteAscii(aMesh, Path.Combine(aDir, "no", "such", "dir.stl")));
            Assert.False(global::Kerf.Engine.Exchange.Stl.RWStl.WriteAscii(null, Path.Combine(aDir, "c.stl")));
            Assert.False(global::Kerf.Engine.Exchange.Stl.RWStl.WriteBinary(new Triangulation(), new MemoryStream()));

            Shape aFace = FaceWith(aMesh, 3.0);
            Assert.True(StlApi.Write(aFace, Path.Combine(aDir, "s.stl"), false));
            Shape aShape = new();
#pragma warning disable CS0618 // the deprecated legacy reader
            Assert.True(StlApi.Read(ref aShape, Path.Combine(aDir, "s.stl")));
#pragma warning restore CS0618
            Assert.Equal(18, CountShapeElements(aShape, ShapeType.Face));
        }
        finally
        {
            Directory.Delete(aDir, true);
        }
    }

    [Fact]
    public void Reversed_and_located_faces_are_written_as_placed()
    {
        Shape aFace = CreateTriangulatedFace();
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithTranslation(new Vector3D(0, 0, 5));
        Shape aMoved = aFace.Moved(new Placement(aTrsf)).Reversed();
        Triangulation? aRead = global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(Write(aMoved, false)));
        Assert.NotNull(aRead);
        Assert.Equal(5.0, aRead.Node(1).Z());
        // reversed: the triangles turn over (nodes 2 and 3 swapped)
        Triangulation? aStraight = global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(Write(aFace, false)));
        aStraight!.Triangle(1).Get(out _, out int s2, out int s3);
        aRead.Triangle(1).Get(out _, out int r2, out int r3);
        Assert.Equal(aStraight.Node(s2).X(), aRead.Node(r3).X());
        Assert.Equal(aStraight.Node(s3).X(), aRead.Node(r2).X());
    }
}
