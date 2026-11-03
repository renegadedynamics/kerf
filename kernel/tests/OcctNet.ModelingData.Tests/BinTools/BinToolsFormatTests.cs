// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using System.Buffers.Binary;
using System.Text;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// The bytes of OCCT's binary BRep formats, checked by hand against BinTools' sources (there is no native oracle for
/// BinTools, open-questions.md item 4): little-endian primitives, the text headers of the table-based format, the
/// records of vertices, curves and locations, and the position-based format's type bytes and references.
/// </summary>
public class BinToolsFormatTests
{
    private static byte[] Bytes(Action<Stream> theWrite)
    {
        var aStream = new MemoryStream();
        theWrite(aStream);
        return aStream.ToArray();
    }

    private sealed class Expected
    {
        private readonly MemoryStream myStream = new();

        public Expected Text(string theText)
        {
            myStream.Write(Encoding.ASCII.GetBytes(theText));
            return this;
        }

        public Expected Byte(int theValue)
        {
            myStream.WriteByte((byte)theValue);
            return this;
        }

        public Expected Int(int theValue)
        {
            Span<byte> aBuf = stackalloc byte[4];
            BinaryPrimitives.WriteInt32LittleEndian(aBuf, theValue);
            myStream.Write(aBuf);
            return this;
        }

        public Expected Real(double theValue)
        {
            Span<byte> aBuf = stackalloc byte[8];
            BinaryPrimitives.WriteDoubleLittleEndian(aBuf, theValue);
            myStream.Write(aBuf);
            return this;
        }

        public byte[] ToArray() => myStream.ToArray();
    }

    [Fact]
    public void Primitives_are_little_endian()
    {
        Assert.Equal(new byte[] { 4, 3, 2, 1 }, Bytes(s => BinTools.PutInteger(s, 0x01020304)));
        Assert.Equal(new byte[] { 0, 0, 0, 0, 0, 0, 0xF0, 0x3F }, Bytes(s => BinTools.PutReal(s, 1.0)));
        Assert.Equal(new byte[] { 0, 0, 0x80, 0x3F }, Bytes(s => BinTools.PutShortReal(s, 1.0f)));
        Assert.Equal(new byte[] { 0x34, 0x12 }, Bytes(s => BinTools.PutExtChar(s, 'ሴ')));
        Assert.Equal(new byte[] { 1, 0 }, Bytes(s => { BinTools.PutBool(s, true); BinTools.PutBool(s, false); }));

        var aStream = new MemoryStream([4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0xF0, 0x3F, 0, 0, 0x80, 0x3F, 0x34, 0x12, 7]);
        BinTools.GetInteger(aStream, out int anInt);
        BinTools.GetReal(aStream, out double aReal);
        BinTools.GetShortReal(aStream, out float aFloat);
        BinTools.GetExtChar(aStream, out char aChar);
        BinTools.GetBool(aStream, out bool aBool);
        Assert.Equal(0x01020304, anInt);
        Assert.Equal(1.0, aReal);
        Assert.Equal(1.0f, aFloat);
        Assert.Equal('ሴ', aChar);
        Assert.True(aBool);

        // the end of the stream (OCCT: Storage_StreamTypeMismatchError, or garbage from a byte read)
        Assert.Throws<global::Kerf.Engine.Foundation.StreamFormatErrorException>(() => BinTools.GetInteger(aStream, out _));
        Assert.Throws<global::Kerf.Engine.Foundation.StreamFormatErrorException>(() => BinTools.GetBool(aStream, out _));
    }

    [Fact]
    public void A_vertex_in_version_4()
    {
        TopoDS_Vertex aVertex = TestShapes.Vertex(new Point3D(1, 2, 3));
        byte[] anExpected = new Expected()
            .Text("\nOpen CASCADE Topology V4, (c) Open Cascade\n")
            .Text("Locations 0\n")
            .Text("Curve2ds 0\n")
            .Text("Curves 0\n")
            .Text("Polygon3D 0\n")
            .Text("PolygonOnTriangulations 0\n")
            .Text("Surfaces 0\n")
            .Text("Triangulations 0\n")
            .Text("\nTShapes 1\n")
            .Byte(7) // TopAbs_VERTEX
            .Real(global::Kerf.Engine.Foundation.Precision.Confusion()).Real(1).Real(2).Real(3)
            .Byte(0) // no point representations
            .Byte(1).Byte(1).Byte(0).Byte(1).Byte(1).Byte(0).Byte(1) // Free, Modified, Checked, Orientable, Closed, Infinite, Convex
            .Byte('*') // no sub-shapes
            .Byte(0).Int(1).Int(0) // the shape: FORWARD, TShape 1 (counted from the last), no location
            .ToArray();
        Assert.Equal(anExpected, BinToolsRoundTripTests.WriteSet(aVertex, 4));

        // the other versions differ only by their header
        byte[] aV1 = BinToolsRoundTripTests.WriteSet(aVertex, 1);
        Assert.StartsWith("\nOpen CASCADE Topology V1 (c)\nLocations 0\n", Encoding.ASCII.GetString(aV1), StringComparison.Ordinal);
        Assert.Equal(anExpected.Length - "V4, (c) Open Cascade".Length + "V1 (c)".Length, aV1.Length);
    }

    [Fact]
    public void An_edge_on_a_line_in_version_2()
    {
        // an edge with a 3D curve and its two vertices, each with no point representation
        var aLine = new Geom_Line(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        TopoDS_Edge anEdge = TestShapes.Edge(aLine, 0, 2);
        byte[] aBytes = BinToolsRoundTripTests.WriteSet(anEdge, 2);
        string aText = Encoding.ASCII.GetString(aBytes);
        Assert.StartsWith("\nOpen CASCADE Topology V2 (c)\nLocations 0\nCurve2ds 0\nCurves 1\n", aText, StringComparison.Ordinal);

        // the line: its type, then location and direction
        int aCurve = aText.IndexOf("Curves 1\n", StringComparison.Ordinal) + "Curves 1\n".Length;
        byte[] aLineBytes = new Expected().Byte(1).Real(0).Real(0).Real(0).Real(1).Real(0).Real(0).ToArray();
        Assert.Equal(aLineBytes, aBytes.AsSpan(aCurve, aLineBytes.Length).ToArray());
        Assert.Equal("Polygon3D 0\n", aText.Substring(aCurve + aLineBytes.Length, 12));

        // the edge record, the last before the shape reference (9 bytes): type, tolerance, SameParameter, SameRange,
        // Degenerated, a 3D curve (1, curve, location, range), the end, the flags, its two vertices (each: orientation,
        // index from the end, location) and the end
        byte[] anEdgeBytes = new Expected()
            .Byte(6) // TopAbs_EDGE
            .Real(global::Kerf.Engine.Foundation.Precision.Confusion()).Byte(1).Byte(1).Byte(0)
            .Byte(1).Int(1).Int(0).Real(0).Real(2)
            .Byte(0)
            .Byte(1).Byte(1).Byte(0).Byte(1).Byte(0).Byte(0).Byte(0)
            .Byte(0).Int(3).Int(0) // FORWARD vertex: index 1 of 3
            .Byte(1).Int(2).Int(0) // REVERSED vertex: index 2 of 3
            .Byte('*')
            .ToArray();
        Assert.Equal(anEdgeBytes, aBytes.AsSpan(aBytes.Length - 9 - anEdgeBytes.Length, anEdgeBytes.Length).ToArray());
        Assert.Equal(new Expected().Byte(0).Int(1).Int(0).ToArray(), aBytes.AsSpan(aBytes.Length - 9).ToArray());
    }

    [Fact]
    public void Locations_elementary_and_complex()
    {
        var aT = new Transform3D();
        aT.SetTranslation(new Vector3D(1, 2, 3));
        var aR = new Transform3D();
        aR.SetScale(new Point3D(), 2.0);
        var aL1 = new Placement(aT);
        var aL2 = new Placement(aR);
        var aSet = new BinTools_LocationSet();
        aSet.Add(aL1 * aL2.Powered(3));

        byte[] anExpected = new Expected()
            .Text("Locations 3\n")
            // the datums, the product's first item (its right-hand factor) first
            .Byte(1).Real(2).Real(0).Real(0).Real(-0.0).Real(0).Real(2).Real(0).Real(-0.0).Real(0).Real(0).Real(2).Real(-0.0) // (1 - 2) * 0
            .Byte(1).Real(1).Real(0).Real(0).Real(1).Real(0).Real(1).Real(0).Real(2).Real(0).Real(0).Real(1).Real(3)
            // the product: (index, power) pairs, then 0
            .Byte(2).Int(1).Int(3).Int(2).Int(1).Int(0)
            .ToArray();
        Assert.Equal(anExpected, Bytes(s => aSet.Write(s)));
    }

    [Fact]
    public void The_position_based_format()
    {
        TopoDS_Vertex aVertex = TestShapes.Vertex(new Point3D(1, 2, 3));
        var aStream = new MemoryStream();
        var aWriter = new BinTools_ShapeWriter();
        aWriter.Write(aVertex, aStream);
        aWriter.Write(aVertex.Reversed(), aStream);
        byte[] anExpected = new Expected()
            .Byte(200 + 4 * 7 + 0) // a FORWARD vertex
            .Byte(7) // no location
            .Real(global::Kerf.Engine.Foundation.Precision.Confusion()).Real(1).Real(2).Real(3)
            .Byte(0) // no point representations
            .Byte(1 | 2 | 8 | 16 | 64) // Free, Modified, Orientable, Closed, Convex
            .Byte(199) // the end of the shape
            .Byte(1).Byte(37) // the same TShape again: an 8-bit reference 37 bytes back
            .Byte(7).Byte(1) // no location, REVERSED
            .ToArray();
        Assert.Equal(anExpected, aStream.ToArray());

        // a null shape, then a location: written, then a new location with the same transformation (another datum, so
        // written again), then that one by reference
        var aT = new Transform3D();
        aT.SetTranslation(new Vector3D(0, 0, 1));
        var aLocations = new MemoryStream();
        var aWriter2 = new BinTools_ShapeWriter();
        aWriter2.Write(new TopoDS_Shape(), aLocations);
        var anOStream = new BinTools_OStream(aLocations);
        aWriter2.WriteLocation(anOStream, new Placement(aT));
        var aLoc = new Placement(aT);
        aWriter2.WriteLocation(anOStream, aLoc);
        aWriter2.WriteLocation(anOStream, aLoc);
        aWriter2.WriteLocation(anOStream, new Placement());
        byte[] anExpected2 = new Expected()
            .Byte(198)
            .Byte(6).Real(1).Real(0).Real(0).Real(0).Real(0).Real(1).Real(0).Real(0).Real(0).Real(0).Real(1).Real(1)
            .Byte(6).Real(1).Real(0).Real(0).Real(0).Real(0).Real(1).Real(0).Real(0).Real(0).Real(0).Real(1).Real(1)
            .Byte(1).Byte(97)
            .Byte(7)
            .ToArray();
        Assert.Equal(anExpected2, aLocations.ToArray());

        // references of 16, 32 and 64 bits
        var aRefs = new MemoryStream();
        var aRefStream = new BinTools_OStream(aRefs);
        for (int i = 0; i < 70000; i++)
        {
            aRefStream = aRefStream << (byte)0;
        }

        aRefStream.WriteReference(69000);
        aRefStream.WriteReference(0);
        byte[] aRefBytes = aRefs.ToArray();
        Assert.Equal(new byte[] { 2, 0xE8, 0x03 }, aRefBytes.AsSpan(70000, 3).ToArray());
        Assert.Equal(new byte[] { 3, 0x73, 0x11, 0x01, 0x00 }, aRefBytes.AsSpan(70003, 5).ToArray());
        var anIStream = new BinTools_IStream(new MemoryStream(aRefBytes) { Position = 70000 });
        anIStream.ReadType();
        Assert.True(anIStream.IsReference());
        Assert.Equal(69000UL, anIStream.ReadReference());
        anIStream.ReadType();
        Assert.Equal(0UL, anIStream.ReadReference());
        Assert.Equal(70008UL, anIStream.Position());
    }

    [Fact]
    public void Shape_type_bytes()
    {
        var aStream = new MemoryStream();
        var anOStream = new BinTools_OStream(aStream);
        anOStream.WriteShape(TopAbs_ShapeEnum.TopAbs_COMPOUND, TopAbs_Orientation.TopAbs_EXTERNAL);
        anOStream.WriteShape(TopAbs_ShapeEnum.TopAbs_FACE, TopAbs_Orientation.TopAbs_REVERSED);
        anOStream.PutBools(true, false, true);
        Assert.Equal(new byte[] { 203, 217, 5 }, aStream.ToArray());
        Assert.Equal(3UL, anOStream.Position());

        var anIStream = new BinTools_IStream(new MemoryStream(aStream.ToArray()));
        anIStream.ReadType();
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_COMPOUND, anIStream.ShapeType());
        Assert.Equal(TopAbs_Orientation.TopAbs_EXTERNAL, anIStream.ShapeOrientation());
        anIStream.ReadType();
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_FACE, anIStream.ShapeType());
        Assert.Equal(TopAbs_Orientation.TopAbs_REVERSED, anIStream.ShapeOrientation());
        anIStream.ReadBools(out bool a1, out bool a2, out bool a3);
        Assert.True(a1);
        Assert.False(a2);
        Assert.True(a3);
    }
}
