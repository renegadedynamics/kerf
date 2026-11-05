// Port of: src/ModelingData/TKBRep/BinTools/BinTools_OStream.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_OStream.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using System.Buffers.Binary;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// A binary output stream that knows its position, for the position-based format of <see cref="ShapeWriter"/>
/// (references to objects written earlier are offsets back to them). Little-endian.
/// </summary>
/// <remarks>
/// OCCT's <c>operator&lt;&lt;</c> overloads are C# shift operators, so ported code reads as the C++ does. The position
/// starts at the stream's position (0 for a stream that can't seek: references are relative, so the bytes are the
/// same).
/// </remarks>
internal sealed class OStream
{
    private readonly Stream _stream;

    // the current position, relative to the stream's position when this object was created
    private ulong _position;

    // OCCT's buffers for 12 reals, 3 integers and 3 floats
    private readonly byte[] _buf = new byte[sizeof(double) * 12];

    /// <summary>Writes to the stream, starting at its current position.</summary>
    public OStream(Stream stream)
    {
        _stream = stream;
        _position = stream.CanSeek ? (ulong)stream.Position : 0;
    }

    /// <summary>The current position in the stream.</summary>
    public ulong Position() => _position;

    /// <summary>The stream written to.</summary>
    public Stream Stream() => _stream;

    /// <summary>Writes a reference to the given position: the offset back to it from here.</summary>
    public void WriteReference(ulong position)
    {
        ulong delta = _position - position;
        if (delta <= 0xFF)
        {
            _stream.WriteByte((byte)ObjectType.ObjectType_Reference8);
            _stream.WriteByte((byte)delta);
            _position += sizeof(byte) * 2;
        }
        else if (delta <= 0xFFFF)
        {
            _stream.WriteByte((byte)ObjectType.ObjectType_Reference16);
            BinaryPrimitives.WriteUInt16LittleEndian(_buf, (ushort)delta);
            _stream.Write(_buf, 0, sizeof(ushort));
            _position += sizeof(byte) + sizeof(ushort);
        }
        else if (delta <= 0xFFFFFFFF)
        {
            _stream.WriteByte((byte)ObjectType.ObjectType_Reference32);
            BinaryPrimitives.WriteUInt32LittleEndian(_buf, (uint)delta);
            _stream.Write(_buf, 0, sizeof(uint));
            _position += sizeof(byte) + sizeof(uint);
        }
        else
        {
            _stream.WriteByte((byte)ObjectType.ObjectType_Reference64);
            BinaryPrimitives.WriteUInt64LittleEndian(_buf, delta);
            _stream.Write(_buf, 0, sizeof(ulong));
            _position += sizeof(byte) + sizeof(ulong);
        }
    }

    /// <summary>Writes the identifier of a shape's type and orientation.</summary>
    public void WriteShape(ShapeType type, Orientation orientation)
    {
        // taking into account that orientation <= 3 and type <= 8
        byte aType = (byte)((byte)ObjectType.ObjectType_EndShape + 1
                            + ((byte)type << 2)
                            + (byte)orientation);
        _stream.WriteByte(aType);
        _position += sizeof(byte);
    }

    /// <summary>Writes an object type.</summary>
    public static OStream operator <<(OStream os, ObjectType type)
    {
        os._stream.WriteByte((byte)type);
        os._position += sizeof(byte);
        return os;
    }

    /// <summary>Writes a byte.</summary>
    public static OStream operator <<(OStream os, byte value)
    {
        os._stream.WriteByte(value);
        os._position += sizeof(byte);
        return os;
    }

    /// <summary>Writes a real.</summary>
    public static OStream operator <<(OStream os, double value)
    {
        BinaryPrimitives.WriteDoubleLittleEndian(os._buf, value);
        os._stream.Write(os._buf, 0, sizeof(double));
        os._position += sizeof(double);
        return os;
    }

    /// <summary>Writes a boolean (one byte).</summary>
    public static OStream operator <<(OStream os, bool value)
    {
        os._stream.WriteByte((byte)(value ? 1 : 0));
        os._position += sizeof(byte);
        return os;
    }

    /// <summary>Writes an integer.</summary>
    public static OStream operator <<(OStream os, int value)
    {
        BinaryPrimitives.WriteInt32LittleEndian(os._buf, value);
        os._stream.Write(os._buf, 0, sizeof(int));
        os._position += sizeof(int);
        return os;
    }

    /// <summary>Writes an extended (UTF-16) character.</summary>
    public static OStream operator <<(OStream os, char value)
    {
        BinaryPrimitives.WriteUInt16LittleEndian(os._buf, value);
        os._stream.Write(os._buf, 0, sizeof(char));
        os._position += sizeof(char);
        return os;
    }

    /// <summary>Writes a 3D point.</summary>
    public static OStream operator <<(OStream os, in Point3D value)
    {
        os.PutReals(value.X, value.Y, value.Z);
        return os;
    }

    /// <summary>Writes a 3D direction.</summary>
    public static OStream operator <<(OStream os, in Direction3D value)
    {
        os.PutReals(value.X, value.Y, value.Z);
        return os;
    }

    /// <summary>Writes a 2D point.</summary>
    public static OStream operator <<(OStream os, in Point2D value)
    {
        BinaryPrimitives.WriteDoubleLittleEndian(os._buf, value.X);
        BinaryPrimitives.WriteDoubleLittleEndian(os._buf.AsSpan(8), value.Y);
        os._stream.Write(os._buf, 0, sizeof(double) * 2);
        os._position += sizeof(double) * 2;
        return os;
    }

    /// <summary>Writes a 2D direction.</summary>
    public static OStream operator <<(OStream os, in Direction2D value)
    {
        BinaryPrimitives.WriteDoubleLittleEndian(os._buf, value.X);
        BinaryPrimitives.WriteDoubleLittleEndian(os._buf.AsSpan(8), value.Y);
        os._stream.Write(os._buf, 0, sizeof(double) * 2);
        os._position += sizeof(double) * 2;
        return os;
    }

    /// <summary>Writes a transformation: three rows of the matrix, each followed by its translation component.</summary>
    public static OStream operator <<(OStream os, in Transform3D value)
    {
        Xyz tr = value.TranslationXyz();
        Matrix3x3 mat = value.VectorialPart;
        Span<byte> buf = os._buf;
        BinaryPrimitives.WriteDoubleLittleEndian(buf, mat[1, 1]);
        BinaryPrimitives.WriteDoubleLittleEndian(buf[8..], mat[1, 2]);
        BinaryPrimitives.WriteDoubleLittleEndian(buf[16..], mat[1, 3]);
        BinaryPrimitives.WriteDoubleLittleEndian(buf[24..], tr.Coord(1));
        BinaryPrimitives.WriteDoubleLittleEndian(buf[32..], mat[2, 1]);
        BinaryPrimitives.WriteDoubleLittleEndian(buf[40..], mat[2, 2]);
        BinaryPrimitives.WriteDoubleLittleEndian(buf[48..], mat[2, 3]);
        BinaryPrimitives.WriteDoubleLittleEndian(buf[56..], tr.Coord(2));
        BinaryPrimitives.WriteDoubleLittleEndian(buf[64..], mat[3, 1]);
        BinaryPrimitives.WriteDoubleLittleEndian(buf[72..], mat[3, 2]);
        BinaryPrimitives.WriteDoubleLittleEndian(buf[80..], mat[3, 3]);
        BinaryPrimitives.WriteDoubleLittleEndian(buf[88..], tr.Coord(3));
        os._stream.Write(os._buf, 0, sizeof(double) * 12);
        os._position += sizeof(double) * 12;
        return os;
    }

    /// <summary>Writes the three node indices of a triangle.</summary>
    public static OStream operator <<(OStream os, in Triangle1 value)
    {
        BinaryPrimitives.WriteInt32LittleEndian(os._buf, value.Value(1));
        BinaryPrimitives.WriteInt32LittleEndian(os._buf.AsSpan(4), value.Value(2));
        BinaryPrimitives.WriteInt32LittleEndian(os._buf.AsSpan(8), value.Value(3));
        os._stream.Write(os._buf, 0, sizeof(int) * 3);
        os._position += sizeof(int) * 3;
        return os;
    }

    /// <summary>Writes a vector of three floats.</summary>
    public static OStream operator <<(OStream os, in NCollection_Vec3<float> value)
    {
        BinaryPrimitives.WriteSingleLittleEndian(os._buf, value.X());
        BinaryPrimitives.WriteSingleLittleEndian(os._buf.AsSpan(4), value.Y());
        BinaryPrimitives.WriteSingleLittleEndian(os._buf.AsSpan(8), value.Z());
        os._stream.Write(os._buf, 0, sizeof(float) * 3);
        os._position += sizeof(float) * 3;
        return os;
    }

    /// <summary>Writes three booleans as one byte (bits 0 to 2).</summary>
    public void PutBools(bool value1, bool value2, bool value3)
    {
        byte value = (byte)((value1 ? 1 : 0) | (value2 ? 2 : 0) | (value3 ? 4 : 0));
        _stream.WriteByte(value);
        _position += sizeof(byte);
    }

    /// <summary>Writes seven booleans as one byte (bits 0 to 6).</summary>
    public void PutBools(bool value1,
                         bool value2,
                         bool value3,
                         bool value4,
                         bool value5,
                         bool value6,
                         bool value7)
    {
        byte value = (byte)((value1 ? 1 : 0) | (value2 ? 2 : 0) | (value3 ? 4 : 0)
                             | (value4 ? 8 : 0) | (value5 ? 16 : 0) | (value6 ? 32 : 0)
                             | (value7 ? 64 : 0));
        _stream.WriteByte(value);
        _position += sizeof(byte);
    }

    private void PutReals(double x, double y, double z)
    {
        BinaryPrimitives.WriteDoubleLittleEndian(_buf, x);
        BinaryPrimitives.WriteDoubleLittleEndian(_buf.AsSpan(8), y);
        BinaryPrimitives.WriteDoubleLittleEndian(_buf.AsSpan(16), z);
        _stream.Write(_buf, 0, sizeof(double) * 3);
        _position += sizeof(double) * 3;
    }
}
