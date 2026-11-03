// Port of: src/ModelingData/TKBRep/BinTools/BinTools_IStream.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_IStream.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Buffers.Binary;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// A binary input stream that knows its position, for the position-based format of <see cref="ShapeReader"/>,
/// which follows references by seeking. Little-endian.
/// </summary>
/// <remarks>
/// The stream must be seekable (OCCT's <c>tellg</c> and <c>seekg</c>). OCCT's <c>operator&gt;&gt;</c> overloads are
/// the <c>Read*</c> methods. Reading past the end of the stream throws <see cref="StreamFormatErrorException"/> (OCCT
/// throws Storage_StreamTypeMismatchError for reals, integers and points, and leaves bytes and floats unread).
/// </remarks>
public sealed class IStream
{
    private readonly Stream _stream;

    // equivalent to tellg returned value for fast access
    private ulong _position;

    // last type that was read
    private ObjectType _lastType;

    private readonly byte[] _buf = new byte[sizeof(double) * 3];

    /// <summary>Reads from the stream, starting at its current position.</summary>
    /// <exception cref="ProgramErrorException">The stream can't seek.</exception>
    public IStream(Stream stream)
    {
        if (!stream.CanSeek)
        {
            throw new ProgramErrorException("BinTools_IStream: the stream must be seekable");
        }

        _stream = stream;
        _position = (ulong)stream.Position;
        _lastType = ObjectType.ObjectType_Unknown;
    }

    /// <summary>Reads and returns the type.</summary>
    public ObjectType ReadType()
    {
        _lastType = (ObjectType)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(_stream);
        _position++;
        return _lastType;
    }

    /// <summary>The last type read.</summary>
    public ObjectType LastType() => _lastType;

    /// <summary>The shape type of the last type read.</summary>
    public ShapeType ShapeType()
    {
        return (ShapeType)(((byte)_lastType - (byte)ObjectType.ObjectType_EndShape - 1) >> 2);
    }

    /// <summary>The shape orientation of the last type read.</summary>
    public Orientation ShapeOrientation()
    {
        return (Orientation)(((byte)_lastType - (byte)ObjectType.ObjectType_EndShape - 1) & 3);
    }

    /// <summary>The current position in the stream.</summary>
    public ulong Position() => _position;

    /// <summary>Moves to the given position.</summary>
    public void GoTo(ulong position)
    {
        if (position > (ulong)_stream.Length)
        {
            // OCCT.NET: untrusted input: OCCT seeks past the end and reads garbage
            throw new StreamFormatErrorException("BinTools_IStream: a position past the end of the stream");
        }

        _stream.Seek((long)position, SeekOrigin.Begin);
        _position = position;
    }

    /// <summary>True if the last type read is a reference.</summary>
    public bool IsReference
    {
        get
        {
            return _lastType == ObjectType.ObjectType_Reference8
                || _lastType == ObjectType.ObjectType_Reference16
                || _lastType == ObjectType.ObjectType_Reference32
                || _lastType == ObjectType.ObjectType_Reference64;
        }
    }

    /// <summary>Reads a reference of the last type read, and returns the position it refers to.</summary>
    public ulong ReadReference()
    {
        ulong delta = 0;
        ulong currentPos = (ulong)_stream.Position;
        switch (_lastType)
        {
            case ObjectType.ObjectType_Reference8:
                delta = (ulong)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(_stream);
                _position++;
                break;
            case ObjectType.ObjectType_Reference16:
            {
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadExactly(_stream, _buf.AsSpan(0, sizeof(ushort)));
                _position += 2;
                delta = BinaryPrimitives.ReadUInt16LittleEndian(_buf);
                break;
            }
            case ObjectType.ObjectType_Reference32:
            {
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadExactly(_stream, _buf.AsSpan(0, sizeof(uint)));
                _position += 4;
                delta = BinaryPrimitives.ReadUInt32LittleEndian(_buf);
                break;
            }
            case ObjectType.ObjectType_Reference64:
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadExactly(_stream, _buf.AsSpan(0, sizeof(ulong)));
                _position += 8;
                delta = BinaryPrimitives.ReadUInt64LittleEndian(_buf);
                break;
            default:
                break;
        }

        if (delta == 0)
        {
            throw new FailureException("BinTools_IStream::ReadReference: invalid reference " + (char)_lastType + "\n");
        }

        if (delta >= currentPos)
        {
            // OCCT.NET: untrusted input: the reference would point before the start of the stream
            throw new StreamFormatErrorException("BinTools_IStream::ReadReference: a reference before the stream");
        }

        return currentPos - delta - 1; // add a type-byte
    }

    /// <summary>The stream read from.</summary>
    public Stream Stream() => _stream;

    /// <summary>Brings the position up to date after the stream was read directly.</summary>
    public void UpdatePosition() => _position = (ulong)_stream.Position;

    /// <summary>
    /// False if reading failed (OCCT's <c>operator bool</c>). The port throws instead of failing, so it is always true.
    /// </summary>
    public static implicit operator bool(IStream stream) => true;

    /// <summary>Reads a real.</summary>
    public double ReadReal()
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadExactly(_stream, _buf.AsSpan(0, sizeof(double)));
        _position += sizeof(double);
        return BinaryPrimitives.ReadDoubleLittleEndian(_buf);
    }

    /// <summary>Reads an integer.</summary>
    public int ReadInteger()
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadExactly(_stream, _buf.AsSpan(0, sizeof(int)));
        _position += sizeof(int);
        return BinaryPrimitives.ReadInt32LittleEndian(_buf);
    }

    /// <summary>Reads a 3D point.</summary>
    public Point3D ReadPnt()
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadExactly(_stream, _buf.AsSpan(0, sizeof(double) * 3));
        Point3D value = new(BinaryPrimitives.ReadDoubleLittleEndian(_buf),
                            BinaryPrimitives.ReadDoubleLittleEndian(_buf.AsSpan(8)),
                            BinaryPrimitives.ReadDoubleLittleEndian(_buf.AsSpan(16)));
        _position += 3 * sizeof(double);
        return value;
    }

    /// <summary>Reads a byte.</summary>
    public byte ReadByte()
    {
        byte value = (byte)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(_stream);
        _position += sizeof(byte);
        return value;
    }

    /// <summary>Reads a boolean (one byte, non-zero for true).</summary>
    public bool ReadBool() => ReadByte() != 0;

    /// <summary>Reads a short real.</summary>
    public float ReadShortReal()
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadExactly(_stream, _buf.AsSpan(0, sizeof(float)));
        _position += sizeof(float);
        return BinaryPrimitives.ReadSingleLittleEndian(_buf);
    }

    /// <summary>Reads a transformation written by <see cref="OStream"/>.</summary>
    public Transform3D ReadTrsf()
    {
        double v10 = ReadReal(), v11 = ReadReal(), v12 = ReadReal(), v0 = ReadReal();
        double v20 = ReadReal(), v21 = ReadReal(), v22 = ReadReal(), v1 = ReadReal();
        double v30 = ReadReal(), v31 = ReadReal(), v32 = ReadReal(), v2 = ReadReal();
        Transform3D theValue = new();
        theValue.SetValues(v10, v11, v12, v0, v20, v21, v22, v1, v30, v31, v32, v2);
        return theValue;
    }

    /// <summary>Reads three booleans from one byte (bits 0 to 2).</summary>
    public void ReadBools(out bool bool1, out bool bool2, out bool bool3)
    {
        byte aByte = ReadByte();
        bool1 = (aByte & 1) == 1;
        bool2 = (aByte & 2) == 2;
        bool3 = (aByte & 4) == 4;
    }

    /// <summary>Reads seven booleans from one byte (bits 0 to 6).</summary>
    public void ReadBools(out bool bool1,
                          out bool bool2,
                          out bool bool3,
                          out bool bool4,
                          out bool bool5,
                          out bool bool6,
                          out bool bool7)
    {
        byte aByte = ReadByte();
        bool1 = (aByte & 1) == 1;
        bool2 = (aByte & 2) == 2;
        bool3 = (aByte & 4) == 4;
        bool4 = (aByte & 8) == 8;
        bool5 = (aByte & 16) == 16;
        bool6 = (aByte & 32) == 32;
        bool7 = (aByte & 64) == 64;
    }
}
