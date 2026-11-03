// Port of: src/ModelingData/TKBRep/BinTools/BinTools_IStream.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_IStream.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using System.Buffers.Binary;

namespace Occt;

/// <summary>
/// A binary input stream that knows its position, for the position-based format of <see cref="BinTools_ShapeReader"/>,
/// which follows references by seeking. Little-endian.
/// </summary>
/// <remarks>
/// The stream must be seekable (OCCT's <c>tellg</c> and <c>seekg</c>). OCCT's <c>operator&gt;&gt;</c> overloads are
/// the <c>Read*</c> methods. Reading past the end of the stream throws <see cref="StreamFormatErrorException"/> (OCCT
/// throws Storage_StreamTypeMismatchError for reals, integers and points, and leaves bytes and floats unread).
/// </remarks>
public sealed class BinTools_IStream
{
    private readonly Stream myStream;

    // equivalent to tellg returned value for fast access
    private ulong myPosition;

    // last type that was read
    private BinTools_ObjectType myLastType;

    private readonly byte[] myBuf = new byte[sizeof(double) * 3];

    /// <summary>Reads from the stream, starting at its current position.</summary>
    /// <exception cref="ProgramErrorException">The stream can't seek.</exception>
    public BinTools_IStream(Stream theStream)
    {
        if (!theStream.CanSeek)
        {
            throw new ProgramErrorException("BinTools_IStream: the stream must be seekable");
        }

        myStream = theStream;
        myPosition = (ulong)theStream.Position;
        myLastType = BinTools_ObjectType.BinTools_ObjectType_Unknown;
    }

    /// <summary>Reads and returns the type.</summary>
    public BinTools_ObjectType ReadType()
    {
        myLastType = (BinTools_ObjectType)BinTools.Get(myStream);
        myPosition++;
        return myLastType;
    }

    /// <summary>The last type read.</summary>
    public BinTools_ObjectType LastType() => myLastType;

    /// <summary>The shape type of the last type read.</summary>
    public TopAbs_ShapeEnum ShapeType()
    {
        return (TopAbs_ShapeEnum)(((byte)myLastType - (byte)BinTools_ObjectType.BinTools_ObjectType_EndShape - 1) >> 2);
    }

    /// <summary>The shape orientation of the last type read.</summary>
    public TopAbs_Orientation ShapeOrientation()
    {
        return (TopAbs_Orientation)(((byte)myLastType - (byte)BinTools_ObjectType.BinTools_ObjectType_EndShape - 1) & 3);
    }

    /// <summary>The current position in the stream.</summary>
    public ulong Position() => myPosition;

    /// <summary>Moves to the given position.</summary>
    public void GoTo(ulong thePosition)
    {
        if (thePosition > (ulong)myStream.Length)
        {
            // OCCT.NET: untrusted input: OCCT seeks past the end and reads garbage
            throw new StreamFormatErrorException("BinTools_IStream: a position past the end of the stream");
        }

        myStream.Seek((long)thePosition, SeekOrigin.Begin);
        myPosition = thePosition;
    }

    /// <summary>True if the last type read is a reference.</summary>
    public bool IsReference()
    {
        return myLastType == BinTools_ObjectType.BinTools_ObjectType_Reference8
            || myLastType == BinTools_ObjectType.BinTools_ObjectType_Reference16
            || myLastType == BinTools_ObjectType.BinTools_ObjectType_Reference32
            || myLastType == BinTools_ObjectType.BinTools_ObjectType_Reference64;
    }

    /// <summary>Reads a reference of the last type read, and returns the position it refers to.</summary>
    public ulong ReadReference()
    {
        ulong aDelta = 0;
        ulong aCurrentPos = (ulong)myStream.Position;
        switch (myLastType)
        {
            case BinTools_ObjectType.BinTools_ObjectType_Reference8:
                aDelta = (ulong)BinTools.Get(myStream);
                myPosition++;
                break;
            case BinTools_ObjectType.BinTools_ObjectType_Reference16:
            {
                BinTools.ReadExactly(myStream, myBuf.AsSpan(0, sizeof(ushort)));
                myPosition += 2;
                aDelta = BinaryPrimitives.ReadUInt16LittleEndian(myBuf);
                break;
            }
            case BinTools_ObjectType.BinTools_ObjectType_Reference32:
            {
                BinTools.ReadExactly(myStream, myBuf.AsSpan(0, sizeof(uint)));
                myPosition += 4;
                aDelta = BinaryPrimitives.ReadUInt32LittleEndian(myBuf);
                break;
            }
            case BinTools_ObjectType.BinTools_ObjectType_Reference64:
                BinTools.ReadExactly(myStream, myBuf.AsSpan(0, sizeof(ulong)));
                myPosition += 8;
                aDelta = BinaryPrimitives.ReadUInt64LittleEndian(myBuf);
                break;
            default:
                break;
        }

        if (aDelta == 0)
        {
            throw new FailureException("BinTools_IStream::ReadReference: invalid reference " + (char)myLastType + "\n");
        }

        if (aDelta >= aCurrentPos)
        {
            // OCCT.NET: untrusted input: the reference would point before the start of the stream
            throw new StreamFormatErrorException("BinTools_IStream::ReadReference: a reference before the stream");
        }

        return aCurrentPos - aDelta - 1; // add a type-byte
    }

    /// <summary>The stream read from.</summary>
    public Stream Stream() => myStream;

    /// <summary>Brings the position up to date after the stream was read directly.</summary>
    public void UpdatePosition() => myPosition = (ulong)myStream.Position;

    /// <summary>
    /// False if reading failed (OCCT's <c>operator bool</c>). The port throws instead of failing, so it is always true.
    /// </summary>
    public static implicit operator bool(BinTools_IStream theStream) => true;

    /// <summary>Reads a real.</summary>
    public double ReadReal()
    {
        BinTools.ReadExactly(myStream, myBuf.AsSpan(0, sizeof(double)));
        myPosition += sizeof(double);
        return BinaryPrimitives.ReadDoubleLittleEndian(myBuf);
    }

    /// <summary>Reads an integer.</summary>
    public int ReadInteger()
    {
        BinTools.ReadExactly(myStream, myBuf.AsSpan(0, sizeof(int)));
        myPosition += sizeof(int);
        return BinaryPrimitives.ReadInt32LittleEndian(myBuf);
    }

    /// <summary>Reads a 3D point.</summary>
    public Point3D ReadPnt()
    {
        BinTools.ReadExactly(myStream, myBuf.AsSpan(0, sizeof(double) * 3));
        Point3D aValue = new(BinaryPrimitives.ReadDoubleLittleEndian(myBuf),
                            BinaryPrimitives.ReadDoubleLittleEndian(myBuf.AsSpan(8)),
                            BinaryPrimitives.ReadDoubleLittleEndian(myBuf.AsSpan(16)));
        myPosition += 3 * sizeof(double);
        return aValue;
    }

    /// <summary>Reads a byte.</summary>
    public byte ReadByte()
    {
        byte aValue = (byte)BinTools.Get(myStream);
        myPosition += sizeof(byte);
        return aValue;
    }

    /// <summary>Reads a boolean (one byte, non-zero for true).</summary>
    public bool ReadBool() => ReadByte() != 0;

    /// <summary>Reads a short real.</summary>
    public float ReadShortReal()
    {
        BinTools.ReadExactly(myStream, myBuf.AsSpan(0, sizeof(float)));
        myPosition += sizeof(float);
        return BinaryPrimitives.ReadSingleLittleEndian(myBuf);
    }

    /// <summary>Reads a transformation written by <see cref="BinTools_OStream"/>.</summary>
    public Transform3D ReadTrsf()
    {
        double aV10 = ReadReal(), aV11 = ReadReal(), aV12 = ReadReal(), aV0 = ReadReal();
        double aV20 = ReadReal(), aV21 = ReadReal(), aV22 = ReadReal(), aV1 = ReadReal();
        double aV30 = ReadReal(), aV31 = ReadReal(), aV32 = ReadReal(), aV2 = ReadReal();
        Transform3D theValue = new();
        theValue.SetValues(aV10, aV11, aV12, aV0, aV20, aV21, aV22, aV1, aV30, aV31, aV32, aV2);
        return theValue;
    }

    /// <summary>Reads three booleans from one byte (bits 0 to 2).</summary>
    public void ReadBools(out bool theBool1, out bool theBool2, out bool theBool3)
    {
        byte aByte = ReadByte();
        theBool1 = (aByte & 1) == 1;
        theBool2 = (aByte & 2) == 2;
        theBool3 = (aByte & 4) == 4;
    }

    /// <summary>Reads seven booleans from one byte (bits 0 to 6).</summary>
    public void ReadBools(out bool theBool1,
                          out bool theBool2,
                          out bool theBool3,
                          out bool theBool4,
                          out bool theBool5,
                          out bool theBool6,
                          out bool theBool7)
    {
        byte aByte = ReadByte();
        theBool1 = (aByte & 1) == 1;
        theBool2 = (aByte & 2) == 2;
        theBool3 = (aByte & 4) == 4;
        theBool4 = (aByte & 8) == 8;
        theBool5 = (aByte & 16) == 16;
        theBool6 = (aByte & 32) == 32;
        theBool7 = (aByte & 64) == 64;
    }
}
