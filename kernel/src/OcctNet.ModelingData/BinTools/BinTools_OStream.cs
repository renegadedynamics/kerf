// Port of: src/ModelingData/TKBRep/BinTools/BinTools_OStream.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_OStream.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using System.Buffers.Binary;

namespace Occt;

/// <summary>
/// A binary output stream that knows its position, for the position-based format of <see cref="BinTools_ShapeWriter"/>
/// (references to objects written earlier are offsets back to them). Little-endian.
/// </summary>
/// <remarks>
/// OCCT's <c>operator&lt;&lt;</c> overloads are C# shift operators, so ported code reads as the C++ does. The position
/// starts at the stream's position (0 for a stream that can't seek: references are relative, so the bytes are the
/// same).
/// </remarks>
public sealed class BinTools_OStream
{
    private readonly Stream myStream;

    // the current position, relative to the stream's position when this object was created
    private ulong myPosition;

    // OCCT's buffers for 12 reals, 3 integers and 3 floats
    private readonly byte[] myBuf = new byte[sizeof(double) * 12];

    /// <summary>Writes to the stream, starting at its current position.</summary>
    public BinTools_OStream(Stream theStream)
    {
        myStream = theStream;
        myPosition = theStream.CanSeek ? (ulong)theStream.Position : 0;
    }

    /// <summary>The current position in the stream.</summary>
    public ulong Position() => myPosition;

    /// <summary>The stream written to.</summary>
    public Stream Stream() => myStream;

    /// <summary>Writes a reference to the given position: the offset back to it from here.</summary>
    public void WriteReference(ulong thePosition)
    {
        ulong aDelta = myPosition - thePosition;
        if (aDelta <= 0xFF)
        {
            myStream.WriteByte((byte)BinTools_ObjectType.BinTools_ObjectType_Reference8);
            myStream.WriteByte((byte)aDelta);
            myPosition += sizeof(byte) * 2;
        }
        else if (aDelta <= 0xFFFF)
        {
            myStream.WriteByte((byte)BinTools_ObjectType.BinTools_ObjectType_Reference16);
            BinaryPrimitives.WriteUInt16LittleEndian(myBuf, (ushort)aDelta);
            myStream.Write(myBuf, 0, sizeof(ushort));
            myPosition += sizeof(byte) + sizeof(ushort);
        }
        else if (aDelta <= 0xFFFFFFFF)
        {
            myStream.WriteByte((byte)BinTools_ObjectType.BinTools_ObjectType_Reference32);
            BinaryPrimitives.WriteUInt32LittleEndian(myBuf, (uint)aDelta);
            myStream.Write(myBuf, 0, sizeof(uint));
            myPosition += sizeof(byte) + sizeof(uint);
        }
        else
        {
            myStream.WriteByte((byte)BinTools_ObjectType.BinTools_ObjectType_Reference64);
            BinaryPrimitives.WriteUInt64LittleEndian(myBuf, aDelta);
            myStream.Write(myBuf, 0, sizeof(ulong));
            myPosition += sizeof(byte) + sizeof(ulong);
        }
    }

    /// <summary>Writes the identifier of a shape's type and orientation.</summary>
    public void WriteShape(TopAbs_ShapeEnum theType, TopAbs_Orientation theOrientation)
    {
        // taking into account that orientation <= 3 and type <= 8
        byte aType = (byte)((byte)BinTools_ObjectType.BinTools_ObjectType_EndShape + 1
                            + ((byte)theType << 2)
                            + (byte)theOrientation);
        myStream.WriteByte(aType);
        myPosition += sizeof(byte);
    }

    /// <summary>Writes an object type.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, BinTools_ObjectType theType)
    {
        OS.myStream.WriteByte((byte)theType);
        OS.myPosition += sizeof(byte);
        return OS;
    }

    /// <summary>Writes a byte.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, byte theValue)
    {
        OS.myStream.WriteByte(theValue);
        OS.myPosition += sizeof(byte);
        return OS;
    }

    /// <summary>Writes a real.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, double theValue)
    {
        BinaryPrimitives.WriteDoubleLittleEndian(OS.myBuf, theValue);
        OS.myStream.Write(OS.myBuf, 0, sizeof(double));
        OS.myPosition += sizeof(double);
        return OS;
    }

    /// <summary>Writes a boolean (one byte).</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, bool theValue)
    {
        OS.myStream.WriteByte((byte)(theValue ? 1 : 0));
        OS.myPosition += sizeof(byte);
        return OS;
    }

    /// <summary>Writes an integer.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, int theValue)
    {
        BinaryPrimitives.WriteInt32LittleEndian(OS.myBuf, theValue);
        OS.myStream.Write(OS.myBuf, 0, sizeof(int));
        OS.myPosition += sizeof(int);
        return OS;
    }

    /// <summary>Writes an extended (UTF-16) character.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, char theValue)
    {
        BinaryPrimitives.WriteUInt16LittleEndian(OS.myBuf, theValue);
        OS.myStream.Write(OS.myBuf, 0, sizeof(char));
        OS.myPosition += sizeof(char);
        return OS;
    }

    /// <summary>Writes a 3D point.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, in Point3D theValue)
    {
        OS.PutReals(theValue.X(), theValue.Y(), theValue.Z());
        return OS;
    }

    /// <summary>Writes a 3D direction.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, in Direction3D theValue)
    {
        OS.PutReals(theValue.X(), theValue.Y(), theValue.Z());
        return OS;
    }

    /// <summary>Writes a 2D point.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, in Point2D theValue)
    {
        BinaryPrimitives.WriteDoubleLittleEndian(OS.myBuf, theValue.X());
        BinaryPrimitives.WriteDoubleLittleEndian(OS.myBuf.AsSpan(8), theValue.Y());
        OS.myStream.Write(OS.myBuf, 0, sizeof(double) * 2);
        OS.myPosition += sizeof(double) * 2;
        return OS;
    }

    /// <summary>Writes a 2D direction.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, in Direction2D theValue)
    {
        BinaryPrimitives.WriteDoubleLittleEndian(OS.myBuf, theValue.X());
        BinaryPrimitives.WriteDoubleLittleEndian(OS.myBuf.AsSpan(8), theValue.Y());
        OS.myStream.Write(OS.myBuf, 0, sizeof(double) * 2);
        OS.myPosition += sizeof(double) * 2;
        return OS;
    }

    /// <summary>Writes a transformation: three rows of the matrix, each followed by its translation component.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, in Transform3D theValue)
    {
        Xyz aTr = theValue.TranslationPart();
        Matrix3x3 aMat = theValue.VectorialPart();
        Span<byte> aBuf = OS.myBuf;
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf, aMat[1, 1]);
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[8..], aMat[1, 2]);
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[16..], aMat[1, 3]);
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[24..], aTr.Coord(1));
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[32..], aMat[2, 1]);
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[40..], aMat[2, 2]);
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[48..], aMat[2, 3]);
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[56..], aTr.Coord(2));
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[64..], aMat[3, 1]);
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[72..], aMat[3, 2]);
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[80..], aMat[3, 3]);
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf[88..], aTr.Coord(3));
        OS.myStream.Write(OS.myBuf, 0, sizeof(double) * 12);
        OS.myPosition += sizeof(double) * 12;
        return OS;
    }

    /// <summary>Writes the three node indices of a triangle.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, in Triangle1 theValue)
    {
        BinaryPrimitives.WriteInt32LittleEndian(OS.myBuf, theValue.Value(1));
        BinaryPrimitives.WriteInt32LittleEndian(OS.myBuf.AsSpan(4), theValue.Value(2));
        BinaryPrimitives.WriteInt32LittleEndian(OS.myBuf.AsSpan(8), theValue.Value(3));
        OS.myStream.Write(OS.myBuf, 0, sizeof(int) * 3);
        OS.myPosition += sizeof(int) * 3;
        return OS;
    }

    /// <summary>Writes a vector of three floats.</summary>
    public static BinTools_OStream operator <<(BinTools_OStream OS, in NCollection_Vec3<float> theValue)
    {
        BinaryPrimitives.WriteSingleLittleEndian(OS.myBuf, theValue.X());
        BinaryPrimitives.WriteSingleLittleEndian(OS.myBuf.AsSpan(4), theValue.Y());
        BinaryPrimitives.WriteSingleLittleEndian(OS.myBuf.AsSpan(8), theValue.Z());
        OS.myStream.Write(OS.myBuf, 0, sizeof(float) * 3);
        OS.myPosition += sizeof(float) * 3;
        return OS;
    }

    /// <summary>Writes three booleans as one byte (bits 0 to 2).</summary>
    public void PutBools(bool theValue1, bool theValue2, bool theValue3)
    {
        byte aValue = (byte)((theValue1 ? 1 : 0) | (theValue2 ? 2 : 0) | (theValue3 ? 4 : 0));
        myStream.WriteByte(aValue);
        myPosition += sizeof(byte);
    }

    /// <summary>Writes seven booleans as one byte (bits 0 to 6).</summary>
    public void PutBools(bool theValue1,
                         bool theValue2,
                         bool theValue3,
                         bool theValue4,
                         bool theValue5,
                         bool theValue6,
                         bool theValue7)
    {
        byte aValue = (byte)((theValue1 ? 1 : 0) | (theValue2 ? 2 : 0) | (theValue3 ? 4 : 0)
                             | (theValue4 ? 8 : 0) | (theValue5 ? 16 : 0) | (theValue6 ? 32 : 0)
                             | (theValue7 ? 64 : 0));
        myStream.WriteByte(aValue);
        myPosition += sizeof(byte);
    }

    private void PutReals(double theX, double theY, double theZ)
    {
        BinaryPrimitives.WriteDoubleLittleEndian(myBuf, theX);
        BinaryPrimitives.WriteDoubleLittleEndian(myBuf.AsSpan(8), theY);
        BinaryPrimitives.WriteDoubleLittleEndian(myBuf.AsSpan(16), theZ);
        myStream.Write(myBuf, 0, sizeof(double) * 3);
        myPosition += sizeof(double) * 3;
    }
}
