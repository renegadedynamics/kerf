// Port of: src/ModelingData/TKBRep/BinTools/BinTools.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using System.Buffers.Binary;
using System.Runtime.CompilerServices;
using System.Text;

namespace Occt;

/// <summary>
/// OCCT's binary BRep format: the primitive writers and readers (little-endian, as OCCT writes on every platform it
/// builds for), and <see cref="Write(in TopoDS_Shape, Stream, bool, bool, BinTools_FormatVersion, ProgressRange)"/>
/// and <see cref="Read(out TopoDS_Shape, Stream, ProgressRange)"/> of a whole shape through a
/// <see cref="BinTools_ShapeSet"/>.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's <c>Standard_OStream</c>/<c>Standard_IStream</c> are a <see cref="Stream"/>. The format mixes a few lines of
/// text (section headers such as <c>"Curves 3\n"</c>, read with C++'s formatted extraction) with binary records.
/// </para>
/// <para>
/// <b>Untrusted input</b> (phase3-design.md, section 7). The readers are OCCT's, with these additions, which throw
/// <see cref="StreamFormatErrorException"/>: input that ends early (OCCT throws Storage_StreamTypeMismatchError from the
/// typed reads and carries on with garbage from the byte reads); counts larger than the rest of the stream could hold
/// (<see cref="Stream.Length"/> for a seekable stream, <see cref="NonSeekableLimit"/> bytes otherwise); words longer
/// than OCCT's 255-character buffers; indices outside the sets read so far; orientations and shape types outside their
/// enums; nesting deeper than <see cref="MaxGeometryNesting"/> (trimmed, offset and swept geometry),
/// <see cref="MaxShapeNesting"/> (shapes) or <see cref="MaxLocationItems"/> (the items of one location); and cycles of
/// references in the position-based format of <see cref="BinTools_ShapeReader"/>. Other exceptions that a malformed
/// stream can raise inside ported code (a null or out-of-range access, which OCCT's <c>OCC_CATCH_SIGNALS</c> turns into a
/// Standard_Failure) are caught where OCCT catches, and surface from the public readers as a Standard_Failure.
/// </para>
/// </remarks>
public static class BinTools
{
    /// <summary>The deepest nesting of trimmed, offset and swept curves and surfaces a reader accepts.</summary>
    public const int MaxGeometryNesting = 64;

    /// <summary>The deepest nesting of shapes (a compound in a compound ...) a reader accepts.</summary>
    public const int MaxShapeNesting = 1000;

    /// <summary>The most items (datum and power pairs) a location read from a stream may have.</summary>
    public const int MaxLocationItems = 1000;

    /// <summary>
    /// The most bytes a count read from a stream that can't seek (whose length isn't known) may imply: 256 MiB.
    /// </summary>
    public const long NonSeekableLimit = 1L << 28;

    // ---------------------------------------------------------------------------------------------------------------
    // Primitives

    /// <summary>Writes a real (8 bytes, little-endian).</summary>
    public static Stream PutReal(Stream OS, double theValue)
    {
        Span<byte> aBuf = stackalloc byte[8];
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf, theValue);
        OS.Write(aBuf);
        return OS;
    }

    /// <summary>Writes a short real (4 bytes, little-endian).</summary>
    public static Stream PutShortReal(Stream OS, float theValue)
    {
        Span<byte> aBuf = stackalloc byte[4];
        BinaryPrimitives.WriteSingleLittleEndian(aBuf, theValue);
        OS.Write(aBuf);
        return OS;
    }

    /// <summary>Writes an integer (4 bytes, little-endian).</summary>
    public static Stream PutInteger(Stream OS, int theValue)
    {
        Span<byte> aBuf = stackalloc byte[4];
        BinaryPrimitives.WriteInt32LittleEndian(aBuf, theValue);
        OS.Write(aBuf);
        return OS;
    }

    /// <summary>Writes a boolean (one byte, 0 or 1).</summary>
    public static Stream PutBool(Stream OS, bool theValue)
    {
        OS.WriteByte((byte)(theValue ? 1 : 0));
        return OS;
    }

    /// <summary>Writes an extended (UTF-16) character (2 bytes, little-endian).</summary>
    public static Stream PutExtChar(Stream OS, char theValue)
    {
        Span<byte> aBuf = stackalloc byte[2];
        BinaryPrimitives.WriteUInt16LittleEndian(aBuf, theValue);
        OS.Write(aBuf);
        return OS;
    }

    /// <summary>Reads a real; throws <see cref="StreamFormatErrorException"/> at the end of the stream.</summary>
    public static Stream GetReal(Stream IS, out double theValue)
    {
        Span<byte> aBuf = stackalloc byte[8];
        ReadExactly(IS, aBuf);
        theValue = BinaryPrimitives.ReadDoubleLittleEndian(aBuf);
        return IS;
    }

    /// <summary>Reads a short real; throws <see cref="StreamFormatErrorException"/> at the end of the stream.</summary>
    public static Stream GetShortReal(Stream IS, out float theValue)
    {
        Span<byte> aBuf = stackalloc byte[4];
        ReadExactly(IS, aBuf);
        theValue = BinaryPrimitives.ReadSingleLittleEndian(aBuf);
        return IS;
    }

    /// <summary>Reads an integer; throws <see cref="StreamFormatErrorException"/> at the end of the stream.</summary>
    public static Stream GetInteger(Stream IS, out int theValue)
    {
        Span<byte> aBuf = stackalloc byte[4];
        ReadExactly(IS, aBuf);
        theValue = BinaryPrimitives.ReadInt32LittleEndian(aBuf);
        return IS;
    }

    /// <summary>
    /// Reads an extended character; throws <see cref="StreamFormatErrorException"/> at the end of the stream.
    /// </summary>
    public static Stream GetExtChar(Stream IS, out char theValue)
    {
        Span<byte> aBuf = stackalloc byte[2];
        ReadExactly(IS, aBuf);
        theValue = (char)BinaryPrimitives.ReadUInt16LittleEndian(aBuf);
        return IS;
    }

    /// <summary>Reads a boolean (any non-zero byte is true).</summary>
    /// <remarks>OCCT reads true at the end of the stream (<c>IS.get()</c> returns EOF); the port throws.</remarks>
    public static Stream GetBool(Stream IS, out bool theValue)
    {
        theValue = Get(IS) != 0;
        return IS;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Shapes

    /// <summary>
    /// Writes the shape in the current binary format (<see cref="BinTools_FormatVersion.BinTools_FormatVersion_CURRENT"/>)
    /// with its triangulations, without normals.
    /// </summary>
    public static void Write(in TopoDS_Shape theShape, Stream theStream, ProgressRange theRange = default)
    {
        Write(theShape, theStream, true, false, BinTools_FormatVersion.BinTools_FormatVersion_CURRENT, theRange);
    }

    /// <summary>
    /// Writes the shape in the given binary format version, with or without triangulations and normals (both ignored
    /// for a face that has only a triangulation, which is always written with its normals).
    /// </summary>
    public static void Write(in TopoDS_Shape theShape,
                             Stream theStream,
                             bool theWithTriangles,
                             bool theWithNormals,
                             BinTools_FormatVersion theVersion,
                             ProgressRange theRange = default)
    {
        var aShapeSet = new BinTools_ShapeSet();
        aShapeSet.SetWithTriangles(theWithTriangles);
        aShapeSet.SetWithNormals(theWithNormals);
        aShapeSet.SetFormatNb((int)theVersion);
        aShapeSet.Add(theShape);
        aShapeSet.Write(theStream, theRange);
        aShapeSet.Write(theShape, theStream);
    }

    /// <summary>
    /// Reads a shape written by <see cref="Write(in TopoDS_Shape, Stream, ProgressRange)"/>. The shape is frozen.
    /// A stream that can't seek is read to its end first.
    /// </summary>
    /// <exception cref="FailureException">The stream isn't a binary shape OCCT can read; for input that can't be read
    /// safely, a <see cref="StreamFormatErrorException"/>.</exception>
    public static void Read(out TopoDS_Shape theShape, Stream theStream, ProgressRange theRange = default)
    {
        theShape = new TopoDS_Shape();
        if (!theStream.CanSeek)
        {
            // OCCT.NET: reading a vertex steps back in the stream (BinTools_ShapeSet::ReadShape's tellg and seekg), so
            // a stream that can't seek is read into memory first
            var aBuffer = new MemoryStream();
            theStream.CopyTo(aBuffer);
            aBuffer.Position = 0;
            theStream = aBuffer;
        }

        var aShapeSet = new BinTools_ShapeSet();
        aShapeSet.SetWithTriangles(true);
        try
        {
            aShapeSet.Read(theStream, theRange);
            aShapeSet.ReadSubs(ref theShape, theStream, aShapeSet.NbShapes());
        }
        catch (Exception anException) when (IsSignal(anException))
        {
            // OCCT.NET: untrusted input: a malformed stream raised an exception that OCCT doesn't catch here
            throw new StreamFormatErrorException("BinTools::Read: " + anException.Message, anException);
        }
    }

    /// <summary>Writes the shape to a file in the current binary format, with triangulations. False on failure.</summary>
    public static bool Write(in TopoDS_Shape theShape, string theFile, ProgressRange theRange = default)
    {
        return Write(theShape, theFile, true, false, BinTools_FormatVersion.BinTools_FormatVersion_CURRENT, theRange);
    }

    /// <summary>Writes the shape to a file in the given binary format version. False if the file can't be written.</summary>
    /// <remarks>OCCT's <c>OSD_FileSystem</c> is left out (decision 69): the file is a <see cref="FileStream"/>.</remarks>
    public static bool Write(in TopoDS_Shape theShape,
                             string theFile,
                             bool theWithTriangles,
                             bool theWithNormals,
                             BinTools_FormatVersion theVersion,
                             ProgressRange theRange = default)
    {
        FileStream aStream;
        try
        {
            aStream = new FileStream(theFile, FileMode.Create, FileAccess.Write);
        }
        catch (IOException)
        {
            return false;
        }
        catch (UnauthorizedAccessException)
        {
            return false;
        }

        using (aStream)
        {
            var aBuffered = new BufferedStream(aStream);
            Write(theShape, aBuffered, theWithTriangles, theWithNormals, theVersion, theRange);
            aBuffered.Flush();
        }

        return true;
    }

    /// <summary>Reads a shape from a binary file. False if the file can't be opened; the shape is frozen.</summary>
    public static bool Read(out TopoDS_Shape theShape, string theFile, ProgressRange theRange = default)
    {
        theShape = new TopoDS_Shape();
        FileStream aStream;
        try
        {
            aStream = new FileStream(theFile, FileMode.Open, FileAccess.Read);
        }
        catch (IOException)
        {
            return false;
        }
        catch (UnauthorizedAccessException)
        {
            return false;
        }

        using (aStream)
        {
            var aBuffered = new BufferedStream(aStream);
            Read(out theShape, aBuffered, theRange);
        }

        // OCCT returns the stream's state; a stream that failed has thrown above
        return true;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The C++ stream operations the format uses, over a Stream

    // istream::read of a fixed size
    internal static void ReadExactly(Stream IS, Span<byte> theBuffer)
    {
        int aRead = 0;
        while (aRead < theBuffer.Length)
        {
            int aNb = IS.Read(theBuffer[aRead..]);
            if (aNb <= 0)
            {
                // OCCT: throws Storage_StreamTypeMismatchError
                // OCCT.NET: untrusted input: the stream ended early
                throw EndOfStream();
            }

            aRead += aNb;
        }
    }

    internal static StreamFormatErrorException EndOfStream() => new("BinTools: unexpected end of the stream");

    // istream::get(): one byte. OCCT returns EOF at the end of the stream and the callers carry on with it.
    internal static int Get(Stream IS)
    {
        int aByte = TakePushedBack(IS);
        if (aByte < 0)
        {
            aByte = IS.ReadByte();
        }

        if (aByte < 0)
        {
            // OCCT.NET: untrusted input: the stream ended early
            throw EndOfStream();
        }

        return aByte;
    }

    // ostream << const char*
    internal static void PutText(Stream OS, string theText)
    {
        Span<byte> aBuf = theText.Length <= 256 ? stackalloc byte[theText.Length] : new byte[theText.Length];
        for (int i = 0; i < theText.Length; i++)
        {
            aBuf[i] = (byte)theText[i];
        }

        OS.Write(aBuf);
    }

    // ostream << int
    internal static void PutText(Stream OS, int theValue) => PutText(OS, theValue.ToString(System.Globalization.CultureInfo.InvariantCulture));

    // isspace() in the C locale
    private static bool IsSpace(int theChar) => theChar is ' ' or '\t' or '\n' or '\v' or '\f' or '\r';

    /// <summary>
    /// C++'s <c>IS &gt;&gt; aBuffer</c> into OCCT's <c>char buffer[255]</c>: white space is skipped, then a word is read
    /// up to the next white space (which is consumed: a formatted read, which skips it anyway, always follows in this
    /// format). Null if the stream ends before a word (C++'s fail).
    /// </summary>
    internal static string? ReadWord(Stream IS)
    {
        int aChar = TakePushedBack(IS);
        if (aChar < 0)
        {
            aChar = IS.ReadByte();
        }

        while (aChar >= 0 && IsSpace(aChar))
        {
            aChar = IS.ReadByte();
        }

        if (aChar < 0)
        {
            return null;
        }

        var aBuilder = new StringBuilder();
        while (aChar >= 0 && !IsSpace(aChar))
        {
            if (aBuilder.Length >= 254)
            {
                // OCCT overflows its buffer
                // OCCT.NET: untrusted input: a word longer than OCCT's buffer
                throw new StreamFormatErrorException("BinTools: a word longer than 254 characters");
            }

            aBuilder.Append((char)aChar);
            aChar = IS.ReadByte();
        }

        return aBuilder.ToString();
    }

    /// <summary>
    /// C++'s <c>IS &gt;&gt; theValue</c> of an int: white space is skipped, then an optional sign and digits; the
    /// character after them is left in the stream.
    /// </summary>
    /// <remarks>
    /// C++ fails (and gives 0) when there are no digits or the number overflows; the port throws
    /// <see cref="StreamFormatErrorException"/> (OCCT goes on to throw or to read garbage).
    /// </remarks>
    internal static int ReadFormattedInteger(Stream IS)
    {
        int aChar = TakePushedBack(IS);
        if (aChar < 0)
        {
            aChar = IS.ReadByte();
        }

        while (aChar >= 0 && IsSpace(aChar))
        {
            aChar = IS.ReadByte();
        }

        bool isNegative = false;
        if (aChar is '+' or '-')
        {
            isNegative = aChar == '-';
            aChar = IS.ReadByte();
        }

        long aValue = 0;
        bool hasDigit = false;
        while (aChar is >= '0' and <= '9')
        {
            hasDigit = true;
            aValue = aValue * 10 + (aChar - '0');
            if (aValue > (long)int.MaxValue + 1)
            {
                // OCCT.NET: untrusted input: the count overflows
                throw new StreamFormatErrorException("BinTools: an integer out of range");
            }

            aChar = IS.ReadByte();
        }

        if (!hasDigit)
        {
            // OCCT.NET: untrusted input: C++ fails and reads 0
            throw new StreamFormatErrorException("BinTools: an integer expected");
        }

        // leave the terminating character in the stream
        if (aChar >= 0)
        {
            PushBack(IS, aChar);
        }

        aValue = isNegative ? -aValue : aValue;
        if (aValue > int.MaxValue || aValue < int.MinValue)
        {
            // OCCT.NET: untrusted input: the count overflows
            throw new StreamFormatErrorException("BinTools: an integer out of range");
        }

        return (int)aValue;
    }

    /// <summary>
    /// C++'s <c>IS &gt;&gt; aChar</c>: white space is skipped, then one character is read.
    /// </summary>
    internal static int ReadFormattedChar(Stream IS)
    {
        int aChar = Get(IS);
        while (IsSpace(aChar))
        {
            aChar = Get(IS);
        }

        return aChar;
    }

    /// <summary>
    /// C++'s <c>IS.getline(aBuffer, 100, '\n')</c>: reads up to the delimiter (consumed, not stored) or 99 characters.
    /// Returns false (C++'s fail) when nothing could be extracted or the line is longer than 99 characters.
    /// </summary>
    internal static bool GetLine(Stream IS, out string theLine, ref bool theIsEof)
    {
        theLine = "";
        if (theIsEof)
        {
            return false;
        }

        var aBuilder = new StringBuilder();
        int aNbExtracted = 0;
        int aChar = TakePushedBack(IS);
        if (aChar < 0)
        {
            aChar = IS.ReadByte();
        }

        while (true)
        {
            if (aChar < 0)
            {
                theIsEof = true;
                theLine = aBuilder.ToString();
                return aNbExtracted > 0;
            }

            if (aChar == '\n')
            {
                theLine = aBuilder.ToString();
                return true;
            }

            if (aBuilder.Length == 99)
            {
                // too long: C++ sets failbit, leaving the character in the stream
                PushBack(IS, aChar);
                theLine = aBuilder.ToString();
                return false;
            }

            aBuilder.Append((char)aChar);
            aNbExtracted++;
            aChar = IS.ReadByte();
        }
    }

    // A one-character push-back, for the character after a formatted integer (C++ peeks it). A seekable stream steps
    // back; another stream keeps it here, and Get, ReadWord and the formatted reads take it first. In this format a
    // formatted integer is always followed by IS.get() (the line feed).
    private static readonly ConditionalWeakTable<Stream, StrongBox<int>> thePushedBack = new();

    private static void PushBack(Stream IS, int theChar)
    {
        if (IS.CanSeek)
        {
            IS.Seek(-1, SeekOrigin.Current);
        }
        else
        {
            thePushedBack.AddOrUpdate(IS, new StrongBox<int>(theChar));
        }
    }

    private static int TakePushedBack(Stream IS)
    {
        if (IS.CanSeek)
        {
            return -1;
        }

        if (thePushedBack.TryGetValue(IS, out StrongBox<int>? aBox))
        {
            thePushedBack.Remove(IS);
            return aBox.Value;
        }

        return -1;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Untrusted input

    /// <summary>
    /// Checks a count read from the stream against what the rest of it could hold, at <paramref name="theBytesPerItem"/>
    /// bytes per item, before anything is allocated for it.
    /// </summary>
    internal static void CheckCount(Stream IS, long theCount, long theBytesPerItem, string theWhat)
    {
        // OCCT.NET: untrusted input
        if (theCount < 0)
        {
            throw new StreamFormatErrorException("BinTools: a negative count of " + theWhat);
        }

        long aRemaining = IS.CanSeek ? Math.Max(0, IS.Length - IS.Position) : NonSeekableLimit;
        if (theBytesPerItem > 0 && theCount > aRemaining / theBytesPerItem)
        {
            throw new StreamFormatErrorException("BinTools: " + theCount + " " + theWhat + " can't fit in the rest of the stream");
        }
    }

    /// <summary>Checks an index read from the stream: <paramref name="theLower"/> to <paramref name="theUpper"/>.</summary>
    internal static void CheckIndex(int theIndex, int theLower, int theUpper, string theWhat)
    {
        // OCCT.NET: untrusted input: OCCT reads outside its maps (an exception in debug builds only)
        if (theIndex < theLower || theIndex > theUpper)
        {
            throw new StreamFormatErrorException("BinTools: no " + theWhat + " " + theIndex);
        }
    }

    /// <summary>Checks the orientation byte read from the stream.</summary>
    internal static TopAbs_Orientation CheckOrientation(int theValue)
    {
        // OCCT.NET: untrusted input: OCCT casts any byte to TopAbs_Orientation
        if (theValue < 0 || theValue > 3)
        {
            throw new StreamFormatErrorException("BinTools: no orientation " + theValue);
        }

        return (TopAbs_Orientation)theValue;
    }

    /// <summary>
    /// The location <c>theLocation.Powered(thePower) * theRight</c>, checked first: a location read from a stream can
    /// make Powered recurse once per power (when it isn't a single datum) and the product recurse once per item.
    /// </summary>
    internal static Placement PoweredTimes(in Placement theLocation, int thePower, in Placement theRight)
    {
        // OCCT.NET: untrusted input: bound the recursion of Powered and Multiplied
        long aNbItems = NbItems(theLocation);
        if (aNbItems > 1 && (thePower == int.MinValue || aNbItems * Math.Abs((long)thePower) > MaxLocationItems))
        {
            throw new StreamFormatErrorException("BinTools: a location with too many items");
        }

        if (aNbItems + NbItems(theRight) > MaxLocationItems + 1)
        {
            throw new StreamFormatErrorException("BinTools: a location with too many items");
        }

        return theLocation.Powered(thePower) * theRight;
    }

    // the number of items of a location, counted up to MaxLocationItems + 1
    private static long NbItems(in Placement theLocation)
    {
        long aNb = 0;
        for (Placement aLoc = theLocation; !aLoc.IsIdentity && aNb <= MaxLocationItems; aLoc = aLoc.NextLocation())
        {
            aNb++;
        }

        return aNb;
    }

    /// <summary>
    /// The exceptions that a malformed stream can raise inside ported code and that OCCT's <c>OCC_CATCH_SIGNALS</c> blocks
    /// catch (as Standard_Failure or a converted signal), except the port's own <see cref="StreamFormatErrorException"/>,
    /// which passes through OCCT's catch blocks unwrapped.
    /// </summary>
    internal static bool IsCaught(Exception theException)
    {
        if (theException is StreamFormatErrorException)
        {
            return false;
        }

        return theException is FailureException || IsSignal(theException);
    }

    // The .NET exceptions that stand for OCCT's access violations and the like on malformed input
    internal static bool IsSignal(Exception theException)
    {
        return theException is NullReferenceException
            or IndexOutOfRangeException
            or ArgumentException
            or InvalidCastException
            or ArithmeticException
            or InvalidOperationException
            or KeyNotFoundException;
    }

    // OCCT's message of a caught exception, "EXCEPTION in ...\n" followed by the exception
    internal static FailureException Rethrow(string theWhere, Exception theException)
    {
        return new FailureException("EXCEPTION in " + theWhere + "\n" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theException.GetType()) + ": "
                                    + theException.Message + "\n", theException);
    }
}
