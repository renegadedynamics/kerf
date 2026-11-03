// Port of: src/ModelingData/TKBRep/BinTools/BinTools.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using System.Buffers.Binary;
using System.Runtime.CompilerServices;
using System.Text;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// OCCT's binary BRep format: the primitive writers and readers (little-endian, as OCCT writes on every platform it
/// builds for), and <see cref="Write(in Shape, Stream, bool, bool, BinaryFormatVersion, ProgressRange)"/>
/// and <see cref="Read(out Shape, Stream, ProgressRange)"/> of a whole shape through a
/// <see cref="BinaryShapeSet"/>.
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
/// references in the position-based format of <see cref="ShapeReader"/>. Other exceptions that a malformed
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
    public static Stream PutReal(Stream os, double value)
    {
        Span<byte> buf = stackalloc byte[8];
        BinaryPrimitives.WriteDoubleLittleEndian(buf, value);
        os.Write(buf);
        return os;
    }

    /// <summary>Writes a short real (4 bytes, little-endian).</summary>
    public static Stream PutShortReal(Stream os, float value)
    {
        Span<byte> buf = stackalloc byte[4];
        BinaryPrimitives.WriteSingleLittleEndian(buf, value);
        os.Write(buf);
        return os;
    }

    /// <summary>Writes an integer (4 bytes, little-endian).</summary>
    public static Stream PutInteger(Stream os, int value)
    {
        Span<byte> buf = stackalloc byte[4];
        BinaryPrimitives.WriteInt32LittleEndian(buf, value);
        os.Write(buf);
        return os;
    }

    /// <summary>Writes a boolean (one byte, 0 or 1).</summary>
    public static Stream PutBool(Stream os, bool value)
    {
        os.WriteByte((byte)(value ? 1 : 0));
        return os;
    }

    /// <summary>Writes an extended (UTF-16) character (2 bytes, little-endian).</summary>
    public static Stream PutExtChar(Stream os, char value)
    {
        Span<byte> buf = stackalloc byte[2];
        BinaryPrimitives.WriteUInt16LittleEndian(buf, value);
        os.Write(buf);
        return os;
    }

    /// <summary>Reads a real; throws <see cref="StreamFormatErrorException"/> at the end of the stream.</summary>
    public static Stream GetReal(Stream IS, out double value)
    {
        Span<byte> buf = stackalloc byte[8];
        ReadExactly(IS, buf);
        value = BinaryPrimitives.ReadDoubleLittleEndian(buf);
        return IS;
    }

    /// <summary>Reads a short real; throws <see cref="StreamFormatErrorException"/> at the end of the stream.</summary>
    public static Stream GetShortReal(Stream IS, out float value)
    {
        Span<byte> buf = stackalloc byte[4];
        ReadExactly(IS, buf);
        value = BinaryPrimitives.ReadSingleLittleEndian(buf);
        return IS;
    }

    /// <summary>Reads an integer; throws <see cref="StreamFormatErrorException"/> at the end of the stream.</summary>
    public static Stream GetInteger(Stream IS, out int value)
    {
        Span<byte> buf = stackalloc byte[4];
        ReadExactly(IS, buf);
        value = BinaryPrimitives.ReadInt32LittleEndian(buf);
        return IS;
    }

    /// <summary>
    /// Reads an extended character; throws <see cref="StreamFormatErrorException"/> at the end of the stream.
    /// </summary>
    public static Stream GetExtChar(Stream IS, out char value)
    {
        Span<byte> buf = stackalloc byte[2];
        ReadExactly(IS, buf);
        value = (char)BinaryPrimitives.ReadUInt16LittleEndian(buf);
        return IS;
    }

    /// <summary>Reads a boolean (any non-zero byte is true).</summary>
    /// <remarks>OCCT reads true at the end of the stream (<c>IS.get()</c> returns EOF); the port throws.</remarks>
    public static Stream GetBool(Stream IS, out bool value)
    {
        value = Get(IS) != 0;
        return IS;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Shapes

    /// <summary>
    /// Writes the shape in the current binary format (<see cref="BinaryFormatVersion.FormatVersion_CURRENT"/>)
    /// with its triangulations, without normals.
    /// </summary>
    public static void Write(in Shape shape, Stream stream, ProgressRange range = default)
    {
        Write(shape, stream, true, false, BinaryFormatVersion.FormatVersion_CURRENT, range);
    }

    /// <summary>
    /// Writes the shape in the given binary format version, with or without triangulations and normals (both ignored
    /// for a face that has only a triangulation, which is always written with its normals).
    /// </summary>
    public static void Write(in Shape shape,
                             Stream stream,
                             bool withTriangles,
                             bool withNormals,
                             BinaryFormatVersion version,
                             ProgressRange range = default)
    {
        var shapeSet = new BinaryShapeSet();
        shapeSet.SetWithTriangles(withTriangles);
        shapeSet.SetWithNormals(withNormals);
        shapeSet.SetFormatNb((int)version);
        shapeSet.Add(shape);
        shapeSet.Write(stream, range);
        shapeSet.Write(shape, stream);
    }

    /// <summary>
    /// Reads a shape written by <see cref="Write(in Shape, Stream, ProgressRange)"/>. The shape is frozen.
    /// A stream that can't seek is read to its end first.
    /// </summary>
    /// <exception cref="FailureException">The stream isn't a binary shape OCCT can read; for input that can't be read
    /// safely, a <see cref="StreamFormatErrorException"/>.</exception>
    public static void Read(out Shape shape, Stream stream, ProgressRange range = default)
    {
        shape = new Shape();
        if (!stream.CanSeek)
        {
            // OCCT.NET: reading a vertex steps back in the stream (BinTools_ShapeSet::ReadShape's tellg and seekg), so
            // a stream that can't seek is read into memory first
            var buffer = new MemoryStream();
            stream.CopyTo(buffer);
            buffer.Position = 0;
            stream = buffer;
        }

        var shapeSet = new BinaryShapeSet();
        shapeSet.SetWithTriangles(true);
        try
        {
            shapeSet.Read(stream, range);
            shapeSet.ReadSubs(ref shape, stream, shapeSet.ShapeCount);
        }
        catch (Exception exception) when (IsSignal(exception))
        {
            // OCCT.NET: untrusted input: a malformed stream raised an exception that OCCT doesn't catch here
            throw new StreamFormatErrorException("BinTools::Read: " + exception.Message, exception);
        }
    }

    /// <summary>Writes the shape to a file in the current binary format, with triangulations. False on failure.</summary>
    public static bool Write(in Shape shape, string file, ProgressRange range = default)
    {
        return Write(shape, file, true, false, BinaryFormatVersion.FormatVersion_CURRENT, range);
    }

    /// <summary>Writes the shape to a file in the given binary format version. False if the file can't be written.</summary>
    /// <remarks>OCCT's <c>OSD_FileSystem</c> is left out (decision 69): the file is a <see cref="FileStream"/>.</remarks>
    public static bool Write(in Shape shape,
                             string file,
                             bool withTriangles,
                             bool withNormals,
                             BinaryFormatVersion version,
                             ProgressRange range = default)
    {
        FileStream stream;
        try
        {
            stream = new FileStream(file, FileMode.Create, FileAccess.Write);
        }
        catch (IOException)
        {
            return false;
        }
        catch (UnauthorizedAccessException)
        {
            return false;
        }

        using (stream)
        {
            var buffered = new BufferedStream(stream);
            Write(shape, buffered, withTriangles, withNormals, version, range);
            buffered.Flush();
        }

        return true;
    }

    /// <summary>Reads a shape from a binary file. False if the file can't be opened; the shape is frozen.</summary>
    public static bool Read(out Shape shape, string file, ProgressRange range = default)
    {
        shape = new Shape();
        FileStream stream;
        try
        {
            stream = new FileStream(file, FileMode.Open, FileAccess.Read);
        }
        catch (IOException)
        {
            return false;
        }
        catch (UnauthorizedAccessException)
        {
            return false;
        }

        using (stream)
        {
            var buffered = new BufferedStream(stream);
            Read(out shape, buffered, range);
        }

        // OCCT returns the stream's state; a stream that failed has thrown above
        return true;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The C++ stream operations the format uses, over a Stream

    // istream::read of a fixed size
    internal static void ReadExactly(Stream IS, Span<byte> buffer)
    {
        int read = 0;
        while (read < buffer.Length)
        {
            int nb = IS.Read(buffer[read..]);
            if (nb <= 0)
            {
                // OCCT: throws Storage_StreamTypeMismatchError
                // OCCT.NET: untrusted input: the stream ended early
                throw EndOfStream();
            }

            read += nb;
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
    internal static void PutText(Stream os, string text)
    {
        Span<byte> buf = text.Length <= 256 ? stackalloc byte[text.Length] : new byte[text.Length];
        for (int i = 0; i < text.Length; i++)
        {
            buf[i] = (byte)text[i];
        }

        os.Write(buf);
    }

    // ostream << int
    internal static void PutText(Stream os, int value) => PutText(os, value.ToString(System.Globalization.CultureInfo.InvariantCulture));

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

        var builder = new StringBuilder();
        while (aChar >= 0 && !IsSpace(aChar))
        {
            if (builder.Length >= 254)
            {
                // OCCT overflows its buffer
                // OCCT.NET: untrusted input: a word longer than OCCT's buffer
                throw new StreamFormatErrorException("BinTools: a word longer than 254 characters");
            }

            builder.Append((char)aChar);
            aChar = IS.ReadByte();
        }

        return builder.ToString();
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

        long value = 0;
        bool hasDigit = false;
        while (aChar is >= '0' and <= '9')
        {
            hasDigit = true;
            value = value * 10 + (aChar - '0');
            if (value > (long)int.MaxValue + 1)
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

        value = isNegative ? -value : value;
        if (value > int.MaxValue || value < int.MinValue)
        {
            // OCCT.NET: untrusted input: the count overflows
            throw new StreamFormatErrorException("BinTools: an integer out of range");
        }

        return (int)value;
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
    internal static bool GetLine(Stream IS, out string line, ref bool isEof)
    {
        line = "";
        if (isEof)
        {
            return false;
        }

        var builder = new StringBuilder();
        int nbExtracted = 0;
        int aChar = TakePushedBack(IS);
        if (aChar < 0)
        {
            aChar = IS.ReadByte();
        }

        while (true)
        {
            if (aChar < 0)
            {
                isEof = true;
                line = builder.ToString();
                return nbExtracted > 0;
            }

            if (aChar == '\n')
            {
                line = builder.ToString();
                return true;
            }

            if (builder.Length == 99)
            {
                // too long: C++ sets failbit, leaving the character in the stream
                PushBack(IS, aChar);
                line = builder.ToString();
                return false;
            }

            builder.Append((char)aChar);
            nbExtracted++;
            aChar = IS.ReadByte();
        }
    }

    // A one-character push-back, for the character after a formatted integer (C++ peeks it). A seekable stream steps
    // back; another stream keeps it here, and Get, ReadWord and the formatted reads take it first. In this format a
    // formatted integer is always followed by IS.get() (the line feed).
    private static readonly ConditionalWeakTable<Stream, StrongBox<int>> PushedBack = new();

    private static void PushBack(Stream IS, int theChar)
    {
        if (IS.CanSeek)
        {
            IS.Seek(-1, SeekOrigin.Current);
        }
        else
        {
            PushedBack.AddOrUpdate(IS, new StrongBox<int>(theChar));
        }
    }

    private static int TakePushedBack(Stream IS)
    {
        if (IS.CanSeek)
        {
            return -1;
        }

        if (PushedBack.TryGetValue(IS, out StrongBox<int>? box))
        {
            PushedBack.Remove(IS);
            return box.Value;
        }

        return -1;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Untrusted input

    /// <summary>
    /// Checks a count read from the stream against what the rest of it could hold, at <paramref name="bytesPerItem"/>
    /// bytes per item, before anything is allocated for it.
    /// </summary>
    internal static void CheckCount(Stream IS, long count, long bytesPerItem, string what)
    {
        // OCCT.NET: untrusted input
        if (count < 0)
        {
            throw new StreamFormatErrorException("BinTools: a negative count of " + what);
        }

        long remaining = IS.CanSeek ? Math.Max(0, IS.Length - IS.Position) : NonSeekableLimit;
        if (bytesPerItem > 0 && count > remaining / bytesPerItem)
        {
            throw new StreamFormatErrorException("BinTools: " + count + " " + what + " can't fit in the rest of the stream");
        }
    }

    /// <summary>Checks an index read from the stream: <paramref name="lower"/> to <paramref name="upper"/>.</summary>
    internal static void CheckIndex(int index, int lower, int upper, string what)
    {
        // OCCT.NET: untrusted input: OCCT reads outside its maps (an exception in debug builds only)
        if (index < lower || index > upper)
        {
            throw new StreamFormatErrorException("BinTools: no " + what + " " + index);
        }
    }

    /// <summary>Checks the orientation byte read from the stream.</summary>
    internal static Orientation CheckOrientation(int value)
    {
        // OCCT.NET: untrusted input: OCCT casts any byte to TopAbs_Orientation
        if (value < 0 || value > 3)
        {
            throw new StreamFormatErrorException("BinTools: no orientation " + value);
        }

        return (Orientation)value;
    }

    /// <summary>
    /// The location <c>theLocation.Powered(thePower) * theRight</c>, checked first: a location read from a stream can
    /// make Powered recurse once per power (when it isn't a single datum) and the product recurse once per item.
    /// </summary>
    internal static Placement PoweredTimes(in Placement location, int power, in Placement right)
    {
        // OCCT.NET: untrusted input: bound the recursion of Powered and Multiplied
        long nbItems = NbItems(location);
        if (nbItems > 1 && (power == int.MinValue || nbItems * Math.Abs((long)power) > MaxLocationItems))
        {
            throw new StreamFormatErrorException("BinTools: a location with too many items");
        }

        if (nbItems + NbItems(right) > MaxLocationItems + 1)
        {
            throw new StreamFormatErrorException("BinTools: a location with too many items");
        }

        return location.Powered(power) * right;
    }

    // the number of items of a location, counted up to MaxLocationItems + 1
    private static long NbItems(in Placement location)
    {
        long nb = 0;
        for (Placement loc = location; !loc.IsIdentity && nb <= MaxLocationItems; loc = loc.NextLocation())
        {
            nb++;
        }

        return nb;
    }

    /// <summary>
    /// The exceptions that a malformed stream can raise inside ported code and that OCCT's <c>OCC_CATCH_SIGNALS</c> blocks
    /// catch (as Standard_Failure or a converted signal), except the port's own <see cref="StreamFormatErrorException"/>,
    /// which passes through OCCT's catch blocks unwrapped.
    /// </summary>
    internal static bool IsCaught(Exception exception)
    {
        if (exception is StreamFormatErrorException)
        {
            return false;
        }

        return exception is FailureException || IsSignal(exception);
    }

    // The .NET exceptions that stand for OCCT's access violations and the like on malformed input
    internal static bool IsSignal(Exception exception)
    {
        return exception is NullReferenceException
            or IndexOutOfRangeException
            or ArgumentException
            or InvalidCastException
            or ArithmeticException
            or InvalidOperationException
            or KeyNotFoundException;
    }

    // OCCT's message of a caught exception, "EXCEPTION in ...\n" followed by the exception
    internal static FailureException Rethrow(string theWhere, Exception exception)
    {
        return new FailureException("EXCEPTION in " + theWhere + "\n" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(exception.GetType()) + ": "
                                    + exception.Message + "\n", exception);
    }
}
