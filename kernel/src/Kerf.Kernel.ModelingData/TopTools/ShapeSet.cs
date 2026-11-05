// Port of: src/ModelingData/TKBRep/TopTools/TopTools_ShapeSet.hxx
// Port of: src/ModelingData/TKBRep/TopTools/TopTools_ShapeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;
using System.Globalization;
using System.Runtime.CompilerServices;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Brep;

/// <summary>
/// A set of shapes, written and read in OCCT's text format: the locations, the geometry (in derived classes) and the
/// TShapes, each sub-shape written before the shapes that contain it. The base of
/// <see cref="BrepShapeSet"/>, which adds the geometry.
/// </summary>
/// <remarks>
/// <para>
/// A working object, as OCCT's: <see cref="Add"/> and <see cref="Read(TextReader, ProgressRange)"/> change the
/// set. The shapes it hands out are frozen: <see cref="Read(TextReader, ProgressRange)"/> freezes every shape
/// it creates once the whole set is read (docs/phase3-design.md, section 4.2), and <see cref="Add"/> takes the
/// caller's shapes as they are.
/// </para>
/// <para>
/// Writing doesn't change the shapes. The text is OCCT's byte for byte, with <c>'\n'</c> line ends. Reading follows
/// C++'s stream extraction (<see cref="StreamUtilities"/>), and rejects input it can't read safely with
/// <see cref="StreamFormatErrorException"/> (design, section 7): counts the input couldn't hold, words longer than
/// OCCT's buffers, references to shapes or locations that don't exist, shapes nested more than
/// <see cref="MaxShapeNesting"/> deep, and input that ends where OCCT would loop for ever.
/// </para>
/// </remarks>
internal class ShapeSet
{
    // OCCT's version strings, by format number
    internal static readonly string[] THE_ASCII_VERSIONS =
    [
        "",
        "CASCADE Topology V1, (c) Matra-Datavision",
        "CASCADE Topology V2, (c) Matra-Datavision",
        "CASCADE Topology V3, (c) Open Cascade",
    ];

    /// <summary>
    /// The deepest nesting of shapes a set read from a stream may have (a vertex is 1, an edge holding it 2, ...):
    /// writing a shape and exploring it recurse once per level.
    /// </summary>
    public const int MaxShapeNesting = 1000;

    private readonly IndexedSet<Shape> _shapes = new(ShapeHasher.Instance);
    private readonly LocationSet _locations = new();
    private int _formatNb;

    /// <summary>Builds an empty set, which writes the current format.</summary>
    public ShapeSet()
    {
        _formatNb = (int)FormatVersion.FormatVersion_CURRENT;
    }

    /// <summary>
    /// Sets the format version to write (<see cref="FormatVersion"/>); a number outside the supported
    /// versions is ignored, as OCCT's assertion does in a release build.
    /// </summary>
    public void SetFormatNb(int formatNb)
    {
        if (!(formatNb >= (int)FormatVersionBounds.TopTools_FormatVersion_LOWER
              && formatNb <= (int)FormatVersionBounds.TopTools_FormatVersion_UPPER))
        {
            // OCCT: Standard_ASSERT_RETURN, "Error: unsupported TopTools version."
            return;
        }

        _formatNb = formatNb;
    }

    /// <summary>
    /// The format version: the one to write, or, after <see cref="Read(TextReader, ProgressRange)"/>, the one
    /// read.
    /// </summary>
    public int FormatNb() => _formatNb;

    /// <summary>Clears the content of the set.</summary>
    public virtual void Clear()
    {
        _shapes.Clear();
        _locations.Clear();
    }

    /// <summary>
    /// Adds the shape, its sub-shapes (first) and their locations and geometry to the set, and returns its index; 0
    /// for a null shape. A shape is stored once whatever its location and orientation.
    /// </summary>
    /// <exception cref="InsufficientExecutionStackException">The shape is nested too deeply for the stack.</exception>
    public int Add(in Shape s)
    {
        if (s.IsNull)
        {
            return 0;
        }

        // Kerf: Add recurses once per level of nesting; throw rather than overflow the stack
        RuntimeHelpers.EnsureSufficientExecutionStack();

        _locations.Add(s.Placement);
        Shape s2 = s;
        s2.Location(new Placement());
        int index = _shapes.IndexOf(s2) + 1;
        if (index == 0)
        {
            AddGeometry(s2);

            for (Iterator its = new(s2, false, false); its.More(); its.Next())
            {
                Add(its.Value());
            }

            index = _shapes.Add(s2) + 1;
        }

        return index;
    }

    /// <summary>The shape of index I.</summary>
    public Shape Shape(int i) => _shapes[i - 1];

    /// <summary>The index of the shape (its TShape and location), 0 if it isn't in the set.</summary>
    public int Index(in Shape s) => _shapes.IndexOf(s) + 1;

    /// <summary>The locations of the set.</summary>
    public LocationSet Locations() => _locations;

    /// <summary>The locations of the set, to change (OCCT's non-const access).</summary>
    public LocationSet ChangeLocations() => _locations;

    private static void Out(TextWriter os, int value) => StreamUtilities.Write(os, value);

    private static void PrintShapeEnum(ShapeType t, TextWriter s, bool c)
    {
        switch (t)
        {
            case ShapeType.Vertex:
                s.Write(c ? "Ve" : "VERTEX   ");
                break;

            case ShapeType.Edge:
                s.Write(c ? "Ed" : "EDGE     ");
                break;

            case ShapeType.Wire:
                s.Write(c ? "Wi" : "WIRE     ");
                break;

            case ShapeType.Face:
                s.Write(c ? "Fa" : "FACE     ");
                break;

            case ShapeType.Shell:
                s.Write(c ? "Sh" : "SHELL    ");
                break;

            case ShapeType.Solid:
                s.Write(c ? "So" : "SOLID    ");
                break;

            case ShapeType.CompSolid:
                s.Write(c ? "CS" : "COMPSOLID");
                break;

            case ShapeType.Compound:
                s.Write(c ? "Co" : "COMPOUND ");
                break;

            case global::Kerf.Topology.ShapeType.Shape:
                s.Write(c ? "Sp" : "SHAPE");
                break;
        }
    }

    private static void PrintOrientation(Orientation o, TextWriter s, bool c)
    {
        switch (o)
        {
            case Forward:
                s.Write(c ? "+" : "FORWARD");
                break;

            case Reversed:
                s.Write(c ? "-" : "REVERSED");
                break;

            case Internal:
                s.Write(c ? "i" : "INTERNAL");
                break;

            case External:
                s.Write(c ? "e" : "EXTERNAL");
                break;
        }
    }

    // the counts of DumpExtent: VERTEX to COMPOUND, then all the shapes
    private int[] CountTypes()
    {
        int[] counts = new int[9];
        int nbSHAPE = _shapes.Count;
        for (int i = 0; i < nbSHAPE; i++)
        {
            ShapeType type = _shapes[i].Type;
            if (type != global::Kerf.Topology.ShapeType.Shape)
            {
                counts[ShapeType.Vertex - type]++;
            }
        }

        counts[8] = nbSHAPE;
        return counts;
    }

    private static readonly string[] ExtentNames =
    [
        " VERTEX    : ", " EDGE      : ", " WIRE      : ", " FACE      : ", " SHELL     : ", " SOLID     : ",
        " COMPSOLID : ", " COMPOUND  : ", " SHAPE     : ",
    ];

    /// <summary>Writes the number of shapes of each type, and of all of them.</summary>
    public TextWriter DumpExtent(TextWriter os)
    {
        int[] counts = CountTypes();
        for (int i = 0; i < counts.Length; i++)
        {
            os.Write(ExtentNames[i]);
            Out(os, counts[i]);
            os.Write("\n");
        }

        return os;
    }

    /// <summary>Appends the number of shapes of each type, and of all of them, to the string.</summary>
    public void DumpExtent(ref string s)
    {
        int[] counts = CountTypes();
        for (int i = 0; i < counts.Length; i++)
        {
            s = s + ExtentNames[i] + counts[i].ToString(CultureInfo.InvariantCulture) + "\n";
        }
    }

    /// <summary>
    /// Dumps the set in a readable form: the TShapes with their flags and sub-shapes, the geometry and the locations.
    /// </summary>
    /// <remarks>
    /// OCCT writes each TShape's address; the port writes its <see cref="RuntimeHelpers.GetHashCode(object)"/>, in
    /// the same sixteen hexadecimal digits.
    /// </remarks>
    public virtual void Dump(TextWriter os)
    {
        //-----------------------------------------
        // dump the shapes
        //-----------------------------------------

        int i, nbShapes = _shapes.Count;

        os.Write("\nDump of ");
        Out(os, nbShapes);
        os.Write(" TShapes");
        os.Write("\n\n-----------------\n\n");

        os.Write("Flags : Free, Modified, Checked, Orientable, Closed, Infinite, Convex, Locked");
        os.Write("\n\n");

        for (i = nbShapes; i >= 1; i--)
        {
            Shape s = _shapes[i - 1];
            os.Write("TShape # ");
            Out(os, nbShapes - i + 1);
            os.Write(" : ");

            // Type and flags

            PrintShapeEnum(s.Type, os, false);
            os.Write(" ");
            os.Write(s.Free() ? "1" : "0");
            os.Write(s.Modified() ? "1" : "0");
            os.Write(s.Checked() ? "1" : "0");
            os.Write(s.OrientableFlag ? "1" : "0");
            os.Write(s.ClosedFlag ? "1" : "0");
            os.Write(s.InfiniteFlag ? "1" : "0");
            os.Write(s.ConvexFlag ? "1" : "0");
            // Kerf: the stored Locked bit, not Locked(), which is also true of every frozen shape (design 4.3)
            os.Write(s.TShape()!.LockedFlag() ? "1" : "0");
            os.Write(" " + RuntimeHelpers.GetHashCode(s.TShape()).ToString("X16", CultureInfo.InvariantCulture) + "\n");

            // sub-shapes
            os.Write("    ");
            Iterator its = new(s, false, false);
            while (its.More())
            {
                Shape sub = its.Value();
                PrintOrientation(sub.Orientation, os, true);
                Out(os, nbShapes - (_shapes.IndexOf(sub.Located(new Placement())) + 1) + 1);
                if (!sub.Placement.IsIdentity)
                {
                    os.Write("(L");
                    Out(os, _locations.Index(sub.Placement));
                    os.Write(")");
                }

                os.Write(" ");
                its.Next();
            }

            os.Write("\n");

            // Geometry

            DumpGeometry(s, os);
        }

        //-----------------------------------------
        // dump the geometry
        //-----------------------------------------

        DumpGeometry(os);

        //-----------------------------------------
        // dump the locations
        //-----------------------------------------

        _locations.Dump(os);

        os.Write("\n");
    }

    /// <summary>
    /// Writes the set in OCCT's text format, which <see cref="Read(TextReader, ProgressRange)"/> reads: the
    /// version line, the locations, the geometry and the TShapes, with 15 significant digits.
    /// </summary>
    public virtual void Write(TextWriter os, ProgressRange progress = default)
    {
        // always use C locale for writing shapes (the port always does)

        // use 15-digit precision
        int prec = StreamUtilities.Precision(os, 15);

        // write the copyright
        os.Write("\n" + THE_ASCII_VERSIONS[_formatNb] + "\n");

        //-----------------------------------------
        // write the locations
        //-----------------------------------------

        using var ps = new ProgressScope(progress, "Writing", 100);

        _locations.Write(os, ps.Next(10));

        if (ps.UserBreak())
        {
            os.Write("Interrupted by the user\n");
            // OCCT restores the locale here but not the precision
            return;
        }

        //-----------------------------------------
        // write the geometry
        //-----------------------------------------

        WriteGeometry(os, ps.Next(75));

        if (ps.UserBreak())
        {
            os.Write("Interrupted by the user\n");
            return;
        }

        //-----------------------------------------
        // write the shapes
        //-----------------------------------------

        int i, nbShapes = _shapes.Count;

        os.Write("\nTShapes ");
        Out(os, nbShapes);
        os.Write("\n");

        // subshapes are written first
        using (var ps1 = new ProgressScope(ps.Next(15), "Shapes", nbShapes))
        {
            for (i = 1; i <= nbShapes && ps1.More(); i++, ps1.Next())
            {
                Shape s = _shapes[i - 1];

                // Type
                PrintShapeEnum(s.Type, os, true);
                os.Write("\n");

                // Geometry
                WriteGeometry(s, os);

                // Flags
                os.Write("\n");
                os.Write(s.Free() ? "1" : "0");
                os.Write(s.Modified() ? "1" : "0");
                os.Write(s.Checked() ? "1" : "0");
                os.Write(s.OrientableFlag ? "1" : "0");
                os.Write(s.ClosedFlag ? "1" : "0");
                os.Write(s.InfiniteFlag ? "1" : "0");
                os.Write(s.ConvexFlag ? "1" : "0");
                os.Write("\n");

                // sub-shapes

                int l = 0;
                Iterator its = new(s, false, false);
                while (its.More())
                {
                    Write(its.Value(), os);
                    l++;
                    if (l == 10)
                    {
                        os.Write("\n");
                        l = 0;
                    }

                    its.Next();
                }

                Write(new Shape(), os); // Null shape to end the list
                os.Write("\n");
            }
        }

        os.Write("\n");
        StreamUtilities.Precision(os, prec);

        if (ps.UserBreak())
        {
            os.Write("Interrupted by the user\n");
        }
    }

    private static ShapeType ReadShapeEnum(TextReader IS)
    {
        char[] buffer = new char[255];
        StreamUtilities.ReadWord(IS, buffer);

        switch (buffer[0])
        {
            case 'V':
                return ShapeType.Vertex;

            case 'E':
                return ShapeType.Edge;

            case 'W':
                return ShapeType.Wire;

            case 'F':
                return ShapeType.Face;

            case 'S':
                if (buffer[1] == 'h')
                {
                    return ShapeType.Shell;
                }
                else
                {
                    return ShapeType.Solid;
                }

            case 'C':
                if (buffer[1] == 'S')
                {
                    return ShapeType.CompSolid;
                }
                else
                {
                    return ShapeType.Compound;
                }
        }

        return ShapeType.Compound;
    }

    /// <summary>
    /// Clears the set and reads one written by <see cref="Write(TextWriter, ProgressRange)"/>, skipping lines
    /// until a version line. The shapes read are frozen once the whole set is read (or when reading stops on an
    /// error). Writes OCCT's console messages ("File was not written with this version of the topology", "Not a
    /// TShape table") and returns with what it has read when the text isn't a set.
    /// </summary>
    /// <exception cref="StreamFormatErrorException">The input can't be read safely (docs/phase3-design.md, section
    /// 7).</exception>
    /// <exception cref="FailureException">OCCT's own exceptions on malformed input, such as
    /// <see cref="IncompatibleShapesException"/> for a sub-shape of the wrong type.</exception>
    public virtual void Read(TextReader IS, ProgressRange progress = default)
    {
        try
        {
            ReadSet(IS, progress);
        }
        catch (Exception exception) when (IsSignal(exception))
        {
            // Kerf: untrusted input: a malformed stream raised an exception in ported code that OCCT doesn't catch
            // (where OCCT reads outside an array or dereferences a null handle)
            throw new StreamFormatErrorException("TopTools_ShapeSet::Read: " + Refusal.Unwrapped(exception).Message, exception);
        }
        finally
        {
            // design 4.2: the shapes freeze once the whole set is read; on a failure too, since the set hands them out
            for (int i = 0; i < _shapes.Count; i++)
            {
                _shapes[i].TShape()!.FreezeGraph();
            }
        }
    }

    // The .NET exceptions that stand for OCCT's undefined behaviour on malformed input
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

    private void ReadSet(TextReader IS, ProgressRange progress)
    {
        // always use C locale for reading shapes (the port always does)

        Clear();

        // Check the version
        char[] vers = new char[101];
        bool isSetFormat = false;
        do
        {
            StreamUtilities.GetLine(IS, vers, 100, '\n');
            // BUC60769 PTV 18.10.2000: remove possible '\r' at the end of the line

            int lv = StreamUtilities.Strlen(vers);
            if (lv > 0)
            {
                for (lv--; lv > 0 && (vers[lv] == '\r' || vers[lv] == '\n'); lv--)
                {
                    vers[lv] = '\0';
                }
            }

            string aVers = StreamUtilities.CString(vers);
            for (int i = (int)FormatVersionBounds.TopTools_FormatVersion_LOWER;
                 i <= (int)FormatVersionBounds.TopTools_FormatVersion_UPPER;
                 ++i)
            {
                if (aVers == THE_ASCII_VERSIONS[i])
                {
                    SetFormatNb(i);
                    isSetFormat = true;
                    break;
                }
            }

            if (isSetFormat)
            {
                break;
            }
        } while (!StreamUtilities.Fail(IS));

        if (StreamUtilities.Fail(IS))
        {
            Console.Out.Write("File was not written with this version of the topology" + '\n');
            return;
        }

        //-----------------------------------------
        // read the locations
        //-----------------------------------------

        using var ps = new ProgressScope(progress, "Reading", 100);

        _locations.Read(IS, ps.Next(10));

        if (ps.UserBreak())
        {
            Console.Out.Write("Interrupted by the user" + '\n');
            return;
        }

        //-----------------------------------------
        // read the geometry
        //-----------------------------------------
        ReadGeometry(IS, ps.Next(75));

        if (ps.UserBreak())
        {
            Console.Out.Write("Interrupted by the user" + '\n');
            return;
        }

        //-----------------------------------------
        // read the shapes
        //-----------------------------------------

        char[] buffer = new char[255];
        StreamUtilities.ReadWord(IS, buffer);
        if (StreamUtilities.CString(buffer) != "TShapes")
        {
            Console.Out.Write("Not a TShape table" + '\n');
            return;
        }

        // OCCT's nbShapes is uninitialised if the count can't be read
        int nbShapes = 0;
        StreamUtilities.Read(IS, ref nbShapes);
        // Kerf: untrusted input: a shape takes at least 13 characters ("Wi\n\n0000000\n*\n")
        StreamUtilities.CheckCount(IS, Math.Max(nbShapes, 0), 8, "shapes");

        // Kerf: untrusted input: the nesting depth of each shape read, by index
        var depths = new List<int> { 0 };

        // OCC19559
        using var ps1 = new ProgressScope(ps.Next(15), "Shapes", nbShapes);
        for (int i = 1; i <= nbShapes && ps1.More(); i++, ps1.Next())
        {
            Shape s = new();

            // Read type and create empty shape.

            ShapeType t = ReadShapeEnum(IS);
            ReadGeometry(t, IS, ref s);

            // Set the flags
            StreamUtilities.ReadWord(IS, buffer);

            // sub-shapes
            Shape ss;
            int depth = 1;
            do
            {
                Read(out ss, IS, nbShapes);
                // Kerf: untrusted input: OCCT loops for ever on a stream that has failed
                StreamUtilities.CheckNotFailed(IS, "TopTools_ShapeSet::Read");
                if (!ss.IsNull)
                {
                    AddShapes(ref s, ss);
                    // Kerf: untrusted input
                    depth = Math.Max(depth, depths[_shapes.IndexOf(ss.Located(new Placement())) + 1] + 1);
                }
            } while (!ss.IsNull);

            if (depth > MaxShapeNesting)
            {
                // Kerf: untrusted input: writing or exploring the shape would recurse this deep
                throw new StreamFormatErrorException("TopTools_ShapeSet::Read: shapes nested more than "
                                                    + MaxShapeNesting + " deep");
            }

            s.Free(buffer[0] == '1');
            s.Modified(buffer[1] == '1');

            bool isChecked = _formatNb >= (int)FormatVersion.FormatVersion_VERSION_2 && buffer[2] == '1';
            s.Checked(isChecked);

            s.Orientable(buffer[3] == '1');
            s.Closed(buffer[4] == '1');
            s.Infinite(buffer[5] == '1');
            s.Convex(buffer[6] == '1');

            // check

            if (_formatNb == (int)FormatVersion.FormatVersion_VERSION_1)
            {
                Check(t, ref s);
            }

            _shapes.Add(s);
            depths.Add(depth);
        }

        if (ps.UserBreak())
        {
            Console.Out.Write("Interrupted by the user" + '\n');
        }
    }

    /// <summary>
    /// Dumps the index, orientation and location of the shape in the set; writes "Null shape" first for a null shape.
    /// </summary>
    public void Dump(in Shape s, TextWriter os)
    {
        if (s.IsNull)
        {
            os.Write("Null shape\n");
        }

        os.Write("Shape : ");
        Out(os, _shapes.IndexOf(s.Located(new Placement())) + 1);
        os.Write(", ");
        PrintOrientation(s.Orientation, os, false);
        if (!s.Placement.IsIdentity)
        {
            os.Write(", location : ");
            Out(os, _locations.Index(s.Placement));
        }

        os.Write("\n");
    }

    /// <summary>
    /// Writes a reference to the shape, which must be in the set: its orientation, its index counted from the end of
    /// the set and its location's index; <c>*</c> for a null shape.
    /// </summary>
    public void Write(in Shape s, TextWriter os)
    {
        if (s.IsNull)
        {
            os.Write("*");
        }
        else
        {
            PrintOrientation(s.Orientation, os, true);
            Out(os, _shapes.Count - (_shapes.IndexOf(s.Located(new Placement())) + 1) + 1);
            os.Write(" ");
            Out(os, _locations.Index(s.Placement));
            os.Write(" ");
        }
    }

    /// <summary>Reads a reference to a shape of the set, written by <see cref="Write(in Shape, TextWriter)"/>.</summary>
    /// <exception cref="StreamFormatErrorException">The reference isn't to a shape of the set.</exception>
    public void Read(out Shape s, TextReader IS)
    {
        Read(out s, IS, _shapes.Count);
    }

    /// <summary>
    /// Reads a reference to a shape, its index counted from the end of a set of <paramref name="nbshapes"/> shapes
    /// (the set being read); a null shape if <paramref name="nbshapes"/> is less than 1.
    /// </summary>
    /// <exception cref="StreamFormatErrorException">The reference isn't to a shape already read.</exception>
    public void Read(out Shape s, TextReader IS, int nbshapes)
    {
        if (nbshapes < 1)
        {
            s = new Shape();
            return;
        }

        // OCCT's buffer is uninitialised; a failed read leaves it so
        char[] buffer = new char[255];
        StreamUtilities.ReadWord(IS, buffer);
        if (buffer[0] == '*')
        {
            s = new Shape();
        }
        else
        {
            long index = (long)nbshapes - StreamUtilities.Atoi(buffer.AsSpan(1)) + 1;
            // Kerf: untrusted input: OCCT reads outside its map
            if (index < 1 || index > _shapes.Count)
            {
                throw new StreamFormatErrorException("TopTools_ShapeSet::Read: no shape "
                                                    + StreamUtilities.CString(buffer));
            }

            s = _shapes[(int)index - 1];
            switch (buffer[0])
            {
                case '+':
                    s.Orient(Forward);
                    break;

                case '-':
                    s.Orient(Reversed);
                    break;

                case 'i':
                    s.Orient(Internal);
                    break;

                case 'e':
                    s.Orient(External);
                    break;
            }

            // OCCT's l is uninitialised if it can't be read
            int l = 0;
            StreamUtilities.Read(IS, ref l);
            s.Location(_locations.Location(l), false);
        }
    }

    /// <summary>Stores the geometry of the shape. The base class has none.</summary>
    public virtual void AddGeometry(in Shape s)
    {
    }

    /// <summary>Dumps the geometry of the set. The base class has none.</summary>
    public virtual void DumpGeometry(TextWriter os)
    {
    }

    /// <summary>Writes the geometry of the set. The base class has none.</summary>
    public virtual void WriteGeometry(TextWriter os, ProgressRange progress = default)
    {
    }

    /// <summary>Reads the geometry of the set. The base class has none.</summary>
    public virtual void ReadGeometry(TextReader IS, ProgressRange progress = default)
    {
    }

    /// <summary>Dumps the geometry of the shape. The base class has none.</summary>
    public virtual void DumpGeometry(in Shape s, TextWriter os)
    {
    }

    /// <summary>Writes the geometry of the shape. The base class has none.</summary>
    public virtual void WriteGeometry(in Shape s, TextWriter os)
    {
    }

    /// <summary>
    /// Reads the geometry of a shape of type T and creates the shape in S (unfrozen: internal). The base class
    /// creates none.
    /// </summary>
    internal virtual void ReadGeometry(ShapeType t, TextReader IS, ref Shape s)
    {
    }

    /// <summary>Adds S2 to S1, an unfrozen shape being read. The base class does nothing.</summary>
    internal virtual void AddShapes(ref Shape s1, in Shape s2)
    {
    }

    /// <summary>
    /// Completes a shape of type T read from a version 1 file, which doesn't store everything later versions do. The
    /// base class does nothing.
    /// </summary>
    internal virtual void Check(ShapeType t, ref Shape s)
    {
    }

    /// <summary>The number of shapes in the set.</summary>
    public int ShapeCount => _shapes.Count;
}
