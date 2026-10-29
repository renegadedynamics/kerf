// Port of: src/ModelingData/TKBRep/TopTools/TopTools_ShapeSet.hxx
// Port of: src/ModelingData/TKBRep/TopTools/TopTools_ShapeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Runtime.CompilerServices;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// A set of shapes, written and read in OCCT's text format: the locations, the geometry (in derived classes) and the
/// TShapes, each sub-shape written before the shapes that contain it. The base of
/// <see cref="BRepTools_ShapeSet"/>, which adds the geometry.
/// </summary>
/// <remarks>
/// <para>
/// A working object, as OCCT's: <see cref="Add"/> and <see cref="Read(TextReader, Message_ProgressRange)"/> change the
/// set. The shapes it hands out are frozen: <see cref="Read(TextReader, Message_ProgressRange)"/> freezes every shape
/// it creates once the whole set is read (docs/phase3-design.md, section 4.2), and <see cref="Add"/> takes the
/// caller's shapes as they are.
/// </para>
/// <para>
/// Writing doesn't change the shapes. The text is OCCT's byte for byte, with <c>'\n'</c> line ends. Reading follows
/// C++'s stream extraction (<see cref="Standard_Stream"/>), and rejects input it can't read safely with
/// <see cref="Storage_StreamFormatError"/> (design, section 7): counts the input couldn't hold, words longer than
/// OCCT's buffers, references to shapes or locations that don't exist, shapes nested more than
/// <see cref="MaxShapeNesting"/> deep, and input that ends where OCCT would loop for ever.
/// </para>
/// </remarks>
public class TopTools_ShapeSet
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

    private readonly NCollection_IndexedMap<TopoDS_Shape> myShapes = new(TopTools_ShapeMapHasher.Instance);
    private readonly TopTools_LocationSet myLocations = new();
    private int myFormatNb;

    /// <summary>Builds an empty set, which writes the current format.</summary>
    public TopTools_ShapeSet()
    {
        myFormatNb = (int)TopTools_FormatVersion.TopTools_FormatVersion_CURRENT;
    }

    /// <summary>
    /// Sets the format version to write (<see cref="TopTools_FormatVersion"/>); a number outside the supported
    /// versions is ignored, as OCCT's assertion does in a release build.
    /// </summary>
    public void SetFormatNb(int theFormatNb)
    {
        if (!(theFormatNb >= (int)TopTools_FormatVersionBounds.TopTools_FormatVersion_LOWER
              && theFormatNb <= (int)TopTools_FormatVersionBounds.TopTools_FormatVersion_UPPER))
        {
            // OCCT: Standard_ASSERT_RETURN, "Error: unsupported TopTools version."
            return;
        }

        myFormatNb = theFormatNb;
    }

    /// <summary>
    /// The format version: the one to write, or, after <see cref="Read(TextReader, Message_ProgressRange)"/>, the one
    /// read.
    /// </summary>
    public int FormatNb() => myFormatNb;

    /// <summary>Clears the content of the set.</summary>
    public virtual void Clear()
    {
        myShapes.Clear();
        myLocations.Clear();
    }

    /// <summary>
    /// Adds the shape, its sub-shapes (first) and their locations and geometry to the set, and returns its index; 0
    /// for a null shape. A shape is stored once whatever its location and orientation.
    /// </summary>
    /// <exception cref="InsufficientExecutionStackException">The shape is nested too deeply for the stack.</exception>
    public int Add(in TopoDS_Shape S)
    {
        if (S.IsNull())
        {
            return 0;
        }

        // OCCT.NET: Add recurses once per level of nesting; throw rather than overflow the stack
        RuntimeHelpers.EnsureSufficientExecutionStack();

        myLocations.Add(S.Location());
        TopoDS_Shape S2 = S;
        S2.Location(new TopLoc_Location());
        int index = myShapes.FindIndex(S2);
        if (index == 0)
        {
            AddGeometry(S2);

            for (TopoDS_Iterator its = new(S2, false, false); its.More(); its.Next())
            {
                Add(its.Value());
            }

            index = myShapes.Add(S2);
        }

        return index;
    }

    /// <summary>The shape of index I.</summary>
    public TopoDS_Shape Shape(int I) => myShapes[I];

    /// <summary>The index of the shape (its TShape and location), 0 if it isn't in the set.</summary>
    public int Index(in TopoDS_Shape S) => myShapes.FindIndex(S);

    /// <summary>The locations of the set.</summary>
    public TopTools_LocationSet Locations() => myLocations;

    /// <summary>The locations of the set, to change (OCCT's non-const access).</summary>
    public TopTools_LocationSet ChangeLocations() => myLocations;

    private static void Out(TextWriter OS, int theValue) => Standard_Stream.Write(OS, theValue);

    private static void PrintShapeEnum(TopAbs_ShapeEnum T, TextWriter S, bool C)
    {
        switch (T)
        {
            case TopAbs_VERTEX:
                S.Write(C ? "Ve" : "VERTEX   ");
                break;

            case TopAbs_EDGE:
                S.Write(C ? "Ed" : "EDGE     ");
                break;

            case TopAbs_WIRE:
                S.Write(C ? "Wi" : "WIRE     ");
                break;

            case TopAbs_FACE:
                S.Write(C ? "Fa" : "FACE     ");
                break;

            case TopAbs_SHELL:
                S.Write(C ? "Sh" : "SHELL    ");
                break;

            case TopAbs_SOLID:
                S.Write(C ? "So" : "SOLID    ");
                break;

            case TopAbs_COMPSOLID:
                S.Write(C ? "CS" : "COMPSOLID");
                break;

            case TopAbs_COMPOUND:
                S.Write(C ? "Co" : "COMPOUND ");
                break;

            case TopAbs_SHAPE:
                S.Write(C ? "Sp" : "SHAPE");
                break;
        }
    }

    private static void PrintOrientation(TopAbs_Orientation O, TextWriter S, bool C)
    {
        switch (O)
        {
            case TopAbs_FORWARD:
                S.Write(C ? "+" : "FORWARD");
                break;

            case TopAbs_REVERSED:
                S.Write(C ? "-" : "REVERSED");
                break;

            case TopAbs_INTERNAL:
                S.Write(C ? "i" : "INTERNAL");
                break;

            case TopAbs_EXTERNAL:
                S.Write(C ? "e" : "EXTERNAL");
                break;
        }
    }

    // the counts of DumpExtent: VERTEX to COMPOUND, then all the shapes
    private int[] CountTypes()
    {
        int[] aCounts = new int[9];
        int nbSHAPE = myShapes.Extent();
        for (int i = 1; i <= nbSHAPE; i++)
        {
            TopAbs_ShapeEnum aType = myShapes[i].ShapeType();
            if (aType != TopAbs_SHAPE)
            {
                aCounts[TopAbs_VERTEX - aType]++;
            }
        }

        aCounts[8] = nbSHAPE;
        return aCounts;
    }

    private static readonly string[] theExtentNames =
    [
        " VERTEX    : ", " EDGE      : ", " WIRE      : ", " FACE      : ", " SHELL     : ", " SOLID     : ",
        " COMPSOLID : ", " COMPOUND  : ", " SHAPE     : ",
    ];

    /// <summary>Writes the number of shapes of each type, and of all of them.</summary>
    public TextWriter DumpExtent(TextWriter OS)
    {
        int[] aCounts = CountTypes();
        for (int i = 0; i < aCounts.Length; i++)
        {
            OS.Write(theExtentNames[i]);
            Out(OS, aCounts[i]);
            OS.Write("\n");
        }

        return OS;
    }

    /// <summary>Appends the number of shapes of each type, and of all of them, to the string.</summary>
    public void DumpExtent(ref string S)
    {
        int[] aCounts = CountTypes();
        for (int i = 0; i < aCounts.Length; i++)
        {
            S = S + theExtentNames[i] + aCounts[i].ToString(CultureInfo.InvariantCulture) + "\n";
        }
    }

    /// <summary>
    /// Dumps the set in a readable form: the TShapes with their flags and sub-shapes, the geometry and the locations.
    /// </summary>
    /// <remarks>
    /// OCCT writes each TShape's address; the port writes its <see cref="RuntimeHelpers.GetHashCode(object)"/>, in
    /// the same sixteen hexadecimal digits.
    /// </remarks>
    public virtual void Dump(TextWriter OS)
    {
        //-----------------------------------------
        // dump the shapes
        //-----------------------------------------

        int i, nbShapes = myShapes.Extent();

        OS.Write("\nDump of ");
        Out(OS, nbShapes);
        OS.Write(" TShapes");
        OS.Write("\n\n-----------------\n\n");

        OS.Write("Flags : Free, Modified, Checked, Orientable, Closed, Infinite, Convex, Locked");
        OS.Write("\n\n");

        for (i = nbShapes; i >= 1; i--)
        {
            TopoDS_Shape S = myShapes[i];
            OS.Write("TShape # ");
            Out(OS, nbShapes - i + 1);
            OS.Write(" : ");

            // Type and flags

            PrintShapeEnum(S.ShapeType(), OS, false);
            OS.Write(" ");
            OS.Write(S.Free() ? "1" : "0");
            OS.Write(S.Modified() ? "1" : "0");
            OS.Write(S.Checked() ? "1" : "0");
            OS.Write(S.Orientable() ? "1" : "0");
            OS.Write(S.Closed() ? "1" : "0");
            OS.Write(S.Infinite() ? "1" : "0");
            OS.Write(S.Convex() ? "1" : "0");
            // OCCT.NET: the stored Locked bit, not Locked(), which is also true of every frozen shape (design 4.3)
            OS.Write(S.TShape()!.LockedFlag() ? "1" : "0");
            OS.Write(" " + RuntimeHelpers.GetHashCode(S.TShape()).ToString("X16", CultureInfo.InvariantCulture) + "\n");

            // sub-shapes
            OS.Write("    ");
            TopoDS_Iterator its = new(S, false, false);
            while (its.More())
            {
                TopoDS_Shape sub = its.Value();
                PrintOrientation(sub.Orientation(), OS, true);
                Out(OS, nbShapes - myShapes.FindIndex(sub.Located(new TopLoc_Location())) + 1);
                if (!sub.Location().IsIdentity())
                {
                    OS.Write("(L");
                    Out(OS, myLocations.Index(sub.Location()));
                    OS.Write(")");
                }

                OS.Write(" ");
                its.Next();
            }

            OS.Write("\n");

            // Geometry

            DumpGeometry(S, OS);
        }

        //-----------------------------------------
        // dump the geometry
        //-----------------------------------------

        DumpGeometry(OS);

        //-----------------------------------------
        // dump the locations
        //-----------------------------------------

        myLocations.Dump(OS);

        OS.Write("\n");
    }

    /// <summary>
    /// Writes the set in OCCT's text format, which <see cref="Read(TextReader, Message_ProgressRange)"/> reads: the
    /// version line, the locations, the geometry and the TShapes, with 15 significant digits.
    /// </summary>
    public virtual void Write(TextWriter OS, Message_ProgressRange theProgress = default)
    {
        // always use C locale for writing shapes (the port always does)

        // use 15-digit precision
        int prec = Standard_Stream.Precision(OS, 15);

        // write the copyright
        OS.Write("\n" + THE_ASCII_VERSIONS[myFormatNb] + "\n");

        //-----------------------------------------
        // write the locations
        //-----------------------------------------

        using var aPS = new Message_ProgressScope(theProgress, "Writing", 100);

        myLocations.Write(OS, aPS.Next(10));

        if (aPS.UserBreak())
        {
            OS.Write("Interrupted by the user\n");
            // OCCT restores the locale here but not the precision
            return;
        }

        //-----------------------------------------
        // write the geometry
        //-----------------------------------------

        WriteGeometry(OS, aPS.Next(75));

        if (aPS.UserBreak())
        {
            OS.Write("Interrupted by the user\n");
            return;
        }

        //-----------------------------------------
        // write the shapes
        //-----------------------------------------

        int i, nbShapes = myShapes.Extent();

        OS.Write("\nTShapes ");
        Out(OS, nbShapes);
        OS.Write("\n");

        // subshapes are written first
        using (var aPS1 = new Message_ProgressScope(aPS.Next(15), "Shapes", nbShapes))
        {
            for (i = 1; i <= nbShapes && aPS1.More(); i++, aPS1.Next())
            {
                TopoDS_Shape S = myShapes[i];

                // Type
                PrintShapeEnum(S.ShapeType(), OS, true);
                OS.Write("\n");

                // Geometry
                WriteGeometry(S, OS);

                // Flags
                OS.Write("\n");
                OS.Write(S.Free() ? "1" : "0");
                OS.Write(S.Modified() ? "1" : "0");
                OS.Write(S.Checked() ? "1" : "0");
                OS.Write(S.Orientable() ? "1" : "0");
                OS.Write(S.Closed() ? "1" : "0");
                OS.Write(S.Infinite() ? "1" : "0");
                OS.Write(S.Convex() ? "1" : "0");
                OS.Write("\n");

                // sub-shapes

                int l = 0;
                TopoDS_Iterator its = new(S, false, false);
                while (its.More())
                {
                    Write(its.Value(), OS);
                    l++;
                    if (l == 10)
                    {
                        OS.Write("\n");
                        l = 0;
                    }

                    its.Next();
                }

                Write(new TopoDS_Shape(), OS); // Null shape to end the list
                OS.Write("\n");
            }
        }

        OS.Write("\n");
        Standard_Stream.Precision(OS, prec);

        if (aPS.UserBreak())
        {
            OS.Write("Interrupted by the user\n");
        }
    }

    private static TopAbs_ShapeEnum ReadShapeEnum(TextReader IS)
    {
        char[] buffer = new char[255];
        Standard_Stream.ReadWord(IS, buffer);

        switch (buffer[0])
        {
            case 'V':
                return TopAbs_VERTEX;

            case 'E':
                return TopAbs_EDGE;

            case 'W':
                return TopAbs_WIRE;

            case 'F':
                return TopAbs_FACE;

            case 'S':
                if (buffer[1] == 'h')
                {
                    return TopAbs_SHELL;
                }
                else
                {
                    return TopAbs_SOLID;
                }

            case 'C':
                if (buffer[1] == 'S')
                {
                    return TopAbs_COMPSOLID;
                }
                else
                {
                    return TopAbs_COMPOUND;
                }
        }

        return TopAbs_COMPOUND;
    }

    /// <summary>
    /// Clears the set and reads one written by <see cref="Write(TextWriter, Message_ProgressRange)"/>, skipping lines
    /// until a version line. The shapes read are frozen once the whole set is read (or when reading stops on an
    /// error). Writes OCCT's console messages ("File was not written with this version of the topology", "Not a
    /// TShape table") and returns with what it has read when the text isn't a set.
    /// </summary>
    /// <exception cref="Storage_StreamFormatError">The input can't be read safely (docs/phase3-design.md, section
    /// 7).</exception>
    /// <exception cref="Standard_Failure">OCCT's own exceptions on malformed input, such as
    /// <see cref="TopoDS_UnCompatibleShapes"/> for a sub-shape of the wrong type.</exception>
    public virtual void Read(TextReader IS, Message_ProgressRange theProgress = default)
    {
        try
        {
            ReadSet(IS, theProgress);
        }
        catch (Exception anException) when (IsSignal(anException))
        {
            // OCCT.NET: untrusted input: a malformed stream raised an exception in ported code that OCCT doesn't catch
            // (where OCCT reads outside an array or dereferences a null handle)
            throw new Storage_StreamFormatError("TopTools_ShapeSet::Read: " + anException.Message, anException);
        }
        finally
        {
            // design 4.2: the shapes freeze once the whole set is read; on a failure too, since the set hands them out
            for (int i = 1; i <= myShapes.Extent(); i++)
            {
                myShapes[i].TShape()!.FreezeGraph();
            }
        }
    }

    // The .NET exceptions that stand for OCCT's undefined behaviour on malformed input
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

    private void ReadSet(TextReader IS, Message_ProgressRange theProgress)
    {
        // always use C locale for reading shapes (the port always does)

        Clear();

        // Check the version
        char[] vers = new char[101];
        bool anIsSetFormat = false;
        do
        {
            Standard_Stream.GetLine(IS, vers, 100, '\n');
            // BUC60769 PTV 18.10.2000: remove possible '\r' at the end of the line

            int lv = Standard_Stream.Strlen(vers);
            if (lv > 0)
            {
                for (lv--; lv > 0 && (vers[lv] == '\r' || vers[lv] == '\n'); lv--)
                {
                    vers[lv] = '\0';
                }
            }

            string aVers = Standard_Stream.CString(vers);
            for (int i = (int)TopTools_FormatVersionBounds.TopTools_FormatVersion_LOWER;
                 i <= (int)TopTools_FormatVersionBounds.TopTools_FormatVersion_UPPER;
                 ++i)
            {
                if (aVers == THE_ASCII_VERSIONS[i])
                {
                    SetFormatNb(i);
                    anIsSetFormat = true;
                    break;
                }
            }

            if (anIsSetFormat)
            {
                break;
            }
        } while (!Standard_Stream.Fail(IS));

        if (Standard_Stream.Fail(IS))
        {
            Console.Out.Write("File was not written with this version of the topology" + '\n');
            return;
        }

        //-----------------------------------------
        // read the locations
        //-----------------------------------------

        using var aPS = new Message_ProgressScope(theProgress, "Reading", 100);

        myLocations.Read(IS, aPS.Next(10));

        if (aPS.UserBreak())
        {
            Console.Out.Write("Interrupted by the user" + '\n');
            return;
        }

        //-----------------------------------------
        // read the geometry
        //-----------------------------------------
        ReadGeometry(IS, aPS.Next(75));

        if (aPS.UserBreak())
        {
            Console.Out.Write("Interrupted by the user" + '\n');
            return;
        }

        //-----------------------------------------
        // read the shapes
        //-----------------------------------------

        char[] buffer = new char[255];
        Standard_Stream.ReadWord(IS, buffer);
        if (Standard_Stream.CString(buffer) != "TShapes")
        {
            Console.Out.Write("Not a TShape table" + '\n');
            return;
        }

        // OCCT's nbShapes is uninitialised if the count can't be read
        int nbShapes = 0;
        Standard_Stream.Read(IS, ref nbShapes);
        // OCCT.NET: untrusted input: a shape takes at least 13 characters ("Wi\n\n0000000\n*\n")
        Standard_Stream.CheckCount(IS, Math.Max(nbShapes, 0), 8, "shapes");

        // OCCT.NET: untrusted input: the nesting depth of each shape read, by index
        var aDepths = new List<int> { 0 };

        // OCC19559
        using var aPS1 = new Message_ProgressScope(aPS.Next(15), "Shapes", nbShapes);
        for (int i = 1; i <= nbShapes && aPS1.More(); i++, aPS1.Next())
        {
            TopoDS_Shape S = new();

            // Read type and create empty shape.

            TopAbs_ShapeEnum T = ReadShapeEnum(IS);
            ReadGeometry(T, IS, ref S);

            // Set the flags
            Standard_Stream.ReadWord(IS, buffer);

            // sub-shapes
            TopoDS_Shape SS;
            int aDepth = 1;
            do
            {
                Read(out SS, IS, nbShapes);
                // OCCT.NET: untrusted input: OCCT loops for ever on a stream that has failed
                Standard_Stream.CheckNotFailed(IS, "TopTools_ShapeSet::Read");
                if (!SS.IsNull())
                {
                    AddShapes(ref S, SS);
                    // OCCT.NET: untrusted input
                    aDepth = Math.Max(aDepth, aDepths[myShapes.FindIndex(SS.Located(new TopLoc_Location()))] + 1);
                }
            } while (!SS.IsNull());

            if (aDepth > MaxShapeNesting)
            {
                // OCCT.NET: untrusted input: writing or exploring the shape would recurse this deep
                throw new Storage_StreamFormatError("TopTools_ShapeSet::Read: shapes nested more than "
                                                    + MaxShapeNesting + " deep");
            }

            S.Free(buffer[0] == '1');
            S.Modified(buffer[1] == '1');

            bool isChecked = myFormatNb >= (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_2 && buffer[2] == '1';
            S.Checked(isChecked);

            S.Orientable(buffer[3] == '1');
            S.Closed(buffer[4] == '1');
            S.Infinite(buffer[5] == '1');
            S.Convex(buffer[6] == '1');

            // check

            if (myFormatNb == (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_1)
            {
                Check(T, ref S);
            }

            myShapes.Add(S);
            aDepths.Add(aDepth);
        }

        if (aPS.UserBreak())
        {
            Console.Out.Write("Interrupted by the user" + '\n');
        }
    }

    /// <summary>
    /// Dumps the index, orientation and location of the shape in the set; writes "Null shape" first for a null shape.
    /// </summary>
    public void Dump(in TopoDS_Shape S, TextWriter OS)
    {
        if (S.IsNull())
        {
            OS.Write("Null shape\n");
        }

        OS.Write("Shape : ");
        Out(OS, myShapes.FindIndex(S.Located(new TopLoc_Location())));
        OS.Write(", ");
        PrintOrientation(S.Orientation(), OS, false);
        if (!S.Location().IsIdentity())
        {
            OS.Write(", location : ");
            Out(OS, myLocations.Index(S.Location()));
        }

        OS.Write("\n");
    }

    /// <summary>
    /// Writes a reference to the shape, which must be in the set: its orientation, its index counted from the end of
    /// the set and its location's index; <c>*</c> for a null shape.
    /// </summary>
    public void Write(in TopoDS_Shape S, TextWriter OS)
    {
        if (S.IsNull())
        {
            OS.Write("*");
        }
        else
        {
            PrintOrientation(S.Orientation(), OS, true);
            Out(OS, myShapes.Extent() - myShapes.FindIndex(S.Located(new TopLoc_Location())) + 1);
            OS.Write(" ");
            Out(OS, myLocations.Index(S.Location()));
            OS.Write(" ");
        }
    }

    /// <summary>Reads a reference to a shape of the set, written by <see cref="Write(in TopoDS_Shape, TextWriter)"/>.</summary>
    /// <exception cref="Storage_StreamFormatError">The reference isn't to a shape of the set.</exception>
    public void Read(out TopoDS_Shape S, TextReader IS)
    {
        Read(out S, IS, myShapes.Extent());
    }

    /// <summary>
    /// Reads a reference to a shape, its index counted from the end of a set of <paramref name="nbshapes"/> shapes
    /// (the set being read); a null shape if <paramref name="nbshapes"/> is less than 1.
    /// </summary>
    /// <exception cref="Storage_StreamFormatError">The reference isn't to a shape already read.</exception>
    public void Read(out TopoDS_Shape S, TextReader IS, int nbshapes)
    {
        if (nbshapes < 1)
        {
            S = new TopoDS_Shape();
            return;
        }

        // OCCT's buffer is uninitialised; a failed read leaves it so
        char[] buffer = new char[255];
        Standard_Stream.ReadWord(IS, buffer);
        if (buffer[0] == '*')
        {
            S = new TopoDS_Shape();
        }
        else
        {
            long anIndex = (long)nbshapes - Standard_Stream.Atoi(buffer.AsSpan(1)) + 1;
            // OCCT.NET: untrusted input: OCCT reads outside its map
            if (anIndex < 1 || anIndex > myShapes.Extent())
            {
                throw new Storage_StreamFormatError("TopTools_ShapeSet::Read: no shape "
                                                    + Standard_Stream.CString(buffer));
            }

            S = myShapes[(int)anIndex];
            switch (buffer[0])
            {
                case '+':
                    S.Orientation(TopAbs_FORWARD);
                    break;

                case '-':
                    S.Orientation(TopAbs_REVERSED);
                    break;

                case 'i':
                    S.Orientation(TopAbs_INTERNAL);
                    break;

                case 'e':
                    S.Orientation(TopAbs_EXTERNAL);
                    break;
            }

            // OCCT's l is uninitialised if it can't be read
            int l = 0;
            Standard_Stream.Read(IS, ref l);
            S.Location(myLocations.Location(l), false);
        }
    }

    /// <summary>Stores the geometry of the shape. The base class has none.</summary>
    public virtual void AddGeometry(in TopoDS_Shape S)
    {
    }

    /// <summary>Dumps the geometry of the set. The base class has none.</summary>
    public virtual void DumpGeometry(TextWriter OS)
    {
    }

    /// <summary>Writes the geometry of the set. The base class has none.</summary>
    public virtual void WriteGeometry(TextWriter OS, Message_ProgressRange theProgress = default)
    {
    }

    /// <summary>Reads the geometry of the set. The base class has none.</summary>
    public virtual void ReadGeometry(TextReader IS, Message_ProgressRange theProgress = default)
    {
    }

    /// <summary>Dumps the geometry of the shape. The base class has none.</summary>
    public virtual void DumpGeometry(in TopoDS_Shape S, TextWriter OS)
    {
    }

    /// <summary>Writes the geometry of the shape. The base class has none.</summary>
    public virtual void WriteGeometry(in TopoDS_Shape S, TextWriter OS)
    {
    }

    /// <summary>
    /// Reads the geometry of a shape of type T and creates the shape in S (unfrozen: internal). The base class
    /// creates none.
    /// </summary>
    internal virtual void ReadGeometry(TopAbs_ShapeEnum T, TextReader IS, ref TopoDS_Shape S)
    {
    }

    /// <summary>Adds S2 to S1, an unfrozen shape being read. The base class does nothing.</summary>
    internal virtual void AddShapes(ref TopoDS_Shape S1, in TopoDS_Shape S2)
    {
    }

    /// <summary>
    /// Completes a shape of type T read from a version 1 file, which doesn't store everything later versions do. The
    /// base class does nothing.
    /// </summary>
    internal virtual void Check(TopAbs_ShapeEnum T, ref TopoDS_Shape S)
    {
    }

    /// <summary>The number of shapes in the set.</summary>
    public int NbShapes() => myShapes.Extent();
}
