// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Text;

namespace Occt;

// BRepTools' Dump, Write and Read: OCCT's text format of shapes (BRepTools_ShapeSet)
public static partial class BRepTools
{
    /// <summary>Dumps the topological structure and the geometry of the shape in a readable form.</summary>
    public static void Dump(in TopoDS_Shape Sh, TextWriter S)
    {
        BRepTools_ShapeSet SS = new();
        SS.Add(Sh);
        SS.Dump(Sh, S);
        SS.Dump(S);
    }

    /// <summary>
    /// Writes the shape to the writer in OCCT's text format, the current version, with its triangulations and
    /// without their normals: the text native <c>BRepTools::Write</c> writes to a stream, byte for byte, with
    /// <c>'\n'</c> line ends.
    /// </summary>
    public static void Write(in TopoDS_Shape theShape, TextWriter theStream, ProgressRange theProgress = default)
    {
        Write(theShape, theStream, true, false, TopTools_FormatVersion.TopTools_FormatVersion_CURRENT, theProgress);
    }

    /// <summary>
    /// Writes the shape to the writer in OCCT's text format, in the version given, with or without its triangulations
    /// and their normals (which a face with a triangulation and no surface always writes).
    /// </summary>
    public static void Write(in TopoDS_Shape theShape,
                             TextWriter theStream,
                             bool theWithTriangles,
                             bool theWithNormals,
                             TopTools_FormatVersion theVersion,
                             ProgressRange theProgress = default)
    {
        BRepTools_ShapeSet aShapeSet = new(theWithTriangles, theWithNormals);
        aShapeSet.SetFormatNb((int)theVersion);
        aShapeSet.Add(theShape);
        aShapeSet.Write(theStream, theProgress);
        aShapeSet.Write(theShape, theStream);
    }

    /// <summary>
    /// Reads a shape written by <see cref="Write(in TopoDS_Shape, TextWriter, ProgressRange)"/> (or native
    /// OCCT) from the reader. The shape is frozen; it is null if the text isn't a shape set.
    /// </summary>
    /// <remarks>
    /// Untrusted input is read safely (docs/phase3-design.md, section 7): counts are checked against the reader's
    /// length, which the port knows for a <see cref="StreamReader"/> over a seekable stream, and for another reader
    /// once given with <see cref="StreamUtilities.SetInputLength"/> (otherwise
    /// <see cref="StreamUtilities.UnknownInputLength"/> is assumed).
    /// </remarks>
    /// <exception cref="StreamFormatErrorException">The input can't be read safely.</exception>
    /// <exception cref="FailureException">OCCT's own exceptions on malformed input.</exception>
    public static void Read(out TopoDS_Shape Sh, TextReader S, ProgressRange theProgress = default)
    {
        Read(out Sh, S, new BRep_Builder(), theProgress);
    }

    /// <summary>Reads a shape from the reader, building it with the builder.</summary>
    internal static void Read(out TopoDS_Shape Sh, TextReader S, BRep_Builder B, ProgressRange theProgress = default)
    {
        BRepTools_ShapeSet SS = new(B);
        SS.Read(S, theProgress);
        ReadShape(SS, out Sh, S);
    }

    // SS.Read(Sh, S), with the exceptions of malformed input reported as the set's Read reports them
    private static void ReadShape(BRepTools_ShapeSet SS, out TopoDS_Shape Sh, TextReader S)
    {
        try
        {
            SS.Read(out Sh, S);
        }
        catch (Exception anException) when (TopTools_ShapeSet.IsSignal(anException))
        {
            // OCCT.NET: untrusted input
            throw new StreamFormatErrorException("BRepTools::Read: " + anException.Message, anException);
        }
    }

    /// <summary>
    /// Writes the shape to a file in OCCT's text format, the current version, with its triangulations: a
    /// <c>DBRep_DrawableShape</c> line, which Draw reads, then the text of
    /// <see cref="Write(in TopoDS_Shape, TextWriter, ProgressRange)"/>. False if the file can't be written.
    /// </summary>
    public static bool Write(in TopoDS_Shape theShape, string theFile, ProgressRange theProgress = default)
    {
        return Write(theShape, theFile, true, false, TopTools_FormatVersion.TopTools_FormatVersion_CURRENT, theProgress);
    }

    /// <summary>
    /// Writes the shape to a file in OCCT's text format, in the version given, with or without its triangulations and
    /// their normals. The file has <c>'\n'</c> line ends, as native OCCT 8 writes it (in binary mode). False if the
    /// file can't be written.
    /// </summary>
    public static bool Write(in TopoDS_Shape theShape,
                             string theFile,
                             bool theWithTriangles,
                             bool theWithNormals,
                             TopTools_FormatVersion theVersion,
                             ProgressRange theProgress = default)
    {
        StreamWriter aStream;
        try
        {
            aStream = new StreamWriter(theFile, false, new UTF8Encoding(false));
        }
        catch (Exception anException) when (anException is IOException or UnauthorizedAccessException
                                                or ArgumentException or NotSupportedException)
        {
            // OCCT: the file system couldn't open the stream
            return false;
        }

        using (aStream)
        {
            try
            {
                BRepTools_ShapeSet SS = new(theWithTriangles, theWithNormals);
                SS.SetFormatNb((int)theVersion);
                SS.Add(theShape);

                aStream.Write("DBRep_DrawableShape\n"); // for easy Draw read
                SS.Write(aStream, theProgress);
                SS.Write(theShape, aStream);
                aStream.Flush();
            }
            catch (IOException)
            {
                // OCCT: the stream isn't good
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Reads a shape from a file in OCCT's text format (lines before the version line, such as Draw's
    /// <c>DBRep_DrawableShape</c>, are skipped). The shape is frozen. False if the file can't be opened or holds no
    /// shape.
    /// </summary>
    /// <exception cref="StreamFormatErrorException">The file can't be read safely.</exception>
    /// <exception cref="FailureException">OCCT's own exceptions on malformed input.</exception>
    public static bool Read(out TopoDS_Shape Sh, string File, ProgressRange theProgress = default)
    {
        return Read(out Sh, File, new BRep_Builder(), theProgress);
    }

    /// <summary>Reads a shape from a file, building it with the builder.</summary>
    internal static bool Read(out TopoDS_Shape Sh, string File, BRep_Builder B, ProgressRange theProgress = default)
    {
        Sh = new TopoDS_Shape();
        StreamReader aStream;
        try
        {
            aStream = new StreamReader(File, Encoding.Latin1, false);
        }
        catch (Exception anException) when (anException is IOException or UnauthorizedAccessException
                                                or ArgumentException or NotSupportedException)
        {
            // OCCT: the file system couldn't open the stream
            return false;
        }

        using (aStream)
        {
            BRepTools_ShapeSet SS = new(B);
            SS.Read(aStream, theProgress);
            if (SS.NbShapes() == 0)
            {
                return false;
            }

            ReadShape(SS, out Sh, aStream);
            return true;
        }
    }
}
