// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Engine;
using System.Text;

namespace Kerf.Engine.Topology.Tools;

// BRepTools' Dump, Write and Read: OCCT's text format of shapes (BRepTools_ShapeSet)
internal static partial class BrepTools
{
    /// <summary>Dumps the topological structure and the geometry of the shape in a readable form.</summary>
    public static void Dump(in Shape sh, TextWriter s)
    {
        BrepShapeSet ss = new();
        ss.Add(sh);
        ss.Dump(sh, s);
        ss.Dump(s);
    }

    /// <summary>
    /// Writes the shape to the writer in OCCT's text format, the current version, with its triangulations and
    /// without their normals: the text native <c>BRepTools::Write</c> writes to a stream, byte for byte, with
    /// <c>'\n'</c> line ends.
    /// </summary>
    public static void Write(in Shape shape, TextWriter stream, ProgressRange progress = default)
    {
        Write(shape, stream, true, false, FormatVersion.FormatVersion_CURRENT, progress);
    }

    /// <summary>
    /// Writes the shape to the writer in OCCT's text format, in the version given, with or without its triangulations
    /// and their normals (which a face with a triangulation and no surface always writes).
    /// </summary>
    public static void Write(in Shape shape,
                             TextWriter stream,
                             bool withTriangles,
                             bool withNormals,
                             FormatVersion version,
                             ProgressRange progress = default)
    {
        BrepShapeSet shapeSet = new(withTriangles, withNormals);
        shapeSet.SetFormatNb((int)version);
        shapeSet.Add(shape);
        shapeSet.Write(stream, progress);
        shapeSet.Write(shape, stream);
    }

    /// <summary>
    /// Reads a shape written by <see cref="Write(in Shape, TextWriter, ProgressRange)"/> (or native
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
    public static void Read(out Shape sh, TextReader s, ProgressRange progress = default)
    {
        Read(out sh, s, new BrepBuilder(), progress);
    }

    /// <summary>Reads a shape from the reader, building it with the builder.</summary>
    internal static void Read(out Shape sh, TextReader s, BrepBuilder b, ProgressRange progress = default)
    {
        BrepShapeSet ss = new(b);
        ss.Read(s, progress);
        ReadShape(ss, out sh, s);
    }

    // SS.Read(Sh, S), with the exceptions of malformed input reported as the set's Read reports them
    private static void ReadShape(BrepShapeSet ss, out Shape sh, TextReader s)
    {
        try
        {
            ss.Read(out sh, s);
        }
        catch (Exception exception) when (ShapeSet.IsSignal(exception))
        {
            // Kerf: untrusted input
            throw new StreamFormatErrorException("BRepTools::Read: " + Refusal.Unwrapped(exception).Message, exception);
        }
    }

    /// <summary>
    /// Writes the shape to a file in OCCT's text format, the current version, with its triangulations: a
    /// <c>DBRep_DrawableShape</c> line, which Draw reads, then the text of
    /// <see cref="Write(in Shape, TextWriter, ProgressRange)"/>. False if the file can't be written.
    /// </summary>
    public static bool Write(in Shape shape, string file, ProgressRange progress = default)
    {
        return Write(shape, file, true, false, FormatVersion.FormatVersion_CURRENT, progress);
    }

    /// <summary>
    /// Writes the shape to a file in OCCT's text format, in the version given, with or without its triangulations and
    /// their normals. The file has <c>'\n'</c> line ends, as native OCCT 8 writes it (in binary mode). False if the
    /// file can't be written.
    /// </summary>
    public static bool Write(in Shape shape,
                             string file,
                             bool withTriangles,
                             bool withNormals,
                             FormatVersion version,
                             ProgressRange progress = default)
    {
        StreamWriter stream;
        try
        {
            stream = new StreamWriter(file, false, new UTF8Encoding(false));
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException
                                                or ArgumentException or NotSupportedException)
        {
            // OCCT: the file system couldn't open the stream
            return false;
        }

        using (stream)
        {
            try
            {
                BrepShapeSet ss = new(withTriangles, withNormals);
                ss.SetFormatNb((int)version);
                ss.Add(shape);

                stream.Write("DBRep_DrawableShape\n"); // for easy Draw read
                ss.Write(stream, progress);
                ss.Write(shape, stream);
                stream.Flush();
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
    public static bool Read(out Shape sh, string file, ProgressRange progress = default)
    {
        return Read(out sh, file, new BrepBuilder(), progress);
    }

    /// <summary>Reads a shape from a file, building it with the builder.</summary>
    internal static bool Read(out Shape sh, string file, BrepBuilder b, ProgressRange progress = default)
    {
        sh = new Shape();
        StreamReader stream;
        try
        {
            stream = new StreamReader(file, Encoding.Latin1, false);
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException
                                                or ArgumentException or NotSupportedException)
        {
            // OCCT: the file system couldn't open the stream
            return false;
        }

        using (stream)
        {
            BrepShapeSet ss = new(b);
            ss.Read(stream, progress);
            if (ss.ShapeCount == 0)
            {
                return false;
            }

            ReadShape(ss, out sh, stream);
            return true;
        }
    }
}
