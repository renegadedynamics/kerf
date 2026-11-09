// Port of: src/DataExchange/TKDESTL/RWStl/RWStl.hxx
// Port of: src/DataExchange/TKDESTL/RWStl/RWStl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using System.Buffers.Binary;
using System.Text;

namespace Kerf.Engine.Exchange.Stl;

/// <summary>Reads and writes triangulations from and to STL, binary or ASCII.</summary>
/// <remarks>
/// <para>
/// Streams first (phase8-plan.md 2.7): every reader and writer takes a .NET <see cref="Stream"/>, and the file
/// paths (OCCT's <c>OSD_Path</c> and <c>const char*</c>) open a file and call them. ASCII is written as bytes, LF line
/// ends; OCCT's file writers open the file in text mode, which writes CR LF on Windows only (decision 1168). The
/// <c>ReadFile</c> forms (OCCT's for files) also take streams: they recognise the format from the content and read
/// multi-domain data; <see cref="ReadStream"/> recognises the format as they do, and reads one domain.
/// </para>
/// <para>
/// The triangulations read are published frozen (copy one to change it). <c>ReadFile</c> returns what was read
/// before an error, as OCCT does, but nothing after a user break or past a limit (<see cref="StlReadLimits"/>),
/// where OCCT returns the part read (decision 1169). The writers only read the mesh. A write error of the stream
/// (an <see cref="IOException"/>) returns false, as a C++ stream's fail state does.
/// </para>
/// </remarks>
internal static class RWStl
{
    private const int THE_STL_SIZEOF_FACET = 50;
    private const int IND_THRESHOLD = 1000; // increment the indicator every 1k triangles
    private const int THE_BUFFER_SIZE = 1024; // The length of buffer to read (in bytes)

    // OCCT's binary header: the text, zero-filled to 80 bytes
    private const string THE_HEADER = "STL Exported by Kerf";

    //! Writing a Little Endian 32 bits integer
    private static void ConvertInteger(int value, Span<byte> result) => BinaryPrimitives.WriteInt32LittleEndian(result, value);

    //! Writing a Little Endian 32 bits float
    private static void ConvertDouble(double value, Span<byte> result) => BinaryPrimitives.WriteSingleLittleEndian(result, (float)value);

    // RWStl.cxx's Reader: collects the nodes and triangles into a triangulation
    private class Reader : global::Kerf.Engine.Exchange.Stl.Reader
    {
        private readonly List<Xyz> _nodes = new();
        private readonly List<Triangle1> _triangles = new();

        //! Add new node
        public override int AddNode(in Xyz pnt)
        {
            _nodes.Add(pnt);
            return _nodes.Count;
        }

        //! Add new triangle
        public override void AddTriangle(int node1, int node2, int node3)
        {
            _triangles.Add(new Triangle1(node1, node2, node3));
        }

        //! Creates Poly_Triangulation from collected data
        public Triangulation? GetTriangulation()
        {
            if (_triangles.Count == 0)
            {
                return null;
            }

            var poly = new Triangulation(_nodes.Count, _triangles.Count, false);
            for (int nodeIter = 0; nodeIter < _nodes.Count; ++nodeIter)
            {
                poly.SetNode(nodeIter + 1, new Point3D(_nodes[nodeIter]));
            }

            for (int triIter = 0; triIter < _triangles.Count; ++triIter)
            {
                poly.SetTriangle(triIter + 1, _triangles[triIter]);
            }

            return poly;
        }

        protected void Clear()
        {
            _nodes.Clear();
            _triangles.Clear();
        }
    }

    private sealed class MultiDomainReader : Reader
    {
        //! Add new solid
        //! Add triangulation to triangulation list for multi-domain case
        public override void AddSolid()
        {
            if (GetTriangulation() is Triangulation currentTri)
            {
                _triangulationList.Append(currentTri);
            }
            Clear();
        }

        //! Returns triangulation list for multi-domain case
        public NCollection_Sequence<Triangulation> ChangeTriangulationList() => _triangulationList;

        private readonly NCollection_Sequence<Triangulation> _triangulationList = new();
    }

    // The port publishes the triangulations it reads frozen
    private static Triangulation? Published(Triangulation? triangulation)
    {
        triangulation?.Freeze();
        return triangulation;
    }

    // The port's: no result after a user break or past a limit (decision 1169)
    private static bool Stopped(global::Kerf.Engine.Exchange.Stl.Reader reader, in ProgressRange progress)
        => reader.LimitExceeded() || progress.UserBreak();

    /// <summary>
    /// Reads an STL file (format recognised from its content; the domains of a multi-domain file make one
    /// triangulation), merging nodes regardless of the angle. Null on an error with nothing read.
    /// </summary>
    public static Triangulation? ReadFile(string file, ProgressRange progress = default, StlReadLimits? limits = null)
        => ReadFile(file, Math.PI / 2.0, progress, limits);

    /// <summary>
    /// Reads an STL file, merging nodes at the same point whose triangles make an angle below
    /// <paramref name="mergeAngle"/> (radians; 90 degrees merges regardless of the angle). Null on an error with
    /// nothing read.
    /// </summary>
    public static Triangulation? ReadFile(string file, double mergeAngle, ProgressRange progress = default, StlReadLimits? limits = null)
    {
        var reader = new Reader();
        reader.SetMergeAngle(mergeAngle);
        reader.SetLimits(limits ?? StlReadLimits.None);
        reader.Read(file, progress);
        // note that returned bool value is ignored intentionally -- even if something went wrong,
        // but some data have been read, we at least will return these data
        return Stopped(reader, progress) ? null : Published(reader.GetTriangulation());
    }

    /// <summary>
    /// <see cref="ReadFile(string, double, ProgressRange, StlReadLimits)"/> on the data of a stream from
    /// its position (an addition).
    /// </summary>
    public static Triangulation? ReadFile(Stream stream, double mergeAngle = Math.PI / 2.0, ProgressRange progress = default, StlReadLimits? limits = null)
    {
        var reader = new Reader();
        reader.SetMergeAngle(mergeAngle);
        reader.SetLimits(limits ?? StlReadLimits.None);
        reader.Read(stream, progress);
        return Stopped(reader, progress) ? null : Published(reader.GetTriangulation());
    }

    /// <summary>
    /// Reads an STL file into one triangulation per domain (solid) of a multi-domain file, into
    /// <paramref name="triangList"/> (cleared first). Empty after a user break or past a limit.
    /// </summary>
    public static void ReadFile(string file, double mergeAngle, NCollection_Sequence<Triangulation> triangList,
                                ProgressRange progress = default, StlReadLimits? limits = null)
    {
        var reader = new MultiDomainReader();
        reader.SetMergeAngle(mergeAngle);
        reader.SetLimits(limits ?? StlReadLimits.None);
        reader.Read(file, progress);
        TakeDomains(reader, triangList, progress);
    }

    /// <summary>
    /// <see cref="ReadFile(string, double, NCollection_Sequence{Triangulation}, ProgressRange, StlReadLimits)"/>
    /// on the data of a stream from its position (an addition).
    /// </summary>
    public static void ReadFile(Stream stream, double mergeAngle, NCollection_Sequence<Triangulation> triangList,
                                ProgressRange progress = default, StlReadLimits? limits = null)
    {
        var reader = new MultiDomainReader();
        reader.SetMergeAngle(mergeAngle);
        reader.SetLimits(limits ?? StlReadLimits.None);
        reader.Read(stream, progress);
        TakeDomains(reader, triangList, progress);
    }

    private static void TakeDomains(MultiDomainReader reader, NCollection_Sequence<Triangulation> triangList, in ProgressRange progress)
    {
        triangList.Clear();
        if (Stopped(reader, progress))
        {
            return;
        }
        NCollection_Sequence<Triangulation> list = reader.ChangeTriangulationList();
        for (int i = 1; i <= list.Length(); ++i)
        {
            triangList.Append(Published(list.Value(i))!);
        }
    }

    /// <summary>Reads a binary STL file (one domain). Null on an error or a user break.</summary>
    public static Triangulation? ReadBinary(string path, ProgressRange progress = default, StlReadLimits? limits = null)
    {
        using FileStream? stream = global::Kerf.Engine.Exchange.Stl.Reader.OpenFile(path);
        if (stream is null)
        {
            return null;
        }
        // OCCT's ReadBinary(OSD_Path) leaves the merge angle at its default
        return ReadBinaryStream(stream, Math.PI / 2.0, progress, limits);
    }

    /// <summary>Reads an ASCII STL file (one domain). Null on an error or a user break.</summary>
    public static Triangulation? ReadAscii(string path, ProgressRange progress = default, StlReadLimits? limits = null)
    {
        using FileStream? stream = global::Kerf.Engine.Exchange.Stl.Reader.OpenFile(path);
        if (stream is null)
        {
            return null;
        }
        return ReadAsciiStream(stream, Math.PI / 2.0, progress, limits);
    }

    /// <summary>Reads binary STL from a stream (one domain). Null on an error or a user break.</summary>
    public static Triangulation? ReadBinaryStream(Stream stream, double mergeAngle = Math.PI / 2.0,
                                                       ProgressRange progress = default, StlReadLimits? limits = null)
    {
        var reader = new Reader();
        reader.SetMergeAngle(mergeAngle);
        reader.SetLimits(limits ?? StlReadLimits.None);
        InputStream? aStream = reader.OpenStream(stream);
        if (aStream is null || !reader.ReadBinary(aStream, progress))
        {
            return null;
        }
        return Published(reader.GetTriangulation());
    }

    /// <summary>Reads ASCII STL from a stream (one domain). Null on an error or a user break.</summary>
    public static Triangulation? ReadAsciiStream(Stream stream, double mergeAngle = Math.PI / 2.0,
                                                      ProgressRange progress = default, StlReadLimits? limits = null)
    {
        var reader = new Reader();
        reader.SetMergeAngle(mergeAngle);
        reader.SetLimits(limits ?? StlReadLimits.None);
        InputStream? aStream = reader.OpenStream(stream);
        if (aStream is null)
        {
            return null;
        }

        // get length of stream to feed progress indicator
        aStream.SeekGEnd();
        long theEnd = aStream.TellG();
        aStream.SeekG(0);

        var buffer = new ReadLineBuffer(THE_BUFFER_SIZE);
        if (!reader.ReadAscii(aStream, buffer, theEnd, progress))
        {
            return null;
        }
        return Published(reader.GetTriangulation());
    }

    /// <summary>
    /// Reads STL from a stream, ASCII or binary as <see cref="ReadFile(Stream, double, ProgressRange, StlReadLimits)"/>
    /// recognises it: ASCII if it is shorter than a binary file of one facet or its first 134 bytes are all at most
    /// <c>~</c>. One domain. Null on an error or a user break.
    /// </summary>
    public static Triangulation? ReadStream(Stream stream, double mergeAngle = Math.PI / 2.0,
                                                 ProgressRange progress = default, StlReadLimits? limits = null)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        if (!stream.CanSeek)
        {
            // the port: the peek below needs to seek back (ReadAsciiStream and ReadBinaryStream copy such a stream
            // into memory anyway)
            var reader = new Reader();
            reader.SetLimits(limits ?? StlReadLimits.None);
            InputStream? copy = reader.OpenStream(stream);
            if (copy is null)
            {
                return null;
            }
            stream = copy.BaseStream;
        }

        // ASCII or binary as RWStl_Reader::Read recognises them (OCCT tested for a leading "solid", which a binary
        // header can start with too, and is case-sensitive; Kerf decision 1498)
        long originalPos = stream.Position;
        Span<byte> header = stackalloc byte[134];
        int nbRead = stream.ReadAtLeast(header, header.Length, throwOnEndOfStream: false);
        stream.Position = originalPos;

        bool isAscii = nbRead < header.Length || header.IndexOfAnyInRange((byte)('~' + 1), byte.MaxValue) < 0;

        if (isAscii)
        {
            return ReadAsciiStream(stream, mergeAngle, progress, limits);
        }
        else
        {
            return ReadBinaryStream(stream, mergeAngle, progress, limits);
        }
    }

    /// <summary>Writes the triangulation to a binary STL file. False if it is null or empty, or on an error.</summary>
    public static bool WriteBinary(Triangulation? mesh, string path, ProgressRange progress = default)
    {
        if (mesh is null || mesh.TriangleCount <= 0)
        {
            return false;
        }

        FileStream? stream = CreateFile(path);
        if (stream is null)
        {
            return false;
        }

        using (stream)
        {
            return WriteBinary(mesh, stream, progress);
        }
    }

    /// <summary>Writes the triangulation to an ASCII STL file. False if it is null or empty, or on an error.</summary>
    public static bool WriteAscii(Triangulation? mesh, string path, ProgressRange progress = default)
    {
        if (mesh is null || mesh.TriangleCount <= 0)
        {
            return false;
        }

        FileStream? stream = CreateFile(path);
        if (stream is null)
        {
            return false;
        }

        using (stream)
        {
            return WriteAscii(mesh, stream, progress);
        }
    }

    /// <summary>Writes the triangulation as ASCII STL to a stream (bytes, LF line ends).</summary>
    public static bool WriteAscii(Triangulation? mesh, Stream stream, ProgressRange progress = default)
    {
        if (mesh is null || mesh.TriangleCount <= 0)
        {
            return false;
        }

        try
        {
            using var writer = new StreamWriter(stream, new UTF8Encoding(false), 1 << 16, leaveOpen: true);
            bool isDone = WriteAscii(mesh, writer, progress);
            writer.Flush();
            return isDone;
        }
        catch (IOException)
        {
            return false;
        }
    }

    /// <summary>Writes the triangulation as ASCII STL to a text writer.</summary>
    public static bool WriteAscii(Triangulation? mesh, TextWriter stream, ProgressRange progress = default)
    {
        if (mesh is null || mesh.TriangleCount <= 0)
        {
            return false;
        }

        try
        {
            // note that space after 'solid' is necessary for many systems
            stream.Write("solid \n");

            int nbTriangles = mesh.TriangleCount;
            using var ps = new ProgressScope(progress, "Triangles", nbTriangles);

            for (int triIter = 1; triIter <= nbTriangles; ++triIter)
            {
                Triangle1 triangle = mesh.Triangle(triIter);
                triangle.Get(out int elem0, out int elem1, out int elem2);
                Point3D p1 = mesh.Node(elem0);
                Point3D p2 = mesh.Node(elem1);
                Point3D p3 = mesh.Node(elem2);
                var vec1 = new Vector3D(p1, p2);
                var vec2 = new Vector3D(p1, p3);
                Vector3D vNorm = vec1.Crossed(vec2);
                if (vNorm.SquareMagnitude > Constants.Resolution())
                {
                    vNorm = vNorm.Normalized();
                }
                else
                {
                    vNorm = vNorm.WithCoord(0.0, 0.0, 0.0);
                }

                // std::scientific << std::setprecision(12)
                stream.Write(" facet normal ");
                stream.Write(E12(vNorm.X));
                stream.Write(' ');
                stream.Write(E12(vNorm.Y));
                stream.Write(' ');
                stream.Write(E12(vNorm.Z));
                stream.Write("\n   outer loop\n");
                WriteVertex(stream, p1);
                WriteVertex(stream, p2);
                WriteVertex(stream, p3);
                stream.Write("   endloop\n endfacet\n");

                // update progress only per 1k triangles
                if ((triIter % IND_THRESHOLD) == 0)
                {
                    if (!ps.More())
                    {
                        return false;
                    }
                    ps.Next(IND_THRESHOLD);
                }
            }

            stream.Write("endsolid\n");
            return true;
        }
        catch (IOException)
        {
            return false;
        }
    }

    private static string E12(double value) => StreamUtilities.FormatScientific(value, 12);

    private static void WriteVertex(TextWriter stream, in Point3D p)
    {
        stream.Write("     vertex ");
        stream.Write(E12(p.X));
        stream.Write(' ');
        stream.Write(E12(p.Y));
        stream.Write(' ');
        stream.Write(E12(p.Z));
        stream.Write('\n');
    }

    /// <summary>Writes the triangulation as binary STL to a stream (OCCT's 80-byte header, then the facets).</summary>
    public static bool WriteBinary(Triangulation? mesh, Stream stream, ProgressRange progress = default)
    {
        if (mesh is null || mesh.TriangleCount <= 0)
        {
            return false;
        }

        try
        {
            Span<byte> header = stackalloc byte[80];
            header.Clear();
            Encoding.ASCII.GetBytes(THE_HEADER, header);
            stream.Write(header);

            int nbTriangles = mesh.TriangleCount;
            using var ps = new ProgressScope(progress, "Triangles", nbTriangles);
            const int nbChunkTriangles = 4096;
            const int chunkSize = nbChunkTriangles * THE_STL_SIZEOF_FACET;
            byte[] data = new byte[chunkSize];
            Span<byte> dataChunk = data;
            Span<byte> conv = stackalloc byte[4];
            ConvertInteger(nbTriangles, conv);
            stream.Write(conv);

            int byteCount = 0;
            for (int triIter = 1; triIter <= nbTriangles; ++triIter)
            {
                Triangle1 triangle = mesh.Triangle(triIter);
                triangle.Get(out int id0, out int id1, out int id2);
                Point3D p1 = mesh.Node(id0);
                Point3D p2 = mesh.Node(id1);
                Point3D p3 = mesh.Node(id2);
                var vec1 = new Vector3D(p1, p2);
                var vec2 = new Vector3D(p1, p3);
                Vector3D vNorm = vec1.Crossed(vec2);
                if (vNorm.SquareMagnitude > Constants.Resolution())
                {
                    vNorm = vNorm.Normalized();
                }
                else
                {
                    vNorm = vNorm.WithCoord(0.0, 0.0, 0.0);
                }
                ConvertDouble(vNorm.X, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(vNorm.Y, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(vNorm.Z, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(p1.X, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(p1.Y, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(p1.Z, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(p2.X, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(p2.Y, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(p2.Z, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(p3.X, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(p3.Y, dataChunk[byteCount..]);
                byteCount += 4;
                ConvertDouble(p3.Z, dataChunk[byteCount..]);
                byteCount += 4;
                dataChunk[byteCount] = 0;
                dataChunk[byteCount + 1] = 0;
                byteCount += 2;

                // Chunk is full, write it out.
                if (byteCount == chunkSize)
                {
                    stream.Write(data, 0, chunkSize);
                    byteCount = 0;
                }

                // update progress only per 1k triangles
                if ((triIter % IND_THRESHOLD) == 0)
                {
                    if (!ps.More())
                    {
                        return false;
                    }
                    ps.Next(IND_THRESHOLD);
                }
            }

            // Write last part if necessary.
            if (byteCount != chunkSize)
            {
                stream.Write(data, 0, byteCount);
            }

            return true;
        }
        catch (IOException)
        {
            return false;
        }
    }

    // OCCT's std::ofstream: false (null) if the file can't be created
    internal static FileStream? CreateFile(string path)
    {
        try
        {
            return new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 16);
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException or ArgumentException or NotSupportedException)
        {
            return null;
        }
    }
}
