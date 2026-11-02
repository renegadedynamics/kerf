// Port of: src/DataExchange/TKDESTL/RWStl/RWStl.hxx
// Port of: src/DataExchange/TKDESTL/RWStl/RWStl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Buffers.Binary;
using System.Text;

namespace Occt;

/// <summary>Reads and writes triangulations from and to STL, binary or ASCII.</summary>
/// <remarks>
/// <para>
/// Streams first (phase8-plan.md 2.7): every reader and writer takes a .NET <see cref="Stream"/>, and the file
/// paths (OCCT's <c>OSD_Path</c> and <c>const char*</c>) open a file and call them. ASCII is written as bytes, LF line
/// ends; OCCT's file writers open the file in text mode, which writes CR LF on Windows only (decision 12202). The
/// <c>ReadFile</c> forms (OCCT's for files) also take streams: they recognise the format from the content and read
/// multi-domain data, where <see cref="ReadStream"/> (as OCCT's) only tests for a leading "solid".
/// </para>
/// <para>
/// The triangulations read are published frozen (copy one to change it). <c>ReadFile</c> returns what was read
/// before an error, as OCCT does, but nothing after a user break or past a limit (<see cref="RWStl_ReadLimits"/>),
/// where OCCT returns the part read (decision 12203). The writers only read the mesh. A write error of the stream
/// (an <see cref="IOException"/>) returns false, as a C++ stream's fail state does.
/// </para>
/// </remarks>
public static class RWStl
{
    private const int THE_STL_SIZEOF_FACET = 50;
    private const int IND_THRESHOLD = 1000; // increment the indicator every 1k triangles
    private const int THE_BUFFER_SIZE = 1024; // The length of buffer to read (in bytes)

    // OCCT's binary header: the text, zero-filled to 80 bytes
    private const string THE_HEADER = "STL Exported by Open CASCADE Technology [dev.opencascade.org]";

    //! Writing a Little Endian 32 bits integer
    private static void convertInteger(int theValue, Span<byte> theResult) => BinaryPrimitives.WriteInt32LittleEndian(theResult, theValue);

    //! Writing a Little Endian 32 bits float
    private static void convertDouble(double theValue, Span<byte> theResult) => BinaryPrimitives.WriteSingleLittleEndian(theResult, (float)theValue);

    // RWStl.cxx's Reader: collects the nodes and triangles into a triangulation
    private class Reader : RWStl_Reader
    {
        private readonly NCollection_DynamicArray<gp_XYZ> myNodes = new();
        private readonly NCollection_DynamicArray<Poly_Triangle> myTriangles = new();

        //! Add new node
        public override int AddNode(in gp_XYZ thePnt)
        {
            myNodes.Append(thePnt);
            return myNodes.Length();
        }

        //! Add new triangle
        public override void AddTriangle(int theNode1, int theNode2, int theNode3)
        {
            myTriangles.Append(new Poly_Triangle(theNode1, theNode2, theNode3));
        }

        //! Creates Poly_Triangulation from collected data
        public Poly_Triangulation? GetTriangulation()
        {
            if (myTriangles.IsEmpty())
            {
                return null;
            }

            var aPoly = new Poly_Triangulation(myNodes.Length(), myTriangles.Length(), false);
            for (int aNodeIter = 0; aNodeIter < myNodes.Length(); ++aNodeIter)
            {
                aPoly.SetNode(aNodeIter + 1, new gp_Pnt(myNodes[aNodeIter]));
            }

            for (int aTriIter = 0; aTriIter < myTriangles.Length(); ++aTriIter)
            {
                aPoly.SetTriangle(aTriIter + 1, myTriangles[aTriIter]);
            }

            return aPoly;
        }

        protected void Clear()
        {
            myNodes.Clear();
            myTriangles.Clear();
        }
    }

    private sealed class MultiDomainReader : Reader
    {
        //! Add new solid
        //! Add triangulation to triangulation list for multi-domain case
        public override void AddSolid()
        {
            if (GetTriangulation() is Poly_Triangulation aCurrentTri)
            {
                myTriangulationList.Append(aCurrentTri);
            }
            Clear();
        }

        //! Returns triangulation list for multi-domain case
        public NCollection_Sequence<Poly_Triangulation> ChangeTriangulationList() => myTriangulationList;

        private readonly NCollection_Sequence<Poly_Triangulation> myTriangulationList = new();
    }

    // The port publishes the triangulations it reads frozen
    private static Poly_Triangulation? Published(Poly_Triangulation? theTriangulation)
    {
        theTriangulation?.Freeze();
        return theTriangulation;
    }

    // The port's: no result after a user break or past a limit (decision 12203)
    private static bool Stopped(RWStl_Reader theReader, in Message_ProgressRange theProgress)
        => theReader.LimitExceeded() || theProgress.UserBreak();

    /// <summary>
    /// Reads an STL file (format recognised from its content; the domains of a multi-domain file make one
    /// triangulation), merging nodes regardless of the angle. Null on an error with nothing read.
    /// </summary>
    public static Poly_Triangulation? ReadFile(string theFile, Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
        => ReadFile(theFile, Math.PI / 2.0, theProgress, theLimits);

    /// <summary>
    /// Reads an STL file, merging nodes at the same point whose triangles make an angle below
    /// <paramref name="theMergeAngle"/> (radians; 90 degrees merges regardless of the angle). Null on an error with
    /// nothing read.
    /// </summary>
    public static Poly_Triangulation? ReadFile(string theFile, double theMergeAngle, Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        var aReader = new Reader();
        aReader.SetMergeAngle(theMergeAngle);
        aReader.SetLimits(theLimits ?? RWStl_ReadLimits.None);
        aReader.Read(theFile, theProgress);
        // note that returned bool value is ignored intentionally -- even if something went wrong,
        // but some data have been read, we at least will return these data
        return Stopped(aReader, theProgress) ? null : Published(aReader.GetTriangulation());
    }

    /// <summary>
    /// <see cref="ReadFile(string, double, Message_ProgressRange, RWStl_ReadLimits)"/> on the data of a stream from
    /// its position (an addition).
    /// </summary>
    public static Poly_Triangulation? ReadFile(Stream theStream, double theMergeAngle = Math.PI / 2.0, Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        var aReader = new Reader();
        aReader.SetMergeAngle(theMergeAngle);
        aReader.SetLimits(theLimits ?? RWStl_ReadLimits.None);
        aReader.Read(theStream, theProgress);
        return Stopped(aReader, theProgress) ? null : Published(aReader.GetTriangulation());
    }

    /// <summary>
    /// Reads an STL file into one triangulation per domain (solid) of a multi-domain file, into
    /// <paramref name="theTriangList"/> (cleared first). Empty after a user break or past a limit.
    /// </summary>
    public static void ReadFile(string theFile, double theMergeAngle, NCollection_Sequence<Poly_Triangulation> theTriangList,
                                Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        var aReader = new MultiDomainReader();
        aReader.SetMergeAngle(theMergeAngle);
        aReader.SetLimits(theLimits ?? RWStl_ReadLimits.None);
        aReader.Read(theFile, theProgress);
        TakeDomains(aReader, theTriangList, theProgress);
    }

    /// <summary>
    /// <see cref="ReadFile(string, double, NCollection_Sequence{Poly_Triangulation}, Message_ProgressRange, RWStl_ReadLimits)"/>
    /// on the data of a stream from its position (an addition).
    /// </summary>
    public static void ReadFile(Stream theStream, double theMergeAngle, NCollection_Sequence<Poly_Triangulation> theTriangList,
                                Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        var aReader = new MultiDomainReader();
        aReader.SetMergeAngle(theMergeAngle);
        aReader.SetLimits(theLimits ?? RWStl_ReadLimits.None);
        aReader.Read(theStream, theProgress);
        TakeDomains(aReader, theTriangList, theProgress);
    }

    private static void TakeDomains(MultiDomainReader theReader, NCollection_Sequence<Poly_Triangulation> theTriangList, in Message_ProgressRange theProgress)
    {
        theTriangList.Clear();
        if (Stopped(theReader, theProgress))
        {
            return;
        }
        NCollection_Sequence<Poly_Triangulation> aList = theReader.ChangeTriangulationList();
        for (int i = 1; i <= aList.Length(); ++i)
        {
            theTriangList.Append(Published(aList.Value(i))!);
        }
    }

    /// <summary>Reads a binary STL file (one domain). Null on an error or a user break.</summary>
    public static Poly_Triangulation? ReadBinary(string thePath, Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        using FileStream? aStream = RWStl_Reader.OpenFile(thePath);
        if (aStream is null)
        {
            return null;
        }
        // OCCT's ReadBinary(OSD_Path) leaves the merge angle at its default
        return ReadBinaryStream(aStream, Math.PI / 2.0, theProgress, theLimits);
    }

    /// <summary>Reads an ASCII STL file (one domain). Null on an error or a user break.</summary>
    public static Poly_Triangulation? ReadAscii(string thePath, Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        using FileStream? aStream = RWStl_Reader.OpenFile(thePath);
        if (aStream is null)
        {
            return null;
        }
        return ReadAsciiStream(aStream, Math.PI / 2.0, theProgress, theLimits);
    }

    /// <summary>Reads binary STL from a stream (one domain). Null on an error or a user break.</summary>
    public static Poly_Triangulation? ReadBinaryStream(Stream theStream, double theMergeAngle = Math.PI / 2.0,
                                                       Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        var aReader = new Reader();
        aReader.SetMergeAngle(theMergeAngle);
        aReader.SetLimits(theLimits ?? RWStl_ReadLimits.None);
        Standard_IStream? aStream = aReader.OpenStream(theStream);
        if (aStream is null || !aReader.ReadBinary(aStream, theProgress))
        {
            return null;
        }
        return Published(aReader.GetTriangulation());
    }

    /// <summary>Reads ASCII STL from a stream (one domain). Null on an error or a user break.</summary>
    public static Poly_Triangulation? ReadAsciiStream(Stream theStream, double theMergeAngle = Math.PI / 2.0,
                                                      Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        var aReader = new Reader();
        aReader.SetMergeAngle(theMergeAngle);
        aReader.SetLimits(theLimits ?? RWStl_ReadLimits.None);
        Standard_IStream? aStream = aReader.OpenStream(theStream);
        if (aStream is null)
        {
            return null;
        }

        // get length of stream to feed progress indicator
        aStream.SeekGEnd();
        long theEnd = aStream.TellG();
        aStream.SeekG(0);

        var aBuffer = new Standard_ReadLineBuffer(THE_BUFFER_SIZE);
        if (!aReader.ReadAscii(aStream, aBuffer, theEnd, theProgress))
        {
            return null;
        }
        return Published(aReader.GetTriangulation());
    }

    /// <summary>
    /// Reads STL from a stream, ASCII if it starts with "solid", binary otherwise (a binary file whose header starts
    /// with "solid" fails: <see cref="ReadFile(Stream, double, Message_ProgressRange, RWStl_ReadLimits)"/> recognises
    /// it). One domain. Null on an error or a user break.
    /// </summary>
    public static Poly_Triangulation? ReadStream(Stream theStream, double theMergeAngle = Math.PI / 2.0,
                                                 Message_ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        ArgumentNullException.ThrowIfNull(theStream);
        if (!theStream.CanSeek)
        {
            // the port: the peek below needs to seek back (ReadAsciiStream and ReadBinaryStream copy such a stream
            // into memory anyway)
            var aReader = new Reader();
            aReader.SetLimits(theLimits ?? RWStl_ReadLimits.None);
            Standard_IStream? aCopy = aReader.OpenStream(theStream);
            if (aCopy is null)
            {
                return null;
            }
            theStream = aCopy.BaseStream;
        }

        // Try to detect ASCII vs Binary format by peeking at the first few bytes
        long anOriginalPos = theStream.Position;
        Span<byte> aHeader = stackalloc byte[5];
        int aNbRead = theStream.ReadAtLeast(aHeader, 5, throwOnEndOfStream: false);
        theStream.Position = anOriginalPos;

        bool isAscii = aNbRead == 5 && aHeader.SequenceEqual("solid"u8);

        if (isAscii)
        {
            return ReadAsciiStream(theStream, theMergeAngle, theProgress, theLimits);
        }
        else
        {
            return ReadBinaryStream(theStream, theMergeAngle, theProgress, theLimits);
        }
    }

    /// <summary>Writes the triangulation to a binary STL file. False if it is null or empty, or on an error.</summary>
    public static bool WriteBinary(Poly_Triangulation? theMesh, string thePath, Message_ProgressRange theProgress = default)
    {
        if (theMesh is null || theMesh.NbTriangles() <= 0)
        {
            return false;
        }

        FileStream? aStream = CreateFile(thePath);
        if (aStream is null)
        {
            return false;
        }

        using (aStream)
        {
            return WriteBinary(theMesh, aStream, theProgress);
        }
    }

    /// <summary>Writes the triangulation to an ASCII STL file. False if it is null or empty, or on an error.</summary>
    public static bool WriteAscii(Poly_Triangulation? theMesh, string thePath, Message_ProgressRange theProgress = default)
    {
        if (theMesh is null || theMesh.NbTriangles() <= 0)
        {
            return false;
        }

        FileStream? aStream = CreateFile(thePath);
        if (aStream is null)
        {
            return false;
        }

        using (aStream)
        {
            return WriteAscii(theMesh, aStream, theProgress);
        }
    }

    /// <summary>Writes the triangulation as ASCII STL to a stream (bytes, LF line ends).</summary>
    public static bool WriteAscii(Poly_Triangulation? theMesh, Stream theStream, Message_ProgressRange theProgress = default)
    {
        if (theMesh is null || theMesh.NbTriangles() <= 0)
        {
            return false;
        }

        try
        {
            using var aWriter = new StreamWriter(theStream, new UTF8Encoding(false), 1 << 16, leaveOpen: true);
            bool isDone = WriteAscii(theMesh, aWriter, theProgress);
            aWriter.Flush();
            return isDone;
        }
        catch (IOException)
        {
            return false;
        }
    }

    /// <summary>Writes the triangulation as ASCII STL to a text writer.</summary>
    public static bool WriteAscii(Poly_Triangulation? theMesh, TextWriter theStream, Message_ProgressRange theProgress = default)
    {
        if (theMesh is null || theMesh.NbTriangles() <= 0)
        {
            return false;
        }

        try
        {
            // note that space after 'solid' is necessary for many systems
            theStream.Write("solid \n");

            int NBTriangles = theMesh.NbTriangles();
            using var aPS = new Message_ProgressScope(theProgress, "Triangles", NBTriangles);

            for (int aTriIter = 1; aTriIter <= NBTriangles; ++aTriIter)
            {
                Poly_Triangle aTriangle = theMesh.Triangle(aTriIter);
                aTriangle.Get(out int anElem0, out int anElem1, out int anElem2);
                gp_Pnt aP1 = theMesh.Node(anElem0);
                gp_Pnt aP2 = theMesh.Node(anElem1);
                gp_Pnt aP3 = theMesh.Node(anElem2);
                var aVec1 = new gp_Vec(aP1, aP2);
                var aVec2 = new gp_Vec(aP1, aP3);
                gp_Vec aVNorm = aVec1.Crossed(aVec2);
                if (aVNorm.SquareMagnitude() > gp.Resolution())
                {
                    aVNorm.Normalize();
                }
                else
                {
                    aVNorm.SetCoord(0.0, 0.0, 0.0);
                }

                // std::scientific << std::setprecision(12)
                theStream.Write(" facet normal ");
                theStream.Write(E12(aVNorm.X()));
                theStream.Write(' ');
                theStream.Write(E12(aVNorm.Y()));
                theStream.Write(' ');
                theStream.Write(E12(aVNorm.Z()));
                theStream.Write("\n   outer loop\n");
                WriteVertex(theStream, aP1);
                WriteVertex(theStream, aP2);
                WriteVertex(theStream, aP3);
                theStream.Write("   endloop\n endfacet\n");

                // update progress only per 1k triangles
                if ((aTriIter % IND_THRESHOLD) == 0)
                {
                    if (!aPS.More())
                    {
                        return false;
                    }
                    aPS.Next(IND_THRESHOLD);
                }
            }

            theStream.Write("endsolid\n");
            return true;
        }
        catch (IOException)
        {
            return false;
        }
    }

    private static string E12(double theValue) => Standard_Stream.FormatScientific(theValue, 12);

    private static void WriteVertex(TextWriter theStream, in gp_Pnt theP)
    {
        theStream.Write("     vertex ");
        theStream.Write(E12(theP.X()));
        theStream.Write(' ');
        theStream.Write(E12(theP.Y()));
        theStream.Write(' ');
        theStream.Write(E12(theP.Z()));
        theStream.Write('\n');
    }

    /// <summary>Writes the triangulation as binary STL to a stream (OCCT's 80-byte header, then the facets).</summary>
    public static bool WriteBinary(Poly_Triangulation? theMesh, Stream theStream, Message_ProgressRange theProgress = default)
    {
        if (theMesh is null || theMesh.NbTriangles() <= 0)
        {
            return false;
        }

        try
        {
            Span<byte> aHeader = stackalloc byte[80];
            aHeader.Clear();
            Encoding.ASCII.GetBytes(THE_HEADER, aHeader);
            theStream.Write(aHeader);

            int aNBTriangles = theMesh.NbTriangles();
            using var aPS = new Message_ProgressScope(theProgress, "Triangles", aNBTriangles);
            const int aNbChunkTriangles = 4096;
            const int aChunkSize = aNbChunkTriangles * THE_STL_SIZEOF_FACET;
            byte[] aData = new byte[aChunkSize];
            Span<byte> aDataChunk = aData;
            Span<byte> aConv = stackalloc byte[4];
            convertInteger(aNBTriangles, aConv);
            theStream.Write(aConv);

            int aByteCount = 0;
            for (int aTriIter = 1; aTriIter <= aNBTriangles; ++aTriIter)
            {
                Poly_Triangle aTriangle = theMesh.Triangle(aTriIter);
                aTriangle.Get(out int id0, out int id1, out int id2);
                gp_Pnt aP1 = theMesh.Node(id0);
                gp_Pnt aP2 = theMesh.Node(id1);
                gp_Pnt aP3 = theMesh.Node(id2);
                var aVec1 = new gp_Vec(aP1, aP2);
                var aVec2 = new gp_Vec(aP1, aP3);
                gp_Vec aVNorm = aVec1.Crossed(aVec2);
                if (aVNorm.SquareMagnitude() > gp.Resolution())
                {
                    aVNorm.Normalize();
                }
                else
                {
                    aVNorm.SetCoord(0.0, 0.0, 0.0);
                }
                convertDouble(aVNorm.X(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aVNorm.Y(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aVNorm.Z(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aP1.X(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aP1.Y(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aP1.Z(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aP2.X(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aP2.Y(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aP2.Z(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aP3.X(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aP3.Y(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                convertDouble(aP3.Z(), aDataChunk[aByteCount..]);
                aByteCount += 4;
                aDataChunk[aByteCount] = 0;
                aDataChunk[aByteCount + 1] = 0;
                aByteCount += 2;

                // Chunk is full, write it out.
                if (aByteCount == aChunkSize)
                {
                    theStream.Write(aData, 0, aChunkSize);
                    aByteCount = 0;
                }

                // update progress only per 1k triangles
                if ((aTriIter % IND_THRESHOLD) == 0)
                {
                    if (!aPS.More())
                    {
                        return false;
                    }
                    aPS.Next(IND_THRESHOLD);
                }
            }

            // Write last part if necessary.
            if (aByteCount != aChunkSize)
            {
                theStream.Write(aData, 0, aByteCount);
            }

            return true;
        }
        catch (IOException)
        {
            return false;
        }
    }

    // OCCT's std::ofstream: false (null) if the file can't be created
    internal static FileStream? CreateFile(string thePath)
    {
        try
        {
            return new FileStream(thePath, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 16);
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException or ArgumentException or NotSupportedException)
        {
            return null;
        }
    }
}
