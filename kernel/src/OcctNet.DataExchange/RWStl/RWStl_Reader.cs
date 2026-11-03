// Port of: src/DataExchange/TKDESTL/RWStl/RWStl_Reader.hxx
// Port of: src/DataExchange/TKDESTL/RWStl/RWStl_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using System.Buffers;
using System.Buffers.Binary;

namespace Occt;

/// <summary>
/// An STL reader, binary or ASCII, calling <see cref="AddNode"/> and <see cref="AddTriangle"/> (and
/// <see cref="AddSolid"/> after each domain of a multi-domain file) for a derived class to build its mesh. Nodes at the
/// same point are merged (see <see cref="SetMergeAngle"/>).
/// </summary>
/// <remarks>
/// <para>
/// The port reads .NET streams: <see cref="Read(Stream, ProgressRange)"/> reads a stream as
/// <see cref="Read(string, ProgressRange)"/> reads a file, from the stream's position (a stream that can't seek
/// is first copied into memory, within <see cref="RWStl_ReadLimits.MaxBytes"/>). The lower-level
/// <see cref="ReadBinary"/>, <see cref="ReadAscii"/> and <see cref="IsAscii"/> take a <see cref="InputStream"/>,
/// which keeps a C++ stream's flags between calls as OCCT's loop over the domains needs.
/// </para>
/// <para>
/// Limits for hostile input (an addition, <see cref="SetLimits"/>): the size of the data and the number of facets.
/// Past one, reading stops with an error and <see cref="LimitExceeded"/> is true. A binary file's header facet count
/// that the data can't hold sizes the node map by the data instead (OCCT allocates for the header's count).
/// Cancellation is through the progress range, as in OCCT.
/// </para>
/// <para>
/// OCCT parses numbers with the C library's <c>strtod</c> in the C locale; the port with <see cref="StreamUtilities.Strtod(ReadOnlySpan{char}, out int)"/>.
/// </para>
/// </remarks>
public abstract class RWStl_Reader : Transient
{
    private const int THE_STL_HEADER_SIZE = 84;
    private const int THE_STL_SIZEOF_FACET = 50;
    private const int THE_STL_MIN_FILE_SIZE = THE_STL_HEADER_SIZE + THE_STL_SIZEOF_FACET;

    private const int THE_BUFFER_SIZE = 1024;

    protected double myMergeAngle;
    protected double myMergeTolearance;

    // the port's limits for hostile input, and whether one stopped the reading
    private RWStl_ReadLimits myLimits = RWStl_ReadLimits.None;
    private bool myLimitExceeded;
    private long myNbFacetsRead;

    /// <summary>Default constructor: merge angle 90 degrees (any), merge tolerance 0.</summary>
    protected RWStl_Reader()
    {
        myMergeAngle = Math.PI / 2.0;
        myMergeTolearance = 0.0;
    }

    // Merges the nodes of the triangles read, then calls the reader's AddNode and AddTriangle
    private sealed class MergeNodeTool : MergeNodesTool
    {
        private readonly RWStl_Reader myReader;
        private readonly NCollection_DataMap<int, int> myNodeIndexMap;

        public MergeNodeTool(RWStl_Reader theReader, int theNbFacets = -1)
            : base(theReader.MergeAngle(), 0.0, theNbFacets)
        {
            myReader = theReader;
            myNodeIndexMap = new NCollection_DataMap<int, int>();
            // avoid redundant allocations as final triangulation is managed by RWStl_Reader subclass
            ChangeOutput() = null;
        }

        // Add new triangle
        public new void AddTriangle(ReadOnlySpan<Xyz> theElemNodes)
        {
            base.AddTriangle(theElemNodes);

            // remap node indices returned by RWStl_Reader::AddNode();
            // this is a waste of time for most cases of sequential index adding, but preserved for keeping
            // RWStl_Reader interface
            Span<int> aNodesSrc = [ElementNodeIndex(0), ElementNodeIndex(1), ElementNodeIndex(2)];
            Span<int> aNodesRes = [-1, -1, -1];
            for (int aNodeIter = 0; aNodeIter < 3; ++aNodeIter)
            {
                // use existing node if found at the same point
                if (!myNodeIndexMap.Find(aNodesSrc[aNodeIter], out aNodesRes[aNodeIter]))
                {
                    aNodesRes[aNodeIter] = myReader.AddNode(theElemNodes[aNodeIter]);
                    myNodeIndexMap.Bind(aNodesSrc[aNodeIter], aNodesRes[aNodeIter]);
                }
            }
            if (aNodesRes[0] != aNodesRes[1] && aNodesRes[1] != aNodesRes[2] && aNodesRes[2] != aNodesRes[0])
            {
                myReader.AddTriangle(aNodesRes[0], aNodesRes[1], aNodesRes[2]);
            }
        }
    }

    // OCCT's readStlFloat: a little-endian float
    private static float readStlFloat(ReadOnlySpan<byte> theData) => BinaryPrimitives.ReadSingleLittleEndian(theData);

    private static Xyz readStlFloatVec3(ReadOnlySpan<byte> theData)
        => new(readStlFloat(theData), readStlFloat(theData[sizeof(float)..]), readStlFloat(theData[(sizeof(float) * 2)..]));

    /// <summary>
    /// Reads an STL file, binary or ASCII (recognised from its first bytes), with several domains if it is a
    /// concatenation of files (nodes are not merged between domains). True on success, false on an error or a user
    /// break.
    /// </summary>
    public bool Read(string theFile, ProgressRange theProgress = default)
    {
        FileStream? aFile = OpenFile(theFile);
        if (aFile is null)
        {
            global::Kerf.Engine.Foundation.Message.SendFail("Error: file '" + theFile + "' is not found");
            return false;
        }

        using (aFile)
        {
            return Read(aFile, theProgress);
        }
    }

    /// <summary>
    /// <see cref="Read(string, ProgressRange)"/> on the data of a stream from its position (an addition: OCCT
    /// reads files only).
    /// </summary>
    public bool Read(Stream theStream, ProgressRange theProgress = default)
    {
        InputStream? aStream = OpenStream(theStream);
        if (aStream is null)
        {
            return false;
        }

        // get length of file to feed progress indicator in Ascii mode
        aStream.SeekGEnd();
        long theEnd = aStream.TellG();
        aStream.SeekG(0);

        // binary STL files cannot be shorter than 134 bytes
        // (80 bytes header + 4 bytes facet count + 50 bytes for one facet);
        // thus assume files shorter than 134 as Ascii without probing
        // (probing may bring stream to fail state if EOF is reached)
        bool isAscii = (ulong)theEnd < THE_STL_MIN_FILE_SIZE || IsAscii(aStream, true);

        var aBuffer = new ReadLineBuffer(THE_BUFFER_SIZE);

        // Note: here we are trying to handle rare but realistic case of
        // STL files which are composed of several STL data blocks
        // running translation in cycle.
        // For this reason use infinite (logarithmic) progress scale,
        // but in special mode so that the first cycle will take ~ 70% of it
        using var aPS = new ProgressScope(theProgress, null, 1, true);
        while (aStream.Good())
        {
            if (isAscii)
            {
                if (!ReadAscii(aStream, aBuffer, theEnd, aPS.Next(2)))
                {
                    break;
                }
            }
            else
            {
                if (!ReadBinary(aStream, aPS.Next(2)))
                {
                    break;
                }
            }
            aStream.SkipWs(); // skip any white spaces
            AddSolid();
        }
        return !aStream.Fail() && !myLimitExceeded;
    }

    /// <summary>
    /// Guesses whether the stream holds ASCII STL from its first 134 bytes: ASCII if it is shorter, or if none of
    /// them is above <c>~</c> (a binary file may start with "solid " too). The stream goes back to its start
    /// (<paramref name="isSeekgAvailable"/>) or the bytes read are put back.
    /// </summary>
    public bool IsAscii(InputStream theStream, bool isSeekgAvailable)
    {
        // read first 134 bytes to detect file format
        Span<byte> aBuffer = stackalloc byte[THE_STL_MIN_FILE_SIZE];
        long aNbRead = theStream.Read(aBuffer);
        if (theStream.Fail())
        {
            global::Kerf.Engine.Foundation.Message.SendFail("Error: Cannot read file");
            return true;
        }

        if (isSeekgAvailable)
        {
            // get back to the beginning
            theStream.SeekG(0);
        }
        else
        {
            // put back the read symbols
            for (long aByteIter = aNbRead; aByteIter > 0; --aByteIter)
            {
                theStream.Unget();
            }
        }

        // if file is shorter than size of binary file with 1 facet, it must be ascii
        if (aNbRead < THE_STL_MIN_FILE_SIZE)
        {
            return true;
        }

        // otherwise, detect binary format by presence of non-ascii symbols in first 128 bytes
        // (note that binary STL file may start with the same bytes "solid " as Ascii one)
        for (int aByteIter = 0; aByteIter < aNbRead; ++aByteIter)
        {
            if (aBuffer[aByteIter] > (byte)'~')
            {
                return false;
            }
        }
        return true;
    }

    // C's isspace and isalpha in the C locale
    private static bool IsSpace(byte theChar) => theChar is (byte)' ' or (byte)'\t' or (byte)'\n' or (byte)'\v' or (byte)'\f' or (byte)'\r';

    private static bool IsAlpha(byte theChar) => (uint)((theChar | 0x20) - 'a') <= 'z' - 'a';

    // The C string a line holds: up to its first '\0' (OCCT's lines are C strings)
    private static ReadOnlySpan<byte> CStr(Span<byte> theLine)
    {
        int aZero = theLine.IndexOf((byte)0);
        return aZero < 0 ? theLine : theLine[..aZero];
    }

    private static bool str_starts_with(ReadOnlySpan<byte> theStr, string theWord, int theN)
    {
        int i = 0;
        while (i < theStr.Length && IsSpace(theStr[i]))
        {
            i++;
        }

        // strncasecmp: ASCII case-insensitive, a shorter string differs
        ReadOnlySpan<byte> aRest = theStr[i..];
        for (int k = 0; k < theN; ++k)
        {
            if (k >= aRest.Length)
            {
                return false;
            }
            byte aChar = aRest[k];
            if (aChar is >= (byte)'A' and <= (byte)'Z')
            {
                aChar = (byte)(aChar | 0x20);
            }
            if (aChar != theWord[k])
            {
                return false;
            }
        }
        return true;
    }

    private static bool ReadVertex(ReadOnlySpan<byte> theStr, out double theX, out double theY, out double theZ)
    {
        int aPos = 0;

        // skip 'vertex'
        while (aPos < theStr.Length && (IsSpace(theStr[aPos]) || IsAlpha(theStr[aPos])))
        {
            ++aPos;
        }

        // read values
        ReadOnlySpan<byte> aBytes = theStr[aPos..];
        char[]? aRented = null;
        Span<char> aChars = aBytes.Length <= 256 ? stackalloc char[aBytes.Length] : (aRented = ArrayPool<char>.Shared.Rent(aBytes.Length));
        aChars = aChars[..aBytes.Length];
        for (int i = 0; i < aBytes.Length; ++i)
        {
            aChars[i] = (char)aBytes[i];
        }

        // strtod's end pointer is its start when nothing is converted: Strtod's count is 0 then
        theX = StreamUtilities.Strtod(aChars, out int aEnd);
        int aStr = aEnd;
        theY = StreamUtilities.Strtod(aChars[aStr..], out aEnd);
        aStr += aEnd;
        theZ = StreamUtilities.Strtod(aChars[aStr..], out aEnd);
        if (aRented is not null)
        {
            ArrayPool<char>.Shared.Return(aRented);
        }

        // OCCT's "aEnd != aStr": whether the third number was converted
        return aEnd != 0;
    }

    /// <summary>
    /// Reads ASCII STL from the stream (opened in binary mode or not), until a line "endsolid", the end of the stream
    /// or an error. <paramref name="theUntilPos"/> only scales the progress. Empty lines are not supported (they
    /// make the reading fail). True on success, false on an error or a user break.
    /// </summary>
    public bool ReadAscii(InputStream theStream, ReadLineBuffer theBuffer, long theUntilPos, ProgressRange theProgress)
    {
        long aStartPos = theStream.TellG();
        Span<byte> aLine;

        // skip header "solid ..."
        bool isLine = theBuffer.ReadLine(theStream, out aLine);
        // skip empty lines
        while (isLine && CStr(aLine).Length == 0)
        {
            isLine = theBuffer.ReadLine(theStream, out aLine);
        }
        if (!isLine)
        {
            global::Kerf.Engine.Foundation.Message.SendFail("Error: premature end of file");
            return false;
        }

        var aMergeTool = new MergeNodeTool(this);
        aMergeTool.SetMergeAngle(myMergeAngle);
        aMergeTool.SetMergeTolerance(myMergeTolearance);

        // report progress every 1 MiB of read data
        const int aStepB = 1024 * 1024;
        int aNbSteps = 1 + (int)((theUntilPos - aStartPos) / aStepB);
        using var aPS = new ProgressScope(theProgress, "Reading text STL file", aNbSteps);
        long aProgressPos = aStartPos + aStepB;
        int aNbLine = 1;

        Span<Xyz> aVertex = stackalloc Xyz[3];
        while (aPS.More())
        {
            if (theStream.TellG() > aProgressPos)
            {
                aPS.Next();
                aProgressPos += aStepB;
            }

            isLine = theBuffer.ReadLine(theStream, out aLine); // "facet normal nx ny nz"
            if (!isLine)
            {
                global::Kerf.Engine.Foundation.Message.SendFail("Error: premature end of file");
                return false;
            }
            if (str_starts_with(CStr(aLine), "endsolid", 8))
            {
                // end of STL code
                break;
            }
            if (!str_starts_with(CStr(aLine), "facet", 5))
            {
                global::Kerf.Engine.Foundation.Message.SendFail("Error: unexpected format of facet at line " + (aNbLine + 1));
                return false;
            }

            isLine = theBuffer.ReadLine(theStream, out aLine); // "outer loop"
            if (!isLine || !str_starts_with(CStr(aLine), "outer", 5))
            {
                global::Kerf.Engine.Foundation.Message.SendFail("Error: unexpected format of facet at line " + (aNbLine + 1));
                return false;
            }

            bool isEOF = false;
            for (int i = 0; i < 3; i++)
            {
                isLine = theBuffer.ReadLine(theStream, out aLine);
                if (!isLine)
                {
                    isEOF = true;
                    break;
                }
                if (!ReadVertex(CStr(aLine), out double aX, out double aY, out double aZ))
                {
                    global::Kerf.Engine.Foundation.Message.SendFail("Error: cannot read vertex coordinates at line " + aNbLine);
                    return false;
                }
                aVertex[i] = new Xyz(aX, aY, aZ);
            }

            // stop reading if end of file is reached;
            // note that well-formatted file never ends by the vertex line
            if (isEOF)
            {
                break;
            }

            aNbLine += 5;

            // add triangle
            if (!CountFacets(1))
            {
                return false;
            }
            aMergeTool.AddTriangle(aVertex);

            theBuffer.ReadLine(theStream, out _); // skip "endloop"
            theBuffer.ReadLine(theStream, out _); // skip "endfacet"

            aNbLine += 2;
        }

        return aPS.More();
    }

    /// <summary>
    /// Reads binary STL from the stream (opened in binary mode): the 84-byte header, then as many facets as the
    /// header's count says. True on success, false on an error or a user break.
    /// </summary>
    public bool ReadBinary(InputStream theStream, ProgressRange theProgress)
    {
        // read file header at first
        Span<byte> aHeader = stackalloc byte[THE_STL_HEADER_SIZE];
        if (theStream.Read(aHeader) != THE_STL_HEADER_SIZE)
        {
            global::Kerf.Engine.Foundation.Message.SendFail("Error: Corrupted binary STL file");
            return false;
        }

        // number of facets is stored as 32-bit integer at position 80
        int aNbFacets = BinaryPrimitives.ReadInt32LittleEndian(aHeader[80..]);

        if (aNbFacets > 0 && !CountFacets(aNbFacets))
        {
            return false;
        }
        // the port: the node map is sized for the facets the data can hold, if the header claims more
        long aNbFacetsInData = (theStream.BaseStream.Length - theStream.BaseStream.Position) / THE_STL_SIZEOF_FACET;
        var aMergeTool = new MergeNodeTool(this, (int)Math.Min(aNbFacets, aNbFacetsInData));
        aMergeTool.SetMergeAngle(myMergeAngle);
        aMergeTool.SetMergeTolerance(myMergeTolearance);

        // don't trust the number of triangles which is coded in the file
        // sometimes it is wrong, and with this technique we don't need to swap endians for integer
        using var aPS = new ProgressScope(theProgress, "Reading binary STL file", aNbFacets);

        // allocate buffer for 80 triangles
        const int THE_CHUNK_NBFACETS = 80;
        byte[] aBuffer = new byte[THE_STL_SIZEOF_FACET * THE_CHUNK_NBFACETS];

        // normal + 3 nodes + 2 extra bytes
        const int aVec3Size = sizeof(float) * 3;
        const int aFaceDataLen = aVec3Size * 4 + 2;
        int aBufferPtr = 0;
        int aNbFacesInBuffer = 0;
        Span<Xyz> aTriNodes = stackalloc Xyz[3];
        for (int aNbFacetRead = 0; aNbFacetRead < aNbFacets && aPS.More();
             ++aNbFacetRead, --aNbFacesInBuffer, aBufferPtr += aFaceDataLen, aPS.Next())
        {
            // read more data
            if (aNbFacesInBuffer <= 0)
            {
                aNbFacesInBuffer = Math.Min(THE_CHUNK_NBFACETS, aNbFacets - aNbFacetRead);
                int aDataToRead = aNbFacesInBuffer * aFaceDataLen;
                if (theStream.Read(aBuffer.AsSpan(0, aDataToRead)) != aDataToRead)
                {
                    global::Kerf.Engine.Foundation.Message.SendFail("Error: binary STL read failed");
                    return false;
                }
                aBufferPtr = 0;
            }

            // get points from buffer
            //    readStlFloatVec3 (aBufferPtr); // skip normal
            ReadOnlySpan<byte> aFacet = aBuffer.AsSpan(aBufferPtr);
            aTriNodes[0] = readStlFloatVec3(aFacet[aVec3Size..]);
            aTriNodes[1] = readStlFloatVec3(aFacet[(aVec3Size * 2)..]);
            aTriNodes[2] = readStlFloatVec3(aFacet[(aVec3Size * 3)..]);
            aMergeTool.AddTriangle(aTriNodes);
        }

        return aPS.More();
    }

    /// <summary>
    /// Called for each node: creates it in the target model and returns its index (as given to
    /// <see cref="AddTriangle"/>).
    /// </summary>
    public abstract int AddNode(in Xyz thePnt);

    /// <summary>Called for each triangle: creates it on the nodes in the target model.</summary>
    public abstract void AddTriangle(int theN1, int theN2, int theN3);

    /// <summary>Called after each domain (solid) of the data: may start a new triangulation.</summary>
    public virtual void AddSolid()
    {
    }

    /// <summary>
    /// The merge angle in radians: nodes at the same point are merged when the angle between their triangles is below
    /// it. 90 degrees (the default) merges regardless of the angle; 0 merges nothing.
    /// </summary>
    public double MergeAngle() => myMergeAngle;

    /// <summary>Sets the merge angle in radians (something like 45 degrees keeps sharp corners unmerged).</summary>
    public void SetMergeAngle(double theAngleRad) => myMergeAngle = theAngleRad;

    /// <summary>The linear merge tolerance: 0 by default (only points with exactly the same coordinates merge).</summary>
    public double MergeTolerance() => myMergeTolearance;

    /// <summary>Sets the linear merge tolerance.</summary>
    public void SetMergeTolerance(double theTolerance) => myMergeTolearance = theTolerance;

    /// <summary>The limits for hostile input (an addition): none by default.</summary>
    public RWStl_ReadLimits Limits() => myLimits;

    /// <summary>Sets the limits for hostile input (an addition).</summary>
    public void SetLimits(RWStl_ReadLimits theLimits) => myLimits = theLimits ?? RWStl_ReadLimits.None;

    /// <summary>True if a limit stopped the reading (an addition).</summary>
    public bool LimitExceeded() => myLimitExceeded;

    // Counts facets against the limit: false (with an error message) past it
    private bool CountFacets(long theNbFacets)
    {
        myNbFacetsRead += theNbFacets;
        if (myNbFacetsRead > myLimits.MaxTriangles)
        {
            myLimitExceeded = true;
            global::Kerf.Engine.Foundation.Message.SendFail("Error: the STL data exceed the limit of " + myLimits.MaxTriangles + " triangles");
            return false;
        }
        return true;
    }

    /// <summary>
    /// The stream to read, from its position, within the size limit: a stream that can't seek is copied into memory
    /// first. Null (with an error message, and <see cref="LimitExceeded"/>) if the data are larger than the limit.
    /// </summary>
    internal InputStream? OpenStream(Stream theStream)
    {
        ArgumentNullException.ThrowIfNull(theStream);
        long aMax = myLimits.MaxBytes;
        if (theStream.CanSeek)
        {
            if (theStream.Length - theStream.Position > aMax)
            {
                return SizeExceeded();
            }
            return new InputStream(theStream);
        }

        var aCopy = new MemoryStream();
        byte[] aChunk = ArrayPool<byte>.Shared.Rent(81920);
        try
        {
            for (;;)
            {
                int aNb = theStream.Read(aChunk, 0, aChunk.Length);
                if (aNb <= 0)
                {
                    break;
                }
                if (aCopy.Length + aNb > aMax)
                {
                    return SizeExceeded();
                }
                aCopy.Write(aChunk, 0, aNb);
            }
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(aChunk);
        }
        aCopy.Position = 0;
        return new InputStream(aCopy);
    }

    private InputStream? SizeExceeded()
    {
        myLimitExceeded = true;
        global::Kerf.Engine.Foundation.Message.SendFail("Error: the STL data exceed the limit of " + myLimits.MaxBytes + " bytes");
        return null;
    }

    // OCCT's OSD_FileSystem::OpenIStream: null if the file can't be opened
    internal static FileStream? OpenFile(string theFile)
    {
        try
        {
            return new FileStream(theFile, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 16);
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException or ArgumentException or NotSupportedException)
        {
            return null;
        }
    }
}
