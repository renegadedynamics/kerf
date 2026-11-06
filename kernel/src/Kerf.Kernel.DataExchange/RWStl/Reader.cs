// Port of: src/DataExchange/TKDESTL/RWStl/RWStl_Reader.hxx
// Port of: src/DataExchange/TKDESTL/RWStl/RWStl_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using System.Buffers;
using System.Buffers.Binary;

namespace Kerf.Engine.Exchange.Stl;

/// <summary>
/// An STL reader, binary or ASCII, calling <see cref="AddNode"/> and <see cref="AddTriangle"/> (and
/// <see cref="AddSolid"/> after each domain of a multi-domain file) for a derived class to build its mesh. Nodes at the
/// same point are merged (see <see cref="SetMergeAngle"/>).
/// </summary>
/// <remarks>
/// <para>
/// The port reads .NET streams: <see cref="Read(Stream, ProgressRange)"/> reads a stream as
/// <see cref="Read(string, ProgressRange)"/> reads a file, from the stream's position (a stream that can't seek
/// is first copied into memory, within <see cref="StlReadLimits.MaxBytes"/>). The lower-level
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
internal abstract class Reader : Transient
{
    private const int THE_STL_HEADER_SIZE = 84;
    private const int THE_STL_SIZEOF_FACET = 50;
    private const int THE_STL_MIN_FILE_SIZE = THE_STL_HEADER_SIZE + THE_STL_SIZEOF_FACET;

    private const int THE_BUFFER_SIZE = 1024;

    protected double _mergeAngle;
    protected double _mergeTolearance;

    // the port's limits for hostile input, and whether one stopped the reading
    private StlReadLimits _limits = StlReadLimits.None;
    private bool _limitExceeded;
    private long _nbFacetsRead;

    /// <summary>Default constructor: merge angle 90 degrees (any), merge tolerance 0.</summary>
    protected Reader()
    {
        _mergeAngle = Math.PI / 2.0;
        _mergeTolearance = 0.0;
    }

    // Merges the nodes of the triangles read, then calls the reader's AddNode and AddTriangle
    private sealed class MergeNodeTool : MergeNodesTool
    {
        private readonly Reader _reader;
        private readonly Dictionary<int, int> _nodeIndexMap;

        public MergeNodeTool(Reader reader, int nbFacets = -1)
            : base(reader.MergeAngle(), 0.0, nbFacets)
        {
            _reader = reader;
            _nodeIndexMap = new Dictionary<int, int>();
            // avoid redundant allocations as final triangulation is managed by RWStl_Reader subclass
            ChangeOutput() = null;
        }

        // Add new triangle
        public new void AddTriangle(ReadOnlySpan<Xyz> elemNodes)
        {
            base.AddTriangle(elemNodes);

            // remap node indices returned by RWStl_Reader::AddNode();
            // this is a waste of time for most cases of sequential index adding, but preserved for keeping
            // RWStl_Reader interface
            Span<int> nodesSrc = [ElementNodeIndex(0), ElementNodeIndex(1), ElementNodeIndex(2)];
            Span<int> nodesRes = [-1, -1, -1];
            for (int nodeIter = 0; nodeIter < 3; ++nodeIter)
            {
                // use existing node if found at the same point
                if (!_nodeIndexMap.TryGetValue(nodesSrc[nodeIter], out nodesRes[nodeIter]))
                {
                    nodesRes[nodeIter] = _reader.AddNode(elemNodes[nodeIter]);
                    _nodeIndexMap[nodesSrc[nodeIter]] = nodesRes[nodeIter];
                }
            }
            if (nodesRes[0] != nodesRes[1] && nodesRes[1] != nodesRes[2] && nodesRes[2] != nodesRes[0])
            {
                _reader.AddTriangle(nodesRes[0], nodesRes[1], nodesRes[2]);
            }
        }
    }

    // OCCT's readStlFloat: a little-endian float
    private static float ReadStlFloat(ReadOnlySpan<byte> data) => BinaryPrimitives.ReadSingleLittleEndian(data);

    private static Xyz ReadStlFloatVec3(ReadOnlySpan<byte> data)
        => new(ReadStlFloat(data), ReadStlFloat(data[sizeof(float)..]), ReadStlFloat(data[(sizeof(float) * 2)..]));

    /// <summary>
    /// Reads an STL file, binary or ASCII (recognised from its first bytes), with several domains if it is a
    /// concatenation of files (nodes are not merged between domains). True on success, false on an error or a user
    /// break.
    /// </summary>
    public bool Read(string file, ProgressRange progress = default)
    {
        FileStream? aFile = OpenFile(file);
        if (aFile is null)
        {
            global::Kerf.Engine.Foundation.Message.SendFail("Error: file '" + file + "' is not found");
            return false;
        }

        using (aFile)
        {
            return Read(aFile, progress);
        }
    }

    /// <summary>
    /// <see cref="Read(string, ProgressRange)"/> on the data of a stream from its position (an addition: OCCT
    /// reads files only).
    /// </summary>
    public bool Read(Stream stream, ProgressRange progress = default)
    {
        InputStream? aStream = OpenStream(stream);
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

        var buffer = new ReadLineBuffer(THE_BUFFER_SIZE);

        // Note: here we are trying to handle rare but realistic case of
        // STL files which are composed of several STL data blocks
        // running translation in cycle.
        // For this reason use infinite (logarithmic) progress scale,
        // but in special mode so that the first cycle will take ~ 70% of it
        using var ps = new ProgressScope(progress, null, 1, true);
        while (aStream.Good())
        {
            if (isAscii)
            {
                if (!ReadAscii(aStream, buffer, theEnd, ps.Next(2)))
                {
                    break;
                }

                // the buffer reads ahead in blocks: the next domain starts where its lines end, not at the stream's
                // position, which may be past it, or at the end (OCCT went on from there, so it lost the domains after
                // the first in files under a block, and could join numbers at a block's end; Kerf decision 1499)
                int anUnread = buffer.UnreadCount;
                if (anUnread > 0)
                {
                    aStream.Clear();
                    aStream.SeekG(aStream.TellG() - anUnread);
                    buffer.Clear();
                }
            }
            else
            {
                if (!ReadBinary(aStream, ps.Next(2)))
                {
                    break;
                }
            }
            aStream.SkipWs(); // skip any white spaces
            AddSolid();
        }
        return !aStream.Fail() && !_limitExceeded;
    }

    /// <summary>
    /// Guesses whether the stream holds ASCII STL from its first 134 bytes: ASCII if it is shorter, or if none of
    /// them is above <c>~</c> (a binary file may start with "solid " too). The stream goes back to its start
    /// (<paramref name="isSeekgAvailable"/>) or the bytes read are put back.
    /// </summary>
    public bool IsAscii(InputStream stream, bool isSeekgAvailable)
    {
        // read first 134 bytes to detect file format
        Span<byte> buffer = stackalloc byte[THE_STL_MIN_FILE_SIZE];
        long nbRead = stream.Read(buffer);
        if (stream.Fail())
        {
            global::Kerf.Engine.Foundation.Message.SendFail("Error: Cannot read file");
            return true;
        }

        if (isSeekgAvailable)
        {
            // get back to the beginning
            stream.SeekG(0);
        }
        else
        {
            // put back the read symbols
            for (long byteIter = nbRead; byteIter > 0; --byteIter)
            {
                stream.Unget();
            }
        }

        // if file is shorter than size of binary file with 1 facet, it must be ascii
        if (nbRead < THE_STL_MIN_FILE_SIZE)
        {
            return true;
        }

        // otherwise, detect binary format by presence of non-ascii symbols in first 128 bytes
        // (note that binary STL file may start with the same bytes "solid " as Ascii one)
        for (int byteIter = 0; byteIter < nbRead; ++byteIter)
        {
            if (buffer[byteIter] > (byte)'~')
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
    private static ReadOnlySpan<byte> CStr(Span<byte> line)
    {
        int zero = line.IndexOf((byte)0);
        return zero < 0 ? line : line[..zero];
    }

    private static bool StrStartsWith(ReadOnlySpan<byte> str, string word, int n)
    {
        int i = 0;
        while (i < str.Length && IsSpace(str[i]))
        {
            i++;
        }

        // strncasecmp: ASCII case-insensitive, a shorter string differs
        ReadOnlySpan<byte> rest = str[i..];
        for (int k = 0; k < n; ++k)
        {
            if (k >= rest.Length)
            {
                return false;
            }
            byte aChar = rest[k];
            if (aChar is >= (byte)'A' and <= (byte)'Z')
            {
                aChar = (byte)(aChar | 0x20);
            }
            if (aChar != word[k])
            {
                return false;
            }
        }
        return true;
    }

    private static bool ReadVertex(ReadOnlySpan<byte> str, out double x, out double y, out double z)
    {
        int pos = 0;

        // skip 'vertex'
        while (pos < str.Length && (IsSpace(str[pos]) || IsAlpha(str[pos])))
        {
            ++pos;
        }

        // read values
        ReadOnlySpan<byte> bytes = str[pos..];
        char[]? rented = null;
        Span<char> chars = bytes.Length <= 256 ? stackalloc char[bytes.Length] : (rented = ArrayPool<char>.Shared.Rent(bytes.Length));
        chars = chars[..bytes.Length];
        for (int i = 0; i < bytes.Length; ++i)
        {
            chars[i] = (char)bytes[i];
        }

        // strtod's end pointer is its start when nothing is converted: Strtod's count is 0 then
        x = StreamUtilities.Strtod(chars, out int end);
        int aStr = end;
        y = StreamUtilities.Strtod(chars[aStr..], out end);
        aStr += end;
        z = StreamUtilities.Strtod(chars[aStr..], out end);
        if (rented is not null)
        {
            ArrayPool<char>.Shared.Return(rented);
        }

        // OCCT's "aEnd != aStr": whether the third number was converted
        return end != 0;
    }

    /// <summary>
    /// Reads ASCII STL from the stream (opened in binary mode or not), until a line "endsolid", the end of the stream
    /// or an error. <paramref name="untilPos"/> only scales the progress. Empty lines are not supported (they
    /// make the reading fail). True on success, false on an error or a user break.
    /// </summary>
    public bool ReadAscii(InputStream stream, ReadLineBuffer buffer, long untilPos, ProgressRange progress)
    {
        long startPos = stream.TellG();
        Span<byte> line;

        // skip header "solid ..."
        bool isLine = buffer.ReadLine(stream, out line);
        // skip empty lines
        while (isLine && CStr(line).Length == 0)
        {
            isLine = buffer.ReadLine(stream, out line);
        }
        if (!isLine)
        {
            global::Kerf.Engine.Foundation.Message.SendFail("Error: premature end of file");
            return false;
        }

        var mergeTool = new MergeNodeTool(this);
        mergeTool.SetMergeAngle(_mergeAngle);
        mergeTool.SetMergeTolerance(_mergeTolearance);

        // report progress every 1 MiB of read data
        const int stepB = 1024 * 1024;
        int nbSteps = 1 + (int)((untilPos - startPos) / stepB);
        using var ps = new ProgressScope(progress, "Reading text STL file", nbSteps);
        long progressPos = startPos + stepB;
        int nbLine = 1;

        Span<Xyz> vertex = stackalloc Xyz[3];
        while (ps.More())
        {
            if (stream.TellG() > progressPos)
            {
                ps.Next();
                progressPos += stepB;
            }

            isLine = buffer.ReadLine(stream, out line); // "facet normal nx ny nz"
            if (!isLine)
            {
                global::Kerf.Engine.Foundation.Message.SendFail("Error: premature end of file");
                return false;
            }
            if (StrStartsWith(CStr(line), "endsolid", 8))
            {
                // end of STL code
                break;
            }
            if (!StrStartsWith(CStr(line), "facet", 5))
            {
                global::Kerf.Engine.Foundation.Message.SendFail("Error: unexpected format of facet at line " + (nbLine + 1));
                return false;
            }

            isLine = buffer.ReadLine(stream, out line); // "outer loop"
            if (!isLine || !StrStartsWith(CStr(line), "outer", 5))
            {
                global::Kerf.Engine.Foundation.Message.SendFail("Error: unexpected format of facet at line " + (nbLine + 1));
                return false;
            }

            bool isEOF = false;
            for (int i = 0; i < 3; i++)
            {
                isLine = buffer.ReadLine(stream, out line);
                if (!isLine)
                {
                    isEOF = true;
                    break;
                }
                if (!ReadVertex(CStr(line), out double x, out double y, out double z))
                {
                    global::Kerf.Engine.Foundation.Message.SendFail("Error: cannot read vertex coordinates at line " + nbLine);
                    return false;
                }
                vertex[i] = new Xyz(x, y, z);
            }

            // stop reading if end of file is reached;
            // note that well-formatted file never ends by the vertex line
            if (isEOF)
            {
                break;
            }

            nbLine += 5;

            // add triangle
            if (!CountFacets(1))
            {
                return false;
            }
            mergeTool.AddTriangle(vertex);

            buffer.ReadLine(stream, out _); // skip "endloop"
            buffer.ReadLine(stream, out _); // skip "endfacet"

            nbLine += 2;
        }

        return ps.More();
    }

    /// <summary>
    /// Reads binary STL from the stream (opened in binary mode): the 84-byte header, then as many facets as the
    /// header's count says. True on success, false on an error or a user break.
    /// </summary>
    public bool ReadBinary(InputStream stream, ProgressRange progress)
    {
        // read file header at first
        Span<byte> header = stackalloc byte[THE_STL_HEADER_SIZE];
        if (stream.Read(header) != THE_STL_HEADER_SIZE)
        {
            global::Kerf.Engine.Foundation.Message.SendFail("Error: Corrupted binary STL file");
            return false;
        }

        // number of facets is stored as 32-bit integer at position 80
        int nbFacets = BinaryPrimitives.ReadInt32LittleEndian(header[80..]);

        if (nbFacets > 0 && !CountFacets(nbFacets))
        {
            return false;
        }
        // the port: the node map is sized for the facets the data can hold, if the header claims more
        long nbFacetsInData = (stream.BaseStream.Length - stream.BaseStream.Position) / THE_STL_SIZEOF_FACET;
        var mergeTool = new MergeNodeTool(this, (int)Math.Min(nbFacets, nbFacetsInData));
        mergeTool.SetMergeAngle(_mergeAngle);
        mergeTool.SetMergeTolerance(_mergeTolearance);

        // don't trust the number of triangles which is coded in the file
        // sometimes it is wrong, and with this technique we don't need to swap endians for integer
        using var ps = new ProgressScope(progress, "Reading binary STL file", nbFacets);

        // allocate buffer for 80 triangles
        const int THE_CHUNK_NBFACETS = 80;
        byte[] buffer = new byte[THE_STL_SIZEOF_FACET * THE_CHUNK_NBFACETS];

        // normal + 3 nodes + 2 extra bytes
        const int vec3Size = sizeof(float) * 3;
        const int faceDataLen = vec3Size * 4 + 2;
        int bufferPtr = 0;
        int nbFacesInBuffer = 0;
        Span<Xyz> triNodes = stackalloc Xyz[3];
        for (int nbFacetRead = 0; nbFacetRead < nbFacets && ps.More();
             ++nbFacetRead, --nbFacesInBuffer, bufferPtr += faceDataLen, ps.Next())
        {
            // read more data
            if (nbFacesInBuffer <= 0)
            {
                nbFacesInBuffer = Math.Min(THE_CHUNK_NBFACETS, nbFacets - nbFacetRead);
                int dataToRead = nbFacesInBuffer * faceDataLen;
                if (stream.Read(buffer.AsSpan(0, dataToRead)) != dataToRead)
                {
                    global::Kerf.Engine.Foundation.Message.SendFail("Error: binary STL read failed");
                    return false;
                }
                bufferPtr = 0;
            }

            // get points from buffer
            //    readStlFloatVec3 (aBufferPtr); // skip normal
            ReadOnlySpan<byte> facet = buffer.AsSpan(bufferPtr);
            triNodes[0] = ReadStlFloatVec3(facet[vec3Size..]);
            triNodes[1] = ReadStlFloatVec3(facet[(vec3Size * 2)..]);
            triNodes[2] = ReadStlFloatVec3(facet[(vec3Size * 3)..]);
            mergeTool.AddTriangle(triNodes);
        }

        return ps.More();
    }

    /// <summary>
    /// Called for each node: creates it in the target model and returns its index (as given to
    /// <see cref="AddTriangle"/>).
    /// </summary>
    public abstract int AddNode(in Xyz pnt);

    /// <summary>Called for each triangle: creates it on the nodes in the target model.</summary>
    public abstract void AddTriangle(int n1, int n2, int n3);

    /// <summary>Called after each domain (solid) of the data: may start a new triangulation.</summary>
    public virtual void AddSolid()
    {
    }

    /// <summary>
    /// The merge angle in radians: nodes at the same point are merged when the angle between their triangles is below
    /// it. 90 degrees (the default) merges regardless of the angle; 0 merges nothing.
    /// </summary>
    public double MergeAngle() => _mergeAngle;

    /// <summary>Sets the merge angle in radians (something like 45 degrees keeps sharp corners unmerged).</summary>
    public void SetMergeAngle(double angleRad) => _mergeAngle = angleRad;

    /// <summary>The linear merge tolerance: 0 by default (only points with exactly the same coordinates merge).</summary>
    public double MergeTolerance() => _mergeTolearance;

    /// <summary>Sets the linear merge tolerance.</summary>
    public void SetMergeTolerance(double tolerance) => _mergeTolearance = tolerance;

    /// <summary>The limits for hostile input (an addition): none by default.</summary>
    public StlReadLimits Limits() => _limits;

    /// <summary>Sets the limits for hostile input (an addition).</summary>
    public void SetLimits(StlReadLimits limits) => _limits = limits ?? StlReadLimits.None;

    /// <summary>True if a limit stopped the reading (an addition).</summary>
    public bool LimitExceeded() => _limitExceeded;

    // Counts facets against the limit: false (with an error message) past it
    private bool CountFacets(long nbFacets)
    {
        _nbFacetsRead += nbFacets;
        if (_nbFacetsRead > _limits.MaxTriangles)
        {
            _limitExceeded = true;
            global::Kerf.Engine.Foundation.Message.SendFail("Error: the STL data exceed the limit of " + _limits.MaxTriangles + " triangles");
            return false;
        }
        return true;
    }

    /// <summary>
    /// The stream to read, from its position, within the size limit: a stream that can't seek is copied into memory
    /// first. Null (with an error message, and <see cref="LimitExceeded"/>) if the data are larger than the limit.
    /// </summary>
    internal InputStream? OpenStream(Stream stream)
    {
        ArgumentNullException.ThrowIfNull(stream);
        long max = _limits.MaxBytes;
        if (stream.CanSeek)
        {
            if (stream.Length - stream.Position > max)
            {
                return SizeExceeded();
            }
            return new InputStream(stream);
        }

        var copy = new MemoryStream();
        byte[] chunk = ArrayPool<byte>.Shared.Rent(81920);
        try
        {
            for (;;)
            {
                int nb = stream.Read(chunk, 0, chunk.Length);
                if (nb <= 0)
                {
                    break;
                }
                if (copy.Length + nb > max)
                {
                    return SizeExceeded();
                }
                copy.Write(chunk, 0, nb);
            }
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(chunk);
        }
        copy.Position = 0;
        return new InputStream(copy);
    }

    private InputStream? SizeExceeded()
    {
        _limitExceeded = true;
        global::Kerf.Engine.Foundation.Message.SendFail("Error: the STL data exceed the limit of " + _limits.MaxBytes + " bytes");
        return null;
    }

    // OCCT's OSD_FileSystem::OpenIStream: null if the file can't be opened
    internal static FileStream? OpenFile(string file)
    {
        try
        {
            return new FileStream(file, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 16);
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException or ArgumentException or NotSupportedException)
        {
            return null;
        }
    }
}
