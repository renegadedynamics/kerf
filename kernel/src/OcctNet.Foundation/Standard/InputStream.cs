// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The behaviour of C++'s std::istream (OCCT's Standard_IStream typedef) in binary mode, for ports that read bytes.

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>
/// C++'s <c>std::istream</c> (OCCT's <c>Standard_IStream</c>) opened in binary mode, over a seekable .NET
/// <see cref="Stream"/>: the state flags (<c>good</c>, <c>eof</c>, <c>fail</c>, <c>bad</c>) and the unformatted
/// operations OCCT's byte readers use (<c>read</c> and <c>gcount</c>, <c>tellg</c>, <c>seekg</c>, <c>unget</c>,
/// <c>&gt;&gt; std::ws</c>), with C++11's rules, so that ported code which tests the flags between calls behaves as
/// OCCT's does.
/// </summary>
/// <remarks>
/// <para>
/// Positions are counted from the stream's position when this object was made, which is position 0 for the ported
/// code (<c>seekg(0, beg)</c> goes back there): a reader given a stream in the middle of a file reads from there as
/// from the start of a file.
/// </para>
/// <para>
/// An <see cref="IOException"/> of the underlying stream sets the bad flag, as a C++ stream's buffer error does; the
/// stream itself is not closed.
/// </para>
/// </remarks>
public sealed class InputStream
{
    private readonly Stream _stream;
    private readonly long _origin;
    private bool _eof;
    private bool _fail;
    private bool _bad;
    private long _gCount;

    /// <summary>Reads <paramref name="stream"/> from its current position.</summary>
    /// <exception cref="ArgumentException">The stream can't read or can't seek.</exception>
    public InputStream(Stream stream)
    {
        ArgumentNullException.ThrowIfNull(stream);
        if (!stream.CanRead || !stream.CanSeek)
        {
            throw new ArgumentException("Standard_IStream: the stream must be readable and seekable", nameof(stream));
        }

        _stream = stream;
        _origin = stream.Position;
    }

    /// <summary>The underlying stream.</summary>
    public Stream BaseStream => _stream;

    /// <summary>C++'s <c>good()</c>: no flag set.</summary>
    public bool Good() => !_eof && !_fail && !_bad;

    /// <summary>C++'s <c>eof()</c>.</summary>
    public bool Eof() => _eof;

    /// <summary>C++'s <c>fail()</c>: the fail or the bad flag (also C++'s <c>!stream</c>).</summary>
    public bool Fail() => _fail || _bad;

    /// <summary>C++'s <c>bad()</c>.</summary>
    public bool Bad() => _bad;

    /// <summary>C++'s <c>clear()</c>: clears every flag.</summary>
    public void Clear()
    {
        _eof = false;
        _fail = false;
        _bad = false;
    }

    /// <summary>C++'s <c>gcount()</c>: the number of bytes the last unformatted input read.</summary>
    public long GCount() => _gCount;

    /// <summary>
    /// C++'s <c>read(s, n)</c>: reads up to the span's length. Fewer bytes (the end of the stream) set the eof and
    /// fail flags; a stream not good beforehand reads nothing and sets the fail flag. Returns <see cref="GCount"/>.
    /// </summary>
    public long Read(Span<byte> buffer)
    {
        _gCount = 0;
        if (!Good())
        {
            _fail = true;
            return 0;
        }

        int nbRead = 0;
        try
        {
            while (nbRead < buffer.Length)
            {
                int nb = _stream.Read(buffer[nbRead..]);
                if (nb <= 0)
                {
                    break;
                }
                nbRead += nb;
            }
        }
        catch (IOException)
        {
            _bad = true;
        }

        _gCount = nbRead;
        if (nbRead < buffer.Length && !_bad)
        {
            _eof = true;
            _fail = true;
        }
        return nbRead;
    }

    /// <summary>C++'s <c>tellg()</c>: the position, or -1 if <see cref="Fail"/>.</summary>
    public long TellG() => Fail() ? -1 : _stream.Position - _origin;

    /// <summary>
    /// C++'s <c>seekg(pos)</c> (C++11): clears the eof flag, then does nothing if <see cref="Fail"/>; a position out of
    /// the stream sets the fail flag.
    /// </summary>
    public void SeekG(long pos)
    {
        _eof = false;
        if (Fail())
        {
            return;
        }

        if (pos < 0 || pos > _stream.Length - _origin)
        {
            _fail = true;
            return;
        }
        _stream.Position = _origin + pos;
    }

    /// <summary>C++'s <c>seekg(0, end)</c>.</summary>
    public void SeekGEnd()
    {
        _eof = false;
        if (Fail())
        {
            return;
        }
        _stream.Position = _stream.Length;
    }

    /// <summary>
    /// C++'s <c>unget()</c> (C++11): clears the eof flag, then puts the last byte back; at the start (or if not good)
    /// sets the bad (fail) flag.
    /// </summary>
    public void Unget()
    {
        _gCount = 0;
        _eof = false;
        if (!Good())
        {
            _fail = true;
            return;
        }
        if (_stream.Position <= _origin)
        {
            _bad = true;
            return;
        }
        _stream.Position -= 1;
    }

    /// <summary>
    /// C++'s <c>stream &gt;&gt; std::ws</c>: skips white space (space, <c>\t \n \v \f \r</c>); the end of the stream
    /// sets the eof flag only; a stream not good beforehand sets the fail flag.
    /// </summary>
    public void SkipWs()
    {
        if (!Good())
        {
            _fail = true;
            return;
        }

        try
        {
            for (;;)
            {
                int aByte = _stream.ReadByte();
                if (aByte < 0)
                {
                    _eof = true;
                    return;
                }
                if (aByte is not (' ' or '\t' or '\n' or '\v' or '\f' or '\r'))
                {
                    _stream.Position -= 1;
                    return;
                }
            }
        }
        catch (IOException)
        {
            _bad = true;
        }
    }
}
