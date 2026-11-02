// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The behaviour of C++'s std::istream (OCCT's Standard_IStream typedef) in binary mode, for ports that read bytes.

namespace Occt;

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
public sealed class Standard_IStream
{
    private readonly Stream myStream;
    private readonly long myOrigin;
    private bool myEof;
    private bool myFail;
    private bool myBad;
    private long myGCount;

    /// <summary>Reads <paramref name="theStream"/> from its current position.</summary>
    /// <exception cref="ArgumentException">The stream can't read or can't seek.</exception>
    public Standard_IStream(Stream theStream)
    {
        ArgumentNullException.ThrowIfNull(theStream);
        if (!theStream.CanRead || !theStream.CanSeek)
        {
            throw new ArgumentException("Standard_IStream: the stream must be readable and seekable", nameof(theStream));
        }

        myStream = theStream;
        myOrigin = theStream.Position;
    }

    /// <summary>The underlying stream.</summary>
    public Stream BaseStream => myStream;

    /// <summary>C++'s <c>good()</c>: no flag set.</summary>
    public bool Good() => !myEof && !myFail && !myBad;

    /// <summary>C++'s <c>eof()</c>.</summary>
    public bool Eof() => myEof;

    /// <summary>C++'s <c>fail()</c>: the fail or the bad flag (also C++'s <c>!stream</c>).</summary>
    public bool Fail() => myFail || myBad;

    /// <summary>C++'s <c>bad()</c>.</summary>
    public bool Bad() => myBad;

    /// <summary>C++'s <c>clear()</c>: clears every flag.</summary>
    public void Clear()
    {
        myEof = false;
        myFail = false;
        myBad = false;
    }

    /// <summary>C++'s <c>gcount()</c>: the number of bytes the last unformatted input read.</summary>
    public long GCount() => myGCount;

    /// <summary>
    /// C++'s <c>read(s, n)</c>: reads up to the span's length. Fewer bytes (the end of the stream) set the eof and
    /// fail flags; a stream not good beforehand reads nothing and sets the fail flag. Returns <see cref="GCount"/>.
    /// </summary>
    public long Read(Span<byte> theBuffer)
    {
        myGCount = 0;
        if (!Good())
        {
            myFail = true;
            return 0;
        }

        int aNbRead = 0;
        try
        {
            while (aNbRead < theBuffer.Length)
            {
                int aNb = myStream.Read(theBuffer[aNbRead..]);
                if (aNb <= 0)
                {
                    break;
                }
                aNbRead += aNb;
            }
        }
        catch (IOException)
        {
            myBad = true;
        }

        myGCount = aNbRead;
        if (aNbRead < theBuffer.Length && !myBad)
        {
            myEof = true;
            myFail = true;
        }
        return aNbRead;
    }

    /// <summary>C++'s <c>tellg()</c>: the position, or -1 if <see cref="Fail"/>.</summary>
    public long TellG() => Fail() ? -1 : myStream.Position - myOrigin;

    /// <summary>
    /// C++'s <c>seekg(pos)</c> (C++11): clears the eof flag, then does nothing if <see cref="Fail"/>; a position out of
    /// the stream sets the fail flag.
    /// </summary>
    public void SeekG(long thePos)
    {
        myEof = false;
        if (Fail())
        {
            return;
        }

        if (thePos < 0 || thePos > myStream.Length - myOrigin)
        {
            myFail = true;
            return;
        }
        myStream.Position = myOrigin + thePos;
    }

    /// <summary>C++'s <c>seekg(0, end)</c>.</summary>
    public void SeekGEnd()
    {
        myEof = false;
        if (Fail())
        {
            return;
        }
        myStream.Position = myStream.Length;
    }

    /// <summary>
    /// C++'s <c>unget()</c> (C++11): clears the eof flag, then puts the last byte back; at the start (or if not good)
    /// sets the bad (fail) flag.
    /// </summary>
    public void Unget()
    {
        myGCount = 0;
        myEof = false;
        if (!Good())
        {
            myFail = true;
            return;
        }
        if (myStream.Position <= myOrigin)
        {
            myBad = true;
            return;
        }
        myStream.Position -= 1;
    }

    /// <summary>
    /// C++'s <c>stream &gt;&gt; std::ws</c>: skips white space (space, <c>\t \n \v \f \r</c>); the end of the stream
    /// sets the eof flag only; a stream not good beforehand sets the fail flag.
    /// </summary>
    public void SkipWs()
    {
        if (!Good())
        {
            myFail = true;
            return;
        }

        try
        {
            for (;;)
            {
                int aByte = myStream.ReadByte();
                if (aByte < 0)
                {
                    myEof = true;
                    return;
                }
                if (aByte is not (' ' or '\t' or '\n' or '\v' or '\f' or '\r'))
                {
                    myStream.Position -= 1;
                    return;
                }
            }
        }
        catch (IOException)
        {
            myBad = true;
        }
    }
}
