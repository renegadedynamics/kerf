// Port of: src/FoundationClasses/TKernel/Standard/Standard_ReadLineBuffer.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>
/// Reads a byte stream line by line, through a buffer of a fixed size: the lines (ending with <c>\n</c> or
/// <c>\r\n</c>, the end of line removed) are returned as spans of the buffer, valid until the next call.
/// </summary>
/// <remarks>
/// <para>
/// OCCT returns a null-terminated <c>const char*</c>, null at the end of the stream: here <see cref="ReadLine(InputStream, out Span{byte})"/>
/// returns false there, and the line's span. OCCT writes the terminating <c>'\0'</c> over the end of line in the
/// buffer; the port does too, so the bytes match, but the span ends before it. The template's <c>FILE*</c> overload
/// is left out.
/// </para>
/// <para>
/// The buffer reads ahead: after a line, the stream's position is past it, at the end of the last block read.
/// </para>
/// </remarks>
public class ReadLineBuffer
{
    private byte[] _readBuffer;          // Temp read buffer
    private byte[] _readBufferLastStr;   // Part of last string of myReadBuffer
    private int _readBufferLastStrSize;  // its size (NCollection_LinearVector's)
    private bool _useReadBufferLastStr;  // Flag to use myReadBufferLastStr during next line reading
    private bool _isMultilineMode;       // Flag to process of the special multi-line case at the end of the line
    private bool _toPutGapInMultiline;   // Flag to put gap space while joining lines in multi-line syntax
    private int _bufferPos;              // Current position in myReadBuffer
    private int _bytesLastRead;          // The number of characters that were read last time from myReadBuffer.

    /// <summary>A buffer of <paramref name="maxBufferSizeBytes"/> bytes (the size of the blocks read).</summary>
    public ReadLineBuffer(int maxBufferSizeBytes)
    {
        _useReadBufferLastStr = false;
        _isMultilineMode = false;
        _toPutGapInMultiline = true;
        _bufferPos = 0;
        _bytesLastRead = 0;
        // allocate read buffer
        _readBuffer = new byte[maxBufferSizeBytes];
        _readBufferLastStr = [];
        _readBufferLastStrSize = 0;
    }

    /// <summary>Clears the buffer and the cached values.</summary>
    public void Clear()
    {
        _readBufferLastStrSize = 0;
        _useReadBufferLastStr = false;
        _isMultilineMode = false;
        _toPutGapInMultiline = true;
        _bufferPos = 0;
        _bytesLastRead = 0;
    }

    /// <summary>
    /// Reads the next line of the stream: false on an error or at the end of the stream (the stream's flags tell
    /// which), the line otherwise (empty lines too).
    /// </summary>
    public bool ReadLine(InputStream stream, out Span<byte> line) => ReadLine(stream, out line, out _);

    /// <summary>
    /// <see cref="ReadLine(InputStream, out Span{byte})"/>, with the number of bytes read from the stream by this
    /// call (0 if the line came from the buffer).
    /// </summary>
    public bool ReadLine(InputStream stream, out Span<byte> line, out long readData)
    {
        byte[]? resultArray = null;
        int resultStart = 0;
        bool isMultiline = false;
        int theLineLength = 0;
        readData = 0;
        line = default;

        while (resultArray is null)
        {
            if (_bufferPos == 0 || _bufferPos >= _bytesLastRead)
            {
                // read new chunk from the stream
                if (!ReadStream(stream, _readBuffer.Length, out _bytesLastRead))
                {
                    // error during file reading
                    break;
                }

                readData = _bytesLastRead;

                if (_bytesLastRead > 0)
                {
                    _bufferPos = 0;
                }
                else
                {
                    // end of the stream
                    if (_useReadBufferLastStr)
                    {
                        // OCCT returns the last string without a terminating '\0' (its callers may read past it); the
                        // span ends at its size
                        theLineLength = _readBufferLastStrSize;
                        resultArray = _readBufferLastStr;
                        resultStart = 0;
                        _useReadBufferLastStr = false;
                    }
                    break;
                }
            }

            int startLinePos = _bufferPos;
            bool isEndLineFound = false;

            // read next line from myReadBuffer
            while (_bufferPos < _bytesLastRead)
            {
                if (_isMultilineMode && _readBuffer[_bufferPos] == '\\')
                {
                    // multi-line syntax
                    if (_bufferPos + 1 == _bytesLastRead
                        || (_bufferPos + 2 == _bytesLastRead && _readBuffer[_bufferPos + 1] == '\r'))
                    {
                        isMultiline = true;
                        if (_toPutGapInMultiline)
                        {
                            _readBuffer[_bufferPos] = (byte)' ';
                            if (_bufferPos + 1 != _bytesLastRead)
                            {
                                _readBuffer[_bufferPos + 1] = (byte)' ';
                            }
                        }
                    }
                    else if (_readBuffer[_bufferPos + 1] == '\n'
                             || (_readBuffer[_bufferPos + 1] == '\r'
                                 && BufferByte(_bufferPos + 2) == '\n'))
                    {
                        int bufferPos = _bufferPos;
                        _bufferPos = bufferPos + (_readBuffer[bufferPos + 1] == '\r' ? 2 : 1);
                        if (_toPutGapInMultiline)
                        {
                            _readBuffer[bufferPos] = (byte)' ';
                            ++bufferPos;
                        }

                        if (_useReadBufferLastStr)
                        {
                            AppendLastStr(startLinePos, bufferPos - startLinePos);
                        }
                        else
                        {
                            _readBufferLastStrSize = 0;
                            AppendLastStr(startLinePos, bufferPos - startLinePos);
                            _useReadBufferLastStr = true;
                        }

                        startLinePos = _bufferPos + 1;
                    }
                }
                else if (_readBuffer[_bufferPos] == '\n')
                {
                    if (!isMultiline)
                    {
                        isEndLineFound = true;
                    }
                    else if (_bufferPos == 1 && _readBuffer[0] == '\r')
                    {
                        --_readBufferLastStrSize;
                        startLinePos += 2;
                        isMultiline = false;
                    }
                    else if (_bufferPos == 0)
                    {
                        startLinePos += 1;
                        if (_readBufferLastStr[_readBufferLastStrSize - 1] == '\\')
                        {
                            --_readBufferLastStrSize;
                        }
                        else
                        {
                            _readBufferLastStrSize -= 2;
                        }
                        isMultiline = false;
                    }
                }

                ++_bufferPos;

                if (isEndLineFound)
                {
                    break;
                }
            }

            if (isEndLineFound)
            {
                if (_useReadBufferLastStr)
                {
                    // append current string to the last "unfinished" string of the previous chunk
                    AppendLastStr(startLinePos, _bufferPos - startLinePos);
                    _useReadBufferLastStr = false;
                    theLineLength = _readBufferLastStrSize;
                    resultArray = _readBufferLastStr;
                    resultStart = 0;
                }
                else
                {
                    if (_readBufferLastStrSize != 0)
                    {
                        _readBufferLastStrSize = 0;
                    }
                    theLineLength = _bufferPos - startLinePos;
                    resultArray = _readBuffer;
                    resultStart = startLinePos;
                }
                // make string null terminated by replacing '\n' or '\r' (before '\n') symbol to null
                // character.
                if (theLineLength > 1 && resultArray[resultStart + theLineLength - 2] == '\r')
                {
                    resultArray[resultStart + theLineLength - 2] = 0;
                    theLineLength -= 2;
                }
                else
                {
                    resultArray[resultStart + theLineLength - 1] = 0;
                    theLineLength -= 1;
                }
            }
            else
            {
                // save "unfinished" part of string to additional buffer
                if (startLinePos != _bufferPos)
                {
                    if (_useReadBufferLastStr)
                    {
                        AppendLastStr(startLinePos, _bufferPos - startLinePos);
                    }
                    else
                    {
                        _readBufferLastStrSize = 0;
                        AppendLastStr(startLinePos, _bufferPos - startLinePos);
                        _useReadBufferLastStr = true;
                    }
                }
            }
        }

        if (resultArray is null)
        {
            return false;
        }
        line = resultArray.AsSpan(resultStart, theLineLength);
        return true;
    }

    /// <summary>
    /// The bytes read from the stream that no line returned yet: the stream's position less this is where the next
    /// line starts (an addition, for readers that go on reading the stream themselves).
    /// </summary>
    public int UnreadCount => (_bufferPos < _bytesLastRead ? _bytesLastRead - _bufferPos : 0)
                              + (_useReadBufferLastStr ? _readBufferLastStrSize : 0);

    /// <summary>
    /// True when the multi-line mode is on (false by default): lines ending with <c>\</c> are joined with the next.
    /// </summary>
    public bool IsMultilineMode => _isMultilineMode;

    /// <summary>
    /// True (the default) to put a space where multi-line syntax joins lines: <c>1/2/3\</c> and <c>4/5/6</c> become
    /// <c>1/2/3 4/5/6</c>, and <c>1/2/34/5/6</c> otherwise.
    /// </summary>
    public bool ToPutGapInMultiline() => _toPutGapInMultiline;

    /// <summary>Sets or unsets the multi-line mode, and whether to put a space where lines are joined.</summary>
    public void SetMultilineMode(bool multilineMode, bool toPutGap = true)
    {
        _isMultilineMode = multilineMode;
        _toPutGapInMultiline = toPutGap;
    }

    // Read from stl stream: true if reading was finished without errors.
    private bool ReadStream(InputStream stream, int len, out int readLen)
    {
        readLen = (int)stream.Read(_readBuffer.AsSpan(0, len));
        return !stream.Bad();
    }

    // OCCT reads myReadBuffer[myBufferPos + 2] past the bytes read (and possibly past the buffer) in multi-line mode:
    // the port reads 0 beyond the buffer
    private int BufferByte(int index) => index < _readBuffer.Length ? _readBuffer[index] : 0;

    // NCollection_LinearVector's Resize(aIns + aCnt) and memcpy of myReadBuffer[theStart, theStart + theCount)
    private void AppendLastStr(int start, int count)
    {
        int newSize = _readBufferLastStrSize + count;
        if (newSize > _readBufferLastStr.Length)
        {
            Array.Resize(ref _readBufferLastStr, Math.Max(newSize, _readBufferLastStr.Length * 2));
        }
        Array.Copy(_readBuffer, start, _readBufferLastStr, _readBufferLastStrSize, count);
        _readBufferLastStrSize = newSize;
    }
}
