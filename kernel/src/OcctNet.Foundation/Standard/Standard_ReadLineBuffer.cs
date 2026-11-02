// Port of: src/FoundationClasses/TKernel/Standard/Standard_ReadLineBuffer.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Reads a byte stream line by line, through a buffer of a fixed size: the lines (ending with <c>\n</c> or
/// <c>\r\n</c>, the end of line removed) are returned as spans of the buffer, valid until the next call.
/// </summary>
/// <remarks>
/// <para>
/// OCCT returns a null-terminated <c>const char*</c>, null at the end of the stream: here <see cref="ReadLine(Standard_IStream, out Span{byte})"/>
/// returns false there, and the line's span. OCCT writes the terminating <c>'\0'</c> over the end of line in the
/// buffer; the port does too, so the bytes match, but the span ends before it. The template's <c>FILE*</c> overload
/// is left out.
/// </para>
/// <para>
/// The buffer reads ahead: after a line, the stream's position is past it, at the end of the last block read.
/// </para>
/// </remarks>
public class Standard_ReadLineBuffer
{
    private byte[] myReadBuffer;          // Temp read buffer
    private byte[] myReadBufferLastStr;   // Part of last string of myReadBuffer
    private int myReadBufferLastStrSize;  // its size (NCollection_LinearVector's)
    private bool myUseReadBufferLastStr;  // Flag to use myReadBufferLastStr during next line reading
    private bool myIsMultilineMode;       // Flag to process of the special multi-line case at the end of the line
    private bool myToPutGapInMultiline;   // Flag to put gap space while joining lines in multi-line syntax
    private int myBufferPos;              // Current position in myReadBuffer
    private int myBytesLastRead;          // The number of characters that were read last time from myReadBuffer.

    /// <summary>A buffer of <paramref name="theMaxBufferSizeBytes"/> bytes (the size of the blocks read).</summary>
    public Standard_ReadLineBuffer(int theMaxBufferSizeBytes)
    {
        myUseReadBufferLastStr = false;
        myIsMultilineMode = false;
        myToPutGapInMultiline = true;
        myBufferPos = 0;
        myBytesLastRead = 0;
        // allocate read buffer
        myReadBuffer = new byte[theMaxBufferSizeBytes];
        myReadBufferLastStr = [];
        myReadBufferLastStrSize = 0;
    }

    /// <summary>Clears the buffer and the cached values.</summary>
    public void Clear()
    {
        myReadBufferLastStrSize = 0;
        myUseReadBufferLastStr = false;
        myIsMultilineMode = false;
        myToPutGapInMultiline = true;
        myBufferPos = 0;
        myBytesLastRead = 0;
    }

    /// <summary>
    /// Reads the next line of the stream: false on an error or at the end of the stream (the stream's flags tell
    /// which), the line otherwise (empty lines too).
    /// </summary>
    public bool ReadLine(Standard_IStream theStream, out Span<byte> theLine) => ReadLine(theStream, out theLine, out _);

    /// <summary>
    /// <see cref="ReadLine(Standard_IStream, out Span{byte})"/>, with the number of bytes read from the stream by this
    /// call (0 if the line came from the buffer).
    /// </summary>
    public bool ReadLine(Standard_IStream theStream, out Span<byte> theLine, out long theReadData)
    {
        byte[]? aResultArray = null;
        int aResultStart = 0;
        bool isMultiline = false;
        int theLineLength = 0;
        theReadData = 0;
        theLine = default;

        while (aResultArray is null)
        {
            if (myBufferPos == 0 || myBufferPos >= myBytesLastRead)
            {
                // read new chunk from the stream
                if (!readStream(theStream, myReadBuffer.Length, out myBytesLastRead))
                {
                    // error during file reading
                    break;
                }

                theReadData = myBytesLastRead;

                if (myBytesLastRead > 0)
                {
                    myBufferPos = 0;
                }
                else
                {
                    // end of the stream
                    if (myUseReadBufferLastStr)
                    {
                        // OCCT returns the last string without a terminating '\0' (its callers may read past it); the
                        // span ends at its size
                        theLineLength = myReadBufferLastStrSize;
                        aResultArray = myReadBufferLastStr;
                        aResultStart = 0;
                        myUseReadBufferLastStr = false;
                    }
                    break;
                }
            }

            int aStartLinePos = myBufferPos;
            bool isEndLineFound = false;

            // read next line from myReadBuffer
            while (myBufferPos < myBytesLastRead)
            {
                if (myIsMultilineMode && myReadBuffer[myBufferPos] == '\\')
                {
                    // multi-line syntax
                    if (myBufferPos + 1 == myBytesLastRead
                        || (myBufferPos + 2 == myBytesLastRead && myReadBuffer[myBufferPos + 1] == '\r'))
                    {
                        isMultiline = true;
                        if (myToPutGapInMultiline)
                        {
                            myReadBuffer[myBufferPos] = (byte)' ';
                            if (myBufferPos + 1 != myBytesLastRead)
                            {
                                myReadBuffer[myBufferPos + 1] = (byte)' ';
                            }
                        }
                    }
                    else if (myReadBuffer[myBufferPos + 1] == '\n'
                             || (myReadBuffer[myBufferPos + 1] == '\r'
                                 && BufferByte(myBufferPos + 2) == '\n'))
                    {
                        int aBufferPos = myBufferPos;
                        myBufferPos = aBufferPos + (myReadBuffer[aBufferPos + 1] == '\r' ? 2 : 1);
                        if (myToPutGapInMultiline)
                        {
                            myReadBuffer[aBufferPos] = (byte)' ';
                            ++aBufferPos;
                        }

                        if (myUseReadBufferLastStr)
                        {
                            AppendLastStr(aStartLinePos, aBufferPos - aStartLinePos);
                        }
                        else
                        {
                            myReadBufferLastStrSize = 0;
                            AppendLastStr(aStartLinePos, aBufferPos - aStartLinePos);
                            myUseReadBufferLastStr = true;
                        }

                        aStartLinePos = myBufferPos + 1;
                    }
                }
                else if (myReadBuffer[myBufferPos] == '\n')
                {
                    if (!isMultiline)
                    {
                        isEndLineFound = true;
                    }
                    else if (myBufferPos == 1 && myReadBuffer[0] == '\r')
                    {
                        --myReadBufferLastStrSize;
                        aStartLinePos += 2;
                        isMultiline = false;
                    }
                    else if (myBufferPos == 0)
                    {
                        aStartLinePos += 1;
                        if (myReadBufferLastStr[myReadBufferLastStrSize - 1] == '\\')
                        {
                            --myReadBufferLastStrSize;
                        }
                        else
                        {
                            myReadBufferLastStrSize -= 2;
                        }
                        isMultiline = false;
                    }
                }

                ++myBufferPos;

                if (isEndLineFound)
                {
                    break;
                }
            }

            if (isEndLineFound)
            {
                if (myUseReadBufferLastStr)
                {
                    // append current string to the last "unfinished" string of the previous chunk
                    AppendLastStr(aStartLinePos, myBufferPos - aStartLinePos);
                    myUseReadBufferLastStr = false;
                    theLineLength = myReadBufferLastStrSize;
                    aResultArray = myReadBufferLastStr;
                    aResultStart = 0;
                }
                else
                {
                    if (myReadBufferLastStrSize != 0)
                    {
                        myReadBufferLastStrSize = 0;
                    }
                    theLineLength = myBufferPos - aStartLinePos;
                    aResultArray = myReadBuffer;
                    aResultStart = aStartLinePos;
                }
                // make string null terminated by replacing '\n' or '\r' (before '\n') symbol to null
                // character.
                if (theLineLength > 1 && aResultArray[aResultStart + theLineLength - 2] == '\r')
                {
                    aResultArray[aResultStart + theLineLength - 2] = 0;
                    theLineLength -= 2;
                }
                else
                {
                    aResultArray[aResultStart + theLineLength - 1] = 0;
                    theLineLength -= 1;
                }
            }
            else
            {
                // save "unfinished" part of string to additional buffer
                if (aStartLinePos != myBufferPos)
                {
                    if (myUseReadBufferLastStr)
                    {
                        AppendLastStr(aStartLinePos, myBufferPos - aStartLinePos);
                    }
                    else
                    {
                        myReadBufferLastStrSize = 0;
                        AppendLastStr(aStartLinePos, myBufferPos - aStartLinePos);
                        myUseReadBufferLastStr = true;
                    }
                }
            }
        }

        if (aResultArray is null)
        {
            return false;
        }
        theLine = aResultArray.AsSpan(aResultStart, theLineLength);
        return true;
    }

    /// <summary>
    /// True when the multi-line mode is on (false by default): lines ending with <c>\</c> are joined with the next.
    /// </summary>
    public bool IsMultilineMode() => myIsMultilineMode;

    /// <summary>
    /// True (the default) to put a space where multi-line syntax joins lines: <c>1/2/3\</c> and <c>4/5/6</c> become
    /// <c>1/2/3 4/5/6</c>, and <c>1/2/34/5/6</c> otherwise.
    /// </summary>
    public bool ToPutGapInMultiline() => myToPutGapInMultiline;

    /// <summary>Sets or unsets the multi-line mode, and whether to put a space where lines are joined.</summary>
    public void SetMultilineMode(bool theMultilineMode, bool theToPutGap = true)
    {
        myIsMultilineMode = theMultilineMode;
        myToPutGapInMultiline = theToPutGap;
    }

    // Read from stl stream: true if reading was finished without errors.
    private bool readStream(Standard_IStream theStream, int theLen, out int theReadLen)
    {
        theReadLen = (int)theStream.Read(myReadBuffer.AsSpan(0, theLen));
        return !theStream.Bad();
    }

    // OCCT reads myReadBuffer[myBufferPos + 2] past the bytes read (and possibly past the buffer) in multi-line mode:
    // the port reads 0 beyond the buffer
    private int BufferByte(int theIndex) => theIndex < myReadBuffer.Length ? myReadBuffer[theIndex] : 0;

    // NCollection_LinearVector's Resize(aIns + aCnt) and memcpy of myReadBuffer[theStart, theStart + theCount)
    private void AppendLastStr(int theStart, int theCount)
    {
        int aNewSize = myReadBufferLastStrSize + theCount;
        if (aNewSize > myReadBufferLastStr.Length)
        {
            Array.Resize(ref myReadBufferLastStr, Math.Max(aNewSize, myReadBufferLastStr.Length * 2));
        }
        Array.Copy(myReadBuffer, theStart, myReadBufferLastStr, myReadBufferLastStrSize, theCount);
        myReadBufferLastStrSize = aNewSize;
    }
}
