// Port of: src/FoundationClasses/TKernel/Message/Message_PrinterOStream.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_PrinterOStream.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_ConsoleColor;
using static Occt.Message_Gravity;

namespace Occt;

/// <summary>
/// A printer writing each message, followed by a new line, to a <see cref="TextWriter"/>: the console's standard
/// output by default, a file, or (an OCCT.NET addition) any writer.
/// </summary>
/// <remarks>
/// Colours: OCCT colours console output by gravity (Windows console attributes or ANSI codes). The port does it through
/// <see cref="Console.ForegroundColor"/>, only when the writer is the console's own output or error writer, and never
/// in a browser. <see cref="Close"/> replaces the destructor: it flushes the writer, and closes it if the printer opened
/// it.
/// </remarks>
public class Message_PrinterOStream : Message_Printer
{
    private TextWriter? myStream;
    private bool myIsFile;
    private bool myToColorize;

    /// <summary>A printer to the console's standard output.</summary>
    public Message_PrinterOStream(Message_Gravity theTraceLevel = Message_Info)
    {
        myStream = Console.Out;
        myIsFile = false;
        myToColorize = true;
        myTraceLevel = theTraceLevel;
    }

    /// <summary>
    /// A printer to the file, appended to or rewritten; "cout" and "cerr" (in any case) are the console's standard output
    /// and error. If the file can't be opened, the printer writes to the console's standard output, as OCCT's.
    /// </summary>
    public Message_PrinterOStream(string theFileName, bool theToAppend, Message_Gravity theTraceLevel = Message_Info)
    {
        myStream = Console.Out;
        myIsFile = false;
        myToColorize = true;
        myTraceLevel = theTraceLevel;
        if (string.Equals(theFileName, "cerr", StringComparison.OrdinalIgnoreCase))
        {
            myStream = Console.Error;
            return;
        }
        else if (string.Equals(theFileName, "cout", StringComparison.OrdinalIgnoreCase))
        {
            myStream = Console.Out;
            return;
        }

        try
        {
            myStream = new StreamWriter(theFileName, theToAppend);
            myIsFile = true;
            myToColorize = false;
        }
        catch (Exception anExc) when (anExc is IOException or UnauthorizedAccessException or ArgumentException
                                          or NotSupportedException)
        {
            myStream = Console.Out;
        }
    }

    /// <summary>A printer to the writer, without colours (an OCCT.NET addition: a C++ caller passes a stream).</summary>
    public Message_PrinterOStream(TextWriter theStream, Message_Gravity theTraceLevel = Message_Info)
    {
        myStream = theStream;
        myIsFile = false;
        myToColorize = false;
        myTraceLevel = theTraceLevel;
    }

    /// <summary>Flushes the writer, closes it if the printer opened it, and stops printing.</summary>
    public void Close()
    {
        if (myStream is null)
        {
            return;
        }
        TextWriter ostr = myStream;
        myStream = null;
        ostr.Flush();
        if (myIsFile)
        {
            ostr.Dispose();
            myIsFile = false;
        }
    }

    /// <summary>The writer (null once closed).</summary>
    public TextWriter? GetStream() => myStream;

    /// <summary>True if console output is coloured by gravity; true by default.</summary>
    public bool ToColorize() => myToColorize;

    /// <summary>Sets whether console output is coloured by gravity.</summary>
    public void SetToColorize(bool theToColorize) => myToColorize = theToColorize;

    protected override void send(string theString, Message_Gravity theGravity)
    {
        if (theGravity < myTraceLevel || myStream is null)
        {
            return;
        }

        Message_ConsoleColor aColor = Message_ConsoleColor_Default;
        bool toIntense = false;
        if (myToColorize && !myIsFile)
        {
            switch (theGravity)
            {
                case Message_Trace:
                    aColor = Message_ConsoleColor_Yellow;
                    break;
                case Message_Info:
                    aColor = Message_ConsoleColor_Green;
                    toIntense = true;
                    break;
                case Message_Warning:
                    aColor = Message_ConsoleColor_Yellow;
                    toIntense = true;
                    break;
                case Message_Alarm:
                    aColor = Message_ConsoleColor_Red;
                    toIntense = true;
                    break;
                case Message_Fail:
                    aColor = Message_ConsoleColor_Red;
                    toIntense = true;
                    break;
            }
        }

        TextWriter aStream = myStream;
        if (toIntense || aColor != Message_ConsoleColor_Default)
        {
            SetConsoleTextColor(aStream, aColor, toIntense);
            aStream.Write(theString);
            SetConsoleTextColor(aStream, Message_ConsoleColor_Default, false);
        }
        else
        {
            aStream.Write(theString);
        }
        aStream.Write('\n');
    }

    /// <summary>
    /// Sets the colour of console text, when <paramref name="theOStream"/> is the console's output or error writer
    /// (nothing otherwise, and nothing in a browser).
    /// </summary>
    public static void SetConsoleTextColor(TextWriter? theOStream, Message_ConsoleColor theTextColor, bool theIsIntenseText = false)
    {
        if (theOStream is null || OperatingSystem.IsBrowser() || OperatingSystem.IsWasi()
            || (!ReferenceEquals(theOStream, Console.Out) && !ReferenceEquals(theOStream, Console.Error)))
        {
            return;
        }

        try
        {
            if (theTextColor == Message_ConsoleColor_Default)
            {
                theOStream.Flush();
                Console.ResetColor();
                return;
            }

            ConsoleColor aColor = theTextColor switch
            {
                Message_ConsoleColor_Black => ConsoleColor.Black,
                Message_ConsoleColor_White => theIsIntenseText ? ConsoleColor.White : ConsoleColor.Gray,
                Message_ConsoleColor_Red => theIsIntenseText ? ConsoleColor.Red : ConsoleColor.DarkRed,
                Message_ConsoleColor_Blue => theIsIntenseText ? ConsoleColor.Blue : ConsoleColor.DarkBlue,
                Message_ConsoleColor_Green => theIsIntenseText ? ConsoleColor.Green : ConsoleColor.DarkGreen,
                Message_ConsoleColor_Yellow => theIsIntenseText ? ConsoleColor.Yellow : ConsoleColor.DarkYellow,
                Message_ConsoleColor_Cyan => theIsIntenseText ? ConsoleColor.Cyan : ConsoleColor.DarkCyan,
                _ => theIsIntenseText ? ConsoleColor.Magenta : ConsoleColor.DarkMagenta,
            };
            theOStream.Flush();
            Console.ForegroundColor = aColor;
        }
        catch (Exception anExc) when (anExc is IOException or PlatformNotSupportedException)
        {
            // no console attached
        }
    }
}
