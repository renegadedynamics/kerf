// Port of: src/FoundationClasses/TKernel/Message/Message_PrinterOStream.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_PrinterOStream.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Kerf.Engine.Foundation.MessageConsoleColor;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Foundation;

/// <summary>
/// A printer writing each message, followed by a new line, to a <see cref="TextWriter"/>: the console's standard
/// output by default, a file, or (a Kerf addition) any writer.
/// </summary>
/// <remarks>
/// Colours: OCCT colours console output by gravity (Windows console attributes or ANSI codes). The port does it through
/// <see cref="Console.ForegroundColor"/>, only when the writer is the console's own output or error writer, and never
/// in a browser. <see cref="Close"/> replaces the destructor: it flushes the writer, and closes it if the printer opened
/// it.
/// </remarks>
internal class PrinterOStream : Printer
{
    private TextWriter? _stream;
    private bool _isFile;
    private bool _toColorize;

    /// <summary>A printer to the console's standard output.</summary>
    public PrinterOStream(Gravity traceLevel = Info)
    {
        _stream = Console.Out;
        _isFile = false;
        _toColorize = true;
        _traceLevel = traceLevel;
    }

    /// <summary>
    /// A printer to the file, appended to or rewritten; "cout" and "cerr" (in any case) are the console's standard output
    /// and error. If the file can't be opened, the printer writes to the console's standard output, as OCCT's.
    /// </summary>
    public PrinterOStream(string fileName, bool toAppend, Gravity traceLevel = Info)
    {
        _stream = Console.Out;
        _isFile = false;
        _toColorize = true;
        _traceLevel = traceLevel;
        if (string.Equals(fileName, "cerr", StringComparison.OrdinalIgnoreCase))
        {
            _stream = Console.Error;
            return;
        }
        else if (string.Equals(fileName, "cout", StringComparison.OrdinalIgnoreCase))
        {
            _stream = Console.Out;
            return;
        }

        try
        {
            _stream = new StreamWriter(fileName, toAppend);
            _isFile = true;
            _toColorize = false;
        }
        catch (Exception exc) when (exc is IOException or UnauthorizedAccessException or ArgumentException
                                          or NotSupportedException)
        {
            _stream = Console.Out;
        }
    }

    /// <summary>A printer to the writer, without colours (a Kerf addition: a C++ caller passes a stream).</summary>
    public PrinterOStream(TextWriter stream, Gravity traceLevel = Info)
    {
        _stream = stream;
        _isFile = false;
        _toColorize = false;
        _traceLevel = traceLevel;
    }

    /// <summary>Flushes the writer, closes it if the printer opened it, and stops printing.</summary>
    public void Close()
    {
        if (_stream is null)
        {
            return;
        }
        TextWriter ostr = _stream;
        _stream = null;
        ostr.Flush();
        if (_isFile)
        {
            ostr.Dispose();
            _isFile = false;
        }
    }

    /// <summary>The writer (null once closed).</summary>
    public TextWriter? GetStream() => _stream;

    /// <summary>True if console output is coloured by gravity; true by default.</summary>
    public bool ToColorize() => _toColorize;

    /// <summary>Sets whether console output is coloured by gravity.</summary>
    public void SetToColorize(bool toColorize) => _toColorize = toColorize;

    protected override void SendCore(string theString, Gravity gravity)
    {
        if (gravity < _traceLevel || _stream is null)
        {
            return;
        }

        MessageConsoleColor color = ConsoleColor_Default;
        bool toIntense = false;
        if (_toColorize && !_isFile)
        {
            switch (gravity)
            {
                case Trace:
                    color = ConsoleColor_Yellow;
                    break;
                case Info:
                    color = ConsoleColor_Green;
                    toIntense = true;
                    break;
                case Warning:
                    color = ConsoleColor_Yellow;
                    toIntense = true;
                    break;
                case Alarm:
                    color = ConsoleColor_Red;
                    toIntense = true;
                    break;
                case Fail:
                    color = ConsoleColor_Red;
                    toIntense = true;
                    break;
            }
        }

        TextWriter stream = _stream;
        if (toIntense || color != ConsoleColor_Default)
        {
            SetConsoleTextColor(stream, color, toIntense);
            stream.Write(theString);
            SetConsoleTextColor(stream, ConsoleColor_Default, false);
        }
        else
        {
            stream.Write(theString);
        }
        stream.Write('\n');
    }

    /// <summary>
    /// Sets the colour of console text, when <paramref name="oStream"/> is the console's output or error writer
    /// (nothing otherwise, and nothing in a browser).
    /// </summary>
    public static void SetConsoleTextColor(TextWriter? oStream, MessageConsoleColor textColor, bool isIntenseText = false)
    {
        if (oStream is null || OperatingSystem.IsBrowser() || OperatingSystem.IsWasi()
            || (!ReferenceEquals(oStream, Console.Out) && !ReferenceEquals(oStream, Console.Error)))
        {
            return;
        }

        try
        {
            if (textColor == ConsoleColor_Default)
            {
                oStream.Flush();
                Console.ResetColor();
                return;
            }

            ConsoleColor color = textColor switch
            {
                ConsoleColor_Black => ConsoleColor.Black,
                ConsoleColor_White => isIntenseText ? ConsoleColor.White : ConsoleColor.Gray,
                ConsoleColor_Red => isIntenseText ? ConsoleColor.Red : ConsoleColor.DarkRed,
                ConsoleColor_Blue => isIntenseText ? ConsoleColor.Blue : ConsoleColor.DarkBlue,
                ConsoleColor_Green => isIntenseText ? ConsoleColor.Green : ConsoleColor.DarkGreen,
                ConsoleColor_Yellow => isIntenseText ? ConsoleColor.Yellow : ConsoleColor.DarkYellow,
                ConsoleColor_Cyan => isIntenseText ? ConsoleColor.Cyan : ConsoleColor.DarkCyan,
                _ => isIntenseText ? ConsoleColor.Magenta : ConsoleColor.DarkMagenta,
            };
            oStream.Flush();
            Console.ForegroundColor = color;
        }
        catch (Exception exc) when (exc is IOException or PlatformNotSupportedException)
        {
            // no console attached
        }
    }
}
