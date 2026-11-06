// Port of: src/FoundationClasses/TKernel/Message/Message_Messenger.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Messenger.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Foundation;

/// <summary>
/// Dispatches every message it receives to each of its printers, so that a library can issue messages without knowing
/// how they are shown.
/// </summary>
/// <remarks>
/// <para>
/// The printers are set when the messenger is made (<see cref="Messenger()"/> has one printer to the console,
/// as OCCT's; <see cref="Messenger(Printer)"/> has the given one). The printer list's mutators
/// (<see cref="AddPrinter"/>, <see cref="RemovePrinter"/>, <see cref="RemovePrinters"/>, <see cref="ChangePrinters"/>)
/// are internal, because the process-wide <see cref="Message.DefaultMessenger"/> is one of these (decision 493);
/// <see cref="Printers"/> returns a copy of the list.
/// </para>
/// <para>
/// TCollection's strings are <see cref="string"/>, so OCCT's <c>Send</c> overloads by string type are one,
/// <see cref="Send(string, Gravity)"/>; OCCT's <c>Send(const Standard_SStream&amp;)</c> takes a
/// <see cref="StringBuilder"/>.
/// </para>
/// </remarks>
internal class Messenger : Transient
{
    /// <summary>
    /// A message built piece by piece (OCCT's stream buffer, used as <c>Messenger->SendFail() &lt;&lt; "a" &lt;&lt; 1;</c>),
    /// sent to its messenger when flushed or disposed.
    /// </summary>
    /// <remarks>
    /// C++'s <c>operator&lt;&lt;</c> is <see cref="Append(string)"/> and its overloads, which return the buffer for
    /// chaining. C++ sends the message when the buffer is destroyed; in C# ported code ends the statement with
    /// <see cref="Flush"/> (<c>Message.SendFail().Append("a").Append(1).Flush();</c>) or disposes the buffer
    /// (<c>using var aSender = Message.SendInfo();</c>). The manipulators <c>std::endl</c>, <c>std::ends</c> and
    /// <c>std::flush</c> are <see cref="Endl"/>, which sends even an empty message. An empty message is not sent
    /// otherwise. Reals are written as C++ streams write them (<c>%g</c>, 6 digits).
    /// </remarks>
    public sealed class StreamBuffer : IDisposable
    {
        private readonly Messenger? _messenger;
        private readonly Gravity _gravity;
        private readonly StringBuilder _stream = new();

        internal StreamBuffer(Messenger? messenger, Gravity gravity)
        {
            _messenger = messenger;
            _gravity = gravity;
        }

        /// <summary>Sends the collected text to the messenger, if there is any or <paramref name="doForce"/> is set.</summary>
        public void Flush(bool doForce = false)
        {
            if (doForce || _stream.Length != 0)
            {
                _messenger?.Send(_stream, _gravity);
                _stream.Clear(); // empty the buffer for possible reuse
            }
        }

        /// <summary>Flushes the buffer (the C++ destructor).</summary>
        public void Dispose() => Flush();

        public StreamBuffer Append(string? arg)
        {
            _stream.Append(arg);
            return this;
        }

        public StreamBuffer Append(char arg)
        {
            _stream.Append(arg);
            return this;
        }

        public StreamBuffer Append(int arg)
        {
            _stream.Append(arg.ToString(CultureInfo.InvariantCulture));
            return this;
        }

        public StreamBuffer Append(long arg)
        {
            _stream.Append(arg.ToString(CultureInfo.InvariantCulture));
            return this;
        }

        public StreamBuffer Append(double arg)
        {
            _stream.Append(StreamUtilities.FormatReal(arg, 6));
            return this;
        }

        public StreamBuffer Append(bool arg)
        {
            _stream.Append(arg ? '1' : '0');
            return this;
        }

        /// <summary>Appends an object's <see cref="object.ToString"/>.</summary>
        public StreamBuffer Append(object? arg)
        {
            _stream.Append(Convert.ToString(arg, CultureInfo.InvariantCulture));
            return this;
        }

        /// <summary>The manipulators <c>std::endl</c>, <c>std::ends</c>, <c>std::flush</c>: sends the message, even empty.</summary>
        public StreamBuffer Endl()
        {
            Flush(true);
            return this;
        }

        /// <summary>The text collected so far.</summary>
        public StringBuilder Stream() => _stream;

        /// <summary>The messenger the buffer sends to.</summary>
        public Messenger? Messenger() => _messenger;
    }

    private readonly NCollection_Sequence<Printer> _printers = new();

    /// <summary>A messenger with one printer to the console's standard output, as OCCT's.</summary>
    public Messenger()
    {
        AddPrinter(new PrinterOStream());
    }

    /// <summary>A messenger with the one printer.</summary>
    public Messenger(Printer printer)
    {
        AddPrinter(printer);
    }

    // A messenger without printers (Message.DefaultMessenger).
    private protected Messenger(bool isEmpty)
    {
        _ = isEmpty;
    }

    internal static Messenger Empty() => new(true);

    /// <summary>Adds a printer, unless it is already in the list. True if added.</summary>
    internal bool AddPrinter(Printer printer)
    {
        // check whether printer is already in the list
        foreach (Printer aPrinter in _printers)
        {
            if (ReferenceEquals(aPrinter, printer))
            {
                return false;
            }
        }
        _printers.Append(printer);
        return true;
    }

    /// <summary>Removes the printer. True if it was in the list.</summary>
    internal bool RemovePrinter(Printer printer)
    {
        // find printer in the list
        for (int i = 1; i <= _printers.Length(); ++i)
        {
            if (ReferenceEquals(_printers.Value(i), printer))
            {
                _printers.Remove(i);
                return true;
            }
        }
        return false;
    }

    /// <summary>Removes the printers of the type (and derived types). Returns how many were removed.</summary>
    internal int RemovePrinters(Type type)
    {
        // remove printers from the list
        int nb = 0;
        for (int i = 1; i <= _printers.Length();)
        {
            Printer? printer = _printers.Value(i);
            if (printer is not null && printer.IsKind(type))
            {
                _printers.Remove(i);
                nb++;
            }
            else
            {
                i++;
            }
        }
        return nb;
    }

    /// <summary>A copy of the list of printers.</summary>
    public NCollection_Sequence<Printer> Printers() => new(_printers);

    /// <summary>The list of printers itself, to change it.</summary>
    internal NCollection_Sequence<Printer> ChangePrinters() => _printers;

    /// <summary>Sends the message to every printer.</summary>
    public void Send(string theString, Gravity gravity = Warning)
    {
        foreach (Printer? printer in _printers)
        {
            printer?.Send(theString, gravity);
        }
    }

    /// <summary>Sends the text of the stream to every printer.</summary>
    public void Send(StringBuilder stream, Gravity gravity = Warning)
    {
        foreach (Printer? printer in _printers)
        {
            printer?.SendStringStream(stream, gravity);
        }
    }

    /// <summary>Sends the object to every printer.</summary>
    public void Send(Transient? theObject, Gravity gravity = Warning)
    {
        foreach (Printer? printer in _printers)
        {
            printer?.SendObject(theObject, gravity);
        }
    }

    /// <summary>A stream buffer for a message of the gravity.</summary>
    public StreamBuffer Send(Gravity gravity) => new(this, gravity);

    /// <summary>A stream buffer for a Fail message.</summary>
    public StreamBuffer SendFail() => Send(Fail);

    /// <summary>A stream buffer for an Alarm message.</summary>
    public StreamBuffer SendAlarm() => Send(Alarm);

    /// <summary>A stream buffer for a Warning message.</summary>
    public StreamBuffer SendWarning() => Send(Warning);

    /// <summary>A stream buffer for an Info message.</summary>
    public StreamBuffer SendInfo() => Send(Info);

    /// <summary>A stream buffer for a Trace message.</summary>
    public StreamBuffer SendTrace() => Send(Trace);

    public void SendFail(string message) => Send(message, Fail);

    public void SendAlarm(string message) => Send(message, Alarm);

    public void SendWarning(string message) => Send(message, Warning);

    public void SendInfo(string message) => Send(message, Info);

    public void SendTrace(string message) => Send(message, Trace);
}
