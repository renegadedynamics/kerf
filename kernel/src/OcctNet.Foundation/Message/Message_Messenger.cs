// Port of: src/FoundationClasses/TKernel/Message/Message_Messenger.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Messenger.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using static Occt.Message_Gravity;

namespace Occt;

/// <summary>
/// Dispatches every message it receives to each of its printers, so that a library can issue messages without knowing
/// how they are shown.
/// </summary>
/// <remarks>
/// <para>
/// The printers are set when the messenger is made (<see cref="Message_Messenger()"/> has one printer to the console,
/// as OCCT's; <see cref="Message_Messenger(Message_Printer)"/> has the given one). The printer list's mutators
/// (<see cref="AddPrinter"/>, <see cref="RemovePrinter"/>, <see cref="RemovePrinters"/>, <see cref="ChangePrinters"/>)
/// are internal, because the process-wide <see cref="Message.DefaultMessenger"/> is one of these (decision 493);
/// <see cref="Printers"/> returns a copy of the list.
/// </para>
/// <para>
/// TCollection's strings are <see cref="string"/>, so OCCT's <c>Send</c> overloads by string type are one,
/// <see cref="Send(string, Message_Gravity)"/>; OCCT's <c>Send(const Standard_SStream&amp;)</c> takes a
/// <see cref="StringBuilder"/>.
/// </para>
/// </remarks>
public class Message_Messenger : Standard_Transient
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
        private readonly Message_Messenger? myMessenger;
        private readonly Message_Gravity myGravity;
        private readonly StringBuilder myStream = new();

        internal StreamBuffer(Message_Messenger? theMessenger, Message_Gravity theGravity)
        {
            myMessenger = theMessenger;
            myGravity = theGravity;
        }

        /// <summary>Sends the collected text to the messenger, if there is any or <paramref name="doForce"/> is set.</summary>
        public void Flush(bool doForce = false)
        {
            if (doForce || myStream.Length != 0)
            {
                myMessenger?.Send(myStream, myGravity);
                myStream.Clear(); // empty the buffer for possible reuse
            }
        }

        /// <summary>Flushes the buffer (the C++ destructor).</summary>
        public void Dispose() => Flush();

        public StreamBuffer Append(string? theArg)
        {
            myStream.Append(theArg);
            return this;
        }

        public StreamBuffer Append(char theArg)
        {
            myStream.Append(theArg);
            return this;
        }

        public StreamBuffer Append(int theArg)
        {
            myStream.Append(theArg.ToString(CultureInfo.InvariantCulture));
            return this;
        }

        public StreamBuffer Append(long theArg)
        {
            myStream.Append(theArg.ToString(CultureInfo.InvariantCulture));
            return this;
        }

        public StreamBuffer Append(double theArg)
        {
            myStream.Append(Standard_Stream.FormatReal(theArg, 6));
            return this;
        }

        public StreamBuffer Append(bool theArg)
        {
            myStream.Append(theArg ? '1' : '0');
            return this;
        }

        /// <summary>Appends an object's <see cref="object.ToString"/>.</summary>
        public StreamBuffer Append(object? theArg)
        {
            myStream.Append(Convert.ToString(theArg, CultureInfo.InvariantCulture));
            return this;
        }

        /// <summary>The manipulators <c>std::endl</c>, <c>std::ends</c>, <c>std::flush</c>: sends the message, even empty.</summary>
        public StreamBuffer Endl()
        {
            Flush(true);
            return this;
        }

        /// <summary>The text collected so far.</summary>
        public StringBuilder Stream() => myStream;

        /// <summary>The messenger the buffer sends to.</summary>
        public Message_Messenger? Messenger() => myMessenger;
    }

    private readonly NCollection_Sequence<Message_Printer> myPrinters = new();

    /// <summary>A messenger with one printer to the console's standard output, as OCCT's.</summary>
    public Message_Messenger()
    {
        AddPrinter(new Message_PrinterOStream());
    }

    /// <summary>A messenger with the one printer.</summary>
    public Message_Messenger(Message_Printer thePrinter)
    {
        AddPrinter(thePrinter);
    }

    // A messenger without printers (Message.DefaultMessenger).
    private protected Message_Messenger(bool theIsEmpty)
    {
        _ = theIsEmpty;
    }

    internal static Message_Messenger Empty() => new(true);

    /// <summary>Adds a printer, unless it is already in the list. True if added.</summary>
    internal bool AddPrinter(Message_Printer thePrinter)
    {
        // check whether printer is already in the list
        foreach (Message_Printer aPrinter in myPrinters)
        {
            if (ReferenceEquals(aPrinter, thePrinter))
            {
                return false;
            }
        }
        myPrinters.Append(thePrinter);
        return true;
    }

    /// <summary>Removes the printer. True if it was in the list.</summary>
    internal bool RemovePrinter(Message_Printer thePrinter)
    {
        // find printer in the list
        for (int i = 1; i <= myPrinters.Length(); ++i)
        {
            if (ReferenceEquals(myPrinters.Value(i), thePrinter))
            {
                myPrinters.Remove(i);
                return true;
            }
        }
        return false;
    }

    /// <summary>Removes the printers of the type (and derived types). Returns how many were removed.</summary>
    internal int RemovePrinters(Type theType)
    {
        // remove printers from the list
        int nb = 0;
        for (int i = 1; i <= myPrinters.Length();)
        {
            Message_Printer? aPrinter = myPrinters.Value(i);
            if (aPrinter is not null && aPrinter.IsKind(theType))
            {
                myPrinters.Remove(i);
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
    public NCollection_Sequence<Message_Printer> Printers() => new(myPrinters);

    /// <summary>The list of printers itself, to change it.</summary>
    internal NCollection_Sequence<Message_Printer> ChangePrinters() => myPrinters;

    /// <summary>Sends the message to every printer.</summary>
    public void Send(string theString, Message_Gravity theGravity = Message_Warning)
    {
        foreach (Message_Printer? aPrinter in myPrinters)
        {
            aPrinter?.Send(theString, theGravity);
        }
    }

    /// <summary>Sends the text of the stream to every printer.</summary>
    public void Send(StringBuilder theStream, Message_Gravity theGravity = Message_Warning)
    {
        foreach (Message_Printer? aPrinter in myPrinters)
        {
            aPrinter?.SendStringStream(theStream, theGravity);
        }
    }

    /// <summary>Sends the object to every printer.</summary>
    public void Send(Standard_Transient? theObject, Message_Gravity theGravity = Message_Warning)
    {
        foreach (Message_Printer? aPrinter in myPrinters)
        {
            aPrinter?.SendObject(theObject, theGravity);
        }
    }

    /// <summary>A stream buffer for a message of the gravity.</summary>
    public StreamBuffer Send(Message_Gravity theGravity) => new(this, theGravity);

    /// <summary>A stream buffer for a Fail message.</summary>
    public StreamBuffer SendFail() => Send(Message_Fail);

    /// <summary>A stream buffer for an Alarm message.</summary>
    public StreamBuffer SendAlarm() => Send(Message_Alarm);

    /// <summary>A stream buffer for a Warning message.</summary>
    public StreamBuffer SendWarning() => Send(Message_Warning);

    /// <summary>A stream buffer for an Info message.</summary>
    public StreamBuffer SendInfo() => Send(Message_Info);

    /// <summary>A stream buffer for a Trace message.</summary>
    public StreamBuffer SendTrace() => Send(Message_Trace);

    public void SendFail(string theMessage) => Send(theMessage, Message_Fail);

    public void SendAlarm(string theMessage) => Send(theMessage, Message_Alarm);

    public void SendWarning(string theMessage) => Send(theMessage, Message_Warning);

    public void SendInfo(string theMessage) => Send(theMessage, Message_Info);

    public void SendTrace(string theMessage) => Send(theMessage, Message_Trace);
}
