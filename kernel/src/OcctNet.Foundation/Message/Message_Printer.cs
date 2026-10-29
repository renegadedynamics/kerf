// Port of: src/FoundationClasses/TKernel/Message/Message_Printer.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Printer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using System.Text;
using static Occt.Message_Gravity;

namespace Occt;

/// <summary>
/// Where a <see cref="Message_Messenger"/> sends its messages. A message has a gravity, and a printer ignores the
/// messages below its trace level (<see cref="Message_Info"/> by default). A subclass implements <see cref="send"/>.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (its trace level is set in place). TCollection's strings are <see cref="string"/>, so
/// OCCT's three <c>Send</c> overloads (C string, AsciiString, ExtendedString) are one. <see cref="SendObject"/> writes
/// the object's type name and, in place of OCCT's pointer, its <see cref="RuntimeHelpers.GetHashCode(object)"/> in
/// hexadecimal.
/// </remarks>
public abstract class Message_Printer : Standard_Transient
{
    protected Message_Gravity myTraceLevel;

    /// <summary>A printer with the trace level <see cref="Message_Info"/>.</summary>
    protected Message_Printer()
    {
        myTraceLevel = Message_Info;
    }

    /// <summary>The trace level: messages of a lower gravity are ignored.</summary>
    public Message_Gravity GetTraceLevel() => myTraceLevel;

    /// <summary>Sets the trace level.</summary>
    public void SetTraceLevel(Message_Gravity theTraceLevel) => myTraceLevel = theTraceLevel;

    /// <summary>Sends a message with the gravity, if it isn't below the trace level.</summary>
    public virtual void Send(string theString, Message_Gravity theGravity)
    {
        if (theGravity >= myTraceLevel)
        {
            send(theString, theGravity);
        }
    }

    /// <summary>Sends the text of a string stream.</summary>
    public virtual void SendStringStream(StringBuilder theStream, Message_Gravity theGravity)
    {
        if (theGravity >= myTraceLevel)
        {
            send(theStream.ToString(), theGravity);
        }
    }

    /// <summary>Sends an object as "type name: address".</summary>
    public virtual void SendObject(Standard_Transient? theObject, Message_Gravity theGravity)
    {
        if (theObject is not null && theGravity >= myTraceLevel)
        {
            send(theObject.DynamicType().Name + ": 0x" + RuntimeHelpers.GetHashCode(theObject).ToString("x8"), theGravity);
        }
    }

    /// <summary>Sends a message with the gravity; implemented by each printer.</summary>
    protected abstract void send(string theString, Message_Gravity theGravity);
}
