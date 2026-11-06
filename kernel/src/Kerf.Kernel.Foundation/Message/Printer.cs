// Port of: src/FoundationClasses/TKernel/Message/Message_Printer.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Printer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using System.Text;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Foundation;

/// <summary>
/// Where a <see cref="Messenger"/> sends its messages. A message has a gravity, and a printer ignores the
/// messages below its trace level (<see cref="Info"/> by default). A subclass implements <see cref="SendCore"/>.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (its trace level is set in place). TCollection's strings are <see cref="string"/>, so
/// OCCT's three <c>Send</c> overloads (C string, AsciiString, ExtendedString) are one. <see cref="SendObject"/> writes
/// the object's type name and, in place of OCCT's pointer, its <see cref="RuntimeHelpers.GetHashCode(object)"/> in
/// hexadecimal.
/// </remarks>
internal abstract class Printer : Transient
{
    protected Gravity _traceLevel;

    /// <summary>A printer with the trace level <see cref="Info"/>.</summary>
    protected Printer()
    {
        _traceLevel = Info;
    }

    /// <summary>The trace level: messages of a lower gravity are ignored.</summary>
    public Gravity GetTraceLevel() => _traceLevel;

    /// <summary>Sets the trace level.</summary>
    public void SetTraceLevel(Gravity traceLevel) => _traceLevel = traceLevel;

    /// <summary>Sends a message with the gravity, if it isn't below the trace level.</summary>
    public virtual void Send(string theString, Gravity gravity)
    {
        if (gravity >= _traceLevel)
        {
            SendCore(theString, gravity);
        }
    }

    /// <summary>Sends the text of a string stream.</summary>
    public virtual void SendStringStream(StringBuilder stream, Gravity gravity)
    {
        if (gravity >= _traceLevel)
        {
            SendCore(stream.ToString(), gravity);
        }
    }

    /// <summary>Sends an object as "type name: address".</summary>
    public virtual void SendObject(Transient? theObject, Gravity gravity)
    {
        if (theObject is not null && gravity >= _traceLevel)
        {
            SendCore(theObject.DynamicTypeName() + ": 0x" + RuntimeHelpers.GetHashCode(theObject).ToString("x8"), gravity);
        }
    }

    /// <summary>Sends a message with the gravity; implemented by each printer.</summary>
    protected abstract void SendCore(string theString, Gravity gravity);
}
