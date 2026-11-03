// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_SignText.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_SignText.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Monitoring;

/// <summary>
/// Provides the basic service to get a text which identifies an object in a context It can be used for other
/// classes (general signatures ...) It can also be used to build a message in which an object is to be
/// identified
/// </summary>
public abstract class SignText : Transient
{

    /// <summary>
    /// Returns an identification of the Signature (a word), given at initialization time
    /// </summary>
    public abstract string Name();

    /// <summary>
    /// Gives a text as a signature for a transient object alone, i.e. without defined context. By default, calls
    /// Text with undefined context (Null Handle) and if empty, then returns DynamicType
    /// </summary>
    public virtual string TextAlone(Transient? ent)
    {
        Transient nulctx = null; // no context
        string atext = Text(ent, nulctx);
        if (atext.Length == 0)
        {
            if (ent is null)
            {
                atext += "(null)";
            }
            else
            {
                atext += ent.DynamicTypeName();
            }
        }
        return atext;
    }

    /// <summary>
    /// Gives a text as a signature for a transient object in a context If the context is senseless, it can be
    /// given as Null Handle empty result if nothing to give (at least the DynamicType could be sent ?)
    /// </summary>
    public abstract string Text(Transient? ent, Transient? context);
}
