// Port of: src/DataExchange/TKXSBase/Interface/Interface_CopyControl.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_CopyControl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// This deferred class describes the services required by CopyTool to work. They are very simple and
/// correspond basically to the management of an indexed map. But they can be provided by various classes
/// which can control a Transfer. Each Starting Entity have at most one Result (Mapping one-one)
/// </summary>
public abstract class Interface_CopyControl : Transient
{

    /// <summary>
    /// Clears List of Copy Results. Gets Ready to begin another Copy Process.
    /// </summary>
    public abstract void Clear();

    /// <summary>
    /// Bind a Result to a Starting Entity identified by its Number
    /// </summary>
    public abstract void Bind(Transient? ent, Transient? res);

    /// <summary>
    /// Searches for the Result bound to a Startingf Entity identified by its Number. If Found, returns True and
    /// fills &lt;res&gt; Else, returns False and nullifies &lt;res&gt;
    /// </summary>
    public abstract bool Search(Transient? ent, out Transient? res);
}
