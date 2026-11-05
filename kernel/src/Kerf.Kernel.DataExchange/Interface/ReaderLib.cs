// Port of: src/DataExchange/TKXSBase/Interface/Interface_ReaderLib.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ReaderLib_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// The library of Interface_ReaderModules: the readers of the entities from a file of a protocol, as <see cref="Library{TModule, TProtocol}"/>
/// instantiated with <see cref="ReaderModule"/> and <see cref="Protocol"/>.
/// </summary>
internal class ReaderLib : Library<ReaderModule, Protocol>
{
    /// <summary>
    /// Creates a library which complies with a protocol: it gets the modules from the global set which are bound to
    /// the protocol and its resources.
    /// </summary>
    public ReaderLib(Protocol? aprotocol) : base(aprotocol)
    {
    }

    /// <summary>A copy of another library (the same modules).</summary>
    public ReaderLib(ReaderLib other) : base(other)
    {
    }

    /// <summary>Creates an empty library: it will later be filled by <c>AddProtocol</c>.</summary>
    public ReaderLib()
    {
    }
}
