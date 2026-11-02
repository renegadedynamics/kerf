// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DataExchange.Tests;

/// <summary>
/// The non-parallel collection of the tests that change process-wide state: the messages loaded into
/// Message_MsgFile, and the global lists of the libraries' modules (registered once by protocols).
/// </summary>
[CollectionDefinition(Name, DisableParallelization = true)]
public sealed class ProcessWideMessages
{
    public const string Name = "ProcessWideMessages";
}
