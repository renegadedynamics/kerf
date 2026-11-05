// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Root.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Root.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>The common services of the TopoDSToStep classes that report errors: the state and the tolerance.</summary>
internal class ExportRoot
{
    protected double _toler;
    protected bool _done;

    protected ExportRoot()
    {
        _done = false;
        _toler = 0.0001;
    }

    /// <summary>The tolerance to be used for writing (modifiable); 0.0001 if not set.</summary>
    public ref double Tolerance() => ref _toler;

    public bool IsDone => _done;
}
