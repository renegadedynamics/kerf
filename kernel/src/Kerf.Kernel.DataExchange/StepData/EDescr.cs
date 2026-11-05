// Port of: src/DataExchange/TKDESTEP/StepData/StepData_EDescr.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_EDescr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>This class is intended to describe the authorized form for an entity, either Simple or Plex.</summary>
internal abstract class EDescr : Transient
{
    /// <summary>Tells if a ESDescr matches a step type: exact or super type.</summary>
    public abstract bool Matches(string steptype);

    /// <summary>Tells if a EDescr is complex (ECDescr) or simple (ESDescr: not complex).</summary>
    public abstract bool IsComplex { get; }

    /// <summary>Creates a described entity (i.e. a simple one).</summary>
    public abstract Described NewEntity();
}
