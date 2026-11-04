// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_Root.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_Root.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The common services of GeomToStep's classes, which report an error.</summary>
/// <remarks>
/// OCCT's GeomToStep classes are used by value: a maker converts in its constructor and gives the STEP entity
/// through <c>Value()</c>, which throws StdFail_NotDone when the conversion wasn't done. The port's makers are
/// classes; the entities they make are new (decision 1239).
/// </remarks>
internal abstract class GeometryRoot
{
    protected bool _done;

    /// <summary>Whether the conversion was done.</summary>
    public bool IsDone
    {
        get
        {
            return _done;
        }
    }
}
