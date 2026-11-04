// Port of: src/ModelingData/TKGeomBase/gce/gce_Root.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>The base of the gce constructions: their status.</summary>
/// <remarks>
/// A construction is computed by its constructor. OCCT leaves the status unset on a few paths (occt-issues.md); the
/// port's starts as <see cref="ErrorType.Done"/>, a .NET field's default.
/// </remarks>
internal class Root
{
    protected ErrorType _theError;

    /// <summary>True if the construction is successful (the status is gce_Done).</summary>
    public bool IsDone => _theError == Done;

    /// <summary>True if the construction has failed (the status is not gce_Done).</summary>
    public bool IsError => _theError != Done;

    /// <summary>
    /// The status of the construction: gce_Done if it is successful, or another value of the gce_ErrorType
    /// enumeration indicating why it failed.
    /// </summary>
    public ErrorType Status() => _theError;
}
