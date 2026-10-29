// Port of: src/ModelingData/TKGeomBase/gce/gce_Root.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>The base of the gce constructions: their status.</summary>
/// <remarks>
/// A construction is computed by its constructor. OCCT leaves the status unset on a few paths (occt-issues.md); the
/// port's starts as <see cref="gce_ErrorType.gce_Done"/>, a .NET field's default.
/// </remarks>
public class gce_Root
{
    protected gce_ErrorType TheError;

    /// <summary>True if the construction is successful (the status is gce_Done).</summary>
    public bool IsDone() => TheError == gce_Done;

    /// <summary>True if the construction has failed (the status is not gce_Done).</summary>
    public bool IsError() => TheError != gce_Done;

    /// <summary>
    /// The status of the construction: gce_Done if it is successful, or another value of the gce_ErrorType
    /// enumeration indicating why it failed.
    /// </summary>
    public gce_ErrorType Status() => TheError;
}
