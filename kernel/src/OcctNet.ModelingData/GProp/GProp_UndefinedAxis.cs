// Port of: src/ModelingData/TKGeomBase/GProp/GProp_UndefinedAxis.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Raised when a method refers to an undefined inertia axis of symmetry.</summary>
[OcctName("GProp_UndefinedAxis")]
public class GProp_UndefinedAxis : Standard_DomainError
{
    public GProp_UndefinedAxis() { }

    public GProp_UndefinedAxis(string? message) : base(message) { }
}
