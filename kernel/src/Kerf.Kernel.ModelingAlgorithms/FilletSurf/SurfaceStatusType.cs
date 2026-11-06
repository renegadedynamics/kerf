// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_StatusType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Fillets.Surfaces;

/// <summary>How many ends of a section of a fillet lie on an edge of their support faces.</summary>
internal enum SurfaceStatusType
{
    TwoExtremityOnEdge,
    OneExtremityOnEdge,
    NoExtremityOnEdge,
}
