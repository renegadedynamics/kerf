// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SITopolTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SITopolTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// A topological tool linked with the surface on which points are classified (OCCT's template class for the
/// interior search; no OCCT class derives from it).
/// </summary>
internal abstract class SITopolTool : Transient
{
    /// <summary>The state of the parametric point <paramref name="p"/> in the domain, with tolerance <paramref name="tol"/>.</summary>
    public abstract Containment Classify(in Point2D p, double tol);
}
