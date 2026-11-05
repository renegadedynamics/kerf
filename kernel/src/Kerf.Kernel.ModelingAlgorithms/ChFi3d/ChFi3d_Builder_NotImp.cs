// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_NotImp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Fillets;

internal abstract partial class FilletBuilderBase
{
    // The walking on a restriction (surface/restriction and restriction/restriction), which only ChFi3d_FilBuilder
    // implements: the root class throws.

    //=================================================================================================

    internal virtual void SimulSurf(SurfData data, ElSpine hGuide, Spine spine, int choix,
                                    FaceSurfaceAdaptor hs1, TopolTool i1, EdgeCurve2DAdaptor pc1,
                                    FaceSurfaceAdaptor hSref1, EdgeCurve2DAdaptor pCref1, ref bool decroch1,
                                    FaceSurfaceAdaptor hs2, TopolTool i2, Orientation or2,
                                    double fleche, double tolGuide, ref double first, ref double last, bool inside,
                                    bool appro, bool forward, bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void SimulSurf(SurfData data, ElSpine hGuide, Spine spine, int choix,
                                    FaceSurfaceAdaptor hs1, TopolTool i1, Orientation or1,
                                    FaceSurfaceAdaptor hs2, TopolTool i2, EdgeCurve2DAdaptor pc2,
                                    FaceSurfaceAdaptor hSref2, EdgeCurve2DAdaptor pCref2, ref bool decroch2,
                                    double fleche, double tolGuide, ref double first, ref double last, bool inside,
                                    bool appro, bool forward, bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void SimulSurf(SurfData data, ElSpine hGuide, Spine spine, int choix,
                                    FaceSurfaceAdaptor hs1, TopolTool i1, EdgeCurve2DAdaptor pc1,
                                    FaceSurfaceAdaptor hSref1, EdgeCurve2DAdaptor pCref1, ref bool decroch1,
                                    Orientation or1, FaceSurfaceAdaptor hs2, TopolTool i2,
                                    EdgeCurve2DAdaptor pc2, FaceSurfaceAdaptor hSref2, EdgeCurve2DAdaptor pCref2,
                                    ref bool decroch2, Orientation or2, double fleche, double tolGuide,
                                    ref double first, ref double last, bool inside, bool appro, bool forward,
                                    bool recP1, bool recRst1, bool recP2, bool recRst2, math_Vector soldep)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine hGuide,
                                      Spine spine, int choix, FaceSurfaceAdaptor hs1, TopolTool i1,
                                      EdgeCurve2DAdaptor pc1, FaceSurfaceAdaptor hSref1, EdgeCurve2DAdaptor pCref1,
                                      ref bool decroch1, FaceSurfaceAdaptor hs2, TopolTool i2,
                                      Orientation or2, double maxStep, double fleche, double tolGuide,
                                      ref double first, ref double last, bool inside, bool appro, bool forward,
                                      bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine hGuide,
                                      Spine spine, int choix, FaceSurfaceAdaptor hs1, TopolTool i1,
                                      Orientation or1, FaceSurfaceAdaptor hs2, TopolTool i2,
                                      EdgeCurve2DAdaptor pc2, FaceSurfaceAdaptor hSref2, EdgeCurve2DAdaptor pCref2,
                                      ref bool decroch2, double maxStep, double fleche, double tolGuide,
                                      ref double first, ref double last, bool inside, bool appro, bool forward,
                                      bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine hGuide,
                                      Spine spine, int choix, FaceSurfaceAdaptor hs1, TopolTool i1,
                                      EdgeCurve2DAdaptor pc1, FaceSurfaceAdaptor hSref1, EdgeCurve2DAdaptor pCref1,
                                      ref bool decroch1, Orientation or1, FaceSurfaceAdaptor hs2,
                                      TopolTool i2, EdgeCurve2DAdaptor pc2, FaceSurfaceAdaptor hSref2,
                                      EdgeCurve2DAdaptor pCref2, ref bool decroch2, Orientation or2,
                                      double maxStep, double fleche, double tolGuide, ref double first,
                                      ref double last, bool inside, bool appro, bool forward, bool recP1,
                                      bool recRst1, bool recP2, bool recRst2, math_Vector soldep)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }
}
