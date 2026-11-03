// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_NotImp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

public abstract partial class ChFi3d_Builder
{
    // The walking on a restriction (surface/restriction and restriction/restriction), which only ChFi3d_FilBuilder
    // implements: the root class throws.

    //=================================================================================================

    internal virtual void SimulSurf(ChFiDS_SurfData Data, ChFiDS_ElSpine HGuide, ChFiDS_Spine Spine, int Choix,
                                    BRepAdaptor_Surface HS1, Adaptor3d_TopolTool I1, BRepAdaptor_Curve2d PC1,
                                    BRepAdaptor_Surface HSref1, BRepAdaptor_Curve2d PCref1, ref bool Decroch1,
                                    BRepAdaptor_Surface HS2, Adaptor3d_TopolTool I2, TopAbs_Orientation Or2,
                                    double Fleche, double TolGuide, ref double First, ref double Last, bool Inside,
                                    bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void SimulSurf(ChFiDS_SurfData Data, ChFiDS_ElSpine HGuide, ChFiDS_Spine Spine, int Choix,
                                    BRepAdaptor_Surface HS1, Adaptor3d_TopolTool I1, TopAbs_Orientation Or1,
                                    BRepAdaptor_Surface HS2, Adaptor3d_TopolTool I2, BRepAdaptor_Curve2d PC2,
                                    BRepAdaptor_Surface HSref2, BRepAdaptor_Curve2d PCref2, ref bool Decroch2,
                                    double Fleche, double TolGuide, ref double First, ref double Last, bool Inside,
                                    bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void SimulSurf(ChFiDS_SurfData Data, ChFiDS_ElSpine HGuide, ChFiDS_Spine Spine, int Choix,
                                    BRepAdaptor_Surface HS1, Adaptor3d_TopolTool I1, BRepAdaptor_Curve2d PC1,
                                    BRepAdaptor_Surface HSref1, BRepAdaptor_Curve2d PCref1, ref bool Decroch1,
                                    TopAbs_Orientation Or1, BRepAdaptor_Surface HS2, Adaptor3d_TopolTool I2,
                                    BRepAdaptor_Curve2d PC2, BRepAdaptor_Surface HSref2, BRepAdaptor_Curve2d PCref2,
                                    ref bool Decroch2, TopAbs_Orientation Or2, double Fleche, double TolGuide,
                                    ref double First, ref double Last, bool Inside, bool Appro, bool Forward,
                                    bool RecP1, bool RecRst1, bool RecP2, bool RecRst2, math_Vector Soldep)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine HGuide,
                                      ChFiDS_Spine Spine, int Choix, BRepAdaptor_Surface HS1, Adaptor3d_TopolTool I1,
                                      BRepAdaptor_Curve2d PC1, BRepAdaptor_Surface HSref1, BRepAdaptor_Curve2d PCref1,
                                      ref bool Decroch1, BRepAdaptor_Surface HS2, Adaptor3d_TopolTool I2,
                                      TopAbs_Orientation Or2, double MaxStep, double Fleche, double TolGuide,
                                      ref double First, ref double Last, bool Inside, bool Appro, bool Forward,
                                      bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine HGuide,
                                      ChFiDS_Spine Spine, int Choix, BRepAdaptor_Surface HS1, Adaptor3d_TopolTool I1,
                                      TopAbs_Orientation Or1, BRepAdaptor_Surface HS2, Adaptor3d_TopolTool I2,
                                      BRepAdaptor_Curve2d PC2, BRepAdaptor_Surface HSref2, BRepAdaptor_Curve2d PCref2,
                                      ref bool Decroch2, double MaxStep, double Fleche, double TolGuide,
                                      ref double First, ref double Last, bool Inside, bool Appro, bool Forward,
                                      bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    //=================================================================================================

    internal virtual void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine HGuide,
                                      ChFiDS_Spine Spine, int Choix, BRepAdaptor_Surface HS1, Adaptor3d_TopolTool I1,
                                      BRepAdaptor_Curve2d PC1, BRepAdaptor_Surface HSref1, BRepAdaptor_Curve2d PCref1,
                                      ref bool Decroch1, TopAbs_Orientation Or1, BRepAdaptor_Surface HS2,
                                      Adaptor3d_TopolTool I2, BRepAdaptor_Curve2d PC2, BRepAdaptor_Surface HSref2,
                                      BRepAdaptor_Curve2d PCref2, ref bool Decroch2, TopAbs_Orientation Or2,
                                      double MaxStep, double Fleche, double TolGuide, ref double First,
                                      ref double Last, bool Inside, bool Appro, bool Forward, bool RecP1,
                                      bool RecRst1, bool RecP2, bool RecRst2, math_Vector Soldep)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }
}
