// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

public partial class ChFi3d_Builder
{
    internal bool CompleteData(ChFiDS_SurfData? Data, Geom_Surface? Surfcoin, Adaptor3d_Surface? S1, Geom2d_Curve? PC1, Adaptor3d_Surface? S2, Geom2d_Curve? PC2, TopAbs_Orientation Or, bool On1, bool Gd1, bool Gd2, bool Gf1, bool Gf2)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool CompleteData(ChFiDS_SurfData? Data, Blend_Function Func, BRepBlend_Line? lin, Adaptor3d_Surface? S1, Adaptor3d_Surface? S2, TopAbs_Orientation Or1, bool Gd1, bool Gd2, bool Gf1, bool Gf2, bool Reversed = false)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool CompleteData(ChFiDS_SurfData? Data, Blend_SurfRstFunction Func, BRepBlend_Line? lin, Adaptor3d_Surface? S1, Adaptor3d_Surface? S2, TopAbs_Orientation Or, bool Reversed = false)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool CompleteData(ChFiDS_SurfData? Data, Blend_RstRstFunction Func, BRepBlend_Line? lin, Adaptor3d_Surface? S1, Adaptor3d_Surface? S2, TopAbs_Orientation Or)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool StoreData(ChFiDS_SurfData? Data, AppBlend_Approx approx, BRepBlend_Line? lin, Adaptor3d_Surface? S1, Adaptor3d_Surface? S2, TopAbs_Orientation Or1, bool Gd1, bool Gd2, bool Gf1, bool Gf2, bool Reversed = false)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool ComputeData(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor3d_TopolTool? I1, Adaptor3d_Surface? S2, Adaptor2d_Curve2d? PC2, Adaptor3d_TopolTool? I2, ref bool Decroch, Blend_SurfRstFunction Func, Blend_FuncInv FInv, Blend_SurfPointFuncInv FInvP, Blend_SurfCurvFuncInv FInvC, double PFirst, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, math_Vector Soldep, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool ComputeData(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor2d_Curve2d? PC1, Adaptor3d_TopolTool? I1, ref bool Decroch1, Adaptor3d_Surface? S2, Adaptor2d_Curve2d? PC2, Adaptor3d_TopolTool? I2, ref bool Decroch2, Blend_RstRstFunction Func, Blend_SurfCurvFuncInv FInv1, Blend_CurvPointFuncInv FInvP1, Blend_SurfCurvFuncInv FInv2, Blend_CurvPointFuncInv FInvP2, double PFirst, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, math_Vector Soldep, bool Inside, bool Appro, bool Forward, bool RecP1, bool RecRst1, bool RecP2, bool RecRst2)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool SimulData(ChFiDS_SurfData? _unused104, ChFiDS_ElSpine? HGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor3d_TopolTool? I1, Adaptor3d_Surface? S2, Adaptor2d_Curve2d? PC2, Adaptor3d_TopolTool? I2, ref bool Decroch, Blend_SurfRstFunction Func, Blend_FuncInv FInv, Blend_SurfPointFuncInv FInvP, Blend_SurfCurvFuncInv FInvC, double PFirst, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, math_Vector Soldep, int NbSecMin, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool SimulData(ChFiDS_SurfData? _unused424, ChFiDS_ElSpine? HGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor2d_Curve2d? PC1, Adaptor3d_TopolTool? I1, ref bool Decroch1, Adaptor3d_Surface? S2, Adaptor2d_Curve2d? PC2, Adaptor3d_TopolTool? I2, ref bool Decroch2, Blend_RstRstFunction Func, Blend_SurfCurvFuncInv FInv1, Blend_CurvPointFuncInv FInvP1, Blend_SurfCurvFuncInv FInv2, Blend_CurvPointFuncInv FInvP2, double PFirst, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, math_Vector Soldep, int NbSecMin, bool Inside, bool Appro, bool Forward, bool RecP1, bool RecRst1, bool RecP2, bool RecRst2)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool ComputeData(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor3d_TopolTool? I1, Adaptor3d_Surface? S2, Adaptor3d_TopolTool? I2, Blend_Function Func, Blend_FuncInv FInv, double PFirst, double MaxStep, double Fleche, double tolguide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, math_Vector Soldep, ref int intf, ref int intl, ref bool Gd1, ref bool Gd2, ref bool Gf1, ref bool Gf2, bool RecOnS1 = false, bool RecOnS2 = false)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool SimulData(ChFiDS_SurfData? _unused424, ChFiDS_ElSpine? HGuide, ChFiDS_ElSpine? AdditionalHGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor3d_TopolTool? I1, Adaptor3d_Surface? S2, Adaptor3d_TopolTool? I2, Blend_Function Func, Blend_FuncInv FInv, double PFirst, double MaxStep, double Fleche, double tolguide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, math_Vector Soldep, int NbSecMin, bool RecOnS1 = false, bool RecOnS2 = false)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

}
