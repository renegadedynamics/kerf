// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

public partial class ChFi3d_Builder
{
    internal void CallPerformSurf(ChFiDS_Stripe? Stripe, bool Simul, NCollection_Sequence<ChFiDS_SurfData> SeqSD, ChFiDS_SurfData? SD, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, BRepAdaptor_Surface? HS1, BRepAdaptor_Surface? HS3, in gp_Pnt2d pp1, in gp_Pnt2d pp3, Adaptor3d_TopolTool? It1, BRepAdaptor_Surface? HS2, BRepAdaptor_Surface? HS4, in gp_Pnt2d pp2, in gp_Pnt2d pp4, Adaptor3d_TopolTool? It2, double MaxStep, double Fleche, double _unused632, ref double First, ref double Last, bool Inside, bool _unused952, bool forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int intf, ref int intl, ref BRepAdaptor_Surface? Surf1, ref BRepAdaptor_Surface? Surf2)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool StripeOrientations(ChFiDS_Spine? Spine, ref TopAbs_Orientation Or1, ref TopAbs_Orientation Or2, ref int ChoixConge)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    private void ConexFaces(ChFiDS_Spine? Spine, int IEdge, ref BRepAdaptor_Surface? HS1, ref BRepAdaptor_Surface? HS2)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    private void StartSol(ChFiDS_Stripe? Stripe, ChFiDS_ElSpine? HGuide, BRepAdaptor_Surface? HS1, BRepAdaptor_Surface? HS2, BRepTopAdaptor_TopolTool? I1, BRepTopAdaptor_TopolTool? I2, ref gp_Pnt2d P1, ref gp_Pnt2d P2, ref double First)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal bool SearchFace(ChFiDS_Spine? Spine, in ChFiDS_CommonPoint Pc, in TopoDS_Face FRef, ref TopoDS_Face FVoi)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    private void PerformSetOfSurfOnElSpine(ChFiDS_ElSpine? HGuide, ChFiDS_Stripe? Stripe, BRepTopAdaptor_TopolTool? It1, BRepTopAdaptor_TopolTool? It2, bool Simul = false)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal void PerformSetOfKPart(ChFiDS_Stripe? Stripe, bool Simul = false)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal void PerformSetOfKGen(ChFiDS_Stripe? Stripe, bool Simul = false)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal void PerformSetOfSurf(ChFiDS_Stripe? Stripe, bool Simul = false)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

}
