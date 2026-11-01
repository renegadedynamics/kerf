// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0649, CS0169 // p7n2-TEMP until ported

namespace Occt;

/// <summary>Computes fillets of constant or evolving radius on the edges of a shape.</summary>
public partial class ChFi3d_FilBuilder : ChFi3d_Builder
{
    private BlendFunc_SectionShape myShape;

    /// <summary>Initializes the builder on <paramref name="S"/>.</summary>
    public ChFi3d_FilBuilder(in TopoDS_Shape S, ChFi3d_FilletShape FShape = ChFi3d_FilletShape.ChFi3d_Rational, double Ta = 1.0e-2)
        : base(S, Ta)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetFilletShape(ChFi3d_FilletShape FShape)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public ChFi3d_FilletShape GetFilletShape()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void Add(in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void Add(double Radius, in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetRadius(Law_Function? C, int IC, int IinC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public bool IsConstant(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public double Radius(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void ResetContour(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetRadius(double Radius, int IC, in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void UnSet(int IC, in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetRadius(double Radius, int IC, in TopoDS_Vertex V)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void UnSet(int IC, in TopoDS_Vertex V)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetRadius(in gp_XY UandR, int IC, int IinC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public bool IsConstant(int IC, in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public double Radius(int IC, in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public bool GetBounds(int IC, in TopoDS_Edge E, ref double F, ref double L)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public Law_Function? GetLaw(int IC, in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetLaw(int IC, in TopoDS_Edge E, Law_Function? L)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void Simulate(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public int NbSurf(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public NCollection_HArray1<ChFiDS_CircSection>? Sect(int IC, int IS)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void SimulKPart(ChFiDS_SurfData? SD)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override bool SimulSurf(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? S1, Adaptor3d_TopolTool? I1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I2, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int intf, ref int intl)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void SimulSurf(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, BRepAdaptor_Curve2d? PC1, BRepAdaptor_Surface? HSref1, BRepAdaptor_Curve2d? PCref1, ref bool Decroch1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, TopAbs_Orientation Or2, double _unused32, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void SimulSurf(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, TopAbs_Orientation Or1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, BRepAdaptor_Curve2d? PC2, BRepAdaptor_Surface? HSref2, BRepAdaptor_Curve2d? PCref2, ref bool Decroch2, double _unused688, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void SimulSurf(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, BRepAdaptor_Curve2d? PC1, BRepAdaptor_Surface? HSref1, BRepAdaptor_Curve2d? PCref1, ref bool Decroch1, TopAbs_Orientation Or1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, BRepAdaptor_Curve2d? PC2, BRepAdaptor_Surface? HSref2, BRepAdaptor_Curve2d? PCref2, ref bool Decroch2, TopAbs_Orientation Or2, double _unused32, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP1, bool RecRst1, bool RecP2, bool RecRst2, math_Vector Soldep)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override bool PerformFirstSection(ChFiDS_Spine? Spine, ChFiDS_ElSpine? HGuide, int Choix, BRepAdaptor_Surface? S1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I1, Adaptor3d_TopolTool? I2, double Par, math_Vector SolDep, ref TopAbs_State Pos1, ref TopAbs_State Pos2)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override bool PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? S1, Adaptor3d_TopolTool? I1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int intf, ref int intl)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, BRepAdaptor_Curve2d? PC1, BRepAdaptor_Surface? HSref1, BRepAdaptor_Curve2d? PCref1, ref bool Decroch1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, TopAbs_Orientation Or2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, TopAbs_Orientation Or1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, BRepAdaptor_Curve2d? PC2, BRepAdaptor_Surface? HSref2, BRepAdaptor_Curve2d? PCref2, ref bool Decroch2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, BRepAdaptor_Curve2d? PC1, BRepAdaptor_Surface? HSref1, BRepAdaptor_Curve2d? PCref1, ref bool Decroch1, TopAbs_Orientation Or1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, BRepAdaptor_Curve2d? PC2, BRepAdaptor_Surface? HSref2, BRepAdaptor_Curve2d? PCref2, ref bool Decroch2, TopAbs_Orientation Or2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP1, bool RecRst1, bool RecP2, bool RecRst2, math_Vector Soldep)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal void SplitSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, BRepBlend_Line? Line)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void ExtentOneCorner(in TopoDS_Vertex V, ChFiDS_Stripe? S)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void ExtentTwoCorner(in TopoDS_Vertex V, NCollection_List<ChFiDS_Stripe> LS)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void ExtentThreeCorner(in TopoDS_Vertex V, NCollection_List<ChFiDS_Stripe> LS)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override void SetRegul()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

}
