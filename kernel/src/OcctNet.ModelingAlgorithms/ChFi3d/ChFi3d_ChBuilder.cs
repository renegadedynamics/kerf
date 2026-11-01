// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_ChBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0649, CS0169, CS0414 // p7n2-TEMP until ported

namespace Occt;

/// <summary>Computes chamfers on the edges of a shape: one distance, two distances, or a distance and an angle.</summary>
public partial class ChFi3d_ChBuilder : ChFi3d_Builder
{
    private ChFiDS_ChamfMode myMode;

    /// <summary>Initializes the builder on <paramref name="S"/>.</summary>
    public ChFi3d_ChBuilder(in TopoDS_Shape S, double Ta = 1.0e-2)
        : base(S, Ta)
    {
        myMode = ChFiDS_ChamfMode.ChFiDS_ClassicChamfer;
    }

    public void Add(in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void Add(double Dis, in TopoDS_Edge E)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetDist(double Dis, int IC, in TopoDS_Face F)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void GetDist(int IC, ref double Dis)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void Add(double Dis1, double Dis2, in TopoDS_Edge E, in TopoDS_Face F)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetDists(double Dis1, double Dis2, int IC, in TopoDS_Face F)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void Dists(int IC, ref double dis1, ref double dis2)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void AddDA(double Dis1, double Angle, in TopoDS_Edge E, in TopoDS_Face F)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetDistAngle(double Dis, double Angle, int IC, in TopoDS_Face F)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void GetDistAngle(int IC, ref double Dis, ref double Angle)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void SetMode(ChFiDS_ChamfMode theMode)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public ChFiDS_ChamfMethod IsChamfer(int IC)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public ChFiDS_ChamfMode Mode()
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    public void ResetContour(int IC)
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

    internal override bool PerformFirstSection(ChFiDS_Spine? Spine, ChFiDS_ElSpine? HGuide, int Choix, BRepAdaptor_Surface? S1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I1, Adaptor3d_TopolTool? I2, double Par, math_Vector SolDep, ref TopAbs_State Pos1, ref TopAbs_State Pos2)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

    internal override bool PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? S1, Adaptor3d_TopolTool? I1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int intf, ref int intl)
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

    private void ConexFaces(ChFiDS_Spine? Spine, int IEdge, ref TopoDS_Face F1, ref TopoDS_Face F2)
    {
        throw new NotImplementedException("7N2: not ported yet");
    }

}
