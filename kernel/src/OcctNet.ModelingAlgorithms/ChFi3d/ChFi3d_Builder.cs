// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0649, CS0169 // p7n2-TEMP until ported

namespace Occt;

/// <summary>
/// Root class of the fillet and chamfer builders: computes the fillets of a shape's contours (the spines), their
/// surfaces and the corners where they meet, writes them into the legacy data structure, and hands that to the
/// topological reconstruction.
/// </summary>
/// <remarks>
/// <para>
/// A working object of the fillet algorithms (decision 21), ported over OCCT's files: this file (the header and
/// <c>ChFi3d_Builder.cxx</c>), <c>_1</c> (construction, queries, spines), <c>_2</c> (stripes), <c>_6</c> (walking
/// and approximation), <c>_C1</c>, <c>_C2</c>, <c>_CnCrn</c> (corners), <c>_SpKP</c> (splitting the analytic cases)
/// and <c>_NotImp</c>. Each file's static functions are in a static class named after it
/// (<c>ChFi3d_Builder_2</c>, ...), as <see cref="ChFi3d_Builder_0"/>'s.
/// </para>
/// <para>
/// <b>The working copy</b> (decisions 1005, 10203): the constructor copies a shape holding frozen TShapes
/// (<see cref="BRepTools_CopyAtEntry"/>) and builds its maps from the copy, so that the vertices whose tolerance OCCT
/// raises (<c>ChFi3d_IndexPointInDS</c>, <c>Compute</c>'s <c>UpdateVertex</c>) and the edges the elementary spines
/// give a 3D curve are the copy's. Edges and vertices given by the caller are mapped onto the copy at the entry of
/// each public member, and those handed back are mapped to the caller's.
/// </para>
/// <para>
/// <b>The seam</b> (decision 992): OCCT's <c>myCoup</c>, a <c>TopOpeBRepBuild_HBuilder</c>, is an
/// <see cref="ChFi3d_Reconstruction"/>, which unit 7R implements. Until then <see cref="Compute"/> fills the data
/// structure exactly as OCCT does and stops at <c>myCoup->Perform(myDS)</c> with a <see cref="NotImplementedException"/>.
/// </para>
/// </remarks>
public abstract partial class ChFi3d_Builder
{
    //=================================================================================================
    // Fields (OCCT's protected and private members)

    internal double tolappangle;
    internal double tolesp;
    internal double tol2d;
    internal double tolapp3d;
    internal double tolapp2d;
    internal double fleche;
    internal GeomAbs_Shape myConti;
    internal ChFiDS_Map myEFMap = new();
    internal ChFiDS_Map myESoMap = new();
    internal ChFiDS_Map myEShMap = new();
    internal ChFiDS_Map myVFMap = new();
    internal ChFiDS_Map myVEMap = new();
    internal TopOpeBRepDS_HDataStructure myDS;
    /// <summary>OCCT's <c>TopOpeBRepBuild_HBuilder</c>: the reconstruction (unit 7R).</summary>
    internal ChFi3d_Reconstruction myCoup;
    internal NCollection_List<ChFiDS_Stripe> myListStripe = new();
    internal ChFiDS_StripeMap myVDataMap = new();
    internal NCollection_List<ChFiDS_Regul> myRegul = new();
    internal NCollection_List<ChFiDS_Stripe> badstripes = new();
    internal NCollection_List<TopoDS_Shape> badvertices = new();
    internal NCollection_DataMap<TopoDS_Shape, NCollection_List<int>> myEVIMap = new(TopTools_ShapeMapHasher.Instance);
    internal NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myEdgeFirstFace = new(TopTools_ShapeMapHasher.Instance);
    internal bool done;
    internal bool hasresult;

    private TopoDS_Shape myShape;
    private double angular;
    private NCollection_List<TopoDS_Shape> myGenerated = new();
    private TopoDS_Shape myShapeResult = new();
    private TopoDS_Shape badShape = new();

    /// <summary>The copy at the entry (OCCT.NET): <c>myShape</c> is its <see cref="BRepTools_CopyAtEntry.Shape"/>.</summary>
    private BRepTools_CopyAtEntry myCopyAtEntry;

    /// <summary>The working shape: the copy of the shape given (OCCT.NET, for tests and unit 7R).</summary>
    internal TopoDS_Shape WorkingShape() => myShape;

    /// <summary>The caller's sub-shape mapped onto the working copy (OCCT.NET).</summary>
    internal TopoDS_Shape ToWork(in TopoDS_Shape S) => myCopyAtEntry.Copied(S);

    /// <summary>A sub-shape of the working copy mapped back to the caller's (OCCT.NET).</summary>
    internal TopoDS_Shape FromWork(in TopoDS_Shape S) => myCopyAtEntry.Original(S);

    //=================================================================================================
    // The pure virtual members (ChFi3d_FilBuilder and ChFi3d_ChBuilder implement them)

    internal abstract void SimulKPart(ChFiDS_SurfData SD);

    internal abstract bool SimulSurf(ChFiDS_SurfData Data, ChFiDS_ElSpine Guide, ChFiDS_Spine Spine, int Choix,
                                     BRepAdaptor_Surface S1, Adaptor3d_TopolTool I1, BRepAdaptor_Surface S2,
                                     Adaptor3d_TopolTool I2, double TolGuide, ref double First, ref double Last,
                                     bool Inside, bool Appro, bool Forward, bool RecOnS1, bool RecOnS2,
                                     math_Vector Soldep, ref int Intf, ref int Intl);

    internal abstract void SetRegul();

    internal abstract bool PerformSurf(NCollection_Sequence<ChFiDS_SurfData> Data, ChFiDS_ElSpine Guide,
                                       ChFiDS_Spine Spine, int Choix, BRepAdaptor_Surface S1, Adaptor3d_TopolTool I1,
                                       BRepAdaptor_Surface S2, Adaptor3d_TopolTool I2, double MaxStep, double Fleche,
                                       double TolGuide, ref double First, ref double Last, bool Inside, bool Appro,
                                       bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int Intf,
                                       ref int Intl);

    internal abstract void PerformTwoCorner(int Index);

    internal abstract void PerformThreeCorner(int Index);

    internal abstract void ExtentOneCorner(in TopoDS_Vertex V, ChFiDS_Stripe S);

    internal abstract void ExtentTwoCorner(in TopoDS_Vertex V, NCollection_List<ChFiDS_Stripe> LS);

    internal abstract void ExtentThreeCorner(in TopoDS_Vertex V, NCollection_List<ChFiDS_Stripe> LS);

    internal abstract bool PerformFirstSection(ChFiDS_Spine S, ChFiDS_ElSpine HGuide, int Choix,
                                               BRepAdaptor_Surface S1, BRepAdaptor_Surface S2,
                                               Adaptor3d_TopolTool I1, Adaptor3d_TopolTool I2, double Par,
                                               math_Vector SolDep, ref TopAbs_State Pos1, ref TopAbs_State Pos2);

    //=================================================================================================
    // ChFi3d_Builder.cxx (stubs until ported)

    private void ExtentAnalyse() => throw new NotImplementedException("7N2: not ported yet");

    /// <summary>Computes the fillets: the stripes, the corners, the data structure; then the reconstruction.</summary>
    public void Compute() => throw new NotImplementedException("7N2: not ported yet");

    private void PerformSingularCorner(int Index) => throw new NotImplementedException("7N2: not ported yet");

    private void PerformFilletOnVertex(int Index) => throw new NotImplementedException("7N2: not ported yet");

    /// <summary>Resets the stripes and the data computed.</summary>
    public void Reset() => throw new NotImplementedException("7N2: not ported yet");

    /// <summary>The faces generated from an edge or vertex of the shape.</summary>
    public NCollection_List<TopoDS_Shape> Generated(in TopoDS_Shape EouV) =>
        throw new NotImplementedException("7N2: not ported yet");
}
