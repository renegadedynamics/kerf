// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeBounds.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeBounds.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeBounds.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The bounds of a shape loaded in the hidden-line data: its outliner and data, its number of isoparametric lines,
/// the ranges of its vertices, edges and faces in the data's arrays, and its encoded min-max box.
/// </summary>
/// <remarks>
/// OCCT's value class (held by value in HLRBRep_InternalAlgo's sequence); a class here, with OCCT's implicit copy as
/// <see cref="HLRBRep_ShapeBounds(HLRBRep_ShapeBounds)"/> (the handles shared). A working object.
/// </remarks>
public sealed class HLRBRep_ShapeBounds
{
    private HLRTopoBRep_OutLiner? myShape;
    private Standard_Transient? myShapeData;
    private int myNbIso;
    private int myVertStart;
    private int myVertEnd;
    private int myEdgeStart;
    private int myEdgeEnd;
    private int myFaceStart;
    private int myFaceEnd;
    private HLRAlgo_EdgesBlock.MinMaxIndices myMinMax;

    public HLRBRep_ShapeBounds()
    {
        myNbIso = 0;
        myVertStart = 0;
        myVertEnd = 0;
        myEdgeStart = 0;
        myEdgeEnd = 0;
        myFaceStart = 0;
        myFaceEnd = 0;
    }

    public HLRBRep_ShapeBounds(HLRTopoBRep_OutLiner S, Standard_Transient? SData, int nbIso, int V1, int V2, int E1,
                               int E2, int F1, int F2)
    {
        myShape = S;
        myShapeData = SData;
        myNbIso = nbIso;
        myVertStart = V1;
        myVertEnd = V2;
        myEdgeStart = E1;
        myEdgeEnd = E2;
        myFaceStart = F1;
        myFaceEnd = F2;
    }

    public HLRBRep_ShapeBounds(HLRTopoBRep_OutLiner S, int nbIso, int V1, int V2, int E1, int E2, int F1, int F2)
    {
        myShape = S;
        myNbIso = nbIso;
        myVertStart = V1;
        myVertEnd = V2;
        myEdgeStart = E1;
        myEdgeEnd = E2;
        myFaceStart = F1;
        myFaceEnd = F2;
    }

    /// <summary>OCCT's implicit copy (the handles shared).</summary>
    public HLRBRep_ShapeBounds(HLRBRep_ShapeBounds theOther)
    {
        myShape = theOther.myShape;
        myShapeData = theOther.myShapeData;
        myNbIso = theOther.myNbIso;
        myVertStart = theOther.myVertStart;
        myVertEnd = theOther.myVertEnd;
        myEdgeStart = theOther.myEdgeStart;
        myEdgeEnd = theOther.myEdgeEnd;
        myFaceStart = theOther.myFaceStart;
        myFaceEnd = theOther.myFaceEnd;
        myMinMax = theOther.myMinMax;
    }

    /// <summary>Shifts the ranges by NV vertices, NE edges and NF faces.</summary>
    public void Translate(int NV, int NE, int NF)
    {
        myVertStart += NV;
        myVertEnd += NV;
        myEdgeStart += NE;
        myEdgeEnd += NE;
        myFaceStart += NF;
        myFaceEnd += NF;
    }

    public void Shape(HLRTopoBRep_OutLiner? S) => myShape = S;

    public HLRTopoBRep_OutLiner? Shape() => myShape;

    public void ShapeData(Standard_Transient? SD) => myShapeData = SD;

    public Standard_Transient? ShapeData() => myShapeData;

    public void NbOfIso(int nbIso) => myNbIso = nbIso;

    public int NbOfIso() => myNbIso;

    /// <summary>The numbers of vertices, edges and faces.</summary>
    public void Sizes(out int NV, out int NE, out int NF)
    {
        NV = myVertEnd + 1 - myVertStart;
        NE = myEdgeEnd + 1 - myEdgeStart;
        NF = myFaceEnd + 1 - myFaceStart;
    }

    /// <summary>The ranges of the vertices, edges and faces.</summary>
    public void Bounds(out int V1, out int V2, out int E1, out int E2, out int F1, out int F2)
    {
        V1 = myVertStart;
        V2 = myVertEnd;
        E1 = myEdgeStart;
        E2 = myEdgeEnd;
        F1 = myFaceStart;
        F2 = myFaceEnd;
    }

    public void UpdateMinMax(in HLRAlgo_EdgesBlock.MinMaxIndices theTotMinMax) => myMinMax = theTotMinMax;

    /// <summary>The encoded min-max box (a reference, as OCCT's).</summary>
    public ref HLRAlgo_EdgesBlock.MinMaxIndices MinMax() => ref myMinMax;
}
