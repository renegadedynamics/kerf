// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// SKELETON (5G): the public API, filled by the IntPatch_Intersection fork.
public class IntPatch_Intersection
{
    public IntPatch_Intersection() { }

    public IntPatch_Intersection(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, Adaptor3d_Surface S2,
                                 Adaptor3d_TopolTool D2, double TolArc, double TolTang)
        => throw new NotImplementedException();

    public IntPatch_Intersection(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, double TolArc, double TolTang)
        => throw new NotImplementedException();

    public void SetTolerances(double TolArc, double TolTang, double UVMaxStep, double Fleche)
        => throw new NotImplementedException();

    public void Perform(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, Adaptor3d_Surface S2, Adaptor3d_TopolTool D2,
                        double TolArc, double TolTang, bool isGeomInt = true, bool theIsReqToKeepRLine = false,
                        bool theIsReqToPostWLProc = true)
        => throw new NotImplementedException();

    public void Perform(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, Adaptor3d_Surface S2, Adaptor3d_TopolTool D2,
                        double TolArc, double TolTang, NCollection_List<IntSurf_PntOn2S> LOfPnts,
                        bool isGeomInt = true, bool theIsReqToKeepRLine = false, bool theIsReqToPostWLProc = true)
        => throw new NotImplementedException();

    public void Perform(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, Adaptor3d_Surface S2, Adaptor3d_TopolTool D2,
                        double U1, double V1, double U2, double V2, double TolArc, double TolTang)
        => throw new NotImplementedException();

    public void Perform(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, double TolArc, double TolTang)
        => throw new NotImplementedException();

    public bool IsDone() => throw new NotImplementedException();
    public bool IsEmpty() => throw new NotImplementedException();
    public bool TangentFaces() => throw new NotImplementedException();
    public bool OppositeFaces() => throw new NotImplementedException();
    public int NbPnts() => throw new NotImplementedException();
    public IntPatch_Point Point(int Index) => throw new NotImplementedException();
    public int NbLines() => throw new NotImplementedException();
    public IntPatch_Line Line(int Index) => throw new NotImplementedException();
    public NCollection_Sequence<IntPatch_Line> SequenceOfLine() => throw new NotImplementedException();

    public void Dump(int Mode, Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, Adaptor3d_Surface S2,
                     Adaptor3d_TopolTool D2, TextWriter theOut)
        => throw new NotImplementedException();

    public static bool CheckSingularPoints(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1,
                                           Adaptor3d_Surface theS2, out double theDist)
        => throw new NotImplementedException();

    public static double DefineUVMaxStep(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                                         Adaptor3d_TopolTool theD2)
        => throw new NotImplementedException();

    public static void PrepareSurfaces(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                                       Adaptor3d_TopolTool theD2, double Tol,
                                       NCollection_DynamicArray<Adaptor3d_Surface> theSeqHS1,
                                       NCollection_DynamicArray<Adaptor3d_Surface> theSeqHS2)
        => throw new NotImplementedException();
}
