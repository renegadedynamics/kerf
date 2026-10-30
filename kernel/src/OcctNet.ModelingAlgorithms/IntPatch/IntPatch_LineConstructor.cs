// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_LineConstructor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_LineConstructor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// SKELETON (5G): filled by the WLineTool/LineConstructor fork.
public class IntPatch_LineConstructor
{
    public IntPatch_LineConstructor(int mode) { }

    public void Perform(NCollection_Sequence<IntPatch_Line> SL, IntPatch_Line L, Adaptor3d_Surface S1,
                        Adaptor3d_TopolTool D1, Adaptor3d_Surface S2, Adaptor3d_TopolTool D2, double Tol)
        => throw new NotImplementedException();

    public int NbLines() => throw new NotImplementedException();

    public IntPatch_Line Line(int index) => throw new NotImplementedException();
}
