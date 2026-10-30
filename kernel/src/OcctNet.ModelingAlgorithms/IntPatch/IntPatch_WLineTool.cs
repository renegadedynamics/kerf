// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLineTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// SKELETON (5G): filled by the WLineTool/LineConstructor fork.
public static class IntPatch_WLineTool
{
    public static readonly double myMaxConcatAngle = M_PI / 6;

    public static IntPatch_WLine? ComputePurgedWLine(IntPatch_WLine theWLine, Adaptor3d_Surface theS1,
                                                     Adaptor3d_Surface theS2, Adaptor3d_TopolTool theDom1,
                                                     Adaptor3d_TopolTool theDom2)
        => throw new NotImplementedException();

    public static void JoinWLines(NCollection_Sequence<IntPatch_Line> theSlin, NCollection_Sequence<IntPatch_Point> theSPnt,
                                  Adaptor3d_Surface theS1, Adaptor3d_Surface theS2, double theTol3D)
        => throw new NotImplementedException();

    public static void ExtendTwoWLines(NCollection_Sequence<IntPatch_Line> theSlin, Adaptor3d_Surface theS1,
                                       Adaptor3d_Surface theS2, double theToler3D, ReadOnlySpan<double> theArrPeriods,
                                       in Bnd_Box2d theBoxS1, in Bnd_Box2d theBoxS2,
                                       NCollection_List<gp_Pnt> theListOfCriticalPoints)
        => throw new NotImplementedException();
}
