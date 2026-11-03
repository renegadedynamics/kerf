// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeIList.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeIList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>Sorting and merging of the interferences of an edge, by their parameters.</summary>
public static class HLRBRep_EdgeIList
{
    /// <summary>Adds the interference I to the list IL, sorted by parameter (after the equal ones).</summary>
    public static void AddInterference(NCollection_List<HLRAlgo_Interference> IL, in HLRAlgo_Interference I,
                                       HLRBRep_EdgeInterferenceTool T)
    {
        NCollection_List<HLRAlgo_Interference>.Iterator It = new(IL);
        double p = T.ParameterOfInterference(I);
        while (It.More())
        {
            if (p < T.ParameterOfInterference(It.Value()))
            {
                IL.InsertBefore(I, It);
                return;
            }
            It.Next();
        }
        IL.Append(I);
    }

    /// <summary>Merges the successive interferences at the same vertex into one with a complex transition.</summary>
    public static void ProcessComplex(NCollection_List<HLRAlgo_Interference> IL, HLRBRep_EdgeInterferenceTool T)
    {
        TopCnx_EdgeFaceTransition transTool = new();
        Direction3D TgtE, NormE, TgtI, NormI;
        const double TolAng = 0.0001;
        double CurvE, CurvI;
        NCollection_List<HLRAlgo_Interference>.Iterator It1 = new(IL);

        while (It1.More())
        {
            NCollection_List<HLRAlgo_Interference>.Iterator It2 = new(It1);
            It2.Next();
            if (It2.More())
            {
                if (T.SameInterferences(It1.Value(), It2.Value())) // OCCT_DEBUG_SI's SimilarInterference left out
                {

                    T.EdgeGeometry(T.ParameterOfInterference(It1.Value()), out TgtE, out NormE, out CurvE);
                    transTool.Reset(TgtE, NormE, CurvE);
                    T.InterferenceBoundaryGeometry(It1.Value(), out TgtI, out NormI, out CurvI);
                    transTool.AddInterference(TolAng, TgtI, NormI, CurvI, It1.Value().Orientation(),
                                              It1.Value().Transition(), It1.Value().BoundaryTransition());

                    while (It2.More())
                    {
                        if (!(T.SameInterferences(It1.Value(), It2.Value())))
                        {
                            break;
                        }

                        T.InterferenceBoundaryGeometry(It2.Value(), out TgtI, out NormI, out CurvI);
                        transTool.AddInterference(TolAng, TgtI, NormI, CurvI, It2.Value().Orientation(),
                                                  It2.Value().Transition(), It2.Value().BoundaryTransition());
                        IL.Remove(It2);
                    }
                    // set the cumulated transition on the interference
                    It1.ChangeValue().Transition(transTool.Transition());
                    It1.ChangeValue().BoundaryTransition(transTool.BoundaryTransition());
                }
            }
            It1.Next();
        }
        // OCCT's commented-out second pass is left out.
    }
}
