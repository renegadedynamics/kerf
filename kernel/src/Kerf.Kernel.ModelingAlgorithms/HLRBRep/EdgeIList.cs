// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeIList.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeIList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Geometry;

namespace Kerf.Engine.HiddenLines;

/// <summary>Sorting and merging of the interferences of an edge, by their parameters.</summary>
internal static class EdgeIList
{
    /// <summary>Adds the interference I to the list IL, sorted by parameter (after the equal ones).</summary>
    public static void AddInterference(NCollection_List<Interference> il, in Interference i,
                                       EdgeInterferenceTool t)
    {
        NCollection_List<Interference>.Iterator it = new(il);
        double p = t.ParameterOfInterference(i);
        while (it.More())
        {
            if (p < t.ParameterOfInterference(it.Value()))
            {
                il.InsertBefore(i, it);
                return;
            }
            it.Next();
        }
        il.Append(i);
    }

    /// <summary>Merges the successive interferences at the same vertex into one with a complex transition.</summary>
    public static void ProcessComplex(NCollection_List<Interference> il, EdgeInterferenceTool t)
    {
        EdgeFaceTransition transTool = new();
        Direction3D tgtE, normE, tgtI, normI;
        const double tolAng = 0.0001;
        double curvE, curvI;
        NCollection_List<Interference>.Iterator it1 = new(il);

        while (it1.More())
        {
            NCollection_List<Interference>.Iterator it2 = new(it1);
            it2.Next();
            if (it2.More())
            {
                if (t.SameInterferences(it1.Value(), it2.Value())) // OCCT_DEBUG_SI's SimilarInterference left out
                {

                    t.EdgeGeometry(t.ParameterOfInterference(it1.Value()), out tgtE, out normE, out curvE);
                    transTool.Reset(tgtE, normE, curvE);
                    t.InterferenceBoundaryGeometry(it1.Value(), out tgtI, out normI, out curvI);
                    transTool.AddInterference(tolAng, tgtI, normI, curvI, it1.Value().Orientation(),
                                              it1.Value().Transition(), it1.Value().BoundaryTransition());

                    while (it2.More())
                    {
                        if (!(t.SameInterferences(it1.Value(), it2.Value())))
                        {
                            break;
                        }

                        t.InterferenceBoundaryGeometry(it2.Value(), out tgtI, out normI, out curvI);
                        transTool.AddInterference(tolAng, tgtI, normI, curvI, it2.Value().Orientation(),
                                                  it2.Value().Transition(), it2.Value().BoundaryTransition());
                        il.Remove(it2);
                    }
                    // set the cumulated transition on the interference
                    it1.ChangeValue().Transition(transTool.Transition());
                    it1.ChangeValue().BoundaryTransition(transTool.BoundaryTransition());
                }
            }
            it1.Next();
        }
        // OCCT's commented-out second pass is left out.
    }
}
