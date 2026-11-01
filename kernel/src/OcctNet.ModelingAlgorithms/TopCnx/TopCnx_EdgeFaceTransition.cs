// Port of: src/ModelingAlgorithms/TKHLR/TopCnx/TopCnx_EdgeFaceTransition.hxx
// Port of: src/ModelingAlgorithms/TKHLR/TopCnx/TopCnx_EdgeFaceTransition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The cumulated transition of the interferences on an edge: the edge's transition through the faces met (from a
/// <see cref="TopTrans_CurveTransition"/>) and the boundary transition (by counting).
/// </summary>
/// <remarks>A working object, as OCCT's. The OCCT_DEBUG traces of unprocessed states are left out.</remarks>
public sealed class TopCnx_EdgeFaceTransition
{
    private readonly TopTrans_CurveTransition myCurveTransition = new();
    private int nbBoundForward;
    private int nbBoundReversed;

    /// <summary>An empty algorithm.</summary>
    public TopCnx_EdgeFaceTransition()
    {
        nbBoundForward = 0;
        nbBoundReversed = 0;
    }

    /// <summary>Initializes with the local description of the edge.</summary>
    public void Reset(in gp_Dir Tgt, in gp_Dir Norm, double Curv)
    {
        myCurveTransition.Reset(Tgt, Norm, Curv);
        nbBoundForward = nbBoundReversed = 0;
    }

    /// <summary>Initializes with a linear edge.</summary>
    public void Reset(in gp_Dir Tgt)
    {
        myCurveTransition.Reset(Tgt);
        nbBoundForward = nbBoundReversed = 0;
    }

    /// <summary>
    /// Adds a curve element of the boundary: <paramref name="Or"/> is the orientation of the interference on the
    /// boundary curve, <paramref name="Tr"/> its transition and <paramref name="BTr"/> its boundary transition.
    /// </summary>
    public void AddInterference(double Tole, in gp_Dir Tang, in gp_Dir Norm, double Curv, TopAbs_Orientation Or,
                                TopAbs_Orientation Tr, TopAbs_Orientation BTr)
    {
        myCurveTransition.Compare(Tole, Tang, Norm, Curv, Tr, Or);
        switch (BTr)
        {
            case TopAbs_FORWARD:
                nbBoundForward++;
                break;
            case TopAbs_REVERSED:
                nbBoundReversed++;
                break;
            case TopAbs_INTERNAL:
            case TopAbs_EXTERNAL:
                break;
        }
    }

    /// <summary>The current cumulated transition.</summary>
    public TopAbs_Orientation Transition()
    {
        TopAbs_State Bef = myCurveTransition.StateBefore();
        TopAbs_State Aft = myCurveTransition.StateAfter();
        if (Bef == TopAbs_IN)
        {
            if (Aft == TopAbs_IN)
            {
                return TopAbs_INTERNAL;
            }
            else if (Aft == TopAbs_OUT)
            {
                return TopAbs_REVERSED;
            }
            // OCCT_DEBUG trace left out
        }
        else if (Bef == TopAbs_OUT)
        {
            if (Aft == TopAbs_IN)
            {
                return TopAbs_FORWARD;
            }
            else if (Aft == TopAbs_OUT)
            {
                return TopAbs_EXTERNAL;
            }
            // OCCT_DEBUG trace left out
        }
        // OCCT_DEBUG trace left out
        return TopAbs_INTERNAL;
    }

    /// <summary>The current cumulated boundary transition.</summary>
    public TopAbs_Orientation BoundaryTransition()
    {
        if (nbBoundForward > nbBoundReversed)
        {
            return TopAbs_FORWARD;
        }
        else if (nbBoundForward < nbBoundReversed)
        {
            return TopAbs_REVERSED;
        }
        // OCCT: both branches of the even/odd test return TopAbs_EXTERNAL
        else if ((nbBoundReversed % 2) == 0)
        {
            return TopAbs_EXTERNAL;
        }
        else
        {
            return TopAbs_EXTERNAL;
        }
    }
}
