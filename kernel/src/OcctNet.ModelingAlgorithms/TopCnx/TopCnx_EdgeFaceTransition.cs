// Port of: src/ModelingAlgorithms/TKHLR/TopCnx/TopCnx_EdgeFaceTransition.hxx
// Port of: src/ModelingAlgorithms/TKHLR/TopCnx/TopCnx_EdgeFaceTransition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

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
    public void Reset(in Direction3D Tgt, in Direction3D Norm, double Curv)
    {
        myCurveTransition.Reset(Tgt, Norm, Curv);
        nbBoundForward = nbBoundReversed = 0;
    }

    /// <summary>Initializes with a linear edge.</summary>
    public void Reset(in Direction3D Tgt)
    {
        myCurveTransition.Reset(Tgt);
        nbBoundForward = nbBoundReversed = 0;
    }

    /// <summary>
    /// Adds a curve element of the boundary: <paramref name="Or"/> is the orientation of the interference on the
    /// boundary curve, <paramref name="Tr"/> its transition and <paramref name="BTr"/> its boundary transition.
    /// </summary>
    public void AddInterference(double Tole, in Direction3D Tang, in Direction3D Norm, double Curv, Orientation Or,
                                Orientation Tr, Orientation BTr)
    {
        myCurveTransition.Compare(Tole, Tang, Norm, Curv, Tr, Or);
        switch (BTr)
        {
            case Forward:
                nbBoundForward++;
                break;
            case Reversed:
                nbBoundReversed++;
                break;
            case Internal:
            case External:
                break;
        }
    }

    /// <summary>The current cumulated transition.</summary>
    public Orientation Transition()
    {
        Containment Bef = myCurveTransition.StateBefore();
        Containment Aft = myCurveTransition.StateAfter();
        if (Bef == In)
        {
            if (Aft == In)
            {
                return Internal;
            }
            else if (Aft == Out)
            {
                return Reversed;
            }
            // OCCT_DEBUG trace left out
        }
        else if (Bef == Out)
        {
            if (Aft == In)
            {
                return Forward;
            }
            else if (Aft == Out)
            {
                return External;
            }
            // OCCT_DEBUG trace left out
        }
        // OCCT_DEBUG trace left out
        return Internal;
    }

    /// <summary>The current cumulated boundary transition.</summary>
    public Orientation BoundaryTransition()
    {
        if (nbBoundForward > nbBoundReversed)
        {
            return Forward;
        }
        else if (nbBoundForward < nbBoundReversed)
        {
            return Reversed;
        }
        // OCCT: both branches of the even/odd test return TopAbs_EXTERNAL
        else if ((nbBoundReversed % 2) == 0)
        {
            return External;
        }
        else
        {
            return External;
        }
    }
}
