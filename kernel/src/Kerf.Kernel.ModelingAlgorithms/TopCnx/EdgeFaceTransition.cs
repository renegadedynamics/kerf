// Port of: src/ModelingAlgorithms/TKHLR/TopCnx/TopCnx_EdgeFaceTransition.hxx
// Port of: src/ModelingAlgorithms/TKHLR/TopCnx/TopCnx_EdgeFaceTransition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Legacy;

/// <summary>
/// The cumulated transition of the interferences on an edge: the edge's transition through the faces met (from a
/// <see cref="CurveTransition"/>) and the boundary transition (by counting).
/// </summary>
/// <remarks>A working object, as OCCT's. The OCCT_DEBUG traces of unprocessed states are left out.</remarks>
internal sealed class EdgeFaceTransition
{
    private readonly CurveTransition _curveTransition = new();
    private int _nbBoundForward;
    private int _nbBoundReversed;

    /// <summary>An empty algorithm.</summary>
    public EdgeFaceTransition()
    {
        _nbBoundForward = 0;
        _nbBoundReversed = 0;
    }

    /// <summary>Initializes with the local description of the edge.</summary>
    public void Reset(in Direction3D tgt, in Direction3D norm, double curv)
    {
        _curveTransition.Reset(tgt, norm, curv);
        _nbBoundForward = _nbBoundReversed = 0;
    }

    /// <summary>Initializes with a linear edge.</summary>
    public void Reset(in Direction3D tgt)
    {
        _curveTransition.Reset(tgt);
        _nbBoundForward = _nbBoundReversed = 0;
    }

    /// <summary>
    /// Adds a curve element of the boundary: <paramref name="or"/> is the orientation of the interference on the
    /// boundary curve, <paramref name="tr"/> its transition and <paramref name="bTr"/> its boundary transition.
    /// </summary>
    public void AddInterference(double tole, in Direction3D tang, in Direction3D norm, double curv, Orientation or,
                                Orientation tr, Orientation bTr)
    {
        _curveTransition.Compare(tole, tang, norm, curv, tr, or);
        switch (bTr)
        {
            case Forward:
                _nbBoundForward++;
                break;
            case Reversed:
                _nbBoundReversed++;
                break;
            case Internal:
            case External:
                break;
        }
    }

    /// <summary>The current cumulated transition.</summary>
    public Orientation Transition()
    {
        Containment bef = _curveTransition.StateBefore();
        Containment aft = _curveTransition.StateAfter();
        if (bef == In)
        {
            if (aft == In)
            {
                return Internal;
            }
            else if (aft == Out)
            {
                return Reversed;
            }
            // OCCT_DEBUG trace left out
        }
        else if (bef == Out)
        {
            if (aft == In)
            {
                return Forward;
            }
            else if (aft == Out)
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
        if (_nbBoundForward > _nbBoundReversed)
        {
            return Forward;
        }
        else if (_nbBoundForward < _nbBoundReversed)
        {
            return Reversed;
        }
        // OCCT: both branches of the even/odd test return TopAbs_EXTERNAL
        else if ((_nbBoundReversed % 2) == 0)
        {
            return External;
        }
        else
        {
            return External;
        }
    }
}
