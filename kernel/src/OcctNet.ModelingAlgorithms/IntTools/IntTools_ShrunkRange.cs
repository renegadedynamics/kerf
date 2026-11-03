// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_ShrunkRange.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_ShrunkRange.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// Computes the working (shrunk) range [t1, t2] of an edge's 3D curve: the part not covered by the tolerance spheres
/// of its vertices, its length, and its bounding box.
/// </summary>
/// <remarks>
/// A working object (plan section 6), used by the pave filler per pave block. As in OCCT, the bounding box is never
/// reset: <see cref="Perform"/> and <see cref="SetShrunkRange"/> add to it, so an object reused for another edge
/// keeps the previous boxes (OCCT's callers make one object per edge).
/// </remarks>
public class IntTools_ShrunkRange
{
    protected Edge myEdge = new();
    protected Vertex myV1 = new();
    protected Vertex myV2 = new();
    protected double myT1;
    protected double myT2;
    protected double myTS1;
    protected double myTS2;
    protected Box myBndBox = new();
    protected IntTools_Context? myCtx;
    protected bool myIsDone;
    protected bool myIsSplittable;
    protected double myLength;

    public IntTools_ShrunkRange()
    {
        myT1 = -99;
        myT2 = myT1;
        myTS1 = myT1;
        myTS2 = myT1;
        myIsDone = false;
        myIsSplittable = false;
        myLength = 0.0;
    }

    /// <summary>Sets the edge, its range and its vertices (resets the status, not the box).</summary>
    public void SetData(in Edge aE, double aT1, double aT2, in Vertex aV1, in Vertex aV2)
    {
        myEdge = aE;
        myV1 = aV1;
        myV2 = aV2;
        myT1 = aT1;
        myT2 = aT2;
        myIsDone = false;
        myIsSplittable = false;
        myLength = 0.0;
    }

    public void SetContext(IntTools_Context? aCtx) => myCtx = aCtx;

    public IntTools_Context? Context() => myCtx;

    /// <summary>Sets the shrunk range and adds the curve on it to the bounding box (without enlarging it).</summary>
    public void SetShrunkRange(double aT1, double aT2)
    {
        myTS1 = aT1;
        myTS2 = aT2;
        //
        EdgeCurveAdaptor aBAC = new(myEdge);
        Add3dCurve.Add(aBAC, aT1, aT2, 0.0, ref myBndBox);
    }

    /// <summary>The shrunk range.</summary>
    public void ShrunkRange(out double aT1, out double aT2)
    {
        aT1 = myTS1;
        aT2 = myTS2;
    }

    /// <summary>The bounding box of the edge on the shrunk range.</summary>
    public Box BndBox() => myBndBox;

    public Edge Edge() => myEdge;

    /// <summary>Computes the shrunk range, the length, the splittable flag and the box.</summary>
    public void Perform()
    {
        myIsDone = false;
        myIsSplittable = false;
        //
        // default tolerance - Precision::Confusion()
        double aDTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        // default parametric tolerance - Precision::PConfusion()
        double aPDTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        //
        if (myT2 - myT1 < aPDTol)
        {
            return;
        }
        //
        Point3D aP1 = Tool.Pnt(myV1);
        Point3D aP2 = Tool.Pnt(myV2);
        double aTolE, aTolV1, aTolV2;
        aTolE = Tool.Tolerance(myEdge);
        aTolV1 = Tool.Tolerance(myV1);
        aTolV2 = Tool.Tolerance(myV2);
        //
        if (aTolV1 < aTolE)
        {
            aTolV1 = aTolE;
        }
        //
        if (aTolV2 < aTolE)
        {
            aTolV2 = aTolE;
        }
        //
        // to have correspondence with intersection precision
        // the tolerances of vertices are increased on Precision::Confusion()
        aTolV1 += aDTol;
        aTolV2 += aDTol;

        // compute the shrunk range - part of the edge not covered
        // by the tolerance spheres of its vertices
        EdgeCurveAdaptor aBAC = new(myEdge);
        if (!BRepLib.FindValidRange(aBAC, aTolE, myT1, aP1, aTolV1, myT2, aP2, aTolV2, ref myTS1, ref myTS2))
        {
            // no valid range
            return;
        }
        if ((myTS2 - myTS1) < aPDTol)
        {
            // micro edge
            return;
        }
        //
        // compute the length of the edge on the shrunk range
        //
        // parametric tolerance for the edge
        // to be used in AbscissaPoint computations
        double aPTolE = aBAC.Resolution(aTolE);
        // for the edges with big tolerance use
        // min parametric tolerance - 1% of its range
        double aPTolEMin = (myT2 - myT1) / 100.0;
        if (aPTolE > aPTolEMin)
        {
            aPTolE = aPTolEMin;
        }
        myLength = AbscissaPoint.Length(aBAC, myTS1, myTS2, aPTolE);
        if (myLength < aDTol)
        {
            // micro edge
            return;
        }
        //
        myIsDone = true;
        //
        // check the shrunk range to have the length not less than
        // 2*aTolE+2*Precision::Confusion()
        // for the edge to have possibility to be split at least once:
        // 2*TolE - minimal diameter of tolerance sphere of splitting vertex
        // 2*Precision::Confusion() - minimal length of the new edges
        if (myLength > (2 * aTolE + 2 * aDTol))
        {
            myIsSplittable = true;
        }
        //
        // build bounding box for the edge on the shrunk range
        Add3dCurve.Add(aBAC, myTS1, myTS2, aTolE + aDTol, ref myBndBox);
    }

    /// <summary>True if the shrunk range was computed.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>False if the shrunk range is too short for the edge to be split, otherwise true.</summary>
    public bool IsSplittable() => myIsSplittable;

    /// <summary>The length of the edge on the shrunk range, if computed.</summary>
    public double Length() => myLength;
}
