// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_ShrunkRange.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_ShrunkRange.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// Computes the working (shrunk) range [t1, t2] of an edge's 3D curve: the part not covered by the tolerance spheres
/// of its vertices, its length, and its bounding box.
/// </summary>
/// <remarks>
/// A working object (plan section 6), used by the pave filler per pave block. As in OCCT, the bounding box is never
/// reset: <see cref="Perform"/> and <see cref="SetShrunkRange"/> add to it, so an object reused for another edge
/// keeps the previous boxes (OCCT's callers make one object per edge).
/// </remarks>
internal class EdgeShrunkRange
{
    protected Edge _edge = new();
    protected Vertex _v1 = new();
    protected Vertex _v2 = new();
    protected double _t1;
    protected double _t2;
    protected double _ts1;
    protected double _ts2;
    protected Box _bndBox = new();
    protected IntersectionContext? _ctx;
    protected bool _isDone;
    protected bool _isSplittable;
    protected double _length;

    public EdgeShrunkRange()
    {
        _t1 = -99;
        _t2 = _t1;
        _ts1 = _t1;
        _ts2 = _t1;
        _isDone = false;
        _isSplittable = false;
        _length = 0.0;
    }

    /// <summary>Sets the edge, its range and its vertices (resets the status, not the box).</summary>
    public void SetData(in Edge aE, double aT1, double aT2, in Vertex aV1, in Vertex aV2)
    {
        _edge = aE;
        _v1 = aV1;
        _v2 = aV2;
        _t1 = aT1;
        _t2 = aT2;
        _isDone = false;
        _isSplittable = false;
        _length = 0.0;
    }

    public void SetContext(IntersectionContext? aCtx) => _ctx = aCtx;

    public IntersectionContext? Context() => _ctx;

    /// <summary>Sets the shrunk range and adds the curve on it to the bounding box (without enlarging it).</summary>
    public void SetShrunkRange(double aT1, double aT2)
    {
        _ts1 = aT1;
        _ts2 = aT2;
        //
        EdgeCurveAdaptor bac = new(_edge);
        Add3dCurve.Add(bac, aT1, aT2, 0.0, ref _bndBox);
    }

    /// <summary>The shrunk range.</summary>
    public void ShrunkRange(out double aT1, out double aT2)
    {
        aT1 = _ts1;
        aT2 = _ts2;
    }

    /// <summary>The bounding box of the edge on the shrunk range.</summary>
    public Box BndBox() => _bndBox;

    public Edge Edge() => _edge;

    /// <summary>Computes the shrunk range, the length, the splittable flag and the box.</summary>
    public void Perform()
    {
        _isDone = false;
        _isSplittable = false;
        //
        // default tolerance - Precision::Confusion()
        double dTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        // default parametric tolerance - Precision::PConfusion()
        double pdTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        //
        if (_t2 - _t1 < pdTol)
        {
            return;
        }
        //
        Point3D p1 = Tool.Pnt(_v1);
        Point3D p2 = Tool.Pnt(_v2);
        double tolE, tolV1, tolV2;
        tolE = Tool.Tolerance(_edge);
        tolV1 = Tool.Tolerance(_v1);
        tolV2 = Tool.Tolerance(_v2);
        //
        if (tolV1 < tolE)
        {
            tolV1 = tolE;
        }
        //
        if (tolV2 < tolE)
        {
            tolV2 = tolE;
        }
        //
        // to have correspondence with intersection precision
        // the tolerances of vertices are increased on Precision::Confusion()
        tolV1 += dTol;
        tolV2 += dTol;

        // compute the shrunk range - part of the edge not covered
        // by the tolerance spheres of its vertices
        EdgeCurveAdaptor bac = new(_edge);
        if (!BrepLibrary.FindValidRange(bac, tolE, _t1, p1, tolV1, _t2, p2, tolV2, ref _ts1, ref _ts2))
        {
            // no valid range
            return;
        }
        if ((_ts2 - _ts1) < pdTol)
        {
            // micro edge
            return;
        }
        //
        // compute the length of the edge on the shrunk range
        //
        // parametric tolerance for the edge
        // to be used in AbscissaPoint computations
        double pTolE = bac.Resolution(tolE);
        // for the edges with big tolerance use
        // min parametric tolerance - 1% of its range
        double pTolEMin = (_t2 - _t1) / 100.0;
        if (pTolE > pTolEMin)
        {
            pTolE = pTolEMin;
        }
        _length = AbscissaPoint.Length(bac, _ts1, _ts2, pTolE);
        if (_length < dTol)
        {
            // micro edge
            return;
        }
        //
        _isDone = true;
        //
        // check the shrunk range to have the length not less than
        // 2*aTolE+2*Precision::Confusion()
        // for the edge to have possibility to be split at least once:
        // 2*TolE - minimal diameter of tolerance sphere of splitting vertex
        // 2*Precision::Confusion() - minimal length of the new edges
        if (_length > (2 * tolE + 2 * dTol))
        {
            _isSplittable = true;
        }
        //
        // build bounding box for the edge on the shrunk range
        Add3dCurve.Add(bac, _ts1, _ts2, tolE + dTol, ref _bndBox);
    }

    /// <summary>True if the shrunk range was computed.</summary>
    public bool IsDone => _isDone;

    /// <summary>False if the shrunk range is too short for the edge to be split, otherwise true.</summary>
    public bool IsSplittable => _isSplittable;

    /// <summary>The length of the edge on the shrunk range, if computed.</summary>
    public double Length() => _length;
}
