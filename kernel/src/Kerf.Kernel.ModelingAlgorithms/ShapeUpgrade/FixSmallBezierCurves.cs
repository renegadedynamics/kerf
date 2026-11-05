// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallBezierCurves.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallBezierCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Replaces the small segments a conversion to Beziers leaves at an edge's end by one: the edge's curves between the
/// parameters are approximated by B-splines and converted again with the split tools, which must give one segment.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the edge; it re-initializes the split tools it was given (the wire
/// divider's own, as in OCCT).
/// </remarks>
internal class FixSmallBezierCurves : FixSmallCurves
{
    /// <summary>Empty constructor.</summary>
    public FixSmallBezierCurves()
    {
    }

    /// <summary>
    /// Approximates the edge's 3D curve, pcurve and (for a seam) reversed pcurve between <paramref name="first"/> and
    /// <paramref name="last"/> by single Bezier segments; false if a curve gives more than one segment or fails.
    /// </summary>
    public override bool Approx(ref Curve3D? curve3d,
                                ref Curve2D? curve2d,
                                ref Curve2D? curve2dR,
                                ref double first,
                                ref double last)
    {
        var sae = new EdgeAnalysis();
        double f, l;
        if (sae.Curve3d(_edge, out Curve3D? c3d, out f, out l, false))
        {
            if (first < f)
            {
                first = f;
            }
            if (last > l)
            {
                last = l;
            }
            Curve3D trc = new TrimmedCurve3D(c3d!, first, last);
            Continuity cont = trc.Continuity;
            if (cont == C3 || cont == CN)
            {
                cont = C2;
            }
            try
            {
                var aproxCurve = new ApproxCurve(trc, Precision(), cont, 1, 9);
                if (aproxCurve.IsDone)
                {
                    Curve3D? newCurve = aproxCurve.InternalCurve();
                    _splitCurve3dTool!.Init(aproxCurve.InternalCurve()!, first, last);
                    _splitCurve3dTool.Perform(true);
                    if (!_splitCurve3dTool.Status(ExtendStatus.Fail))
                    {
                        NCollection_HArray1<Curve3D> theSegments3d = _splitCurve3dTool.GetCurves()!;
                        if (theSegments3d.Length() > 1)
                        {
                            return false;
                        }
                        curve3d = theSegments3d.Value(1);
                    }
                }
            }
            catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
            {
                // OCCT_DEBUG output left out
                return false;
            }
        }
        if (_face.IsNull)
        {
            return true;
        }
        Surface surf = Tool.Surface(_face, out _)!;
        var ads = new SurfaceAdaptor(surf); // = new GeomAdaptor_Surface(aSurf);
        double prec = Math.Max(ads.UResolution(Precision()), ads.VResolution(Precision()));
        if (sae.PCurve(_edge, _face, out Curve2D? c2d, out f, out l, false))
        {
            if (first < f)
            {
                first = f;
            }
            if (last > l)
            {
                last = l;
            }
            Curve2D trc2d = new TrimmedCurve2D(c2d!, first, last);
            Continuity cont = trc2d.Continuity;
            try
            {
                var aproxCurve2d = new ApproxCurve2D(trc2d, prec, cont, 1, 9);
                if (aproxCurve2d.IsDone)
                {
                    Curve2D? newCurve = aproxCurve2d.InternalCurve();
                    _splitCurve2dTool!.Init(aproxCurve2d.InternalCurve()!, first, last);
                    _splitCurve2dTool.Perform(true);
                    if (_splitCurve2dTool.Status(ExtendStatus.Fail))
                    {
                        return false;
                    }
                    NCollection_HArray1<Curve2D> theSegments2d = _splitCurve2dTool.GetCurves()!;
                    if (theSegments2d.Length() > 1)
                    {
                        return false; // ShapeAnalysis_Surface
                    }
                    curve2d = theSegments2d.Value(1);
                }
            }
            catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
            {
                // OCCT_DEBUG output left out
                return false;
            }
        }
        bool isSeam = Tool.IsClosed(_edge, _face);
        if (isSeam)
        {
            Shape tmpE = _edge.Reversed();
            Edge erev = ShapeCasts.Edge(tmpE);
            if (sae.PCurve(erev, _face, out Curve2D? c2, out _, out _, false))
            {
                // OCCT compares with the forward pcurve's range here (f, l), as written
                if (first > f)
                {
                    first = f;
                }
                if (last > l)
                {
                    last = l;
                }
                Curve2D trc2d = new TrimmedCurve2D(c2!, first, last);
                Continuity cont = trc2d.Continuity;
                var aproxCurve2d = new ApproxCurve2D(trc2d, prec, cont, 1, 9);
                try
                {
                    if (aproxCurve2d.IsDone)
                    {
                        Curve2D? newCurve = aproxCurve2d.InternalCurve();
                        _splitCurve2dTool!.Init(aproxCurve2d.InternalCurve()!, first, last);
                        _splitCurve2dTool.Perform(true);
                        if (!_splitCurve2dTool.Status(ExtendStatus.Done))
                        {
                            return false;
                        }
                        NCollection_HArray1<Curve2D> theSegments2d = _splitCurve2dTool.GetCurves()!;
                        if (theSegments2d.Length() > 1)
                        {
                            return false; // ShapeAnalysis_Surface
                        }
                        curve2dR = theSegments2d.Value(1);
                    }
                }
                catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
                {
                    // OCCT_DEBUG output left out
                    return false;
                }
            }
        }
        return true;
    }
}
