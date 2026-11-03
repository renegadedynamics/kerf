// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallBezierCurves.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallBezierCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Replaces the small segments a conversion to Beziers leaves at an edge's end by one: the edge's curves between the
/// parameters are approximated by B-splines and converted again with the split tools, which must give one segment.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the edge; it re-initializes the split tools it was given (the wire
/// divider's own, as in OCCT).
/// </remarks>
public class ShapeUpgrade_FixSmallBezierCurves : ShapeUpgrade_FixSmallCurves
{
    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_FixSmallBezierCurves()
    {
    }

    /// <summary>
    /// Approximates the edge's 3D curve, pcurve and (for a seam) reversed pcurve between <paramref name="First"/> and
    /// <paramref name="Last"/> by single Bezier segments; false if a curve gives more than one segment or fails.
    /// </summary>
    public override bool Approx(ref Curve3D? Curve3d,
                                ref Curve2D? Curve2d,
                                ref Curve2D? Curve2dR,
                                ref double First,
                                ref double Last)
    {
        var sae = new ShapeAnalysis_Edge();
        double f, l;
        if (sae.Curve3d(myEdge, out Curve3D? c3d, out f, out l, false))
        {
            if (First < f)
            {
                First = f;
            }
            if (Last > l)
            {
                Last = l;
            }
            Curve3D trc = new TrimmedCurve3D(c3d!, First, Last);
            Continuity aCont = trc.Continuity();
            if (aCont == C3 || aCont == CN)
            {
                aCont = C2;
            }
            try
            {
                var AproxCurve = new ApproxCurve(trc, Precision(), aCont, 1, 9);
                if (AproxCurve.IsDone)
                {
                    Curve3D? newCurve = AproxCurve.InternalCurve();
                    mySplitCurve3dTool!.Init(AproxCurve.InternalCurve()!, First, Last);
                    mySplitCurve3dTool.Perform(true);
                    if (!mySplitCurve3dTool.Status(ShapeExtend_FAIL))
                    {
                        NCollection_HArray1<Curve3D> theSegments3d = mySplitCurve3dTool.GetCurves()!;
                        if (theSegments3d.Length() > 1)
                        {
                            return false;
                        }
                        Curve3d = theSegments3d.Value(1);
                    }
                }
            }
            catch (Exception anException) when (ShapeUpgrade.IsCaughtFailure(anException))
            {
                // OCCT_DEBUG output left out
                return false;
            }
        }
        if (myFace.IsNull)
        {
            return true;
        }
        Surface aSurf = Tool.Surface(myFace, out _)!;
        var ads = new SurfaceAdaptor(aSurf); // = new GeomAdaptor_Surface(aSurf);
        double prec = Math.Max(ads.UResolution(Precision()), ads.VResolution(Precision()));
        if (sae.PCurve(myEdge, myFace, out Curve2D? c2d, out f, out l, false))
        {
            if (First < f)
            {
                First = f;
            }
            if (Last > l)
            {
                Last = l;
            }
            Curve2D trc2d = new TrimmedCurve2D(c2d!, First, Last);
            Continuity aCont = trc2d.Continuity();
            try
            {
                var AproxCurve2d = new ApproxCurve2D(trc2d, prec, aCont, 1, 9);
                if (AproxCurve2d.IsDone)
                {
                    Curve2D? newCurve = AproxCurve2d.InternalCurve();
                    mySplitCurve2dTool!.Init(AproxCurve2d.InternalCurve()!, First, Last);
                    mySplitCurve2dTool.Perform(true);
                    if (mySplitCurve2dTool.Status(ShapeExtend_FAIL))
                    {
                        return false;
                    }
                    NCollection_HArray1<Curve2D> theSegments2d = mySplitCurve2dTool.GetCurves()!;
                    if (theSegments2d.Length() > 1)
                    {
                        return false; // ShapeAnalysis_Surface
                    }
                    Curve2d = theSegments2d.Value(1);
                }
            }
            catch (Exception anException) when (ShapeUpgrade.IsCaughtFailure(anException))
            {
                // OCCT_DEBUG output left out
                return false;
            }
        }
        bool isSeam = Tool.IsClosed(myEdge, myFace);
        if (isSeam)
        {
            Shape tmpE = myEdge.Reversed();
            Edge erev = ShapeCasts.Edge(tmpE);
            if (sae.PCurve(erev, myFace, out Curve2D? c2, out _, out _, false))
            {
                // OCCT compares with the forward pcurve's range here (f, l), as written
                if (First > f)
                {
                    First = f;
                }
                if (Last > l)
                {
                    Last = l;
                }
                Curve2D trc2d = new TrimmedCurve2D(c2!, First, Last);
                Continuity aCont = trc2d.Continuity();
                var AproxCurve2d = new ApproxCurve2D(trc2d, prec, aCont, 1, 9);
                try
                {
                    if (AproxCurve2d.IsDone)
                    {
                        Curve2D? newCurve = AproxCurve2d.InternalCurve();
                        mySplitCurve2dTool!.Init(AproxCurve2d.InternalCurve()!, First, Last);
                        mySplitCurve2dTool.Perform(true);
                        if (!mySplitCurve2dTool.Status(ShapeExtend_DONE))
                        {
                            return false;
                        }
                        NCollection_HArray1<Curve2D> theSegments2d = mySplitCurve2dTool.GetCurves()!;
                        if (theSegments2d.Length() > 1)
                        {
                            return false; // ShapeAnalysis_Surface
                        }
                        Curve2dR = theSegments2d.Value(1);
                    }
                }
                catch (Exception anException) when (ShapeUpgrade.IsCaughtFailure(anException))
                {
                    // OCCT_DEBUG output left out
                    return false;
                }
            }
        }
        return true;
    }
}
