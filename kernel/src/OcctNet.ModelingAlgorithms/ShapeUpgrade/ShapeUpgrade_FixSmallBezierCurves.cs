// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallBezierCurves.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallBezierCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
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
    public override bool Approx(ref Geom_Curve? Curve3d,
                                ref Geom2d_Curve? Curve2d,
                                ref Geom2d_Curve? Curve2dR,
                                ref double First,
                                ref double Last)
    {
        var sae = new ShapeAnalysis_Edge();
        double f, l;
        if (sae.Curve3d(myEdge, out Geom_Curve? c3d, out f, out l, false))
        {
            if (First < f)
            {
                First = f;
            }
            if (Last > l)
            {
                Last = l;
            }
            Geom_Curve trc = new Geom_TrimmedCurve(c3d!, First, Last);
            Continuity aCont = trc.Continuity();
            if (aCont == C3 || aCont == CN)
            {
                aCont = C2;
            }
            try
            {
                var AproxCurve = new GeomConvert_ApproxCurve(trc, Precision(), aCont, 1, 9);
                if (AproxCurve.IsDone())
                {
                    Geom_Curve? newCurve = AproxCurve.InternalCurve();
                    mySplitCurve3dTool!.Init(AproxCurve.InternalCurve()!, First, Last);
                    mySplitCurve3dTool.Perform(true);
                    if (!mySplitCurve3dTool.Status(ShapeExtend_FAIL))
                    {
                        NCollection_HArray1<Geom_Curve> theSegments3d = mySplitCurve3dTool.GetCurves()!;
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
        if (myFace.IsNull())
        {
            return true;
        }
        Geom_Surface aSurf = BRep_Tool.Surface(myFace, out _)!;
        var ads = new GeomAdaptor_Surface(aSurf); // = new GeomAdaptor_Surface(aSurf);
        double prec = Math.Max(ads.UResolution(Precision()), ads.VResolution(Precision()));
        if (sae.PCurve(myEdge, myFace, out Geom2d_Curve? c2d, out f, out l, false))
        {
            if (First < f)
            {
                First = f;
            }
            if (Last > l)
            {
                Last = l;
            }
            Geom2d_Curve trc2d = new Geom2d_TrimmedCurve(c2d!, First, Last);
            Continuity aCont = trc2d.Continuity();
            try
            {
                var AproxCurve2d = new Geom2dConvert_ApproxCurve(trc2d, prec, aCont, 1, 9);
                if (AproxCurve2d.IsDone())
                {
                    Geom2d_Curve? newCurve = AproxCurve2d.InternalCurve();
                    mySplitCurve2dTool!.Init(AproxCurve2d.InternalCurve()!, First, Last);
                    mySplitCurve2dTool.Perform(true);
                    if (mySplitCurve2dTool.Status(ShapeExtend_FAIL))
                    {
                        return false;
                    }
                    NCollection_HArray1<Geom2d_Curve> theSegments2d = mySplitCurve2dTool.GetCurves()!;
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
        bool isSeam = BRep_Tool.IsClosed(myEdge, myFace);
        if (isSeam)
        {
            TopoDS_Shape tmpE = myEdge.Reversed();
            TopoDS_Edge erev = TopoDS.Edge(tmpE);
            if (sae.PCurve(erev, myFace, out Geom2d_Curve? c2, out _, out _, false))
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
                Geom2d_Curve trc2d = new Geom2d_TrimmedCurve(c2!, First, Last);
                Continuity aCont = trc2d.Continuity();
                var AproxCurve2d = new Geom2dConvert_ApproxCurve(trc2d, prec, aCont, 1, 9);
                try
                {
                    if (AproxCurve2d.IsDone())
                    {
                        Geom2d_Curve? newCurve = AproxCurve2d.InternalCurve();
                        mySplitCurve2dTool!.Init(AproxCurve2d.InternalCurve()!, First, Last);
                        mySplitCurve2dTool.Perform(true);
                        if (!mySplitCurve2dTool.Status(ShapeExtend_DONE))
                        {
                            return false;
                        }
                        NCollection_HArray1<Geom2d_Curve> theSegments2d = mySplitCurve2dTool.GetCurves()!;
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
