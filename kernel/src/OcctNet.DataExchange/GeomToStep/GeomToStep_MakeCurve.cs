// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCurve.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The mapping of Curve from Geom and Geom2d to a STEP curve: lines, conics, bounded curves, and trimmed curves (a
/// B-spline or Bezier basis segmented on a copy, any other basis passed untrimmed). Offset curves aren't mapped.
/// </summary>
public sealed class GeomToStep_MakeCurve : GeomToStep_Root
{
    private StepGeom_Curve theCurve;

    //=============================================================================
    // Creation d' une Curve de prostep a partir d' une Curve de Geom
    //=============================================================================

    /// <summary>A 3D curve.</summary>
    public GeomToStep_MakeCurve(Curve3D C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is LineCurve3D)
        {
            LineCurve3D L = C as LineCurve3D;
            GeomToStep_MakeLine MkLine = new GeomToStep_MakeLine(L, theLocalFactors);
            theCurve = MkLine.Value();
        }
        else if (C is ConicCurve3D)
        {
            ConicCurve3D L = C as ConicCurve3D;
            GeomToStep_MakeConic MkConic = new GeomToStep_MakeConic(L, theLocalFactors);
            theCurve = MkConic.Value();
        }
        else if (C is TrimmedCurve3D)
        {
            TrimmedCurve3D T = C as TrimmedCurve3D;
            Curve3D B = T.BasisCurve();
            //    TANT PIS, on passe la courbe de base ...
            if (B is BSplineCurve3D)
            {
                BSplineCurve3D BS = B.Copy() as BSplineCurve3D;
                BS.Segment(T.FirstParameter(), T.LastParameter());
                B = BS;
            }
            else if (B is BezierCurve3D)
            {
                BezierCurve3D BZ = B.Copy() as BezierCurve3D;
                BZ.Segment(T.FirstParameter(), T.LastParameter());
                B = BZ;
            }
            else
            {
                // OCCT_DEBUG output left out
            }
            GeomToStep_MakeCurve MkBasisC = new GeomToStep_MakeCurve(B, theLocalFactors);
            theCurve = MkBasisC.Value();
        }
        else if (C is BoundedCurve3D)
        {
            BoundedCurve3D L = C as BoundedCurve3D;
            GeomToStep_MakeBoundedCurve MkBoundedC = new GeomToStep_MakeBoundedCurve(L, theLocalFactors);
            theCurve = MkBoundedC.Value();
        }
        else
        {
            done = false;
        }
    }

    /// <summary>A 3D curve (default factors).</summary>
    public GeomToStep_MakeCurve(Curve3D C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d'une Curve de prostep a partir d' une Curve de Geom2d
    //=============================================================================

    /// <summary>A 2D curve (an indirect circle or ellipse, which STEP can't place, converted to a B-spline).</summary>
    public GeomToStep_MakeCurve(Curve2D C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is LineCurve2D)
        {
            LineCurve2D L = C as LineCurve2D;
            GeomToStep_MakeLine MkLine = new GeomToStep_MakeLine(L, theLocalFactors);
            theCurve = MkLine.Value();
        }
        else if (C is ConicCurve2D)
        {
            // ----------------------------------------------------------------------
            // A Circle of an Ellipse can be indirect. An indirect Axis in not
            // mappable onto STEP. Then to avoid changing the topology, the Circle
            // or the Ellipse are converted into BSpline Curves
            // ----------------------------------------------------------------------

            if (C is CircleCurve2D)
            {
                CircleCurve2D theC2d = C as CircleCurve2D;
                Circle2D C2d = theC2d.Circ2d();
                if (!C2d.IsDirect)
                {
                    // OCCT_DEBUG output left out
                    BSplineCurve2D aBSplineCurve2d = Geom2dConvert2D.CurveToBSplineCurve(theC2d);
                    BoundedCurve2D aBC2d = aBSplineCurve2d; // to avoid ambiguity
                    GeomToStep_MakeBoundedCurve MkBoundedC = new GeomToStep_MakeBoundedCurve(aBC2d, theLocalFactors);
                    theCurve = MkBoundedC.Value();
                }
                else
                {
                    ConicCurve2D L = C as ConicCurve2D;
                    GeomToStep_MakeConic MkConic = new GeomToStep_MakeConic(L, theLocalFactors);
                    theCurve = MkConic.Value();
                }
            }
            else if (C is EllipseCurve2D)
            {
                EllipseCurve2D theE2d = C as EllipseCurve2D;
                Ellipse2D E2d = theE2d.Elips2d();
                if (!E2d.IsDirect)
                {
                    // OCCT_DEBUG output left out
                    BSplineCurve2D aBSplineCurve2d = Geom2dConvert2D.CurveToBSplineCurve(theE2d);
                    BoundedCurve2D aBC2d = aBSplineCurve2d; // to avoid ambiguity
                    GeomToStep_MakeBoundedCurve MkBoundedC = new GeomToStep_MakeBoundedCurve(aBC2d, theLocalFactors);
                    theCurve = MkBoundedC.Value();
                }
                else
                {
                    ConicCurve2D L = C as ConicCurve2D;
                    GeomToStep_MakeConic MkConic = new GeomToStep_MakeConic(L, theLocalFactors);
                    theCurve = MkConic.Value();
                }
            }
            else
            {
                ConicCurve2D L = C as ConicCurve2D;
                GeomToStep_MakeConic MkConic = new GeomToStep_MakeConic(L, theLocalFactors);
                theCurve = MkConic.Value();
            }
        }
        else if (C is BoundedCurve2D)
        {
            // OCCT: Geom2d_TrimmedCurve is a bounded curve, so the branch below is never reached and a trimmed
            // curve goes to MakeBoundedCurve, which doesn't map it (occt-issues.md, 8D)
            BoundedCurve2D L = C as BoundedCurve2D;
            GeomToStep_MakeBoundedCurve MkBoundedC = new GeomToStep_MakeBoundedCurve(L, theLocalFactors);
            theCurve = MkBoundedC.Value();
        }
        else if (C is TrimmedCurve2D)
        {
            GeomToStep_MakeCurve aMaker = new GeomToStep_MakeCurve((C as TrimmedCurve2D).BasisCurve(), theLocalFactors);
            theCurve = aMaker.Value();
        }
        else
        {
            done = false;
        }
    }

    /// <summary>A 2D curve.</summary>
    public GeomToStep_MakeCurve(Curve2D C) : this(C, new StepData_Factors())
    {
    }

    /// <summary>The curve made.</summary>
    public StepGeom_Curve Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeCurve::Value() - no result");
        return theCurve;
    }
}
