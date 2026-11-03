// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCurve.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

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
    public GeomToStep_MakeCurve(Geom_Curve C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is Geom_Line)
        {
            Geom_Line L = C as Geom_Line;
            GeomToStep_MakeLine MkLine = new GeomToStep_MakeLine(L, theLocalFactors);
            theCurve = MkLine.Value();
        }
        else if (C is Geom_Conic)
        {
            Geom_Conic L = C as Geom_Conic;
            GeomToStep_MakeConic MkConic = new GeomToStep_MakeConic(L, theLocalFactors);
            theCurve = MkConic.Value();
        }
        else if (C is Geom_TrimmedCurve)
        {
            Geom_TrimmedCurve T = C as Geom_TrimmedCurve;
            Geom_Curve B = T.BasisCurve();
            //    TANT PIS, on passe la courbe de base ...
            if (B is Geom_BSplineCurve)
            {
                Geom_BSplineCurve BS = B.Copy() as Geom_BSplineCurve;
                BS.Segment(T.FirstParameter(), T.LastParameter());
                B = BS;
            }
            else if (B is Geom_BezierCurve)
            {
                Geom_BezierCurve BZ = B.Copy() as Geom_BezierCurve;
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
        else if (C is Geom_BoundedCurve)
        {
            Geom_BoundedCurve L = C as Geom_BoundedCurve;
            GeomToStep_MakeBoundedCurve MkBoundedC = new GeomToStep_MakeBoundedCurve(L, theLocalFactors);
            theCurve = MkBoundedC.Value();
        }
        else
        {
            done = false;
        }
    }

    /// <summary>A 3D curve (default factors).</summary>
    public GeomToStep_MakeCurve(Geom_Curve C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d'une Curve de prostep a partir d' une Curve de Geom2d
    //=============================================================================

    /// <summary>A 2D curve (an indirect circle or ellipse, which STEP can't place, converted to a B-spline).</summary>
    public GeomToStep_MakeCurve(Geom2d_Curve C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is Geom2d_Line)
        {
            Geom2d_Line L = C as Geom2d_Line;
            GeomToStep_MakeLine MkLine = new GeomToStep_MakeLine(L, theLocalFactors);
            theCurve = MkLine.Value();
        }
        else if (C is Geom2d_Conic)
        {
            // ----------------------------------------------------------------------
            // A Circle of an Ellipse can be indirect. An indirect Axis in not
            // mappable onto STEP. Then to avoid changing the topology, the Circle
            // or the Ellipse are converted into BSpline Curves
            // ----------------------------------------------------------------------

            if (C is Geom2d_Circle)
            {
                Geom2d_Circle theC2d = C as Geom2d_Circle;
                Circle2D C2d = theC2d.Circ2d();
                if (!C2d.IsDirect)
                {
                    // OCCT_DEBUG output left out
                    Geom2d_BSplineCurve aBSplineCurve2d = Geom2dConvert.CurveToBSplineCurve(theC2d);
                    Geom2d_BoundedCurve aBC2d = aBSplineCurve2d; // to avoid ambiguity
                    GeomToStep_MakeBoundedCurve MkBoundedC = new GeomToStep_MakeBoundedCurve(aBC2d, theLocalFactors);
                    theCurve = MkBoundedC.Value();
                }
                else
                {
                    Geom2d_Conic L = C as Geom2d_Conic;
                    GeomToStep_MakeConic MkConic = new GeomToStep_MakeConic(L, theLocalFactors);
                    theCurve = MkConic.Value();
                }
            }
            else if (C is Geom2d_Ellipse)
            {
                Geom2d_Ellipse theE2d = C as Geom2d_Ellipse;
                Ellipse2D E2d = theE2d.Elips2d();
                if (!E2d.IsDirect)
                {
                    // OCCT_DEBUG output left out
                    Geom2d_BSplineCurve aBSplineCurve2d = Geom2dConvert.CurveToBSplineCurve(theE2d);
                    Geom2d_BoundedCurve aBC2d = aBSplineCurve2d; // to avoid ambiguity
                    GeomToStep_MakeBoundedCurve MkBoundedC = new GeomToStep_MakeBoundedCurve(aBC2d, theLocalFactors);
                    theCurve = MkBoundedC.Value();
                }
                else
                {
                    Geom2d_Conic L = C as Geom2d_Conic;
                    GeomToStep_MakeConic MkConic = new GeomToStep_MakeConic(L, theLocalFactors);
                    theCurve = MkConic.Value();
                }
            }
            else
            {
                Geom2d_Conic L = C as Geom2d_Conic;
                GeomToStep_MakeConic MkConic = new GeomToStep_MakeConic(L, theLocalFactors);
                theCurve = MkConic.Value();
            }
        }
        else if (C is Geom2d_BoundedCurve)
        {
            // OCCT: Geom2d_TrimmedCurve is a bounded curve, so the branch below is never reached and a trimmed
            // curve goes to MakeBoundedCurve, which doesn't map it (occt-issues.md, 8D)
            Geom2d_BoundedCurve L = C as Geom2d_BoundedCurve;
            GeomToStep_MakeBoundedCurve MkBoundedC = new GeomToStep_MakeBoundedCurve(L, theLocalFactors);
            theCurve = MkBoundedC.Value();
        }
        else if (C is Geom2d_TrimmedCurve)
        {
            GeomToStep_MakeCurve aMaker = new GeomToStep_MakeCurve((C as Geom2d_TrimmedCurve).BasisCurve(), theLocalFactors);
            theCurve = aMaker.Value();
        }
        else
        {
            done = false;
        }
    }

    /// <summary>A 2D curve.</summary>
    public GeomToStep_MakeCurve(Geom2d_Curve C) : this(C, new StepData_Factors())
    {
    }

    /// <summary>The curve made.</summary>
    public StepGeom_Curve Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeCurve::Value() - no result");
        return theCurve;
    }
}
