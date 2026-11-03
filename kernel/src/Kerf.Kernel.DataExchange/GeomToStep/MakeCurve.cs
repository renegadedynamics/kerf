// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCurve.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>
/// The mapping of Curve from Geom and Geom2d to a STEP curve: lines, conics, bounded curves, and trimmed curves (a
/// B-spline or Bezier basis segmented on a copy, any other basis passed untrimmed). Offset curves aren't mapped.
/// </summary>
public sealed class MakeCurve : GeometryRoot
{
    private StepCurve _curve;

    //=============================================================================
    // Creation d' une Curve de prostep a partir d' une Curve de Geom
    //=============================================================================

    /// <summary>A 3D curve.</summary>
    public MakeCurve(Curve3D c, in Factors localFactors)
    {
        _done = true;
        if (c is LineCurve3D)
        {
            LineCurve3D l = c as LineCurve3D;
            GeometryMakeLine mkLine = new GeometryMakeLine(l, localFactors);
            _curve = mkLine.Value();
        }
        else if (c is ConicCurve3D)
        {
            ConicCurve3D l = c as ConicCurve3D;
            MakeConic mkConic = new MakeConic(l, localFactors);
            _curve = mkConic.Value();
        }
        else if (c is TrimmedCurve3D)
        {
            TrimmedCurve3D t = c as TrimmedCurve3D;
            Curve3D b = t.BasisCurve();
            //    TANT PIS, on passe la courbe de base ...
            if (b is BSplineCurve3D)
            {
                BSplineCurve3D bs = b.Copy() as BSplineCurve3D;
                bs.Segment(t.FirstParameter(), t.LastParameter());
                b = bs;
            }
            else if (b is BezierCurve3D)
            {
                BezierCurve3D bz = b.Copy() as BezierCurve3D;
                bz.Segment(t.FirstParameter(), t.LastParameter());
                b = bz;
            }
            else
            {
                // OCCT_DEBUG output left out
            }
            MakeCurve mkBasisC = new MakeCurve(b, localFactors);
            _curve = mkBasisC.Value();
        }
        else if (c is BoundedCurve3D)
        {
            BoundedCurve3D l = c as BoundedCurve3D;
            MakeBoundedCurve mkBoundedC = new MakeBoundedCurve(l, localFactors);
            _curve = mkBoundedC.Value();
        }
        else
        {
            _done = false;
        }
    }

    /// <summary>A 3D curve (default factors).</summary>
    public MakeCurve(Curve3D c) : this(c, new Factors())
    {
    }

    //=============================================================================
    // Creation d'une Curve de prostep a partir d' une Curve de Geom2d
    //=============================================================================

    /// <summary>A 2D curve (an indirect circle or ellipse, which STEP can't place, converted to a B-spline).</summary>
    public MakeCurve(Curve2D c, in Factors localFactors)
    {
        _done = true;
        if (c is LineCurve2D)
        {
            LineCurve2D l = c as LineCurve2D;
            GeometryMakeLine mkLine = new GeometryMakeLine(l, localFactors);
            _curve = mkLine.Value();
        }
        else if (c is ConicCurve2D)
        {
            // ----------------------------------------------------------------------
            // A Circle of an Ellipse can be indirect. An indirect Axis in not
            // mappable onto STEP. Then to avoid changing the topology, the Circle
            // or the Ellipse are converted into BSpline Curves
            // ----------------------------------------------------------------------

            if (c is CircleCurve2D)
            {
                CircleCurve2D theC2d = c as CircleCurve2D;
                Circle2D c2d = theC2d.Circ2d();
                if (!c2d.IsDirect)
                {
                    // OCCT_DEBUG output left out
                    BSplineCurve2D bSplineCurve2d = Geom2dConvert2D.CurveToBSplineCurve(theC2d);
                    BoundedCurve2D bc2d = bSplineCurve2d; // to avoid ambiguity
                    MakeBoundedCurve mkBoundedC = new MakeBoundedCurve(bc2d, localFactors);
                    _curve = mkBoundedC.Value();
                }
                else
                {
                    ConicCurve2D l = c as ConicCurve2D;
                    MakeConic mkConic = new MakeConic(l, localFactors);
                    _curve = mkConic.Value();
                }
            }
            else if (c is EllipseCurve2D)
            {
                EllipseCurve2D theE2d = c as EllipseCurve2D;
                Ellipse2D e2d = theE2d.Elips2d();
                if (!e2d.IsDirect)
                {
                    // OCCT_DEBUG output left out
                    BSplineCurve2D bSplineCurve2d = Geom2dConvert2D.CurveToBSplineCurve(theE2d);
                    BoundedCurve2D bc2d = bSplineCurve2d; // to avoid ambiguity
                    MakeBoundedCurve mkBoundedC = new MakeBoundedCurve(bc2d, localFactors);
                    _curve = mkBoundedC.Value();
                }
                else
                {
                    ConicCurve2D l = c as ConicCurve2D;
                    MakeConic mkConic = new MakeConic(l, localFactors);
                    _curve = mkConic.Value();
                }
            }
            else
            {
                ConicCurve2D l = c as ConicCurve2D;
                MakeConic mkConic = new MakeConic(l, localFactors);
                _curve = mkConic.Value();
            }
        }
        else if (c is BoundedCurve2D)
        {
            // OCCT: Geom2d_TrimmedCurve is a bounded curve, so the branch below is never reached and a trimmed
            // curve goes to MakeBoundedCurve, which doesn't map it (occt-issues.md, 8D)
            BoundedCurve2D l = c as BoundedCurve2D;
            MakeBoundedCurve mkBoundedC = new MakeBoundedCurve(l, localFactors);
            _curve = mkBoundedC.Value();
        }
        else if (c is TrimmedCurve2D)
        {
            MakeCurve maker = new MakeCurve((c as TrimmedCurve2D).BasisCurve(), localFactors);
            _curve = maker.Value();
        }
        else
        {
            _done = false;
        }
    }

    /// <summary>A 2D curve.</summary>
    public MakeCurve(Curve2D c) : this(c, new Factors())
    {
    }

    /// <summary>The curve made.</summary>
    public StepCurve Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeCurve::Value() - no result");
        return _curve;
    }
}
