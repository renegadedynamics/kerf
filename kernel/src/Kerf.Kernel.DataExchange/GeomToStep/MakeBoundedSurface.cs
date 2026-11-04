// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>
/// The mapping of BoundedSurface from Geom to a STEP bounded_surface: B-spline surfaces (periodic ones made
/// non-periodic on a copy), Bezier surfaces (converted to B-splines) and rectangular trimmed surfaces.
/// </summary>
internal sealed class MakeBoundedSurface : GeometryRoot
{
    private StepBoundedSurface _boundedSurface;

    //=============================================================================
    // Creation d' une BoundedSurface de prostep a partir d' une BoundedSurface
    // de Geom
    //=============================================================================

    /// <summary>A bounded surface.</summary>
    public MakeBoundedSurface(BoundedSurface s, in Factors localFactors)
    {
        _done = true;
        if (s is BSplineSurface)
        {
            BSplineSurface bs = s as BSplineSurface;
            // UPDATE FMA 1-04-96
            if (s.IsUPeriodic || s.IsVPeriodic)
            {
                BSplineSurface newBS = bs.Copy() as BSplineSurface;
                newBS.SetUNotPeriodic();
                newBS.SetVNotPeriodic();
                bs = newBS;
            }
            if (bs.IsURational || bs.IsVRational)
            {
                MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface mkRatBSplineS =
                    new MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface(bs, localFactors);
                _boundedSurface = mkRatBSplineS.Value();
            }
            else
            {
                MakeBSplineSurfaceWithKnots mkBSplineS = new MakeBSplineSurfaceWithKnots(bs, localFactors);
                _boundedSurface = mkBSplineS.Value();
            }
        }
        else if (s is BezierSurface)
        {
            BezierSurface sur = s as BezierSurface;
            BSplineSurface bs = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(sur);
            if (bs.IsURational || bs.IsVRational)
            {
                MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface mkRatBSplineS =
                    new MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface(bs, localFactors);
                _boundedSurface = mkRatBSplineS.Value();
            }
            else
            {
                MakeBSplineSurfaceWithKnots mkBSplineS = new MakeBSplineSurfaceWithKnots(bs, localFactors);
                _boundedSurface = mkBSplineS.Value();
            }
        }
        else if (s is TrimmedSurface)
        {
            TrimmedSurface sur = s as TrimmedSurface;
            MakeRectangularTrimmedSurface mkRTSurf = new MakeRectangularTrimmedSurface(sur, localFactors);
            _boundedSurface = mkRTSurf.Value();
        }
        else
        {
            _done = false;
        }
    }

    /// <summary>A bounded surface (default factors).</summary>
    public MakeBoundedSurface(BoundedSurface s) : this(s, new Factors())
    {
    }

    /// <summary>The bounded_surface made.</summary>
    public StepBoundedSurface Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeBoundedSurface::Value() - no result");
        return _boundedSurface;
    }
}
