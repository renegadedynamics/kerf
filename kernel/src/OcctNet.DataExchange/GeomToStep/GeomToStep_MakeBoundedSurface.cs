// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The mapping of BoundedSurface from Geom to a STEP bounded_surface: B-spline surfaces (periodic ones made
/// non-periodic on a copy), Bezier surfaces (converted to B-splines) and rectangular trimmed surfaces.
/// </summary>
public sealed class GeomToStep_MakeBoundedSurface : GeomToStep_Root
{
    private StepGeom_BoundedSurface theBoundedSurface;

    //=============================================================================
    // Creation d' une BoundedSurface de prostep a partir d' une BoundedSurface
    // de Geom
    //=============================================================================

    /// <summary>A bounded surface.</summary>
    public GeomToStep_MakeBoundedSurface(BoundedSurface S, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (S is BSplineSurface)
        {
            BSplineSurface BS = S as BSplineSurface;
            // UPDATE FMA 1-04-96
            if (S.IsUPeriodic || S.IsVPeriodic)
            {
                BSplineSurface newBS = BS.Copy() as BSplineSurface;
                newBS.SetUNotPeriodic();
                newBS.SetVNotPeriodic();
                BS = newBS;
            }
            if (BS.IsURational || BS.IsVRational)
            {
                GeomToStep_MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface MkRatBSplineS =
                    new GeomToStep_MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface(BS, theLocalFactors);
                theBoundedSurface = MkRatBSplineS.Value();
            }
            else
            {
                GeomToStep_MakeBSplineSurfaceWithKnots MkBSplineS = new GeomToStep_MakeBSplineSurfaceWithKnots(BS, theLocalFactors);
                theBoundedSurface = MkBSplineS.Value();
            }
        }
        else if (S is BezierSurface)
        {
            BezierSurface Sur = S as BezierSurface;
            BSplineSurface BS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(Sur);
            if (BS.IsURational || BS.IsVRational)
            {
                GeomToStep_MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface MkRatBSplineS =
                    new GeomToStep_MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface(BS, theLocalFactors);
                theBoundedSurface = MkRatBSplineS.Value();
            }
            else
            {
                GeomToStep_MakeBSplineSurfaceWithKnots MkBSplineS = new GeomToStep_MakeBSplineSurfaceWithKnots(BS, theLocalFactors);
                theBoundedSurface = MkBSplineS.Value();
            }
        }
        else if (S is TrimmedSurface)
        {
            TrimmedSurface Sur = S as TrimmedSurface;
            GeomToStep_MakeRectangularTrimmedSurface MkRTSurf = new GeomToStep_MakeRectangularTrimmedSurface(Sur, theLocalFactors);
            theBoundedSurface = MkRTSurf.Value();
        }
        else
        {
            done = false;
        }
    }

    /// <summary>A bounded surface (default factors).</summary>
    public GeomToStep_MakeBoundedSurface(BoundedSurface S) : this(S, new StepData_Factors())
    {
    }

    /// <summary>The bounded_surface made.</summary>
    public StepGeom_BoundedSurface Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeBoundedSurface::Value() - no result");
        return theBoundedSurface;
    }
}
