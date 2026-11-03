// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeRectangularTrimmedSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeRectangularTrimmedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Geometry;

namespace Occt;

/// <summary>The mapping of RectangularTrimmedSurface from Geom to a STEP rectangular_trimmed_surface.</summary>
public sealed class GeomToStep_MakeRectangularTrimmedSurface : GeomToStep_Root
{
    private StepGeom_RectangularTrimmedSurface theRectangularTrimmedSurface;

    //=============================================================================
    // Creation d' une rectangular_trimmed_surface de STEP
    // a partir d' une RectangularTrimmedSurface de Geom
    //=============================================================================

    /// <summary>
    /// A rectangular trimmed surface: its basis surface, and its bounds with angles in degrees (OCCT writes 180/π,
    /// not the plane angle factor) and lengths divided by the length factor. Not done if the basis isn't mapped.
    /// </summary>
    public GeomToStep_MakeRectangularTrimmedSurface(TrimmedSurface RTSurf, in StepData_Factors theLocalFactors)
    {
        StepGeom_RectangularTrimmedSurface StepRTS = new StepGeom_RectangularTrimmedSurface();

        TCollection_HAsciiString aName = new TCollection_HAsciiString("");

        GeomToStep_MakeSurface mkSurf = new GeomToStep_MakeSurface(RTSurf.BasisSurface(), theLocalFactors);
        if (!mkSurf.IsDone())
        {
            done = false;
            return;
        }
        StepGeom_Surface StepSurf = mkSurf.Value();

        RTSurf.Bounds(out double U1, out double U2, out double V1, out double V2);

        // -----------------------------------------
        // Modification of the Trimming Parameters ?
        // -----------------------------------------

        // OCCT: degrees whatever the plane angle factor, which StepToGeom reads them with (occt-issues.md, 8D)
        double AngleFact = 180.0 / M_PI;
        double uFact = 1.0;
        double vFact = 1.0;
        double LengthFact = theLocalFactors.LengthFactor();
        Surface theSurf = RTSurf.BasisSurface();
        if (theSurf is CylindricalSurface)
        {
            uFact = AngleFact;
            vFact = 1.0 / LengthFact;
        }
        else if (theSurf is RevolutionSurface)
        {
            uFact = AngleFact;
        }
        else if (theSurf is ToroidalSurface || theSurf is SphericalSurface)
        {
            uFact = AngleFact;
            vFact = AngleFact;
        }
        else if (theSurf is ConicalSurface)
        {
            ConicalSurface conicS = theSurf as ConicalSurface;
            double semAng = conicS.SemiAngle();
            uFact = AngleFact;
            vFact = Math.Cos(semAng) / LengthFact;
        }
        else if (theSurf is PlaneSurface)
        {
            uFact = vFact = 1.0 / LengthFact;
        }

        U1 = U1 * uFact;
        U2 = U2 * uFact;
        V1 = V1 * vFact;
        V2 = V2 * vFact;

        StepRTS.Init(aName, StepSurf, U1, U2, V1, V2, true, true);
        theRectangularTrimmedSurface = StepRTS;
        done = true;
    }

    /// <summary>A rectangular trimmed surface (default factors).</summary>
    public GeomToStep_MakeRectangularTrimmedSurface(TrimmedSurface RTSurf) : this(RTSurf, new StepData_Factors())
    {
    }

    /// <summary>The rectangular_trimmed_surface made.</summary>
    public StepGeom_RectangularTrimmedSurface Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeRectangularTrimmedSurface::Value() - no result");
        return theRectangularTrimmedSurface;
    }
}
