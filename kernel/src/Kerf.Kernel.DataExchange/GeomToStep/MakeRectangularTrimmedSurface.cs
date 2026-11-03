// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeRectangularTrimmedSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeRectangularTrimmedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of RectangularTrimmedSurface from Geom to a STEP rectangular_trimmed_surface.</summary>
public sealed class MakeRectangularTrimmedSurface : GeometryRoot
{
    private StepRectangularTrimmedSurface _rectangularTrimmedSurface;

    //=============================================================================
    // Creation d' une rectangular_trimmed_surface de STEP
    // a partir d' une RectangularTrimmedSurface de Geom
    //=============================================================================

    /// <summary>
    /// A rectangular trimmed surface: its basis surface, and its bounds with angles in degrees (OCCT writes 180/π,
    /// not the plane angle factor) and lengths divided by the length factor. Not done if the basis isn't mapped.
    /// </summary>
    public MakeRectangularTrimmedSurface(TrimmedSurface rtSurf, in Factors localFactors)
    {
        StepRectangularTrimmedSurface stepRTS = new StepRectangularTrimmedSurface();

        TCollection_HAsciiString name = new TCollection_HAsciiString("");

        MakeSurface mkSurf = new MakeSurface(rtSurf.BasisSurface(), localFactors);
        if (!mkSurf.IsDone)
        {
            _done = false;
            return;
        }
        StepSurface stepSurf = mkSurf.Value();

        rtSurf.Bounds(out double u1, out double u2, out double v1, out double v2);

        // -----------------------------------------
        // Modification of the Trimming Parameters ?
        // -----------------------------------------

        // OCCT: degrees whatever the plane angle factor, which StepToGeom reads them with (occt-issues.md, 8D)
        double angleFact = 180.0 / M_PI;
        double uFact = 1.0;
        double vFact = 1.0;
        double lengthFact = localFactors.LengthFactor();
        Surface theSurf = rtSurf.BasisSurface();
        if (theSurf is CylindricalSurface)
        {
            uFact = angleFact;
            vFact = 1.0 / lengthFact;
        }
        else if (theSurf is RevolutionSurface)
        {
            uFact = angleFact;
        }
        else if (theSurf is ToroidalSurface || theSurf is SphericalSurface)
        {
            uFact = angleFact;
            vFact = angleFact;
        }
        else if (theSurf is ConicalSurface)
        {
            ConicalSurface conicS = theSurf as ConicalSurface;
            double semAng = conicS.SemiAngle();
            uFact = angleFact;
            vFact = Math.Cos(semAng) / lengthFact;
        }
        else if (theSurf is PlaneSurface)
        {
            uFact = vFact = 1.0 / lengthFact;
        }

        u1 = u1 * uFact;
        u2 = u2 * uFact;
        v1 = v1 * vFact;
        v2 = v2 * vFact;

        stepRTS.Init(name, stepSurf, u1, u2, v1, v2, true, true);
        _rectangularTrimmedSurface = stepRTS;
        _done = true;
    }

    /// <summary>A rectangular trimmed surface (default factors).</summary>
    public MakeRectangularTrimmedSurface(TrimmedSurface rtSurf) : this(rtSurf, new Factors())
    {
    }

    /// <summary>The rectangular_trimmed_surface made.</summary>
    public StepRectangularTrimmedSurface Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeRectangularTrimmedSurface::Value() - no result");
        return _rectangularTrimmedSurface;
    }
}
