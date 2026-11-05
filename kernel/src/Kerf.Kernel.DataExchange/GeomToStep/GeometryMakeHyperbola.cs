// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeHyperbola.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeHyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of Hyperbola from Geom and Geom2d to a STEP hyperbola.</summary>
internal sealed class GeometryMakeHyperbola : GeometryRoot
{
    private StepHyperbola _hyperbola;

    //=============================================================================
    // Creation d'une hyperbola de prostep a partir d'une hyperbola de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D hyperbola.</summary>
    public GeometryMakeHyperbola(HyperbolaCurve2D c, in Factors localFactors)
    {
        Hyperbola2D gpHyp;
        gpHyp = c.Hyperbola;

        StepHyperbola hStep = new StepHyperbola();
        StepAxis2Placement ax2 = new StepAxis2Placement();
        StepAxis2Placement2d ax2Step;
        double majorR, minorR;

        MakeAxis2Placement2d mkAxis2 = new MakeAxis2Placement2d(gpHyp.Axis, localFactors);
        ax2Step = mkAxis2.Value();
        majorR = gpHyp.MajorRadius;
        minorR = gpHyp.MinorRadius;
        ax2.SetValue(ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        hStep.Init(name, ax2, majorR, minorR);
        _hyperbola = hStep;
        _done = true;
    }

    /// <summary>A 2D hyperbola.</summary>
    public GeometryMakeHyperbola(HyperbolaCurve2D c) : this(c, new Factors())
    {
    }

    //=============================================================================
    // Creation d'une hyperbola de prostep a partir d'une hyperbola de
    // Geom
    //=============================================================================

    /// <summary>A 3D hyperbola, its position and radii divided by the length factor.</summary>
    public GeometryMakeHyperbola(HyperbolaCurve3D c, in Factors localFactors)
    {
        Hyperbola3D gpHyp;
        gpHyp = c.Hyperbola;

        StepHyperbola hStep = new StepHyperbola();
        StepAxis2Placement ax2 = new StepAxis2Placement();
        StepAxis2Placement3d ax2Step;
        double majorR, minorR;

        Axis2Placement3dMaker mkAxis2 = new Axis2Placement3dMaker(gpHyp.Position, localFactors);
        ax2Step = mkAxis2.Value();
        majorR = gpHyp.MajorRadius;
        minorR = gpHyp.MinorRadius;
        ax2.SetValue(ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        double fact = localFactors.LengthFactor();
        hStep.Init(name, ax2, majorR / fact, minorR / fact);
        _hyperbola = hStep;
        _done = true;
    }

    /// <summary>A 3D hyperbola (default factors).</summary>
    public GeometryMakeHyperbola(HyperbolaCurve3D c) : this(c, new Factors())
    {
    }

    //=============================================================================
    // return the result
    //=============================================================================

    /// <summary>The hyperbola made.</summary>
    public StepHyperbola Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeHyperbola::Value() - no result");
        return _hyperbola;
    }
}
