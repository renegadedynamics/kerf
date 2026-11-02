// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeHyperbola.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeHyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of Hyperbola from Geom and Geom2d to a STEP hyperbola.</summary>
public sealed class GeomToStep_MakeHyperbola : GeomToStep_Root
{
    private StepGeom_Hyperbola theHyperbola;

    //=============================================================================
    // Creation d'une hyperbola de prostep a partir d'une hyperbola de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D hyperbola.</summary>
    public GeomToStep_MakeHyperbola(Geom2d_Hyperbola C, in StepData_Factors theLocalFactors)
    {
        gp_Hypr2d gpHyp;
        gpHyp = C.Hypr2d();

        StepGeom_Hyperbola HStep = new StepGeom_Hyperbola();
        StepGeom_Axis2Placement Ax2 = new StepGeom_Axis2Placement();
        StepGeom_Axis2Placement2d Ax2Step;
        double majorR, minorR;

        GeomToStep_MakeAxis2Placement2d MkAxis2 = new GeomToStep_MakeAxis2Placement2d(gpHyp.Axis(), theLocalFactors);
        Ax2Step = MkAxis2.Value();
        majorR = gpHyp.MajorRadius();
        minorR = gpHyp.MinorRadius();
        Ax2.SetValue(Ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        HStep.Init(name, Ax2, majorR, minorR);
        theHyperbola = HStep;
        done = true;
    }

    /// <summary>A 2D hyperbola.</summary>
    public GeomToStep_MakeHyperbola(Geom2d_Hyperbola C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d'une hyperbola de prostep a partir d'une hyperbola de
    // Geom
    //=============================================================================

    /// <summary>A 3D hyperbola, its position and radii divided by the length factor.</summary>
    public GeomToStep_MakeHyperbola(Geom_Hyperbola C, in StepData_Factors theLocalFactors)
    {
        gp_Hypr gpHyp;
        gpHyp = C.Hypr();

        StepGeom_Hyperbola HStep = new StepGeom_Hyperbola();
        StepGeom_Axis2Placement Ax2 = new StepGeom_Axis2Placement();
        StepGeom_Axis2Placement3d Ax2Step;
        double majorR, minorR;

        GeomToStep_MakeAxis2Placement3d MkAxis2 = new GeomToStep_MakeAxis2Placement3d(gpHyp.Position(), theLocalFactors);
        Ax2Step = MkAxis2.Value();
        majorR = gpHyp.MajorRadius();
        minorR = gpHyp.MinorRadius();
        Ax2.SetValue(Ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        double fact = theLocalFactors.LengthFactor();
        HStep.Init(name, Ax2, majorR / fact, minorR / fact);
        theHyperbola = HStep;
        done = true;
    }

    /// <summary>A 3D hyperbola (default factors).</summary>
    public GeomToStep_MakeHyperbola(Geom_Hyperbola C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // return the result
    //=============================================================================

    /// <summary>The hyperbola made.</summary>
    public StepGeom_Hyperbola Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeHyperbola::Value() - no result");
        return theHyperbola;
    }
}
