// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeEllipse.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeEllipse.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeEllipse_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of Ellipse from Geom and Geom2d (and Elips from gp) to a STEP ellipse.</summary>
public sealed class GeomToStep_MakeEllipse : GeomToStep_Root
{
    private StepGeom_Ellipse theEllipse;

    //=============================================================================
    // Creation d'une ellipse de prostep a partir d'une ellipse 3d de gp
    //=============================================================================

    /// <summary>A 3D ellipse, its position and radii divided by the length factor.</summary>
    public GeomToStep_MakeEllipse(in gp_Elips E, in StepData_Factors theLocalFactors)
    {
        Make(E, theLocalFactors);
    }

    /// <summary>A 3D ellipse (default factors).</summary>
    public GeomToStep_MakeEllipse(in gp_Elips E) : this(E, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d'une ellipse de prostep a partir d'une ellipse de
    // Geom
    //=============================================================================

    /// <summary>A 3D ellipse, its position and radii divided by the length factor.</summary>
    public GeomToStep_MakeEllipse(Geom_Ellipse Cer, in StepData_Factors theLocalFactors)
    {
        gp_Elips E;
        E = Cer.Elips();
        Make(E, theLocalFactors);
    }

    /// <summary>A 3D ellipse (default factors).</summary>
    public GeomToStep_MakeEllipse(Geom_Ellipse Cer) : this(Cer, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d'une ellipse 2d de prostep a partir d'une ellipse de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D ellipse.</summary>
    public GeomToStep_MakeEllipse(Geom2d_Ellipse Cer, in StepData_Factors theLocalFactors)
    {
        gp_Elips2d E2d;
        E2d = Cer.Elips2d();

        StepGeom_Ellipse EStep = new StepGeom_Ellipse();
        StepGeom_Axis2Placement Ax2 = new StepGeom_Axis2Placement();
        StepGeom_Axis2Placement2d Ax2Step;
        double majorR, minorR;

        GeomToStep_MakeAxis2Placement2d MkAxis2 = new GeomToStep_MakeAxis2Placement2d(E2d.Axis(), theLocalFactors);
        Ax2Step = MkAxis2.Value();
        majorR = E2d.MajorRadius();
        minorR = E2d.MinorRadius();
        Ax2.SetValue(Ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        EStep.Init(name, Ax2, majorR, minorR);
        theEllipse = EStep;
        done = true;
    }

    /// <summary>A 2D ellipse.</summary>
    public GeomToStep_MakeEllipse(Geom2d_Ellipse Cer) : this(Cer, new StepData_Factors())
    {
    }

    // GeomToStep_MakeEllipse_gen.pxx, the body OCCT includes in the 3D constructors
    private void Make(in gp_Elips E, in StepData_Factors theLocalFactors)
    {
        StepGeom_Ellipse EStep = new StepGeom_Ellipse();
        StepGeom_Axis2Placement Ax2 = new StepGeom_Axis2Placement();
        StepGeom_Axis2Placement3d Ax2Step;
        double majorR, minorR;

        GeomToStep_MakeAxis2Placement3d MkAxis2 = new GeomToStep_MakeAxis2Placement3d(E.Position(), theLocalFactors);
        Ax2Step = MkAxis2.Value();
        majorR = E.MajorRadius();
        minorR = E.MinorRadius();
        Ax2.SetValue(Ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        double fact = theLocalFactors.LengthFactor();
        EStep.Init(name, Ax2, majorR / fact, minorR / fact);
        theEllipse = EStep;
        done = true;
    }

    /// <summary>The ellipse made.</summary>
    public StepGeom_Ellipse Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeEllipse::Value() - no result");
        return theEllipse;
    }
}
