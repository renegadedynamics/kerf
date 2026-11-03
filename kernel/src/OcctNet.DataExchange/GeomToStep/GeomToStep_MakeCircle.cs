// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCircle.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCircle.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCircle_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Geometry;

namespace Occt;

/// <summary>The mapping of Circle from Geom and Geom2d (and Circ from gp) to a STEP circle.</summary>
public sealed class GeomToStep_MakeCircle : GeomToStep_Root
{
    private StepGeom_Circle theCircle;

    //=============================================================================
    // Creation d' un cercle de prostep a partir d' un cercle 3d de gp
    //=============================================================================

    /// <summary>A 3D circle, its position and radius divided by the length factor.</summary>
    public GeomToStep_MakeCircle(in Circle3D C, in StepData_Factors theLocalFactors)
    {
        Make(C, theLocalFactors);
    }

    /// <summary>A 3D circle (default factors).</summary>
    public GeomToStep_MakeCircle(in Circle3D C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un cercle de prostep a partir d' un cercle de
    // Geom
    //=============================================================================

    /// <summary>A 3D circle, its position and radius divided by the length factor.</summary>
    public GeomToStep_MakeCircle(Geom_Circle Cer, in StepData_Factors theLocalFactors)
    {
        Circle3D C;
        C = Cer.Circ();
        Make(C, theLocalFactors);
    }

    /// <summary>A 3D circle (default factors).</summary>
    public GeomToStep_MakeCircle(Geom_Circle Cer) : this(Cer, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un cercle 2d de prostep a partir d' un cercle de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D circle.</summary>
    public GeomToStep_MakeCircle(Geom2d_Circle Cer, in StepData_Factors theLocalFactors)
    {
        Circle2D C2d;
        C2d = Cer.Circ2d();

        StepGeom_Circle CStep = new StepGeom_Circle();
        StepGeom_Axis2Placement Ax2 = new StepGeom_Axis2Placement();
        StepGeom_Axis2Placement2d Ax2Step;
        double Rayon;

        GeomToStep_MakeAxis2Placement2d MkAxis2 = new GeomToStep_MakeAxis2Placement2d(C2d.Position(), theLocalFactors);
        Ax2Step = MkAxis2.Value();
        Rayon = C2d.Radius();
        Ax2.SetValue(Ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        CStep.Init(name, Ax2, Rayon);
        theCircle = CStep;
        done = true;
    }

    /// <summary>A 2D circle.</summary>
    public GeomToStep_MakeCircle(Geom2d_Circle Cer) : this(Cer, new StepData_Factors())
    {
    }

    // GeomToStep_MakeCircle_gen.pxx, the body OCCT includes in the 3D constructors
    private void Make(in Circle3D C, in StepData_Factors theLocalFactors)
    {
        StepGeom_Circle CStep = new StepGeom_Circle();
        StepGeom_Axis2Placement Ax2 = new StepGeom_Axis2Placement();
        StepGeom_Axis2Placement3d Ax2Step;
        double Rayon;

        GeomToStep_MakeAxis2Placement3d MkAxis2 = new GeomToStep_MakeAxis2Placement3d(C.Position(), theLocalFactors);
        Ax2Step = MkAxis2.Value();
        Rayon = C.Radius();
        Ax2.SetValue(Ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        CStep.Init(name, Ax2, Rayon / theLocalFactors.LengthFactor());
        theCircle = CStep;
        done = true;
    }

    /// <summary>The circle made.</summary>
    public StepGeom_Circle Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeCircle::Value() - no result");
        return theCircle;
    }
}
