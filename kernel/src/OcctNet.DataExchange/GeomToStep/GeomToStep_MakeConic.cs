// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConic.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of Conic from Geom and Geom2d to a STEP conic (circle, ellipse, hyperbola, parabola).</summary>
public sealed class GeomToStep_MakeConic : GeomToStep_Root
{
    private StepGeom_Conic theConic;

    //=============================================================================
    // Creation d' une Conic de prostep a partir d' une Conic de Geom
    //=============================================================================

    /// <summary>A 3D conic.</summary>
    public GeomToStep_MakeConic(Geom_Conic C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is Geom_Circle)
        {
            Geom_Circle Cer = C as Geom_Circle;
            GeomToStep_MakeCircle MkCircle = new GeomToStep_MakeCircle(Cer, theLocalFactors);
            theConic = MkCircle.Value();
        }
        else if (C is Geom_Ellipse)
        {
            Geom_Ellipse Ell = C as Geom_Ellipse;
            GeomToStep_MakeEllipse MkEllipse = new GeomToStep_MakeEllipse(Ell, theLocalFactors);
            theConic = MkEllipse.Value();
        }
        else if (C is Geom_Hyperbola)
        {
            Geom_Hyperbola Hyp = C as Geom_Hyperbola;
            GeomToStep_MakeHyperbola MkHyperbola = new GeomToStep_MakeHyperbola(Hyp, theLocalFactors);
            theConic = MkHyperbola.Value();
        }
        else if (C is Geom_Parabola)
        {
            Geom_Parabola Par = C as Geom_Parabola;
            GeomToStep_MakeParabola MkParabola = new GeomToStep_MakeParabola(Par, theLocalFactors);
            theConic = MkParabola.Value();
        }
        else
        {
            // OCCT_DEBUG output left out
            done = false;
        }
    }

    /// <summary>A 3D conic (default factors).</summary>
    public GeomToStep_MakeConic(Geom_Conic C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une Conic2d de prostep a partir d' une Conic de Geom2d
    //=============================================================================

    /// <summary>A 2D conic.</summary>
    public GeomToStep_MakeConic(Geom2d_Conic C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is Geom2d_Circle)
        {
            Geom2d_Circle Cer = C as Geom2d_Circle;
            GeomToStep_MakeCircle MkCircle = new GeomToStep_MakeCircle(Cer, theLocalFactors);
            theConic = MkCircle.Value();
        }
        else if (C is Geom2d_Ellipse)
        {
            Geom2d_Ellipse Ell = C as Geom2d_Ellipse;
            GeomToStep_MakeEllipse MkEllipse = new GeomToStep_MakeEllipse(Ell, theLocalFactors);
            theConic = MkEllipse.Value();
        }
        else if (C is Geom2d_Hyperbola)
        {
            Geom2d_Hyperbola Hyp = C as Geom2d_Hyperbola;
            GeomToStep_MakeHyperbola MkHyperbola = new GeomToStep_MakeHyperbola(Hyp, theLocalFactors);
            theConic = MkHyperbola.Value();
        }
        else if (C is Geom2d_Parabola)
        {
            Geom2d_Parabola Par = C as Geom2d_Parabola;
            GeomToStep_MakeParabola MkParabola = new GeomToStep_MakeParabola(Par, theLocalFactors);
            theConic = MkParabola.Value();
        }
        else
        {
            // Attention : Other 2d conics shall be implemented ...
            //             To be performed later !
            // OCCT_DEBUG output left out
            done = false;
        }
    }

    /// <summary>A 2D conic.</summary>
    public GeomToStep_MakeConic(Geom2d_Conic C) : this(C, new StepData_Factors())
    {
    }

    /// <summary>The conic made.</summary>
    public StepGeom_Conic Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeConic::Value() - no result");
        return theConic;
    }
}
