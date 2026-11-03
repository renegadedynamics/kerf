// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConic.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Geometry;

namespace Occt;

/// <summary>The mapping of Conic from Geom and Geom2d to a STEP conic (circle, ellipse, hyperbola, parabola).</summary>
public sealed class GeomToStep_MakeConic : GeomToStep_Root
{
    private StepGeom_Conic theConic;

    //=============================================================================
    // Creation d' une Conic de prostep a partir d' une Conic de Geom
    //=============================================================================

    /// <summary>A 3D conic.</summary>
    public GeomToStep_MakeConic(ConicCurve3D C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is CircleCurve3D)
        {
            CircleCurve3D Cer = C as CircleCurve3D;
            GeomToStep_MakeCircle MkCircle = new GeomToStep_MakeCircle(Cer, theLocalFactors);
            theConic = MkCircle.Value();
        }
        else if (C is EllipseCurve3D)
        {
            EllipseCurve3D Ell = C as EllipseCurve3D;
            GeomToStep_MakeEllipse MkEllipse = new GeomToStep_MakeEllipse(Ell, theLocalFactors);
            theConic = MkEllipse.Value();
        }
        else if (C is HyperbolaCurve3D)
        {
            HyperbolaCurve3D Hyp = C as HyperbolaCurve3D;
            GeomToStep_MakeHyperbola MkHyperbola = new GeomToStep_MakeHyperbola(Hyp, theLocalFactors);
            theConic = MkHyperbola.Value();
        }
        else if (C is ParabolaCurve3D)
        {
            ParabolaCurve3D Par = C as ParabolaCurve3D;
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
    public GeomToStep_MakeConic(ConicCurve3D C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une Conic2d de prostep a partir d' une Conic de Geom2d
    //=============================================================================

    /// <summary>A 2D conic.</summary>
    public GeomToStep_MakeConic(ConicCurve2D C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is CircleCurve2D)
        {
            CircleCurve2D Cer = C as CircleCurve2D;
            GeomToStep_MakeCircle MkCircle = new GeomToStep_MakeCircle(Cer, theLocalFactors);
            theConic = MkCircle.Value();
        }
        else if (C is EllipseCurve2D)
        {
            EllipseCurve2D Ell = C as EllipseCurve2D;
            GeomToStep_MakeEllipse MkEllipse = new GeomToStep_MakeEllipse(Ell, theLocalFactors);
            theConic = MkEllipse.Value();
        }
        else if (C is HyperbolaCurve2D)
        {
            HyperbolaCurve2D Hyp = C as HyperbolaCurve2D;
            GeomToStep_MakeHyperbola MkHyperbola = new GeomToStep_MakeHyperbola(Hyp, theLocalFactors);
            theConic = MkHyperbola.Value();
        }
        else if (C is ParabolaCurve2D)
        {
            ParabolaCurve2D Par = C as ParabolaCurve2D;
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
    public GeomToStep_MakeConic(ConicCurve2D C) : this(C, new StepData_Factors())
    {
    }

    /// <summary>The conic made.</summary>
    public StepGeom_Conic Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeConic::Value() - no result");
        return theConic;
    }
}
