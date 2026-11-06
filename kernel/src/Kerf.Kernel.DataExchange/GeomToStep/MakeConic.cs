// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConic.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of Conic from Geom and Geom2d to a STEP conic (circle, ellipse, hyperbola, parabola).</summary>
internal sealed class MakeConic : GeometryRoot
{
    private StepConic _conic;

    //=============================================================================
    // Creation d' une Conic de prostep a partir d' une Conic de Geom
    //=============================================================================

    /// <summary>A 3D conic.</summary>
    public MakeConic(ConicCurve3D c, in Factors localFactors)
    {
        _done = true;
        if (c is CircleCurve3D)
        {
            CircleCurve3D cer = c as CircleCurve3D;
            GeometryMakeCircle mkCircle = new GeometryMakeCircle(cer, localFactors);
            _conic = mkCircle.Value();
        }
        else if (c is EllipseCurve3D)
        {
            EllipseCurve3D ell = c as EllipseCurve3D;
            GeometryMakeEllipse mkEllipse = new GeometryMakeEllipse(ell, localFactors);
            _conic = mkEllipse.Value();
        }
        else if (c is HyperbolaCurve3D)
        {
            HyperbolaCurve3D hyp = c as HyperbolaCurve3D;
            GeometryMakeHyperbola mkHyperbola = new GeometryMakeHyperbola(hyp, localFactors);
            _conic = mkHyperbola.Value();
        }
        else if (c is ParabolaCurve3D)
        {
            ParabolaCurve3D par = c as ParabolaCurve3D;
            MakeParabola mkParabola = new MakeParabola(par, localFactors);
            _conic = mkParabola.Value();
        }
        else
        {
            // OCCT_DEBUG output left out
            _done = false;
        }
    }

    /// <summary>A 3D conic (default factors).</summary>
    public MakeConic(ConicCurve3D c) : this(c, new Factors())
    {
    }

    //=============================================================================
    // Creation d' une Conic2d de prostep a partir d' une Conic de Geom2d
    //=============================================================================

    /// <summary>A 2D conic.</summary>
    public MakeConic(ConicCurve2D c, in Factors localFactors)
    {
        _done = true;
        if (c is CircleCurve2D)
        {
            CircleCurve2D cer = c as CircleCurve2D;
            GeometryMakeCircle mkCircle = new GeometryMakeCircle(cer, localFactors);
            _conic = mkCircle.Value();
        }
        else if (c is EllipseCurve2D)
        {
            EllipseCurve2D ell = c as EllipseCurve2D;
            GeometryMakeEllipse mkEllipse = new GeometryMakeEllipse(ell, localFactors);
            _conic = mkEllipse.Value();
        }
        else if (c is HyperbolaCurve2D)
        {
            HyperbolaCurve2D hyp = c as HyperbolaCurve2D;
            GeometryMakeHyperbola mkHyperbola = new GeometryMakeHyperbola(hyp, localFactors);
            _conic = mkHyperbola.Value();
        }
        else if (c is ParabolaCurve2D)
        {
            ParabolaCurve2D par = c as ParabolaCurve2D;
            MakeParabola mkParabola = new MakeParabola(par, localFactors);
            _conic = mkParabola.Value();
        }
        else
        {
            // Attention : Other 2d conics shall be implemented ...
            //             To be performed later !
            // OCCT_DEBUG output left out
            _done = false;
        }
    }

    /// <summary>A 2D conic.</summary>
    public MakeConic(ConicCurve2D c) : this(c, new Factors())
    {
    }

    /// <summary>The conic made.</summary>
    public StepConic Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeConic::Value() - no result");
        return _conic;
    }
}
