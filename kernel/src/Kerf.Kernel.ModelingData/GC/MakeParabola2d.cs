// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeParabola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeParabola2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeParabola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom2d_Parabola: from a gp_Parab2d, a mirror axis (or axis system) and the focal length, a
/// directrix and a focus, or a focus and the apex.
/// </summary>
internal sealed class MakeParabola2d : Root
{
    private ParabolaCurve2D? _theParabola;

    /// <summary>A parabola from the gp_Parab2d Prb.</summary>
    public MakeParabola2d(in Parabola2D prb)
    {
        _theError = Done;
        _theParabola = new ParabolaCurve2D(prb);
    }

    /// <summary>A parabola of mirror axis MirrorAxis; fails with gce_NullFocusLength if Focal is negative.</summary>
    public MakeParabola2d(in Axis2D mirrorAxis, double focal, bool sense)
    {
        if (focal < 0.0)
        {
            _theError = NullFocusLength;
        }
        else
        {
            _theError = Done;
            _theParabola = new ParabolaCurve2D(mirrorAxis, focal, sense);
        }
    }

    /// <summary>A parabola in the axis system Axis; fails with gce_NullFocusLength if Focal is negative.</summary>
    public MakeParabola2d(in Frame2D axis, double focal)
    {
        if (focal < 0.0)
        {
            _theError = NullFocusLength;
        }
        else
        {
            _theError = Done;
            _theParabola = new ParabolaCurve2D(axis, focal);
        }
    }

    /// <summary>A parabola of directrix D and focus F.</summary>
    public MakeParabola2d(in Axis2D d, in Point2D f, bool sense = true)
    {
        _theError = Done;
        Parabola2D para = new Parabola2D(d, f, sense);
        _theParabola = new ParabolaCurve2D(para);
    }

    /// <summary>A parabola of focus S1 and apex O; fails with the status of gce_MakeParab2d.</summary>
    public MakeParabola2d(in Point2D s1, in Point2D o)
    {
        MakeParab2d p = new MakeParab2d(s1, o);
        _theError = p.Status();
        if (_theError == Done)
        {
            _theParabola = new ParabolaCurve2D(p.Value());
        }
    }

    /// <summary>The constructed parabola (frozen).</summary>
    /// <exception cref="NotDoneException">No parabola was constructed.</exception>
    public ParabolaCurve2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeParabola2d::Value() - no result");
        _theParabola!.Freeze();
        return _theParabola;
    }

    /// <summary>The constructed parabola (OCCT's implicit conversion).</summary>
    public static explicit operator ParabolaCurve2D(MakeParabola2d maker) => maker.Value();
}
