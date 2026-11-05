// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Parab2d: from a symmetry axis or an axis system and a focal length, from a directrix and a
/// focus, or from a focus and a vertex.
/// </summary>
internal sealed class MakeParab2d : Root
{
    private Parabola2D _theParab2d = new();

    /// <summary>
    /// A parabola from its axis system and focal length (0 is accepted); fails with gce_NullFocusLength if Focal &lt; 0.
    /// </summary>
    public MakeParab2d(in Frame2D a, double focal)
    {
        if (focal < 0.0)
        {
            _theError = NullFocusLength;
        }
        else
        {
            _theParab2d = new Parabola2D(a, focal);
            _theError = Done;
        }
    }

    /// <summary>
    /// A parabola from its symmetry axis, focal length (0 is accepted) and sense of parametrization; fails with
    /// gce_NullFocusLength if Focal &lt; 0.
    /// </summary>
    public MakeParab2d(in Axis2D mirrorAxis, double focal, bool sense = true)
    {
        if (focal < 0.0)
        {
            _theError = NullFocusLength;
        }
        else
        {
            _theParab2d = new Parabola2D(mirrorAxis, focal, sense);
            _theError = Done;
        }
    }

    /// <summary>A parabola from its directrix D and focus F.</summary>
    public MakeParab2d(in Axis2D d, in Point2D f, bool sense = true)
    {
        _theParab2d = new Parabola2D(d, f, sense);
        _theError = Done;
    }

    /// <summary>
    /// A parabola from its focus S and vertex Center; fails with gce_NullAxis if they coincide.
    /// </summary>
    public MakeParab2d(in Point2D s, in Point2D center, bool sense = true)
    {
        if (s.DistanceTo(center) >= Constants.Resolution())
        {
            Direction2D xAxis = new(s.XY() - center.XY());
            _theParab2d = new Parabola2D(new Axis2D(center, xAxis), s.DistanceTo(center), sense);
            _theError = Done;
        }
        else
        {
            _theError = NullAxis;
        }
    }

    /// <summary>The constructed parabola.</summary>
    /// <exception cref="NotDoneException">No parabola was constructed.</exception>
    public Parabola2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeParab2d::Value() - no result");
        return _theParab2d;
    }

    /// <summary>Alias for Value().</summary>
    public Parabola2D Operator() => Value();

    /// <summary>The constructed parabola (OCCT's implicit conversion operator).</summary>
    public static explicit operator Parabola2D(MakeParab2d maker) => maker.Operator();
}
