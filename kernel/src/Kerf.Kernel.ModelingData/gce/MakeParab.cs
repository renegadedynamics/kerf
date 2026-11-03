// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Parab: from an axis system and a focal length, or from a directrix and a focus. The vertex
/// is the location of the axis system, its X axis the symmetry axis (towards the focus); the equation in it is
/// Y**2 = (2*P) * X, P being the distance between focus and directrix and the focal length P / 2.
/// </summary>
public sealed class MakeParab : Root
{
    private Parabola3D _theParab = new();

    /// <summary>A parabola from its axis system and focal length; fails with gce_NullFocusLength if Focal &lt; 0.</summary>
    public MakeParab(in Frame3D a2, double focal)
    {
        if (focal < 0.0)
        {
            _theError = NullFocusLength;
        }
        else
        {
            _theParab = new Parabola3D(a2, focal);
            _theError = Done;
        }
    }

    /// <summary>A parabola from its directrix D and focus F.</summary>
    public MakeParab(in Axis3D d, in Point3D f)
    {
        _theParab = new Parabola3D(d, f);
        _theError = Done;
    }

    /// <summary>The constructed parabola.</summary>
    /// <exception cref="NotDoneException">No parabola was constructed.</exception>
    public Parabola3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeParab::Value() - no result");
        return _theParab;
    }

    /// <summary>Alias for Value().</summary>
    public Parabola3D Operator() => Value();

    /// <summary>The constructed parabola (OCCT's implicit conversion operator).</summary>
    public static explicit operator Parabola3D(MakeParab maker) => maker.Operator();
}
