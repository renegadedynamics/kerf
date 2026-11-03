// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeMirror2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeMirror2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeMirror.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Curves;

/// <summary>The symmetry with respect to a point or an axis in 2D, as a Geom2d_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
public sealed class MakeMirror2d
{
    private readonly Transformation2 _theMirror;

    /// <summary>The symmetry with respect to Point.</summary>
    public MakeMirror2d(in Point2D point)
    {
        _theMirror = new Transformation2();
        _theMirror.SetMirror(point);
    }

    /// <summary>The symmetry with respect to the axis Axis.</summary>
    public MakeMirror2d(in Axis2D axis)
    {
        _theMirror = new Transformation2();
        _theMirror.SetMirror(axis);
    }

    /// <summary>The symmetry with respect to the line Line.</summary>
    public MakeMirror2d(in Line2D line)
    {
        _theMirror = new Transformation2();
        _theMirror.SetMirror(new Axis2D(line.Location(), line.Direction()));
    }

    /// <summary>The symmetry with respect to the axis through Point with direction Direc.</summary>
    public MakeMirror2d(in Point2D point, in Direction2D direc)
    {
        _theMirror = new Transformation2();
        _theMirror.SetMirror(new Axis2D(point, direc));
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Transformation2 Value()
    {
        _theMirror.Freeze();
        return _theMirror;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Transformation2(MakeMirror2d maker) => maker.Value();
}
