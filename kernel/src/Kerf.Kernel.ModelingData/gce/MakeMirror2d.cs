// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeMirror2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeMirror2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Values;

/// <summary>A symmetry in 2D space about a point or an axis, as a gp_Trsf2d.</summary>
internal sealed class MakeMirror2d
{
    private Transform2D _theMirror2d = new();

    /// <summary>A central symmetry about a point.</summary>
    public MakeMirror2d(in Point2D point) => _theMirror2d = _theMirror2d.WithMirror(point);

    /// <summary>An axial symmetry about an axis.</summary>
    public MakeMirror2d(in Axis2D axis) => _theMirror2d = _theMirror2d.WithMirror(axis);

    /// <summary>An axial symmetry about a line.</summary>
    public MakeMirror2d(in Line2D line) => _theMirror2d = _theMirror2d.WithMirror(new Axis2D(line.Location, line.Direction));

    /// <summary>An axial symmetry about the axis through a point with a direction.</summary>
    public MakeMirror2d(in Point2D point, in Direction2D direc) => _theMirror2d = _theMirror2d.WithMirror(new Axis2D(point, direc));

    /// <summary>The constructed transformation.</summary>
    public Transform2D Value() => _theMirror2d;

    /// <summary>Alias for Value().</summary>
    public Transform2D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform2D(MakeMirror2d maker) => maker.Operator();
}
