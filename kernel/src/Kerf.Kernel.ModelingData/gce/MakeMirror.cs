// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeMirror.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeMirror.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Values;

/// <summary>A symmetry in 3D space about a point, an axis or a plane, as a gp_Trsf.</summary>
internal sealed class MakeMirror
{
    private Transform3D _theMirror = new();

    /// <summary>A central symmetry about a point.</summary>
    public MakeMirror(in Point3D point) => _theMirror = _theMirror.WithMirror(point);

    /// <summary>An axial symmetry about an axis.</summary>
    public MakeMirror(in Axis3D axis) => _theMirror = _theMirror.WithMirror(axis);

    /// <summary>An axial symmetry about a line.</summary>
    public MakeMirror(in Line3D line) => _theMirror = _theMirror.WithMirror(new Axis3D(line.Location, line.Direction));

    /// <summary>An axial symmetry about the axis through a point with a direction.</summary>
    public MakeMirror(in Point3D point, in Direction3D direc) => _theMirror = _theMirror.WithMirror(new Axis3D(point, direc));

    /// <summary>A planar symmetry about a plane.</summary>
    public MakeMirror(in Frame3D plane) => _theMirror = _theMirror.WithMirror(plane);

    /// <summary>A planar symmetry about a plane.</summary>
    public MakeMirror(in Plane3D plane) => _theMirror = _theMirror.WithMirror(plane.Position.Frame);

    /// <summary>The constructed transformation.</summary>
    public Transform3D Value() => _theMirror;

    /// <summary>Alias for Value().</summary>
    public Transform3D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform3D(MakeMirror maker) => maker.Operator();
}
