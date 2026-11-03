// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeMirror.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeMirror.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A symmetry in 3D space about a point, an axis or a plane, as a gp_Trsf.</summary>
public sealed class gce_MakeMirror
{
    private Transform3D TheMirror = new();

    /// <summary>A central symmetry about a point.</summary>
    public gce_MakeMirror(in Point3D Point) => TheMirror.SetMirror(Point);

    /// <summary>An axial symmetry about an axis.</summary>
    public gce_MakeMirror(in Axis3D Axis) => TheMirror.SetMirror(Axis);

    /// <summary>An axial symmetry about a line.</summary>
    public gce_MakeMirror(in Line3D Line) => TheMirror.SetMirror(new Axis3D(Line.Location(), Line.Direction()));

    /// <summary>An axial symmetry about the axis through a point with a direction.</summary>
    public gce_MakeMirror(in Point3D Point, in Direction3D Direc) => TheMirror.SetMirror(new Axis3D(Point, Direc));

    /// <summary>A planar symmetry about a plane.</summary>
    public gce_MakeMirror(in Frame3D Plane) => TheMirror.SetMirror(Plane);

    /// <summary>A planar symmetry about a plane.</summary>
    public gce_MakeMirror(in Plane3D Plane) => TheMirror.SetMirror(Plane.Position().Ax2());

    /// <summary>The constructed transformation.</summary>
    public Transform3D Value() => TheMirror;

    /// <summary>Alias for Value().</summary>
    public Transform3D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform3D(gce_MakeMirror theMaker) => theMaker.Operator();
}
