// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeMirror.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeMirror.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Curves;

/// <summary>The symmetry with respect to a point, an axis or a plane, as a Geom_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
public sealed class MakeMirror
{
    private readonly GeometryTransformation _theMirror;

    /// <summary>The symmetry with respect to Point.</summary>
    public MakeMirror(in Point3D point)
    {
        _theMirror = new GeometryTransformation();
        _theMirror.SetMirror(point);
    }

    /// <summary>The symmetry with respect to the axis Axis.</summary>
    public MakeMirror(in Axis3D axis)
    {
        _theMirror = new GeometryTransformation();
        _theMirror.SetMirror(axis);
    }

    /// <summary>The symmetry with respect to the line Line.</summary>
    public MakeMirror(in Line3D line)
    {
        _theMirror = new GeometryTransformation();
        _theMirror.SetMirror(new Axis3D(line.Location, line.Direction));
    }

    /// <summary>The symmetry with respect to the axis through Point with direction Direc.</summary>
    public MakeMirror(in Point3D point, in Direction3D direc)
    {
        _theMirror = new GeometryTransformation();
        _theMirror.SetMirror(new Axis3D(point, direc));
    }

    /// <summary>The symmetry with respect to the plane (Location, XDirection, YDirection) of Plane.</summary>
    public MakeMirror(in Frame3D plane)
    {
        _theMirror = new GeometryTransformation();
        _theMirror.SetMirror(plane);
    }

    /// <summary>The symmetry with respect to the plane Plane.</summary>
    public MakeMirror(in Plane3D plane)
    {
        _theMirror = new GeometryTransformation();
        _theMirror.SetMirror(plane.Position.Frame);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public GeometryTransformation Value()
    {
        _theMirror.Freeze();
        return _theMirror;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator GeometryTransformation(MakeMirror maker) => maker.Value();
}
