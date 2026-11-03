// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeMirror.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeMirror.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The symmetry with respect to a point, an axis or a plane, as a Geom_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
public sealed class GC_MakeMirror
{
    private readonly Geom_Transformation TheMirror;

    /// <summary>The symmetry with respect to Point.</summary>
    public GC_MakeMirror(in Point3D Point)
    {
        TheMirror = new Geom_Transformation();
        TheMirror.SetMirror(Point);
    }

    /// <summary>The symmetry with respect to the axis Axis.</summary>
    public GC_MakeMirror(in Axis3D Axis)
    {
        TheMirror = new Geom_Transformation();
        TheMirror.SetMirror(Axis);
    }

    /// <summary>The symmetry with respect to the line Line.</summary>
    public GC_MakeMirror(in Line3D Line)
    {
        TheMirror = new Geom_Transformation();
        TheMirror.SetMirror(new Axis3D(Line.Location(), Line.Direction()));
    }

    /// <summary>The symmetry with respect to the axis through Point with direction Direc.</summary>
    public GC_MakeMirror(in Point3D Point, in Direction3D Direc)
    {
        TheMirror = new Geom_Transformation();
        TheMirror.SetMirror(new Axis3D(Point, Direc));
    }

    /// <summary>The symmetry with respect to the plane (Location, XDirection, YDirection) of Plane.</summary>
    public GC_MakeMirror(in Frame3D Plane)
    {
        TheMirror = new Geom_Transformation();
        TheMirror.SetMirror(Plane);
    }

    /// <summary>The symmetry with respect to the plane Plane.</summary>
    public GC_MakeMirror(in Plane3D Plane)
    {
        TheMirror = new Geom_Transformation();
        TheMirror.SetMirror(Plane.Position().Ax2());
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Geom_Transformation Value()
    {
        TheMirror.Freeze();
        return TheMirror;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_Transformation(GC_MakeMirror theMaker) => theMaker.Value();
}
