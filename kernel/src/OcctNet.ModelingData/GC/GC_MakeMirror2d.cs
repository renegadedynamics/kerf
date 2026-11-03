// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeMirror2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeMirror2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeMirror.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The symmetry with respect to a point or an axis in 2D, as a Geom2d_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
public sealed class GC_MakeMirror2d
{
    private readonly Geom2d_Transformation TheMirror;

    /// <summary>The symmetry with respect to Point.</summary>
    public GC_MakeMirror2d(in Point2D Point)
    {
        TheMirror = new Geom2d_Transformation();
        TheMirror.SetMirror(Point);
    }

    /// <summary>The symmetry with respect to the axis Axis.</summary>
    public GC_MakeMirror2d(in Axis2D Axis)
    {
        TheMirror = new Geom2d_Transformation();
        TheMirror.SetMirror(Axis);
    }

    /// <summary>The symmetry with respect to the line Line.</summary>
    public GC_MakeMirror2d(in Line2D Line)
    {
        TheMirror = new Geom2d_Transformation();
        TheMirror.SetMirror(new Axis2D(Line.Location(), Line.Direction()));
    }

    /// <summary>The symmetry with respect to the axis through Point with direction Direc.</summary>
    public GC_MakeMirror2d(in Point2D Point, in Direction2D Direc)
    {
        TheMirror = new Geom2d_Transformation();
        TheMirror.SetMirror(new Axis2D(Point, Direc));
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Geom2d_Transformation Value()
    {
        TheMirror.Freeze();
        return TheMirror;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_Transformation(GC_MakeMirror2d theMaker) => theMaker.Value();
}
