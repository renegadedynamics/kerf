// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeMirror2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeMirror2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A symmetry in 2D space about a point or an axis, as a gp_Trsf2d.</summary>
public sealed class gce_MakeMirror2d
{
    private gp_Trsf2d TheMirror2d = new();

    /// <summary>A central symmetry about a point.</summary>
    public gce_MakeMirror2d(in gp_Pnt2d Point) => TheMirror2d.SetMirror(Point);

    /// <summary>An axial symmetry about an axis.</summary>
    public gce_MakeMirror2d(in gp_Ax2d Axis) => TheMirror2d.SetMirror(Axis);

    /// <summary>An axial symmetry about a line.</summary>
    public gce_MakeMirror2d(in gp_Lin2d Line) => TheMirror2d.SetMirror(new gp_Ax2d(Line.Location(), Line.Direction()));

    /// <summary>An axial symmetry about the axis through a point with a direction.</summary>
    public gce_MakeMirror2d(in gp_Pnt2d Point, in gp_Dir2d Direc) => TheMirror2d.SetMirror(new gp_Ax2d(Point, Direc));

    /// <summary>The constructed transformation.</summary>
    public gp_Trsf2d Value() => TheMirror2d;

    /// <summary>Alias for Value().</summary>
    public gp_Trsf2d Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Trsf2d(gce_MakeMirror2d theMaker) => theMaker.Operator();
}
