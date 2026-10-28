// Port of: src/FoundationClasses/TKMath/gp/gp.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// The 2D constants of the gp package; the 3D ones are in gp.cs.
public static partial class gp
{
    private static readonly gp_Pnt2d s_Origin2d = new(0, 0);
    private static readonly gp_Dir2d s_DX2d = new(gp_Dir2d.D.X);
    private static readonly gp_Dir2d s_DY2d = new(gp_Dir2d.D.Y);
    private static readonly gp_Ax2d s_OX2d = new(new gp_Pnt2d(0, 0), gp_Dir2d.D.X);
    private static readonly gp_Ax2d s_OY2d = new(new gp_Pnt2d(0, 0), gp_Dir2d.D.Y);

    public static ref readonly gp_Pnt2d Origin2d() => ref s_Origin2d;

    public static ref readonly gp_Dir2d DX2d() => ref s_DX2d;

    public static ref readonly gp_Dir2d DY2d() => ref s_DY2d;

    /// <summary>The X axis of the global 2D system: origin <see cref="Origin2d"/>, direction <see cref="DX2d"/>.</summary>
    public static ref readonly gp_Ax2d OX2d() => ref s_OX2d;

    /// <summary>The Y axis of the global 2D system: origin <see cref="Origin2d"/>, direction <see cref="DY2d"/>.</summary>
    public static ref readonly gp_Ax2d OY2d() => ref s_OY2d;
}
