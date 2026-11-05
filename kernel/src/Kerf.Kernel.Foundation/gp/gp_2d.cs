// Port of: src/FoundationClasses/TKMath/gp/gp.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry;

// The 2D constants of the gp package; the 3D ones are in gp.cs.
internal static partial class Constants
{
    private static readonly Point2D OriginPoint2d = new(0, 0);
    private static readonly Direction2D Dx2d = new(Direction2D.D.X);
    private static readonly Direction2D Dy2d = new(Direction2D.D.Y);
    private static readonly Axis2D Ox2d = new(new Point2D(0, 0), Direction2D.D.X);
    private static readonly Axis2D Oy2d = new(new Point2D(0, 0), Direction2D.D.Y);

    public static ref readonly Point2D Origin2d() => ref OriginPoint2d;

    public static ref readonly Direction2D DX2d() => ref Dx2d;

    public static ref readonly Direction2D DY2d() => ref Dy2d;

    /// <summary>The X axis of the global 2D system: origin <see cref="Origin2d"/>, direction <see cref="DX2d"/>.</summary>
    public static ref readonly Axis2D OX2d() => ref Ox2d;

    /// <summary>The Y axis of the global 2D system: origin <see cref="Origin2d"/>, direction <see cref="DY2d"/>.</summary>
    public static ref readonly Axis2D OY2d() => ref Oy2d;
}
