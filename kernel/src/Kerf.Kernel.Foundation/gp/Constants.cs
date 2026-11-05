// Port of: src/FoundationClasses/TKMath/gp/gp.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using System.Globalization;
using System.Text;

namespace Kerf.Engine.Geometry;

/// <summary>The constants of the gp package: the resolution, and the origin, axes and planes of the global system.</summary>
/// <remarks>The 2D constants are in gp_2d.cs.</remarks>
internal static partial class Constants
{
    private static readonly Point3D OriginPoint = new(0, 0, 0);
    private static readonly Direction3D Dx = new(Direction3D.D.X);
    private static readonly Direction3D Dy = new(Direction3D.D.Y);
    private static readonly Direction3D Dz = new(Direction3D.D.Z);
    private static readonly Axis3D Ox = new(new Point3D(0, 0, 0), Direction3D.D.X);
    private static readonly Axis3D Oy = new(new Point3D(0, 0, 0), Direction3D.D.Y);
    private static readonly Axis3D Oz = new(new Point3D(0, 0, 0), Direction3D.D.Z);
    private static readonly Frame3D Xoy = new(new Point3D(0, 0, 0), Direction3D.D.Z);
    private static readonly Frame3D Zox = new(new Point3D(0, 0, 0), Direction3D.D.Y);
    private static readonly Frame3D Yoz = new(new Point3D(0, 0, 0), Direction3D.D.X);

    /// <summary>The tolerance below which a length or a vector's magnitude counts as zero (<see cref="Standard.RealSmall"/>).</summary>
    public static double Resolution() => RealSmall();

    public static ref readonly Point3D Origin() => ref OriginPoint;

    public static ref readonly Direction3D DX() => ref Dx;

    public static ref readonly Direction3D DY() => ref Dy;

    public static ref readonly Direction3D DZ() => ref Dz;

    public static ref readonly Axis3D OX() => ref Ox;

    public static ref readonly Axis3D OY() => ref Oy;

    public static ref readonly Axis3D OZ() => ref Oz;

    public static ref readonly Frame3D XOY() => ref Xoy;

    public static ref readonly Frame3D ZOX() => ref Zox;

    public static ref readonly Frame3D YOZ() => ref Yoz;

    /// <summary>"name(a, b, c)", each number round-trippable and culture-invariant, for the gp types' ToString.</summary>
    internal static string Format(string name, params ReadOnlySpan<double> values)
    {
        var builder = new StringBuilder(name).Append('(');
        for (int i = 0; i < values.Length; i++)
        {
            if (i > 0)
            {
                builder.Append(", ");
            }

            builder.Append(values[i].ToString("R", CultureInfo.InvariantCulture));
        }

        return builder.Append(')').ToString();
    }
}
