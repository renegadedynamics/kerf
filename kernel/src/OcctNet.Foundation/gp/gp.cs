// Port of: src/FoundationClasses/TKMath/gp/gp.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;

namespace Occt;

/// <summary>The constants of the gp package: the resolution, and the origin, axes and planes of the global system.</summary>
/// <remarks>The 2D constants are in gp_2d.cs.</remarks>
public static partial class gp
{
    private static readonly gp_Pnt s_Origin = new(0, 0, 0);
    private static readonly gp_Dir s_DX = new(gp_Dir.D.X);
    private static readonly gp_Dir s_DY = new(gp_Dir.D.Y);
    private static readonly gp_Dir s_DZ = new(gp_Dir.D.Z);
    private static readonly gp_Ax1 s_OX = new(new gp_Pnt(0, 0, 0), gp_Dir.D.X);
    private static readonly gp_Ax1 s_OY = new(new gp_Pnt(0, 0, 0), gp_Dir.D.Y);
    private static readonly gp_Ax1 s_OZ = new(new gp_Pnt(0, 0, 0), gp_Dir.D.Z);
    private static readonly gp_Ax2 s_XOY = new(new gp_Pnt(0, 0, 0), gp_Dir.D.Z);
    private static readonly gp_Ax2 s_ZOX = new(new gp_Pnt(0, 0, 0), gp_Dir.D.Y);
    private static readonly gp_Ax2 s_YOZ = new(new gp_Pnt(0, 0, 0), gp_Dir.D.X);

    /// <summary>The tolerance below which a length or a vector's magnitude counts as zero (<see cref="Standard.RealSmall"/>).</summary>
    public static double Resolution() => RealSmall();

    public static ref readonly gp_Pnt Origin() => ref s_Origin;

    public static ref readonly gp_Dir DX() => ref s_DX;

    public static ref readonly gp_Dir DY() => ref s_DY;

    public static ref readonly gp_Dir DZ() => ref s_DZ;

    public static ref readonly gp_Ax1 OX() => ref s_OX;

    public static ref readonly gp_Ax1 OY() => ref s_OY;

    public static ref readonly gp_Ax1 OZ() => ref s_OZ;

    public static ref readonly gp_Ax2 XOY() => ref s_XOY;

    public static ref readonly gp_Ax2 ZOX() => ref s_ZOX;

    public static ref readonly gp_Ax2 YOZ() => ref s_YOZ;

    /// <summary>"name(a, b, c)", each number round-trippable and culture-invariant, for the gp types' ToString.</summary>
    internal static string Format(string theName, params ReadOnlySpan<double> theValues)
    {
        var aBuilder = new StringBuilder(theName).Append('(');
        for (int i = 0; i < theValues.Length; i++)
        {
            if (i > 0)
            {
                aBuilder.Append(", ");
            }

            aBuilder.Append(theValues[i].ToString("R", CultureInfo.InvariantCulture));
        }

        return aBuilder.Append(')').ToString();
    }
}
