// Port of: src/Visualization/TKV3d/Prs3d/Prs3d.hxx (GetDeflection only)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;
using Occt;

namespace Kerf.Engine.HiddenLines.Presentation;

/// <summary>
/// The presentation services of TKV3d that meshing for display needs: the absolute deflection a relative deviation
/// coefficient gives a bounding box.
/// </summary>
/// <remarks>
/// Kerf ports only the deflection functions of Prs3d (unit 4L2): the rest builds graphic primitives, and the port
/// has no viewer.
/// </remarks>
internal static class Prs3d
{
    /// <summary>
    /// The absolute deflection for the relative deflection <paramref name="deviationCoefficient"/> (from
    /// <see cref="Drawer.DeviationCoefficient"/>): the largest extent of the box times the coefficient times 4,
    /// at least the confusion tolerance.
    /// </summary>
    public static double GetDeflection(in Xyz bndMin, in Xyz bndMax, double deviationCoefficient)
    {
        Xyz diag = bndMax - bndMin;
        // NCollection_Vec3::maxComp
        double maxComp = Math.Max(Math.Max(diag.X(), diag.Y()), diag.Z());
        return Math.Max(maxComp * deviationCoefficient * 4.0, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>
    /// The absolute deflection for the relative deflection <paramref name="deviationCoefficient"/> on the box, or
    /// <paramref name="maximalChordialDeviation"/> (from <see cref="Drawer.MaximalChordialDeviation"/>) if the
    /// box is void or infinite.
    /// </summary>
    public static double GetDeflection(in Box bndBox, double deviationCoefficient, double maximalChordialDeviation)
    {
        if (bndBox.IsVoid)
        {
            return maximalChordialDeviation;
        }

        Box aBndBox = bndBox;
        if (bndBox.IsOpen)
        {
            if (!bndBox.HasFinitePart)
            {
                return maximalChordialDeviation;
            }

            aBndBox = bndBox.FinitePart();
        }

        aBndBox.Get(out double xMin, out double yMin, out double zMin, out double xMax, out double yMax, out double zMax);
        return GetDeflection(new Xyz(xMin, yMin, zMin), new Xyz(xMax, yMax, zMax), deviationCoefficient);
    }
}
