// Port of: src/Visualization/TKV3d/Prs3d/Prs3d.hxx (GetDeflection only)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

namespace Occt;

/// <summary>
/// The presentation services of TKV3d that meshing for display needs: the absolute deflection a relative deviation
/// coefficient gives a bounding box.
/// </summary>
/// <remarks>
/// OCCT.NET ports only the deflection functions of Prs3d (unit 4L2): the rest builds graphic primitives, and the port
/// has no viewer.
/// </remarks>
public static class Prs3d
{
    /// <summary>
    /// The absolute deflection for the relative deflection <paramref name="theDeviationCoefficient"/> (from
    /// <see cref="Prs3d_Drawer.DeviationCoefficient"/>): the largest extent of the box times the coefficient times 4,
    /// at least the confusion tolerance.
    /// </summary>
    public static double GetDeflection(in Xyz theBndMin, in Xyz theBndMax, double theDeviationCoefficient)
    {
        Xyz aDiag = theBndMax - theBndMin;
        // NCollection_Vec3::maxComp
        double aMaxComp = Math.Max(Math.Max(aDiag.X(), aDiag.Y()), aDiag.Z());
        return Math.Max(aMaxComp * theDeviationCoefficient * 4.0, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>
    /// The absolute deflection for the relative deflection <paramref name="theDeviationCoefficient"/> on the box, or
    /// <paramref name="theMaximalChordialDeviation"/> (from <see cref="Prs3d_Drawer.MaximalChordialDeviation"/>) if the
    /// box is void or infinite.
    /// </summary>
    public static double GetDeflection(in Box theBndBox, double theDeviationCoefficient, double theMaximalChordialDeviation)
    {
        if (theBndBox.IsVoid)
        {
            return theMaximalChordialDeviation;
        }

        Box aBndBox = theBndBox;
        if (theBndBox.IsOpen)
        {
            if (!theBndBox.HasFinitePart)
            {
                return theMaximalChordialDeviation;
            }

            aBndBox = theBndBox.FinitePart();
        }

        aBndBox.Get(out double aXMin, out double aYMin, out double aZMin, out double aXMax, out double aYMax, out double aZMax);
        return GetDeflection(new Xyz(aXMin, aYMin, aZMin), new Xyz(aXMax, aYMax, aZMax), theDeviationCoefficient);
    }
}
