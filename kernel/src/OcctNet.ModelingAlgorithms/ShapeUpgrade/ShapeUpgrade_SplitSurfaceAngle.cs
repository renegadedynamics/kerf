// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceAngle.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceAngle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Splits a surface of revolution, cylindrical, conical, spherical or toroidal surface (or a trimmed or offset one) in
/// U into segments no longer than the maximal angle.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
public class ShapeUpgrade_SplitSurfaceAngle : ShapeUpgrade_SplitSurface
{
    private double myMaxAngle;

    /// <summary>Creates the tool with the maximal angle.</summary>
    public ShapeUpgrade_SplitSurfaceAngle(double MaxAngle)
    {
        myMaxAngle = MaxAngle;
    }

    /// <summary>Sets the maximal angle.</summary>
    public void SetMaxAngle(double MaxAngle)
    {
        myMaxAngle = MaxAngle;
    }

    /// <summary>Returns the maximal angle.</summary>
    public double MaxAngle() => myMaxAngle;

    /// <summary>Computes the U splitting values; the inherited Build splits.</summary>
    public override void Compute(bool Segment)
    {
        Geom_Surface S;
        double U1 = 0.0, U2 = 0.0;
        bool isRect = false;
        if (mySurface is Geom_RectangularTrimmedSurface rts)
        {
            isRect = true;
            rts.Bounds(out U1, out U2, out _, out _);
            S = rts.BasisSurface();
        }
        else if (mySurface is Geom_OffsetSurface ofs)
        {
            S = ofs.BasisSurface();
        }
        else
        {
            S = mySurface!;
        }

        if (S is Geom_SurfaceOfRevolution || S is Geom_ConicalSurface || S is Geom_ToroidalSurface
            || S is Geom_CylindricalSurface || S is Geom_SphericalSurface)
        {
            double UFirst = myUSplitValues!.Sequence().First();
            double ULast = myUSplitValues.Sequence().Last();
            double maxAngle = myMaxAngle; // maximal u length of segment
            double uLength = ULast - UFirst;
            int nbSegments = (int)((uLength - Kerf.Engine.Foundation.Precision.Angular()) / maxAngle) + 1;
            if (nbSegments == 1)
            {
                if (!isRect || !(uLength < maxAngle) || !((U2 - U1) < maxAngle))
                {
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                }
            }
            double segAngle = uLength / nbSegments;
            double currAngle = segAngle + UFirst;
            var splitValues = new NCollection_HSequence<double>();
            for (int i = 1; i < nbSegments; i++, currAngle += segAngle)
            {
                splitValues.Append(currAngle);
            }
            SetUSplitValues(splitValues);
        }
    }
}
