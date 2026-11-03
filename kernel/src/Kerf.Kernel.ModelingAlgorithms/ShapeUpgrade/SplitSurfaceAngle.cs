// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceAngle.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceAngle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Splits a surface of revolution, cylindrical, conical, spherical or toroidal surface (or a trimmed or offset one) in
/// U into segments no longer than the maximal angle.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
public class SplitSurfaceAngle : SplitSurface
{
    private double _maxAngle;

    /// <summary>Creates the tool with the maximal angle.</summary>
    public SplitSurfaceAngle(double maxAngle)
    {
        _maxAngle = maxAngle;
    }

    /// <summary>Sets the maximal angle.</summary>
    public void SetMaxAngle(double maxAngle)
    {
        _maxAngle = maxAngle;
    }

    /// <summary>Returns the maximal angle.</summary>
    public double MaxAngle() => _maxAngle;

    /// <summary>Computes the U splitting values; the inherited Build splits.</summary>
    public override void Compute(bool segment)
    {
        Surface s;
        double u1 = 0.0, u2 = 0.0;
        bool isRect = false;
        if (_surface is TrimmedSurface rts)
        {
            isRect = true;
            rts.ParameterBounds(out u1, out u2, out _, out _);
            s = rts.BasisSurface;
        }
        else if (_surface is OffsetSurface ofs)
        {
            s = ofs.BasisSurface;
        }
        else
        {
            s = _surface!;
        }

        if (s is RevolutionSurface || s is ConicalSurface || s is ToroidalSurface
            || s is CylindricalSurface || s is SphericalSurface)
        {
            double uFirst = _uSplitValues!.Sequence().First();
            double uLast = _uSplitValues.Sequence().Last();
            double maxAngle = _maxAngle; // maximal u length of segment
            double uLength = uLast - uFirst;
            int nbSegments = (int)((uLength - Kerf.Engine.Foundation.Precision.Angular()) / maxAngle) + 1;
            if (nbSegments == 1)
            {
                if (!isRect || !(uLength < maxAngle) || !((u2 - u1) < maxAngle))
                {
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                }
            }
            double segAngle = uLength / nbSegments;
            double currAngle = segAngle + uFirst;
            var splitValues = new NCollection_HSequence<double>();
            for (int i = 1; i < nbSegments; i++, currAngle += segAngle)
            {
                splitValues.Append(currAngle);
            }
            SetUSplitValues(splitValues);
        }
    }
}
