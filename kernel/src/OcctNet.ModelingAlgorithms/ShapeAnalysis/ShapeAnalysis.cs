// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Tools for analysing shapes and their geometry: periodic adjustment, wire bounds and orientation, outer wires.</summary>
public static class ShapeAnalysis
{
    /// <summary>
    /// Returns a shift required to move point <paramref name="Val"/> to the range [ToVal-Period/2, ToVal+Period/2].
    /// The shift is a multiple of the period; 0 if the point is already in range.
    /// </summary>
    public static double AdjustByPeriod(double Val, double ToVal, double Period)
    {
        double diff = Val - ToVal;
        double D    = Math.Abs(diff);
        double P    = Math.Abs(Period);
        if (D <= 0.5 * P)
        {
            return 0.0;
        }

        if (P < 1e-100)
        {
            return diff;
        }

        return (diff > 0 ? -P : P) * Math.Floor(D / P + 0.5);
    }

    /// <summary>Returns a shift required to move point <paramref name="Val"/> to the range [ValMin, ValMax].</summary>
    public static double AdjustToPeriod(double Val, double ValMin, double ValMax)
    {
        return AdjustByPeriod(Val, 0.5 * (ValMin + ValMax), ValMax - ValMin);
    }

    // TODO(4S-C): FindBounds, TotCross2D, ContourArea, IsOuterBound, OuterWire, GetFaceUVBounds.
}
