// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Curve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Healing.Custom;

/// <summary>Converts a closed B-spline curve to its periodic form.</summary>
/// <remarks>A working object (decision 21).</remarks>
public sealed class CustomCurve
{
    private Curve3D? _curve;

    /// <summary>An empty tool.</summary>
    public CustomCurve()
    {
    }

    /// <summary>A tool on the curve.</summary>
    public CustomCurve(Curve3D? c)
    {
        Init(c);
    }

    /// <summary>Sets the curve.</summary>
    public void Init(Curve3D? c)
    {
        _curve = c;
    }

    /// <summary>
    /// Tries to convert the curve to the periodic form, and returns the result: null unless the curve is a B-spline
    /// closed within <paramref name="preci"/> (Precision::Confusion() if negative) that could be converted. With
    /// <paramref name="substitute"/>, the result replaces the tool's curve.
    /// </summary>
    /// <remarks>
    /// OCCT makes the curve itself periodic when its knots needn't be rearranged; the port makes a copy of a frozen
    /// curve periodic instead (decision 974), so the caller's curve never changes.
    /// </remarks>
    public Curve3D? ConvertToPeriodic(bool substitute, double preci = -1)
    {
        Curve3D? newCurve = null;
        BSplineCurve3D? bSpl = _curve as BSplineCurve3D;
        if (bSpl is null)
        {
            return newCurve;
        }

        // PTV 13.02.02: check if curve closed with tolerance
        bool closed = CurveAnalysis.IsClosed(_curve!, preci);

        if (!closed)
        {
            return newCurve;
        }

        bool converted = false; //: p6

        if (closed && !bSpl.IsPeriodic && bSpl.PoleCount > 3)
        {
            bool set = true;
            // if degree+1 at ends, first change it to 1 by rearranging knots
            if (bSpl.Multiplicity(1) == bSpl.Degree + 1
                && bSpl.Multiplicity(bSpl.KnotCount) == bSpl.Degree + 1)
            {
                NCollection_Array1<Point3D> oldPoles = bSpl.InternalPoles();
                NCollection_Array1<double> oldWeights = bSpl.InternalWeightsArray();
                int nbKnots = bSpl.KnotCount;
                NCollection_Array1<double> oldKnots = bSpl.InternalKnots();
                NCollection_Array1<int> oldMults = bSpl.InternalMultiplicities();

                NCollection_Array1<double> newKnots = new(1, nbKnots + 2);
                NCollection_Array1<int> newMults = new(1, nbKnots + 2);
                double a =
                    0.5 * (bSpl.Knot(2) - bSpl.Knot(1) + bSpl.Knot(nbKnots) - bSpl.Knot(nbKnots - 1));

                newKnots[1] = oldKnots[1] - a;
                newKnots[nbKnots + 2] = oldKnots[nbKnots] + a;
                newMults[1] = newMults[nbKnots + 2] = 1;
                for (int i = 2; i <= nbKnots + 1; i++)
                {
                    newKnots[i] = oldKnots[i - 1];
                    newMults[i] = oldMults[i - 1];
                }
                newMults[2] = newMults[nbKnots + 1] = bSpl.Degree;
                BSplineCurve3D res = new BSplineCurve3D(oldPoles,
                                                              oldWeights,
                                                              newKnots,
                                                              newMults,
                                                              bSpl.Degree,
                                                              bSpl.IsPeriodic);
                bSpl = res;
            }
            else if (bSpl.Multiplicity(1) > bSpl.Degree
                     || bSpl.Multiplicity(bSpl.KnotCount) > bSpl.Degree + 1)
            {
                set = false;
            }
            if (set)
            {
                // OCCT.NET: OCCT makes the caller's curve periodic in place; a frozen curve is copied first
                // (decision 974)
                if (bSpl.IsFrozen)
                {
                    bSpl = (BSplineCurve3D)bSpl.Copy();
                }
                bSpl.SetPeriodic(); // make periodic
                converted = true;
            }
        }
        if (!converted)
        {
            return newCurve;
        }
        newCurve = bSpl;
        if (substitute)
        {
            _curve = newCurve;
        }
        return newCurve;
    }
}
