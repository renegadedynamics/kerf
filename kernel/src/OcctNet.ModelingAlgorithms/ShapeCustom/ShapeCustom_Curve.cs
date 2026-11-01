// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Curve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Converts a closed B-spline curve to its periodic form.</summary>
/// <remarks>A working object (decision 21).</remarks>
public sealed class ShapeCustom_Curve
{
    private Geom_Curve? myCurve;

    /// <summary>An empty tool.</summary>
    public ShapeCustom_Curve()
    {
    }

    /// <summary>A tool on the curve.</summary>
    public ShapeCustom_Curve(Geom_Curve? C)
    {
        Init(C);
    }

    /// <summary>Sets the curve.</summary>
    public void Init(Geom_Curve? C)
    {
        myCurve = C;
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
    public Geom_Curve? ConvertToPeriodic(bool substitute, double preci = -1)
    {
        Geom_Curve? newCurve = null;
        Geom_BSplineCurve? BSpl = myCurve as Geom_BSplineCurve;
        if (BSpl is null)
        {
            return newCurve;
        }

        // PTV 13.02.02: check if curve closed with tolerance
        bool closed = ShapeAnalysis_Curve.IsClosed(myCurve!, preci);

        if (!closed)
        {
            return newCurve;
        }

        bool converted = false; //: p6

        if (closed && !BSpl.IsPeriodic() && BSpl.NbPoles() > 3)
        {
            bool set = true;
            // if degree+1 at ends, first change it to 1 by rearranging knots
            if (BSpl.Multiplicity(1) == BSpl.Degree() + 1
                && BSpl.Multiplicity(BSpl.NbKnots()) == BSpl.Degree() + 1)
            {
                NCollection_Array1<gp_Pnt> oldPoles = BSpl.InternalPoles();
                NCollection_Array1<double> oldWeights = BSpl.InternalWeightsArray();
                int nbKnots = BSpl.NbKnots();
                NCollection_Array1<double> oldKnots = BSpl.InternalKnots();
                NCollection_Array1<int> oldMults = BSpl.InternalMultiplicities();

                NCollection_Array1<double> newKnots = new(1, nbKnots + 2);
                NCollection_Array1<int> newMults = new(1, nbKnots + 2);
                double a =
                    0.5 * (BSpl.Knot(2) - BSpl.Knot(1) + BSpl.Knot(nbKnots) - BSpl.Knot(nbKnots - 1));

                newKnots[1] = oldKnots[1] - a;
                newKnots[nbKnots + 2] = oldKnots[nbKnots] + a;
                newMults[1] = newMults[nbKnots + 2] = 1;
                for (int i = 2; i <= nbKnots + 1; i++)
                {
                    newKnots[i] = oldKnots[i - 1];
                    newMults[i] = oldMults[i - 1];
                }
                newMults[2] = newMults[nbKnots + 1] = BSpl.Degree();
                Geom_BSplineCurve res = new Geom_BSplineCurve(oldPoles,
                                                              oldWeights,
                                                              newKnots,
                                                              newMults,
                                                              BSpl.Degree(),
                                                              BSpl.IsPeriodic());
                BSpl = res;
            }
            else if (BSpl.Multiplicity(1) > BSpl.Degree()
                     || BSpl.Multiplicity(BSpl.NbKnots()) > BSpl.Degree() + 1)
            {
                set = false;
            }
            if (set)
            {
                // OCCT.NET: OCCT makes the caller's curve periodic in place; a frozen curve is copied first
                // (decision 974)
                if (BSpl.IsFrozen())
                {
                    BSpl = (Geom_BSplineCurve)BSpl.Copy();
                }
                BSpl.SetPeriodic(); // make periodic
                converted = true;
            }
        }
        if (!converted)
        {
            return newCurve;
        }
        newCurve = BSpl;
        if (substitute)
        {
            myCurve = newCurve;
        }
        return newCurve;
    }
}
