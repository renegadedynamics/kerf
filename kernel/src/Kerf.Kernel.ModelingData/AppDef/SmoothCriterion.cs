// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_SmoothCriterion.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_SmoothCriterion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.FiniteElements;
using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>
/// The criterion minimised by <see cref="Variational"/> to smooth points into a curve: a weighted sum of a
/// least-squares term and quality terms (tension, flexion, jerk).
/// </summary>
/// <remarks>
/// A working object (decision 21): AppDef_Variational sets its parameters, curve, estimations and weights, and
/// reads the assembly data back. The arrays and the curve it is given or returns are shared, as OCCT's handles are
/// (decision 207).
/// </remarks>
internal abstract class SmoothCriterion : Transient
{
    /// <summary>Sets the parameters of the points (kept, shared).</summary>
    public abstract void SetParameters(NCollection_HArray1<double> parameters);

    /// <summary>Sets the curve whose elements the criterion evaluates (kept, shared).</summary>
    public abstract void SetCurve(Curve c);

    /// <summary>The curve associated with this criterion.</summary>
    public Curve? Curve()
    {
        GetCurve(out Curve? curve);
        return curve;
    }

    /// <summary>The curve associated with this criterion (null if none was set).</summary>
    public abstract void GetCurve(out Curve? c);

    /// <summary>Sets the estimations of the three quality criteria.</summary>
    public abstract void SetEstimation(double e1, double e2, double e3);

    /// <summary>The estimated length of the curve, for reading and writing.</summary>
    public abstract ref double EstLength();

    /// <summary>The estimations of the three quality criteria.</summary>
    public abstract void GetEstimation(out double e1, out double e2, out double e3);

    /// <summary>The assembly table: the global indices of the coefficients of each (dimension, element).</summary>
    public abstract NCollection_HArray2<NCollection_HArray1<int>> AssemblyTable();

    /// <summary>The dependence table of the dimensions.</summary>
    public abstract NCollection_HArray2<int> DependenceTable();

    /// <summary>
    /// The values of the quality criteria, and a code telling whether the estimations were good (0), changed (1)
    /// or bad (2). Updates the estimations.
    /// </summary>
    public abstract int QualityValues(double j1min, double j2min, double j3min, out double j1, out double j2,
                                      out double j3);

    /// <summary>The maximum, quadratic and average distances between the points and the curve.</summary>
    public abstract void ErrorValues(out double maxError, out double quadraticError, out double averageError);

    /// <summary>Writes into <paramref name="h"/> the Hessian of the criterion on an element.</summary>
    public abstract void Hessian(int element, int dimension1, int dimension2, Matrix h);

    /// <summary>Writes into <paramref name="g"/> the gradient of the criterion on an element.</summary>
    public abstract void Gradient(int element, int dimension, math_Vector g);

    /// <summary>Converts the assembly vector into the curve.</summary>
    public abstract void InputVector(math_Vector x, NCollection_HArray2<NCollection_HArray1<int>> assTable);

    /// <summary>Sets the weights of the least-squares and quality terms, and the shares of the three quality criteria.</summary>
    public abstract void SetWeight(double quadraticWeight, double qualityWeight, double percentJ1, double percentJ2,
                                   double percentJ3);

    /// <summary>The weights of the least-squares and quality terms.</summary>
    public abstract void GetWeight(out double quadraticWeight, out double qualityWeight);

    /// <summary>Sets the weights of the points (copied).</summary>
    public abstract void SetWeight(NCollection_Array1<double> weight);
}
