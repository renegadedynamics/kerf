// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_SmoothCriterion.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_SmoothCriterion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The criterion minimised by <see cref="AppDef_Variational"/> to smooth points into a curve: a weighted sum of a
/// least-squares term and quality terms (tension, flexion, jerk).
/// </summary>
/// <remarks>
/// A working object (decision 21): AppDef_Variational sets its parameters, curve, estimations and weights, and
/// reads the assembly data back. The arrays and the curve it is given or returns are shared, as OCCT's handles are
/// (decision 207).
/// </remarks>
public abstract class AppDef_SmoothCriterion : Standard_Transient
{
    /// <summary>Sets the parameters of the points (kept, shared).</summary>
    public abstract void SetParameters(NCollection_HArray1<double> Parameters);

    /// <summary>Sets the curve whose elements the criterion evaluates (kept, shared).</summary>
    public abstract void SetCurve(FEmTool_Curve C);

    /// <summary>The curve associated with this criterion.</summary>
    public FEmTool_Curve? Curve()
    {
        GetCurve(out FEmTool_Curve? aCurve);
        return aCurve;
    }

    /// <summary>The curve associated with this criterion (null if none was set).</summary>
    public abstract void GetCurve(out FEmTool_Curve? C);

    /// <summary>Sets the estimations of the three quality criteria.</summary>
    public abstract void SetEstimation(double E1, double E2, double E3);

    /// <summary>The estimated length of the curve, for reading and writing.</summary>
    public abstract ref double EstLength();

    /// <summary>The estimations of the three quality criteria.</summary>
    public abstract void GetEstimation(out double E1, out double E2, out double E3);

    /// <summary>The assembly table: the global indices of the coefficients of each (dimension, element).</summary>
    public abstract NCollection_HArray2<NCollection_HArray1<int>> AssemblyTable();

    /// <summary>The dependence table of the dimensions.</summary>
    public abstract NCollection_HArray2<int> DependenceTable();

    /// <summary>
    /// The values of the quality criteria, and a code telling whether the estimations were good (0), changed (1)
    /// or bad (2). Updates the estimations.
    /// </summary>
    public abstract int QualityValues(double J1min, double J2min, double J3min, out double J1, out double J2,
                                      out double J3);

    /// <summary>The maximum, quadratic and average distances between the points and the curve.</summary>
    public abstract void ErrorValues(out double MaxError, out double QuadraticError, out double AverageError);

    /// <summary>Writes into <paramref name="H"/> the Hessian of the criterion on an element.</summary>
    public abstract void Hessian(int Element, int Dimension1, int Dimension2, math_Matrix H);

    /// <summary>Writes into <paramref name="G"/> the gradient of the criterion on an element.</summary>
    public abstract void Gradient(int Element, int Dimension, math_Vector G);

    /// <summary>Converts the assembly vector into the curve.</summary>
    public abstract void InputVector(math_Vector X, NCollection_HArray2<NCollection_HArray1<int>> AssTable);

    /// <summary>Sets the weights of the least-squares and quality terms, and the shares of the three quality criteria.</summary>
    public abstract void SetWeight(double QuadraticWeight, double QualityWeight, double percentJ1, double percentJ2,
                                   double percentJ3);

    /// <summary>The weights of the least-squares and quality terms.</summary>
    public abstract void GetWeight(out double QuadraticWeight, out double QualityWeight);

    /// <summary>Sets the weights of the points (copied).</summary>
    public abstract void SetWeight(NCollection_Array1<double> Weight);
}
