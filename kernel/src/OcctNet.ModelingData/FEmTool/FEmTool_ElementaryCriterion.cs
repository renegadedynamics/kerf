// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ElementaryCriterion.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ElementaryCriterion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>A criterion J(E) on an element E (a polynomial piece of a curve), to be minimised.</summary>
/// <remarks>
/// A working object (decision 21): <see cref="Set(NCollection_HArray2{double})"/> keeps the coefficient array it is
/// given, as OCCT keeps the handle, and the criterion reads it when evaluated.
/// </remarks>
public abstract class FEmTool_ElementaryCriterion : Transient
{
    protected NCollection_HArray2<double>? myCoeff;
    protected double myFirst;
    protected double myLast;

    protected FEmTool_ElementaryCriterion()
    {
        myFirst = 0.0;
        myLast = 0.0;
    }

    /// <summary>Sets the coefficients of the element (the curve): (degree, dimension), shared.</summary>
    public void Set(NCollection_HArray2<double> Coeff)
    {
        myCoeff = Coeff;
    }

    /// <summary>Sets the definition interval of the element.</summary>
    public virtual void Set(double FirstKnot, double LastKnot)
    {
        myFirst = FirstKnot;
        myLast = LastKnot;
    }

    /// <summary>Which pairs of dimensions depend on each other (non-zero where they do).</summary>
    public abstract NCollection_HArray2<int> DependenceTable();

    /// <summary>J(E), where E is the current element.</summary>
    public abstract double Value();

    /// <summary>
    /// The coefficients of the Hessian matrix of J(E) crossed between dimensions <paramref name="Dim1"/> and
    /// <paramref name="Dim2"/>. Throws Standard_DomainError if they are independent.
    /// </summary>
    public abstract void Hessian(int Dim1, int Dim2, Matrix H);

    /// <summary>The coefficients of the gradient of J(E) in dimension <paramref name="Dim"/>.</summary>
    public abstract void Gradient(int Dim, math_Vector G);
}
