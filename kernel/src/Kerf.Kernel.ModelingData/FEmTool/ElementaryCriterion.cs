// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ElementaryCriterion.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ElementaryCriterion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;
using Kerf.Engine;

namespace Kerf.Engine.Approximation.FiniteElements;

/// <summary>A criterion J(E) on an element E (a polynomial piece of a curve), to be minimised.</summary>
/// <remarks>
/// A working object (decision 21): <see cref="Set(NCollection_HArray2{double})"/> keeps the coefficient array it is
/// given, as OCCT keeps the handle, and the criterion reads it when evaluated.
/// </remarks>
internal abstract class ElementaryCriterion : Transient
{
    protected NCollection_HArray2<double>? _coeff;
    protected double _first;
    protected double _last;

    protected ElementaryCriterion()
    {
        _first = 0.0;
        _last = 0.0;
    }

    /// <summary>Sets the coefficients of the element (the curve): (degree, dimension), shared.</summary>
    public void Set(NCollection_HArray2<double> coeff)
    {
        _coeff = coeff;
    }

    /// <summary>Sets the definition interval of the element.</summary>
    public virtual void Set(double firstKnot, double lastKnot)
    {
        _first = firstKnot;
        _last = lastKnot;
    }

    /// <summary>Which pairs of dimensions depend on each other (non-zero where they do).</summary>
    public abstract NCollection_HArray2<int> DependenceTable();

    /// <summary>J(E), where E is the current element.</summary>
    public abstract double Value();

    /// <summary>
    /// The coefficients of the Hessian matrix of J(E) crossed between dimensions <paramref name="dim1"/> and
    /// <paramref name="dim2"/>. Throws Standard_DomainError if they are independent.
    /// </summary>
    public abstract void Hessian(int dim1, int dim2, Matrix h);

    /// <summary>The coefficients of the gradient of J(E) in dimension <paramref name="dim"/>.</summary>
    public abstract void Gradient(int dim, math_Vector g);
}
