// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Function.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Function.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>The root class of evolution laws: a real function of one real parameter on bounded intervals.</summary>
/// <remarks>
/// A working object (decision 21, as the adaptors, decision 101): the laws keep OCCT's public setters, and some
/// cache evaluation state. <see cref="Trim"/> returns a new law; it never changes this one.
/// </remarks>
public abstract class Law_Function : Transient
{
    public abstract Continuity Continuity();

    /// <summary>The number of intervals for continuity S; may be one if Continuity() is at least S.</summary>
    public abstract int NbIntervals(Continuity S);

    /// <summary>Stores in T the parameters bounding the intervals of continuity S; T.Length() must exceed NbIntervals(S).</summary>
    public abstract void Intervals(NCollection_Array1<double> T, Continuity S);

    /// <summary>The value of the function at X.</summary>
    public abstract double Value(double X);

    /// <summary>The value F and the first derivative D at X.</summary>
    public abstract void D1(double X, out double F, out double D);

    /// <summary>The value and the first and second derivatives at X.</summary>
    public abstract void D2(double X, out double F, out double D, out double D2);

    /// <summary>A law equivalent to this one between PFirst and PLast; Tol tests for confusion of points.</summary>
    public abstract Law_Function Trim(double PFirst, double PLast, double Tol);

    /// <summary>The parametric bounds of the function.</summary>
    public abstract void Bounds(out double PFirst, out double PLast);
}
