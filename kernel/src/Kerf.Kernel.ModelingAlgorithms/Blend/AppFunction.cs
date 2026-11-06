// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_AppFunction.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_AppFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of a blending surface along a guide line, as the walking and the approximation see it: a
/// <see cref="FunctionSetWithDerivatives"/> solved at each parameter of the guide, and the section generator of
/// AppBlend_AppSurf (decision 845).
/// </summary>
/// <remarks>A working object (decision 21): the walking sets its parameter and calls it repeatedly.</remarks>
internal abstract class AppFunction : FunctionSetWithDerivatives, ISectionGenerator<BlendPoint>
{
    /// <summary>Sets the value of the parameter along the guide line: the plane in which the solution is found.</summary>
    public abstract void Set(double param);

    /// <summary>Sets the bounds of the parametric interval on the guide line.</summary>
    public abstract void Set(double first, double last);

    /// <summary>The parametric tolerance of each variable, from the 3d tolerance <paramref name="tol"/>.</summary>
    public abstract void GetTolerance(math_Vector tolerance, double tol);

    /// <summary>The lowest and greatest values allowed for each variable.</summary>
    public abstract void GetBounds(math_Vector infBound, math_Vector supBound);

    /// <summary>True if <paramref name="sol"/> is a zero of the function at the current guide parameter.</summary>
    public abstract bool IsSolution(math_Vector sol, double tol);

    /// <summary>The minimal distance between the two extremities of the computed sections.</summary>
    public abstract double GetMinimalDistance();

    /// <summary>The point on the first support.</summary>
    public abstract Point3D Pnt1();

    /// <summary>The point on the second support.</summary>
    public abstract Point3D Pnt2();

    /// <summary>True if the sections are rational.</summary>
    public abstract bool IsRational { get; }

    /// <summary>The length of the largest section.</summary>
    public abstract double GetSectionSize();

    /// <summary>The weights of the section with the minimal weights.</summary>
    public abstract void GetMinimalWeight(NCollection_Array1<double> weigths);

    /// <summary>The number of intervals of continuity <paramref name="s"/>.</summary>
    public abstract int NbIntervals(Continuity s);

    /// <summary>The bounds of the intervals of continuity <paramref name="s"/> (NbIntervals + 1 values).</summary>
    public abstract void Intervals(NCollection_Array1<double> t, Continuity s);

    /// <summary>The shape of every section: its poles, knots, degree and 2d poles.</summary>
    public abstract void GetShape(out int nbPoles, out int nbKnots, out int degree, out int nbPoles2d);

    /// <summary>The tolerances of the approximation from the boundary, surface and angle tolerances.</summary>
    public abstract void GetTolerance(double boundTol, double surfTol, double angleTol, math_Vector tol3d,
                                      math_Vector tol1D);

    /// <summary>Copies the sections' knots into <paramref name="tKnots"/>.</summary>
    public abstract void Knots(NCollection_Array1<double> tKnots);

    /// <summary>Copies the sections' multiplicities into <paramref name="tMults"/>.</summary>
    public abstract void Mults(NCollection_Array1<int> tMults);

    /// <summary>The section at <paramref name="p"/> with its first derivatives; false when they are not computed.</summary>
    public abstract bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weigths, NCollection_Array1<double> dWeigths);

    /// <summary>The section at <paramref name="p"/>.</summary>
    public abstract void Section(in BlendPoint p, NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<double> weigths);

    /// <summary>
    /// The section at <paramref name="p"/> with its first and second derivatives; false when they are not computed.
    /// </summary>
    public abstract bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<Point2D> poles2d,
                                 NCollection_Array1<Vector2D> dPoles2d, NCollection_Array1<Vector2D> d2Poles2d,
                                 NCollection_Array1<double> weigths, NCollection_Array1<double> dWeigths,
                                 NCollection_Array1<double> d2Weigths);

    /// <summary>
    /// The parametric tolerances on the support of the 2d curve <paramref name="ic2d"/> for the 3d tolerance
    /// <paramref name="tol"/>.
    /// </summary>
    public abstract void Resolution(int ic2d, double tol, out double tolU, out double tolV);

    /// <summary>The parameter of the point <paramref name="p"/> on the guide.</summary>
    public double Parameter(in BlendPoint p)
    {
        return p.Parameter();
    }

    bool ISectionGenerator<BlendPoint>.Section(BlendPoint p, NCollection_Array1<Point3D> poles,
                                                         NCollection_Array1<Vector3D> dPoles,
                                                         NCollection_Array1<Point2D> poles2d,
                                                         NCollection_Array1<Vector2D> dPoles2d,
                                                         NCollection_Array1<double> weigths,
                                                         NCollection_Array1<double> dWeigths)
        => Section(in p, poles, dPoles, poles2d, dPoles2d, weigths, dWeigths);

    void ISectionGenerator<BlendPoint>.Section(BlendPoint p, NCollection_Array1<Point3D> poles,
                                                         NCollection_Array1<Point2D> poles2d,
                                                         NCollection_Array1<double> weigths)
        => Section(in p, poles, poles2d, weigths);

    double ISectionGenerator<BlendPoint>.Parameter(BlendPoint p) => Parameter(in p);
}
