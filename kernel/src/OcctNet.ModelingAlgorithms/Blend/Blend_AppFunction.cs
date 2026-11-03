// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_AppFunction.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_AppFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function of a blending surface along a guide line, as the walking and the approximation see it: a
/// <see cref="FunctionSetWithDerivatives"/> solved at each parameter of the guide, and the section generator of
/// AppBlend_AppSurf (decision 845).
/// </summary>
/// <remarks>A working object (decision 21): the walking sets its parameter and calls it repeatedly.</remarks>
public abstract class Blend_AppFunction : FunctionSetWithDerivatives, IAppBlend_SectionGenerator<Blend_Point>
{
    /// <summary>Sets the value of the parameter along the guide line: the plane in which the solution is found.</summary>
    public abstract void Set(double Param);

    /// <summary>Sets the bounds of the parametric interval on the guide line.</summary>
    public abstract void Set(double First, double Last);

    /// <summary>The parametric tolerance of each variable, from the 3d tolerance <paramref name="Tol"/>.</summary>
    public abstract void GetTolerance(math_Vector Tolerance, double Tol);

    /// <summary>The lowest and greatest values allowed for each variable.</summary>
    public abstract void GetBounds(math_Vector InfBound, math_Vector SupBound);

    /// <summary>True if <paramref name="Sol"/> is a zero of the function at the current guide parameter.</summary>
    public abstract bool IsSolution(math_Vector Sol, double Tol);

    /// <summary>The minimal distance between the two extremities of the computed sections.</summary>
    public abstract double GetMinimalDistance();

    /// <summary>The point on the first support.</summary>
    public abstract Point3D Pnt1();

    /// <summary>The point on the second support.</summary>
    public abstract Point3D Pnt2();

    /// <summary>True if the sections are rational.</summary>
    public abstract bool IsRational();

    /// <summary>The length of the largest section.</summary>
    public abstract double GetSectionSize();

    /// <summary>The weights of the section with the minimal weights.</summary>
    public abstract void GetMinimalWeight(NCollection_Array1<double> Weigths);

    /// <summary>The number of intervals of continuity <paramref name="S"/>.</summary>
    public abstract int NbIntervals(Continuity S);

    /// <summary>The bounds of the intervals of continuity <paramref name="S"/> (NbIntervals + 1 values).</summary>
    public abstract void Intervals(NCollection_Array1<double> T, Continuity S);

    /// <summary>The shape of every section: its poles, knots, degree and 2d poles.</summary>
    public abstract void GetShape(out int NbPoles, out int NbKnots, out int Degree, out int NbPoles2d);

    /// <summary>The tolerances of the approximation from the boundary, surface and angle tolerances.</summary>
    public abstract void GetTolerance(double BoundTol, double SurfTol, double AngleTol, math_Vector Tol3d,
                                      math_Vector Tol1D);

    /// <summary>Copies the sections' knots into <paramref name="TKnots"/>.</summary>
    public abstract void Knots(NCollection_Array1<double> TKnots);

    /// <summary>Copies the sections' multiplicities into <paramref name="TMults"/>.</summary>
    public abstract void Mults(NCollection_Array1<int> TMults);

    /// <summary>The section at <paramref name="P"/> with its first derivatives; false when they are not computed.</summary>
    public abstract bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                                 NCollection_Array1<double> Weigths, NCollection_Array1<double> DWeigths);

    /// <summary>The section at <paramref name="P"/>.</summary>
    public abstract void Section(in Blend_Point P, NCollection_Array1<Point3D> Poles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<double> Weigths);

    /// <summary>
    /// The section at <paramref name="P"/> with its first and second derivatives; false when they are not computed.
    /// </summary>
    public abstract bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Vector3D> D2Poles, NCollection_Array1<Point2D> Poles2d,
                                 NCollection_Array1<Vector2D> DPoles2d, NCollection_Array1<Vector2D> D2Poles2d,
                                 NCollection_Array1<double> Weigths, NCollection_Array1<double> DWeigths,
                                 NCollection_Array1<double> D2Weigths);

    /// <summary>
    /// The parametric tolerances on the support of the 2d curve <paramref name="IC2d"/> for the 3d tolerance
    /// <paramref name="Tol"/>.
    /// </summary>
    public abstract void Resolution(int IC2d, double Tol, out double TolU, out double TolV);

    /// <summary>The parameter of the point <paramref name="P"/> on the guide.</summary>
    public double Parameter(in Blend_Point P)
    {
        return P.Parameter();
    }

    bool IAppBlend_SectionGenerator<Blend_Point>.Section(Blend_Point P, NCollection_Array1<Point3D> Poles,
                                                         NCollection_Array1<Vector3D> DPoles,
                                                         NCollection_Array1<Point2D> Poles2d,
                                                         NCollection_Array1<Vector2D> DPoles2d,
                                                         NCollection_Array1<double> Weigths,
                                                         NCollection_Array1<double> DWeigths)
        => Section(in P, Poles, DPoles, Poles2d, DPoles2d, Weigths, DWeigths);

    void IAppBlend_SectionGenerator<Blend_Point>.Section(Blend_Point P, NCollection_Array1<Point3D> Poles,
                                                         NCollection_Array1<Point2D> Poles2d,
                                                         NCollection_Array1<double> Weigths)
        => Section(in P, Poles, Poles2d, Weigths);

    double IAppBlend_SectionGenerator<Blend_Point>.Parameter(Blend_Point P) => Parameter(in P);
}
