// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepFunction.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function swept by <see cref="Approx_SweepApproximation"/>: at each parameter v, a section given as the poles
/// (3D, 2D, weights) of a B-spline of fixed shape, and its derivatives in v.
/// </summary>
/// <remarks>
/// Implementations (GeomFill's sweep functions) are working objects: <see cref="SetInterval"/> and
/// <see cref="SetTolerance"/> change them. The optional members throw Standard_NotImplemented, as in OCCT.
/// </remarks>
public abstract class Approx_SweepFunction : Transient
{
    /// <summary>Computes the section for v = Param.</summary>
    public abstract bool D0(double Param,
                            double First,
                            double Last,
                            NCollection_Array1<Point3D> Poles,
                            NCollection_Array1<Point2D> Poles2d,
                            NCollection_Array1<double> Weigths);

    /// <summary>Computes the section and its first derivative in v for v = Param (C1 and C2 approximations only).</summary>
    public virtual bool D1(double Param,
                           double First,
                           double Last,
                           NCollection_Array1<Point3D> Poles,
                           NCollection_Array1<Vector3D> DPoles,
                           NCollection_Array1<Point2D> Poles2d,
                           NCollection_Array1<Vector2D> DPoles2d,
                           NCollection_Array1<double> Weigths,
                           NCollection_Array1<double> DWeigths)
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::D1");
    }

    /// <summary>Computes the section and its first two derivatives in v for v = Param (C2 approximations only).</summary>
    public virtual bool D2(double Param,
                           double First,
                           double Last,
                           NCollection_Array1<Point3D> Poles,
                           NCollection_Array1<Vector3D> DPoles,
                           NCollection_Array1<Vector3D> D2Poles,
                           NCollection_Array1<Point2D> Poles2d,
                           NCollection_Array1<Vector2D> DPoles2d,
                           NCollection_Array1<Vector2D> D2Poles2d,
                           NCollection_Array1<double> Weigths,
                           NCollection_Array1<double> DWeigths,
                           NCollection_Array1<double> D2Weigths)
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::D2");
    }

    /// <summary>The number of 2D curves to approximate.</summary>
    public abstract int Nb2dCurves();

    /// <summary>The shape of a section: its number of poles and knots, and its degree.</summary>
    public abstract void SectionShape(out int NbPoles, out int NbKnots, out int Degree);

    /// <summary>The knots of the section.</summary>
    public abstract void Knots(NCollection_Array1<double> TKnots);

    /// <summary>The multiplicities of the section.</summary>
    public abstract void Mults(NCollection_Array1<int> TMults);

    /// <summary>Whether the sections are rational.</summary>
    public abstract bool IsRational();

    /// <summary>The number of intervals of continuity S.</summary>
    public abstract int NbIntervals(Continuity S);

    /// <summary>Stores in T the parameters bounding the intervals of continuity S.</summary>
    public abstract void Intervals(NCollection_Array1<double> T, Continuity S);

    /// <summary>Sets the bounds of the parametric interval of the function.</summary>
    public abstract void SetInterval(double First, double Last);

    /// <summary>The resolutions in the 2D subspace Index, for a 3D tolerance Tol.</summary>
    public virtual void Resolution(int Index, double Tol, out double TolU, out double TolV)
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::Resolution");
    }

    /// <summary>
    /// The tolerances of the approximation (one per pole) that satisfy the boundary error BoundTol, the tangent
    /// error AngleTol at the boundary and the error SurfTol inside the surface.
    /// </summary>
    public abstract void GetTolerance(double BoundTol, double SurfTol, double AngleTol,
                                      NCollection_Array1<double> Tol3d);

    /// <summary>Sets the tolerances of the numerical algorithms D0, D1 and D2 may run.</summary>
    public abstract void SetTolerance(double Tol3d, double Tol2d);

    /// <summary>A rough barycentre of the surface (rational sections only).</summary>
    public virtual Point3D BarycentreOfSurf()
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::BarycentreOfSurf");
    }

    /// <summary>The length of the largest section.</summary>
    public virtual double MaximalSection()
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::MaximalSection()");
    }

    /// <summary>The minimal weight of each pole over all the sections (rational sections only).</summary>
    public virtual void GetMinimalWeight(NCollection_Array1<double> Weigths)
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::GetMinimalWeight");
    }
}
