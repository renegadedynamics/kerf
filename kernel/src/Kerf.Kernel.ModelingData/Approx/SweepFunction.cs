// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepFunction.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>
/// The function swept by <see cref="SweepApproximation"/>: at each parameter v, a section given as the poles
/// (3D, 2D, weights) of a B-spline of fixed shape, and its derivatives in v.
/// </summary>
/// <remarks>
/// Implementations (GeomFill's sweep functions) are working objects: <see cref="SetInterval"/> and
/// <see cref="SetTolerance"/> change them. The optional members throw Standard_NotImplemented, as in OCCT.
/// </remarks>
public abstract class SweepFunction : Transient
{
    /// <summary>Computes the section for v = Param.</summary>
    public abstract bool D0(double param,
                            double first,
                            double last,
                            NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Point2D> poles2d,
                            NCollection_Array1<double> weigths);

    /// <summary>Computes the section and its first derivative in v for v = Param (C1 and C2 approximations only).</summary>
    public virtual bool D1(double param,
                           double first,
                           double last,
                           NCollection_Array1<Point3D> poles,
                           NCollection_Array1<Vector3D> dPoles,
                           NCollection_Array1<Point2D> poles2d,
                           NCollection_Array1<Vector2D> dPoles2d,
                           NCollection_Array1<double> weigths,
                           NCollection_Array1<double> dWeigths)
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::D1");
    }

    /// <summary>Computes the section and its first two derivatives in v for v = Param (C2 approximations only).</summary>
    public virtual bool D2(double param,
                           double first,
                           double last,
                           NCollection_Array1<Point3D> poles,
                           NCollection_Array1<Vector3D> dPoles,
                           NCollection_Array1<Vector3D> d2Poles,
                           NCollection_Array1<Point2D> poles2d,
                           NCollection_Array1<Vector2D> dPoles2d,
                           NCollection_Array1<Vector2D> d2Poles2d,
                           NCollection_Array1<double> weigths,
                           NCollection_Array1<double> dWeigths,
                           NCollection_Array1<double> d2Weigths)
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::D2");
    }

    /// <summary>The number of 2D curves to approximate.</summary>
    public abstract int Nb2dCurves();

    /// <summary>The shape of a section: its number of poles and knots, and its degree.</summary>
    public abstract void SectionShape(out int nbPoles, out int nbKnots, out int degree);

    /// <summary>The knots of the section.</summary>
    public abstract void Knots(NCollection_Array1<double> tKnots);

    /// <summary>The multiplicities of the section.</summary>
    public abstract void Mults(NCollection_Array1<int> tMults);

    /// <summary>Whether the sections are rational.</summary>
    public abstract bool IsRational { get; }

    /// <summary>The number of intervals of continuity S.</summary>
    public abstract int NbIntervals(Continuity s);

    /// <summary>Stores in T the parameters bounding the intervals of continuity S.</summary>
    public abstract void Intervals(NCollection_Array1<double> t, Continuity s);

    /// <summary>Sets the bounds of the parametric interval of the function.</summary>
    public abstract void SetInterval(double first, double last);

    /// <summary>The resolutions in the 2D subspace Index, for a 3D tolerance Tol.</summary>
    public virtual void Resolution(int index, double tol, out double tolU, out double tolV)
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::Resolution");
    }

    /// <summary>
    /// The tolerances of the approximation (one per pole) that satisfy the boundary error BoundTol, the tangent
    /// error AngleTol at the boundary and the error SurfTol inside the surface.
    /// </summary>
    public abstract void GetTolerance(double boundTol, double surfTol, double angleTol,
                                      NCollection_Array1<double> tol3d);

    /// <summary>Sets the tolerances of the numerical algorithms D0, D1 and D2 may run.</summary>
    public abstract void SetTolerance(double tol3d, double tol2d);

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
    public virtual void GetMinimalWeight(NCollection_Array1<double> weigths)
    {
        throw new MethodNotImplementedException("Approx_SweepFunction::GetMinimalWeight");
    }
}
