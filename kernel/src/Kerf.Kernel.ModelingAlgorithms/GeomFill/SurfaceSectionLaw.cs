// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionLaw.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>A section law for sweeping: the poles and weights of the section as functions of the parameter v.</summary>
/// <remarks>A working object (decision 21), as <see cref="SurfaceLocationLaw"/>.</remarks>
internal abstract class SurfaceSectionLaw : Transient
{
    /// <summary>Computes the section for v = Param.</summary>
    public abstract bool D0(double param, NCollection_Array1<Point3D> poles, NCollection_Array1<double> weigths);

    /// <summary>Computes the section and its first derivative in v (used only for C1 or C2 approximation).</summary>
    public virtual bool D1(double param, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                           NCollection_Array1<double> weigths, NCollection_Array1<double> dWeigths)
    {
        throw new MethodNotImplementedException("GeomFill_SectionLaw::D1");
    }

    /// <summary>Computes the section and its first and second derivatives in v (used only for C2 approximation).</summary>
    public virtual bool D2(double param, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                           NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<double> weigths,
                           NCollection_Array1<double> dWeigths, NCollection_Array1<double> d2Weigths)
    {
        throw new MethodNotImplementedException("GeomFill_SectionLaw::D2");
    }

    /// <summary>A B-spline surface whose v-isos are the sections if there is one, else null (the default).</summary>
    public virtual BSplineSurface? BSplineSurface() => null;

    /// <summary>The format of a section.</summary>
    public abstract void SectionShape(out int nbPoles, out int nbKnots, out int degree);

    /// <summary>The knots of the section.</summary>
    public abstract void Knots(NCollection_Array1<double> tKnots);

    /// <summary>The multiplicities of the section.</summary>
    public abstract void Mults(NCollection_Array1<int> tMults);

    /// <summary>Whether the sections are rational.</summary>
    public abstract bool IsRational { get; }

    /// <summary>Whether the sections are periodic.</summary>
    public abstract bool IsUPeriodic { get; }

    /// <summary>Whether the law is periodic.</summary>
    public abstract bool IsVPeriodic { get; }

    /// <summary>The number of intervals for continuity S; may be one if the law is at least S.</summary>
    public abstract int NbIntervals(Continuity s);

    /// <summary>Stores in T the parameters bounding the intervals of continuity S; T.Length() must exceed NbIntervals(S).</summary>
    public abstract void Intervals(NCollection_Array1<double> t, Continuity s);

    /// <summary>Sets the bounds of the parametric interval, which determines the derivatives there if the law is not Cn.</summary>
    public abstract void SetInterval(double first, double last);

    /// <summary>The bounds of the parametric interval.</summary>
    public abstract void GetInterval(out double first, out double last);

    /// <summary>The bounds of the parametric domain of the law (not changed by SetInterval).</summary>
    public abstract void GetDomain(out double first, out double last);

    /// <summary>
    /// The tolerances of each pole to reach in approximation, for BoundTol at the boundary, AngleTol (radians) on
    /// the tangent at the boundary and SurfTol inside the surface.
    /// </summary>
    public abstract void GetTolerance(double boundTol, double surfTol, double angleTol, NCollection_Array1<double> tol3d);

    /// <summary>Sets the tolerances of a law that runs numerical algorithms; does nothing by default.</summary>
    public virtual void SetTolerance(double tol3d, double tol2d)
    {
        // Ne fait Rien
    }

    /// <summary>A rough barycentre of the surface (used only if the law is rational).</summary>
    public virtual Point3D BarycentreOfSurf()
    {
        throw new MethodNotImplementedException("GeomFill_SectionLaw::BarycentreOfSurf");
    }

    /// <summary>The length of the greatest section, for G1 control.</summary>
    public abstract double MaximalSection();

    /// <summary>The minimal weight of each pole over all sections (used only if the law is rational).</summary>
    public virtual void GetMinimalWeight(NCollection_Array1<double> weigths)
    {
        throw new MethodNotImplementedException("GeomFill_SectionLaw::GetMinimalWeight");
    }

    /// <summary>Whether all sections are equal.</summary>
    public virtual bool IsConstant(ref double error)
    {
        error = 0.0;
        return false;
    }

    /// <summary>A copy of the constant section, if the law is constant.</summary>
    public virtual Curve3D ConstantSection()
    {
        throw new DomainErrorException("GeomFill_SectionLaw::ConstantSection");
    }

    /// <summary>Whether all sections are circles in one plane with one centre and a linear radius; false by default.</summary>
    public virtual bool IsConicalLaw(ref double error)
    {
        error = 0.0;
        return false;
    }

    /// <summary>The circle section at Param, if the law is conical.</summary>
    public virtual Curve3D CirclSection(double param)
    {
        throw new DomainErrorException("GeomFill_SectionLaw::CirclSection");
    }
}
