// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Boundary.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Boundary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The root of the boundaries of a contour around a gap to fill (GeomFill_ConstrainedFilling):
/// <see cref="SimpleBound"/> (a free curve), <see cref="BoundWithSurf"/> (a curve on a surface,
/// with the surface's normals as a tangency constraint) and <see cref="DegeneratedBound"/> (a point).
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Reparametrize"/> and the tolerance setters change it, and
/// GeomFill_ConstrainedFilling reparametrises the boundaries it is given, as OCCT does.
/// </remarks>
internal abstract class Boundary : Transient
{
    private double _t3d;
    private double _tang;

    protected Boundary(double tol3d, double tolang)
    {
        _t3d = tol3d;
        _tang = tolang;
    }

    public abstract Point3D Value(double u);

    public abstract void D1(double u, out Point3D p, out Vector3D v);

    /// <summary>Whether the boundary has normals (a tangency constraint); false by default.</summary>
    public virtual bool HasNormals => false;

    /// <summary>The normal at <paramref name="u"/>; throws Standard_Failure for a boundary without normals.</summary>
    public virtual Vector3D Norm(double u)
    {
        throw new FailureException("GeomFill_Boundary::Norm : Undefined normals");
    }

    /// <summary>The normal and its derivative at <paramref name="u"/>; throws Standard_Failure without normals.</summary>
    public virtual void D1Norm(double u, out Vector3D n, out Vector3D dn)
    {
        throw new FailureException("GeomFill_Boundary::Norm : Undefined normals");
    }

    /// <summary>
    /// Reparametrises the boundary on [First, Last], with the derivatives DF and DL at the ends if HasDF and HasDL,
    /// reversed if Rev.
    /// </summary>
    public abstract void Reparametrize(double first, double last, bool hasDF, bool hasDL, double df, double dl,
                                       bool rev);

    /// <summary>The points at the two bounds.</summary>
    public void Points(out Point3D pFirst, out Point3D pLast)
    {
        double f, l;
        Bounds(out f, out l);
        pFirst = Value(f);
        pLast = Value(l);
    }

    public abstract void Bounds(out double first, out double last);

    public abstract bool IsDegenerated { get; }

    public double Tol3d() => _t3d;

    public void Tol3d(double tol)
    {
        _t3d = tol;
    }

    public double Tolang() => _tang;

    public void Tolang(double tol)
    {
        _tang = tol;
    }
}
