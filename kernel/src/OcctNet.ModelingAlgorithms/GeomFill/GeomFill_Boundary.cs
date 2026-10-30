// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Boundary.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Boundary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The root of the boundaries of a contour around a gap to fill (GeomFill_ConstrainedFilling):
/// <see cref="GeomFill_SimpleBound"/> (a free curve), <see cref="GeomFill_BoundWithSurf"/> (a curve on a surface,
/// with the surface's normals as a tangency constraint) and <see cref="GeomFill_DegeneratedBound"/> (a point).
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Reparametrize"/> and the tolerance setters change it, and
/// GeomFill_ConstrainedFilling reparametrises the boundaries it is given, as OCCT does.
/// </remarks>
public abstract class GeomFill_Boundary : Standard_Transient
{
    private double myT3d;
    private double myTang;

    protected GeomFill_Boundary(double Tol3d, double Tolang)
    {
        myT3d = Tol3d;
        myTang = Tolang;
    }

    public abstract gp_Pnt Value(double U);

    public abstract void D1(double U, out gp_Pnt P, out gp_Vec V);

    /// <summary>Whether the boundary has normals (a tangency constraint); false by default.</summary>
    public virtual bool HasNormals() => false;

    /// <summary>The normal at <paramref name="U"/>; throws Standard_Failure for a boundary without normals.</summary>
    public virtual gp_Vec Norm(double U)
    {
        throw new Standard_Failure("GeomFill_Boundary::Norm : Undefined normals");
    }

    /// <summary>The normal and its derivative at <paramref name="U"/>; throws Standard_Failure without normals.</summary>
    public virtual void D1Norm(double U, out gp_Vec N, out gp_Vec DN)
    {
        throw new Standard_Failure("GeomFill_Boundary::Norm : Undefined normals");
    }

    /// <summary>
    /// Reparametrises the boundary on [First, Last], with the derivatives DF and DL at the ends if HasDF and HasDL,
    /// reversed if Rev.
    /// </summary>
    public abstract void Reparametrize(double First, double Last, bool HasDF, bool HasDL, double DF, double DL,
                                       bool Rev);

    /// <summary>The points at the two bounds.</summary>
    public void Points(out gp_Pnt PFirst, out gp_Pnt PLast)
    {
        double f, l;
        Bounds(out f, out l);
        PFirst = Value(f);
        PLast = Value(l);
    }

    public abstract void Bounds(out double First, out double Last);

    public abstract bool IsDegenerated();

    public double Tol3d() => myT3d;

    public void Tol3d(double Tol)
    {
        myT3d = Tol;
    }

    public double Tolang() => myTang;

    public void Tolang(double Tol)
    {
        myTang = Tol;
    }
}
