// Port of: src/ModelingData/TKBRep/BRep/BRep_PointRepresentation.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointRepresentation.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointRepresentation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The root of a vertex's point representations (points on curves, on surfaces, on curves on surfaces): a location
/// and a parameter.
/// </summary>
/// <remarks>
/// The queries are public; the setters are internal and throw <see cref="Standard_ImmutableObject"/> once the
/// representation is frozen, with its vertex. The geometry it holds is frozen when it takes it.
/// </remarks>
public abstract class BRep_PointRepresentation : Standard_Transient
{
    private TopLoc_Location myLocation;
    private double myParameter;

    protected BRep_PointRepresentation(double P, in TopLoc_Location L)
    {
        myLocation = L;
        myParameter = P;
    }

    /// <summary>A point on a 3D curve.</summary>
    public virtual bool IsPointOnCurve() => false;

    /// <summary>A point on a 2D curve on a surface.</summary>
    public virtual bool IsPointOnCurveOnSurface() => false;

    /// <summary>A point on a surface.</summary>
    public virtual bool IsPointOnSurface() => false;

    /// <summary>A point on the curve <paramref name="C"/>.</summary>
    public virtual bool IsPointOnCurve(Geom_Curve? C, in TopLoc_Location L) => false;

    /// <summary>A point on the 2D curve <paramref name="PC"/> on the surface <paramref name="S"/>.</summary>
    public virtual bool IsPointOnCurveOnSurface(Geom2d_Curve? PC, Geom_Surface? S, in TopLoc_Location L) => false;

    /// <summary>A point on the surface <paramref name="S"/>.</summary>
    public virtual bool IsPointOnSurface(Geom_Surface? S, in TopLoc_Location L) => false;

    /// <summary>The location of the representation.</summary>
    public TopLoc_Location Location() => myLocation;

    internal void Location(in TopLoc_Location L)
    {
        CheckMutable();
        myLocation = L;
    }

    /// <summary>The parameter of the point.</summary>
    public double Parameter() => myParameter;

    internal void Parameter(double P)
    {
        CheckMutable();
        myParameter = P;
    }

    public virtual double Parameter2() => throw new Standard_DomainError("BRep_PointRepresentation");

    internal virtual void Parameter2(double P) => throw new Standard_DomainError("BRep_PointRepresentation");

    public virtual Geom_Curve? Curve() => throw new Standard_DomainError("BRep_PointRepresentation");

    internal virtual void Curve(Geom_Curve? C) => throw new Standard_DomainError("BRep_PointRepresentation");

    public virtual Geom2d_Curve? PCurve() => throw new Standard_DomainError("BRep_PointRepresentation");

    internal virtual void PCurve(Geom2d_Curve? C) => throw new Standard_DomainError("BRep_PointRepresentation");

    public virtual Geom_Surface? Surface() => throw new Standard_DomainError("BRep_PointRepresentation");

    internal virtual void Surface(Geom_Surface? S) => throw new Standard_DomainError("BRep_PointRepresentation");
}
