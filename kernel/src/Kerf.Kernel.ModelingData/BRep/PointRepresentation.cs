// Port of: src/ModelingData/TKBRep/BRep/BRep_PointRepresentation.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointRepresentation.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointRepresentation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Topology.Brep;

/// <summary>
/// The root of a vertex's point representations (points on curves, on surfaces, on curves on surfaces): a location
/// and a parameter.
/// </summary>
/// <remarks>
/// The queries are public; the setters are internal and throw <see cref="ImmutableObjectException"/> once the
/// representation is frozen, with its vertex. The geometry it holds is frozen when it takes it.
/// </remarks>
internal abstract class PointRepresentation : Transient
{
    private Placement _location;
    private double _parameter;

    protected PointRepresentation(double p, in Placement l)
    {
        _location = l;
        _parameter = p;
    }

    /// <summary>A point on a 3D curve.</summary>
    public virtual bool IsPointOnCurve() => false;

    /// <summary>A point on a 2D curve on a surface.</summary>
    public virtual bool IsPointOnCurveOnSurface() => false;

    /// <summary>A point on a surface.</summary>
    public virtual bool IsPointOnSurface() => false;

    /// <summary>A point on the curve <paramref name="c"/>.</summary>
    public virtual bool IsPointOnCurve(Curve3D? c, in Placement l) => false;

    /// <summary>A point on the 2D curve <paramref name="pc"/> on the surface <paramref name="s"/>.</summary>
    public virtual bool IsPointOnCurveOnSurface(Curve2D? pc, Surface? s, in Placement l) => false;

    /// <summary>A point on the surface <paramref name="s"/>.</summary>
    public virtual bool IsPointOnSurface(Surface? s, in Placement l) => false;

    /// <summary>The location of the representation.</summary>
    public Placement Location() => _location;

    internal void Location(in Placement l)
    {
        CheckMutable();
        _location = l;
    }

    /// <summary>The parameter of the point.</summary>
    public double Parameter() => _parameter;

    internal void Parameter(double p)
    {
        CheckMutable();
        _parameter = p;
    }

    public virtual double Parameter2() => throw new DomainErrorException("BRep_PointRepresentation");

    internal virtual void Parameter2(double p) => throw new DomainErrorException("BRep_PointRepresentation");

    public virtual Curve3D? Curve() => throw new DomainErrorException("BRep_PointRepresentation");

    internal virtual void Curve(Curve3D? c) => throw new DomainErrorException("BRep_PointRepresentation");

    public virtual Curve2D? PCurve() => throw new DomainErrorException("BRep_PointRepresentation");

    internal virtual void PCurve(Curve2D? c) => throw new DomainErrorException("BRep_PointRepresentation");

    public virtual Surface? Surface() => throw new DomainErrorException("BRep_PointRepresentation");

    internal virtual void Surface(Surface? s) => throw new DomainErrorException("BRep_PointRepresentation");
}
