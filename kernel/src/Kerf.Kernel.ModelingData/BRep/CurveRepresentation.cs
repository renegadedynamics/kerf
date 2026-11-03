// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveRepresentation.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveRepresentation.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveRepresentation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Meshing;

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>
/// The root of an edge's curve representations (3D curves, curves on surfaces, continuities between surfaces and
/// polygons). Holds a location.
/// </summary>
/// <remarks>
/// The queries are public. The setters are internal to OCCT.NET and throw <see cref="ImmutableObjectException"/> once
/// the representation is frozen, which happens with its edge (docs/phase3-design.md, section 4). The geometry and
/// meshes a representation holds are frozen as soon as it takes them (section 4.1). OCCT's virtual accessors throw
/// <see cref="DomainErrorException"/> where a representation has no such data.
/// </remarks>
internal abstract class CurveRepresentation : Transient
{
    protected Placement _location;

    protected CurveRepresentation(in Placement l) => _location = l;

    /// <summary>A 3D curve representation.</summary>
    public virtual bool IsCurve3D => false;

    /// <summary>A curve in the parametric space of a surface.</summary>
    public virtual bool IsCurveOnSurface() => false;

    /// <summary>A continuity between two surfaces.</summary>
    public virtual bool IsRegularity() => false;

    /// <summary>A curve with two parametric curves on the same surface.</summary>
    public virtual bool IsCurveOnClosedSurface => false;

    /// <summary>A curve in the parametric space of <paramref name="s"/> with location <paramref name="l"/>.</summary>
    public virtual bool IsCurveOnSurface(Surface? s, in Placement l) => false;

    /// <summary>A regularity between <paramref name="s1"/> and <paramref name="s2"/> with locations L1 and L2.</summary>
    public virtual bool IsRegularity(Surface? s1, Surface? s2, in Placement l1, in Placement l2)
        => false;

    /// <summary>A 3D polygon representation.</summary>
    public virtual bool IsPolygon3D => false;

    /// <summary>A representation by an array of nodes on a triangulation.</summary>
    public virtual bool IsPolygonOnTriangulation() => false;

    /// <summary>A polygon in the definition of <paramref name="t"/> with location <paramref name="l"/>.</summary>
    public virtual bool IsPolygonOnTriangulation(Triangulation? t, in Placement l) => false;

    /// <summary>A representation by two arrays of nodes on a triangulation.</summary>
    public virtual bool IsPolygonOnClosedTriangulation => false;

    /// <summary>A polygon in the parametric space of a surface.</summary>
    public virtual bool IsPolygonOnSurface() => false;

    /// <summary>A polygon in the parametric space of <paramref name="s"/> with location <paramref name="l"/>.</summary>
    public virtual bool IsPolygonOnSurface(Surface? s, in Placement l) => false;

    /// <summary>Two 2D polygons in the parametric space of a surface.</summary>
    public virtual bool IsPolygonOnClosedSurface => false;

    /// <summary>The location of the representation.</summary>
    public Placement Location() => _location;

    /// <summary>
    /// Freezes the meshes the representation holds (OCCT.NET). Representations freeze what they take when they take
    /// it, except the mesher's, which attaches its triangulations and polygons unfrozen (decision 488); freezing the
    /// edge's graph (<c>BRep_TEdge.FreezeRepresentations</c>) freezes them through this.
    /// </summary>
    internal virtual void FreezeContents()
    {
    }

    internal void Location(in Placement l)
    {
        CheckMutable();
        _location = l;
    }

    public virtual Curve3D? Curve3D() => throw new DomainErrorException("BRep_CurveRepresentation");

    internal virtual void Curve3D(Curve3D? c) => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Surface? Surface() => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Curve2D? PCurve() => throw new DomainErrorException("BRep_CurveRepresentation");

    internal virtual void PCurve(Curve2D? c) => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Curve2D? PCurve2() => throw new DomainErrorException("BRep_CurveRepresentation");

    internal virtual void PCurve2(Curve2D? c) => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Polygon3D? Polygon3D() => throw new DomainErrorException("BRep_CurveRepresentation");

    internal virtual void Polygon3D(Polygon3D? p) => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Polygon2D? Polygon() => throw new DomainErrorException("BRep_CurveRepresentation");

    internal virtual void Polygon(Polygon2D? p) => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Polygon2D? Polygon2() => throw new DomainErrorException("BRep_CurveRepresentation");

    internal virtual void Polygon2(Polygon2D? p) => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Triangulation? Triangulation() => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual PolygonOnTriangulation? PolygonOnTriangulation()
        => throw new DomainErrorException("BRep_CurveRepresentation");

    internal virtual void PolygonOnTriangulation(PolygonOnTriangulation? p)
        => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual PolygonOnTriangulation? PolygonOnTriangulation2()
        => throw new DomainErrorException("BRep_CurveRepresentation");

    internal virtual void PolygonOnTriangulation2(PolygonOnTriangulation? p2)
        => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Surface? Surface2() => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Placement Location2() => throw new DomainErrorException("BRep_CurveRepresentation");

    public virtual Continuity Continuity() => throw new DomainErrorException("BRep_CurveRepresentation");

    internal virtual void Continuity(Continuity c) => throw new DomainErrorException("BRep_CurveRepresentation");

    /// <summary>A copy of this representation: new and unfrozen, sharing the (frozen) geometry.</summary>
    public abstract CurveRepresentation Copy();
}
