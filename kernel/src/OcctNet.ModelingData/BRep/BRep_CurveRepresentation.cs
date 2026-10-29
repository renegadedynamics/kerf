// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveRepresentation.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveRepresentation.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveRepresentation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The root of an edge's curve representations (3D curves, curves on surfaces, continuities between surfaces and
/// polygons). Holds a location.
/// </summary>
/// <remarks>
/// The queries are public. The setters are internal to OCCT.NET and throw <see cref="Standard_ImmutableObject"/> once
/// the representation is frozen, which happens with its edge (docs/phase3-design.md, section 4). The geometry and
/// meshes a representation holds are frozen as soon as it takes them (section 4.1). OCCT's virtual accessors throw
/// <see cref="Standard_DomainError"/> where a representation has no such data.
/// </remarks>
public abstract class BRep_CurveRepresentation : Standard_Transient
{
    protected TopLoc_Location myLocation;

    protected BRep_CurveRepresentation(in TopLoc_Location L) => myLocation = L;

    /// <summary>A 3D curve representation.</summary>
    public virtual bool IsCurve3D() => false;

    /// <summary>A curve in the parametric space of a surface.</summary>
    public virtual bool IsCurveOnSurface() => false;

    /// <summary>A continuity between two surfaces.</summary>
    public virtual bool IsRegularity() => false;

    /// <summary>A curve with two parametric curves on the same surface.</summary>
    public virtual bool IsCurveOnClosedSurface() => false;

    /// <summary>A curve in the parametric space of <paramref name="S"/> with location <paramref name="L"/>.</summary>
    public virtual bool IsCurveOnSurface(Geom_Surface? S, in TopLoc_Location L) => false;

    /// <summary>A regularity between <paramref name="S1"/> and <paramref name="S2"/> with locations L1 and L2.</summary>
    public virtual bool IsRegularity(Geom_Surface? S1, Geom_Surface? S2, in TopLoc_Location L1, in TopLoc_Location L2)
        => false;

    /// <summary>A 3D polygon representation.</summary>
    public virtual bool IsPolygon3D() => false;

    /// <summary>A representation by an array of nodes on a triangulation.</summary>
    public virtual bool IsPolygonOnTriangulation() => false;

    /// <summary>A polygon in the definition of <paramref name="T"/> with location <paramref name="L"/>.</summary>
    public virtual bool IsPolygonOnTriangulation(Poly_Triangulation? T, in TopLoc_Location L) => false;

    /// <summary>A representation by two arrays of nodes on a triangulation.</summary>
    public virtual bool IsPolygonOnClosedTriangulation() => false;

    /// <summary>A polygon in the parametric space of a surface.</summary>
    public virtual bool IsPolygonOnSurface() => false;

    /// <summary>A polygon in the parametric space of <paramref name="S"/> with location <paramref name="L"/>.</summary>
    public virtual bool IsPolygonOnSurface(Geom_Surface? S, in TopLoc_Location L) => false;

    /// <summary>Two 2D polygons in the parametric space of a surface.</summary>
    public virtual bool IsPolygonOnClosedSurface() => false;

    /// <summary>The location of the representation.</summary>
    public TopLoc_Location Location() => myLocation;

    internal void Location(in TopLoc_Location L)
    {
        CheckMutable();
        myLocation = L;
    }

    public virtual Geom_Curve? Curve3D() => throw new Standard_DomainError("BRep_CurveRepresentation");

    internal virtual void Curve3D(Geom_Curve? C) => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Geom_Surface? Surface() => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Geom2d_Curve? PCurve() => throw new Standard_DomainError("BRep_CurveRepresentation");

    internal virtual void PCurve(Geom2d_Curve? C) => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Geom2d_Curve? PCurve2() => throw new Standard_DomainError("BRep_CurveRepresentation");

    internal virtual void PCurve2(Geom2d_Curve? C) => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Poly_Polygon3D? Polygon3D() => throw new Standard_DomainError("BRep_CurveRepresentation");

    internal virtual void Polygon3D(Poly_Polygon3D? P) => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Poly_Polygon2D? Polygon() => throw new Standard_DomainError("BRep_CurveRepresentation");

    internal virtual void Polygon(Poly_Polygon2D? P) => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Poly_Polygon2D? Polygon2() => throw new Standard_DomainError("BRep_CurveRepresentation");

    internal virtual void Polygon2(Poly_Polygon2D? P) => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Poly_Triangulation? Triangulation() => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Poly_PolygonOnTriangulation? PolygonOnTriangulation()
        => throw new Standard_DomainError("BRep_CurveRepresentation");

    internal virtual void PolygonOnTriangulation(Poly_PolygonOnTriangulation? P)
        => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Poly_PolygonOnTriangulation? PolygonOnTriangulation2()
        => throw new Standard_DomainError("BRep_CurveRepresentation");

    internal virtual void PolygonOnTriangulation2(Poly_PolygonOnTriangulation? P2)
        => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual Geom_Surface? Surface2() => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual TopLoc_Location Location2() => throw new Standard_DomainError("BRep_CurveRepresentation");

    public virtual GeomAbs_Shape Continuity() => throw new Standard_DomainError("BRep_CurveRepresentation");

    internal virtual void Continuity(GeomAbs_Shape C) => throw new Standard_DomainError("BRep_CurveRepresentation");

    /// <summary>A copy of this representation: new and unfrozen, sharing the (frozen) geometry.</summary>
    public abstract BRep_CurveRepresentation Copy();
}
