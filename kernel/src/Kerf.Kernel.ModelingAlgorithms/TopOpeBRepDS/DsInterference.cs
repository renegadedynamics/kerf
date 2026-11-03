// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Interference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Interference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>
/// The attachment of a new geometry on a geometry or shape of the legacy data structure: for example an
/// intersection point on an edge or on a curve.
/// </summary>
/// <remarks>
/// <para>
/// It holds the <see cref="Transition()"/> (how the interference separates the geometry it lies on into IN and OUT),
/// the kind and index of its support (FACE, EDGE, VERTEX, SURFACE, CURVE) and the kind and index of its geometry
/// (SURFACE, CURVE, POINT, or a shape).
/// </para>
/// <para>
/// A working object of the fillet builders and the data structure: the setters are internal. The same interference
/// is often held by several lists of the data structure (a shape's and a curve's), so a change is seen by all.
/// </para>
/// </remarks>
public class DsInterference : Transient
{
    private DsTransition _transition;
    private int _support;
    private int _geometry;
    private DsKind _supportType;
    private DsKind _geometryType;

    /// <summary>An empty interference.</summary>
    /// <remarks>OCCT leaves the indices and kinds unset; the port's are 0 and POINT (C#'s zero).</remarks>
    public DsInterference()
    {
        _transition = new DsTransition();
    }

    /// <summary>
    /// The interference of the geometry <paramref name="g"/> of kind <paramref name="gt"/> on the support
    /// <paramref name="s"/> of kind <paramref name="st"/>, with the transition <paramref name="t"/>.
    /// </summary>
    public DsInterference(in DsTransition t,
                                     DsKind st,
                                     int s,
                                     DsKind gt,
                                     int g)
    {
        _transition = t;
        _support = s;
        _geometry = g;
        _supportType = st;
        _geometryType = gt;
    }

    /// <summary>A copy of the transition, support and geometry of <paramref name="i"/>.</summary>
    public DsInterference(DsInterference i)
    {
        _transition = i.Transition();
        _support = i.Support();
        _geometry = i.Geometry();
        _supportType = i.SupportType();
        _geometryType = i.GeometryType();
    }

    /// <summary>The transition.</summary>
    public DsTransition Transition() => _transition;

    internal ref DsTransition ChangeTransition() => ref _transition;

    internal void Transition(in DsTransition t)
    {
        _transition = t;
    }

    /// <summary>The geometry's kind and index, then the support's.</summary>
    public void GKGSKS(out DsKind gk, out int g, out DsKind sk, out int s)
    {
        gk = _geometryType;
        g = _geometry;
        sk = _supportType;
        s = _support;
    }

    /// <summary>The kind of the support.</summary>
    public DsKind SupportType() => _supportType;

    /// <summary>The index of the support in the data structure.</summary>
    public int Support() => _support;

    /// <summary>The kind of the geometry.</summary>
    public DsKind GeometryType() => _geometryType;

    /// <summary>The index of the geometry in the data structure.</summary>
    public int Geometry() => _geometry;

    internal void SetGeometry(int gi)
    {
        _geometry = gi;
    }

    internal void SupportType(DsKind st)
    {
        _supportType = st;
    }

    internal void Support(int s)
    {
        _support = s;
    }

    internal void GeometryType(DsKind gt)
    {
        _geometryType = gt;
    }

    internal void Geometry(int g)
    {
        _geometry = g;
    }

    /// <summary>Whether <paramref name="i"/> has the same support kind and index.</summary>
    public bool HasSameSupport(DsInterference i)
        => (_supportType == i._supportType && _support == i._support);

    /// <summary>Whether <paramref name="i"/> has the same geometry kind and index.</summary>
    public bool HasSameGeometry(DsInterference i)
        => (_geometryType == i._geometryType && _geometry == i._geometry);
}
