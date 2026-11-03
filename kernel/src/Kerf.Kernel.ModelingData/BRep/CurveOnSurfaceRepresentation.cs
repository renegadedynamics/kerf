// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnSurface.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of an edge by a curve in the parametric space of a surface.</summary>
internal class CurveOnSurfaceRepresentation : GCurve
{
    protected Point2D _uv1;
    protected Point2D _uv2;
    private Curve2D? _pCurve;
    private Surface? _surface;

    /// <summary>The pcurve <paramref name="pc"/> on <paramref name="s"/> with location <paramref name="l"/> (both frozen here).</summary>
    public CurveOnSurfaceRepresentation(Curve2D pc, Surface? s, in Placement l)
        : base(l, pc.FirstParameter, pc.LastParameter)
    {
        pc.Freeze(); // OCCT.NET: attached geometry is frozen
        s?.Freeze();
        _uv1 = new Point2D();
        _uv2 = new Point2D();
        _pCurve = pc;
        _surface = s;
    }

    /// <summary>
    /// OCCT.NET: puts the pcurve on <paramref name="s"/>, which stands for its surface changed in place by OCCT (the
    /// offset surface whose basis BRepOffset_Tool's EnlargeGeometry enlarges, decision 1101).
    /// </summary>
    internal void ReplaceSurface(Surface s)
    {
        CheckMutable();
        s.Freeze();
        _surface = s;
    }

    internal void SetUVPoints(in Point2D p1, in Point2D p2)
    {
        CheckMutable();
        _uv1 = p1;
        _uv2 = p2;
    }

    /// <summary>The UV points at the ends of the range.</summary>
    public void UVPoints(out Point2D p1, out Point2D p2)
    {
        p1 = _uv1;
        p2 = _uv2;
    }

    /// <summary>The point at parameter <paramref name="u"/>.</summary>
    public override void D0(double u, out Point3D p)
    {
        // should be D0 NYI
        Point2D p2d = _pCurve!.Value(u);
        p = _surface!.Value(p2d.X, p2d.Y);
        p = p.Transformed(_location.Transformation);
    }

    /// <summary>True.</summary>
    public override bool IsCurveOnSurface() => true;

    /// <summary>A curve in the parametric space of <paramref name="s"/> with location <paramref name="l"/>.</summary>
    public override bool IsCurveOnSurface(Surface? s, in Placement l)
        => ReferenceEquals(s, _surface) && (l.IsEqual(_location));

    public override Surface? Surface() => _surface;

    public override Curve2D? PCurve() => _pCurve;

    internal override void PCurve(Curve2D? c)
    {
        CheckMutable();
        c?.Freeze(); // OCCT.NET: attached geometry is frozen
        _pCurve = c;
    }

    /// <summary>A copy of this representation.</summary>
    public override CurveRepresentation Copy()
    {
        CurveOnSurfaceRepresentation c = new CurveOnSurfaceRepresentation(_pCurve!, _surface, Location());

        c.SetRange(First(), Last());
        c.SetUVPoints(_uv1, _uv2);

        return c;
    }

    /// <summary>Recomputes the UV points after the range changed.</summary>
    internal override void Update()
    {
        CheckMutable();
        double f = First();
        double l = Last();
        bool isneg = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f);
        bool ispos = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l);
        if (!isneg)
        {
            _pCurve!.D0(f, out _uv1);
        }

        if (!ispos)
        {
            _pCurve!.D0(l, out _uv2);
        }
    }
}
