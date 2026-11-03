// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnClosedSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnClosedSurface.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnClosedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of an edge by two pcurves on a closed surface (a seam).</summary>
public class CurveOnClosedSurface : CurveOnSurfaceRepresentation
{
    private Curve2D? _pCurve2;
    private Continuity _continuity;
    private Point2D _uv21;
    private Point2D _uv22;

    /// <summary>The pcurves <paramref name="pc1"/> and <paramref name="pc2"/> on <paramref name="s"/> (all frozen here).</summary>
    public CurveOnClosedSurface(Curve2D pc1,
                                     Curve2D? pc2,
                                     Surface? s,
                                     in Placement l,
                                     Continuity c)
        : base(pc1, s, l)
    {
        pc2?.Freeze(); // OCCT.NET: attached geometry is frozen
        _pCurve2 = pc2;
        _continuity = c;
        _uv21 = new Point2D();
        _uv22 = new Point2D();
    }

    internal void SetUVPoints2(in Point2D p1, in Point2D p2)
    {
        CheckMutable();
        _uv21 = p1;
        _uv22 = p2;
    }

    /// <summary>The UV points of the second pcurve at the ends of the range.</summary>
    public void UVPoints2(out Point2D p1, out Point2D p2)
    {
        p1 = _uv21;
        p2 = _uv22;
    }

    /// <summary>True.</summary>
    public override bool IsCurveOnClosedSurface => true;

    /// <summary>True.</summary>
    public override bool IsRegularity() => true;

    /// <summary>A curve on two surfaces (continuity).</summary>
    public override bool IsRegularity(Surface? s1, Surface? s2, in Placement l1, in Placement l2)
        => ReferenceEquals(Surface(), s1) && ReferenceEquals(Surface(), s2) && (Location() == l1) && (Location() == l2);

    public override Curve2D? PCurve2() => _pCurve2;

    /// <summary>Surface().</summary>
    public override Surface? Surface2() => Surface();

    /// <summary>Location().</summary>
    public override Placement Location2() => Location();

    public override Continuity Continuity() => _continuity;

    internal override void Continuity(Continuity c)
    {
        CheckMutable();
        _continuity = c;
    }

    internal override void PCurve2(Curve2D? c)
    {
        CheckMutable();
        c?.Freeze(); // OCCT.NET: attached geometry is frozen
        _pCurve2 = c;
    }

    /// <summary>A copy of this representation.</summary>
    public override CurveRepresentation Copy()
    {
        CurveOnClosedSurface c =
            new CurveOnClosedSurface(PCurve()!, PCurve2(), Surface(), Location(), _continuity);

        c.SetRange(First(), Last());
        c.SetUVPoints(_uv1, _uv2);
        c.SetUVPoints2(_uv21, _uv22);

        return c;
    }

    /// <summary>Recomputes the UV points of both pcurves after the range changed.</summary>
    internal override void Update()
    {
        CheckMutable();
        if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(First()))
        {
            _pCurve2!.D0(First(), out _uv21);
        }

        if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(Last()))
        {
            _pCurve2!.D0(Last(), out _uv22);
        }

        base.Update();
    }
}
