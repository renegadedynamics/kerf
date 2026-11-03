// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_TrsfModification.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_TrsfModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Healing.Custom;

/// <summary>
/// Complements <see cref="TrsfModification"/> to give reversible scaling of tolerances: it uses the
/// tolerances stored in the shapes (not those <see cref="Tool"/> returns, which are at least
/// Precision::Confusion()), multiplied by the scale factor.
/// </summary>
public class CustomTrsfModification : TrsfModification
{
    /// <summary>The modification applying <paramref name="t"/>.</summary>
    public CustomTrsfModification(in Transform3D t)
        : base(t)
    {
    }

    /// <summary>The inherited method; Tol is the face's own tolerance times the scale factor.</summary>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        bool result = base.NewSurface(f, out s, out l, out tol, out revWires, out revFace);
        tol = ((BrepTFace)f.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the edge's own tolerance times the scale factor.</summary>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement l, out double tol)
    {
        bool result = base.NewCurve(e, out c, out l, out tol);
        tol = ((BrepTEdge)e.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the vertex's own tolerance times the scale factor.</summary>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        bool result = base.NewPoint(v, out p, out tol);
        tol = ((BrepTVertex)v.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the edge's own tolerance times the scale factor.</summary>
    public override bool NewCurve2d(in Edge e,
                                    in Face f,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol)
    {
        bool result = base.NewCurve2d(e, f, newE, newF, out c, out tol);
        tol = ((BrepTEdge)e.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the vertex's own tolerance times the scale factor.</summary>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        bool result = base.NewParameter(v, e, ref p, ref tol);
        tol = ((BrepTVertex)v.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }
}
