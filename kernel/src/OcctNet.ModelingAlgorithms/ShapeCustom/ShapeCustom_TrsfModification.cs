// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_TrsfModification.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_TrsfModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// Complements <see cref="BRepTools_TrsfModification"/> to give reversible scaling of tolerances: it uses the
/// tolerances stored in the shapes (not those <see cref="BRep_Tool"/> returns, which are at least
/// Precision::Confusion()), multiplied by the scale factor.
/// </summary>
public class ShapeCustom_TrsfModification : BRepTools_TrsfModification
{
    /// <summary>The modification applying <paramref name="T"/>.</summary>
    public ShapeCustom_TrsfModification(in Transform3D T)
        : base(T)
    {
    }

    /// <summary>The inherited method; Tol is the face's own tolerance times the scale factor.</summary>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        bool result = base.NewSurface(F, out S, out L, out Tol, out RevWires, out RevFace);
        Tol = ((BRep_TFace)F.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the edge's own tolerance times the scale factor.</summary>
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out Placement L, out double Tol)
    {
        bool result = base.NewCurve(E, out C, out L, out Tol);
        Tol = ((BRep_TEdge)E.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the vertex's own tolerance times the scale factor.</summary>
    public override bool NewPoint(in TopoDS_Vertex V, out Point3D P, out double Tol)
    {
        bool result = base.NewPoint(V, out P, out Tol);
        Tol = ((BRep_TVertex)V.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the edge's own tolerance times the scale factor.</summary>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol)
    {
        bool result = base.NewCurve2d(E, F, NewE, NewF, out C, out Tol);
        Tol = ((BRep_TEdge)E.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the vertex's own tolerance times the scale factor.</summary>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        bool result = base.NewParameter(V, E, ref P, ref Tol);
        Tol = ((BRep_TVertex)V.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }
}
