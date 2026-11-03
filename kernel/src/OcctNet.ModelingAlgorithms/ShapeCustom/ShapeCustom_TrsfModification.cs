// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_TrsfModification.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_TrsfModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// Complements <see cref="TrsfModification"/> to give reversible scaling of tolerances: it uses the
/// tolerances stored in the shapes (not those <see cref="Tool"/> returns, which are at least
/// Precision::Confusion()), multiplied by the scale factor.
/// </summary>
public class ShapeCustom_TrsfModification : TrsfModification
{
    /// <summary>The modification applying <paramref name="T"/>.</summary>
    public ShapeCustom_TrsfModification(in Transform3D T)
        : base(T)
    {
    }

    /// <summary>The inherited method; Tol is the face's own tolerance times the scale factor.</summary>
    public override bool NewSurface(in Face F,
                                    out Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        bool result = base.NewSurface(F, out S, out L, out Tol, out RevWires, out RevFace);
        Tol = ((BrepTFace)F.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the edge's own tolerance times the scale factor.</summary>
    public override bool NewCurve(in Edge E, out Curve3D? C, out Placement L, out double Tol)
    {
        bool result = base.NewCurve(E, out C, out L, out Tol);
        Tol = ((BrepTEdge)E.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the vertex's own tolerance times the scale factor.</summary>
    public override bool NewPoint(in Vertex V, out Point3D P, out double Tol)
    {
        bool result = base.NewPoint(V, out P, out Tol);
        Tol = ((BrepTVertex)V.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the edge's own tolerance times the scale factor.</summary>
    public override bool NewCurve2d(in Edge E,
                                    in Face F,
                                    in Edge NewE,
                                    in Face NewF,
                                    out Curve2D? C,
                                    out double Tol)
    {
        bool result = base.NewCurve2d(E, F, NewE, NewF, out C, out Tol);
        Tol = ((BrepTEdge)E.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }

    /// <summary>The inherited method; Tol is the vertex's own tolerance times the scale factor.</summary>
    public override bool NewParameter(in Vertex V, in Edge E, ref double P, ref double Tol)
    {
        bool result = base.NewParameter(V, E, ref P, ref Tol);
        Tol = ((BrepTVertex)V.TShape()!).Tolerance() * Math.Abs(Trsf().ScaleFactor());
        return result;
    }
}
