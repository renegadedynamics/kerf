// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_Vertex.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_Vertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Healing.Build;

/// <summary>Builds vertices: the combination of two vertices into one covering both.</summary>
/// <remarks>The vertices returned are frozen; ported code that goes on writing into them uses the Internal forms.</remarks>
public sealed class VertexBuilder
{
    /// <summary>
    /// A new vertex whose tolerance sphere covers those of <paramref name="v1"/> and <paramref name="v2"/>, its
    /// tolerance multiplied by <paramref name="tolFactor"/>. Frozen.
    /// </summary>
    public Vertex CombineVertex(in Vertex v1, in Vertex v2, double tolFactor = 1.0001)
    {
        return Frozen(InternalCombineVertex(v1, v2, tolFactor));
    }

    /// <summary>As the vertex form, from the points and tolerances. Frozen.</summary>
    public Vertex CombineVertex(in Point3D pnt1, in Point3D pnt2, double tol1, double tol2, double tolFactor = 1.0001)
    {
        return Frozen(InternalCombineVertex(pnt1, pnt2, tol1, tol2, tolFactor));
    }

    private static Vertex Frozen(Vertex v)
    {
        v.TShape()!.FreezeGraph();
        return v;
    }

    /// <summary>OCCT's CombineVertex: the new vertex, not frozen.</summary>
    internal Vertex InternalCombineVertex(in Vertex v1, in Vertex v2, double tolFactor = 1.0001)
    {
        return InternalCombineVertex(Tool.Pnt(v1), Tool.Pnt(v2), Tool.Tolerance(v1), Tool.Tolerance(v2), tolFactor);
    }

    /// <summary>OCCT's CombineVertex: the new vertex, not frozen.</summary>
    internal Vertex InternalCombineVertex(in Point3D pnt1, in Point3D pnt2, double tol1, double tol2, double tolFactor = 1.0001)
    {
        Point3D pos;
        double tol;

        Vector3D v = new Vector3D(pnt2.XYZ() - pnt1.XYZ());
        double dist = v.Magnitude;

        // #47 rln 09.12.98 S4054 PRO14323 entity 2844
        if (dist + tol2 <= tol1)
        {
            pos = pnt1;
            tol = tol1;
        }
        else if (dist + tol1 <= tol2)
        {
            pos = pnt2;
            tol = tol2;
        }
        else
        {
            tol = 0.5 * (dist + tol1 + tol2);
            double s = (dist > 0.0) ? (tol2 - tol1) / dist : 0.0; // szv#4:S4163:12Mar99 anti-exception
            pos = new Point3D(0.5 * ((1 - s) * pnt1.XYZ() + (1 + s) * pnt2.XYZ()));
        }

        Vertex V = new Vertex();
        var b = new BrepBuilder();
        b.MakeVertex(ref V, pos, tolFactor * tol);
        return V;
    }
}
