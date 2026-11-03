// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ToolTriangulatedShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ToolTriangulatedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Meshing;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Library;

/// <summary>Computes the nodal normals of a face's triangulation.</summary>
/// <remarks>
/// A triangulation attached to a face is frozen (design 4.1): <see cref="ComputeNormals(in Face, Triangulation?)"/>
/// writes into the triangulation it is given, as OCCT does, and throws <see cref="ImmutableObjectException"/> on a
/// frozen one without normals; <see cref="NormalsComputed"/> returns a copy with normals instead.
/// </remarks>
public static class ToolTriangulatedShape
{
    /// <summary>
    /// Computes the nodal normals of <paramref name="tris"/> from the face's surface at the UV nodes (or, without
    /// UV nodes or surface, by averaging the triangles' normals). Does nothing if it already has normals.
    /// </summary>
    public static void ComputeNormals(in Face face, Triangulation? tris)
    {
        var polyConnect = new Connect();
        ComputeNormals(face, tris, polyConnect);
    }

    /// <summary>
    /// <see cref="ComputeNormals(in Face, Triangulation?)"/> with a tool for exploring the triangulation
    /// (initialised on it, or loaded with it when needed).
    /// </summary>
    public static void ComputeNormals(in Face face, Triangulation? tris, Connect polyConnect)
    {
        if (tris is null || tris.HasNormals)
        {
            return;
        }

        // take in face the surface location
        Face zeroFace = ShapeCasts.Face(face.Located(new Placement()));
        Surface? surf = Tool.Surface(zeroFace);
        if (!tris.HasUVNodes || surf is null)
        {
            // compute normals by averaging triangulation normals sharing the same vertex
            global::Kerf.Engine.Meshing.Poly.ComputeNormals(tris);
            return;
        }

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        int tri0, tri1, tri2;
        var norm = new Direction3D();
        tris.AddNormals();
        for (int nodeIter = 1; nodeIter <= tris.NodeCount; ++nodeIter)
        {
            // try to retrieve normal from real surface first, when UV coordinates are available
            if (global::Kerf.Engine.Geometry.Library.GeomLib.NormEstim(surf, tris.UVNode(nodeIter), tol, ref norm) > 1)
            {
                if (!ReferenceEquals(polyConnect.Triangulation(), tris))
                {
                    polyConnect.Load(tris);
                }

                // compute flat normals
                var eqPlan = new Xyz(0.0, 0.0, 0.0);
                for (polyConnect.Initialize(nodeIter); polyConnect.More(); polyConnect.Next())
                {
                    tris.Triangle(polyConnect.Value()).Get(out tri0, out tri1, out tri2);
                    Xyz v1 = tris.Node(tri1).Coord() - tris.Node(tri0).Coord();
                    Xyz v2 = tris.Node(tri2).Coord() - tris.Node(tri1).Coord();
                    Xyz vv = v1 ^ v2;
                    double mod = vv.Modulus();
                    if (mod >= tol)
                    {
                        eqPlan += vv / mod;
                    }
                }
                double modMax = eqPlan.Modulus();
                norm = (modMax > tol) ? new Direction3D(eqPlan) : Constants.DZ();
            }

            tris.SetNormal(nodeIter, norm);
        }
    }

    /// <summary>
    /// The functional form of <see cref="ComputeNormals(in Face, Triangulation?)"/>: the triangulation itself
    /// if it has normals, otherwise a new (unfrozen) copy of it with the normals computed. Null for null.
    /// </summary>
    public static Triangulation? NormalsComputed(in Face face, Triangulation? tris)
    {
        if (tris is null || tris.HasNormals)
        {
            return tris;
        }
        Triangulation copy = tris.Copy();
        ComputeNormals(face, copy);
        return copy;
    }
}
