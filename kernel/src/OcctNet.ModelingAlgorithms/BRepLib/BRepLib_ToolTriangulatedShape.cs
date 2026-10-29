// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ToolTriangulatedShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ToolTriangulatedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Computes the nodal normals of a face's triangulation.</summary>
/// <remarks>
/// A triangulation attached to a face is frozen (design 4.1): <see cref="ComputeNormals(in TopoDS_Face, Poly_Triangulation?)"/>
/// writes into the triangulation it is given, as OCCT does, and throws <see cref="Standard_ImmutableObject"/> on a
/// frozen one without normals; <see cref="NormalsComputed"/> returns a copy with normals instead.
/// </remarks>
public static class BRepLib_ToolTriangulatedShape
{
    /// <summary>
    /// Computes the nodal normals of <paramref name="theTris"/> from the face's surface at the UV nodes (or, without
    /// UV nodes or surface, by averaging the triangles' normals). Does nothing if it already has normals.
    /// </summary>
    public static void ComputeNormals(in TopoDS_Face theFace, Poly_Triangulation? theTris)
    {
        var aPolyConnect = new Poly_Connect();
        ComputeNormals(theFace, theTris, aPolyConnect);
    }

    /// <summary>
    /// <see cref="ComputeNormals(in TopoDS_Face, Poly_Triangulation?)"/> with a tool for exploring the triangulation
    /// (initialised on it, or loaded with it when needed).
    /// </summary>
    public static void ComputeNormals(in TopoDS_Face theFace, Poly_Triangulation? theTris, Poly_Connect thePolyConnect)
    {
        if (theTris is null || theTris.HasNormals())
        {
            return;
        }

        // take in face the surface location
        TopoDS_Face aZeroFace = TopoDS.Face(theFace.Located(new TopLoc_Location()));
        Geom_Surface? aSurf = BRep_Tool.Surface(aZeroFace);
        if (!theTris.HasUVNodes() || aSurf is null)
        {
            // compute normals by averaging triangulation normals sharing the same vertex
            Poly.ComputeNormals(theTris);
            return;
        }

        double aTol = Precision.Confusion();
        int aTri0, aTri1, aTri2;
        var aNorm = new gp_Dir();
        theTris.AddNormals();
        for (int aNodeIter = 1; aNodeIter <= theTris.NbNodes(); ++aNodeIter)
        {
            // try to retrieve normal from real surface first, when UV coordinates are available
            if (GeomLib.NormEstim(aSurf, theTris.UVNode(aNodeIter), aTol, ref aNorm) > 1)
            {
                if (!ReferenceEquals(thePolyConnect.Triangulation(), theTris))
                {
                    thePolyConnect.Load(theTris);
                }

                // compute flat normals
                var eqPlan = new gp_XYZ(0.0, 0.0, 0.0);
                for (thePolyConnect.Initialize(aNodeIter); thePolyConnect.More(); thePolyConnect.Next())
                {
                    theTris.Triangle(thePolyConnect.Value()).Get(out aTri0, out aTri1, out aTri2);
                    gp_XYZ v1 = theTris.Node(aTri1).Coord() - theTris.Node(aTri0).Coord();
                    gp_XYZ v2 = theTris.Node(aTri2).Coord() - theTris.Node(aTri1).Coord();
                    gp_XYZ vv = v1 ^ v2;
                    double aMod = vv.Modulus();
                    if (aMod >= aTol)
                    {
                        eqPlan += vv / aMod;
                    }
                }
                double aModMax = eqPlan.Modulus();
                aNorm = (aModMax > aTol) ? new gp_Dir(eqPlan) : gp.DZ();
            }

            theTris.SetNormal(aNodeIter, aNorm);
        }
    }

    /// <summary>
    /// The functional form of <see cref="ComputeNormals(in TopoDS_Face, Poly_Triangulation?)"/>: the triangulation itself
    /// if it has normals, otherwise a new (unfrozen) copy of it with the normals computed. Null for null.
    /// </summary>
    public static Poly_Triangulation? NormalsComputed(in TopoDS_Face theFace, Poly_Triangulation? theTris)
    {
        if (theTris is null || theTris.HasNormals())
        {
            return theTris;
        }
        Poly_Triangulation aCopy = theTris.Copy();
        ComputeNormals(theFace, aCopy);
        return aCopy;
    }
}
