// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_DirectModification.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_DirectModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The modification that converts indirect elementary surfaces (left-handed frames) into direct ones, and cones with
/// a negative semi-angle into cones with a positive one, reversing the faces, wires and pcurves accordingly.
/// </summary>
/// <remarks>
/// The edges it changes are the modifier's new edges, never the caller's (decision 973): <see cref="NewCurve"/>
/// rebuilds every edge with a pcurve on an indirect surface, and <see cref="NewCurve2d"/> writes only into that new
/// edge (<c>NewE</c>).
/// </remarks>
public class ShapeCustom_DirectModification : ShapeCustom_Modification
{
    /// <summary>An empty modification.</summary>
    public ShapeCustom_DirectModification()
    {
    }

    // S4181 returns 0 - none, 1 - indirect, 2 - negative cone, 3 - indirect negative cone
    private static int IsIndirectSurface(ref Geom_Surface S, in TopLoc_Location L)
    {
        int result = 0;

        Geom_Surface TS = S;
        while (TS is Geom_RectangularTrimmedSurface aRTS)
        {
            TS = aRTS.BasisSurface();
        }

        if (TS is Geom_ElementarySurface ES)
        {
            // is the surface indirect ?
            gp_Trsf t = L.Transformation();
            bool neg = t.IsNegative();
            bool det = (t.VectorialPart().Determinant() < 0.0);
            bool dir = ES.Position().Direct();
            if ((neg != det) == dir)
            {
                result = 1;
            }
            if (ES is Geom_ConicalSurface CS)
            {
                // does the cone have negative semiangle ?
                if (CS.SemiAngle() < 0.0)
                {
                    result += 2;
                }
            }
            if (result != 0)
            {
                S = TS;
            }
        }

        return result;
    }

    /// <summary>
    /// True if the face lies on an indirect surface or a cone with a negative semi-angle: then S is the surface
    /// reversed (in U for an indirect surface, in U and V for a negative cone, in V for both), and the face and its
    /// wires are reversed for an indirect surface.
    /// </summary>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out TopLoc_Location L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        Tol = 0.0;
        RevWires = RevFace = false;
        S = BRep_Tool.Surface(F, out L);
        // OCCT.NET: OCCT dereferences the surface of a face without one; the port finds it not indirect
        Geom_Surface aS = S!;

        switch (IsIndirectSurface(ref aS, L))
        {
            case 1:
            { // Indirect surface
                // UReverse a copy of S
                S = aS.UReversed();
                RevWires = true;
                RevFace = true;
                break;
            }
            case 2:
            { // Negative cone
                // U- and VReverse a copy of S
                S = aS.VReversed();
                S.UReverse();
                RevWires = false;
                RevFace = false;
                break;
            }
            case 3:
            { // Indirect negative cone
                // VReverse a copy of S
                S = aS.VReversed();
                RevWires = true;
                RevFace = true;
                break;
            }
            default:
                S = aS;
                return false;
        }

        SendMsg(F, new Message_Msg("DirectModification.NewSurface.MSG0"));

        Tol = BRep_Tool.Tolerance(F);

        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on an indirect surface (so that the edge is copied): then C is a copy of its 3D
    /// curve.
    /// </summary>
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out TopLoc_Location L, out double Tol)
    {
        C = null;
        L = new TopLoc_Location();
        Tol = 0.0;
        //: p5 abv 26 Feb 99: force copying of edge if any its pcurve will be replaced
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;

        // iterate on pcurves
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not BRep_GCurve GC || !GC.IsCurveOnSurface())
            {
                continue;
            }
            Geom_Surface S = GC.Surface()!;
            TopLoc_Location Loc = GC.Location();
            if (IsIndirectSurface(ref S, Loc) == 0)
            {
                continue;
            }
            C = BRep_Tool.Curve(E, out L, out double f, out double l);
            if (C is not null)
            {
                C = (Geom_Curve)C.Copy();
            }
            Tol = BRep_Tool.Tolerance(E);
            return true;
        }
        return false;
    }

    /// <summary>False: 3D points are never modified.</summary>
    public override bool NewPoint(in TopoDS_Vertex V, out gp_Pnt P, out double Tol)
    {
        P = new gp_Pnt();
        Tol = 0.0;
        // 3d points are never modified
        return false;
    }

    /// <summary>
    /// True if the face lies on an indirect surface (C is the pcurve mirrored accordingly) or the edge was copied (C
    /// is a copy of the pcurve).
    /// </summary>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol)
    {
        C = null;
        Tol = 0.0;
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location L);

        Geom_Surface aS = S!;
        int result = IsIndirectSurface(ref aS, L);
        if (result == 0 && E.IsSame(NewE))
        {
            return false;
        }

        C = BRep_Tool.CurveOnSurface(E, F, out double f, out double l);
        Tol = BRep_Tool.Tolerance(E);

        if (result != 0)
        {
            gp_Trsf2d T = new gp_Trsf2d();

            switch (result)
            {
                case 1:
                { // Indirect surface
                    // mirror the PCurve about the V axis
                    T.SetMirror(gp.OY2d());
                    C = (Geom2d_Curve)C!.Transformed(T);
                    break;
                }
                case 2:
                { // Negative cone
                    // mirror the PCurve about the U and V axis
                    T.SetMirror(gp.OX2d());
                    C = (Geom2d_Curve)C!.Transformed(T);
                    T.SetMirror(gp.OY2d());
                    C.Transform(T);
                    break;
                }
                case 3:
                { // Indirect negative cone
                    // mirror the PCurve about the U axis
                    T.SetMirror(gp.OX2d());
                    C = (Geom2d_Curve)C!.Transformed(T);
                    break;
                }
            }

            // #26 rln When seam edge contains triangulations trimming is lost by BRep_Builder::UpdateEdge
            if (BRepTools.IsReallyClosed(E, F))
            {
                // szv#4:S4163:12Mar99 SGI warning
                TopoDS_Shape sh = NewE.Reversed();
                Geom2d_Curve? tmp = BRep_Tool.CurveOnSurface(TopoDS.Edge(sh), NewF, out f, out l);
                if (tmp is null)
                {
                    tmp = BRep_Tool.CurveOnSurface(E, F, out f, out l);
                    BRep_Builder B = new BRep_Builder();
                    // OCCT.NET: NewE is the modifier's new edge here, never the caller's (decision 973)
                    B.UpdateEdge(NewE, tmp, C, NewF, Tol);
                    B.Range(NewE, NewF, f, l);
                    // anyway, tmp will be removed later by BRepTools_Modifier
                }
            }
        }
        else
        {
            //: p5 abv 26 Feb 99: force copying of pcurves if edge was copied
            if (C is not null)
            {
                C = (Geom2d_Curve)C.Copy();
            }
        }

        return true;
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override GeomAbs_Shape Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        return BRep_Tool.Continuity(E, F1, F2);
    }
}
