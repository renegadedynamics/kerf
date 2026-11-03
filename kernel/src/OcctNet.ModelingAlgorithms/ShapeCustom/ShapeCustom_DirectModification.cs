// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_DirectModification.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_DirectModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;

using Kerf.Topology;

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
    private static int IsIndirectSurface(ref Surface S, in Placement L)
    {
        int result = 0;

        Surface TS = S;
        while (TS is TrimmedSurface aRTS)
        {
            TS = aRTS.BasisSurface();
        }

        if (TS is ElementarySurface ES)
        {
            // is the surface indirect ?
            Transform3D t = L.Transformation();
            bool neg = t.IsNegative;
            bool det = (t.VectorialPart().Determinant() < 0.0);
            bool dir = ES.Position().Direct();
            if ((neg != det) == dir)
            {
                result = 1;
            }
            if (ES is ConicalSurface CS)
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
    public override bool NewSurface(in Face F,
                                    out Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        Tol = 0.0;
        RevWires = RevFace = false;
        S = Tool.Surface(F, out L);
        // OCCT.NET: OCCT dereferences the surface of a face without one; the port finds it not indirect
        Surface aS = S!;

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

        SendMsg(F, new Msg("DirectModification.NewSurface.MSG0"));

        Tol = Tool.Tolerance(F);

        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on an indirect surface (so that the edge is copied): then C is a copy of its 3D
    /// curve.
    /// </summary>
    public override bool NewCurve(in Edge E, out Curve3D? C, out Placement L, out double Tol)
    {
        C = null;
        L = new Placement();
        Tol = 0.0;
        //: p5 abv 26 Feb 99: force copying of edge if any its pcurve will be replaced
        BrepTEdge TE = (BrepTEdge)E.TShape()!;

        // iterate on pcurves
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not GCurve GC || !GC.IsCurveOnSurface())
            {
                continue;
            }
            Surface S = GC.Surface()!;
            Placement Loc = GC.Location();
            if (IsIndirectSurface(ref S, Loc) == 0)
            {
                continue;
            }
            C = Tool.Curve(E, out L, out double f, out double l);
            if (C is not null)
            {
                C = (Curve3D)C.Copy();
            }
            Tol = Tool.Tolerance(E);
            return true;
        }
        return false;
    }

    /// <summary>False: 3D points are never modified.</summary>
    public override bool NewPoint(in Vertex V, out Point3D P, out double Tol)
    {
        P = new Point3D();
        Tol = 0.0;
        // 3d points are never modified
        return false;
    }

    /// <summary>
    /// True if the face lies on an indirect surface (C is the pcurve mirrored accordingly) or the edge was copied (C
    /// is a copy of the pcurve).
    /// </summary>
    public override bool NewCurve2d(in Edge E,
                                    in Face F,
                                    in Edge NewE,
                                    in Face NewF,
                                    out Curve2D? C,
                                    out double Tol)
    {
        C = null;
        Tol = 0.0;
        Surface? S = Tool.Surface(F, out Placement L);

        Surface aS = S!;
        int result = IsIndirectSurface(ref aS, L);
        if (result == 0 && E.IsSame(NewE))
        {
            return false;
        }

        C = Tool.CurveOnSurface(E, F, out double f, out double l);
        Tol = Tool.Tolerance(E);

        if (result != 0)
        {
            Transform2D T = new Transform2D();

            switch (result)
            {
                case 1:
                { // Indirect surface
                    // mirror the PCurve about the V axis
                    T.SetMirror(Constants.OY2d());
                    C = (Curve2D)C!.Transformed(T);
                    break;
                }
                case 2:
                { // Negative cone
                    // mirror the PCurve about the U and V axis
                    T.SetMirror(Constants.OX2d());
                    C = (Curve2D)C!.Transformed(T);
                    T.SetMirror(Constants.OY2d());
                    C.Transform(T);
                    break;
                }
                case 3:
                { // Indirect negative cone
                    // mirror the PCurve about the U axis
                    T.SetMirror(Constants.OX2d());
                    C = (Curve2D)C!.Transformed(T);
                    break;
                }
            }

            // #26 rln When seam edge contains triangulations trimming is lost by BRep_Builder::UpdateEdge
            if (BrepTools.IsReallyClosed(E, F))
            {
                // szv#4:S4163:12Mar99 SGI warning
                Shape sh = NewE.Reversed();
                Curve2D? tmp = Tool.CurveOnSurface(ShapeCasts.Edge(sh), NewF, out f, out l);
                if (tmp is null)
                {
                    tmp = Tool.CurveOnSurface(E, F, out f, out l);
                    BrepBuilder B = new BrepBuilder();
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
                C = (Curve2D)C.Copy();
            }
        }

        return true;
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in Vertex V, in Edge E, ref double P, ref double Tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override Continuity Continuity(in Edge E,
                                             in Face F1,
                                             in Face F2,
                                             in Edge NewE,
                                             in Face NewF1,
                                             in Face NewF2)
    {
        return Tool.Continuity(E, F1, F2);
    }
}
