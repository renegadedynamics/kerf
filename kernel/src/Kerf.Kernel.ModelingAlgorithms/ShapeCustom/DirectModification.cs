// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_DirectModification.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_DirectModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Engine.Healing.Custom;

/// <summary>
/// The modification that converts indirect elementary surfaces (left-handed frames) into direct ones, and cones with
/// a negative semi-angle into cones with a positive one, reversing the faces, wires and pcurves accordingly.
/// </summary>
/// <remarks>
/// The edges it changes are the modifier's new edges, never the caller's (decision 973): <see cref="NewCurve"/>
/// rebuilds every edge with a pcurve on an indirect surface, and <see cref="NewCurve2d"/> writes only into that new
/// edge (<c>NewE</c>).
/// </remarks>
internal class DirectModification : CustomModification
{
    /// <summary>An empty modification.</summary>
    public DirectModification()
    {
    }

    // S4181 returns 0 - none, 1 - indirect, 2 - negative cone, 3 - indirect negative cone
    private static int IsIndirectSurface(ref Surface s, in Placement l)
    {
        int result = 0;

        Surface ts = s;
        while (ts is TrimmedSurface rts)
        {
            ts = rts.BasisSurface;
        }

        if (ts is ElementarySurface es)
        {
            // is the surface indirect ?
            Transform3D t = l.Transformation;
            bool neg = t.IsNegative;
            bool det = (t.VectorialPart.Determinant < 0.0);
            bool dir = es.Position.IsDirect;
            if ((neg != det) == dir)
            {
                result = 1;
            }
            if (es is ConicalSurface cs)
            {
                // does the cone have negative semiangle ?
                if (cs.SemiAngle < 0.0)
                {
                    result += 2;
                }
            }
            if (result != 0)
            {
                s = ts;
            }
        }

        return result;
    }

    /// <summary>
    /// True if the face lies on an indirect surface or a cone with a negative semi-angle: then S is the surface
    /// reversed (in U for an indirect surface, in U and V for a negative cone, in V for both), and the face and its
    /// wires are reversed for an indirect surface.
    /// </summary>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        tol = 0.0;
        revWires = revFace = false;
        s = Tool.Surface(f, out l);
        // Kerf: OCCT dereferences the surface of a face without one; the port finds it not indirect
        Surface aS = s!;

        switch (IsIndirectSurface(ref aS, l))
        {
            case 1:
            { // Indirect surface
                // UReverse a copy of S
                s = aS.UReversed();
                revWires = true;
                revFace = true;
                break;
            }
            case 2:
            { // Negative cone
                // U- and VReverse a copy of S
                s = aS.VReversed();
                s.UReverse();
                revWires = false;
                revFace = false;
                break;
            }
            case 3:
            { // Indirect negative cone
                // VReverse a copy of S
                s = aS.VReversed();
                revWires = true;
                revFace = true;
                break;
            }
            default:
                s = aS;
                return false;
        }

        SendMsg(f, new Msg("DirectModification.NewSurface.MSG0"));

        tol = Tool.Tolerance(f);

        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on an indirect surface (so that the edge is copied): then C is a copy of its 3D
    /// curve.
    /// </summary>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement L, out double tol)
    {
        c = null;
        L = new Placement();
        tol = 0.0;
        //: p5 abv 26 Feb 99: force copying of edge if any its pcurve will be replaced
        BrepTEdge te = (BrepTEdge)e.TShape()!;

        // iterate on pcurves
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not GCurve gc || !gc.IsCurveOnSurface())
            {
                continue;
            }
            Surface s = gc.Surface()!;
            Placement loc = gc.Location();
            if (IsIndirectSurface(ref s, loc) == 0)
            {
                continue;
            }
            c = Tool.Curve(e, out L, out double f, out double l);
            if (c is not null)
            {
                c = (Curve3D)c.Copy();
            }
            tol = Tool.Tolerance(e);
            return true;
        }
        return false;
    }

    /// <summary>False: 3D points are never modified.</summary>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        p = new Point3D();
        tol = 0.0;
        // 3d points are never modified
        return false;
    }

    /// <summary>
    /// True if the face lies on an indirect surface (C is the pcurve mirrored accordingly) or the edge was copied (C
    /// is a copy of the pcurve).
    /// </summary>
    public override bool NewCurve2d(in Edge e,
                                    in Face F,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol)
    {
        c = null;
        tol = 0.0;
        Surface? S = Tool.Surface(F, out Placement L);

        Surface s = S!;
        int result = IsIndirectSurface(ref s, L);
        if (result == 0 && e.IsSame(newE))
        {
            return false;
        }

        c = Tool.CurveOnSurface(e, F, out double f, out double l);
        tol = Tool.Tolerance(e);

        if (result != 0)
        {
            Transform2D t = new Transform2D();

            switch (result)
            {
                case 1:
                { // Indirect surface
                    // mirror the PCurve about the V axis
                    t = t.WithMirror(Constants.OY2d());
                    c = (Curve2D)c!.Transformed(t);
                    break;
                }
                case 2:
                { // Negative cone
                    // mirror the PCurve about the U and V axis
                    t = t.WithMirror(Constants.OX2d());
                    c = (Curve2D)c!.Transformed(t);
                    t = t.WithMirror(Constants.OY2d());
                    c.Transform(t);
                    break;
                }
                case 3:
                { // Indirect negative cone
                    // mirror the PCurve about the U axis
                    t = t.WithMirror(Constants.OX2d());
                    c = (Curve2D)c!.Transformed(t);
                    break;
                }
            }

            // #26 rln When seam edge contains triangulations trimming is lost by BRep_Builder::UpdateEdge
            if (BrepTools.IsReallyClosed(e, F))
            {
                // szv#4:S4163:12Mar99 SGI warning
                Shape sh = newE.Reversed();
                Curve2D? tmp = Tool.CurveOnSurface(ShapeCasts.Edge(sh), newF, out f, out l);
                if (tmp is null)
                {
                    tmp = Tool.CurveOnSurface(e, F, out f, out l);
                    BrepBuilder b = new BrepBuilder();
                    // Kerf: NewE is the modifier's new edge here, never the caller's (decision 973)
                    b.UpdateEdge(newE, tmp, c, newF, tol);
                    b.Range(newE, newF, f, l);
                    // anyway, tmp will be removed later by BRepTools_Modifier
                }
            }
        }
        else
        {
            //: p5 abv 26 Feb 99: force copying of pcurves if edge was copied
            if (c is not null)
            {
                c = (Curve2D)c.Copy();
            }
        }

        return true;
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override Continuity Continuity(in Edge e,
                                             in Face f1,
                                             in Face f2,
                                             in Edge newE,
                                             in Face newF1,
                                             in Face newF2)
    {
        return Tool.Continuity(e, f1, f2);
    }
}
