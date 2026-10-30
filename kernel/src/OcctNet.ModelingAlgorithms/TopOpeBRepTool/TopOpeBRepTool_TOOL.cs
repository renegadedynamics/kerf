// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_TOOL.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_TOOL.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The legacy Booleans' topological and geometric tools.</summary>
/// <remarks>
/// Only what the face maker of the 2D offset reaches is ported (plan 2.11, decision 585): the two <c>TggeomE</c> and
/// the file's <c>CheckEdgeLength</c>. The other members (about a hundred, for the Booleans' interferences) are left
/// out.
/// </remarks>
public static class TopOpeBRepTool_TOOL
{
    // The edge is longer than the confusion tolerance (measured on ten chords), or doesn't have exactly one vertex
    private static bool CheckEdgeLength(in TopoDS_Edge E)
    {
        BRepAdaptor_Curve BC = new(E);

        NCollection_IndexedMap<TopoDS_Shape> aM = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(E, TopAbs_VERTEX, aM);
        int i, anExtent, aN = 10;
        double ln = 0.0, d, t, f, l, dt;
        anExtent = aM.Extent();

        if (anExtent != 1)
        {
            return true;
        }

        gp_Pnt p1, p2;
        f = BC.FirstParameter();
        l = BC.LastParameter();
        dt = (l - f) / aN;

        p1 = BC.Value(f);
        for (i = 1; i <= aN; i++)
        {
            t = f + i * dt;

            if (i == aN)
            {
                p2 = BC.Value(l);
            }
            else
            {
                p2 = BC.Value(t);
            }

            d = p1.Distance(p2);
            ln += d;
            p1 = p2;
        }

        return ln > Precision.Confusion();
    }

    // Tg2d, Tg2dApp, TgINSIDE, ..., the other members: not ported (plan 2.11).

    /// <summary>
    /// The unit tangent <paramref name="Tg"/> of the curve <paramref name="BC"/> at <paramref name="par"/>; false if
    /// the parameter is out of the curve's range (by more than the parametric resolution of its tolerance).
    /// </summary>
    public static bool TggeomE(double par, BRepAdaptor_Curve BC, out gp_Vec Tg)
    {
        Tg = new gp_Vec();
        double f = BC.FirstParameter(), l = BC.LastParameter();
        double tolE = BC.Tolerance();
        double tolp = BC.Resolution(tolE);

        bool onf = Math.Abs(f - par) < tolp;
        bool onl = Math.Abs(l - par) < tolp;
        bool inbounds = (f < par) && (par < l);

        if ((!inbounds) && (!onf) && (!onl))
        {
            return false;
        }
        double thepar = par;

        BC.D1(thepar, out gp_Pnt _, out Tg);
        Tg.Normalize();
        return true;
    }

    /// <summary>
    /// The unit tangent <paramref name="Tg"/> of the edge <paramref name="E"/> at <paramref name="par"/>; false for a
    /// degenerated edge, a closed edge no longer than the confusion tolerance, or a parameter out of range.
    /// </summary>
    public static bool TggeomE(double par, in TopoDS_Edge E, out gp_Vec Tg)
    {
        Tg = new gp_Vec();
        bool isdgE = BRep_Tool.Degenerated(E);
        if (isdgE)
        {
            return false;
        }

        BRepAdaptor_Curve BC = new(E);
        // modified by NIZNHY-PKV Fri Aug  4 09:49:31 2000 f
        if (!CheckEdgeLength(E))
        {
            return false;
        }
        // modified by NIZNHY-PKV Fri Aug  4 09:49:36 2000 t

        return TggeomE(par, BC, out Tg);
    }
}
