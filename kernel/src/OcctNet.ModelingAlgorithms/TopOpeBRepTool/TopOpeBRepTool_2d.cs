// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_2d.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The legacy Booleans' pcurve functions (<c>FC2D_*</c>): the pcurve of an edge on a face, stored on the edge ("old")
/// or computed by projecting its 3D curve ("new").
/// </summary>
/// <remarks>
/// <para>
/// OCCT's free functions, as a static class named after the file (translation rules, section 9). Only what the face
/// maker of the 2D offset reaches is ported (plan 2.11, decision 585). Left out: <c>FC2D_Prepare</c>,
/// <c>FC2D_HasC3D</c>, <c>FC2D_EditableCurveOnSurface</c>, and the <c>FC2D_CurveOnSurface</c> overload with a reference
/// edge (with its <c>FC2D_translate</c>), all called by the Booleans only.
/// </para>
/// <para>
/// OCCT keeps the "new" pcurves in a per-thread cache that only <c>FC2D_Prepare</c> (and <c>FC2D_FancestorE</c>,
/// which needs it) creates. Neither runs in the port, so the cache never exists: <see cref="FC2D_HasNewCurveOnSurface(in TopoDS_Edge, in TopoDS_Face, out Geom2d_Curve?, out double, out double, out double)"/>
/// is false and <see cref="FC2D_AddNewCurveOnSurface"/> returns 1, as OCCT's code does without a cache (decision 792).
/// </para>
/// </remarks>
public static class TopOpeBRepTool_2d
{
    /// <summary>Whether <paramref name="E"/> has a pcurve on <paramref name="F"/>, stored or computed.</summary>
    public static bool FC2D_HasCurveOnSurface(in TopoDS_Edge E, in TopoDS_Face F)
    {
        bool hasold = FC2D_HasOldCurveOnSurface(E, F, out _);
        bool hasnew = FC2D_HasNewCurveOnSurface(E, F, out _);
        bool b = hasold || hasnew;
        return b;
    }

    /// <summary>
    /// The pcurve of <paramref name="E"/> on <paramref name="F"/> as BRep_Tool gives it (stored, or computed on a
    /// plane), its range, and the edge's tolerance; whether there is one.
    /// </summary>
    public static bool FC2D_HasOldCurveOnSurface(in TopoDS_Edge E,
                                                 in TopoDS_Face F,
                                                 out Geom2d_Curve? C2D,
                                                 out double f2d,
                                                 out double l2d,
                                                 out double tol)
    {
        bool hasold = false;
        tol = BRep_Tool.Tolerance(E);
        C2D = BRep_Tool.CurveOnSurface(E, F, out f2d, out l2d);
        hasold = C2D is not null;
        return hasold;
    }

    /// <summary>The pcurve of <paramref name="E"/> on <paramref name="F"/> as BRep_Tool gives it; whether there is one.</summary>
    public static bool FC2D_HasOldCurveOnSurface(in TopoDS_Edge E, in TopoDS_Face F, out Geom2d_Curve? C2D)
    {
        bool hasold = FC2D_HasOldCurveOnSurface(E, F, out C2D, out _, out _, out _);
        return hasold;
    }

    /// <summary>
    /// The pcurve of <paramref name="E"/> on <paramref name="F"/> in the cache of new pcurves. The cache never exists
    /// in the port (see the remarks), so this is false and leaves its outputs null or zero.
    /// </summary>
    public static bool FC2D_HasNewCurveOnSurface(in TopoDS_Edge E,
                                                 in TopoDS_Face F,
                                                 out Geom2d_Curve? C2D,
                                                 out double f2d,
                                                 out double l2d,
                                                 out double tol)
    {
        // FC2D_PNewCurveOnSurface: no cache, no pcurve
        C2D = null;
        f2d = 0.0;
        l2d = 0.0;
        tol = 0.0;
        bool hasnew = false;
        return hasnew;
    }

    /// <summary>The pcurve of <paramref name="E"/> on <paramref name="F"/> in the cache of new pcurves (never, see the remarks).</summary>
    public static bool FC2D_HasNewCurveOnSurface(in TopoDS_Edge E, in TopoDS_Face F, out Geom2d_Curve? C2D)
    {
        bool b = FC2D_HasNewCurveOnSurface(E, F, out C2D, out _, out _, out _);
        return b;
    }

    /// <summary>
    /// Stores a computed pcurve in the cache of new pcurves: 1 (nothing stored) when <paramref name="C2D"/> is null or
    /// the cache doesn't exist, which in the port is always (see the remarks).
    /// </summary>
    public static int FC2D_AddNewCurveOnSurface(Geom2d_Curve? C2D,
                                                in TopoDS_Edge E,
                                                in TopoDS_Face F,
                                                double f2d,
                                                double l2d,
                                                double tol)
    {
        if (C2D is null)
        {
            return 1;
        }
        // OCCT makes a TopOpeBRepTool_C2DF here and returns 1 as the cache is absent
        return 1;
    }

    // FC2D_make2d (the overload without a reference edge)
    private static Geom2d_Curve? FC2D_make2d(in TopoDS_Edge E,
                                             in TopoDS_Face F,
                                             ref double f2d,
                                             ref double l2d,
                                             ref double tol,
                                             bool trim3d = false)
    {
        Geom2d_Curve? C2D = BRep_Tool.CurveOnSurface(E, F, out f2d, out l2d);
        if (C2D is not null)
        {
            return C2D;
        }

        // pas de 2D
        Geom_Curve? C1 = BRep_Tool.Curve(E, out TopLoc_Location eloc, out double f3d, out double l3d);
        bool hasC3D = C1 is not null;
        if (hasC3D)
        {
            bool elocid = eloc.IsIdentity();
            Geom_Curve C2;
            if (elocid)
            {
                C2 = C1!;
            }
            else
            {
                C2 = (Geom_Curve)C1!.Transformed(eloc.Transformation());
            }
            double f = 0.0, l = 0.0;
            if (trim3d)
            {
                f = f3d;
                l = l3d;
            }
            C2D = TopOpeBRepTool_CurveTool.MakePCurveOnFace(F, C2, out tol, f, l);
            f2d = f3d;
            l2d = l3d;
            return C2D;
        }

        // OCCT: without a 3D curve, FC2D_FancestorE looks for a face of the edge in a map that only FC2D_Prepare (the
        // Booleans, not ported) creates; called without it, native dereferences a null pointer. The port throws.
        throw new Standard_NullObject("FC2D_FancestorE: the ancestor map of FC2D_Prepare does not exist");
    } // make2d1

    /// <summary>
    /// Computes the pcurve of <paramref name="E"/> on <paramref name="F"/> (the stored one, or the projection of its 3D
    /// curve, trimmed to its range with <paramref name="trim3d"/>) and adds it to the cache of new pcurves.
    /// </summary>
    /// <exception cref="Standard_NullObject">The edge has neither a pcurve on the face nor a 3D curve (native crashes).</exception>
    public static Geom2d_Curve? FC2D_MakeCurveOnSurface(in TopoDS_Edge E,
                                                        in TopoDS_Face F,
                                                        ref double f,
                                                        ref double l,
                                                        ref double tol,
                                                        bool trim3d = false)
    {
        Geom2d_Curve? C2D = FC2D_make2d(E, F, ref f, ref l, ref tol, trim3d);
        FC2D_AddNewCurveOnSurface(C2D, E, F, f, l, tol);
        return C2D;
    }

    /// <summary>
    /// The pcurve of <paramref name="E"/> on <paramref name="F"/>, its range and tolerance: the stored one, or a
    /// computed one (<see cref="FC2D_MakeCurveOnSurface"/>).
    /// </summary>
    /// <exception cref="Standard_NullObject">The edge has neither a pcurve on the face nor a 3D curve (native crashes).</exception>
    public static Geom2d_Curve? FC2D_CurveOnSurface(in TopoDS_Edge E,
                                                    in TopoDS_Face F,
                                                    out double f,
                                                    out double l,
                                                    out double tol,
                                                    bool trim3d = false)
    {
        Geom2d_Curve? C2D;
        bool hasold = FC2D_HasOldCurveOnSurface(E, F, out C2D, out f, out l, out tol);
        if (hasold)
        {
            return C2D;
        }
        bool hasnew = FC2D_HasNewCurveOnSurface(E, F, out C2D, out double fn, out double ln, out double toln);
        if (hasnew)
        {
            f = fn;
            l = ln;
            tol = toln;
            return C2D;
        }
        C2D = FC2D_MakeCurveOnSurface(E, F, ref f, ref l, ref tol, trim3d);
        return C2D;
    }
}
