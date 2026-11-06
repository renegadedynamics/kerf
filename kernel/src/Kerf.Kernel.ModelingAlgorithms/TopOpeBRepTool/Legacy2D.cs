// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_2d.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Booleans.Legacy.Tools;

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
/// which needs it) creates. Neither runs in the port, so the cache never exists: <see cref="FC2D_HasNewCurveOnSurface(in Edge, in Face, out Curve2D?, out double, out double, out double)"/>
/// is false and <see cref="FC2D_AddNewCurveOnSurface"/> returns 1, as OCCT's code does without a cache (decision 792).
/// </para>
/// </remarks>
internal static class Legacy2D
{
    /// <summary>Whether <paramref name="e"/> has a pcurve on <paramref name="f"/>, stored or computed.</summary>
    public static bool FC2D_HasCurveOnSurface(in Edge e, in Face f)
    {
        bool hasold = FC2D_HasOldCurveOnSurface(e, f, out _);
        bool hasnew = FC2D_HasNewCurveOnSurface(e, f, out _);
        bool b = hasold || hasnew;
        return b;
    }

    /// <summary>
    /// The pcurve of <paramref name="e"/> on <paramref name="f"/> as BRep_Tool gives it (stored, or computed on a
    /// plane), its range, and the edge's tolerance; whether there is one.
    /// </summary>
    public static bool FC2D_HasOldCurveOnSurface(in Edge e,
                                                 in Face f,
                                                 out Curve2D? c2D,
                                                 out double f2d,
                                                 out double l2d,
                                                 out double tol)
    {
        bool hasold = false;
        tol = Tool.Tolerance(e);
        c2D = Tool.CurveOnSurface(e, f, out f2d, out l2d);
        hasold = c2D is not null;
        return hasold;
    }

    /// <summary>The pcurve of <paramref name="e"/> on <paramref name="f"/> as BRep_Tool gives it; whether there is one.</summary>
    public static bool FC2D_HasOldCurveOnSurface(in Edge e, in Face f, out Curve2D? c2D)
    {
        bool hasold = FC2D_HasOldCurveOnSurface(e, f, out c2D, out _, out _, out _);
        return hasold;
    }

    /// <summary>
    /// The pcurve of <paramref name="e"/> on <paramref name="f"/> in the cache of new pcurves. The cache never exists
    /// in the port (see the remarks), so this is false and leaves its outputs null or zero.
    /// </summary>
    public static bool FC2D_HasNewCurveOnSurface(in Edge e,
                                                 in Face f,
                                                 out Curve2D? c2D,
                                                 out double f2d,
                                                 out double l2d,
                                                 out double tol)
    {
        // FC2D_PNewCurveOnSurface: no cache, no pcurve
        c2D = null;
        f2d = 0.0;
        l2d = 0.0;
        tol = 0.0;
        bool hasnew = false;
        return hasnew;
    }

    /// <summary>The pcurve of <paramref name="e"/> on <paramref name="f"/> in the cache of new pcurves (never, see the remarks).</summary>
    public static bool FC2D_HasNewCurveOnSurface(in Edge e, in Face f, out Curve2D? c2D)
    {
        bool b = FC2D_HasNewCurveOnSurface(e, f, out c2D, out _, out _, out _);
        return b;
    }

    /// <summary>
    /// Stores a computed pcurve in the cache of new pcurves: 1 (nothing stored) when <paramref name="c2D"/> is null or
    /// the cache doesn't exist, which in the port is always (see the remarks).
    /// </summary>
    public static int FC2D_AddNewCurveOnSurface(Curve2D? c2D,
                                                in Edge e,
                                                in Face f,
                                                double f2d,
                                                double l2d,
                                                double tol)
    {
        if (c2D is null)
        {
            return 1;
        }
        // OCCT makes a TopOpeBRepTool_C2DF here and returns 1 as the cache is absent
        return 1;
    }

    // FC2D_make2d (the overload without a reference edge)
    private static Curve2D? FC2D_make2d(in Edge e,
                                             in Face F,
                                             ref double f2d,
                                             ref double l2d,
                                             ref double tol,
                                             bool trim3d = false)
    {
        Curve2D? c2D = Tool.CurveOnSurface(e, F, out f2d, out l2d);
        if (c2D is not null)
        {
            return c2D;
        }

        // pas de 2D
        Curve3D? c1 = Tool.Curve(e, out Placement eloc, out double f3d, out double l3d);
        bool hasC3D = c1 is not null;
        if (hasC3D)
        {
            bool elocid = eloc.IsIdentity;
            Curve3D c2;
            if (elocid)
            {
                c2 = c1!;
            }
            else
            {
                c2 = (Curve3D)c1!.Transformed(eloc.Transformation);
            }
            double f = 0.0, l = 0.0;
            if (trim3d)
            {
                f = f3d;
                l = l3d;
            }
            c2D = ToolCurveTool.MakePCurveOnFace(F, c2, out tol, f, l);
            f2d = f3d;
            l2d = l3d;
            return c2D;
        }

        // OCCT: without a 3D curve, FC2D_FancestorE looks for a face of the edge in a map that only FC2D_Prepare (the
        // Booleans, not ported) creates; called without it, native dereferences a null pointer. The port throws.
        throw new NullObjectException("FC2D_FancestorE: the ancestor map of FC2D_Prepare does not exist");
    } // make2d1

    /// <summary>
    /// Computes the pcurve of <paramref name="e"/> on <paramref name="F"/> (the stored one, or the projection of its 3D
    /// curve, trimmed to its range with <paramref name="trim3d"/>) and adds it to the cache of new pcurves.
    /// </summary>
    /// <exception cref="NullObjectException">The edge has neither a pcurve on the face nor a 3D curve (native crashes).</exception>
    public static Curve2D? FC2D_MakeCurveOnSurface(in Edge e,
                                                        in Face F,
                                                        ref double f,
                                                        ref double l,
                                                        ref double tol,
                                                        bool trim3d = false)
    {
        Curve2D? c2D = FC2D_make2d(e, F, ref f, ref l, ref tol, trim3d);
        FC2D_AddNewCurveOnSurface(c2D, e, F, f, l, tol);
        return c2D;
    }

    /// <summary>
    /// The pcurve of <paramref name="e"/> on <paramref name="F"/>, its range and tolerance: the stored one, or a
    /// computed one (<see cref="FC2D_MakeCurveOnSurface"/>).
    /// </summary>
    /// <exception cref="NullObjectException">The edge has neither a pcurve on the face nor a 3D curve (native crashes).</exception>
    public static Curve2D? FC2D_CurveOnSurface(in Edge e,
                                                    in Face F,
                                                    out double f,
                                                    out double l,
                                                    out double tol,
                                                    bool trim3d = false)
    {
        Curve2D? c2D;
        bool hasold = FC2D_HasOldCurveOnSurface(e, F, out c2D, out f, out l, out tol);
        if (hasold)
        {
            return c2D;
        }
        bool hasnew = FC2D_HasNewCurveOnSurface(e, F, out c2D, out double fn, out double ln, out double toln);
        if (hasnew)
        {
            f = fn;
            l = ln;
            tol = toln;
            return c2D;
        }
        c2D = FC2D_MakeCurveOnSurface(e, F, ref f, ref l, ref tol, trim3d);
        return c2D;
    }
}
