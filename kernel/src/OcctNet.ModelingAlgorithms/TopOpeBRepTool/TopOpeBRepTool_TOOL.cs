// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_TOOL.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_TOOL.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>The legacy Booleans' topological and geometric tools.</summary>
/// <remarks>
/// Only what the face maker of the 2D offset reaches is ported (plan 2.11, decision 585): the two <c>TggeomE</c> and
/// the file's <c>CheckEdgeLength</c>; and what <see cref="ChFi3d.IsTangentFaces"/> reaches for the 3D offset's
/// analysis (unit 7E): <c>Nt</c>, <c>NggeomF</c> with the file's <c>FUN_ngF</c>, and <c>minDUV</c>. The other members
/// (about a hundred, for the Booleans' interferences) are left out.
/// </remarks>
public static class TopOpeBRepTool_TOOL
{
    // The edge is longer than the confusion tolerance (measured on ten chords), or doesn't have exactly one vertex
    private static bool CheckEdgeLength(in Edge E)
    {
        EdgeCurveAdaptor BC = new(E);

        NCollection_IndexedMap<Shape> aM = new(ShapeHasher.Instance);
        Exploration.MapShapes(E, ShapeType.Vertex, aM);
        int i, anExtent, aN = 10;
        double ln = 0.0, d, t, f, l, dt;
        anExtent = aM.Extent();

        if (anExtent != 1)
        {
            return true;
        }

        Point3D p1, p2;
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

        return ln > global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    // Tg2d, Tg2dApp, TgINSIDE, ..., the other members: not ported (plan 2.11).

    /// <summary>
    /// The unit tangent <paramref name="Tg"/> of the curve <paramref name="BC"/> at <paramref name="par"/>; false if
    /// the parameter is out of the curve's range (by more than the parametric resolution of its tolerance).
    /// </summary>
    public static bool TggeomE(double par, EdgeCurveAdaptor BC, out Vector3D Tg)
    {
        Tg = new Vector3D();
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

        BC.D1(thepar, out Point3D _, out Tg);
        Tg.Normalize();
        return true;
    }

    /// <summary>
    /// The unit tangent <paramref name="Tg"/> of the edge <paramref name="E"/> at <paramref name="par"/>; false for a
    /// degenerated edge, a closed edge no longer than the confusion tolerance, or a parameter out of range.
    /// </summary>
    public static bool TggeomE(double par, in Edge E, out Vector3D Tg)
    {
        Tg = new Vector3D();
        bool isdgE = Tool.Degenerated(E);
        if (isdgE)
        {
            return false;
        }

        EdgeCurveAdaptor BC = new(E);
        // modified by NIZNHY-PKV Fri Aug  4 09:49:31 2000 f
        if (!CheckEdgeLength(E))
        {
            return false;
        }
        // modified by NIZNHY-PKV Fri Aug  4 09:49:36 2000 t

        return TggeomE(par, BC, out Tg);
    }

    //=================================================================================================

    /// <summary>
    /// The unit normal <paramref name="normt"/> of the face <paramref name="f"/> at <paramref name="uv"/>, with the
    /// face's orientation; false (and <paramref name="normt"/> unchanged) where the geometric normal can't be computed.
    /// </summary>
    public static bool Nt(in Point2D uv, in Face f, ref Direction3D normt)
    {
        bool ok = NggeomF(uv, f, out Vector3D nggeom);
        if (!ok)
        {
            return false;
        }
        normt = new Direction3D(nggeom);
        if (f.Orientation() == Orientation.Reversed)
        {
            normt.Reverse();
        }
        return true;
    }

    //=================================================================================================

    private static bool FUN_ngF(in Point2D uv, in Face F, ref Vector3D ngF)
    {
        FaceSurfaceAdaptor bs = new(F);
        double tol3d = bs.Tolerance();
        double tolu = bs.UResolution(tol3d);
        double tolv = bs.VResolution(tol3d);

        // ###############################
        // nyi : all geometries are direct
        // ###############################
        Point3D p;
        Vector3D d1u, d1v;
        bs.D1(uv.X(), uv.Y(), out p, out d1u, out d1v);

        double delta = minDUV(F);
        delta *= 1.0e-1;

        double du = d1u.Magnitude();
        double dv = d1v.Magnitude();
        bool kpart = (du < tolu) || (dv < tolv);
        if (kpart)
        {
            SurfaceKind ST = bs.GetType();
            if (ST == SurfaceKind.Cone)
            {
                bool nullx = (Math.Abs(uv.X()) < tolu);
                bool apex = nullx && (Math.Abs(uv.Y()) < tolv);
                if (apex)
                {
                    Direction3D axis = bs.Cone().Axis().Direction();
                    Vector3D ng = new(axis);
                    ng.Reverse();
                    ngF = ng;
                    return true;
                }
                else if (du < tolu)
                {
                    double x = uv.X();

                    double y = uv.Y();
                    double vf = bs.FirstVParameter();

                    if (Math.Abs(vf - y) < tolu)
                    {
                        vf += delta;
                    }
                    else
                    {
                        vf -= delta;
                    }

                    // modified by NIZHNY-MZV  Fri Nov 26 12:38:55 1999
                    y = vf;
                    bs.D1(x, y, out p, out d1u, out d1v);
                    Vector3D ng = d1u ^ d1v;

                    ngF = ng;
                    return true;
                }
            }
            if (ST == SurfaceKind.Sphere)
            {
                double pisur2 = M_PI * .5;
                double u = uv.X(), v = uv.Y();
                bool vpisur2 = (Math.Abs(v - pisur2) < tolv);
                bool vmoinspisur2 = (Math.Abs(v + pisur2) < tolv);
                bool apex = vpisur2 || vmoinspisur2;
                if (apex)
                {
                    Point3D center = bs.Sphere().Location();
                    Point3D value = bs.Value(u, v);
                    Vector3D ng = new(center, value);
                    ngF = ng;
                    return true;
                }
            }
            // (OCCT_DEBUG trace "FUN_tool_nggeomF NYI" left out)
            return false;
        } // kpart

        Direction3D udir = new(d1u);
        Direction3D vdir = new(d1v);
        ngF = new Vector3D(udir ^ vdir);
        return true;
    }

    /// <summary>
    /// The geometric normal <paramref name="ng"/> (not unit at a cone's apex or a degenerate place) of the face's
    /// surface at <paramref name="uv"/>, without the face's orientation; false where it can't be computed.
    /// </summary>
    public static bool NggeomF(in Point2D uv, in Face f, out Vector3D ng)
    {
        ng = new Vector3D();
        return FUN_ngF(uv, f, ref ng);
    }

    //=================================================================================================

    /// <summary>The smaller of the face's parametric extents in U and V.</summary>
    public static double minDUV(in Face F)
    {
        FaceSurfaceAdaptor BS = new(F);
        double delta = BS.LastUParameter() - BS.FirstUParameter();
        double tmp = BS.LastVParameter() - BS.FirstVParameter();
        delta = (tmp < delta) ? tmp : delta;
        return delta;
    }
}
