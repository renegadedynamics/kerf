// Port of: src/ModelingData/TKBRep/BRep/BRep_Tool.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Meshing.MeshPurpose;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Topology.Brep;

/// <summary>
/// Reads the geometry of BRep shapes: the surface of a face, the curves of an edge, the point of a vertex, their
/// tolerances, parameters and polygons.
/// </summary>
/// <remarks>
/// The geometry returned is the shapes' own, frozen, except where a method says it returns a transformed copy. Outputs
/// are <c>out</c> parameters. <see cref="SetUVPoints(in Edge, Surface?, in Placement, in Point2D, in Point2D)"/>
/// changes an edge and is internal.
/// </remarks>
internal static class Tool
{
    // Checks trim range against the effective basis curve.
    private static bool IsValidTrimRange(Curve3D curve, double first, double last)
    {
        double trimFirst = Min(first, last);
        double trimLast = Max(first, last);
        if (!(trimFirst < trimLast))
        {
            return false;
        }

        // Unwrap nested trims.
        Curve3D basisCurve = curve;
        for (TrimmedCurve3D? trim = basisCurve as TrimmedCurve3D; trim is not null;
             trim = basisCurve as TrimmedCurve3D)
        {
            basisCurve = trim.BasisCurve;
        }

        return basisCurve.IsPeriodic
               || (basisCurve.FirstParameter - trimFirst <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                   && trimLast - basisCurve.LastParameter <= global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    /// <summary>The surface of the face, with in <paramref name="l"/> the location for it.</summary>
    public static Surface? Surface(in Face f, out Placement l)
    {
        BrepTFace tf = (BrepTFace)f.TShape()!;
        l = f.Placement * tf.Location();
        return tf.Surface();
    }

    /// <summary>The surface of the face, placed: a transformed copy if the face has a location.</summary>
    public static Surface? Surface(in Face f)
    {
        BrepTFace tf = (BrepTFace)f.TShape()!;
        Surface? s = tf.Surface();

        if (s is null)
        {
            return s;
        }

        Placement l = f.Placement * tf.Location();
        if (!l.IsIdentity)
        {
            Geometry3D copy = s.Transformed(l.Transformation);
            return (Surface)copy;
        }

        return s;
    }

    /// <summary>
    /// The face's triangulation for the purpose (the active one for NONE), with in <paramref name="location"/> the
    /// location for it; null if there is none.
    /// </summary>
    public static Triangulation? Triangulation(in Face face,
                                                    out Placement location,
                                                    MeshPurpose meshPurpose = MeshPurpose_NONE)
    {
        location = face.Placement;
        BrepTFace tFace = (BrepTFace)face.TShape()!;
        return tFace.Triangulation(meshPurpose);
    }

    /// <summary>The face's triangulations, with in <paramref name="location"/> the location for them.</summary>
    /// <remarks>Internal: the list is the face's own. Callers use <see cref="BrepTFace.TriangulationList"/>.</remarks>
    internal static NCollection_List<Triangulation> Triangulations(in Face face,
                                                                       out Placement location)
    {
        location = face.Placement;
        BrepTFace tFace = (BrepTFace)face.TShape()!;
        return tFace.Triangulations();
    }

    /// <summary>The tolerance of the face, at least Precision::Confusion().</summary>
    public static double Tolerance(in Face f)
    {
        BrepTFace tf = (BrepTFace)f.TShape()!;
        double p = tf.Tolerance();
        double pMin = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (p > pMin)
        {
            return p;
        }
        else
        {
            return pMin;
        }
    }

    /// <summary>The NaturalRestriction flag of the face.</summary>
    public static bool NaturalRestriction(in Face f)
    {
        BrepTFace tf = (BrepTFace)f.TShape()!;
        return tf.NaturalRestriction();
    }

    /// <summary>
    /// The 3D curve of the edge, or null; in <paramref name="l"/> the location for it, in <paramref name="first"/> and
    /// <paramref name="last"/> its range.
    /// </summary>
    public static Curve3D? Curve(in Edge e, out Placement l, out double first, out double last)
    {
        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurve3D)
            {
                Curve3DRepresentation gc = (Curve3DRepresentation)cr;
                l = e.Placement * gc.Location();
                gc.Range(out first, out last);
                return gc.Curve3D();
            }

            itcr.Next();
        }

        l = new Placement();
        first = last = 0.0;
        return null;
    }

    /// <summary>
    /// The 3D curve of the edge, placed (a transformed copy if the edge has a location), or null; its range in
    /// <paramref name="first"/> and <paramref name="last"/>.
    /// </summary>
    public static Curve3D? Curve(in Edge e, out double first, out double last)
    {
        Curve3D? c = Curve(e, out Placement l, out first, out last);
        if (c is not null)
        {
            if (!l.IsIdentity)
            {
                Geometry3D copy = c.Transformed(l.Transformation);
                return (Curve3D)copy;
            }
        }

        return c;
    }

    /// <summary>True if the face has a surface.</summary>
    public static bool IsGeometric(in Face f)
    {
        BrepTFace tf = (BrepTFace)f.TShape()!;
        Surface? s = tf.Surface();
        return s is not null;
    }

    /// <summary>True if the edge has a 3D curve or a curve on a surface.</summary>
    public static bool IsGeometric(in Edge e)
    {
        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurve3D)
            {
                Curve3DRepresentation? gc = cr as Curve3DRepresentation;
                if (gc is not null && gc.Curve3D() is not null)
                {
                    return true;
                }
            }
            else if (cr.IsCurveOnSurface())
            {
                return true;
            }

            itcr.Next();
        }

        return false;
    }

    /// <summary>The 3D polygon of the edge, or null; in <paramref name="l"/> the location for it.</summary>
    public static Polygon3D? Polygon3D(in Edge e, out Placement l)
    {
        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygon3D)
            {
                Polygon3DRepresentation gc = (Polygon3DRepresentation)cr;
                l = e.Placement * gc.Location();
                return gc.Polygon3D();
            }

            itcr.Next();
        }

        l = new Placement();
        return null;
    }

    /// <summary>
    /// The curve of the edge in the parametric space of the face, or null; its range in <paramref name="first"/> and
    /// <paramref name="last"/>. For a plane without a stored pcurve, the projection of the 3D curve.
    /// </summary>
    public static Curve2D? CurveOnSurface(in Edge e, in Face f, out double first, out double last)
        => CurveOnSurface(e, f, out first, out last, out _);

    /// <summary>
    /// The curve of the edge in the parametric space of the face, or null; its range in <paramref name="first"/> and
    /// <paramref name="last"/>, and whether it is stored on the edge (false for a computed projection on a plane).
    /// </summary>
    public static Curve2D? CurveOnSurface(in Edge e,
                                               in Face f,
                                               out double first,
                                               out double last,
                                               out bool isStored)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        Edge localEdge = e;
        if (f.Orientation == Reversed)
        {
            localEdge.Reverse();
        }

        return CurveOnSurface(localEdge, s, l, out first, out last, out isStored);
    }

    /// <summary>
    /// The curve of the edge in the parametric space of the surface, or null; its range in <paramref name="first"/>
    /// and <paramref name="last"/>. For a plane without a stored pcurve, the projection of the 3D curve.
    /// </summary>
    public static Curve2D? CurveOnSurface(in Edge e,
                                               Surface? s,
                                               in Placement l,
                                               out double first,
                                               out double last)
        => CurveOnSurface(e, s, l, out first, out last, out _);

    /// <summary>
    /// The curve of the edge in the parametric space of the surface, or null; its range in <paramref name="first"/>
    /// and <paramref name="last"/>, and whether it is stored on the edge (false for a computed projection on a plane).
    /// </summary>
    public static Curve2D? CurveOnSurface(in Edge e,
                                               Surface? s,
                                               in Placement l,
                                               out double first,
                                               out double last,
                                               out bool isStored)
    {
        bool eisreversed = (e.Orientation == Reversed);
        isStored = true;

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        if (itcr.More())
        {
            Placement loc = l.Predivided(e.Placement);
            while (itcr.More())
            {
                CurveRepresentation cr = itcr.Value();
                if (cr.IsCurveOnSurface(s, loc))
                {
                    GCurve gc = (GCurve)cr;
                    gc.Range(out first, out last);
                    if (gc.IsCurveOnClosedSurface && eisreversed)
                    {
                        return gc.PCurve2();
                    }
                    else
                    {
                        return gc.PCurve();
                    }
                }

                itcr.Next();
            }
        }

        // Curve is not found. Try projection on plane
        isStored = false;
        return CurveOnPlane(e, s, l, out first, out last);
    }

    /// <summary>
    /// For a plane, the projection of the edge's 3D curve on it, as a curve in its parametric space; its range in
    /// <paramref name="first"/> and <paramref name="last"/>. Null for other surfaces or an edge without a 3D curve.
    /// </summary>
    public static Curve2D? CurveOnPlane(in Edge e,
                                             Surface? s,
                                             in Placement L,
                                             out double first,
                                             out double last)
    {
        first = last = 0.0;

        // Check if the surface is planar
        PlaneSurface? gp;
        TrimmedSurface? grts = s as TrimmedSurface;
        if (grts is not null)
        {
            gp = grts.BasisSurface as PlaneSurface;
        }
        else
        {
            gp = s as PlaneSurface;
        }

        if (gp is null)
        {
            // not a plane
            return null;
        }

        // Check existence of 3d curve in edge
        Curve3D? c3D = Tool.Curve(e, out Placement curveLocation, out double f, out double l);
        if (c3D is null)
        {
            // no 3d curve
            return null;
        }

        curveLocation = curveLocation.Predivided(L);
        first = f;
        last = l;

        // Transform curve and update parameters in account of scale factor
        if (!curveLocation.IsIdentity)
        {
            Transform3D trsf = curveLocation.Transformation;
            c3D = (Curve3D)c3D.Transformed(trsf);
            f = c3D.TransformedParameter(f, trsf);
            l = c3D.TransformedParameter(l, trsf);
        }

        if (!IsValidTrimRange(c3D, f, l))
        {
            return null;
        }

        // A line in the plane, nearly every edge asking this (unit 11B): the same arithmetic as the projection below, in
        // the same order, without its dozen objects. A line that isn't in the plane is left to the projection, which makes
        // it a B-spline; so is a reversed range, for which the trimmed curve reverses the line.
        if (f < l && c3D is LineCurve3D lineCurve
            && global::Kerf.Engine.Geometry.Projection.ProjectOnPlane.TryProjectLineKeepingParameters(gp.Position, gp.Position.Direction, lineCurve.Line, out Line3D lineOnPlane))
        {
            var lineCurve2D = new LineCurve2D(global::Kerf.Engine.Geometry.Projection.PlaneProjector.ProjectLine(gp.Plane, lineOnPlane));
            lineCurve2D.Freeze();
            return lineCurve2D;
        }

        return CurveOnPlaneByProjection(c3D, f, l, gp);
    }

    // OCCT's route, for any curve on a plane; f and l have been checked by IsValidTrimRange
    internal static Curve2D CurveOnPlaneByProjection(Curve3D c3D, double f, double l, PlaneSurface gp)
    {
        var trimmedCurve = new TrimmedCurve3D(c3D, f, l, true, false);

        // Perform projection
        Curve3D? projOnPlane = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(trimmedCurve, gp, gp.Position.Direction, true);

        var hs = new SurfaceAdaptor(gp);
        var hc = new CurveAdaptor(projOnPlane!);

        var proj = new ProjectedCurve(hs, hc);
        Curve2D pc = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(proj);

        if (pc is TrimmedCurve2D tc)
        {
            pc = tc.BasisCurve;
        }

        return pc;
    }

    /// <summary>The first curve on a surface of the edge, with its surface, location and range; nulls if none.</summary>
    public static void CurveOnSurface(in Edge e,
                                      out Curve2D? c,
                                      out Surface? s,
                                      out Placement l,
                                      out double first,
                                      out double last)
    {
        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface())
            {
                GCurve gc = (GCurve)cr;
                c = gc.PCurve();
                s = gc.Surface();
                l = e.Placement * gc.Location();
                gc.Range(out first, out last);
                return;
            }

            itcr.Next();
        }

        c = null;
        s = null;
        l = new Placement();
        first = last = 0.0;
    }

    /// <summary>
    /// The <paramref name="index"/>-th curve on a surface of the edge (from 1; a seam counts twice), with its surface,
    /// location and range; nulls if none. The outputs are left unchanged if Index is less than 1.
    /// </summary>
    public static void CurveOnSurface(in Edge e,
                                      ref Curve2D? c,
                                      ref Surface? s,
                                      ref Placement l,
                                      ref double first,
                                      ref double last,
                                      int index)
    {
        if (index < 1)
        {
            return;
        }

        int i = 0;
        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        for (; itcr.More(); itcr.Next())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface())
            {
                GCurve gc = (GCurve)cr;
                ++i;
                // Compare index taking into account the fact that for the curves on
                // closed surfaces there are two PCurves
                if (i == index)
                {
                    c = gc.PCurve();
                }
                else if (gc.IsCurveOnClosedSurface && (++i == index))
                {
                    c = gc.PCurve2();
                }
                else
                {
                    continue;
                }

                s = gc.Surface();
                l = e.Placement * gc.Location();
                gc.Range(out first, out last);
                return;
            }
        }

        c = null;
        s = null;
        l = new Placement();
        first = last = 0.0;
    }

    /// <summary>The polygon of the edge in the parametric space of the face, or null.</summary>
    public static Polygon2D? PolygonOnSurface(in Edge e, in Face f)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        Edge localEdge = e;
        if (f.Orientation == Reversed)
        {
            localEdge.Reverse();
        }

        return PolygonOnSurface(localEdge, s, l);
    }

    /// <summary>The polygon of the edge in the parametric space of the surface, or null.</summary>
    public static Polygon2D? PolygonOnSurface(in Edge e, Surface? s, in Placement L)
    {
        Placement l = L.Predivided(e.Placement);
        bool eisreversed = (e.Orientation == Reversed);

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnSurface(s, l))
            {
                if (cr.IsPolygonOnClosedSurface && eisreversed)
                {
                    return cr.Polygon2();
                }
                else
                {
                    return cr.Polygon();
                }
            }

            itcr.Next();
        }

        return null;
    }

    /// <summary>The first polygon on a surface of the edge, with its surface and location; nulls if none.</summary>
    public static void PolygonOnSurface(in Edge e,
                                        out Polygon2D? p,
                                        out Surface? s,
                                        out Placement l)
    {
        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnSurface())
            {
                PolygonOnSurface ps = (PolygonOnSurface)cr;
                p = ps.Polygon();
                s = ps.Surface();
                l = e.Placement * ps.Location();
                return;
            }

            itcr.Next();
        }

        l = new Placement();
        p = null;
        s = null;
    }

    /// <summary>The <paramref name="index"/>-th polygon on a surface of the edge (from 1), with its surface and location; nulls if none.</summary>
    public static void PolygonOnSurface(in Edge e,
                                        out Polygon2D? p,
                                        out Surface? s,
                                        out Placement l,
                                        int index)
    {
        int i = 0;

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnSurface())
            {
                PolygonOnSurface ps = (PolygonOnSurface)cr;
                i++;
                if (i > index)
                {
                    break;
                }

                if (i == index)
                {
                    p = ps.Polygon();
                    s = ps.Surface();
                    l = e.Placement * ps.Location();
                    return;
                }
            }

            itcr.Next();
        }

        l = new Placement();
        p = null;
        s = null;
    }

    /// <summary>The polygon of the edge on the triangulation <paramref name="t"/> with location <paramref name="L"/>, or null.</summary>
    public static PolygonOnTriangulation? PolygonOnTriangulation(in Edge e,
                                                                      Triangulation? t,
                                                                      in Placement L)
    {
        Placement l = L.Predivided(e.Placement);
        bool eisreversed = (e.Orientation == Reversed);

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnTriangulation(t, l))
            {
                if (cr.IsPolygonOnClosedTriangulation && eisreversed)
                {
                    return cr.PolygonOnTriangulation2();
                }
                else
                {
                    return cr.PolygonOnTriangulation();
                }
            }

            itcr.Next();
        }

        return null;
    }

    /// <summary>The first polygon on a triangulation of the edge, with its triangulation and location; nulls if none.</summary>
    public static void PolygonOnTriangulation(in Edge e,
                                              out PolygonOnTriangulation? p,
                                              out Triangulation? t,
                                              out Placement l)
    {
        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnTriangulation())
            {
                PolygonOnTriangulationRepresentation pt = (PolygonOnTriangulationRepresentation)cr;
                p = pt.PolygonOnTriangulation();
                t = pt.Triangulation();
                l = e.Placement * pt.Location();
                return;
            }

            itcr.Next();
        }

        l = new Placement();
        p = null;
        t = null;
    }

    /// <summary>
    /// The <paramref name="index"/>-th polygon on a triangulation of the edge (from 1), with its triangulation and
    /// location; nulls if none.
    /// </summary>
    public static void PolygonOnTriangulation(in Edge e,
                                              out PolygonOnTriangulation? p,
                                              out Triangulation? t,
                                              out Placement l,
                                              int index)
    {
        int i = 0;

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnTriangulation())
            {
                PolygonOnTriangulationRepresentation pt = (PolygonOnTriangulationRepresentation)cr;
                i++;
                if (i > index)
                {
                    break;
                }

                if (i == index)
                {
                    t = pt.Triangulation();
                    p = pt.PolygonOnTriangulation();
                    l = e.Placement * pt.Location();
                    return;
                }
            }

            itcr.Next();
        }

        l = new Placement();
        p = null;
        t = null;
    }

    /// <summary>
    /// True if the edge has two pcurves in the parametric space of the face (it is on the closing curve of a closed
    /// surface), or two polygons on its triangulation.
    /// </summary>
    public static bool IsClosed(in Edge e, in Face f)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        if (IsClosed(e, s, l))
        {
            return true;
        }

        Triangulation? t = Tool.Triangulation(f, out l);
        return IsClosed(e, t, l);
    }

    /// <summary>
    /// True if the edge has two pcurves in the parametric space of the surface (it is on the closing curve of a closed
    /// surface). False for planes.
    /// </summary>
    public static bool IsClosed(in Edge e, Surface? s, in Placement L)
    {
        // modified by NIZNHY-PKV Fri Oct 17 12:16:58 2008f
        if (IsPlane(s))
        {
            return false;
        }

        Placement l = L.Predivided(e.Placement);

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(s, l) && cr.IsCurveOnClosedSurface)
            {
                return true;
            }

            itcr.Next();
        }

        return false;
    }

    /// <summary>True if the edge has two arrays of indices in the triangulation.</summary>
    public static bool IsClosed(in Edge e, Triangulation? t, in Placement L)
    {
        if (t is null)
        {
            return false;
        }

        Placement l = L.Predivided(e.Placement);

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnTriangulation(t, l) && cr.IsPolygonOnClosedTriangulation)
            {
                return true;
            }

            itcr.Next();
        }

        return false;
    }

    /// <summary>The tolerance of the edge, at least Precision::Confusion().</summary>
    public static double Tolerance(in Edge e)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        double p = te.Tolerance();
        double pMin = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (p > pMin)
        {
            return p;
        }
        else
        {
            return pMin;
        }
    }

    /// <summary>The SameParameter flag of the edge.</summary>
    public static bool SameParameter(in Edge e)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        return te.SameParameter();
    }

    /// <summary>The SameRange flag of the edge.</summary>
    public static bool SameRange(in Edge e)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        return te.SameRange();
    }

    /// <summary>True if the edge is degenerated.</summary>
    public static bool Degenerated(in Edge e)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        return te.Degenerated();
    }

    /// <summary>The range of the edge: of its 3D curve, else of its first curve on a surface; zeros if none.</summary>
    public static void Range(in Edge e, out double first, out double last)
    {
        //  set the range to all the representations
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurve3D)
            {
                Curve3DRepresentation CR = (Curve3DRepresentation)cr;
                if (CR.Curve3D() is not null)
                {
                    first = CR.First();
                    last = CR.Last();
                    return;
                }
            }
            else if (cr.IsCurveOnSurface())
            {
                GCurve CR = (GCurve)cr;
                first = CR.First();
                last = CR.Last();
                return;
            }

            itcr.Next();
        }

        first = last = 0.0;
    }

    /// <summary>The range of the edge's curve on the surface; the edge's range if it has none.</summary>
    /// <remarks>OCCT sets the edge's Modified flag here; the port does too, unless the edge is frozen.</remarks>
    public static void Range(in Edge e, Surface? s, in Placement L, out double first, out double last)
    {
        Placement l = L.Predivided(e.Placement);

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        first = last = 0.0;

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(s, l))
            {
                CurveOnSurfaceRepresentation CR = (CurveOnSurfaceRepresentation)cr;
                CR.Range(out first, out last);
                break;
            }

            itcr.Next();
        }

        if (!itcr.More())
        {
            Range(e, out first, out last);
        }

        // OCCT (occt-issues.md): a query that sets the Modified flag. Kerf: frozen: not on a frozen edge.
        if (!e.TShape()!.IsFrozen)
        {
            e.TShape()!.Modified(true);
        }
    }

    /// <summary>The range of the edge's curve on the face; the edge's range if it has none.</summary>
    public static void Range(in Edge e, in Face f, out double first, out double last)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        Range(e, s, l, out first, out last);
    }

    /// <summary>
    /// The UV points at the ends of the edge's curve on the surface; for a plane without one, the projections of the
    /// vertices; zeros otherwise.
    /// </summary>
    public static void UVPoints(in Edge e,
                                Surface? s,
                                in Placement L,
                                out Point2D pFirst,
                                out Point2D pLast)
    {
        Placement l = L.Predivided(e.Placement);
        bool eisreversed = (e.Orientation == Reversed);

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(s, l))
            {
                if (cr.IsCurveOnClosedSurface && eisreversed)
                {
                    CurveOnClosedSurface CR = (CurveOnClosedSurface)cr;
                    CR.UVPoints2(out pFirst, out pLast);
                }
                else
                {
                    CurveOnSurfaceRepresentation CR = (CurveOnSurfaceRepresentation)cr;
                    CR.UVPoints(out pFirst, out pLast);
                }

                return;
            }

            itcr.Next();
        }

        // for planar surface project the vertices
        // modif 21-05-97 : for RectangularTrimmedSurface, project the vertices
        PlaneSurface? gp;
        TrimmedSurface? grts = s as TrimmedSurface;
        if (grts is not null)
        {
            gp = grts.BasisSurface as PlaneSurface;
        }
        else
        {
            gp = s as PlaneSurface;
        }
        // end of modification from 21-05-97

        if (gp is not null)
        {
            // get the two vertices
            Exploration.Vertices(e, out Vertex vf, out Vertex vl);

            Placement linverted = L.Inverted();
            vf.Move(linverted, false);
            vl.Move(linverted, false);
            double u, v;
            Plane3D pln = gp.Plane;

            u = v = 0.0;
            if (!vf.IsNull)
            {
                Point3D pf = Tool.Pnt(vf);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(pln, pf, out u, out v);
            }

            pFirst = new Point2D(u, v);

            u = v = 0.0;
            if (!vl.IsNull)
            {
                Point3D pl = Tool.Pnt(vl);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(pln, pl, out u, out v);
            }

            pLast = new Point2D(u, v);
        }
        else
        {
            pFirst = new Point2D(0.0, 0.0);
            pLast = new Point2D(0.0, 0.0);
        }
    }

    /// <summary>The UV points at the ends of the edge's curve on the face.</summary>
    public static void UVPoints(in Edge e, in Face f, out Point2D pFirst, out Point2D pLast)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        Edge localEdge = e;
        if (f.Orientation == Reversed)
        {
            localEdge.Reverse();
        }

        UVPoints(localEdge, s, l, out pFirst, out pLast);
    }

    /// <summary>Sets the UV points at the ends of the edge's curve on the surface.</summary>
    /// <remarks>Internal: it changes the edge, and throws <see cref="ImmutableObjectException"/> on a frozen one.</remarks>
    internal static void SetUVPoints(in Edge e,
                                     Surface? s,
                                     in Placement L,
                                     in Point2D pFirst,
                                     in Point2D pLast)
    {
        Placement l = L.Predivided(e.Placement);
        bool eisreversed = (e.Orientation == Reversed);

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(s, l))
            {
                if (cr.IsCurveOnClosedSurface && eisreversed)
                {
                    CurveOnClosedSurface cs = (CurveOnClosedSurface)cr;
                    cs.SetUVPoints2(pFirst, pLast);
                }
                else
                {
                    CurveOnSurfaceRepresentation cs = (CurveOnSurfaceRepresentation)cr;
                    cs.SetUVPoints(pFirst, pLast);
                }
            }

            itcr.Next();
        }
    }

    /// <summary>Sets the UV points at the ends of the edge's curve on the face.</summary>
    /// <remarks>Internal: it changes the edge, and throws <see cref="ImmutableObjectException"/> on a frozen one.</remarks>
    internal static void SetUVPoints(in Edge e, in Face f, in Point2D pFirst, in Point2D pLast)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        Edge localEdge = e;
        if (f.Orientation == Reversed)
        {
            localEdge.Reverse();
        }

        SetUVPoints(localEdge, s, l, pFirst, pLast);
    }

    /// <summary>True if the edge has a continuity between the surfaces of the two faces.</summary>
    public static bool HasContinuity(in Edge e, in Face f1, in Face f2)
    {
        Surface? s1 = Tool.Surface(f1, out Placement l1);
        Surface? s2 = Tool.Surface(f2, out Placement l2);
        return HasContinuity(e, s1, s2, l1, l2);
    }

    /// <summary>The continuity of the edge between the surfaces of the two faces (C0 if none is stored).</summary>
    public static Continuity Continuity(in Edge e, in Face f1, in Face f2)
    {
        Surface? s1 = Tool.Surface(f1, out Placement l1);
        Surface? s2 = Tool.Surface(f2, out Placement l2);
        return Continuity(e, s1, s2, l1, l2);
    }

    /// <summary>True if the edge has a continuity between the two surfaces.</summary>
    public static bool HasContinuity(in Edge e,
                                     Surface? s1,
                                     Surface? s2,
                                     in Placement L1,
                                     in Placement L2)
    {
        Placement eloc = e.Placement;
        Placement l1 = L1.Predivided(eloc);
        Placement l2 = L2.Predivided(eloc);

        // find the representation
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsRegularity(s1, s2, l1, l2))
            {
                return true;
            }

            itcr.Next();
        }

        return false;
    }

    /// <summary>The continuity of the edge between the two surfaces (C0 if none is stored).</summary>
    public static Continuity Continuity(in Edge e,
                                           Surface? s1,
                                           Surface? s2,
                                           in Placement L1,
                                           in Placement L2)
    {
        Placement l1 = L1.Predivided(e.Placement);
        Placement l2 = L2.Predivided(e.Placement);

        // find the representation
        // OCCT reads ChangeCurves(), which the port guards against frozen edges: Curves() is the same list
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)e.TShape()!).Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsRegularity(s1, s2, l1, l2))
            {
                return cr.Continuity();
            }

            itcr.Next();
        }

        return C0;
    }

    /// <summary>True if the edge has a continuity between some two surfaces.</summary>
    public static bool HasContinuity(in Edge e)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        for (; itcr.More(); itcr.Next())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsRegularity())
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>The highest continuity stored on the edge (C0 if none).</summary>
    public static Continuity MaxContinuity(in Edge edge)
    {
        Continuity maxCont = C0;
        // OCCT reads ChangeCurves(), which the port guards against frozen edges: Curves() is the same list
        for (var reprIter = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)edge.TShape()!).Curves());
             reprIter.More();
             reprIter.Next())
        {
            CurveRepresentation repr = reprIter.Value();
            if (repr.IsRegularity())
            {
                Continuity cont = repr.Continuity();
                if ((int)cont > (int)maxCont)
                {
                    maxCont = cont;
                }
            }
        }

        return maxCont;
    }

    /// <summary>The 3D point of the vertex, placed.</summary>
    /// <exception cref="NullObjectException">The vertex is null.</exception>
    public static Point3D Pnt(in Vertex v)
    {
        BrepTVertex? tv = (BrepTVertex?)v.TShape();

        if (tv is null)
        {
            throw new NullObjectException("BRep_Tool:: TopoDS_Vertex hasn't gp_Pnt");
        }

        Point3D p = tv.Pnt();
        if (v.Placement.IsIdentity)
        {
            return p;
        }

        return p.Transformed(v.Placement.Transformation);
    }

    /// <summary>The tolerance of the vertex, at least Precision::Confusion().</summary>
    /// <exception cref="NullObjectException">The vertex is null.</exception>
    public static double Tolerance(in Vertex v)
    {
        BrepTVertex? tVert = (BrepTVertex?)v.TShape();

        if (tVert is null)
        {
            throw new NullObjectException("BRep_Tool:: TopoDS_Vertex hasn't gp_Pnt");
        }

        double p = tVert.Tolerance();
        double pMin = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (p > pMin)
        {
            return p;
        }
        else
        {
            return pMin;
        }
    }

    /// <summary>
    /// The parameter of the vertex on the edge, if it has one: true, with the parameter. On false the port sets
    /// <paramref name="param"/> to 0 (OCCT leaves it unchanged).
    /// </summary>
    public static bool Parameter(in Vertex v, in Edge e, out double param)
    {
        param = 0.0;
        // Search the vertex in the edge

        bool rev = false;
        Shape vf = new Shape();
        Orientation orient = Internal;

        var itv = new Iterator(e.Oriented(Forward));

        // if the edge has no vertices
        // and is degenerated use the vertex orientation
        // RLE, june 94

        if (!itv.More() && Tool.Degenerated(e))
        {
            orient = v.Orientation;
        }

        while (itv.More())
        {
            Shape vcur = itv.Value();
            if (v.IsSame(vcur))
            {
                if (vf.IsNull)
                {
                    vf = vcur;
                }
                else
                {
                    rev = e.Orientation == Reversed;
                    if (vcur.Orientation == v.Orientation)
                    {
                        vf = vcur;
                    }
                }
            }

            itv.Next();
        }

        if (!vf.IsNull)
        {
            orient = vf.Orientation;
        }

        double f, l;

        if (orient == Forward)
        {
            Tool.Range(e, out f, out l);
            param = (rev) ? l : f;
            return true;
        }

        else if (orient == Reversed)
        {
            Tool.Range(e, out f, out l);
            param = (rev) ? f : l;
            return true;
        }

        else
        {
            Curve3D? c = Tool.Curve(e, out Placement L, out f, out l);
            L = L.Predivided(v.Placement);
            if (c is not null || Tool.Degenerated(e))
            {
                BrepTVertex tv = (BrepTVertex)v.TShape()!;
                var itpr = new NCollection_List<PointRepresentation>.Iterator(tv.Points());

                while (itpr.More())
                {
                    PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnCurve(c, L))
                    {
                        double p = pr.Parameter();
                        double res = p; // SVV 4 nov 99 - to avoid warnings on Linux
                        if (c is not null)
                        {
                            // Closed curves RLE 16 june 94
                            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f))
                            {
                                param = pr.Parameter(); // p;
                                return true;
                            }

                            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
                            {
                                param = pr.Parameter(); // p;
                                return true;
                            }

                            Point3D pf = c.Value(f).Transformed(L.Transformation);
                            Point3D pl = c.Value(l).Transformed(L.Transformation);
                            double tol = Tool.Tolerance(v);
                            if (pf.DistanceTo(pl) < tol)
                            {
                                if (pf.DistanceTo(Tool.Pnt(v)) < tol)
                                {
                                    if (v.Orientation == Forward)
                                    {
                                        res = f; // p = f;
                                    }
                                    else
                                    {
                                        res = l; // p = l;
                                    }
                                }
                            }
                        }

                        param = res; // p;
                        return true;
                    }

                    itpr.Next();
                }
            }
            else
            {
                // no 3d curve !!
                // let us try with the first pcurve
                Tool.CurveOnSurface(e, out Curve2D? pc, out Surface? s, out L, out f, out l);
                L = L.Predivided(v.Placement);
                BrepTVertex tv = (BrepTVertex)v.TShape()!;
                var itpr = new NCollection_List<PointRepresentation>.Iterator(tv.Points());

                while (itpr.More())
                {
                    PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnCurveOnSurface(pc, s, L))
                    {
                        double p = pr.Parameter();
                        // Closed curves RLE 16 june 94
                        if (pc!.IsClosed)
                        {
                            if ((p == pc.FirstParameter) || (p == pc.LastParameter))
                            {
                                if (v.Orientation == Forward)
                                {
                                    p = pc.FirstParameter;
                                }
                                else
                                {
                                    p = pc.LastParameter;
                                }
                            }
                        }

                        param = p;
                        return true;
                    }

                    itpr.Next();
                }
            }
        }

        return false;
    }

    /// <summary>The parameter of the vertex on the edge.</summary>
    /// <exception cref="NoSuchObjectException">The vertex has no parameter on the edge.</exception>
    public static double Parameter(in Vertex v, in Edge e)
    {
        if (Parameter(v, e, out double p))
        {
            return p;
        }

        throw new NoSuchObjectException("BRep_Tool:: no parameter on edge");
    }

    /// <summary>The parameter of the vertex on the edge's curve on the face.</summary>
    /// <exception cref="NoSuchObjectException">The vertex has no parameter on the edge.</exception>
    public static double Parameter(in Vertex v, in Edge e, in Face f)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        return Tool.Parameter(v, e, s, l);
    }

    /// <summary>The parameter of the vertex on the edge's curve on the surface.</summary>
    /// <exception cref="NoSuchObjectException">The vertex has no parameter on the edge.</exception>
    public static double Parameter(in Vertex v, in Edge e, Surface? s, in Placement L)
    {
        // Search the vertex in the edge

        bool rev = false;
        Shape vf = new Shape();
        var itv = new Iterator(e.Oriented(Forward));

        while (itv.More())
        {
            if (v.IsSame(itv.Value()))
            {
                if (vf.IsNull)
                {
                    vf = itv.Value();
                }
                else
                {
                    rev = e.Orientation == Reversed;
                    if (itv.Value().Orientation == v.Orientation)
                    {
                        vf = itv.Value();
                    }
                }
            }

            itv.Next();
        }

        Orientation orient = Internal;
        if (!vf.IsNull)
        {
            orient = vf.Orientation;
        }

        double f, l;

        if (orient == Forward)
        {
            Tool.Range(e, s, L, out f, out l);
            return (rev) ? l : f;
        }

        else if (orient == Reversed)
        {
            Tool.Range(e, s, L, out f, out l);
            return (rev) ? f : l;
        }

        else
        {
            Curve2D? pc = Tool.CurveOnSurface(e, s, L, out f, out l);
            BrepTVertex tv = (BrepTVertex)v.TShape()!;
            var itpr = new NCollection_List<PointRepresentation>.Iterator(tv.Points());

            while (itpr.More())
            {
                if (itpr.Value().IsPointOnCurveOnSurface(pc, s, L))
                {
                    return itpr.Value().Parameter();
                }

                itpr.Next();
            }
        }

        //----------------------------------------------------------

        Curve3D? c = Tool.Curve(e, out Placement l1, out f, out l);
        l1 = l1.Predivided(v.Placement);
        if (c is not null || Degenerated(e))
        {
            BrepTVertex tv = (BrepTVertex)v.TShape()!;
            var itpr = new NCollection_List<PointRepresentation>.Iterator(tv.Points());

            while (itpr.More())
            {
                PointRepresentation pr = itpr.Value();
                if (pr.IsPointOnCurve(c, l1))
                {
                    double p = pr.Parameter();
                    double res = p;
                    if (c is not null)
                    {
                        // Closed curves RLE 16 june 94
                        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f))
                        {
                            return res;
                        }

                        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
                        {
                            return res;
                        }

                        Point3D pf = c.Value(f).Transformed(l1.Transformation);
                        Point3D pl = c.Value(l).Transformed(l1.Transformation);
                        double tol = Tool.Tolerance(v);
                        if (pf.DistanceTo(pl) < tol)
                        {
                            if (pf.DistanceTo(Tool.Pnt(v)) < tol)
                            {
                                if (v.Orientation == Forward)
                                {
                                    res = f;
                                }
                                else
                                {
                                    res = l;
                                }
                            }
                        }
                    }

                    return res;
                }

                itpr.Next();
            }
        }

        //----------------------------------------------------------

        throw new NoSuchObjectException("BRep_Tool:: no parameter on edge");
    }

    /// <summary>The parameters of the vertex on the face.</summary>
    /// <exception cref="NoSuchObjectException">The vertex has no parameters on the face.</exception>
    public static Point2D Parameters(in Vertex v, in Face f)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        l = l.Predivided(v.Placement);
        BrepTVertex tv = (BrepTVertex)v.TShape()!;
        var itpr = new NCollection_List<PointRepresentation>.Iterator(tv.Points());

        // It is checked if there is PointRepresentation (case non Manifold)
        while (itpr.More())
        {
            if (itpr.Value().IsPointOnSurface(s, l))
            {
                return new Point2D(itpr.Value().Parameter(), itpr.Value().Parameter2());
            }

            itpr.Next();
        }

        Vertex vf, vl;
        Edge e;
        // Otherwise the edges are searched (PMN 4/06/97) It is not possible to succeed 999/1000!
        // even if often there is a way to make more economically than above...
        var exp = new Explorer();
        for (exp.Init(f, ShapeType.Edge); exp.More(); exp.Next())
        {
            e = ShapeCasts.Edge(exp.Current());
            Exploration.Vertices(e, out vf, out vl);
            if ((v.IsSame(vf)) || (v.IsSame(vl)))
            {
                UVPoints(e, f, out Point2D pf, out Point2D pl);
                if (v.IsSame(vf))
                {
                    return pf;
                }
                else
                {
                    return pl; // Ambiguity (natural) for degenerated edges.
                }
            }
        }

        throw new NoSuchObjectException("BRep_Tool:: no parameters on surface");
    }

    /// <summary>
    /// True for a shell whose edges are each shared by two of its faces, a wire whose vertices are each shared by two
    /// of its edges, or an edge whose two vertices are the same; for other shapes, the Closed flag.
    /// </summary>
    public static bool IsClosed(in Shape shape)
    {
        if (shape.Type == ShapeType.Shell)
        {
            return NoFreeEdge(shape);
        }
        else if (shape.Type == ShapeType.Wire)
        {
            var map = new HashSet<Shape>(ShapeHasher.Instance);
            var exp = new Explorer(shape.Oriented(Forward), ShapeType.Vertex);
            bool hasBound = false;
            for (; exp.More(); exp.Next())
            {
                Shape v = exp.Current();
                if (v.Orientation == Internal || v.Orientation == External)
                {
                    continue;
                }

                hasBound = true;
                if (!map.Add(v))
                {
                    map.Remove(v);
                }
            }

            return hasBound && map.Count == 0;
        }
        else if (shape.Type == ShapeType.Edge)
        {
            Exploration.Vertices(ShapeCasts.Edge(shape), out Vertex vFirst, out Vertex vLast);
            return !vFirst.IsNull && vFirst.IsSame(vLast);
        }

        return shape.ClosedFlag;
    }

    /// <summary>True for a face taken as a one-face shell: each of its edges is met twice.</summary>
    internal static bool IsClosedAsFace(in Shape face) => NoFreeEdge(face);

    private static bool NoFreeEdge(in Shape shape)
    {
        var map = new HashSet<Shape>(ShapeHasher.Instance);
        var exp = new Explorer(shape.Oriented(Forward), ShapeType.Edge);
        bool hasBound = false;
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            if (Tool.Degenerated(e) || e.Orientation == Internal || e.Orientation == External)
            {
                continue;
            }

            hasBound = true;
            if (!map.Add(e))
            {
                map.Remove(e);
            }
        }

        return hasBound && map.Count == 0;
    }

    // modified by NIZNHY-PKV Fri Oct 17 14:09:58 2008 f
    private static bool IsPlane(Surface? aS)
    {
        bool bRet;
        PlaneSurface? gp;
        TrimmedSurface? grts = aS as TrimmedSurface;
        OffsetSurface? gofs = aS as OffsetSurface;

        if (gofs is not null)
        {
            gp = gofs.BasisSurface as PlaneSurface;
        }
        else if (grts is not null)
        {
            gp = grts.BasisSurface as PlaneSurface;
        }
        else
        {
            gp = aS as PlaneSurface;
        }

        bRet = gp is not null;

        return bRet;
    }

    /// <summary>The largest tolerance of the sub-shapes of the given type (FACE, EDGE or VERTEX); 0 otherwise.</summary>
    public static double MaxTolerance(in Shape shape, ShapeType subShape)
    {
        double tol = 0.0;

        // Explorer Shape-Subshape.
        var expSS = new Explorer(shape, subShape);
        if (subShape == ShapeType.Face)
        {
            for (; expSS.More(); expSS.Next())
            {
                Shape currentSubShape = expSS.Current();
                tol = Max(tol, Tolerance(ShapeCasts.Face(currentSubShape)));
            }
        }
        else if (subShape == ShapeType.Edge)
        {
            for (; expSS.More(); expSS.Next())
            {
                Shape currentSubShape = expSS.Current();
                tol = Max(tol, Tolerance(ShapeCasts.Edge(currentSubShape)));
            }
        }
        else if (subShape == ShapeType.Vertex)
        {
            for (; expSS.More(); expSS.Next())
            {
                Shape currentSubShape = expSS.Current();
                tol = Max(tol, Tolerance(ShapeCasts.Vertex(currentSubShape)));
            }
        }

        return tol;
    }
}
