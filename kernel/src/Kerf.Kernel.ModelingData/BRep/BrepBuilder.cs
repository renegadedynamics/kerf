// Port of: src/ModelingData/TKBRep/BRep/BRep_Builder.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Builder.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Topology.Brep;

/// <summary>
/// Builds and updates BRep shapes: vertices, edges and faces with their geometry, and their tolerances, ranges,
/// flags and polygons.
/// </summary>
/// <remarks>
/// <para>
/// Internal to OCCT.NET but for the face of a triangulation, which the modelling layer makes when it reads a mesh (Kerf
/// unit 10B; engine, internal, in unit 10L). Every update throws
/// <see cref="LockedShapeException"/> on a locked shape, and in the port every frozen shape is locked, unless a working
/// overlay is open (<see cref="WorkingOverlay"/>, Kerf unit 10H1): then the update of a frozen shape goes into the
/// overlay's working copy of it, and the frozen shape doesn't change. A frozen child can be added to an unfrozen parent. Every curve, surface, triangulation or polygon the builder stores is frozen.
/// </para>
/// <para>
/// The Make methods take the shape by <c>ref</c>: some read it first (they throw if it holds a locked shape). The
/// Update methods change only the TShape and take it by <c>in</c>.
/// </para>
/// </remarks>
public sealed class BrepBuilder : Builder
{
    public BrepBuilder()
    {
    }

    // Insert a 3d curve <C> with location <L> in a list of curve representations <lcr>
    private static void UpdateCurves(NCollection_List<CurveRepresentation> lcr, Curve3D? c, in Placement L)
    {
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);
        GCurve? gc = null;
        double f = 0.0, l = 0.0;

        while (itcr.More())
        {
            gc = itcr.Value() as GCurve;
            if (gc is not null)
            {
                gc.Range(out f, out l);
                if (gc.IsCurve3D)
                {
                    break;
                }
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            itcr.Value().Curve3D(c);
            itcr.Value().Location(L);
        }
        else
        {
            Curve3DRepresentation c3d = new Curve3DRepresentation(c, L);
            // test if there is already a range
            if (gc is not null)
            {
                c3d.SetRange(f, l);
            }

            lcr.Append(c3d);
        }
    }

    // Insert a pcurve <C> on surface <S> with location <L> in a list of curve representations <lcr>.
    // Remove the pcurve on <S> from <lcr> if <C> is null
    private static void UpdateCurves(NCollection_List<CurveRepresentation> lcr,
                                     Curve2D? c,
                                     Surface? s,
                                     in Placement L)
    {
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);
        double f = -global::Kerf.Engine.Foundation.Precision.Infinite(), l = global::Kerf.Engine.Foundation.Precision.Infinite();
        // search the range of the 3d curve
        // and remove any existing representation

        while (itcr.More())
        {
            GCurve? gc = itcr.Value() as GCurve;
            if (gc is not null)
            {
                if (gc.IsCurve3D)
                {
                    //      if (!C.IsNull()) { //xpu031198, degenerate edge
                    // xpu151298 : parameters can be set for null curves
                    //             see lbo & flo, to determine whether range is defined
                    //             compare first and last parameters with default values.
                    gc.Range(out f, out l);
                }

                if (gc.IsCurveOnSurface(s, L))
                {
                    // remove existing curve on surface
                    lcr.Remove(itcr);
                }
                else
                {
                    itcr.Next();
                }
            }
            else
            {
                itcr.Next();
            }
        }

        if (c is not null)
        {
            CurveOnSurfaceRepresentation cos = new CurveOnSurfaceRepresentation(c, s, L);
            cos.Range(out double fCur, out double lCur);
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f))
            {
                fCur = f;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
            {
                lCur = l;
            }

            cos.SetRange(fCur, lCur);
            lcr.Append(cos);
        }
    }

    // Insert a pcurve <C> on surface <S> with location <L> and UV points <Pf>, <Pl> in a list of curve
    // representations <lcr>. Remove the pcurve on <S> from <lcr> if <C> is null
    private static void UpdateCurves(NCollection_List<CurveRepresentation> lcr,
                                     Curve2D? c,
                                     Surface? s,
                                     in Placement L,
                                     in Point2D pf,
                                     in Point2D pl)
    {
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);
        double f = -global::Kerf.Engine.Foundation.Precision.Infinite(), l = global::Kerf.Engine.Foundation.Precision.Infinite();
        // search the range of the 3d curve
        // and remove any existing representation

        while (itcr.More())
        {
            GCurve? gc = itcr.Value() as GCurve;
            if (gc is not null)
            {
                if (gc.IsCurve3D)
                {
                    //      if (!C.IsNull()) { //xpu031198, degenerate edge
                    // xpu151298 : parameters can be set for null curves
                    //             see lbo & flo, to determine whether range is defined
                    //             compare first and last parameters with default values.
                    gc.Range(out f, out l);
                }

                if (gc.IsCurveOnSurface(s, L))
                {
                    // remove existing curve on surface
                    lcr.Remove(itcr);
                }
                else
                {
                    itcr.Next();
                }
            }
            else
            {
                itcr.Next();
            }
        }

        if (c is not null)
        {
            CurveOnSurfaceRepresentation cos = new CurveOnSurfaceRepresentation(c, s, L);
            cos.Range(out double fCur, out double lCur);
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f))
            {
                fCur = f;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
            {
                lCur = l;
            }

            cos.SetRange(fCur, lCur);
            cos.SetUVPoints(pf, pl);
            lcr.Append(cos);
        }
    }

    // Insert two pcurves <C1,C2> on surface <S> with location <L> in a list of curve representations <lcr>.
    // Remove the pcurves on <S> from <lcr> if <C1> or <C2> is null
    private static void UpdateCurves(NCollection_List<CurveRepresentation> lcr,
                                     Curve2D? c1,
                                     Curve2D? c2,
                                     Surface? s,
                                     in Placement L)
    {
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);
        double f = -global::Kerf.Engine.Foundation.Precision.Infinite(), l = global::Kerf.Engine.Foundation.Precision.Infinite();

        while (itcr.More())
        {
            GCurve? gc = itcr.Value() as GCurve;
            if (gc is not null)
            {
                if (gc.IsCurve3D)
                {
                    gc.Range(out f, out l);
                }

                bool iscos = gc.IsCurveOnSurface(s, L);
                if (iscos)
                {
                    break;
                }
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            lcr.Remove(itcr);
        }

        if (c1 is not null && c2 is not null)
        {
            CurveOnClosedSurface cos = new CurveOnClosedSurface(c1, c2, s, L, C0);
            cos.Range(out double fCur, out double lCur);
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f))
            {
                fCur = f;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
            {
                lCur = l;
            }

            cos.SetRange(fCur, lCur);
            lcr.Append(cos);
        }
    }

    // Insert two pcurves <C1,C2> on surface <S> with location <L> and the UV points of the second <Pf>, <Pl> in a
    // list of curve representations <lcr>. Remove the pcurves on <S> from <lcr> if <C1> or <C2> is null
    private static void UpdateCurves(NCollection_List<CurveRepresentation> lcr,
                                     Curve2D? c1,
                                     Curve2D? c2,
                                     Surface? s,
                                     in Placement L,
                                     in Point2D pf,
                                     in Point2D pl)
    {
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);
        double f = -global::Kerf.Engine.Foundation.Precision.Infinite(), l = global::Kerf.Engine.Foundation.Precision.Infinite();

        while (itcr.More())
        {
            GCurve? gc = itcr.Value() as GCurve;
            if (gc is not null)
            {
                if (gc.IsCurve3D)
                {
                    gc.Range(out f, out l);
                }

                bool iscos = gc.IsCurveOnSurface(s, L);
                if (iscos)
                {
                    break;
                }
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            lcr.Remove(itcr);
        }

        if (c1 is not null && c2 is not null)
        {
            CurveOnClosedSurface cos = new CurveOnClosedSurface(c1, c2, s, L, C0);
            cos.Range(out double fCur, out double lCur);
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f))
            {
                fCur = f;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
            {
                lCur = l;
            }

            cos.SetRange(fCur, lCur);
            cos.SetUVPoints2(pf, pl);
            lcr.Append(cos);
        }
    }

    private static void UpdateCurves(NCollection_List<CurveRepresentation> lcr,
                                     Surface? s1,
                                     Surface? s2,
                                     in Placement l1,
                                     in Placement l2,
                                     Continuity c)
    {
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);
        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            bool isregu = cr.IsRegularity(s1, s2, l1, l2);
            if (isregu)
            {
                break;
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            cr.Continuity(c);
        }
        else
        {
            CurveOn2Surfaces cos = new CurveOn2Surfaces(s1, s2, l1, l2, c);
            lcr.Append(cos);
        }
    }

    private static void UpdatePoints(NCollection_List<PointRepresentation> lpr, double p, Curve3D? c, in Placement l)
    {
        var itpr = new NCollection_List<PointRepresentation>.Iterator(lpr);
        while (itpr.More())
        {
            PointRepresentation pr = itpr.Value();
            bool isponc = pr.IsPointOnCurve(c, l);
            if (isponc)
            {
                break;
            }

            itpr.Next();
        }

        if (itpr.More())
        {
            PointRepresentation pr = itpr.Value();
            pr.Parameter(p);
        }
        else
        {
            PointOnCurve poc = new PointOnCurve(p, c, l);
            lpr.Append(poc);
        }
    }

    private static void UpdatePoints(NCollection_List<PointRepresentation> lpr,
                                     double p,
                                     Curve2D? pc,
                                     Surface? s,
                                     in Placement l)
    {
        var itpr = new NCollection_List<PointRepresentation>.Iterator(lpr);
        while (itpr.More())
        {
            PointRepresentation pr = itpr.Value();
            bool isponcons = pr.IsPointOnCurveOnSurface(pc, s, l);
            if (isponcons)
            {
                break;
            }

            itpr.Next();
        }

        if (itpr.More())
        {
            PointRepresentation pr = itpr.Value();
            pr.Parameter(p);
        }
        else
        {
            PointOnCurveOnSurface pocs = new PointOnCurveOnSurface(p, pc, s, l);
            lpr.Append(pocs);
        }
    }

    private static void UpdatePoints(NCollection_List<PointRepresentation> lpr,
                                     double p1,
                                     double p2,
                                     Surface? s,
                                     in Placement l)
    {
        var itpr = new NCollection_List<PointRepresentation>.Iterator(lpr);
        while (itpr.More())
        {
            PointRepresentation pr = itpr.Value();
            bool ispons = pr.IsPointOnSurface(s, l);
            if (ispons)
            {
                break;
            }

            itpr.Next();
        }

        if (itpr.More())
        {
            PointRepresentation pr = itpr.Value();
            pr.Parameter(p1);
            //    pr->Parameter(p2); // skv
            pr.Parameter2(p2); // skv
        }
        else
        {
            PointOnSurface pos = new PointOnSurface(p1, p2, s, l);
            lpr.Append(pos);
        }
    }

    /// <summary>Makes an undefined face.</summary>
    internal void MakeFace(ref Face f)
    {
        BrepTFace tf = new BrepTFace();
        MakeShape(ref ShapeCasts.AsShapeRef(ref f), tf);
    }

    /// <summary>Makes a face with a surface (frozen here) and a tolerance.</summary>
    internal void MakeFace(ref Face f, Surface? s, double tol)
    {
        BrepTFace tf = new BrepTFace();
        // Kerf: inside a working overlay a variable holding a frozen shape takes the new TShape, as one holding its
        // copy did (unit 10H5)
        if (!f.IsNull && WorkingOverlay.IsLocked(f.TShape()!))
        {
            throw new LockedShapeException("BRep_Builder::MakeFace");
        }

        tf.Surface(s);
        tf.Tolerance(tol);
        MakeShape(ref ShapeCasts.AsShapeRef(ref f), tf);
    }

    /// <summary>Makes a face with a triangulation (frozen here) and no surface.</summary>
    public void MakeFace(ref Face face, Triangulation? triangulation)
    {
        BrepTFace tFace = new BrepTFace();
        // Kerf: inside a working overlay a variable holding a frozen shape takes the new TShape, as one holding its
        // copy did (unit 10H5)
        if (!face.IsNull && WorkingOverlay.IsLocked(face.TShape()!))
        {
            throw new LockedShapeException("BRep_Builder::MakeFace");
        }

        tFace.Triangulation(triangulation);
        MakeShape(ref ShapeCasts.AsShapeRef(ref face), tFace);
    }

    /// <summary>Makes a face with a list of triangulations (frozen here), one of them active (the first if null).</summary>
    internal void MakeFace(ref Face face,
                           NCollection_List<Triangulation> triangulations,
                           Triangulation? activeTriangulation = null)
    {
        BrepTFace tFace = new BrepTFace();
        // Kerf: inside a working overlay a variable holding a frozen shape takes the new TShape, as one holding its
        // copy did (unit 10H5)
        if (!face.IsNull && WorkingOverlay.IsLocked(face.TShape()!))
        {
            throw new LockedShapeException("BRep_Builder::MakeFace");
        }

        tFace.Triangulations(triangulations, activeTriangulation);
        MakeShape(ref ShapeCasts.AsShapeRef(ref face), tFace);
    }

    /// <summary>Makes a face with a surface (frozen here), a location for it and a tolerance.</summary>
    internal void MakeFace(ref Face f, Surface? s, in Placement l, double tol)
    {
        BrepTFace tf = new BrepTFace();
        // Kerf: inside a working overlay a variable holding a frozen shape takes the new TShape, as one holding its
        // copy did (unit 10H5)
        if (!f.IsNull && WorkingOverlay.IsLocked(f.TShape()!))
        {
            throw new LockedShapeException("BRep_Builder::MakeFace");
        }

        tf.Surface(s);
        tf.Tolerance(tol);
        tf.Location(l);
        MakeShape(ref ShapeCasts.AsShapeRef(ref f), tf);
    }

    /// <summary>Updates the face's surface (frozen here), its location and the tolerance.</summary>
    internal void UpdateFace(in Face f, Surface? s, in Placement l, double tol)
    {
        BrepTFace tf = (BrepTFace)f.TShape()!;
        if (WorkingOverlay.IsLocked(tf))
        {
            throw new LockedShapeException("BRep_Builder::UpdateFace");
        }

        tf.Surface(s);
        tf.Tolerance(tol);
        tf.Location(l.Predivided(f.Placement));
        f.TShape()!.Modified(true);
    }

    /// <summary>
    /// Changes the face's triangulation (frozen here). With <paramref name="toReset"/>, the face's list of
    /// triangulations becomes just this one; otherwise it becomes active, replacing the active one if not in the list.
    /// </summary>
    internal void UpdateFace(in Face face, Triangulation? triangulation, bool toReset = true)
        => UpdateFaceCore(face, triangulation, toReset, true);

    /// <summary>
    /// <see cref="UpdateFace(in Face, Triangulation?, bool)"/> without freezing the triangulation: the
    /// mesher's attach (BRepMesh_ShapeTool only, decision 488), so that its post-processor can set the triangulation's
    /// deflection and parameters, as OCCT's. The triangulation freezes with the face's graph when the result is
    /// published. (Until unit 10H6 named <c>UpdateFaceUnfrozen</c>; it concerns the mesh's freezing, not the face's:
    /// a frozen face takes it in a working overlay, as every update.)
    /// </summary>
    internal void UpdateFaceMesh(in Face face, Triangulation? triangulation, bool toReset = true)
        => UpdateFaceCore(face, triangulation, toReset, false);

    private static void UpdateFaceCore(in Face face, Triangulation? triangulation, bool toReset,
                                   bool toFreeze)
    {
        BrepTFace tFace = (BrepTFace)face.TShape()!;
        if (WorkingOverlay.IsLocked(tFace))
        {
            throw new LockedShapeException("BRep_Builder::UpdateFace");
        }

        tFace.Triangulation(triangulation, toReset, toFreeze);
        face.TShape()!.Modified(true);
    }

    /// <summary>Updates the face's tolerance.</summary>
    internal void UpdateFace(in Face f, double tol)
    {
        BrepTFace tf = (BrepTFace)f.TShape()!;
        if (WorkingOverlay.IsLocked(tf))
        {
            throw new LockedShapeException("BRep_Builder::UpdateFace");
        }

        tf.Tolerance(tol);
        f.TShape()!.Modified(true);
    }

    /// <summary>Sets the NaturalRestriction flag of the face.</summary>
    internal void NaturalRestriction(in Face f, bool n)
    {
        BrepTFace tf = (BrepTFace)f.TShape()!;
        if (WorkingOverlay.IsLocked(tf))
        {
            throw new LockedShapeException("BRep_Builder::NaturalRestriction");
        }

        tf.NaturalRestriction(n);
        f.TShape()!.Modified(true);
    }

    /// <summary>Makes an undefined edge.</summary>
    internal void MakeEdge(ref Edge e)
    {
        BrepTEdge te = new BrepTEdge();
        // Kerf: inside a working overlay a variable holding a frozen shape takes the new TShape, as one holding its
        // copy did (unit 10H5)
        if (!e.IsNull && WorkingOverlay.IsLocked(e.TShape()!))
        {
            throw new LockedShapeException("BRep_Builder::MakeEdge");
        }

        MakeShape(ref ShapeCasts.AsShapeRef(ref e), te);
    }

    /// <summary>Makes an edge with a 3D curve (frozen here) and a tolerance.</summary>
    internal void MakeEdge(ref Edge e, Curve3D? c, double tol)
    {
        MakeEdge(ref e);
        UpdateEdge(e, c, new Placement(), tol);
    }

    /// <summary>Makes an edge with a 3D curve (frozen here), a location for it and a tolerance.</summary>
    internal void MakeEdge(ref Edge e, Curve3D? c, in Placement l, double tol)
    {
        MakeEdge(ref e);
        UpdateEdge(e, c, l, tol);
    }

    /// <summary>Makes an edge with a 3D polygon (frozen here).</summary>
    internal void MakeEdge(ref Edge e, Polygon3D? p)
    {
        MakeEdge(ref e);
        UpdateEdge(e, p);
    }

    /// <summary>Makes an edge with a polygon on a triangulation (both frozen here).</summary>
    internal void MakeEdge(ref Edge e, PolygonOnTriangulation? p, Triangulation? t)
    {
        MakeEdge(ref e);
        UpdateEdge(e, p, t, new Placement());
    }

    /// <summary>Makes an edge with a polygon on a triangulation (both frozen here) with a location.</summary>
    internal void MakeEdge(ref Edge e, PolygonOnTriangulation? p, Triangulation? t, in Placement l)
    {
        MakeEdge(ref e);
        UpdateEdge(e, p, t, l);
    }

    /// <summary>Sets the edge's 3D curve (frozen here) and updates the tolerance.</summary>
    internal void UpdateEdge(in Edge e, Curve3D? c, double tol)
        => UpdateEdge(e, c, new Placement(), tol);

    /// <summary>Sets the edge's 3D curve (frozen here) with a location and updates the tolerance.</summary>
    internal void UpdateEdge(in Edge e, Curve3D? c, in Placement L, double tol)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(e.Placement);

        UpdateCurves(te.ChangeCurves(), c, l);

        te.UpdateTolerance(tol);
        te.Modified(true);
    }

    /// <summary>Sets the edge's curve on the face's surface (frozen here; removes it if null) and updates the tolerance.</summary>
    internal void UpdateEdge(in Edge e, Curve2D? c, in Face f, double tol)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        UpdateEdge(e, c, s, l, tol);
    }

    /// <summary>
    /// Sets the edge's two curves on the face's closed surface (frozen here; removes them if one is null) and updates
    /// the tolerance.
    /// </summary>
    internal void UpdateEdge(in Edge e, Curve2D? c1, Curve2D? c2, in Face f, double tol)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        UpdateEdge(e, c1, c2, s, l, tol);
    }

    /// <summary>Sets the edge's curve on the surface (frozen here; removes it if null) and updates the tolerance.</summary>
    internal void UpdateEdge(in Edge e, Curve2D? c, Surface? s, in Placement L, double tol)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(e.Placement);

        UpdateCurves(te.ChangeCurves(), c, s, l);

        te.UpdateTolerance(tol);
        te.Modified(true);
    }

    /// <summary>
    /// Sets the edge's curve on the surface (frozen here) with its UV end points, and updates the tolerance (for the
    /// second format, XML persistence).
    /// </summary>
    internal void UpdateEdge(in Edge e,
                             Curve2D? c,
                             Surface? s,
                             in Placement L,
                             double tol,
                             in Point2D pf,
                             in Point2D pl)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(e.Placement);

        UpdateCurves(te.ChangeCurves(), c, s, l, pf, pl);

        te.UpdateTolerance(tol);
        te.Modified(true);
    }

    /// <summary>
    /// Sets the edge's two curves on the closed surface (frozen here; removes them if one is null) and updates the
    /// tolerance.
    /// </summary>
    internal void UpdateEdge(in Edge e,
                             Curve2D? c1,
                             Curve2D? c2,
                             Surface? s,
                             in Placement L,
                             double tol)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(e.Placement);

        UpdateCurves(te.ChangeCurves(), c1, c2, s, l);

        te.UpdateTolerance(tol);
        te.Modified(true);
    }

    /// <summary>
    /// Sets the edge's two curves on the closed surface (frozen here) with the UV end points of the second, and updates
    /// the tolerance (for the second format, XML persistence).
    /// </summary>
    internal void UpdateEdge(in Edge e,
                             Curve2D? c1,
                             Curve2D? c2,
                             Surface? s,
                             in Placement L,
                             double tol,
                             in Point2D pf,
                             in Point2D pl)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(e.Placement);

        UpdateCurves(te.ChangeCurves(), c1, c2, s, l, pf, pl);

        te.UpdateTolerance(tol);
        te.Modified(true);
    }

    /// <summary>Sets the edge's 3D polygon (frozen here; removes it if null).</summary>
    internal void UpdateEdge(in Edge e, Polygon3D? p) => UpdateEdge(e, p, new Placement());

    /// <summary>Sets the edge's 3D polygon (frozen here; removes it if null) with a location.</summary>
    internal void UpdateEdge(in Edge e, Polygon3D? p, in Placement l) => UpdateEdgeCore(e, p, l, true);

    /// <summary>
    /// <see cref="UpdateEdge(in Edge, Polygon3D?, in Placement)"/> without freezing the polygon: the
    /// mesher's attach (BRepMesh_ShapeTool only, decision 488; see <see cref="UpdateFaceMesh"/>).
    /// </summary>
    internal void UpdateEdgeMesh(in Edge e, Polygon3D? p, in Placement l) => UpdateEdgeCore(e, p, l, false);

    private static void UpdateEdgeCore(in Edge e, Polygon3D? p, in Placement L, bool toFreeze)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            if (itcr.Value().IsPolygon3D)
            {
                if (p is null)
                {
                    lcr.Remove(itcr);
                }
                else
                {
                    ((Polygon3DRepresentation)itcr.Value()).Polygon3D(p, toFreeze);
                }

                te.Modified(true);
                return;
            }

            itcr.Next();
        }

        Placement l = L.Predivided(e.Placement);
        Polygon3DRepresentation p3d = new Polygon3DRepresentation(p, l, toFreeze);
        lcr.Append(p3d);

        te.Modified(true);
    }

    /// <summary>Sets the edge's polygon on the triangulation (both frozen here; removes it if null).</summary>
    internal void UpdateEdge(in Edge e, PolygonOnTriangulation? p, Triangulation? t)
        => UpdateEdge(e, p, t, new Placement());

    /// <summary>Sets the edge's polygon on the triangulation (both frozen here; removes it if null) with a location.</summary>
    internal void UpdateEdge(in Edge e, PolygonOnTriangulation? p, Triangulation? t, in Placement l)
        => UpdateEdgeCore(e, p, t, l, true);

    /// <summary>
    /// <see cref="UpdateEdge(in Edge, PolygonOnTriangulation?, Triangulation?, in Placement)"/>
    /// without freezing the polygon and the triangulation: the mesher's attach (BRepMesh_ShapeTool only, decision 488).
    /// </summary>
    internal void UpdateEdgeMesh(in Edge e, PolygonOnTriangulation? p, Triangulation? t,
                                     in Placement l)
        => UpdateEdgeCore(e, p, t, l, false);

    private static void UpdateEdgeCore(in Edge e, PolygonOnTriangulation? p, Triangulation? t,
                                   in Placement L, bool toFreeze)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(e.Placement);

        bool isModified = false;

        NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            if (itcr.Value().IsPolygonOnTriangulation(t, l))
            {
                lcr.Remove(itcr);
                isModified = true;
                break;
            }

            itcr.Next();
        }

        if (p is not null)
        {
            PolygonOnTriangulationRepresentation pt = new PolygonOnTriangulationRepresentation(p, t, l, toFreeze);
            lcr.Append(pt);
            isModified = true;
        }

        if (isModified)
        {
            te.Modified(true);
        }
    }

    /// <summary>Sets the edge's two polygons on the triangulation (all frozen here; removes them if one is null).</summary>
    internal void UpdateEdge(in Edge e,
                             PolygonOnTriangulation? p1,
                             PolygonOnTriangulation? p2,
                             Triangulation? t)
        => UpdateEdge(e, p1, p2, t, new Placement());

    /// <summary>
    /// Sets the edge's two polygons on the triangulation (all frozen here; removes them if one is null) with a location.
    /// </summary>
    internal void UpdateEdge(in Edge e,
                             PolygonOnTriangulation? p1,
                             PolygonOnTriangulation? p2,
                             Triangulation? t,
                             in Placement l)
        => UpdateEdgeCore(e, p1, p2, t, l, true);

    /// <summary>
    /// The seam form of <see cref="UpdateEdgeMesh(in Edge, PolygonOnTriangulation?, Triangulation?, in Placement)"/>
    /// (decision 488).
    /// </summary>
    internal void UpdateEdgeMesh(in Edge e,
                                     PolygonOnTriangulation? p1,
                                     PolygonOnTriangulation? p2,
                                     Triangulation? t,
                                     in Placement l)
        => UpdateEdgeCore(e, p1, p2, t, l, false);

    private static void UpdateEdgeCore(in Edge e,
                                   PolygonOnTriangulation? p1,
                                   PolygonOnTriangulation? p2,
                                   Triangulation? t,
                                   in Placement L,
                                   bool toFreeze)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(e.Placement);

        bool isModified = false;

        NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            if (itcr.Value().IsPolygonOnTriangulation(t, l)) // szv:was L
            {
                lcr.Remove(itcr);
                isModified = true;
                break;
            }

            itcr.Next();
        }

        if (p1 is not null && p2 is not null)
        {
            PolygonOnClosedTriangulation pt = new PolygonOnClosedTriangulation(p1, p2, t, l, toFreeze);
            lcr.Append(pt);
            isModified = true;
        }

        if (isModified)
        {
            te.Modified(true);
        }
    }

    /// <summary>Sets the edge's polygon on the face's surface (frozen here; removes it if null).</summary>
    internal void UpdateEdge(in Edge e, Polygon2D? p, in Face f)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        UpdateEdge(e, p, s, l);
    }

    /// <summary>Sets the edge's polygon on the surface (frozen here; removes it if null).</summary>
    internal void UpdateEdge(in Edge e, Polygon2D? p, Surface? s, in Placement L)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(e.Placement);

        NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();

        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);
        while (itcr.More())
        {
            if (itcr.Value().IsPolygonOnSurface(s, l))
            {
                break;
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            lcr.Remove(itcr);
        }

        if (p is not null)
        {
            PolygonOnSurface ps = new PolygonOnSurface(p, s, l);
            lcr.Append(ps);
        }

        te.Modified(true);
    }

    /// <summary>Sets the edge's two polygons on the face's closed surface (frozen here; removes them if one is null).</summary>
    internal void UpdateEdge(in Edge e, Polygon2D? p1, Polygon2D? p2, in Face f)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        UpdateEdge(e, p1, p2, s, l);
    }

    /// <summary>Sets the edge's two polygons on the closed surface (frozen here; removes them if one is null).</summary>
    internal void UpdateEdge(in Edge e, Polygon2D? p1, Polygon2D? p2, Surface? s, in Placement L)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(e.Placement);

        NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();

        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);
        while (itcr.More())
        {
            if (itcr.Value().IsPolygonOnSurface(s, l))
            {
                break;
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            lcr.Remove(itcr);
        }

        if (p1 is not null && p2 is not null)
        {
            // OCCT (occt-issues.md): the polygons are stored with the identity location, not l
            PolygonOnClosedSurface ps = new PolygonOnClosedSurface(p1, p2, s, new Placement());
            lcr.Append(ps);
        }

        te.Modified(true);
    }

    /// <summary>Raises the edge's tolerance to <paramref name="tol"/> if it is lower.</summary>
    internal void UpdateEdge(in Edge e, double tol)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateEdge");
        }

        te.UpdateTolerance(tol);
        te.Modified(true);
    }

    /// <summary>Sets the continuity of the edge between the surfaces of the two faces.</summary>
    internal void Continuity(in Edge e, in Face f1, in Face f2, Continuity c)
    {
        Surface? s1 = Tool.Surface(f1, out Placement l1);
        Surface? s2 = Tool.Surface(f2, out Placement l2);
        Continuity(e, s1, s2, l1, l2, c);
    }

    /// <summary>Sets the continuity of the edge between the two surfaces (frozen here).</summary>
    internal void Continuity(in Edge e,
                             Surface? s1,
                             Surface? s2,
                             in Placement L1,
                             in Placement L2,
                             Continuity c)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::Continuity");
        }

        Placement l1 = L1.Predivided(e.Placement);
        Placement l2 = L2.Predivided(e.Placement);

        UpdateCurves(te.ChangeCurves(), s1, s2, l1, l2, c);

        te.Modified(true);
    }

    /// <summary>Sets the SameParameter flag of the edge.</summary>
    internal void SameParameter(in Edge e, bool s)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::SameParameter");
        }

        te.SameParameter(s);
        te.Modified(true);
    }

    /// <summary>Sets the SameRange flag of the edge.</summary>
    internal void SameRange(in Edge e, bool s)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::SameRange");
        }

        te.SameRange(s);
        te.Modified(true);
    }

    /// <summary>Sets the Degenerated flag of the edge; a degenerated edge gets a null 3D curve.</summary>
    internal void Degenerated(in Edge e, bool d)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::Degenerated");
        }

        te.Degenerated(d);
        if (d)
        {
            // set a null 3d curve
            UpdateCurves(te.ChangeCurves(), (Curve3D?)null, e.Placement);
        }

        te.Modified(true);
    }

    /// <summary>Sets the range of the edge's curves: all of them, or only the 3D curve if <paramref name="only3d"/>.</summary>
    internal void Range(in Edge e, double first, double last, bool only3d = false)
    {
        //  set the range to all the representations if Only3d=FALSE
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::Range");
        }

        NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            GCurve? gc = itcr.Value() as GCurve;
            if (gc is not null && (!only3d || gc.IsCurve3D))
            {
                gc.SetRange(first, last);
            }

            itcr.Next();
        }

        te.Modified(true);
    }

    /// <summary>Sets the range of the edge's curve on the surface.</summary>
    /// <exception cref="DomainErrorException">The edge has no curve on the surface.</exception>
    internal void Range(in Edge e, Surface? s, in Placement L, double first, double last)
    {
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        if (WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::Range");
        }

        Placement l = L.Predivided(e.Placement);

        NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            GCurve? gc = itcr.Value() as GCurve;
            if (gc is not null && gc.IsCurveOnSurface(s, l))
            {
                gc.SetRange(first, last);
                break;
            }

            itcr.Next();
        }

        if (!itcr.More())
        {
            throw new DomainErrorException("BRep_Builder::Range, no pcurve");
        }

        te.Modified(true);
    }

    /// <summary>Sets the range of the edge's curve on the face.</summary>
    /// <exception cref="DomainErrorException">The edge has no curve on the face.</exception>
    internal void Range(in Edge e, in Face f, double first, double last)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        Range(e, s, l, first, last);
    }

    /// <summary>Adds to <paramref name="eout"/> the curves on surfaces and the continuities of <paramref name="ein"/>.</summary>
    internal void Transfert(in Edge ein, in Edge eout)
    {
        BrepTEdge te = (BrepTEdge)ein.TShape()!;
        double tol = te.Tolerance();

        NCollection_List<CurveRepresentation> lcr = te.Curves();
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();

            if (cr.IsCurveOnSurface())
            {
                UpdateEdge(eout, cr.PCurve(), cr.Surface(), ein.Placement * cr.Location(), tol);
            }

            else if (cr.IsCurveOnClosedSurface)
            {
                UpdateEdge(eout, cr.PCurve(), cr.PCurve2(), cr.Surface(), ein.Placement * cr.Location(), tol);
            }

            if (cr.IsRegularity())
            {
                Continuity(eout,
                           cr.Surface(),
                           cr.Surface2(),
                           ein.Placement * cr.Location(),
                           ein.Placement * cr.Location2(),
                           cr.Continuity());
            }

            itcr.Next();
        }
    }

    /// <summary>Makes an undefined vertex.</summary>
    internal void MakeVertex(ref Vertex v)
    {
        BrepTVertex tv = new BrepTVertex();
        MakeShape(ref ShapeCasts.AsShapeRef(ref v), tv);
    }

    /// <summary>Makes a vertex at the point with the tolerance.</summary>
    internal void MakeVertex(ref Vertex v, in Point3D p, double tol)
    {
        MakeVertex(ref v);
        UpdateVertex(v, p, tol);
    }

    /// <summary>Sets the vertex's point and raises its tolerance to <paramref name="tol"/> if lower.</summary>
    internal void UpdateVertex(in Vertex v, in Point3D p, double tol)
    {
        BrepTVertex tv = (BrepTVertex)v.TShape()!;
        if (WorkingOverlay.IsLocked(tv))
        {
            throw new LockedShapeException("BRep_Builder::UpdateVertex");
        }

        tv.Pnt(p.Transformed(v.Placement.Inverted().Transformation));
        tv.UpdateTolerance(tol);
        tv.Modified(true);
    }

    /// <summary>Sets the parameter of the vertex on the edge, and raises the vertex's tolerance to <paramref name="tol"/>.</summary>
    /// <exception cref="DomainErrorException">The parameter is infinite.</exception>
    internal void UpdateVertex(in Vertex v, double par, in Edge e, double tol)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(par) || global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(par))
        {
            throw new DomainErrorException("BRep_Builder::Infinite parameter");
        }

        BrepTVertex tv = (BrepTVertex)v.TShape()!;
        BrepTEdge te = (BrepTEdge)e.TShape()!;

        if (WorkingOverlay.IsLocked(tv) || WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateVertex");
        }

        Placement l = e.Placement.Predivided(v.Placement);

        // Search the vertex in the edge
        Orientation ori = Internal;

        var itv = new Iterator(e.Oriented(Forward));

        // if the edge has no vertices
        // and is degenerated use the vertex orientation
        // RLE, june 94

        if (!itv.More() && te.Degenerated())
        {
            ori = v.Orientation;
        }

        while (itv.More())
        {
            Shape vcur = itv.Value();
            if (v.IsSame(vcur))
            {
                ori = vcur.Orientation;
                if (ori == v.Orientation)
                {
                    break;
                }
            }

            itv.Next();
        }

        NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            GCurve? gc = itcr.Value() as GCurve;
            if (gc is not null)
            {
                if (ori == Forward)
                {
                    gc.First(par);
                }

                else if (ori == Reversed)
                {
                    gc.Last(par);
                }

                else
                {
                    NCollection_List<PointRepresentation> lpr = tv.ChangePoints();
                    Placement gCloc = gc.Location();
                    Placement lgCloc = l * gCloc;
                    if (gc.IsCurve3D)
                    {
                        Curve3D? gc3d = gc.Curve3D();
                        UpdatePoints(lpr, par, gc3d, lgCloc);
                    }
                    else if (gc.IsCurveOnSurface())
                    {
                        Curve2D? gCpc = gc.PCurve();
                        Surface? gCsu = gc.Surface();
                        UpdatePoints(lpr, par, gCpc, gCsu, lgCloc);
                    }
                }
            }

            itcr.Next();
        }

        if ((ori != Forward) && (ori != Reversed))
        {
            tv.Modified(true);
        }

        tv.UpdateTolerance(tol);
        te.Modified(true);
    }

    /// <summary>
    /// Sets the parameter of the vertex on the edge's curve on the face, and raises the vertex's tolerance to
    /// <paramref name="tol"/>.
    /// </summary>
    internal void UpdateVertex(in Vertex v, double par, in Edge e, in Face f, double tol)
    {
        Surface? s = Tool.Surface(f, out Placement l);
        UpdateVertex(v, par, e, s, l, tol);
    }

    /// <summary>
    /// Sets the parameter of the vertex on the edge's curve on the surface, and raises the vertex's tolerance to
    /// <paramref name="tol"/>.
    /// </summary>
    /// <exception cref="DomainErrorException">The parameter is infinite, or the edge has no curve on the surface.</exception>
    internal void UpdateVertex(in Vertex v,
                               double par,
                               in Edge e,
                               Surface? s,
                               in Placement L,
                               double tol)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(par) || global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(par))
        {
            throw new DomainErrorException("BRep_Builder::Infinite parameter");
        }

        // Find the curve representation
        Placement l = L.Predivided(v.Placement);

        BrepTVertex tv = (BrepTVertex)v.TShape()!;
        BrepTEdge te = (BrepTEdge)e.TShape()!;

        if (WorkingOverlay.IsLocked(tv) || WorkingOverlay.IsLocked(te))
        {
            throw new LockedShapeException("BRep_Builder::UpdateVertex");
        }

        // Search the vertex in the edge
        Orientation ori = Internal;

        var itv = new Iterator(e.Oriented(Forward));

        // if the edge has no vertices
        // and is degenerated use the vertex orientation
        // RLE, june 94

        if (!itv.More() && te.Degenerated())
        {
            ori = v.Orientation;
        }

        while (itv.More())
        {
            Shape vcur = itv.Value();
            if (v.IsSame(vcur))
            {
                ori = vcur.Orientation;
                if (ori == v.Orientation)
                {
                    break;
                }
            }

            itv.Next();
        }

        NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            GCurve? gc = itcr.Value() as GCurve;
            if (gc is not null)
            {
                //      if (GC->IsCurveOnSurface(S,l)) {
                if (gc.IsCurveOnSurface(s, L))
                { // xpu020198 : BUC60407
                    if (ori == Forward)
                    {
                        gc.First(par);
                    }

                    else if (ori == Reversed)
                    {
                        gc.Last(par);
                    }

                    else
                    {
                        NCollection_List<PointRepresentation> lpr = tv.ChangePoints();
                        Curve2D? gCpc = gc.PCurve();
                        UpdatePoints(lpr, par, gCpc, s, l);
                        tv.Modified(true);
                    }

                    break;
                }
            }

            itcr.Next();
        }

        if (!itcr.More())
        {
            throw new DomainErrorException("BRep_Builder:: no pcurve");
        }

        tv.UpdateTolerance(tol);
        te.Modified(true);
    }

    /// <summary>Sets the parameters of the vertex on the face, and raises its tolerance to <paramref name="tol"/>.</summary>
    internal void UpdateVertex(in Vertex ve, double u, double v, in Face f, double tol)
    {
        BrepTVertex tv = (BrepTVertex)ve.TShape()!;

        if (WorkingOverlay.IsLocked(tv))
        {
            throw new LockedShapeException("BRep_Builder::UpdateVertex");
        }

        Surface? s = Tool.Surface(f, out Placement l);
        l = l.Predivided(ve.Placement);
        NCollection_List<PointRepresentation> lpr = tv.ChangePoints();
        UpdatePoints(lpr, u, v, s, l);

        tv.UpdateTolerance(tol);
        tv.Modified(true);
    }

    /// <summary>Raises the vertex's tolerance to <paramref name="tol"/> if it is lower.</summary>
    internal void UpdateVertex(in Vertex v, double tol)
    {
        BrepTVertex tv = (BrepTVertex)v.TShape()!;

        if (WorkingOverlay.IsLocked(tv))
        {
            throw new LockedShapeException("BRep_Builder::UpdateVertex");
        }

        tv.UpdateTolerance(tol);
        tv.Modified(true);
    }

    /// <summary>Sets on <paramref name="eout"/> the parameter that <paramref name="vin"/> has on <paramref name="ein"/>, for <paramref name="vout"/>.</summary>
    internal void Transfert(in Edge ein, in Edge eout, in Vertex vin, in Vertex vout)
    {
        double tol = Tool.Tolerance(vin);
        double parin = Tool.Parameter(vin, ein);
        UpdateVertex(vout, parin, eout, tol);
    }
}
