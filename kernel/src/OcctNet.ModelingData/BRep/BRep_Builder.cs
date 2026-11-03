// Port of: src/ModelingData/TKBRep/BRep/BRep_Builder.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Builder.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Builds and updates BRep shapes: vertices, edges and faces with their geometry, and their tolerances, ranges,
/// flags and polygons.
/// </summary>
/// <remarks>
/// <para>
/// Internal to OCCT.NET but for the face of a triangulation, which the modelling layer makes when it reads a mesh (Kerf
/// unit 10B; engine, internal, in unit 10L). Every update throws
/// <see cref="TopoDS_LockedShape"/> on a locked shape, and in the port every frozen shape is locked. A frozen child can
/// be added to an unfrozen parent. Every curve, surface, triangulation or polygon the builder stores is frozen.
/// </para>
/// <para>
/// The Make methods take the shape by <c>ref</c>: some read it first (they throw if it holds a locked shape). The
/// Update methods change only the TShape and take it by <c>in</c>.
/// </para>
/// </remarks>
public sealed class BRep_Builder : TopoDS_Builder
{
    public BRep_Builder()
    {
    }

    // Insert a 3d curve <C> with location <L> in a list of curve representations <lcr>
    private static void UpdateCurves(NCollection_List<BRep_CurveRepresentation> lcr, Geom_Curve? C, in Placement L)
    {
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);
        BRep_GCurve? GC = null;
        double f = 0.0, l = 0.0;

        while (itcr.More())
        {
            GC = itcr.Value() as BRep_GCurve;
            if (GC is not null)
            {
                GC.Range(out f, out l);
                if (GC.IsCurve3D())
                {
                    break;
                }
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            itcr.Value().Curve3D(C);
            itcr.Value().Location(L);
        }
        else
        {
            BRep_Curve3D C3d = new BRep_Curve3D(C, L);
            // test if there is already a range
            if (GC is not null)
            {
                C3d.SetRange(f, l);
            }

            lcr.Append(C3d);
        }
    }

    // Insert a pcurve <C> on surface <S> with location <L> in a list of curve representations <lcr>.
    // Remove the pcurve on <S> from <lcr> if <C> is null
    private static void UpdateCurves(NCollection_List<BRep_CurveRepresentation> lcr,
                                     Geom2d_Curve? C,
                                     Geom_Surface? S,
                                     in Placement L)
    {
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);
        double f = -global::Kerf.Engine.Foundation.Precision.Infinite(), l = global::Kerf.Engine.Foundation.Precision.Infinite();
        // search the range of the 3d curve
        // and remove any existing representation

        while (itcr.More())
        {
            BRep_GCurve? GC = itcr.Value() as BRep_GCurve;
            if (GC is not null)
            {
                if (GC.IsCurve3D())
                {
                    //      if (!C.IsNull()) { //xpu031198, degenerate edge
                    // xpu151298 : parameters can be set for null curves
                    //             see lbo & flo, to determine whether range is defined
                    //             compare first and last parameters with default values.
                    GC.Range(out f, out l);
                }

                if (GC.IsCurveOnSurface(S, L))
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

        if (C is not null)
        {
            BRep_CurveOnSurface COS = new BRep_CurveOnSurface(C, S, L);
            COS.Range(out double aFCur, out double aLCur);
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f))
            {
                aFCur = f;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
            {
                aLCur = l;
            }

            COS.SetRange(aFCur, aLCur);
            lcr.Append(COS);
        }
    }

    // Insert a pcurve <C> on surface <S> with location <L> and UV points <Pf>, <Pl> in a list of curve
    // representations <lcr>. Remove the pcurve on <S> from <lcr> if <C> is null
    private static void UpdateCurves(NCollection_List<BRep_CurveRepresentation> lcr,
                                     Geom2d_Curve? C,
                                     Geom_Surface? S,
                                     in Placement L,
                                     in Point2D Pf,
                                     in Point2D Pl)
    {
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);
        double f = -global::Kerf.Engine.Foundation.Precision.Infinite(), l = global::Kerf.Engine.Foundation.Precision.Infinite();
        // search the range of the 3d curve
        // and remove any existing representation

        while (itcr.More())
        {
            BRep_GCurve? GC = itcr.Value() as BRep_GCurve;
            if (GC is not null)
            {
                if (GC.IsCurve3D())
                {
                    //      if (!C.IsNull()) { //xpu031198, degenerate edge
                    // xpu151298 : parameters can be set for null curves
                    //             see lbo & flo, to determine whether range is defined
                    //             compare first and last parameters with default values.
                    GC.Range(out f, out l);
                }

                if (GC.IsCurveOnSurface(S, L))
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

        if (C is not null)
        {
            BRep_CurveOnSurface COS = new BRep_CurveOnSurface(C, S, L);
            COS.Range(out double aFCur, out double aLCur);
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f))
            {
                aFCur = f;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
            {
                aLCur = l;
            }

            COS.SetRange(aFCur, aLCur);
            COS.SetUVPoints(Pf, Pl);
            lcr.Append(COS);
        }
    }

    // Insert two pcurves <C1,C2> on surface <S> with location <L> in a list of curve representations <lcr>.
    // Remove the pcurves on <S> from <lcr> if <C1> or <C2> is null
    private static void UpdateCurves(NCollection_List<BRep_CurveRepresentation> lcr,
                                     Geom2d_Curve? C1,
                                     Geom2d_Curve? C2,
                                     Geom_Surface? S,
                                     in Placement L)
    {
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);
        double f = -global::Kerf.Engine.Foundation.Precision.Infinite(), l = global::Kerf.Engine.Foundation.Precision.Infinite();

        while (itcr.More())
        {
            BRep_GCurve? GC = itcr.Value() as BRep_GCurve;
            if (GC is not null)
            {
                if (GC.IsCurve3D())
                {
                    GC.Range(out f, out l);
                }

                bool iscos = GC.IsCurveOnSurface(S, L);
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

        if (C1 is not null && C2 is not null)
        {
            BRep_CurveOnClosedSurface COS = new BRep_CurveOnClosedSurface(C1, C2, S, L, C0);
            COS.Range(out double aFCur, out double aLCur);
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f))
            {
                aFCur = f;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
            {
                aLCur = l;
            }

            COS.SetRange(aFCur, aLCur);
            lcr.Append(COS);
        }
    }

    // Insert two pcurves <C1,C2> on surface <S> with location <L> and the UV points of the second <Pf>, <Pl> in a
    // list of curve representations <lcr>. Remove the pcurves on <S> from <lcr> if <C1> or <C2> is null
    private static void UpdateCurves(NCollection_List<BRep_CurveRepresentation> lcr,
                                     Geom2d_Curve? C1,
                                     Geom2d_Curve? C2,
                                     Geom_Surface? S,
                                     in Placement L,
                                     in Point2D Pf,
                                     in Point2D Pl)
    {
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);
        double f = -global::Kerf.Engine.Foundation.Precision.Infinite(), l = global::Kerf.Engine.Foundation.Precision.Infinite();

        while (itcr.More())
        {
            BRep_GCurve? GC = itcr.Value() as BRep_GCurve;
            if (GC is not null)
            {
                if (GC.IsCurve3D())
                {
                    GC.Range(out f, out l);
                }

                bool iscos = GC.IsCurveOnSurface(S, L);
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

        if (C1 is not null && C2 is not null)
        {
            BRep_CurveOnClosedSurface COS = new BRep_CurveOnClosedSurface(C1, C2, S, L, C0);
            COS.Range(out double aFCur, out double aLCur);
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f))
            {
                aFCur = f;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
            {
                aLCur = l;
            }

            COS.SetRange(aFCur, aLCur);
            COS.SetUVPoints2(Pf, Pl);
            lcr.Append(COS);
        }
    }

    private static void UpdateCurves(NCollection_List<BRep_CurveRepresentation> lcr,
                                     Geom_Surface? S1,
                                     Geom_Surface? S2,
                                     in Placement L1,
                                     in Placement L2,
                                     Continuity C)
    {
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);
        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            bool isregu = cr.IsRegularity(S1, S2, L1, L2);
            if (isregu)
            {
                break;
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            cr.Continuity(C);
        }
        else
        {
            BRep_CurveOn2Surfaces COS = new BRep_CurveOn2Surfaces(S1, S2, L1, L2, C);
            lcr.Append(COS);
        }
    }

    private static void UpdatePoints(NCollection_List<BRep_PointRepresentation> lpr, double p, Geom_Curve? C, in Placement L)
    {
        var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(lpr);
        while (itpr.More())
        {
            BRep_PointRepresentation pr = itpr.Value();
            bool isponc = pr.IsPointOnCurve(C, L);
            if (isponc)
            {
                break;
            }

            itpr.Next();
        }

        if (itpr.More())
        {
            BRep_PointRepresentation pr = itpr.Value();
            pr.Parameter(p);
        }
        else
        {
            BRep_PointOnCurve POC = new BRep_PointOnCurve(p, C, L);
            lpr.Append(POC);
        }
    }

    private static void UpdatePoints(NCollection_List<BRep_PointRepresentation> lpr,
                                     double p,
                                     Geom2d_Curve? PC,
                                     Geom_Surface? S,
                                     in Placement L)
    {
        var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(lpr);
        while (itpr.More())
        {
            BRep_PointRepresentation pr = itpr.Value();
            bool isponcons = pr.IsPointOnCurveOnSurface(PC, S, L);
            if (isponcons)
            {
                break;
            }

            itpr.Next();
        }

        if (itpr.More())
        {
            BRep_PointRepresentation pr = itpr.Value();
            pr.Parameter(p);
        }
        else
        {
            BRep_PointOnCurveOnSurface POCS = new BRep_PointOnCurveOnSurface(p, PC, S, L);
            lpr.Append(POCS);
        }
    }

    private static void UpdatePoints(NCollection_List<BRep_PointRepresentation> lpr,
                                     double p1,
                                     double p2,
                                     Geom_Surface? S,
                                     in Placement L)
    {
        var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(lpr);
        while (itpr.More())
        {
            BRep_PointRepresentation pr = itpr.Value();
            bool ispons = pr.IsPointOnSurface(S, L);
            if (ispons)
            {
                break;
            }

            itpr.Next();
        }

        if (itpr.More())
        {
            BRep_PointRepresentation pr = itpr.Value();
            pr.Parameter(p1);
            //    pr->Parameter(p2); // skv
            pr.Parameter2(p2); // skv
        }
        else
        {
            BRep_PointOnSurface POS = new BRep_PointOnSurface(p1, p2, S, L);
            lpr.Append(POS);
        }
    }

    /// <summary>Makes an undefined face.</summary>
    internal void MakeFace(ref TopoDS_Face F)
    {
        BRep_TFace TF = new BRep_TFace();
        MakeShape(ref TopoDS.AsShapeRef(ref F), TF);
    }

    /// <summary>Makes a face with a surface (frozen here) and a tolerance.</summary>
    internal void MakeFace(ref TopoDS_Face F, Geom_Surface? S, double Tol)
    {
        BRep_TFace TF = new BRep_TFace();
        if (!F.IsNull() && F.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::MakeFace");
        }

        TF.Surface(S);
        TF.Tolerance(Tol);
        MakeShape(ref TopoDS.AsShapeRef(ref F), TF);
    }

    /// <summary>Makes a face with a triangulation (frozen here) and no surface.</summary>
    public void MakeFace(ref TopoDS_Face theFace, Triangulation? theTriangulation)
    {
        BRep_TFace aTFace = new BRep_TFace();
        if (!theFace.IsNull() && theFace.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::MakeFace");
        }

        aTFace.Triangulation(theTriangulation);
        MakeShape(ref TopoDS.AsShapeRef(ref theFace), aTFace);
    }

    /// <summary>Makes a face with a list of triangulations (frozen here), one of them active (the first if null).</summary>
    internal void MakeFace(ref TopoDS_Face theFace,
                           NCollection_List<Triangulation> theTriangulations,
                           Triangulation? theActiveTriangulation = null)
    {
        BRep_TFace aTFace = new BRep_TFace();
        if (!theFace.IsNull() && theFace.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::MakeFace");
        }

        aTFace.Triangulations(theTriangulations, theActiveTriangulation);
        MakeShape(ref TopoDS.AsShapeRef(ref theFace), aTFace);
    }

    /// <summary>Makes a face with a surface (frozen here), a location for it and a tolerance.</summary>
    internal void MakeFace(ref TopoDS_Face F, Geom_Surface? S, in Placement L, double Tol)
    {
        BRep_TFace TF = new BRep_TFace();
        if (!F.IsNull() && F.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::MakeFace");
        }

        TF.Surface(S);
        TF.Tolerance(Tol);
        TF.Location(L);
        MakeShape(ref TopoDS.AsShapeRef(ref F), TF);
    }

    /// <summary>Updates the face's surface (frozen here), its location and the tolerance.</summary>
    internal void UpdateFace(in TopoDS_Face F, Geom_Surface? S, in Placement L, double Tol)
    {
        BRep_TFace TF = (BRep_TFace)F.TShape()!;
        if (TF.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateFace");
        }

        TF.Surface(S);
        TF.Tolerance(Tol);
        TF.Location(L.Predivided(F.Location()));
        F.TShape()!.Modified(true);
    }

    /// <summary>
    /// Changes the face's triangulation (frozen here). With <paramref name="theToReset"/>, the face's list of
    /// triangulations becomes just this one; otherwise it becomes active, replacing the active one if not in the list.
    /// </summary>
    internal void UpdateFace(in TopoDS_Face theFace, Triangulation? theTriangulation, bool theToReset = true)
        => updateFace(theFace, theTriangulation, theToReset, true);

    /// <summary>
    /// <see cref="UpdateFace(in TopoDS_Face, Triangulation?, bool)"/> without freezing the triangulation: the
    /// mesher's attach (BRepMesh_ShapeTool only, decision 488). The triangulation freezes with the face's graph when
    /// the result is published.
    /// </summary>
    internal void UpdateFaceUnfrozen(in TopoDS_Face theFace, Triangulation? theTriangulation, bool theToReset = true)
        => updateFace(theFace, theTriangulation, theToReset, false);

    private static void updateFace(in TopoDS_Face theFace, Triangulation? theTriangulation, bool theToReset,
                                   bool theToFreeze)
    {
        BRep_TFace aTFace = (BRep_TFace)theFace.TShape()!;
        if (aTFace.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateFace");
        }

        aTFace.Triangulation(theTriangulation, theToReset, theToFreeze);
        theFace.TShape()!.Modified(true);
    }

    /// <summary>Updates the face's tolerance.</summary>
    internal void UpdateFace(in TopoDS_Face F, double Tol)
    {
        BRep_TFace TF = (BRep_TFace)F.TShape()!;
        if (TF.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateFace");
        }

        TF.Tolerance(Tol);
        F.TShape()!.Modified(true);
    }

    /// <summary>Sets the NaturalRestriction flag of the face.</summary>
    internal void NaturalRestriction(in TopoDS_Face F, bool N)
    {
        BRep_TFace TF = (BRep_TFace)F.TShape()!;
        if (TF.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::NaturalRestriction");
        }

        TF.NaturalRestriction(N);
        F.TShape()!.Modified(true);
    }

    /// <summary>Makes an undefined edge.</summary>
    internal void MakeEdge(ref TopoDS_Edge E)
    {
        BRep_TEdge TE = new BRep_TEdge();
        if (!E.IsNull() && E.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::MakeEdge");
        }

        MakeShape(ref TopoDS.AsShapeRef(ref E), TE);
    }

    /// <summary>Makes an edge with a 3D curve (frozen here) and a tolerance.</summary>
    internal void MakeEdge(ref TopoDS_Edge E, Geom_Curve? C, double Tol)
    {
        MakeEdge(ref E);
        UpdateEdge(E, C, new Placement(), Tol);
    }

    /// <summary>Makes an edge with a 3D curve (frozen here), a location for it and a tolerance.</summary>
    internal void MakeEdge(ref TopoDS_Edge E, Geom_Curve? C, in Placement L, double Tol)
    {
        MakeEdge(ref E);
        UpdateEdge(E, C, L, Tol);
    }

    /// <summary>Makes an edge with a 3D polygon (frozen here).</summary>
    internal void MakeEdge(ref TopoDS_Edge E, Polygon3D? P)
    {
        MakeEdge(ref E);
        UpdateEdge(E, P);
    }

    /// <summary>Makes an edge with a polygon on a triangulation (both frozen here).</summary>
    internal void MakeEdge(ref TopoDS_Edge E, PolygonOnTriangulation? P, Triangulation? T)
    {
        MakeEdge(ref E);
        UpdateEdge(E, P, T, new Placement());
    }

    /// <summary>Makes an edge with a polygon on a triangulation (both frozen here) with a location.</summary>
    internal void MakeEdge(ref TopoDS_Edge E, PolygonOnTriangulation? P, Triangulation? T, in Placement L)
    {
        MakeEdge(ref E);
        UpdateEdge(E, P, T, L);
    }

    /// <summary>Sets the edge's 3D curve (frozen here) and updates the tolerance.</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Geom_Curve? C, double Tol)
        => UpdateEdge(E, C, new Placement(), Tol);

    /// <summary>Sets the edge's 3D curve (frozen here) with a location and updates the tolerance.</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Geom_Curve? C, in Placement L, double Tol)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(E.Location());

        UpdateCurves(TE.ChangeCurves(), C, l);

        TE.UpdateTolerance(Tol);
        TE.Modified(true);
    }

    /// <summary>Sets the edge's curve on the face's surface (frozen here; removes it if null) and updates the tolerance.</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Geom2d_Curve? C, in TopoDS_Face F, double Tol)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out Placement l);
        UpdateEdge(E, C, S, l, Tol);
    }

    /// <summary>
    /// Sets the edge's two curves on the face's closed surface (frozen here; removes them if one is null) and updates
    /// the tolerance.
    /// </summary>
    internal void UpdateEdge(in TopoDS_Edge E, Geom2d_Curve? C1, Geom2d_Curve? C2, in TopoDS_Face F, double Tol)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out Placement l);
        UpdateEdge(E, C1, C2, S, l, Tol);
    }

    /// <summary>Sets the edge's curve on the surface (frozen here; removes it if null) and updates the tolerance.</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Geom2d_Curve? C, Geom_Surface? S, in Placement L, double Tol)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(E.Location());

        UpdateCurves(TE.ChangeCurves(), C, S, l);

        TE.UpdateTolerance(Tol);
        TE.Modified(true);
    }

    /// <summary>
    /// Sets the edge's curve on the surface (frozen here) with its UV end points, and updates the tolerance (for the
    /// second format, XML persistence).
    /// </summary>
    internal void UpdateEdge(in TopoDS_Edge E,
                             Geom2d_Curve? C,
                             Geom_Surface? S,
                             in Placement L,
                             double Tol,
                             in Point2D Pf,
                             in Point2D Pl)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(E.Location());

        UpdateCurves(TE.ChangeCurves(), C, S, l, Pf, Pl);

        TE.UpdateTolerance(Tol);
        TE.Modified(true);
    }

    /// <summary>
    /// Sets the edge's two curves on the closed surface (frozen here; removes them if one is null) and updates the
    /// tolerance.
    /// </summary>
    internal void UpdateEdge(in TopoDS_Edge E,
                             Geom2d_Curve? C1,
                             Geom2d_Curve? C2,
                             Geom_Surface? S,
                             in Placement L,
                             double Tol)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(E.Location());

        UpdateCurves(TE.ChangeCurves(), C1, C2, S, l);

        TE.UpdateTolerance(Tol);
        TE.Modified(true);
    }

    /// <summary>
    /// Sets the edge's two curves on the closed surface (frozen here) with the UV end points of the second, and updates
    /// the tolerance (for the second format, XML persistence).
    /// </summary>
    internal void UpdateEdge(in TopoDS_Edge E,
                             Geom2d_Curve? C1,
                             Geom2d_Curve? C2,
                             Geom_Surface? S,
                             in Placement L,
                             double Tol,
                             in Point2D Pf,
                             in Point2D Pl)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(E.Location());

        UpdateCurves(TE.ChangeCurves(), C1, C2, S, l, Pf, Pl);

        TE.UpdateTolerance(Tol);
        TE.Modified(true);
    }

    /// <summary>Sets the edge's 3D polygon (frozen here; removes it if null).</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Polygon3D? P) => UpdateEdge(E, P, new Placement());

    /// <summary>Sets the edge's 3D polygon (frozen here; removes it if null) with a location.</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Polygon3D? P, in Placement L) => updateEdge(E, P, L, true);

    /// <summary>
    /// <see cref="UpdateEdge(in TopoDS_Edge, Polygon3D?, in Placement)"/> without freezing the polygon: the
    /// mesher's attach (BRepMesh_ShapeTool only, decision 488).
    /// </summary>
    internal void UpdateEdgeUnfrozen(in TopoDS_Edge E, Polygon3D? P, in Placement L) => updateEdge(E, P, L, false);

    private static void updateEdge(in TopoDS_Edge E, Polygon3D? P, in Placement L, bool theToFreeze)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            if (itcr.Value().IsPolygon3D())
            {
                if (P is null)
                {
                    lcr.Remove(itcr);
                }
                else
                {
                    ((BRep_Polygon3D)itcr.Value()).Polygon3D(P, theToFreeze);
                }

                TE.Modified(true);
                return;
            }

            itcr.Next();
        }

        Placement l = L.Predivided(E.Location());
        BRep_Polygon3D P3d = new BRep_Polygon3D(P, l, theToFreeze);
        lcr.Append(P3d);

        TE.Modified(true);
    }

    /// <summary>Sets the edge's polygon on the triangulation (both frozen here; removes it if null).</summary>
    internal void UpdateEdge(in TopoDS_Edge E, PolygonOnTriangulation? P, Triangulation? T)
        => UpdateEdge(E, P, T, new Placement());

    /// <summary>Sets the edge's polygon on the triangulation (both frozen here; removes it if null) with a location.</summary>
    internal void UpdateEdge(in TopoDS_Edge E, PolygonOnTriangulation? P, Triangulation? T, in Placement L)
        => updateEdge(E, P, T, L, true);

    /// <summary>
    /// <see cref="UpdateEdge(in TopoDS_Edge, PolygonOnTriangulation?, Triangulation?, in Placement)"/>
    /// without freezing the polygon and the triangulation: the mesher's attach (BRepMesh_ShapeTool only, decision 488).
    /// </summary>
    internal void UpdateEdgeUnfrozen(in TopoDS_Edge E, PolygonOnTriangulation? P, Triangulation? T,
                                     in Placement L)
        => updateEdge(E, P, T, L, false);

    private static void updateEdge(in TopoDS_Edge E, PolygonOnTriangulation? P, Triangulation? T,
                                   in Placement L, bool theToFreeze)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(E.Location());

        bool isModified = false;

        NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            if (itcr.Value().IsPolygonOnTriangulation(T, l))
            {
                lcr.Remove(itcr);
                isModified = true;
                break;
            }

            itcr.Next();
        }

        if (P is not null)
        {
            BRep_PolygonOnTriangulation PT = new BRep_PolygonOnTriangulation(P, T, l, theToFreeze);
            lcr.Append(PT);
            isModified = true;
        }

        if (isModified)
        {
            TE.Modified(true);
        }
    }

    /// <summary>Sets the edge's two polygons on the triangulation (all frozen here; removes them if one is null).</summary>
    internal void UpdateEdge(in TopoDS_Edge E,
                             PolygonOnTriangulation? P1,
                             PolygonOnTriangulation? P2,
                             Triangulation? T)
        => UpdateEdge(E, P1, P2, T, new Placement());

    /// <summary>
    /// Sets the edge's two polygons on the triangulation (all frozen here; removes them if one is null) with a location.
    /// </summary>
    internal void UpdateEdge(in TopoDS_Edge E,
                             PolygonOnTriangulation? P1,
                             PolygonOnTriangulation? P2,
                             Triangulation? T,
                             in Placement L)
        => updateEdge(E, P1, P2, T, L, true);

    /// <summary>
    /// The seam form of <see cref="UpdateEdgeUnfrozen(in TopoDS_Edge, PolygonOnTriangulation?, Triangulation?, in Placement)"/>
    /// (decision 488).
    /// </summary>
    internal void UpdateEdgeUnfrozen(in TopoDS_Edge E,
                                     PolygonOnTriangulation? P1,
                                     PolygonOnTriangulation? P2,
                                     Triangulation? T,
                                     in Placement L)
        => updateEdge(E, P1, P2, T, L, false);

    private static void updateEdge(in TopoDS_Edge E,
                                   PolygonOnTriangulation? P1,
                                   PolygonOnTriangulation? P2,
                                   Triangulation? T,
                                   in Placement L,
                                   bool theToFreeze)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(E.Location());

        bool isModified = false;

        NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            if (itcr.Value().IsPolygonOnTriangulation(T, l)) // szv:was L
            {
                lcr.Remove(itcr);
                isModified = true;
                break;
            }

            itcr.Next();
        }

        if (P1 is not null && P2 is not null)
        {
            BRep_PolygonOnClosedTriangulation PT = new BRep_PolygonOnClosedTriangulation(P1, P2, T, l, theToFreeze);
            lcr.Append(PT);
            isModified = true;
        }

        if (isModified)
        {
            TE.Modified(true);
        }
    }

    /// <summary>Sets the edge's polygon on the face's surface (frozen here; removes it if null).</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Polygon2D? P, in TopoDS_Face F)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out Placement l);
        UpdateEdge(E, P, S, l);
    }

    /// <summary>Sets the edge's polygon on the surface (frozen here; removes it if null).</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Polygon2D? P, Geom_Surface? S, in Placement L)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(E.Location());

        NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();

        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);
        while (itcr.More())
        {
            if (itcr.Value().IsPolygonOnSurface(S, l))
            {
                break;
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            lcr.Remove(itcr);
        }

        if (P is not null)
        {
            BRep_PolygonOnSurface PS = new BRep_PolygonOnSurface(P, S, l);
            lcr.Append(PS);
        }

        TE.Modified(true);
    }

    /// <summary>Sets the edge's two polygons on the face's closed surface (frozen here; removes them if one is null).</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Polygon2D? P1, Polygon2D? P2, in TopoDS_Face F)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out Placement l);
        UpdateEdge(E, P1, P2, S, l);
    }

    /// <summary>Sets the edge's two polygons on the closed surface (frozen here; removes them if one is null).</summary>
    internal void UpdateEdge(in TopoDS_Edge E, Polygon2D? P1, Polygon2D? P2, Geom_Surface? S, in Placement L)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        Placement l = L.Predivided(E.Location());

        NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();

        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);
        while (itcr.More())
        {
            if (itcr.Value().IsPolygonOnSurface(S, l))
            {
                break;
            }

            itcr.Next();
        }

        if (itcr.More())
        {
            lcr.Remove(itcr);
        }

        if (P1 is not null && P2 is not null)
        {
            // OCCT (occt-issues.md): the polygons are stored with the identity location, not l
            BRep_PolygonOnClosedSurface PS = new BRep_PolygonOnClosedSurface(P1, P2, S, new Placement());
            lcr.Append(PS);
        }

        TE.Modified(true);
    }

    /// <summary>Raises the edge's tolerance to <paramref name="Tol"/> if it is lower.</summary>
    internal void UpdateEdge(in TopoDS_Edge E, double Tol)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateEdge");
        }

        TE.UpdateTolerance(Tol);
        TE.Modified(true);
    }

    /// <summary>Sets the continuity of the edge between the surfaces of the two faces.</summary>
    internal void Continuity(in TopoDS_Edge E, in TopoDS_Face F1, in TopoDS_Face F2, Continuity C)
    {
        Geom_Surface? S1 = BRep_Tool.Surface(F1, out Placement l1);
        Geom_Surface? S2 = BRep_Tool.Surface(F2, out Placement l2);
        Continuity(E, S1, S2, l1, l2, C);
    }

    /// <summary>Sets the continuity of the edge between the two surfaces (frozen here).</summary>
    internal void Continuity(in TopoDS_Edge E,
                             Geom_Surface? S1,
                             Geom_Surface? S2,
                             in Placement L1,
                             in Placement L2,
                             Continuity C)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::Continuity");
        }

        Placement l1 = L1.Predivided(E.Location());
        Placement l2 = L2.Predivided(E.Location());

        UpdateCurves(TE.ChangeCurves(), S1, S2, l1, l2, C);

        TE.Modified(true);
    }

    /// <summary>Sets the SameParameter flag of the edge.</summary>
    internal void SameParameter(in TopoDS_Edge E, bool S)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::SameParameter");
        }

        TE.SameParameter(S);
        TE.Modified(true);
    }

    /// <summary>Sets the SameRange flag of the edge.</summary>
    internal void SameRange(in TopoDS_Edge E, bool S)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::SameRange");
        }

        TE.SameRange(S);
        TE.Modified(true);
    }

    /// <summary>Sets the Degenerated flag of the edge; a degenerated edge gets a null 3D curve.</summary>
    internal void Degenerated(in TopoDS_Edge E, bool D)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::Degenerated");
        }

        TE.Degenerated(D);
        if (D)
        {
            // set a null 3d curve
            UpdateCurves(TE.ChangeCurves(), (Geom_Curve?)null, E.Location());
        }

        TE.Modified(true);
    }

    /// <summary>Sets the range of the edge's curves: all of them, or only the 3D curve if <paramref name="Only3d"/>.</summary>
    internal void Range(in TopoDS_Edge E, double First, double Last, bool Only3d = false)
    {
        //  set the range to all the representations if Only3d=FALSE
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::Range");
        }

        NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            BRep_GCurve? GC = itcr.Value() as BRep_GCurve;
            if (GC is not null && (!Only3d || GC.IsCurve3D()))
            {
                GC.SetRange(First, Last);
            }

            itcr.Next();
        }

        TE.Modified(true);
    }

    /// <summary>Sets the range of the edge's curve on the surface.</summary>
    /// <exception cref="DomainErrorException">The edge has no curve on the surface.</exception>
    internal void Range(in TopoDS_Edge E, Geom_Surface? S, in Placement L, double First, double Last)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::Range");
        }

        Placement l = L.Predivided(E.Location());

        NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            BRep_GCurve? GC = itcr.Value() as BRep_GCurve;
            if (GC is not null && GC.IsCurveOnSurface(S, l))
            {
                GC.SetRange(First, Last);
                break;
            }

            itcr.Next();
        }

        if (!itcr.More())
        {
            throw new DomainErrorException("BRep_Builder::Range, no pcurve");
        }

        TE.Modified(true);
    }

    /// <summary>Sets the range of the edge's curve on the face.</summary>
    /// <exception cref="DomainErrorException">The edge has no curve on the face.</exception>
    internal void Range(in TopoDS_Edge E, in TopoDS_Face F, double First, double Last)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out Placement l);
        Range(E, S, l, First, Last);
    }

    /// <summary>Adds to <paramref name="Eout"/> the curves on surfaces and the continuities of <paramref name="Ein"/>.</summary>
    internal void Transfert(in TopoDS_Edge Ein, in TopoDS_Edge Eout)
    {
        BRep_TEdge TE = (BRep_TEdge)Ein.TShape()!;
        double tol = TE.Tolerance();

        NCollection_List<BRep_CurveRepresentation> lcr = TE.Curves();
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            BRep_CurveRepresentation CR = itcr.Value();

            if (CR.IsCurveOnSurface())
            {
                UpdateEdge(Eout, CR.PCurve(), CR.Surface(), Ein.Location() * CR.Location(), tol);
            }

            else if (CR.IsCurveOnClosedSurface())
            {
                UpdateEdge(Eout, CR.PCurve(), CR.PCurve2(), CR.Surface(), Ein.Location() * CR.Location(), tol);
            }

            if (CR.IsRegularity())
            {
                Continuity(Eout,
                           CR.Surface(),
                           CR.Surface2(),
                           Ein.Location() * CR.Location(),
                           Ein.Location() * CR.Location2(),
                           CR.Continuity());
            }

            itcr.Next();
        }
    }

    /// <summary>Makes an undefined vertex.</summary>
    internal void MakeVertex(ref TopoDS_Vertex V)
    {
        BRep_TVertex TV = new BRep_TVertex();
        MakeShape(ref TopoDS.AsShapeRef(ref V), TV);
    }

    /// <summary>Makes a vertex at the point with the tolerance.</summary>
    internal void MakeVertex(ref TopoDS_Vertex V, in Point3D P, double Tol)
    {
        MakeVertex(ref V);
        UpdateVertex(V, P, Tol);
    }

    /// <summary>Sets the vertex's point and raises its tolerance to <paramref name="Tol"/> if lower.</summary>
    internal void UpdateVertex(in TopoDS_Vertex V, in Point3D P, double Tol)
    {
        BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
        if (TV.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateVertex");
        }

        TV.Pnt(P.Transformed(V.Location().Inverted().Transformation()));
        TV.UpdateTolerance(Tol);
        TV.Modified(true);
    }

    /// <summary>Sets the parameter of the vertex on the edge, and raises the vertex's tolerance to <paramref name="Tol"/>.</summary>
    /// <exception cref="DomainErrorException">The parameter is infinite.</exception>
    internal void UpdateVertex(in TopoDS_Vertex V, double Par, in TopoDS_Edge E, double Tol)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(Par) || global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(Par))
        {
            throw new DomainErrorException("BRep_Builder::Infinite parameter");
        }

        BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;

        if (TV.Locked() || TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateVertex");
        }

        Placement L = E.Location().Predivided(V.Location());

        // Search the vertex in the edge
        TopAbs_Orientation ori = TopAbs_INTERNAL;

        var itv = new TopoDS_Iterator(E.Oriented(TopAbs_FORWARD));

        // if the edge has no vertices
        // and is degenerated use the vertex orientation
        // RLE, june 94

        if (!itv.More() && TE.Degenerated())
        {
            ori = V.Orientation();
        }

        while (itv.More())
        {
            TopoDS_Shape Vcur = itv.Value();
            if (V.IsSame(Vcur))
            {
                ori = Vcur.Orientation();
                if (ori == V.Orientation())
                {
                    break;
                }
            }

            itv.Next();
        }

        NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            BRep_GCurve? GC = itcr.Value() as BRep_GCurve;
            if (GC is not null)
            {
                if (ori == TopAbs_FORWARD)
                {
                    GC.First(Par);
                }

                else if (ori == TopAbs_REVERSED)
                {
                    GC.Last(Par);
                }

                else
                {
                    NCollection_List<BRep_PointRepresentation> lpr = TV.ChangePoints();
                    Placement GCloc = GC.Location();
                    Placement LGCloc = L * GCloc;
                    if (GC.IsCurve3D())
                    {
                        Geom_Curve? GC3d = GC.Curve3D();
                        UpdatePoints(lpr, Par, GC3d, LGCloc);
                    }
                    else if (GC.IsCurveOnSurface())
                    {
                        Geom2d_Curve? GCpc = GC.PCurve();
                        Geom_Surface? GCsu = GC.Surface();
                        UpdatePoints(lpr, Par, GCpc, GCsu, LGCloc);
                    }
                }
            }

            itcr.Next();
        }

        if ((ori != TopAbs_FORWARD) && (ori != TopAbs_REVERSED))
        {
            TV.Modified(true);
        }

        TV.UpdateTolerance(Tol);
        TE.Modified(true);
    }

    /// <summary>
    /// OCCT.NET: frozen. <see cref="UpdateVertex(in TopoDS_Vertex, double, in TopoDS_Edge, double)"/> for a vertex that
    /// may be frozen: when the vertex is FORWARD or REVERSED in the edge and already at least as tolerant as
    /// <paramref name="Tol"/>, the call would leave the vertex as it is (it sets only the edge's curve ranges), so on a
    /// frozen vertex only the ranges are set. Any other write into a frozen vertex throws, as UpdateVertex does.
    /// </summary>
    /// <remarks>
    /// For algorithms that, as OCCT does, build on demand after their result is published, sharing its vertices:
    /// a sweep's history builds the edge a vertex generates, and sets its vertices' parameters on it.
    /// </remarks>
    internal void UpdateVertexOfFrozen(in TopoDS_Vertex V, double Par, in TopoDS_Edge E, double Tol)
    {
        BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        if (!TV.IsFrozen || TE.Locked() || Tol > TV.Tolerance()
            || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(Par) || global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(Par))
        {
            UpdateVertex(V, Par, E, Tol);
            return;
        }

        // the vertex's orientation in the edge, as UpdateVertex finds it
        TopAbs_Orientation ori = TopAbs_INTERNAL;
        var itv = new TopoDS_Iterator(E.Oriented(TopAbs_FORWARD));
        if (!itv.More() && TE.Degenerated())
        {
            ori = V.Orientation();
        }
        while (itv.More())
        {
            TopoDS_Shape Vcur = itv.Value();
            if (V.IsSame(Vcur))
            {
                ori = Vcur.Orientation();
                if (ori == V.Orientation())
                {
                    break;
                }
            }
            itv.Next();
        }
        if (ori != TopAbs_FORWARD && ori != TopAbs_REVERSED)
        {
            UpdateVertex(V, Par, E, Tol); // would add a point representation: throws TopoDS_LockedShape
            return;
        }

        for (var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.ChangeCurves()); itcr.More(); itcr.Next())
        {
            if (itcr.Value() is BRep_GCurve GC)
            {
                if (ori == TopAbs_FORWARD)
                {
                    GC.First(Par);
                }
                else
                {
                    GC.Last(Par);
                }
            }
        }
        TE.Modified(true);
    }

    /// <summary>
    /// Sets the parameter of the vertex on the edge's curve on the face, and raises the vertex's tolerance to
    /// <paramref name="Tol"/>.
    /// </summary>
    internal void UpdateVertex(in TopoDS_Vertex V, double Par, in TopoDS_Edge E, in TopoDS_Face F, double Tol)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out Placement l);
        UpdateVertex(V, Par, E, S, l, Tol);
    }

    /// <summary>
    /// Sets the parameter of the vertex on the edge's curve on the surface, and raises the vertex's tolerance to
    /// <paramref name="Tol"/>.
    /// </summary>
    /// <exception cref="DomainErrorException">The parameter is infinite, or the edge has no curve on the surface.</exception>
    internal void UpdateVertex(in TopoDS_Vertex V,
                               double Par,
                               in TopoDS_Edge E,
                               Geom_Surface? S,
                               in Placement L,
                               double Tol)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(Par) || global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(Par))
        {
            throw new DomainErrorException("BRep_Builder::Infinite parameter");
        }

        // Find the curve representation
        Placement l = L.Predivided(V.Location());

        BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;

        if (TV.Locked() || TE.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateVertex");
        }

        // Search the vertex in the edge
        TopAbs_Orientation ori = TopAbs_INTERNAL;

        var itv = new TopoDS_Iterator(E.Oriented(TopAbs_FORWARD));

        // if the edge has no vertices
        // and is degenerated use the vertex orientation
        // RLE, june 94

        if (!itv.More() && TE.Degenerated())
        {
            ori = V.Orientation();
        }

        while (itv.More())
        {
            TopoDS_Shape Vcur = itv.Value();
            if (V.IsSame(Vcur))
            {
                ori = Vcur.Orientation();
                if (ori == V.Orientation())
                {
                    break;
                }
            }

            itv.Next();
        }

        NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);

        while (itcr.More())
        {
            BRep_GCurve? GC = itcr.Value() as BRep_GCurve;
            if (GC is not null)
            {
                //      if (GC->IsCurveOnSurface(S,l)) {
                if (GC.IsCurveOnSurface(S, L))
                { // xpu020198 : BUC60407
                    if (ori == TopAbs_FORWARD)
                    {
                        GC.First(Par);
                    }

                    else if (ori == TopAbs_REVERSED)
                    {
                        GC.Last(Par);
                    }

                    else
                    {
                        NCollection_List<BRep_PointRepresentation> lpr = TV.ChangePoints();
                        Geom2d_Curve? GCpc = GC.PCurve();
                        UpdatePoints(lpr, Par, GCpc, S, l);
                        TV.Modified(true);
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

        TV.UpdateTolerance(Tol);
        TE.Modified(true);
    }

    /// <summary>Sets the parameters of the vertex on the face, and raises its tolerance to <paramref name="Tol"/>.</summary>
    internal void UpdateVertex(in TopoDS_Vertex Ve, double U, double V, in TopoDS_Face F, double Tol)
    {
        BRep_TVertex TV = (BRep_TVertex)Ve.TShape()!;

        if (TV.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateVertex");
        }

        Geom_Surface? S = BRep_Tool.Surface(F, out Placement L);
        L = L.Predivided(Ve.Location());
        NCollection_List<BRep_PointRepresentation> lpr = TV.ChangePoints();
        UpdatePoints(lpr, U, V, S, L);

        TV.UpdateTolerance(Tol);
        TV.Modified(true);
    }

    /// <summary>Raises the vertex's tolerance to <paramref name="Tol"/> if it is lower.</summary>
    internal void UpdateVertex(in TopoDS_Vertex V, double Tol)
    {
        BRep_TVertex TV = (BRep_TVertex)V.TShape()!;

        if (TV.Locked())
        {
            throw new TopoDS_LockedShape("BRep_Builder::UpdateVertex");
        }

        TV.UpdateTolerance(Tol);
        TV.Modified(true);
    }

    /// <summary>Sets on <paramref name="Eout"/> the parameter that <paramref name="Vin"/> has on <paramref name="Ein"/>, for <paramref name="Vout"/>.</summary>
    internal void Transfert(in TopoDS_Edge Ein, in TopoDS_Edge Eout, in TopoDS_Vertex Vin, in TopoDS_Vertex Vout)
    {
        double tol = BRep_Tool.Tolerance(Vin);
        double parin = BRep_Tool.Parameter(Vin, Ein);
        UpdateVertex(Vout, parin, Eout, tol);
    }
}
