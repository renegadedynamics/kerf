// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Tools on BRep shapes: the UV bounds of faces, wires and edges, the outer wire of a face, comparisons, the update
/// of cached data, and the cleaning of meshes and geometry.
/// </summary>
/// <remarks>
/// <para>
/// A partial class: Read, Write and Dump come with the shape sets (unit 3C).
/// </para>
/// <para>
/// The statics that change the shape they are given (Update, UpdateFaceUVPoints, Clean, CleanGeometry,
/// RemoveUnusedPCurves, the triangulation loaders, RemoveInternals, EvalAndUpdateTol) are internal: they work on
/// unfrozen shapes inside OCCT.NET, and throw on frozen ones as BRep_Builder does. Their public functional twins,
/// which return a changed copy, come with BRepTools_Modifier (unit 3B; docs/phase3-design.md, section 4.6).
/// </para>
/// </remarks>
public static partial class BRepTools
{
    /// <summary>The UV bounds of the face (zeros if it has none).</summary>
    public static void UVBounds(in TopoDS_Face F, out double UMin, out double UMax, out double VMin, out double VMax)
    {
        Bnd_Box2d B = new Bnd_Box2d();
        AddUVBounds(F, ref B);
        if (!B.IsVoid())
        {
            B.Get(out UMin, out VMin, out UMax, out VMax);
        }
        else
        {
            UMin = UMax = VMin = VMax = 0.0;
        }
    }

    /// <summary>The UV bounds of the wire on the face (zeros if it has none).</summary>
    public static void UVBounds(in TopoDS_Face F,
                                in TopoDS_Wire W,
                                out double UMin,
                                out double UMax,
                                out double VMin,
                                out double VMax)
    {
        Bnd_Box2d B = new Bnd_Box2d();
        AddUVBounds(F, W, ref B);
        if (!B.IsVoid())
        {
            B.Get(out UMin, out VMin, out UMax, out VMax);
        }
        else
        {
            UMin = UMax = VMin = VMax = 0.0;
        }
    }

    /// <summary>The UV bounds of the edge on the face (zeros if it has none).</summary>
    public static void UVBounds(in TopoDS_Face F,
                                in TopoDS_Edge E,
                                out double UMin,
                                out double UMax,
                                out double VMin,
                                out double VMax)
    {
        Bnd_Box2d B = new Bnd_Box2d();
        AddUVBounds(F, E, ref B);
        if (!B.IsVoid())
        {
            B.Get(out UMin, out VMin, out UMax, out VMax);
        }
        else
        {
            UMin = UMax = VMin = VMax = 0.0;
        }
    }

    /// <summary>
    /// Adds the UV bounds of the face to <paramref name="B"/>: those of its edges' pcurves, or the surface's natural
    /// bounds if there are none.
    /// </summary>
    public static void AddUVBounds(in TopoDS_Face FF, ref Bnd_Box2d B)
    {
        TopoDS_Face F = FF;
        F.Orientation(TopAbs_FORWARD);
        var ex = new TopExp_Explorer(F, TopAbs_EDGE);

        // fill box for the given face
        Bnd_Box2d aBox = new Bnd_Box2d();
        for (; ex.More(); ex.Next())
        {
            BRepTools.AddUVBounds(F, TopoDS.Edge(ex.Current()), ref aBox);
        }

        // if the box is empty (face without edges or without pcurves),
        // get natural bounds
        if (aBox.IsVoid())
        {
            Geom_Surface? aSurf = BRep_Tool.Surface(F, out TopLoc_Location L);
            if (aSurf is null)
            {
                return;
            }

            aSurf.Bounds(out double UMin, out double UMax, out double VMin, out double VMax);
            aBox.Update(UMin, VMin, UMax, VMax);
        }

        // add face box to result
        B.Add(aBox);
    }

    /// <summary>Adds the UV bounds of the wire on the face to <paramref name="B"/>.</summary>
    public static void AddUVBounds(in TopoDS_Face F, in TopoDS_Wire W, ref Bnd_Box2d B)
    {
        var ex = new TopExp_Explorer();
        for (ex.Init(W, TopAbs_EDGE); ex.More(); ex.Next())
        {
            BRepTools.AddUVBounds(F, TopoDS.Edge(ex.Current()), ref B);
        }
    }

    /// <summary>
    /// Adds the UV bounds of the edge's pcurve on the face to <paramref name="aB"/>, clipped to the surface's bounds in
    /// a non-periodic direction.
    /// </summary>
    public static void AddUVBounds(in TopoDS_Face aF, in TopoDS_Edge aE, ref Bnd_Box2d aB)
    {
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        double aUmin, aUmax, aVmin, aVmax;
        Bnd_Box2d aBoxC = new Bnd_Box2d(), aBoxS = new Bnd_Box2d();
        //
        Geom2d_Curve? aC2D = BRep_Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2);
        if (aC2D is null)
        {
            return;
        }

        //
        BndLib_Add2dCurve.Add(aC2D, aT1, aT2, 0.0, ref aBoxC);
        if (!aBoxC.IsVoid())
        {
            aBoxC.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        }

        //
        Geom_Surface aS = BRep_Tool.Surface(aF, out TopLoc_Location aLoc)!;
        aS.Bounds(out aUmin, out aUmax, out aVmin, out aVmax);

        if (aS.DynamicType() == typeof(Geom_RectangularTrimmedSurface))
        {
            Geom_RectangularTrimmedSurface aSt = (Geom_RectangularTrimmedSurface)aS;
            aS = aSt.BasisSurface();
        }

        //
        if (!aS.IsUPeriodic())
        {
            bool isUPeriodic = false;

            // Additional verification for U-periodicity for B-spline surfaces
            // 1. Verify that the surface is U-closed (if such flag is false). Verification uses 2 points
            // 2. Verify periodicity of surface inside UV-bounds of the edge. Verification uses 3 or 6
            // points.
            if (aS.DynamicType() == typeof(Geom_BSplineSurface) && (aXmin < aUmin || aXmax > aUmax))
            {
                double aTol2 = 100 * Precision.Confusion() * Precision.Confusion();
                isUPeriodic = true;
                gp_Pnt P1, P2;
                // 1. Verify that the surface is U-closed
                if (!aS.IsUClosed())
                {
                    double aVStep = aVmax - aVmin;
                    for (double aV = aVmin; aV <= aVmax; aV += aVStep)
                    {
                        P1 = aS.Value(aUmin, aV);
                        P2 = aS.Value(aUmax, aV);
                        if (P1.SquareDistance(P2) > aTol2)
                        {
                            isUPeriodic = false;
                            break;
                        }
                    }
                }

                // 2. Verify periodicity of surface inside UV-bounds of the edge
                if (isUPeriodic) // the flag still not changed
                {
                    double aV = (aVmin + aVmax) * 0.5;
                    Span<double> aU = stackalloc double[6];   // values of U lying out of surface boundaries
                    Span<double> aUpp = stackalloc double[6]; // corresponding U-values plus/minus period
                    int aNbPnt = 0;
                    if (aXmin < aUmin)
                    {
                        aU[0] = aXmin;
                        aU[1] = (aXmin + aUmin) * 0.5;
                        aU[2] = aUmin;
                        aUpp[0] = aU[0] + aUmax - aUmin;
                        aUpp[1] = aU[1] + aUmax - aUmin;
                        aUpp[2] = aU[2] + aUmax - aUmin;
                        aNbPnt += 3;
                    }

                    if (aXmax > aUmax)
                    {
                        aU[aNbPnt] = aUmax;
                        aU[aNbPnt + 1] = (aXmax + aUmax) * 0.5;
                        aU[aNbPnt + 2] = aXmax;
                        aUpp[aNbPnt] = aU[aNbPnt] - aUmax + aUmin;
                        aUpp[aNbPnt + 1] = aU[aNbPnt + 1] - aUmax + aUmin;
                        aUpp[aNbPnt + 2] = aU[aNbPnt + 2] - aUmax + aUmin;
                        aNbPnt += 3;
                    }

                    for (int anInd = 0; anInd < aNbPnt; anInd++)
                    {
                        P1 = aS.Value(aU[anInd], aV);
                        P2 = aS.Value(aUpp[anInd], aV);
                        if (P1.SquareDistance(P2) > aTol2)
                        {
                            isUPeriodic = false;
                            break;
                        }
                    }
                }
            }

            if (!isUPeriodic)
            {
                if ((aXmin < aUmin) && (aUmin < aXmax))
                {
                    aXmin = aUmin;
                }

                if ((aXmin < aUmax) && (aUmax < aXmax))
                {
                    aXmax = aUmax;
                }
            }
        }

        if (!aS.IsVPeriodic())
        {
            bool isVPeriodic = false;

            // Additional verification for V-periodicity for B-spline surfaces
            // 1. Verify that the surface is V-closed (if such flag is false). Verification uses 2 points
            // 2. Verify periodicity of surface inside UV-bounds of the edge. Verification uses 3 or 6
            // points.
            if (aS.DynamicType() == typeof(Geom_BSplineSurface) && (aYmin < aVmin || aYmax > aVmax))
            {
                double aTol2 = 100 * Precision.Confusion() * Precision.Confusion();
                isVPeriodic = true;
                gp_Pnt P1, P2;
                // 1. Verify that the surface is V-closed
                if (!aS.IsVClosed())
                {
                    double aUStep = aUmax - aUmin;
                    for (double aU = aUmin; aU <= aUmax; aU += aUStep)
                    {
                        P1 = aS.Value(aU, aVmin);
                        P2 = aS.Value(aU, aVmax);
                        if (P1.SquareDistance(P2) > aTol2)
                        {
                            isVPeriodic = false;
                            break;
                        }
                    }
                }

                // 2. Verify periodicity of surface inside UV-bounds of the edge
                if (isVPeriodic) // the flag still not changed
                {
                    double aU = (aUmin + aUmax) * 0.5;
                    Span<double> aV = stackalloc double[6];   // values of V lying out of surface boundaries
                    Span<double> aVpp = stackalloc double[6]; // corresponding V-values plus/minus period
                    int aNbPnt = 0;
                    if (aYmin < aVmin)
                    {
                        aV[0] = aYmin;
                        aV[1] = (aYmin + aVmin) * 0.5;
                        aV[2] = aVmin;
                        aVpp[0] = aV[0] + aVmax - aVmin;
                        aVpp[1] = aV[1] + aVmax - aVmin;
                        aVpp[2] = aV[2] + aVmax - aVmin;
                        aNbPnt += 3;
                    }

                    if (aYmax > aVmax)
                    {
                        aV[aNbPnt] = aVmax;
                        aV[aNbPnt + 1] = (aYmax + aVmax) * 0.5;
                        aV[aNbPnt + 2] = aYmax;
                        aVpp[aNbPnt] = aV[aNbPnt] - aVmax + aVmin;
                        aVpp[aNbPnt + 1] = aV[aNbPnt + 1] - aVmax + aVmin;
                        aVpp[aNbPnt + 2] = aV[aNbPnt + 2] - aVmax + aVmin;
                        aNbPnt += 3;
                    }

                    for (int anInd = 0; anInd < aNbPnt; anInd++)
                    {
                        P1 = aS.Value(aU, aV[anInd]);
                        P2 = aS.Value(aU, aVpp[anInd]);
                        if (P1.SquareDistance(P2) > aTol2)
                        {
                            isVPeriodic = false;
                            break;
                        }
                    }
                }
            }

            if (!isVPeriodic)
            {
                if ((aYmin < aVmin) && (aVmin < aYmax))
                {
                    aYmin = aVmin;
                }

                if ((aYmin < aVmax) && (aVmax < aYmax))
                {
                    aYmax = aVmax;
                }
            }
        }

        aBoxS.Update(aXmin, aYmin, aXmax, aYmax);

        aB.Add(aBoxS);
    }

    /// <summary>Updates the cached data of a vertex: nothing to do.</summary>
    internal static void Update(in TopoDS_Vertex V)
    {
    }

    /// <summary>Updates the cached data of an edge: nothing to do.</summary>
    internal static void Update(in TopoDS_Edge E)
    {
    }

    /// <summary>Updates the cached data of a wire: nothing to do.</summary>
    internal static void Update(in TopoDS_Wire W)
    {
    }

    /// <summary>Updates the UV points of the face's edges, unless the face is checked, and marks it checked.</summary>
    internal static void Update(in TopoDS_Face F)
    {
        // OCCT.NET: frozen: a frozen face can't have changed since it was frozen, and can't be marked
        if (F.TShape()!.IsFrozen())
        {
            return;
        }

        if (!F.Checked())
        {
            UpdateFaceUVPoints(F);
            F.TShape()!.Checked(true);
        }
    }

    /// <summary>Updates the faces of the shell.</summary>
    internal static void Update(in TopoDS_Shell S)
    {
        var ex = new TopExp_Explorer(S, TopAbs_FACE);
        while (ex.More())
        {
            Update(TopoDS.Face(ex.Current()));
            ex.Next();
        }
    }

    /// <summary>Updates the faces of the solid.</summary>
    internal static void Update(in TopoDS_Solid S)
    {
        var ex = new TopExp_Explorer(S, TopAbs_FACE);
        while (ex.More())
        {
            Update(TopoDS.Face(ex.Current()));
            ex.Next();
        }
    }

    /// <summary>Updates the faces of the composite solid.</summary>
    internal static void Update(in TopoDS_CompSolid CS)
    {
        var ex = new TopExp_Explorer(CS, TopAbs_FACE);
        while (ex.More())
        {
            Update(TopoDS.Face(ex.Current()));
            ex.Next();
        }
    }

    /// <summary>Updates the faces of the compound.</summary>
    internal static void Update(in TopoDS_Compound C)
    {
        var ex = new TopExp_Explorer(C, TopAbs_FACE);
        while (ex.More())
        {
            Update(TopoDS.Face(ex.Current()));
            ex.Next();
        }
    }

    /// <summary>Updates the shape according to its type.</summary>
    internal static void Update(in TopoDS_Shape S)
    {
        switch (S.ShapeType())
        {
            case TopAbs_VERTEX:
                Update(TopoDS.Vertex(S));
                break;

            case TopAbs_EDGE:
                Update(TopoDS.Edge(S));
                break;

            case TopAbs_WIRE:
                Update(TopoDS.Wire(S));
                break;

            case TopAbs_FACE:
                Update(TopoDS.Face(S));
                break;

            case TopAbs_SHELL:
                Update(TopoDS.Shell(S));
                break;

            case TopAbs_SOLID:
                Update(TopoDS.Solid(S));
                break;

            case TopAbs_COMPSOLID:
                Update(TopoDS.CompSolid(S));
                break;

            case TopAbs_COMPOUND:
                Update(TopoDS.Compound(S));
                break;

            default:
                break;
        }
    }

    /// <summary>
    /// Resets the UV points of the face's edges to the end points of their pcurves on it. Stops at the first locked
    /// (in the port: also frozen) edge.
    /// </summary>
    internal static void UpdateFaceUVPoints(in TopoDS_Face theF)
    {
        // For each edge of the face <F> reset the UV points to the bounding
        // points of the parametric curve of the edge on the face.

        // Get surface of the face
        Geom_Surface? aSurf = BRep_Tool.Surface(theF, out TopLoc_Location aLoc);
        // Iterate on edges and reset UV points
        var anExpE = new TopExp_Explorer(theF, TopAbs_EDGE);
        for (; anExpE.More(); anExpE.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(anExpE.Current());

            BRep_TEdge TE = (BRep_TEdge)aE.TShape()!;
            if (TE.Locked())
            {
                // OCCT (occt-issues.md): returns at the first locked edge instead of skipping it
                return;
            }

            TopLoc_Location aELoc = aLoc.Predivided(aE.Location());
            // Edge representations
            NCollection_List<BRep_CurveRepresentation> aLCR = TE.ChangeCurves();
            var itLCR = new NCollection_List<BRep_CurveRepresentation>.Iterator(aLCR);
            for (; itLCR.More(); itLCR.Next())
            {
                BRep_GCurve? GC = itLCR.Value() as BRep_GCurve;
                if (GC is not null && GC.IsCurveOnSurface(aSurf, aELoc))
                {
                    // Update UV points
                    GC.Update();
                    break;
                }
            }
        }
    }

    /// <summary>True if the vertices are the same, or closer than either's tolerance.</summary>
    public static bool Compare(in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        if (V1.IsSame(V2))
        {
            return true;
        }

        gp_Pnt p1 = BRep_Tool.Pnt(V1);
        gp_Pnt p2 = BRep_Tool.Pnt(V2);
        double l = p1.Distance(p2);
        if (l <= BRep_Tool.Tolerance(V1))
        {
            return true;
        }

        if (l <= BRep_Tool.Tolerance(V2))
        {
            return true;
        }

        return false;
    }

    /// <summary>True if the edges are the same (IsSame).</summary>
    public static bool Compare(in TopoDS_Edge E1, in TopoDS_Edge E2) => E1.IsSame(E2);

    /// <summary>
    /// The outer wire of the face: the one whose UV bounds contain all the others' (the first wire if there is one
    /// wire), or a null wire.
    /// </summary>
    public static TopoDS_Wire OuterWire(in TopoDS_Face F)
    {
        TopoDS_Wire Wres = new TopoDS_Wire();
        var expw = new TopExp_Explorer(F, TopAbs_WIRE);

        if (expw.More())
        {
            Wres = TopoDS.Wire(expw.Current());
            expw.Next();
            if (expw.More())
            {
                BRepTools.UVBounds(F, Wres, out double UMin, out double UMax, out double VMin, out double VMax);
                while (expw.More())
                {
                    TopoDS_Wire W = TopoDS.Wire(expw.Current());
                    BRepTools.UVBounds(F, W, out double umin, out double umax, out double vmin, out double vmax);
                    if (((umin - UMin) <= Precision.PConfusion()) && ((umax - UMax) >= -Precision.PConfusion())
                        && ((vmin - VMin) <= Precision.PConfusion()) && ((vmax - VMax) >= -Precision.PConfusion()))
                    {
                        Wres = W;
                        UMin = umin;
                        UMax = umax;
                        VMin = vmin;
                        VMax = vmax;
                    }

                    expw.Next();
                }
            }
        }

        return Wres;
    }

    /// <summary>Adds the non-degenerated edges of the shape to <paramref name="M"/>.</summary>
    public static void Map3DEdges(in TopoDS_Shape S, NCollection_IndexedMap<TopoDS_Shape> M)
    {
        var Ex = new TopExp_Explorer();
        for (Ex.Init(S, TopAbs_EDGE); Ex.More(); Ex.Next())
        {
            if (!BRep_Tool.Degenerated(TopoDS.Edge(Ex.Current())))
            {
                M.Add(Ex.Current());
            }
        }
    }

    /// <summary>
    /// Removes the triangulations of the faces that have a surface, and the polygons of the edges that have geometry
    /// (with <paramref name="theForce"/>, all the polygons on triangulations).
    /// </summary>
    internal static void Clean(in TopoDS_Shape theShape, bool theForce = false)
    {
        if (theShape.IsNull())
        {
            return;
        }

        BRep_Builder aBuilder = new BRep_Builder();
        Poly_Triangulation? aNullTriangulation = null;
        Poly_PolygonOnTriangulation? aNullPoly = null;

        var aShapeMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopLoc_Location anEmptyLoc = new TopLoc_Location();

        var aFaceIt = new TopExp_Explorer(theShape, TopAbs_FACE);
        for (; aFaceIt.More(); aFaceIt.Next())
        {
            TopoDS_Shape aFaceNoLoc = aFaceIt.Value();
            aFaceNoLoc.Location(anEmptyLoc);
            if (!aShapeMap.Add(aFaceNoLoc))
            {
                // the face has already been processed
                continue;
            }

            TopoDS_Face aFace = TopoDS.Face(aFaceIt.Current());
            if (!BRep_Tool.IsGeometric(aFace))
            {
                // Do not remove triangulation as there is no surface to recompute it.
                continue;
            }

            Poly_Triangulation? aTriangulation = BRep_Tool.Triangulation(aFace, out TopLoc_Location aLoc);
            if (aTriangulation is null)
            {
                continue;
            }

            // Nullify edges
            // Theoretically, the edges on the face (with surface) may have no geometry
            // (no curve 3d or 2d or both). Such faces should be considered as invalid and
            // are not supported by current implementation. So, both triangulation of the face
            // and polygon on triangulation of the edges are removed unconditionally.
            var aEdgeIt = new TopExp_Explorer(aFace, TopAbs_EDGE);
            for (; aEdgeIt.More(); aEdgeIt.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(aEdgeIt.Current());
                aBuilder.UpdateEdge(anEdge, aNullPoly, aTriangulation, aLoc);
            }

            aBuilder.UpdateFace(aFace, aNullTriangulation);
        }

        // Iterate over all edges seeking for 3d polygons
        Poly_Polygon3D? aNullPoly3d = null;
        var aEdgeIt2 = new TopExp_Explorer(theShape, TopAbs_EDGE);
        for (; aEdgeIt2.More(); aEdgeIt2.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(aEdgeIt2.Value());
            anEdge.Location(anEmptyLoc);

            if (!aShapeMap.Add(anEdge))
            {
                // the edge has already been processed
                continue;
            }

            if (!BRep_Tool.IsGeometric(TopoDS.Edge(anEdge)))
            {
                // Do not remove polygon 3d as there is no curve to recompute it.
                continue;
            }

            Poly_Polygon3D? aPoly3d = BRep_Tool.Polygon3D(anEdge, out TopLoc_Location aLoc);
            if (aPoly3d is not null)
            {
                aBuilder.UpdateEdge(anEdge, aNullPoly3d);
            }

            if (theForce)
            {
                BRep_CurveRepresentation aCR;
                BRep_TEdge aTE = (BRep_TEdge)anEdge.TShape()!;
                NCollection_List<BRep_CurveRepresentation> aLCR = aTE.ChangeCurves();
                var anIterCR = new NCollection_List<BRep_CurveRepresentation>.Iterator(aLCR);
                // find and remove all representations
                while (anIterCR.More())
                {
                    aCR = anIterCR.Value();
                    if (aCR.IsPolygonOnTriangulation())
                    {
                        aLCR.Remove(anIterCR);
                    }
                    else
                    {
                        anIterCR.Next();
                    }
                }

                aTE.Modified(true);
            }
        }
    }

    /// <summary>Removes the surfaces of the faces, the curves of the edges and the unused pcurves.</summary>
    internal static void CleanGeometry(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return;
        }

        BRep_Builder aBuilder = new BRep_Builder();

        for (var aFaceIt = new TopExp_Explorer(theShape, TopAbs_FACE); aFaceIt.More(); aFaceIt.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceIt.Current());

            Geom_Surface? aSurface = BRep_Tool.Surface(aFace, out TopLoc_Location aLocation);

            for (var aEdgeIt = new TopExp_Explorer(aFace, TopAbs_EDGE); aEdgeIt.More(); aEdgeIt.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(aEdgeIt.Current());
                aBuilder.UpdateEdge(anEdge, (Geom2d_Curve?)null, aSurface, aLocation, BRep_Tool.Tolerance(anEdge));
            }

            aBuilder.UpdateFace(aFace, (Geom_Surface?)null, aFace.Location(), BRep_Tool.Tolerance(aFace));
        }

        for (var aEdgeIt2 = new TopExp_Explorer(theShape, TopAbs_EDGE); aEdgeIt2.More(); aEdgeIt2.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(aEdgeIt2.Current());

            aBuilder.UpdateEdge(anEdge, (Geom_Curve?)null, new TopLoc_Location(), BRep_Tool.Tolerance(anEdge));
        }

        RemoveUnusedPCurves(theShape);
    }

    /// <summary>Removes the pcurves and continuities on surfaces that none of the shape's faces uses.</summary>
    internal static void RemoveUnusedPCurves(in TopoDS_Shape S)
    {
        var UsedSurfaces = new NCollection_Map<object>(ReferenceEqualityComparer.Instance);

        var Explo = new TopExp_Explorer(S, TopAbs_FACE);
        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(Explo.Current());
            Geom_Surface? aSurf = BRep_Tool.Surface(aFace, out TopLoc_Location aLoc);
            // OCCT's map of handles holds the null handle too; a .NET set doesn't take null, and no curve
            // representation compares its surface with a null surface below except as "not used"
            if (aSurf is not null)
            {
                UsedSurfaces.Add(aSurf);
            }
            else
            {
                UsedSurfaces.Add(theNullSurface);
            }
        }

        var Emap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, Emap);

        int i;
        for (i = 1; i <= Emap.Extent(); i++)
        {
            BRep_TEdge TE = (BRep_TEdge)Emap.FindKey(i).TShape()!;
            NCollection_List<BRep_CurveRepresentation> lcr = TE.ChangeCurves();
            var itrep = new NCollection_List<BRep_CurveRepresentation>.Iterator(lcr);
            while (itrep.More())
            {
                bool ToRemove = false;

                BRep_CurveRepresentation CurveRep = itrep.Value();
                if (CurveRep.IsCurveOnSurface())
                {
                    Geom_Surface? aSurface = CurveRep.Surface();
                    if (!UsedSurfaces.Contains((object?)aSurface ?? theNullSurface))
                    {
                        ToRemove = true;
                    }
                }
                else if (CurveRep.IsRegularity())
                {
                    Geom_Surface? Surf1 = CurveRep.Surface();
                    Geom_Surface? Surf2 = CurveRep.Surface2();
                    ToRemove = (!UsedSurfaces.Contains((object?)Surf1 ?? theNullSurface)
                                || !UsedSurfaces.Contains((object?)Surf2 ?? theNullSurface));
                }

                if (ToRemove)
                {
                    lcr.Remove(itrep);
                }
                else
                {
                    itrep.Next();
                }
            }
        }
    }

    // Stands for OCCT's null handle in the set of used surfaces
    private static readonly object theNullSurface = new();

    /// <summary>
    /// True if every face of the shape has a triangulation with a deflection at most <paramref name="theLinDefl"/> and
    /// every edge a polygon on it; with <paramref name="theToCheckFreeEdges"/>, also every free edge a 3D polygon (or a
    /// polygon on a triangulation) with such a deflection.
    /// </summary>
    public static bool Triangulation(in TopoDS_Shape theShape, double theLinDefl, bool theToCheckFreeEdges = false)
    {
        var anEdgeIter = new TopExp_Explorer();
        TopLoc_Location aDummyLoc;
        for (var aFaceIter = new TopExp_Explorer(theShape, TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceIter.Current());
            Poly_Triangulation? aTri = BRep_Tool.Triangulation(aFace, out aDummyLoc);
            if (aTri is null || aTri.Deflection() > theLinDefl)
            {
                return false;
            }

            for (anEdgeIter.Init(aFace, TopAbs_EDGE); anEdgeIter.More(); anEdgeIter.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anEdgeIter.Current());
                Poly_PolygonOnTriangulation? aPoly = BRep_Tool.PolygonOnTriangulation(anEdge, aTri, aDummyLoc);
                if (aPoly is null)
                {
                    return false;
                }
            }
        }

        if (!theToCheckFreeEdges)
        {
            return true;
        }

        Poly_Triangulation? anEdgeTri;
        for (anEdgeIter.Init(theShape, TopAbs_EDGE, TopAbs_FACE); anEdgeIter.More(); anEdgeIter.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anEdgeIter.Current());
            Poly_Polygon3D? aPolygon = BRep_Tool.Polygon3D(anEdge, out aDummyLoc);
            if (aPolygon is not null)
            {
                if (aPolygon.Deflection() > theLinDefl)
                {
                    return false;
                }
            }
            else
            {
                BRep_Tool.PolygonOnTriangulation(anEdge, out Poly_PolygonOnTriangulation? aPoly, out anEdgeTri, out aDummyLoc);
                if (aPoly is null || anEdgeTri is null || anEdgeTri.Deflection() > theLinDefl)
                {
                    return false;
                }
            }
        }

        return true;
    }

    /// <summary>
    /// Loads the deferred data of the faces' triangulations (the active one if <paramref name="theTriangulationIdx"/>
    /// is -1, else the one of that index, from 0), activating them if asked. True if any was loaded.
    /// </summary>
    /// <remarks>OCCT's file-system argument is left out, as Poly_Triangulation's (decision 69).</remarks>
    internal static bool LoadTriangulation(in TopoDS_Shape theShape, int theTriangulationIdx = -1, bool theToSetAsActive = false)
    {
        Standard_ProgramError_Raise_if(!(theTriangulationIdx >= -1), "Invalid negative triangulation index!");

        bool wasLoaded = false;
        BRep_Builder aBuilder = new BRep_Builder();
        for (var aFaceIter = new TopExp_Explorer(theShape, TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceIter.Current());
            Poly_Triangulation? aTriangulation = null;
            if (theTriangulationIdx == -1)
            {
                // load an active triangulation
                aTriangulation = BRep_Tool.Triangulation(aFace, out _);
            }
            else
            {
                NCollection_List<Poly_Triangulation> aTriangulations = BRep_Tool.Triangulations(aFace, out _);
                if (theTriangulationIdx >= aTriangulations.Extent())
                {
                    // triangulation index is out of range
                    continue;
                }

                int aTriangulationIdx = 0;
                for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(aTriangulations);
                     anIter.More();
                     anIter.Next(), aTriangulationIdx++)
                {
                    if (aTriangulationIdx != theTriangulationIdx)
                    {
                        continue;
                    }

                    aTriangulation = anIter.Value();
                    break;
                }
            }

            if (aTriangulation is null || !aTriangulation.HasDeferredData())
            {
                // NULL triangulation, already loaded triangulation or triangulation without deferred storage
                // cannot be loaded
                continue;
            }

            if (aTriangulation.LoadDeferredData())
            {
                wasLoaded = true;
                if (theToSetAsActive && (theTriangulationIdx != -1)) // triangulation is already active
                {
                    aBuilder.UpdateFace(aFace, aTriangulation, false);
                }
            }
        }

        return wasLoaded;
    }

    /// <summary>Loads the deferred data of all the faces' triangulations. True if any was loaded.</summary>
    /// <remarks>OCCT's file-system argument is left out, as Poly_Triangulation's (decision 69).</remarks>
    internal static bool LoadAllTriangulations(in TopoDS_Shape theShape)
    {
        bool wasLoaded = false;
        for (var aFaceIter = new TopExp_Explorer(theShape, TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceIter.Current());
            for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(BRep_Tool.Triangulations(aFace, out _));
                 anIter.More();
                 anIter.Next())
            {
                Poly_Triangulation aTriangulation = anIter.Value();
                if (aTriangulation is null || !aTriangulation.HasDeferredData())
                {
                    // NULL triangulation, already loaded triangulation or triangulation without deferred
                    // storage cannot be loaded
                    continue;
                }

                wasLoaded = aTriangulation.LoadDeferredData();
            }
        }

        return wasLoaded;
    }

    /// <summary>
    /// Releases the data of the faces' triangulations that have deferred storage (the active one if
    /// <paramref name="theTriangulationIdx"/> is -1, else the one of that index). True if any was unloaded.
    /// </summary>
    internal static bool UnloadTriangulation(in TopoDS_Shape theShape, int theTriangulationIdx = -1)
    {
        Standard_ProgramError_Raise_if(!(theTriangulationIdx >= -1), "Invalid negative triangulation index!");

        bool wasUnloaded = false;
        for (var aFaceIter = new TopExp_Explorer(theShape, TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceIter.Current());
            Poly_Triangulation? aTriangulation = null;
            if (theTriangulationIdx == -1)
            {
                // unload an active triangulation
                aTriangulation = BRep_Tool.Triangulation(aFace, out _);
            }
            else
            {
                int aTriangulationIdx = 0;
                NCollection_List<Poly_Triangulation> aTriangulations = BRep_Tool.Triangulations(aFace, out _);
                if (theTriangulationIdx >= aTriangulations.Extent())
                {
                    // triangulation index is out of range
                    continue;
                }

                for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(aTriangulations);
                     anIter.More();
                     anIter.Next(), aTriangulationIdx++)
                {
                    if (aTriangulationIdx != theTriangulationIdx)
                    {
                        continue;
                    }

                    aTriangulation = anIter.Value();
                    break;
                }
            }

            if (aTriangulation is null || !aTriangulation.HasDeferredData())
            {
                // NULL triangulation or triangulation without deferred storage cannot be unloaded
                continue;
            }

            wasUnloaded = aTriangulation.UnloadDeferredData();
        }

        return wasUnloaded;
    }

    /// <summary>Releases the data of all the faces' triangulations that have deferred storage. True if any was unloaded.</summary>
    internal static bool UnloadAllTriangulations(in TopoDS_Shape theShape)
    {
        bool wasUnloaded = false;
        for (var aFaceIter = new TopExp_Explorer(theShape, TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceIter.Current());
            Poly_Triangulation? aTriangulation;
            for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(BRep_Tool.Triangulations(aFace, out _));
                 anIter.More();
                 anIter.Next())
            {
                aTriangulation = anIter.Value();
                if (aTriangulation is null || !aTriangulation.HasDeferredData())
                {
                    // NULL triangulation or triangulation without deferred storage cannot be unloaded
                    continue;
                }

                wasUnloaded = aTriangulation.UnloadDeferredData();
            }
        }

        return wasUnloaded;
    }

    /// <summary>
    /// Activates the triangulation of index <paramref name="theTriangulationIdx"/> (from 0) of each face; without
    /// <paramref name="theToActivateStrictly"/>, the last one of a face with fewer. True if any was changed.
    /// </summary>
    internal static bool ActivateTriangulation(in TopoDS_Shape theShape, int theTriangulationIdx, bool theToActivateStrictly = false)
    {
        Standard_ProgramError_Raise_if(!(theTriangulationIdx > -1), "Invalid negative triangulation index!");

        bool wasActivated = false;
        BRep_Builder aBuilder = new BRep_Builder();
        for (var aFaceIter = new TopExp_Explorer(theShape, TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceIter.Current());
            int aTriangulationIdx = theTriangulationIdx;
            NCollection_List<Poly_Triangulation> aTriangulations = BRep_Tool.Triangulations(aFace, out _);
            int aTriangulationsNb = aTriangulations.Extent();
            if (theTriangulationIdx >= aTriangulationsNb)
            {
                // triangulation index is out of range
                if (theToActivateStrictly)
                {
                    // skip activation
                    continue;
                }

                // use last available
                aTriangulationIdx = aTriangulationsNb - 1;
            }

            Poly_Triangulation? anActiveTriangulation = null;
            int aTriangulationIter = 0;
            for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(aTriangulations);
                 anIter.More();
                 anIter.Next(), aTriangulationIter++)
            {
                if (aTriangulationIter != aTriangulationIdx)
                {
                    continue;
                }

                anActiveTriangulation = anIter.Value();
                break;
            }

            if (anActiveTriangulation is null)
            {
                continue;
            }

            aBuilder.UpdateFace(aFace, anActiveTriangulation, false);
            wasActivated = true;
        }

        return wasActivated;
    }

    /// <summary>True if the edge is closed on the face (a seam) and appears twice in it.</summary>
    public static bool IsReallyClosed(in TopoDS_Edge E, in TopoDS_Face F)
    {
        if (!BRep_Tool.IsClosed(E, F))
        {
            return false;
        }

        int nbocc = 0;
        var exp = new TopExp_Explorer();
        for (exp.Init(F, TopAbs_EDGE); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(E))
            {
                nbocc++;
            }
        }

        return nbocc == 2;
    }

    /// <summary>Whether the face is closed in U and in V: it has a seam along V (U-closed) or along U (V-closed).</summary>
    public static void DetectClosedness(in TopoDS_Face theFace, out bool theUclosed, out bool theVclosed)
    {
        theUclosed = theVclosed = false;

        var Explo = new TopExp_Explorer(theFace, TopAbs_EDGE);
        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(Explo.Current());
            if (BRep_Tool.IsClosed(anEdge, theFace) && BRepTools.IsReallyClosed(anEdge, theFace))
            {
                Geom2d_Curve PCurve1 = BRep_Tool.CurveOnSurface(anEdge, theFace, out double fpar, out double lpar)!;
                Geom2d_Curve PCurve2 =
                    BRep_Tool.CurveOnSurface(TopoDS.Edge(anEdge.Reversed()), theFace, out fpar, out lpar)!;
                gp_Pnt2d Point1 = PCurve1.Value(fpar);
                gp_Pnt2d Point2 = PCurve2.Value(fpar);
                bool IsUiso = (Math.Abs(Point1.X() - Point2.X()) > Math.Abs(Point1.Y() - Point2.Y()));
                if (IsUiso)
                {
                    theUclosed = true;
                }
                else
                {
                    theVclosed = true;
                }
            }
        }
    }

    /// <summary>
    /// The deviation of the edge's pcurve on the surface from its 3D curve over [f, l]; raises the edge's tolerance to
    /// it if larger. Returns the tolerance.
    /// </summary>
    /// <exception cref="Standard_NotImplemented">Until unit 5A (GeomLib_CheckCurveOnSurface) is merged.</exception>
    internal static double EvalAndUpdateTol(in TopoDS_Edge theE,
                                            Geom_Curve C3d,
                                            Geom2d_Curve C2d,
                                            Geom_Surface S,
                                            double f,
                                            double l)
    {
        // TODO(5A): GeomLib_CheckCurveOnSurface, then the estimate by sample points of OCCT's error statuses 2 and 3.
        throw new Standard_NotImplemented("BRepTools::EvalAndUpdateTol needs GeomLib_CheckCurveOnSurface (unit 5A)");
    }

    /// <summary>The orientation of the edge in the face.</summary>
    /// <exception cref="Standard_ConstructionError">The edge isn't in the face.</exception>
    public static TopAbs_Orientation OriEdgeInFace(in TopoDS_Edge E, in TopoDS_Face F)
    {
        var Exp = new TopExp_Explorer(F.Oriented(TopAbs_FORWARD), TopAbs_EDGE);

        for (; Exp.More(); Exp.Next())
        {
            if (Exp.Current().IsSame(E))
            {
                return Exp.Current().Orientation();
            }
        }

        throw new Standard_ConstructionError("BRepTools::OriEdgeInFace");
    }

    // Looks for internal sub-shapes which has to be kept to preserve topological connectivity.
    private static void findInternalsToKeep(in TopoDS_Shape theS,
                                            NCollection_Map<TopoDS_Shape> theAllNonInternals,
                                            NCollection_Map<TopoDS_Shape> theAllInternals,
                                            NCollection_Map<TopoDS_Shape> theShapesToKeep)
    {
        for (var it = new TopoDS_Iterator(theS, true); it.More(); it.Next())
        {
            TopoDS_Shape aSS = it.Value();
            findInternalsToKeep(aSS, theAllNonInternals, theAllInternals, theShapesToKeep);

            if (aSS.Orientation() == TopAbs_INTERNAL)
            {
                theAllInternals.Add(aSS);
            }
            else
            {
                theAllNonInternals.Add(aSS);
            }

            if (theAllNonInternals.Contains(aSS) && theAllInternals.Contains(aSS))
            {
                theShapesToKeep.Add(aSS);
            }
        }
    }

    // Removes sub-shapes from the shape
    private static void removeShapes(in TopoDS_Shape theS, NCollection_List<TopoDS_Shape> theLS)
    {
        BRep_Builder aBB = new BRep_Builder();
        bool isFree = theS.Free();
        theS.Free(true);

        for (var it = new NCollection_List<TopoDS_Shape>.Iterator(theLS); it.More(); it.Next())
        {
            aBB.Remove(theS, it.Value());
        }

        theS.Free(isFree);
    }

    // Removes recursively all internal sub-shapes from the given shape.
    // Returns true if all sub-shapes have been removed from the shape.
    private static bool removeInternals(in TopoDS_Shape theS, NCollection_Map<TopoDS_Shape>? theShapesToKeep)
    {
        var aLRemove = new NCollection_List<TopoDS_Shape>();
        for (var it = new TopoDS_Iterator(theS, true); it.More(); it.Next())
        {
            TopoDS_Shape aSS = it.Value();
            if (aSS.Orientation() == TopAbs_INTERNAL)
            {
                if (theShapesToKeep is null || !theShapesToKeep.Contains(aSS))
                {
                    aLRemove.Append(aSS);
                }
            }
            else
            {
                // OCCT passes the iterator's current shape by non-const reference; only its TShape changes
                if (removeInternals(aSS, theShapesToKeep))
                {
                    aLRemove.Append(aSS);
                }
            }
        }

        int aNbSToRemove = aLRemove.Extent();
        if (aNbSToRemove != 0)
        {
            removeShapes(theS, aLRemove);
            return (theS.NbChildren() == 0);
        }

        return false;
    }

    /// <summary>
    /// Removes the INTERNAL sub-shapes of the shape, recursively; without <paramref name="theForce"/>, keeps those
    /// that connect the remaining sub-shapes.
    /// </summary>
    /// <remarks>Takes the shape by <c>in</c> (OCCT: a non-const reference): only its TShape changes.</remarks>
    internal static void RemoveInternals(in TopoDS_Shape theS, bool theForce = false)
    {
        NCollection_Map<TopoDS_Shape>? pMKeep = null;
        var aMKeep = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        if (!theForce)
        {
            // Find all internal sub-shapes which has to be kept to preserve topological connectivity.
            // Note that if the multi-connected shape is not directly contained in some shape,
            // but as a part of bigger sub-shape which will be removed, the multi-connected
            // shape is going to be removed also, breaking topological connectivity.
            // For instance, <theS> is a compound of the face and edge, which does not
            // belong to the face. The face contains internal wire and the edge shares
            // the vertex with one of the vertices of that wire. The vertex is not directly
            // contained in the face, thus will be removed as part of internal wire, and topological
            // connectivity between edge and face will be lost.
            var anAllNonInternals = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var anAllInternals = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            findInternalsToKeep(theS, anAllNonInternals, anAllInternals, aMKeep);
            if (aMKeep.Extent() != 0)
            {
                pMKeep = aMKeep;
            }
        }

        removeInternals(theS, pMKeep);
    }

    /// <summary>
    /// Adds to <paramref name="theProblemShapes"/> the sub-shapes (the shape included) whose own location has a scale
    /// or is negative.
    /// </summary>
    public static void CheckLocations(in TopoDS_Shape theS, NCollection_List<TopoDS_Shape> theProblemShapes)
    {
        if (theS.IsNull())
        {
            return;
        }

        var aMapS = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theS, aMapS, false, false);

        int i;
        for (i = 1; i <= aMapS.Extent(); ++i)
        {
            TopoDS_Shape anS = aMapS.FindKey(i);
            TopLoc_Location aLoc = anS.Location();
            gp_Trsf aTrsf = aLoc.Transformation();
            bool isBadTrsf = aTrsf.IsNegative()
                             || (Math.Abs(Math.Abs(aTrsf.ScaleFactor()) - 1.0) > TopLoc_Location.ScalePrec());

            if (isBadTrsf)
            {
                theProblemShapes.Append(anS);
            }
        }
    }
}
