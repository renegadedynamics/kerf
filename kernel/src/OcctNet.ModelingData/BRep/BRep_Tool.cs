// Port of: src/ModelingData/TKBRep/BRep/BRep_Tool.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.Poly_MeshPurpose;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Reads the geometry of BRep shapes: the surface of a face, the curves of an edge, the point of a vertex, their
/// tolerances, parameters and polygons.
/// </summary>
/// <remarks>
/// The geometry returned is the shapes' own, frozen, except where a method says it returns a transformed copy. Outputs
/// are <c>out</c> parameters. <see cref="SetUVPoints(in TopoDS_Edge, Geom_Surface?, in TopLoc_Location, in gp_Pnt2d, in gp_Pnt2d)"/>
/// changes an edge and is internal.
/// </remarks>
public static class BRep_Tool
{
    // Checks trim range against the effective basis curve.
    private static bool isValidTrimRange(Geom_Curve theCurve, double theFirst, double theLast)
    {
        double aTrimFirst = Min(theFirst, theLast);
        double aTrimLast = Max(theFirst, theLast);
        if (!(aTrimFirst < aTrimLast))
        {
            return false;
        }

        // Unwrap nested trims.
        Geom_Curve aBasisCurve = theCurve;
        for (Geom_TrimmedCurve? aTrim = aBasisCurve as Geom_TrimmedCurve; aTrim is not null;
             aTrim = aBasisCurve as Geom_TrimmedCurve)
        {
            aBasisCurve = aTrim.BasisCurve();
        }

        return aBasisCurve.IsPeriodic()
               || (aBasisCurve.FirstParameter() - aTrimFirst <= Precision.PConfusion()
                   && aTrimLast - aBasisCurve.LastParameter() <= Precision.PConfusion());
    }

    /// <summary>The surface of the face, with in <paramref name="L"/> the location for it.</summary>
    public static Geom_Surface? Surface(in TopoDS_Face F, out TopLoc_Location L)
    {
        BRep_TFace TF = (BRep_TFace)F.TShape()!;
        L = F.Location() * TF.Location();
        return TF.Surface();
    }

    /// <summary>The surface of the face, placed: a transformed copy if the face has a location.</summary>
    public static Geom_Surface? Surface(in TopoDS_Face F)
    {
        BRep_TFace TF = (BRep_TFace)F.TShape()!;
        Geom_Surface? S = TF.Surface();

        if (S is null)
        {
            return S;
        }

        TopLoc_Location L = F.Location() * TF.Location();
        if (!L.IsIdentity())
        {
            Geom_Geometry aCopy = S.Transformed(L.Transformation());
            return (Geom_Surface)aCopy;
        }

        return S;
    }

    /// <summary>
    /// The face's triangulation for the purpose (the active one for NONE), with in <paramref name="theLocation"/> the
    /// location for it; null if there is none.
    /// </summary>
    public static Poly_Triangulation? Triangulation(in TopoDS_Face theFace,
                                                    out TopLoc_Location theLocation,
                                                    Poly_MeshPurpose theMeshPurpose = Poly_MeshPurpose_NONE)
    {
        theLocation = theFace.Location();
        BRep_TFace aTFace = (BRep_TFace)theFace.TShape()!;
        return aTFace.Triangulation(theMeshPurpose);
    }

    /// <summary>The face's triangulations, with in <paramref name="theLocation"/> the location for them.</summary>
    /// <remarks>Internal: the list is the face's own. Callers use <see cref="BRep_TFace.TriangulationList"/>.</remarks>
    internal static NCollection_List<Poly_Triangulation> Triangulations(in TopoDS_Face theFace,
                                                                       out TopLoc_Location theLocation)
    {
        theLocation = theFace.Location();
        BRep_TFace aTFace = (BRep_TFace)theFace.TShape()!;
        return aTFace.Triangulations();
    }

    /// <summary>The tolerance of the face, at least Precision::Confusion().</summary>
    public static double Tolerance(in TopoDS_Face F)
    {
        BRep_TFace TF = (BRep_TFace)F.TShape()!;
        double p = TF.Tolerance();
        double pMin = Precision.Confusion();
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
    public static bool NaturalRestriction(in TopoDS_Face F)
    {
        BRep_TFace TF = (BRep_TFace)F.TShape()!;
        return TF.NaturalRestriction();
    }

    /// <summary>
    /// The 3D curve of the edge, or null; in <paramref name="L"/> the location for it, in <paramref name="First"/> and
    /// <paramref name="Last"/> its range.
    /// </summary>
    public static Geom_Curve? Curve(in TopoDS_Edge E, out TopLoc_Location L, out double First, out double Last)
    {
        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurve3D())
            {
                BRep_Curve3D GC = (BRep_Curve3D)cr;
                L = E.Location() * GC.Location();
                GC.Range(out First, out Last);
                return GC.Curve3D();
            }

            itcr.Next();
        }

        L = new TopLoc_Location();
        First = Last = 0.0;
        return null;
    }

    /// <summary>
    /// The 3D curve of the edge, placed (a transformed copy if the edge has a location), or null; its range in
    /// <paramref name="First"/> and <paramref name="Last"/>.
    /// </summary>
    public static Geom_Curve? Curve(in TopoDS_Edge E, out double First, out double Last)
    {
        Geom_Curve? C = Curve(E, out TopLoc_Location L, out First, out Last);
        if (C is not null)
        {
            if (!L.IsIdentity())
            {
                Geom_Geometry aCopy = C.Transformed(L.Transformation());
                return (Geom_Curve)aCopy;
            }
        }

        return C;
    }

    /// <summary>True if the face has a surface.</summary>
    public static bool IsGeometric(in TopoDS_Face F)
    {
        BRep_TFace TF = (BRep_TFace)F.TShape()!;
        Geom_Surface? S = TF.Surface();
        return S is not null;
    }

    /// <summary>True if the edge has a 3D curve or a curve on a surface.</summary>
    public static bool IsGeometric(in TopoDS_Edge E)
    {
        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurve3D())
            {
                BRep_Curve3D? GC = cr as BRep_Curve3D;
                if (GC is not null && GC.Curve3D() is not null)
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

    /// <summary>The 3D polygon of the edge, or null; in <paramref name="L"/> the location for it.</summary>
    public static Poly_Polygon3D? Polygon3D(in TopoDS_Edge E, out TopLoc_Location L)
    {
        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygon3D())
            {
                BRep_Polygon3D GC = (BRep_Polygon3D)cr;
                L = E.Location() * GC.Location();
                return GC.Polygon3D();
            }

            itcr.Next();
        }

        L = new TopLoc_Location();
        return null;
    }

    /// <summary>
    /// The curve of the edge in the parametric space of the face, or null; its range in <paramref name="First"/> and
    /// <paramref name="Last"/>. For a plane without a stored pcurve, the projection of the 3D curve.
    /// </summary>
    public static Geom2d_Curve? CurveOnSurface(in TopoDS_Edge E, in TopoDS_Face F, out double First, out double Last)
        => CurveOnSurface(E, F, out First, out Last, out _);

    /// <summary>
    /// The curve of the edge in the parametric space of the face, or null; its range in <paramref name="First"/> and
    /// <paramref name="Last"/>, and whether it is stored on the edge (false for a computed projection on a plane).
    /// </summary>
    public static Geom2d_Curve? CurveOnSurface(in TopoDS_Edge E,
                                               in TopoDS_Face F,
                                               out double First,
                                               out double Last,
                                               out bool theIsStored)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location l);
        TopoDS_Edge aLocalEdge = E;
        if (F.Orientation() == TopAbs_REVERSED)
        {
            aLocalEdge.Reverse();
        }

        return CurveOnSurface(aLocalEdge, S, l, out First, out Last, out theIsStored);
    }

    /// <summary>
    /// The curve of the edge in the parametric space of the surface, or null; its range in <paramref name="First"/>
    /// and <paramref name="Last"/>. For a plane without a stored pcurve, the projection of the 3D curve.
    /// </summary>
    public static Geom2d_Curve? CurveOnSurface(in TopoDS_Edge E,
                                               Geom_Surface? S,
                                               in TopLoc_Location L,
                                               out double First,
                                               out double Last)
        => CurveOnSurface(E, S, L, out First, out Last, out _);

    /// <summary>
    /// The curve of the edge in the parametric space of the surface, or null; its range in <paramref name="First"/>
    /// and <paramref name="Last"/>, and whether it is stored on the edge (false for a computed projection on a plane).
    /// </summary>
    public static Geom2d_Curve? CurveOnSurface(in TopoDS_Edge E,
                                               Geom_Surface? S,
                                               in TopLoc_Location L,
                                               out double First,
                                               out double Last,
                                               out bool theIsStored)
    {
        bool Eisreversed = (E.Orientation() == TopAbs_REVERSED);
        theIsStored = true;

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        if (itcr.More())
        {
            TopLoc_Location loc = L.Predivided(E.Location());
            while (itcr.More())
            {
                BRep_CurveRepresentation cr = itcr.Value();
                if (cr.IsCurveOnSurface(S, loc))
                {
                    BRep_GCurve GC = (BRep_GCurve)cr;
                    GC.Range(out First, out Last);
                    if (GC.IsCurveOnClosedSurface() && Eisreversed)
                    {
                        return GC.PCurve2();
                    }
                    else
                    {
                        return GC.PCurve();
                    }
                }

                itcr.Next();
            }
        }

        // Curve is not found. Try projection on plane
        theIsStored = false;
        return CurveOnPlane(E, S, L, out First, out Last);
    }

    /// <summary>
    /// For a plane, the projection of the edge's 3D curve on it, as a curve in its parametric space; its range in
    /// <paramref name="First"/> and <paramref name="Last"/>. Null for other surfaces or an edge without a 3D curve.
    /// </summary>
    /// <exception cref="Standard_NotImplemented">The projection itself, until unit 6A (ProjLib_ProjectedCurve and GeomProjLib) is merged.</exception>
    public static Geom2d_Curve? CurveOnPlane(in TopoDS_Edge E,
                                             Geom_Surface? S,
                                             in TopLoc_Location L,
                                             out double First,
                                             out double Last)
    {
        First = Last = 0.0;

        // Check if the surface is planar
        Geom_Plane? GP;
        Geom_RectangularTrimmedSurface? GRTS = S as Geom_RectangularTrimmedSurface;
        if (GRTS is not null)
        {
            GP = GRTS.BasisSurface() as Geom_Plane;
        }
        else
        {
            GP = S as Geom_Plane;
        }

        if (GP is null)
        {
            // not a plane
            return null;
        }

        // Check existence of 3d curve in edge
        Geom_Curve? C3D = BRep_Tool.Curve(E, out TopLoc_Location aCurveLocation, out double f, out double l);
        if (C3D is null)
        {
            // no 3d curve
            return null;
        }

        aCurveLocation = aCurveLocation.Predivided(L);
        First = f;
        Last = l;

        // Transform curve and update parameters in account of scale factor
        if (!aCurveLocation.IsIdentity())
        {
            gp_Trsf aTrsf = aCurveLocation.Transformation();
            C3D = (Geom_Curve)C3D.Transformed(aTrsf);
            f = C3D.TransformedParameter(f, aTrsf);
            l = C3D.TransformedParameter(l, aTrsf);
        }

        if (!isValidTrimRange(C3D, f, l))
        {
            return null;
        }

        var aTrimmedCurve = new Geom_TrimmedCurve(C3D, f, l, true, false);

        // Perform projection
        Geom_Curve? ProjOnPlane = GeomProjLib.ProjectOnPlane(aTrimmedCurve, GP, GP.Position().Direction(), true);

        var HS = new GeomAdaptor_Surface(GP);
        var HC = new GeomAdaptor_Curve(ProjOnPlane!);

        var Proj = new ProjLib_ProjectedCurve(HS, HC);
        Geom2d_Curve pc = Geom2dAdaptor.MakeCurve(Proj);

        if (pc is Geom2d_TrimmedCurve TC)
        {
            pc = TC.BasisCurve();
        }

        return pc;
    }

    /// <summary>The first curve on a surface of the edge, with its surface, location and range; nulls if none.</summary>
    public static void CurveOnSurface(in TopoDS_Edge E,
                                      out Geom2d_Curve? C,
                                      out Geom_Surface? S,
                                      out TopLoc_Location L,
                                      out double First,
                                      out double Last)
    {
        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface())
            {
                BRep_GCurve GC = (BRep_GCurve)cr;
                C = GC.PCurve();
                S = GC.Surface();
                L = E.Location() * GC.Location();
                GC.Range(out First, out Last);
                return;
            }

            itcr.Next();
        }

        C = null;
        S = null;
        L = new TopLoc_Location();
        First = Last = 0.0;
    }

    /// <summary>
    /// The <paramref name="Index"/>-th curve on a surface of the edge (from 1; a seam counts twice), with its surface,
    /// location and range; nulls if none. The outputs are left unchanged if Index is less than 1.
    /// </summary>
    public static void CurveOnSurface(in TopoDS_Edge E,
                                      ref Geom2d_Curve? C,
                                      ref Geom_Surface? S,
                                      ref TopLoc_Location L,
                                      ref double First,
                                      ref double Last,
                                      int Index)
    {
        if (Index < 1)
        {
            return;
        }

        int i = 0;
        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface())
            {
                BRep_GCurve GC = (BRep_GCurve)cr;
                ++i;
                // Compare index taking into account the fact that for the curves on
                // closed surfaces there are two PCurves
                if (i == Index)
                {
                    C = GC.PCurve();
                }
                else if (GC.IsCurveOnClosedSurface() && (++i == Index))
                {
                    C = GC.PCurve2();
                }
                else
                {
                    continue;
                }

                S = GC.Surface();
                L = E.Location() * GC.Location();
                GC.Range(out First, out Last);
                return;
            }
        }

        C = null;
        S = null;
        L = new TopLoc_Location();
        First = Last = 0.0;
    }

    /// <summary>The polygon of the edge in the parametric space of the face, or null.</summary>
    public static Poly_Polygon2D? PolygonOnSurface(in TopoDS_Edge E, in TopoDS_Face F)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location l);
        TopoDS_Edge aLocalEdge = E;
        if (F.Orientation() == TopAbs_REVERSED)
        {
            aLocalEdge.Reverse();
        }

        return PolygonOnSurface(aLocalEdge, S, l);
    }

    /// <summary>The polygon of the edge in the parametric space of the surface, or null.</summary>
    public static Poly_Polygon2D? PolygonOnSurface(in TopoDS_Edge E, Geom_Surface? S, in TopLoc_Location L)
    {
        TopLoc_Location l = L.Predivided(E.Location());
        bool Eisreversed = (E.Orientation() == TopAbs_REVERSED);

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnSurface(S, l))
            {
                if (cr.IsPolygonOnClosedSurface() && Eisreversed)
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
    public static void PolygonOnSurface(in TopoDS_Edge E,
                                        out Poly_Polygon2D? P,
                                        out Geom_Surface? S,
                                        out TopLoc_Location L)
    {
        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnSurface())
            {
                BRep_PolygonOnSurface PS = (BRep_PolygonOnSurface)cr;
                P = PS.Polygon();
                S = PS.Surface();
                L = E.Location() * PS.Location();
                return;
            }

            itcr.Next();
        }

        L = new TopLoc_Location();
        P = null;
        S = null;
    }

    /// <summary>The <paramref name="Index"/>-th polygon on a surface of the edge (from 1), with its surface and location; nulls if none.</summary>
    public static void PolygonOnSurface(in TopoDS_Edge E,
                                        out Poly_Polygon2D? P,
                                        out Geom_Surface? S,
                                        out TopLoc_Location L,
                                        int Index)
    {
        int i = 0;

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnSurface())
            {
                BRep_PolygonOnSurface PS = (BRep_PolygonOnSurface)cr;
                i++;
                if (i > Index)
                {
                    break;
                }

                if (i == Index)
                {
                    P = PS.Polygon();
                    S = PS.Surface();
                    L = E.Location() * PS.Location();
                    return;
                }
            }

            itcr.Next();
        }

        L = new TopLoc_Location();
        P = null;
        S = null;
    }

    /// <summary>The polygon of the edge on the triangulation <paramref name="T"/> with location <paramref name="L"/>, or null.</summary>
    public static Poly_PolygonOnTriangulation? PolygonOnTriangulation(in TopoDS_Edge E,
                                                                      Poly_Triangulation? T,
                                                                      in TopLoc_Location L)
    {
        TopLoc_Location l = L.Predivided(E.Location());
        bool Eisreversed = (E.Orientation() == TopAbs_REVERSED);

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnTriangulation(T, l))
            {
                if (cr.IsPolygonOnClosedTriangulation() && Eisreversed)
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
    public static void PolygonOnTriangulation(in TopoDS_Edge E,
                                              out Poly_PolygonOnTriangulation? P,
                                              out Poly_Triangulation? T,
                                              out TopLoc_Location L)
    {
        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnTriangulation())
            {
                BRep_PolygonOnTriangulation PT = (BRep_PolygonOnTriangulation)cr;
                P = PT.PolygonOnTriangulation();
                T = PT.Triangulation();
                L = E.Location() * PT.Location();
                return;
            }

            itcr.Next();
        }

        L = new TopLoc_Location();
        P = null;
        T = null;
    }

    /// <summary>
    /// The <paramref name="Index"/>-th polygon on a triangulation of the edge (from 1), with its triangulation and
    /// location; nulls if none.
    /// </summary>
    public static void PolygonOnTriangulation(in TopoDS_Edge E,
                                              out Poly_PolygonOnTriangulation? P,
                                              out Poly_Triangulation? T,
                                              out TopLoc_Location L,
                                              int Index)
    {
        int i = 0;

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnTriangulation())
            {
                BRep_PolygonOnTriangulation PT = (BRep_PolygonOnTriangulation)cr;
                i++;
                if (i > Index)
                {
                    break;
                }

                if (i == Index)
                {
                    T = PT.Triangulation();
                    P = PT.PolygonOnTriangulation();
                    L = E.Location() * PT.Location();
                    return;
                }
            }

            itcr.Next();
        }

        L = new TopLoc_Location();
        P = null;
        T = null;
    }

    /// <summary>
    /// True if the edge has two pcurves in the parametric space of the face (it is on the closing curve of a closed
    /// surface), or two polygons on its triangulation.
    /// </summary>
    public static bool IsClosed(in TopoDS_Edge E, in TopoDS_Face F)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location l);
        if (IsClosed(E, S, l))
        {
            return true;
        }

        Poly_Triangulation? T = BRep_Tool.Triangulation(F, out l);
        return IsClosed(E, T, l);
    }

    /// <summary>
    /// True if the edge has two pcurves in the parametric space of the surface (it is on the closing curve of a closed
    /// surface). False for planes.
    /// </summary>
    public static bool IsClosed(in TopoDS_Edge E, Geom_Surface? S, in TopLoc_Location L)
    {
        // modified by NIZNHY-PKV Fri Oct 17 12:16:58 2008f
        if (IsPlane(S))
        {
            return false;
        }

        TopLoc_Location l = L.Predivided(E.Location());

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(S, l) && cr.IsCurveOnClosedSurface())
            {
                return true;
            }

            itcr.Next();
        }

        return false;
    }

    /// <summary>True if the edge has two arrays of indices in the triangulation.</summary>
    public static bool IsClosed(in TopoDS_Edge E, Poly_Triangulation? T, in TopLoc_Location L)
    {
        if (T is null)
        {
            return false;
        }

        TopLoc_Location l = L.Predivided(E.Location());

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnTriangulation(T, l) && cr.IsPolygonOnClosedTriangulation())
            {
                return true;
            }

            itcr.Next();
        }

        return false;
    }

    /// <summary>The tolerance of the edge, at least Precision::Confusion().</summary>
    public static double Tolerance(in TopoDS_Edge E)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        double p = TE.Tolerance();
        double pMin = Precision.Confusion();
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
    public static bool SameParameter(in TopoDS_Edge E)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        return TE.SameParameter();
    }

    /// <summary>The SameRange flag of the edge.</summary>
    public static bool SameRange(in TopoDS_Edge E)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        return TE.SameRange();
    }

    /// <summary>True if the edge is degenerated.</summary>
    public static bool Degenerated(in TopoDS_Edge E)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        return TE.Degenerated();
    }

    /// <summary>The range of the edge: of its 3D curve, else of its first curve on a surface; zeros if none.</summary>
    public static void Range(in TopoDS_Edge E, out double First, out double Last)
    {
        //  set the range to all the representations
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurve3D())
            {
                BRep_Curve3D CR = (BRep_Curve3D)cr;
                if (CR.Curve3D() is not null)
                {
                    First = CR.First();
                    Last = CR.Last();
                    return;
                }
            }
            else if (cr.IsCurveOnSurface())
            {
                BRep_GCurve CR = (BRep_GCurve)cr;
                First = CR.First();
                Last = CR.Last();
                return;
            }

            itcr.Next();
        }

        First = Last = 0.0;
    }

    /// <summary>The range of the edge's curve on the surface; the edge's range if it has none.</summary>
    /// <remarks>OCCT sets the edge's Modified flag here; the port does too, unless the edge is frozen.</remarks>
    public static void Range(in TopoDS_Edge E, Geom_Surface? S, in TopLoc_Location L, out double First, out double Last)
    {
        TopLoc_Location l = L.Predivided(E.Location());

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        First = Last = 0.0;

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(S, l))
            {
                BRep_CurveOnSurface CR = (BRep_CurveOnSurface)cr;
                CR.Range(out First, out Last);
                break;
            }

            itcr.Next();
        }

        if (!itcr.More())
        {
            Range(E, out First, out Last);
        }

        // OCCT (occt-issues.md): a query that sets the Modified flag. OCCT.NET: frozen: not on a frozen edge.
        if (!E.TShape()!.IsFrozen())
        {
            E.TShape()!.Modified(true);
        }
    }

    /// <summary>The range of the edge's curve on the face; the edge's range if it has none.</summary>
    public static void Range(in TopoDS_Edge E, in TopoDS_Face F, out double First, out double Last)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location L);
        Range(E, S, L, out First, out Last);
    }

    /// <summary>
    /// The UV points at the ends of the edge's curve on the surface; for a plane without one, the projections of the
    /// vertices; zeros otherwise.
    /// </summary>
    public static void UVPoints(in TopoDS_Edge E,
                                Geom_Surface? S,
                                in TopLoc_Location L,
                                out gp_Pnt2d PFirst,
                                out gp_Pnt2d PLast)
    {
        TopLoc_Location l = L.Predivided(E.Location());
        bool Eisreversed = (E.Orientation() == TopAbs_REVERSED);

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(S, l))
            {
                if (cr.IsCurveOnClosedSurface() && Eisreversed)
                {
                    BRep_CurveOnClosedSurface CR = (BRep_CurveOnClosedSurface)cr;
                    CR.UVPoints2(out PFirst, out PLast);
                }
                else
                {
                    BRep_CurveOnSurface CR = (BRep_CurveOnSurface)cr;
                    CR.UVPoints(out PFirst, out PLast);
                }

                return;
            }

            itcr.Next();
        }

        // for planar surface project the vertices
        // modif 21-05-97 : for RectangularTrimmedSurface, project the vertices
        Geom_Plane? GP;
        Geom_RectangularTrimmedSurface? GRTS = S as Geom_RectangularTrimmedSurface;
        if (GRTS is not null)
        {
            GP = GRTS.BasisSurface() as Geom_Plane;
        }
        else
        {
            GP = S as Geom_Plane;
        }
        // end of modification from 21-05-97

        if (GP is not null)
        {
            // get the two vertices
            TopExp.Vertices(E, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);

            TopLoc_Location Linverted = L.Inverted();
            Vf.Move(Linverted, false);
            Vl.Move(Linverted, false);
            double u, v;
            gp_Pln pln = GP.Pln();

            u = v = 0.0;
            if (!Vf.IsNull())
            {
                gp_Pnt PF = BRep_Tool.Pnt(Vf);
                ElSLib.Parameters(pln, PF, out u, out v);
            }

            PFirst = new gp_Pnt2d(u, v);

            u = v = 0.0;
            if (!Vl.IsNull())
            {
                gp_Pnt PL = BRep_Tool.Pnt(Vl);
                ElSLib.Parameters(pln, PL, out u, out v);
            }

            PLast = new gp_Pnt2d(u, v);
        }
        else
        {
            PFirst = new gp_Pnt2d(0.0, 0.0);
            PLast = new gp_Pnt2d(0.0, 0.0);
        }
    }

    /// <summary>The UV points at the ends of the edge's curve on the face.</summary>
    public static void UVPoints(in TopoDS_Edge E, in TopoDS_Face F, out gp_Pnt2d PFirst, out gp_Pnt2d PLast)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location L);
        TopoDS_Edge aLocalEdge = E;
        if (F.Orientation() == TopAbs_REVERSED)
        {
            aLocalEdge.Reverse();
        }

        UVPoints(aLocalEdge, S, L, out PFirst, out PLast);
    }

    /// <summary>Sets the UV points at the ends of the edge's curve on the surface.</summary>
    /// <remarks>Internal: it changes the edge, and throws <see cref="Standard_ImmutableObject"/> on a frozen one.</remarks>
    internal static void SetUVPoints(in TopoDS_Edge E,
                                     Geom_Surface? S,
                                     in TopLoc_Location L,
                                     in gp_Pnt2d PFirst,
                                     in gp_Pnt2d PLast)
    {
        TopLoc_Location l = L.Predivided(E.Location());
        bool Eisreversed = (E.Orientation() == TopAbs_REVERSED);

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(S, l))
            {
                if (cr.IsCurveOnClosedSurface() && Eisreversed)
                {
                    BRep_CurveOnClosedSurface CS = (BRep_CurveOnClosedSurface)cr;
                    CS.SetUVPoints2(PFirst, PLast);
                }
                else
                {
                    BRep_CurveOnSurface CS = (BRep_CurveOnSurface)cr;
                    CS.SetUVPoints(PFirst, PLast);
                }
            }

            itcr.Next();
        }
    }

    /// <summary>Sets the UV points at the ends of the edge's curve on the face.</summary>
    /// <remarks>Internal: it changes the edge, and throws <see cref="Standard_ImmutableObject"/> on a frozen one.</remarks>
    internal static void SetUVPoints(in TopoDS_Edge E, in TopoDS_Face F, in gp_Pnt2d PFirst, in gp_Pnt2d PLast)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location L);
        TopoDS_Edge aLocalEdge = E;
        if (F.Orientation() == TopAbs_REVERSED)
        {
            aLocalEdge.Reverse();
        }

        SetUVPoints(aLocalEdge, S, L, PFirst, PLast);
    }

    /// <summary>True if the edge has a continuity between the surfaces of the two faces.</summary>
    public static bool HasContinuity(in TopoDS_Edge E, in TopoDS_Face F1, in TopoDS_Face F2)
    {
        Geom_Surface? S1 = BRep_Tool.Surface(F1, out TopLoc_Location l1);
        Geom_Surface? S2 = BRep_Tool.Surface(F2, out TopLoc_Location l2);
        return HasContinuity(E, S1, S2, l1, l2);
    }

    /// <summary>The continuity of the edge between the surfaces of the two faces (C0 if none is stored).</summary>
    public static GeomAbs_Shape Continuity(in TopoDS_Edge E, in TopoDS_Face F1, in TopoDS_Face F2)
    {
        Geom_Surface? S1 = BRep_Tool.Surface(F1, out TopLoc_Location l1);
        Geom_Surface? S2 = BRep_Tool.Surface(F2, out TopLoc_Location l2);
        return Continuity(E, S1, S2, l1, l2);
    }

    /// <summary>True if the edge has a continuity between the two surfaces.</summary>
    public static bool HasContinuity(in TopoDS_Edge E,
                                     Geom_Surface? S1,
                                     Geom_Surface? S2,
                                     in TopLoc_Location L1,
                                     in TopLoc_Location L2)
    {
        TopLoc_Location Eloc = E.Location();
        TopLoc_Location l1 = L1.Predivided(Eloc);
        TopLoc_Location l2 = L2.Predivided(Eloc);

        // find the representation
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsRegularity(S1, S2, l1, l2))
            {
                return true;
            }

            itcr.Next();
        }

        return false;
    }

    /// <summary>The continuity of the edge between the two surfaces (C0 if none is stored).</summary>
    public static GeomAbs_Shape Continuity(in TopoDS_Edge E,
                                           Geom_Surface? S1,
                                           Geom_Surface? S2,
                                           in TopLoc_Location L1,
                                           in TopLoc_Location L2)
    {
        TopLoc_Location l1 = L1.Predivided(E.Location());
        TopLoc_Location l2 = L2.Predivided(E.Location());

        // find the representation
        // OCCT reads ChangeCurves(), which the port guards against frozen edges: Curves() is the same list
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)E.TShape()!).Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsRegularity(S1, S2, l1, l2))
            {
                return cr.Continuity();
            }

            itcr.Next();
        }

        return GeomAbs_C0;
    }

    /// <summary>True if the edge has a continuity between some two surfaces.</summary>
    public static bool HasContinuity(in TopoDS_Edge E)
    {
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            BRep_CurveRepresentation CR = itcr.Value();
            if (CR.IsRegularity())
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>The highest continuity stored on the edge (C0 if none).</summary>
    public static GeomAbs_Shape MaxContinuity(in TopoDS_Edge theEdge)
    {
        GeomAbs_Shape aMaxCont = GeomAbs_C0;
        // OCCT reads ChangeCurves(), which the port guards against frozen edges: Curves() is the same list
        for (var aReprIter = new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)theEdge.TShape()!).Curves());
             aReprIter.More();
             aReprIter.Next())
        {
            BRep_CurveRepresentation aRepr = aReprIter.Value();
            if (aRepr.IsRegularity())
            {
                GeomAbs_Shape aCont = aRepr.Continuity();
                if ((int)aCont > (int)aMaxCont)
                {
                    aMaxCont = aCont;
                }
            }
        }

        return aMaxCont;
    }

    /// <summary>The 3D point of the vertex, placed.</summary>
    /// <exception cref="Standard_NullObject">The vertex is null.</exception>
    public static gp_Pnt Pnt(in TopoDS_Vertex V)
    {
        BRep_TVertex? TV = (BRep_TVertex?)V.TShape();

        if (TV is null)
        {
            throw new Standard_NullObject("BRep_Tool:: TopoDS_Vertex hasn't gp_Pnt");
        }

        gp_Pnt P = TV.Pnt();
        if (V.Location().IsIdentity())
        {
            return P;
        }

        return P.Transformed(V.Location().Transformation());
    }

    /// <summary>The tolerance of the vertex, at least Precision::Confusion().</summary>
    /// <exception cref="Standard_NullObject">The vertex is null.</exception>
    public static double Tolerance(in TopoDS_Vertex V)
    {
        BRep_TVertex? aTVert = (BRep_TVertex?)V.TShape();

        if (aTVert is null)
        {
            throw new Standard_NullObject("BRep_Tool:: TopoDS_Vertex hasn't gp_Pnt");
        }

        double p = aTVert.Tolerance();
        double pMin = Precision.Confusion();
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
    /// <paramref name="theParam"/> to 0 (OCCT leaves it unchanged).
    /// </summary>
    public static bool Parameter(in TopoDS_Vertex theV, in TopoDS_Edge theE, out double theParam)
    {
        theParam = 0.0;
        // Search the vertex in the edge

        bool rev = false;
        TopoDS_Shape VF = new TopoDS_Shape();
        TopAbs_Orientation orient = TopAbs_INTERNAL;

        var itv = new TopoDS_Iterator(theE.Oriented(TopAbs_FORWARD));

        // if the edge has no vertices
        // and is degenerated use the vertex orientation
        // RLE, june 94

        if (!itv.More() && BRep_Tool.Degenerated(theE))
        {
            orient = theV.Orientation();
        }

        while (itv.More())
        {
            TopoDS_Shape Vcur = itv.Value();
            if (theV.IsSame(Vcur))
            {
                if (VF.IsNull())
                {
                    VF = Vcur;
                }
                else
                {
                    rev = theE.Orientation() == TopAbs_REVERSED;
                    if (Vcur.Orientation() == theV.Orientation())
                    {
                        VF = Vcur;
                    }
                }
            }

            itv.Next();
        }

        if (!VF.IsNull())
        {
            orient = VF.Orientation();
        }

        double f, l;

        if (orient == TopAbs_FORWARD)
        {
            BRep_Tool.Range(theE, out f, out l);
            theParam = (rev) ? l : f;
            return true;
        }

        else if (orient == TopAbs_REVERSED)
        {
            BRep_Tool.Range(theE, out f, out l);
            theParam = (rev) ? f : l;
            return true;
        }

        else
        {
            Geom_Curve? C = BRep_Tool.Curve(theE, out TopLoc_Location L, out f, out l);
            L = L.Predivided(theV.Location());
            if (C is not null || BRep_Tool.Degenerated(theE))
            {
                BRep_TVertex TV = (BRep_TVertex)theV.TShape()!;
                var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(TV.Points());

                while (itpr.More())
                {
                    BRep_PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnCurve(C, L))
                    {
                        double p = pr.Parameter();
                        double res = p; // SVV 4 nov 99 - to avoid warnings on Linux
                        if (C is not null)
                        {
                            // Closed curves RLE 16 june 94
                            if (Precision.IsNegativeInfinite(f))
                            {
                                theParam = pr.Parameter(); // p;
                                return true;
                            }

                            if (Precision.IsPositiveInfinite(l))
                            {
                                theParam = pr.Parameter(); // p;
                                return true;
                            }

                            gp_Pnt Pf = C.Value(f).Transformed(L.Transformation());
                            gp_Pnt Pl = C.Value(l).Transformed(L.Transformation());
                            double tol = BRep_Tool.Tolerance(theV);
                            if (Pf.Distance(Pl) < tol)
                            {
                                if (Pf.Distance(BRep_Tool.Pnt(theV)) < tol)
                                {
                                    if (theV.Orientation() == TopAbs_FORWARD)
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

                        theParam = res; // p;
                        return true;
                    }

                    itpr.Next();
                }
            }
            else
            {
                // no 3d curve !!
                // let us try with the first pcurve
                BRep_Tool.CurveOnSurface(theE, out Geom2d_Curve? PC, out Geom_Surface? S, out L, out f, out l);
                L = L.Predivided(theV.Location());
                BRep_TVertex TV = (BRep_TVertex)theV.TShape()!;
                var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(TV.Points());

                while (itpr.More())
                {
                    BRep_PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnCurveOnSurface(PC, S, L))
                    {
                        double p = pr.Parameter();
                        // Closed curves RLE 16 june 94
                        if (PC!.IsClosed())
                        {
                            if ((p == PC.FirstParameter()) || (p == PC.LastParameter()))
                            {
                                if (theV.Orientation() == TopAbs_FORWARD)
                                {
                                    p = PC.FirstParameter();
                                }
                                else
                                {
                                    p = PC.LastParameter();
                                }
                            }
                        }

                        theParam = p;
                        return true;
                    }

                    itpr.Next();
                }
            }
        }

        return false;
    }

    /// <summary>The parameter of the vertex on the edge.</summary>
    /// <exception cref="Standard_NoSuchObject">The vertex has no parameter on the edge.</exception>
    public static double Parameter(in TopoDS_Vertex V, in TopoDS_Edge E)
    {
        if (Parameter(V, E, out double p))
        {
            return p;
        }

        throw new Standard_NoSuchObject("BRep_Tool:: no parameter on edge");
    }

    /// <summary>The parameter of the vertex on the edge's curve on the face.</summary>
    /// <exception cref="Standard_NoSuchObject">The vertex has no parameter on the edge.</exception>
    public static double Parameter(in TopoDS_Vertex V, in TopoDS_Edge E, in TopoDS_Face F)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location L);
        return BRep_Tool.Parameter(V, E, S, L);
    }

    /// <summary>The parameter of the vertex on the edge's curve on the surface.</summary>
    /// <exception cref="Standard_NoSuchObject">The vertex has no parameter on the edge.</exception>
    public static double Parameter(in TopoDS_Vertex V, in TopoDS_Edge E, Geom_Surface? S, in TopLoc_Location L)
    {
        // Search the vertex in the edge

        bool rev = false;
        TopoDS_Shape VF = new TopoDS_Shape();
        var itv = new TopoDS_Iterator(E.Oriented(TopAbs_FORWARD));

        while (itv.More())
        {
            if (V.IsSame(itv.Value()))
            {
                if (VF.IsNull())
                {
                    VF = itv.Value();
                }
                else
                {
                    rev = E.Orientation() == TopAbs_REVERSED;
                    if (itv.Value().Orientation() == V.Orientation())
                    {
                        VF = itv.Value();
                    }
                }
            }

            itv.Next();
        }

        TopAbs_Orientation orient = TopAbs_INTERNAL;
        if (!VF.IsNull())
        {
            orient = VF.Orientation();
        }

        double f, l;

        if (orient == TopAbs_FORWARD)
        {
            BRep_Tool.Range(E, S, L, out f, out l);
            return (rev) ? l : f;
        }

        else if (orient == TopAbs_REVERSED)
        {
            BRep_Tool.Range(E, S, L, out f, out l);
            return (rev) ? f : l;
        }

        else
        {
            Geom2d_Curve? PC = BRep_Tool.CurveOnSurface(E, S, L, out f, out l);
            BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
            var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(TV.Points());

            while (itpr.More())
            {
                if (itpr.Value().IsPointOnCurveOnSurface(PC, S, L))
                {
                    return itpr.Value().Parameter();
                }

                itpr.Next();
            }
        }

        //----------------------------------------------------------

        Geom_Curve? C = BRep_Tool.Curve(E, out TopLoc_Location L1, out f, out l);
        L1 = L1.Predivided(V.Location());
        if (C is not null || Degenerated(E))
        {
            BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
            var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(TV.Points());

            while (itpr.More())
            {
                BRep_PointRepresentation pr = itpr.Value();
                if (pr.IsPointOnCurve(C, L1))
                {
                    double p = pr.Parameter();
                    double res = p;
                    if (C is not null)
                    {
                        // Closed curves RLE 16 june 94
                        if (Precision.IsNegativeInfinite(f))
                        {
                            return res;
                        }

                        if (Precision.IsPositiveInfinite(l))
                        {
                            return res;
                        }

                        gp_Pnt Pf = C.Value(f).Transformed(L1.Transformation());
                        gp_Pnt Pl = C.Value(l).Transformed(L1.Transformation());
                        double tol = BRep_Tool.Tolerance(V);
                        if (Pf.Distance(Pl) < tol)
                        {
                            if (Pf.Distance(BRep_Tool.Pnt(V)) < tol)
                            {
                                if (V.Orientation() == TopAbs_FORWARD)
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

        throw new Standard_NoSuchObject("BRep_Tool:: no parameter on edge");
    }

    /// <summary>The parameters of the vertex on the face.</summary>
    /// <exception cref="Standard_NoSuchObject">The vertex has no parameters on the face.</exception>
    public static gp_Pnt2d Parameters(in TopoDS_Vertex V, in TopoDS_Face F)
    {
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location L);
        L = L.Predivided(V.Location());
        BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
        var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(TV.Points());

        // It is checked if there is PointRepresentation (case non Manifold)
        while (itpr.More())
        {
            if (itpr.Value().IsPointOnSurface(S, L))
            {
                return new gp_Pnt2d(itpr.Value().Parameter(), itpr.Value().Parameter2());
            }

            itpr.Next();
        }

        TopoDS_Vertex Vf, Vl;
        TopoDS_Edge E;
        // Otherwise the edges are searched (PMN 4/06/97) It is not possible to succeed 999/1000!
        // even if often there is a way to make more economically than above...
        var exp = new TopExp_Explorer();
        for (exp.Init(F, TopAbs_EDGE); exp.More(); exp.Next())
        {
            E = TopoDS.Edge(exp.Current());
            TopExp.Vertices(E, out Vf, out Vl);
            if ((V.IsSame(Vf)) || (V.IsSame(Vl)))
            {
                UVPoints(E, F, out gp_Pnt2d Pf, out gp_Pnt2d Pl);
                if (V.IsSame(Vf))
                {
                    return Pf;
                }
                else
                {
                    return Pl; // Ambiguity (natural) for degenerated edges.
                }
            }
        }

        throw new Standard_NoSuchObject("BRep_Tool:: no parameters on surface");
    }

    /// <summary>
    /// True for a shell whose edges are each shared by two of its faces, a wire whose vertices are each shared by two
    /// of its edges, or an edge whose two vertices are the same; for other shapes, the Closed flag.
    /// </summary>
    public static bool IsClosed(in TopoDS_Shape theShape)
    {
        if (theShape.ShapeType() == TopAbs_SHELL)
        {
            var aMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var exp = new TopExp_Explorer(theShape.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
            bool hasBound = false;
            for (; exp.More(); exp.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(exp.Current());
                if (BRep_Tool.Degenerated(E) || E.Orientation() == TopAbs_INTERNAL || E.Orientation() == TopAbs_EXTERNAL)
                {
                    continue;
                }

                hasBound = true;
                if (!aMap.Add(E))
                {
                    aMap.Remove(E);
                }
            }

            return hasBound && aMap.IsEmpty();
        }
        else if (theShape.ShapeType() == TopAbs_WIRE)
        {
            var aMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var exp = new TopExp_Explorer(theShape.Oriented(TopAbs_FORWARD), TopAbs_VERTEX);
            bool hasBound = false;
            for (; exp.More(); exp.Next())
            {
                TopoDS_Shape V = exp.Current();
                if (V.Orientation() == TopAbs_INTERNAL || V.Orientation() == TopAbs_EXTERNAL)
                {
                    continue;
                }

                hasBound = true;
                if (!aMap.Add(V))
                {
                    aMap.Remove(V);
                }
            }

            return hasBound && aMap.IsEmpty();
        }
        else if (theShape.ShapeType() == TopAbs_EDGE)
        {
            TopExp.Vertices(TopoDS.Edge(theShape), out TopoDS_Vertex aVFirst, out TopoDS_Vertex aVLast);
            return !aVFirst.IsNull() && aVFirst.IsSame(aVLast);
        }

        return theShape.Closed();
    }

    // modified by NIZNHY-PKV Fri Oct 17 14:09:58 2008 f
    private static bool IsPlane(Geom_Surface? aS)
    {
        bool bRet;
        Geom_Plane? aGP;
        Geom_RectangularTrimmedSurface? aGRTS = aS as Geom_RectangularTrimmedSurface;
        Geom_OffsetSurface? aGOFS = aS as Geom_OffsetSurface;

        if (aGOFS is not null)
        {
            aGP = aGOFS.BasisSurface() as Geom_Plane;
        }
        else if (aGRTS is not null)
        {
            aGP = aGRTS.BasisSurface() as Geom_Plane;
        }
        else
        {
            aGP = aS as Geom_Plane;
        }

        bRet = aGP is not null;

        return bRet;
    }

    /// <summary>The largest tolerance of the sub-shapes of the given type (FACE, EDGE or VERTEX); 0 otherwise.</summary>
    public static double MaxTolerance(in TopoDS_Shape theShape, TopAbs_ShapeEnum theSubShape)
    {
        double aTol = 0.0;

        // Explorer Shape-Subshape.
        var anExpSS = new TopExp_Explorer(theShape, theSubShape);
        if (theSubShape == TopAbs_FACE)
        {
            for (; anExpSS.More(); anExpSS.Next())
            {
                TopoDS_Shape aCurrentSubShape = anExpSS.Current();
                aTol = Max(aTol, Tolerance(TopoDS.Face(aCurrentSubShape)));
            }
        }
        else if (theSubShape == TopAbs_EDGE)
        {
            for (; anExpSS.More(); anExpSS.Next())
            {
                TopoDS_Shape aCurrentSubShape = anExpSS.Current();
                aTol = Max(aTol, Tolerance(TopoDS.Edge(aCurrentSubShape)));
            }
        }
        else if (theSubShape == TopAbs_VERTEX)
        {
            for (; anExpSS.More(); anExpSS.Next())
            {
                TopoDS_Shape aCurrentSubShape = anExpSS.Current();
                aTol = Max(aTol, Tolerance(TopoDS.Vertex(aCurrentSubShape)));
            }
        }

        return aTol;
    }
}
