// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Edge.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepExtrema_SupportType;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Tool for analysing edges: queries on the 3D curve and pcurves (taking the edge's orientation into account), and
/// checks of the vertices against the curves, of SameParameter, of overlapping. Each check sets a status, read with
/// <see cref="Status"/>.
/// </summary>
/// <remarks>A working object (decision 21): the checks change its status.</remarks>
public sealed class ShapeAnalysis_Edge
{
    private int myStatus;

    /// <summary>An analyser with an empty status.</summary>
    public ShapeAnalysis_Edge()
    {
        myStatus = 0; // ShapeExtend::EncodeStatus (ShapeExtend_OK);
    }

    /// <summary>Tells if the edge has a 3D curve.</summary>
    public bool HasCurve3d(in TopoDS_Edge edge)
    {
        Geom_Curve? c3d = BRep_Tool.Curve(edge, out _, out _);
        return c3d is not null;
    }

    /// <summary>
    /// The 3D curve of the edge, transformed by its location, with its range; if <paramref name="orient"/> and the
    /// edge is reversed, the range is swapped. Returns false if there is no 3D curve.
    /// </summary>
    public bool Curve3d(in TopoDS_Edge edge, out Geom_Curve? C3d, out double cf, out double cl, bool orient = true)
    {
        C3d = BRep_Tool.Curve(edge, out TopLoc_Location L, out cf, out cl);
        if (C3d is not null && !L.IsIdentity())
        {
            C3d = (Geom_Curve)C3d.Transformed(L.Transformation());
            cf = C3d.TransformedParameter(cf, L.Transformation());
            cl = C3d.TransformedParameter(cl, L.Transformation());
        }

        if (orient)
        {
            if (edge.Orientation() == TopAbs_REVERSED)
            {
                double tmp = cf;
                cf = cl;
                cl = tmp;
            }
        }

        return C3d is not null;
    }

    /// <summary>Tells if the edge is closed in 3D: its curve is closed and its two vertices are the same.</summary>
    public bool IsClosed3d(in TopoDS_Edge edge)
    {
        Geom_Curve? c3d = BRep_Tool.Curve(edge, out _, out _);
        if (c3d is null)
        {
            return false;
        }

        if (!c3d.IsClosed())
        {
            return false;
        }

        return FirstVertex(edge).IsSame(LastVertex(edge));
    }

    /// <summary>Tells if the edge has a pcurve on the face.</summary>
    public bool HasPCurve(in TopoDS_Edge edge, in TopoDS_Face face)
    {
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return HasPCurve(edge, S, L);
    }

    /// <summary>Tells if the edge has a pcurve on the surface at the location.</summary>
    public bool HasPCurve(in TopoDS_Edge edge, Geom_Surface? surface, in TopLoc_Location location)
    {
        Geom2d_Curve? c2d = BRep_Tool.CurveOnSurface(edge, surface, location, out _, out _);
        return c2d is not null;
    }

    /// <summary>
    /// The pcurve of the edge on the face, with its range; if <paramref name="orient"/> and the edge is reversed, the
    /// range is swapped. Returns false if there is none.
    /// </summary>
    public bool PCurve(in TopoDS_Edge edge, in TopoDS_Face face, out Geom2d_Curve? C2d, out double cf, out double cl, bool orient = true)
    {
        //: abv 20.05.02: take into account face orientation
        // COMMENTED BACK - NEEDS MORE CHANGES IN ALL SHAPEHEALING
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return PCurve(edge, S, L, out C2d, out cf, out cl, orient);
    }

    /// <summary>The pcurve of the edge on the surface at the location, with its range (swapped as above).</summary>
    public bool PCurve(in TopoDS_Edge edge,
                       Geom_Surface? surface,
                       in TopLoc_Location location,
                       out Geom2d_Curve? C2d,
                       out double cf,
                       out double cl,
                       bool orient = true)
    {
        C2d = BRep_Tool.CurveOnSurface(edge, surface, location, out cf, out cl);
        if (orient && edge.Orientation() == TopAbs_REVERSED)
        {
            double tmp = cf;
            cf = cl;
            cl = tmp;
        }

        return C2d is not null;
    }

    /// <summary>The UV points of the ends of the edge's pcurve on the face; false if there is no pcurve.</summary>
    /// <remarks>The outputs are the origin when there is no pcurve (OCCT leaves them unchanged).</remarks>
    public bool BoundUV(in TopoDS_Edge edge, in TopoDS_Face face, out gp_Pnt2d first, out gp_Pnt2d last)
    {
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return BoundUV(edge, S, L, out first, out last);
    }

    /// <summary>The UV points of the ends of the edge's pcurve on the surface; false if there is no pcurve.</summary>
    public bool BoundUV(in TopoDS_Edge edge, Geom_Surface? surface, in TopLoc_Location location, out gp_Pnt2d first, out gp_Pnt2d last)
    {
        if (!PCurve(edge, surface, location, out Geom2d_Curve? c2d, out double uf, out double ul))
        {
            first = new gp_Pnt2d();
            last = new gp_Pnt2d();
            return false;
        }

        first = c2d!.Value(uf);
        last = c2d.Value(ul);
        return true;
    }

    /// <summary>Tells if the edge is a seam of the face (two pcurves on it).</summary>
    public bool IsSeam(in TopoDS_Edge edge, in TopoDS_Face face)
    {
        return BRep_Tool.IsClosed(edge, face);
    }

    /// <summary>Tells if the edge is a seam of the surface at the location.</summary>
    public bool IsSeam(in TopoDS_Edge edge, Geom_Surface? surface, in TopLoc_Location location)
    {
        return BRep_Tool.IsClosed(edge, surface, location);
    }

    /// <summary>The first vertex of the edge, taking its orientation into account (reversed for a reversed edge).</summary>
    public TopoDS_Vertex FirstVertex(in TopoDS_Edge edge)
    {
        TopoDS_Vertex V;
        if (edge.Orientation() == TopAbs_REVERSED)
        {
            V = TopExp.LastVertex(edge);
            V.Reverse();
        }
        else
        {
            V = TopExp.FirstVertex(edge);
        }

        return V;
    }

    /// <summary>The last vertex of the edge, taking its orientation into account (reversed for a reversed edge).</summary>
    public TopoDS_Vertex LastVertex(in TopoDS_Edge edge)
    {
        TopoDS_Vertex V;
        if (edge.Orientation() == TopAbs_REVERSED)
        {
            V = TopExp.FirstVertex(edge);
            V.Reverse();
        }
        else
        {
            V = TopExp.LastVertex(edge);
        }

        return V;
    }

    /// <summary>
    /// The end point of the edge's pcurve on the face (its start, or its end if <paramref name="atEnd"/>) and the
    /// tangent there, in the edge's direction. With <paramref name="dparam"/> &gt; 0, the tangent is the chord to the
    /// point at that fraction of the range. False if there is no pcurve or the tangent is null.
    /// </summary>
    public bool GetEndTangent2d(in TopoDS_Edge edge, in TopoDS_Face face, bool atEnd, out gp_Pnt2d pos, out gp_Vec2d tang, double dparam = 0.0)
    {
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return GetEndTangent2d(edge, S, L, atEnd, out pos, out tang, dparam);
    }

    /// <summary>As the face form, on the surface at the location.</summary>
    /// <remarks><paramref name="pos"/> is the origin when there is no pcurve (OCCT leaves it unchanged).</remarks>
    public bool GetEndTangent2d(in TopoDS_Edge edge,
                                Geom_Surface? S,
                                in TopLoc_Location L,
                                bool atEnd,
                                out gp_Pnt2d pos,
                                out gp_Vec2d tang,
                                double dparam = 0.0)
    {
        pos = new gp_Pnt2d();
        if (!PCurve(edge, S, L, out Geom2d_Curve? c2d, out double cf, out double cl))
        {
            tang = new gp_Vec2d(0, 0);
            return false;
        }

        tang = new gp_Vec2d();
        double dpnew = dparam;
        if (dpnew > Precision.Confusion())
        {
            double par1, par2, delta = (cl - cf) * dpnew;
            if (Math.Abs(delta) < Precision.PConfusion())
            {
                dpnew = 0.0;
            }
            else
            {
                gp_Pnt2d ptmp = new gp_Pnt2d();
                if (atEnd)
                {
                    par1 = cl;
                    par2 = cl - delta;
                    c2d!.D0(par1, out pos);
                    c2d.D0(par2, out ptmp);
                    tang = new gp_Vec2d(pos.XY() - ptmp.XY());
                }
                else
                {
                    par1 = cf;
                    par2 = cf + delta;
                    c2d!.D0(par1, out pos);
                    c2d.D0(par2, out ptmp);
                    tang = new gp_Vec2d(ptmp.XY() - pos.XY());
                }

                if (tang.SquareMagnitude() < Precision.PConfusion() * Precision.PConfusion())
                {
                    dpnew = 0.0;
                }
            }
        }

        if (dpnew <= Precision.Confusion())
        {
            // get non-null tangency searching until 3rd derivative, or as straight btw ends
            double par = (atEnd ? cl : cf);
            c2d!.D1(par, out pos, out tang);
            if (tang.SquareMagnitude() < Precision.PConfusion() * Precision.PConfusion())
            {
                c2d.D2(par, out pos, out gp_Vec2d d1, out tang);
                if (tang.SquareMagnitude() < Precision.PConfusion() * Precision.PConfusion())
                {
                    c2d.D3(par, out pos, out d1, out gp_Vec2d d2, out tang);
                    if (tang.SquareMagnitude() < Precision.PConfusion() * Precision.PConfusion())
                    {
                        c2d.D0((atEnd ? cf : cl), out gp_Pnt2d p2);
                        tang = new gp_Vec2d(p2.XY() - pos.XY());
                        if (tang.SquareMagnitude() < Precision.PConfusion() * Precision.PConfusion())
                        {
                            return false;
                        }
                    }
                }
            }

            if (edge.Orientation() == TopAbs_REVERSED)
            {
                tang.Reverse();
            }
        }

        return true;
    }

    /// <summary>
    /// Checks the ends of the 3D curve against the vertices (within <paramref name="preci"/>, or each vertex's
    /// tolerance if negative); <paramref name="vtx"/> 1 or 2 checks only that vertex. True if a vertex is off:
    /// DONE1 for the first, DONE2 for the last; FAIL1 if there is no 3D curve.
    /// </summary>
    public bool CheckVerticesWithCurve3d(in TopoDS_Edge edge, double preci = -1, int vtx = 0)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        TopoDS_Vertex V1 = FirstVertex(edge);
        TopoDS_Vertex V2 = LastVertex(edge);
        gp_Pnt p1v = BRep_Tool.Pnt(V1);
        gp_Pnt p2v = BRep_Tool.Pnt(V2);

        if (!Curve3d(edge, out Geom_Curve? c3d, out double cf, out double cl))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        //  on va faire les checks ...
        if (vtx != 2)
        {
            //  1er VTX
            gp_Pnt p13d = c3d!.Value(cf);
            if (p1v.Distance(p13d) > (preci < 0 ? BRep_Tool.Tolerance(V1) : preci))
            {
                myStatus |= (int)ShapeExtend_DONE1; // OCCT ors the enumerator, which equals its encoding for DONE1 and DONE2
            }
        }

        if (vtx != 1)
        {
            //  2me VTX
            gp_Pnt p23d = c3d!.Value(cl);
            if (p2v.Distance(p23d) > (preci < 0 ? BRep_Tool.Tolerance(V2) : preci))
            {
                myStatus |= (int)ShapeExtend_DONE2;
            }
        }

        return Status(ShapeExtend_DONE);
    }

    /// <summary>
    /// Checks the ends of the pcurve on the face against the vertices, as <see cref="CheckVerticesWithCurve3d"/>;
    /// FAIL1 if there is no pcurve.
    /// </summary>
    public bool CheckVerticesWithPCurve(in TopoDS_Edge edge, in TopoDS_Face face, double preci = -1, int vtx = 0)
    {
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return CheckVerticesWithPCurve(edge, S, L, preci, vtx); // szv#4:S4163:12Mar99 `vtx,preci` wrong parameters order
    }

    /// <summary>As the face form, on the surface at the location.</summary>
    public bool CheckVerticesWithPCurve(in TopoDS_Edge edge, Geom_Surface? surf, in TopLoc_Location loc, double preci = -1, int vtx = 0)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        TopoDS_Vertex V1 = FirstVertex(edge);
        TopoDS_Vertex V2 = LastVertex(edge);
        gp_Pnt p1v = BRep_Tool.Pnt(V1);
        gp_Pnt p2v = BRep_Tool.Pnt(V2);

        if (!PCurve(edge, surf, loc, out Geom2d_Curve? c2d, out double cf, out double cl))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        // on va faire les checks ...
        if (vtx != 2)
        { //  1er VTX
            gp_Pnt2d p1uv = c2d!.Value(cf);
            gp_Pnt p12d = surf!.Value(p1uv.X(), p1uv.Y());
            if (!loc.IsIdentity())
            {
                p12d.Transform(loc.Transformation());
            }

            if (p1v.Distance(p12d) > (preci < 0 ? BRep_Tool.Tolerance(V1) : preci))
            {
                myStatus |= (int)ShapeExtend_DONE1;
            }
        }

        if (vtx != 1)
        { //  2me VTX
            gp_Pnt2d p2uv = c2d!.Value(cl);
            gp_Pnt p22d = surf!.Value(p2uv.X(), p2uv.Y());
            if (!loc.IsIdentity())
            {
                p22d.Transform(loc.Transformation());
            }

            if (p2v.Distance(p22d) > (preci < 0 ? BRep_Tool.Tolerance(V2) : preci))
            {
                myStatus |= (int)ShapeExtend_DONE2;
            }
        }

        return Status(ShapeExtend_DONE);
    }

    // The file-static CheckVertexTolerance of ShapeAnalysis_Edge.cxx.
    private static int checkVertexTolerance(in TopoDS_Edge edge, in TopoDS_Face face, bool checkAll, out double toler1, out double toler2)
    {
        int Status = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        var sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex V1 = sae.FirstVertex(edge);
        TopoDS_Vertex V2 = sae.LastVertex(edge);
        if (V1.IsNull() || V2.IsNull())
        { //: p1 abv 22 Feb 99: r76sy.stp
            Status |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            // OCCT leaves the tolerances unchanged here; the port's outputs are 0
            toler1 = toler2 = 0.0;
            return Status;
        }

        double old1 = BRep_Tool.Tolerance(V1);
        double old2 = BRep_Tool.Tolerance(V2);
        gp_Pnt pnt1 = BRep_Tool.Pnt(V1);
        gp_Pnt pnt2 = BRep_Tool.Pnt(V2);

        double a, b;
        if (!sae.Curve3d(edge, out Geom_Curve? c3d, out a, out b, true))
        {
            if (!BRep_Tool.Degenerated(edge))
            {
                Status |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }

            toler1 = toler2 = 0.0;
        }
        else
        {
            toler1 = pnt1.SquareDistance(c3d!.Value(a));
            toler2 = pnt2.SquareDistance(c3d.Value(b));
        }

        if (checkAll)
        {
            BRep_TEdge TE = (BRep_TEdge)edge.TShape()!;
            for (var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves()); itcr.More(); itcr.Next())
            {
                if (itcr.Value() is not BRep_GCurve GC || !GC.IsCurveOnSurface())
                {
                    continue;
                }

                Geom_Surface S = GC.Surface()!;
                TopLoc_Location L = edge.Location() * GC.Location();
                sae.PCurve(edge, S, L, out Geom2d_Curve? pcurve, out a, out b, true);
                gp_Pnt2d p1 = pcurve!.Value(a);
                gp_Pnt2d p2 = pcurve.Value(b);
                gp_Pnt P1 = S.Value(p1.X(), p1.Y()).Transformed(L.Transformation());
                gp_Pnt P2 = S.Value(p2.X(), p2.Y()).Transformed(L.Transformation());
                toler1 = Max(toler1, pnt1.SquareDistance(P1));
                toler2 = Max(toler2, pnt2.SquareDistance(P2));
            }
        }
        //: abv 10.06.02: porting C40 -> dev (CC670-12608.stp)
        // Check with given face is needed for plane surfaces (if no stored pcurves)
        else if (!face.IsNull())
        {
            Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
            if (sae.PCurve(edge, S, L, out Geom2d_Curve? pcurve, out a, out b, true))
            {
                gp_Pnt2d p1 = pcurve!.Value(a);
                gp_Pnt2d p2 = pcurve.Value(b);
                gp_Pnt P1 = S!.Value(p1.X(), p1.Y()).Transformed(L.Transformation());
                gp_Pnt P2 = S.Value(p2.X(), p2.Y()).Transformed(L.Transformation());
                toler1 = Max(toler1, pnt1.SquareDistance(P1));
                toler2 = Max(toler2, pnt2.SquareDistance(P2));
            }
            else
            {
                Status |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            }
        }

        //: o8 abv 19 Feb 99: CTS18541.stp #18559: coeff 1.0001 added
        // szv 18 Aug 99: edge tolerance is taken in consideration
        double tole = BRep_Tool.Tolerance(edge);
        toler1 = Max(1.0000001 * Math.Sqrt(toler1), tole);
        toler2 = Max(1.0000001 * Math.Sqrt(toler2), tole);
        if (toler1 > old1)
        {
            Status |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        if (toler2 > old2)
        {
            Status |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }

        return Status;
    }

    /// <summary>
    /// Computes the tolerances the vertices need to cover the ends of the 3D curve and of the pcurve on the face (at
    /// least the edge's tolerance). True if one exceeds the vertex's present tolerance: DONE1 for the first, DONE2
    /// for the last. FAIL1 if a vertex is missing, FAIL2 if there is no 3D curve, FAIL3 if no pcurve on the face.
    /// </summary>
    public bool CheckVertexTolerance(in TopoDS_Edge edge, in TopoDS_Face face, out double toler1, out double toler2)
    {
        myStatus = checkVertexTolerance(edge, face, false, out toler1, out toler2);
        return Status(ShapeExtend_DONE);
    }

    /// <summary>As the face form, against every pcurve of the edge.</summary>
    public bool CheckVertexTolerance(in TopoDS_Edge edge, out double toler1, out double toler2)
    {
        TopoDS_Face F = new TopoDS_Face();
        myStatus = checkVertexTolerance(edge, F, true, out toler1, out toler2);
        return Status(ShapeExtend_DONE);
    }

    /// <summary>
    /// Checks that the ends of the 3D curve and of the pcurve on the face match within the vertices' tolerances. True
    /// if they don't; DONE1 if they would match reversed. False for a plane, FAIL1 without a pcurve, FAIL2 without a
    /// 3D curve.
    /// </summary>
    public bool CheckCurve3dWithPCurve(in TopoDS_Edge edge, in TopoDS_Face face)
    {
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return CheckCurve3dWithPCurve(edge, S!, L);
    }

    /// <summary>As the face form, on the surface at the location.</summary>
    public bool CheckCurve3dWithPCurve(in TopoDS_Edge edge, Geom_Surface surface, in TopLoc_Location location)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        if (surface is Geom_Plane)
        {
            return false;
        }

        if (!PCurve(edge, surface, location, out Geom2d_Curve? c2d, out double f2d, out double l2d, false))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        if (!Curve3d(edge, out Geom_Curve? c3d, out double f3d, out double l3d, false))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            return false;
        }

        TopoDS_Vertex aFirstVert = FirstVertex(edge);
        TopoDS_Vertex aLastVert = LastVertex(edge);

        if (aFirstVert.IsNull() || aLastVert.IsNull())
        {
            return false;
        }

        double preci1 = BRep_Tool.Tolerance(aFirstVert), preci2 = BRep_Tool.Tolerance(aLastVert);

        gp_Pnt2d p2d1 = c2d!.Value(f2d), p2d2 = c2d.Value(l2d);

        // #39 rln 17.11.98 S4054, annie_surf.igs entity 39
        return CheckPoints(c3d!.Value(f3d),
                           c3d.Value(l3d),
                           surface.Value(p2d1.X(), p2d1.Y()).Transformed(location.Transformation()),
                           surface.Value(p2d2.X(), p2d2.Y()).Transformed(location.Transformation()),
                           preci1,
                           preci2);
    }

    private bool CheckPoints(in gp_Pnt P1A, in gp_Pnt P1B, in gp_Pnt P2A, in gp_Pnt P2B, double preci1, double preci2)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (P1A.SquareDistance(P2A) <= preci1 * preci1 && P1B.SquareDistance(P2B) <= preci2 * preci2)
        {
            return false;
        }
        else if (P1A.Distance(P2B) + (P1B.Distance(P2A)) < P1A.Distance(P2A) + (P1B.Distance(P2B)))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        return true;
    }

    /// <summary>Tells if the status of the last check holds <paramref name="Status"/>.</summary>
    public bool Status(ShapeExtend_Status Status)
    {
        return ShapeExtend.DecodeStatus(myStatus, Status);
    }

    /// <summary>
    /// Computes the largest deviation of the edge's pcurves from its 3D curve (<paramref name="maxdev"/>), as
    /// BRepLib_ValidateEdge does at <paramref name="NbControl"/> points. True if it exceeds the edge's tolerance
    /// (DONE1) or the edge isn't SameParameter (DONE2). False for a degenerated edge; FAIL1 without a 3D curve, FAIL2
    /// if a deviation couldn't be computed.
    /// </summary>
    /// <remarks><paramref name="maxdev"/> is unchanged for a degenerated edge, as in OCCT.</remarks>
    public bool CheckSameParameter(in TopoDS_Edge edge, ref double maxdev, int NbControl = 23)
    {
        TopoDS_Face anEmptyFace = new TopoDS_Face();
        return CheckSameParameter(edge, anEmptyFace, ref maxdev, NbControl);
    }

    /// <summary>As above, for the pcurve on the face only (or the projection on a plane without a stored pcurve).</summary>
    public bool CheckSameParameter(in TopoDS_Edge theEdge, in TopoDS_Face theFace, ref double theMaxdev, int theNbControl = 23)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (BRep_Tool.Degenerated(theEdge))
        {
            return false;
        }

        theMaxdev = 0;

        // Get same parameter flag
        BRep_TEdge TE = (BRep_TEdge)theEdge.TShape()!;
        bool SameParameter = TE.SameParameter();

        // Get 3D curve of the edge
        Geom_Curve? aC3D = BRep_Tool.Curve(theEdge, out TopLoc_Location aCurveLoc, out double aFirst, out double aLast);
        if (aC3D is null)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        if (!aCurveLoc.IsIdentity())
        {
            gp_Trsf aTrsf = aCurveLoc.Transformation();
            aC3D = (Geom_Curve)aC3D.Transformed(aTrsf);
            aFirst = aC3D.TransformedParameter(aFirst, aTrsf);
            aLast = aC3D.TransformedParameter(aLast, aTrsf);
        }

        // Create adaptor for the curve
        var aGAC = new GeomAdaptor_Curve(aC3D, aFirst, aLast);

        Geom_Surface? aFaceSurf = null;
        TopLoc_Location aFaceLoc = new TopLoc_Location();
        if (!theFace.IsNull())
        {
            aFaceSurf = BRep_Tool.Surface(theFace, out aFaceLoc);
        }

        bool IsPCurveFound = false;
        int i = 1;

        // Iterate on all curve representations
        for (;;)
        {
            Geom2d_Curve? aPC = null;
            Geom_Surface? aS = null;
            TopLoc_Location aLoc = new TopLoc_Location();
            double f = 0, l = 0;
            BRep_Tool.CurveOnSurface(theEdge, ref aPC, ref aS, ref aLoc, ref f, ref l, i);

            if (aPC is null)
            {
                // No more curves
                break;
            }

            ++i;

            // If the input face is not null, check that the curve is on its surface
            if (aFaceSurf is not null)
            {
                if (aFaceSurf != aS || aFaceLoc != aLoc)
                {
                    continue;
                }
            }

            IsPCurveFound = true;

            // Apply transformations for the surface
            Geom_Surface aST = (Geom_Surface)aS!.Transformed(aLoc.Transformation());

            // Compute deviation between curves
            var GHPC = new Geom2dAdaptor_Curve(aPC, f, l);
            var GAHS = new GeomAdaptor_Surface(aST);

            var ACS = new Adaptor3d_CurveOnSurface(GHPC, GAHS);

            var aValidateEdge = new BRepLib_ValidateEdge(aGAC, ACS, SameParameter);
            aValidateEdge.SetControlPointsNumber(theNbControl - 1);
            aValidateEdge.Process();
            aValidateEdge.UpdateTolerance(ref theMaxdev);
            if (!aValidateEdge.IsDone())
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }
        }

        // For the planar face and non-existing  2d curve
        // check the deviation for the projection of the 3d curve on plane
        if (!IsPCurveFound && aFaceSurf is not null)
        {
            Geom2d_Curve? aPC = BRep_Tool.CurveOnPlane(theEdge, aFaceSurf, aFaceLoc, out _, out _);
            if (aPC is not null)
            {
                var GHPC = new Geom2dAdaptor_Curve(aPC, aFirst, aLast);

                Geom_Surface aST = (Geom_Surface)aFaceSurf.Transformed(aFaceLoc.Transformation());
                var GAHS = new GeomAdaptor_Surface(aST);

                var ACS = new Adaptor3d_CurveOnSurface(GHPC, GAHS);

                var aValidateEdgeOnPlane = new BRepLib_ValidateEdge(aGAC, ACS, SameParameter);
                aValidateEdgeOnPlane.SetControlPointsNumber(theNbControl - 1);
                aValidateEdgeOnPlane.Process();
                aValidateEdgeOnPlane.UpdateTolerance(ref theMaxdev);
                if (!aValidateEdgeOnPlane.IsDone())
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                }
            }
        }

        if (theMaxdev > TE.Tolerance())
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        if (!SameParameter)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }

        return Status(ShapeExtend_DONE);
    }

    // The file-static IsOverlapPartEdges of ShapeAnalysis_Edge.cxx.
    private static bool IsOverlapPartEdges(in TopoDS_Edge theFirstEdge,
                                           in TopoDS_Edge theSecEdge,
                                           double theTolerance,
                                           double theStep,
                                           double theStartLength,
                                           double theEndLength)
    {
        var aAdCurve1 = new BRepAdaptor_Curve(theFirstEdge);
        var aMinDist = new BRepExtrema_DistShapeShape();
        aMinDist.LoadS1(theSecEdge);
        for (double aS = theStartLength; aS <= theEndLength; aS += theStep / 2)
        {
            gp_Pnt aPoint;
            if (aS <= Precision.Confusion())
            {
                TopoDS_Vertex V1 = TopExp.FirstVertex(theFirstEdge, true);
                aPoint = BRep_Tool.Pnt(V1);
            }
            else
            {
                var aAbsPoint = new GCPnts_AbscissaPoint(Precision.Confusion(), aAdCurve1, aS, aAdCurve1.FirstParameter());
                if (aAbsPoint.IsDone())
                {
                    aAdCurve1.D0(aAbsPoint.Parameter(), out aPoint);
                }
                else
                {
                    continue;
                }
            }

            var aB = new BRep_Builder();
            TopoDS_Vertex aV = new TopoDS_Vertex();
            aB.MakeVertex(ref aV, aPoint, Precision.Confusion());
            aMinDist.LoadS2(aV);
            aMinDist.Perform();
            if (aMinDist.IsDone() && aMinDist.Value() >= theTolerance)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Checks whether the two edges overlap within <paramref name="theTolOverlap"/>: over the whole shorter edge
    /// (DONE3), or, if <paramref name="theDomainDist"/> isn't 0, over a segment of that length around their closest
    /// points (DONE4). <paramref name="theTolOverlap"/> receives the minimal distance found by the second check.
    /// </summary>
    /// <remarks>As in OCCT, the status isn't reset first: the flags add to the previous check's.</remarks>
    public bool CheckOverlapping(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2, ref double theTolOverlap, double theDomainDist = 0.0)
    {
        bool isOverlap = false;
        var aAdCurve1 = new BRepAdaptor_Curve(theEdge1);
        double aLength1 = GCPnts_AbscissaPoint.Length(aAdCurve1);
        var aAdCurve2 = new BRepAdaptor_Curve(theEdge2);
        double aLength2 = GCPnts_AbscissaPoint.Length(aAdCurve2);
        TopoDS_Edge aFirstEdge = (aLength1 >= aLength2 ? theEdge2 : theEdge1);
        TopoDS_Edge aSecEdge = (aLength1 >= aLength2 ? theEdge1 : theEdge2);
        double aLength = Min(aLength1, aLength2);

        // check overalpping between edges on whole edges
        double aStep = Min(aLength1, aLength2) / 2;
        isOverlap = IsOverlapPartEdges(aFirstEdge, aSecEdge, theTolOverlap, aStep, 0.0, Min(aLength1, aLength2));

        if (isOverlap)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            return isOverlap;
        }

        if (theDomainDist == 0.0)
        {
            return isOverlap;
        }

        // check overalpping between edges on segment with length less than theDomainDist

        double aDomainTol = (theDomainDist > Min(aLength1, aLength2) ? Min(aLength1, aLength2) : theDomainDist);
        var aMinDist = new BRepExtrema_DistShapeShape(aFirstEdge, aSecEdge, theTolOverlap);
        double aresTol = theTolOverlap;
        if (aMinDist.IsDone())
        {
            aresTol = aMinDist.Value();
            if (aresTol >= theTolOverlap)
            {
                return false;
            }

            int NbSol = aMinDist.NbSolution();
            for (int i = 1; i <= NbSol && !isOverlap; i++)
            {
                BRepExtrema_SupportType aType1 = aMinDist.SupportTypeShape1(i);
                double aEndLength, aStartLength, aLengthP;
                if (aType1 == BRepExtrema_IsVertex)
                {
                    TopoDS_Shape aSupportShape1 = aMinDist.SupportOnShape1(i);
                    TopExp.Vertices(aFirstEdge, out TopoDS_Vertex aV1, out _, true);
                    if (aV1.IsSame(aSupportShape1))
                    {
                        aLengthP = 0.0;
                    }
                    else
                    {
                        aLengthP = aLength;
                    }
                }
                else if (aType1 == BRepExtrema_IsOnEdge)
                {
                    aMinDist.ParOnEdgeS1(i, out double aParam1);
                    BRep_Tool.Range(aFirstEdge, out double aFirst, out _);
                    var anAdaptor = new BRepAdaptor_Curve(aFirstEdge);
                    aLengthP = GCPnts_AbscissaPoint.Length(anAdaptor, aFirst, aParam1);
                }
                else
                {
                    continue;
                }

                aStartLength = aLengthP - aDomainTol / 2;
                if (aStartLength < 0.0)
                {
                    aStartLength = 0;
                    aEndLength = aDomainTol;
                }

                aEndLength = aLengthP + aDomainTol / 2;
                if (aEndLength > aLength)
                {
                    aEndLength = aLength;
                    aStartLength = aEndLength - aDomainTol;
                }

                aStep = (aEndLength - aStartLength) / 5;
                isOverlap = (IsOverlapPartEdges(aFirstEdge, aSecEdge, theTolOverlap, aStep, aStartLength, aEndLength));
            }
        }

        if (isOverlap)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
        }

        theTolOverlap = aresTol;
        return isOverlap;
    }

    /// <summary>
    /// Tells if the range [theFirst, theLast] is valid for the pcurve: within its parameter range (of the basis curve
    /// for a trimmed curve), or no longer than its period for a periodic curve, up to Precision::PConfusion.
    /// </summary>
    public bool CheckPCurveRange(double theFirst, double theLast, Geom2d_Curve thePC)
    {
        double eps = Precision.PConfusion();
        bool isValid = true;
        bool IsPeriodic = thePC.IsPeriodic();
        double aPeriod = RealLast();
        if (IsPeriodic)
        {
            aPeriod = thePC.Period();
        }

        double fp = thePC.FirstParameter(), lp = thePC.LastParameter();
        if (thePC.GetType() == typeof(Geom2d_TrimmedCurve))
        {
            Geom2d_Curve aC = ((Geom2d_TrimmedCurve)thePC).BasisCurve();
            fp = aC.FirstParameter();
            lp = aC.LastParameter();
            IsPeriodic = aC.IsPeriodic();
            if (IsPeriodic)
            {
                aPeriod = aC.Period();
            }
        }

        if (IsPeriodic && (theLast - theFirst > aPeriod + eps))
        {
            isValid = false;
        }
        else if (!IsPeriodic && (theFirst < fp - eps || theLast > lp + eps))
        {
            isValid = false;
        }

        return isValid;
    }
}
