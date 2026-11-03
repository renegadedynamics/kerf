// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Edge.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Measurement.Distance.SupportType;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Tool for analysing edges: queries on the 3D curve and pcurves (taking the edge's orientation into account), and
/// checks of the vertices against the curves, of SameParameter, of overlapping. Each check sets a status, read with
/// <see cref="Status"/>.
/// </summary>
/// <remarks>A working object (decision 21): the checks change its status.</remarks>
public sealed class EdgeAnalysis
{
    private int _status;

    /// <summary>An analyser with an empty status.</summary>
    public EdgeAnalysis()
    {
        _status = 0; // ShapeExtend::EncodeStatus (ShapeExtend_OK);
    }

    /// <summary>Tells if the edge has a 3D curve.</summary>
    public bool HasCurve3d(in Edge edge)
    {
        Curve3D? c3d = Tool.Curve(edge, out _, out _);
        return c3d is not null;
    }

    /// <summary>
    /// The 3D curve of the edge, transformed by its location, with its range; if <paramref name="orient"/> and the
    /// edge is reversed, the range is swapped. Returns false if there is no 3D curve.
    /// </summary>
    public bool Curve3d(in Edge edge, out Curve3D? c3d, out double cf, out double cl, bool orient = true)
    {
        c3d = Tool.Curve(edge, out Placement l, out cf, out cl);
        if (c3d is not null && !l.IsIdentity)
        {
            c3d = (Curve3D)c3d.Transformed(l.Transformation);
            cf = c3d.TransformedParameter(cf, l.Transformation);
            cl = c3d.TransformedParameter(cl, l.Transformation);
        }

        if (orient)
        {
            if (edge.Orientation == Reversed)
            {
                double tmp = cf;
                cf = cl;
                cl = tmp;
            }
        }

        return c3d is not null;
    }

    /// <summary>Tells if the edge is closed in 3D: its curve is closed and its two vertices are the same.</summary>
    public bool IsClosed3d(in Edge edge)
    {
        Curve3D? c3d = Tool.Curve(edge, out _, out _);
        if (c3d is null)
        {
            return false;
        }

        if (!c3d.IsClosed)
        {
            return false;
        }

        return FirstVertex(edge).IsSame(LastVertex(edge));
    }

    /// <summary>Tells if the edge has a pcurve on the face.</summary>
    public bool HasPCurve(in Edge edge, in Face face)
    {
        Surface? s = Tool.Surface(face, out Placement l);
        return HasPCurve(edge, s, l);
    }

    /// <summary>Tells if the edge has a pcurve on the surface at the location.</summary>
    public bool HasPCurve(in Edge edge, Surface? surface, in Placement location)
    {
        Curve2D? c2d = Tool.CurveOnSurface(edge, surface, location, out _, out _);
        return c2d is not null;
    }

    /// <summary>
    /// The pcurve of the edge on the face, with its range; if <paramref name="orient"/> and the edge is reversed, the
    /// range is swapped. Returns false if there is none.
    /// </summary>
    public bool PCurve(in Edge edge, in Face face, out Curve2D? c2d, out double cf, out double cl, bool orient = true)
    {
        //: abv 20.05.02: take into account face orientation
        // COMMENTED BACK - NEEDS MORE CHANGES IN ALL SHAPEHEALING
        Surface? s = Tool.Surface(face, out Placement l);
        return PCurve(edge, s, l, out c2d, out cf, out cl, orient);
    }

    /// <summary>The pcurve of the edge on the surface at the location, with its range (swapped as above).</summary>
    public bool PCurve(in Edge edge,
                       Surface? surface,
                       in Placement location,
                       out Curve2D? c2d,
                       out double cf,
                       out double cl,
                       bool orient = true)
    {
        c2d = Tool.CurveOnSurface(edge, surface, location, out cf, out cl);
        if (orient && edge.Orientation == Reversed)
        {
            double tmp = cf;
            cf = cl;
            cl = tmp;
        }

        return c2d is not null;
    }

    /// <summary>The UV points of the ends of the edge's pcurve on the face; false if there is no pcurve.</summary>
    /// <remarks>The outputs are the origin when there is no pcurve (OCCT leaves them unchanged).</remarks>
    public bool BoundUV(in Edge edge, in Face face, out Point2D first, out Point2D last)
    {
        Surface? s = Tool.Surface(face, out Placement l);
        return BoundUV(edge, s, l, out first, out last);
    }

    /// <summary>The UV points of the ends of the edge's pcurve on the surface; false if there is no pcurve.</summary>
    public bool BoundUV(in Edge edge, Surface? surface, in Placement location, out Point2D first, out Point2D last)
    {
        if (!PCurve(edge, surface, location, out Curve2D? c2d, out double uf, out double ul))
        {
            first = new Point2D();
            last = new Point2D();
            return false;
        }

        first = c2d!.Value(uf);
        last = c2d.Value(ul);
        return true;
    }

    /// <summary>Tells if the edge is a seam of the face (two pcurves on it).</summary>
    public bool IsSeam(in Edge edge, in Face face)
    {
        return Tool.IsClosed(edge, face);
    }

    /// <summary>Tells if the edge is a seam of the surface at the location.</summary>
    public bool IsSeam(in Edge edge, Surface? surface, in Placement location)
    {
        return Tool.IsClosed(edge, surface, location);
    }

    /// <summary>The first vertex of the edge, taking its orientation into account (reversed for a reversed edge).</summary>
    public Vertex FirstVertex(in Edge edge)
    {
        Vertex v;
        if (edge.Orientation == Reversed)
        {
            v = Exploration.LastVertex(edge);
            v.Reverse();
        }
        else
        {
            v = Exploration.FirstVertex(edge);
        }

        return v;
    }

    /// <summary>The last vertex of the edge, taking its orientation into account (reversed for a reversed edge).</summary>
    public Vertex LastVertex(in Edge edge)
    {
        Vertex v;
        if (edge.Orientation == Reversed)
        {
            v = Exploration.FirstVertex(edge);
            v.Reverse();
        }
        else
        {
            v = Exploration.LastVertex(edge);
        }

        return v;
    }

    /// <summary>
    /// The end point of the edge's pcurve on the face (its start, or its end if <paramref name="atEnd"/>) and the
    /// tangent there, in the edge's direction. With <paramref name="dparam"/> &gt; 0, the tangent is the chord to the
    /// point at that fraction of the range. False if there is no pcurve or the tangent is null.
    /// </summary>
    public bool GetEndTangent2d(in Edge edge, in Face face, bool atEnd, out Point2D pos, out Vector2D tang, double dparam = 0.0)
    {
        Surface? s = Tool.Surface(face, out Placement l);
        return GetEndTangent2d(edge, s, l, atEnd, out pos, out tang, dparam);
    }

    /// <summary>As the face form, on the surface at the location.</summary>
    /// <remarks><paramref name="pos"/> is the origin when there is no pcurve (OCCT leaves it unchanged).</remarks>
    public bool GetEndTangent2d(in Edge edge,
                                Surface? s,
                                in Placement l,
                                bool atEnd,
                                out Point2D pos,
                                out Vector2D tang,
                                double dparam = 0.0)
    {
        pos = new Point2D();
        if (!PCurve(edge, s, l, out Curve2D? c2d, out double cf, out double cl))
        {
            tang = new Vector2D(0, 0);
            return false;
        }

        tang = new Vector2D();
        double dpnew = dparam;
        if (dpnew > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            double par1, par2, delta = (cl - cf) * dpnew;
            if (Math.Abs(delta) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                dpnew = 0.0;
            }
            else
            {
                Point2D ptmp = new Point2D();
                if (atEnd)
                {
                    par1 = cl;
                    par2 = cl - delta;
                    c2d!.D0(par1, out pos);
                    c2d.D0(par2, out ptmp);
                    tang = new Vector2D(pos.XY() - ptmp.XY());
                }
                else
                {
                    par1 = cf;
                    par2 = cf + delta;
                    c2d!.D0(par1, out pos);
                    c2d.D0(par2, out ptmp);
                    tang = new Vector2D(ptmp.XY() - pos.XY());
                }

                if (tang.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    dpnew = 0.0;
                }
            }
        }

        if (dpnew <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // get non-null tangency searching until 3rd derivative, or as straight btw ends
            double par = (atEnd ? cl : cf);
            c2d!.D1(par, out pos, out tang);
            if (tang.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                c2d.D2(par, out pos, out Vector2D d1, out tang);
                if (tang.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    c2d.D3(par, out pos, out d1, out Vector2D d2, out tang);
                    if (tang.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        c2d.D0((atEnd ? cf : cl), out Point2D p2);
                        tang = new Vector2D(p2.XY() - pos.XY());
                        if (tang.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            return false;
                        }
                    }
                }
            }

            if (edge.Orientation == Reversed)
            {
                tang = tang.Reversed();
            }
        }

        return true;
    }

    /// <summary>
    /// Checks the ends of the 3D curve against the vertices (within <paramref name="preci"/>, or each vertex's
    /// tolerance if negative); <paramref name="vtx"/> 1 or 2 checks only that vertex. True if a vertex is off:
    /// DONE1 for the first, DONE2 for the last; FAIL1 if there is no 3D curve.
    /// </summary>
    public bool CheckVerticesWithCurve3d(in Edge edge, double preci = -1, int vtx = 0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        Vertex v1 = FirstVertex(edge);
        Vertex v2 = LastVertex(edge);
        Point3D p1v = Tool.Pnt(v1);
        Point3D p2v = Tool.Pnt(v2);

        if (!Curve3d(edge, out Curve3D? c3d, out double cf, out double cl))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        //  on va faire les checks ...
        if (vtx != 2)
        {
            //  1er VTX
            Point3D p13d = c3d!.Value(cf);
            if (p1v.DistanceTo(p13d) > (preci < 0 ? Tool.Tolerance(v1) : preci))
            {
                _status |= (int)ExtendStatus.DONE1; // OCCT ors the enumerator, which equals its encoding for DONE1 and DONE2
            }
        }

        if (vtx != 1)
        {
            //  2me VTX
            Point3D p23d = c3d!.Value(cl);
            if (p2v.DistanceTo(p23d) > (preci < 0 ? Tool.Tolerance(v2) : preci))
            {
                _status |= (int)ExtendStatus.DONE2;
            }
        }

        return Status(ExtendStatus.Done);
    }

    /// <summary>
    /// Checks the ends of the pcurve on the face against the vertices, as <see cref="CheckVerticesWithCurve3d"/>;
    /// FAIL1 if there is no pcurve.
    /// </summary>
    public bool CheckVerticesWithPCurve(in Edge edge, in Face face, double preci = -1, int vtx = 0)
    {
        Surface? s = Tool.Surface(face, out Placement l);
        return CheckVerticesWithPCurve(edge, s, l, preci, vtx); // szv#4:S4163:12Mar99 `vtx,preci` wrong parameters order
    }

    /// <summary>As the face form, on the surface at the location.</summary>
    public bool CheckVerticesWithPCurve(in Edge edge, Surface? surf, in Placement loc, double preci = -1, int vtx = 0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        Vertex v1 = FirstVertex(edge);
        Vertex v2 = LastVertex(edge);
        Point3D p1v = Tool.Pnt(v1);
        Point3D p2v = Tool.Pnt(v2);

        if (!PCurve(edge, surf, loc, out Curve2D? c2d, out double cf, out double cl))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        // on va faire les checks ...
        if (vtx != 2)
        { //  1er VTX
            Point2D p1uv = c2d!.Value(cf);
            Point3D p12d = surf!.Value(p1uv.X, p1uv.Y);
            if (!loc.IsIdentity)
            {
                p12d = p12d.Transformed(loc.Transformation);
            }

            if (p1v.DistanceTo(p12d) > (preci < 0 ? Tool.Tolerance(v1) : preci))
            {
                _status |= (int)ExtendStatus.DONE1;
            }
        }

        if (vtx != 1)
        { //  2me VTX
            Point2D p2uv = c2d!.Value(cl);
            Point3D p22d = surf!.Value(p2uv.X, p2uv.Y);
            if (!loc.IsIdentity)
            {
                p22d = p22d.Transformed(loc.Transformation);
            }

            if (p2v.DistanceTo(p22d) > (preci < 0 ? Tool.Tolerance(v2) : preci))
            {
                _status |= (int)ExtendStatus.DONE2;
            }
        }

        return Status(ExtendStatus.Done);
    }

    // The file-static CheckVertexTolerance of ShapeAnalysis_Edge.cxx.
    private static int CheckVertexToleranceCore(in Edge edge, in Face face, bool checkAll, out double toler1, out double toler2)
    {
        int status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        var sae = new EdgeAnalysis();
        Vertex v1 = sae.FirstVertex(edge);
        Vertex v2 = sae.LastVertex(edge);
        if (v1.IsNull || v2.IsNull)
        { //: p1 abv 22 Feb 99: r76sy.stp
            status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            // OCCT leaves the tolerances unchanged here; the port's outputs are 0
            toler1 = toler2 = 0.0;
            return status;
        }

        double old1 = Tool.Tolerance(v1);
        double old2 = Tool.Tolerance(v2);
        Point3D pnt1 = Tool.Pnt(v1);
        Point3D pnt2 = Tool.Pnt(v2);

        double a, b;
        if (!sae.Curve3d(edge, out Curve3D? c3d, out a, out b, true))
        {
            if (!Tool.Degenerated(edge))
            {
                status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }

            toler1 = toler2 = 0.0;
        }
        else
        {
            toler1 = pnt1.SquareDistanceTo(c3d!.Value(a));
            toler2 = pnt2.SquareDistanceTo(c3d.Value(b));
        }

        if (checkAll)
        {
            BrepTEdge te = (BrepTEdge)edge.TShape()!;
            for (var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves()); itcr.More(); itcr.Next())
            {
                if (itcr.Value() is not GCurve gc || !gc.IsCurveOnSurface())
                {
                    continue;
                }

                Surface s = gc.Surface()!;
                Placement l = edge.Placement * gc.Location();
                sae.PCurve(edge, s, l, out Curve2D? pcurve, out a, out b, true);
                Point2D p1 = pcurve!.Value(a);
                Point2D p2 = pcurve.Value(b);
                Point3D P1 = s.Value(p1.X, p1.Y).Transformed(l.Transformation);
                Point3D P2 = s.Value(p2.X, p2.Y).Transformed(l.Transformation);
                toler1 = Max(toler1, pnt1.SquareDistanceTo(P1));
                toler2 = Max(toler2, pnt2.SquareDistanceTo(P2));
            }
        }
        //: abv 10.06.02: porting C40 -> dev (CC670-12608.stp)
        // Check with given face is needed for plane surfaces (if no stored pcurves)
        else if (!face.IsNull)
        {
            Surface? s = Tool.Surface(face, out Placement l);
            if (sae.PCurve(edge, s, l, out Curve2D? pcurve, out a, out b, true))
            {
                Point2D p1 = pcurve!.Value(a);
                Point2D p2 = pcurve.Value(b);
                Point3D P1 = s!.Value(p1.X, p1.Y).Transformed(l.Transformation);
                Point3D P2 = s.Value(p2.X, p2.Y).Transformed(l.Transformation);
                toler1 = Max(toler1, pnt1.SquareDistanceTo(P1));
                toler2 = Max(toler2, pnt2.SquareDistanceTo(P2));
            }
            else
            {
                status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            }
        }

        //: o8 abv 19 Feb 99: CTS18541.stp #18559: coeff 1.0001 added
        // szv 18 Aug 99: edge tolerance is taken in consideration
        double tole = Tool.Tolerance(edge);
        toler1 = Max(1.0000001 * Math.Sqrt(toler1), tole);
        toler2 = Max(1.0000001 * Math.Sqrt(toler2), tole);
        if (toler1 > old1)
        {
            status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        if (toler2 > old2)
        {
            status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }

        return status;
    }

    /// <summary>
    /// Computes the tolerances the vertices need to cover the ends of the 3D curve and of the pcurve on the face (at
    /// least the edge's tolerance). True if one exceeds the vertex's present tolerance: DONE1 for the first, DONE2
    /// for the last. FAIL1 if a vertex is missing, FAIL2 if there is no 3D curve, FAIL3 if no pcurve on the face.
    /// </summary>
    public bool CheckVertexTolerance(in Edge edge, in Face face, out double toler1, out double toler2)
    {
        _status = CheckVertexToleranceCore(edge, face, false, out toler1, out toler2);
        return Status(ExtendStatus.Done);
    }

    /// <summary>As the face form, against every pcurve of the edge.</summary>
    public bool CheckVertexTolerance(in Edge edge, out double toler1, out double toler2)
    {
        Face f = new Face();
        _status = CheckVertexToleranceCore(edge, f, true, out toler1, out toler2);
        return Status(ExtendStatus.Done);
    }

    /// <summary>
    /// Checks that the ends of the 3D curve and of the pcurve on the face match within the vertices' tolerances. True
    /// if they don't; DONE1 if they would match reversed. False for a plane, FAIL1 without a pcurve, FAIL2 without a
    /// 3D curve.
    /// </summary>
    public bool CheckCurve3dWithPCurve(in Edge edge, in Face face)
    {
        Surface? s = Tool.Surface(face, out Placement l);
        return CheckCurve3dWithPCurve(edge, s!, l);
    }

    /// <summary>As the face form, on the surface at the location.</summary>
    public bool CheckCurve3dWithPCurve(in Edge edge, Surface surface, in Placement location)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        if (surface is PlaneSurface)
        {
            return false;
        }

        if (!PCurve(edge, surface, location, out Curve2D? c2d, out double f2d, out double l2d, false))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        if (!Curve3d(edge, out Curve3D? c3d, out double f3d, out double l3d, false))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            return false;
        }

        Vertex firstVert = FirstVertex(edge);
        Vertex lastVert = LastVertex(edge);

        if (firstVert.IsNull || lastVert.IsNull)
        {
            return false;
        }

        double preci1 = Tool.Tolerance(firstVert), preci2 = Tool.Tolerance(lastVert);

        Point2D p2d1 = c2d!.Value(f2d), p2d2 = c2d.Value(l2d);

        // #39 rln 17.11.98 S4054, annie_surf.igs entity 39
        return CheckPoints(c3d!.Value(f3d),
                           c3d.Value(l3d),
                           surface.Value(p2d1.X, p2d1.Y).Transformed(location.Transformation),
                           surface.Value(p2d2.X, p2d2.Y).Transformed(location.Transformation),
                           preci1,
                           preci2);
    }

    private bool CheckPoints(in Point3D p1A, in Point3D p1B, in Point3D p2A, in Point3D p2B, double preci1, double preci2)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (p1A.SquareDistanceTo(p2A) <= preci1 * preci1 && p1B.SquareDistanceTo(p2B) <= preci2 * preci2)
        {
            return false;
        }
        else if (p1A.DistanceTo(p2B) + (p1B.DistanceTo(p2A)) < p1A.DistanceTo(p2A) + (p1B.DistanceTo(p2B)))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        return true;
    }

    /// <summary>Tells if the status of the last check holds <paramref name="status"/>.</summary>
    public bool Status(ExtendStatus status)
    {
        return global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);
    }

    /// <summary>
    /// Computes the largest deviation of the edge's pcurves from its 3D curve (<paramref name="maxdev"/>), as
    /// BRepLib_ValidateEdge does at <paramref name="nbControl"/> points. True if it exceeds the edge's tolerance
    /// (DONE1) or the edge isn't SameParameter (DONE2). False for a degenerated edge; FAIL1 without a 3D curve, FAIL2
    /// if a deviation couldn't be computed.
    /// </summary>
    /// <remarks><paramref name="maxdev"/> is unchanged for a degenerated edge, as in OCCT.</remarks>
    public bool CheckSameParameter(in Edge edge, ref double maxdev, int nbControl = 23)
    {
        Face emptyFace = new Face();
        return CheckSameParameter(edge, emptyFace, ref maxdev, nbControl);
    }

    /// <summary>As above, for the pcurve on the face only (or the projection on a plane without a stored pcurve).</summary>
    public bool CheckSameParameter(in Edge edge, in Face face, ref double maxdev, int nbControl = 23)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (Tool.Degenerated(edge))
        {
            return false;
        }

        maxdev = 0;

        // Get same parameter flag
        BrepTEdge te = (BrepTEdge)edge.TShape()!;
        bool sameParameter = te.SameParameter();

        // Get 3D curve of the edge
        Curve3D? c3D = Tool.Curve(edge, out Placement curveLoc, out double first, out double last);
        if (c3D is null)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        if (!curveLoc.IsIdentity)
        {
            Transform3D trsf = curveLoc.Transformation;
            c3D = (Curve3D)c3D.Transformed(trsf);
            first = c3D.TransformedParameter(first, trsf);
            last = c3D.TransformedParameter(last, trsf);
        }

        // Create adaptor for the curve
        var gac = new CurveAdaptor(c3D, first, last);

        Surface? faceSurf = null;
        Placement faceLoc = new Placement();
        if (!face.IsNull)
        {
            faceSurf = Tool.Surface(face, out faceLoc);
        }

        bool isPCurveFound = false;
        int i = 1;

        // Iterate on all curve representations
        for (;;)
        {
            Curve2D? pc = null;
            Surface? s = null;
            Placement loc = new Placement();
            double f = 0, l = 0;
            Tool.CurveOnSurface(edge, ref pc, ref s, ref loc, ref f, ref l, i);

            if (pc is null)
            {
                // No more curves
                break;
            }

            ++i;

            // If the input face is not null, check that the curve is on its surface
            if (faceSurf is not null)
            {
                if (faceSurf != s || !faceLoc.IsEqual(loc))
                {
                    continue;
                }
            }

            isPCurveFound = true;

            // Apply transformations for the surface
            Surface st = (Surface)s!.Transformed(loc.Transformation);

            // Compute deviation between curves
            var ghpc = new CurveAdaptor2D(pc, f, l);
            var gahs = new SurfaceAdaptor(st);

            var acs = new CurveOnSurface(ghpc, gahs);

            var validateEdge = new ValidateEdge(gac, acs, sameParameter);
            validateEdge.SetControlPointsNumber(nbControl - 1);
            validateEdge.Process();
            validateEdge.UpdateTolerance(ref maxdev);
            if (!validateEdge.IsDone)
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }
        }

        // For the planar face and non-existing  2d curve
        // check the deviation for the projection of the 3d curve on plane
        if (!isPCurveFound && faceSurf is not null)
        {
            Curve2D? pc = Tool.CurveOnPlane(edge, faceSurf, faceLoc, out _, out _);
            if (pc is not null)
            {
                var ghpc = new CurveAdaptor2D(pc, first, last);

                Surface st = (Surface)faceSurf.Transformed(faceLoc.Transformation);
                var gahs = new SurfaceAdaptor(st);

                var acs = new CurveOnSurface(ghpc, gahs);

                var validateEdgeOnPlane = new ValidateEdge(gac, acs, sameParameter);
                validateEdgeOnPlane.SetControlPointsNumber(nbControl - 1);
                validateEdgeOnPlane.Process();
                validateEdgeOnPlane.UpdateTolerance(ref maxdev);
                if (!validateEdgeOnPlane.IsDone)
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                }
            }
        }

        if (maxdev > te.Tolerance())
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        if (!sameParameter)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }

        return Status(ExtendStatus.Done);
    }

    // The file-static IsOverlapPartEdges of ShapeAnalysis_Edge.cxx.
    private static bool IsOverlapPartEdges(in Edge firstEdge,
                                           in Edge secEdge,
                                           double tolerance,
                                           double step,
                                           double startLength,
                                           double endLength)
    {
        var adCurve1 = new EdgeCurveAdaptor(firstEdge);
        var minDist = new DistanceShapeShape();
        minDist.LoadS1(secEdge);
        // The step is half the shorter edge's length: 0 for a zero-length edge, whose one point is checked once, where
        // OCCT looped without end (decision 1431).
        for (double s = startLength; s <= endLength; s = NextSample(s, step))
        {
            Point3D point;
            if (s <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                Vertex v1 = Exploration.FirstVertex(firstEdge, true);
                point = Tool.Pnt(v1);
            }
            else
            {
                var absPoint = new AbscissaPoint(global::Kerf.Engine.Foundation.Precision.Confusion(), adCurve1, s, adCurve1.FirstParameter());
                if (absPoint.IsDone)
                {
                    adCurve1.D0(absPoint.Parameter(), out point);
                }
                else
                {
                    continue;
                }
            }

            var b = new BrepBuilder();
            Vertex v = new Vertex();
            b.MakeVertex(ref v, point, global::Kerf.Engine.Foundation.Precision.Confusion());
            minDist.LoadS2(v);
            minDist.Perform();
            if (minDist.IsDone && minDist.Value() >= tolerance)
            {
                return false;
            }
        }

        return true;
    }

    // The next sampled length, beyond the end when the step no longer advances.
    private static double NextSample(double s, double step)
    {
        double next = s + step / 2;
        return next > s ? next : double.PositiveInfinity;
    }

    /// <summary>
    /// Checks whether the two edges overlap within <paramref name="tolOverlap"/>: over the whole shorter edge
    /// (DONE3), or, if <paramref name="domainDist"/> isn't 0, over a segment of that length around their closest
    /// points (DONE4). <paramref name="tolOverlap"/> receives the minimal distance found by the second check.
    /// </summary>
    /// <remarks>As in OCCT, the status isn't reset first: the flags add to the previous check's.</remarks>
    public bool CheckOverlapping(in Edge edge1, in Edge edge2, ref double tolOverlap, double domainDist = 0.0)
    {
        bool isOverlap = false;
        var adCurve1 = new EdgeCurveAdaptor(edge1);
        double length1 = AbscissaPoint.Length(adCurve1);
        var adCurve2 = new EdgeCurveAdaptor(edge2);
        double length2 = AbscissaPoint.Length(adCurve2);
        Edge firstEdge = (length1 >= length2 ? edge2 : edge1);
        Edge secEdge = (length1 >= length2 ? edge1 : edge2);
        double length = Min(length1, length2);

        // check overalpping between edges on whole edges
        double step = Min(length1, length2) / 2;
        isOverlap = IsOverlapPartEdges(firstEdge, secEdge, tolOverlap, step, 0.0, Min(length1, length2));

        if (isOverlap)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
            return isOverlap;
        }

        if (domainDist == 0.0)
        {
            return isOverlap;
        }

        // check overalpping between edges on segment with length less than theDomainDist

        double domainTol = (domainDist > Min(length1, length2) ? Min(length1, length2) : domainDist);
        var minDist = new DistanceShapeShape(firstEdge, secEdge, tolOverlap);
        double aresTol = tolOverlap;
        if (minDist.IsDone)
        {
            aresTol = minDist.Value();
            if (aresTol >= tolOverlap)
            {
                return false;
            }

            int nbSol = minDist.SolutionCount;
            for (int i = 1; i <= nbSol && !isOverlap; i++)
            {
                SupportType type1 = minDist.SupportTypeShape1(i);
                double endLength, startLength, lengthP;
                if (type1 == SupportType.IsVertex)
                {
                    Shape supportShape1 = minDist.SupportOnShape1(i);
                    Exploration.Vertices(firstEdge, out Vertex v1, out _, true);
                    if (v1.IsSame(supportShape1))
                    {
                        lengthP = 0.0;
                    }
                    else
                    {
                        lengthP = length;
                    }
                }
                else if (type1 == SupportType.IsOnEdge)
                {
                    minDist.ParOnEdgeS1(i, out double param1);
                    Tool.Range(firstEdge, out double first, out _);
                    var adaptor = new EdgeCurveAdaptor(firstEdge);
                    lengthP = AbscissaPoint.Length(adaptor, first, param1);
                }
                else
                {
                    continue;
                }

                startLength = lengthP - domainTol / 2;
                if (startLength < 0.0)
                {
                    startLength = 0;
                    endLength = domainTol;
                }

                endLength = lengthP + domainTol / 2;
                if (endLength > length)
                {
                    endLength = length;
                    startLength = endLength - domainTol;
                }

                step = (endLength - startLength) / 5;
                isOverlap = (IsOverlapPartEdges(firstEdge, secEdge, tolOverlap, step, startLength, endLength));
            }
        }

        if (isOverlap)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
        }

        tolOverlap = aresTol;
        return isOverlap;
    }

    /// <summary>
    /// Tells if the range [theFirst, theLast] is valid for the pcurve: within its parameter range (of the basis curve
    /// for a trimmed curve), or no longer than its period for a periodic curve, up to Precision::PConfusion.
    /// </summary>
    public bool CheckPCurveRange(double first, double last, Curve2D pc)
    {
        double eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool isValid = true;
        bool isPeriodic = pc.IsPeriodic;
        double period = RealLast();
        if (isPeriodic)
        {
            period = pc.Period;
        }

        double fp = pc.FirstParameter, lp = pc.LastParameter;
        if (pc.GetType() == typeof(TrimmedCurve2D))
        {
            Curve2D c = ((TrimmedCurve2D)pc).BasisCurve;
            fp = c.FirstParameter;
            lp = c.LastParameter;
            isPeriodic = c.IsPeriodic;
            if (isPeriodic)
            {
                period = c.Period;
            }
        }

        if (isPeriodic && (last - first > period + eps))
        {
            isValid = false;
        }
        else if (!isPeriodic && (first < fp - eps || last > lp + eps))
        {
            isValid = false;
        }

        return isValid;
    }
}
