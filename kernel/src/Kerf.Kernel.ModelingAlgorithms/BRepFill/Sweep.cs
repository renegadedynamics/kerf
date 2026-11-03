// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Sweep.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Sweep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Filling.TransitionStyle;
using static Kerf.Engine.Construction.Library.LibraryFaceError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Construction.Surfaces.ApproxStyle;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// The topological sweep of a section law along a location law: a wire (the section is a vertex) or a shell of faces
/// built from GeomFill_Sweep's surfaces, with the corners of a path that is not G1 treated by the transition mode.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21) used by <see cref="Pipe"/> and <see cref="PipeShell"/>, as in
/// OCCT: it writes into the section edges and vertices it is given (the bounds, the section law's shapes), builds on
/// them, and hands out the unfrozen result and tables. Its callers give it shapes they own (decision 924); a frozen
/// shape reaching a write throws <see cref="ImmutableObjectException"/>. It calls <c>SetInterval</c> and
/// <c>SetTolerance</c> on the location law's elementary laws, which are BRepFill's per-edge copies (decision 900).
/// </para>
/// </remarks>
internal sealed class Sweep
{
    private bool _isDone;
    private readonly bool _kPart;
    private double _tol3d;
    private double _boundTol;
    private double _tol2d;
    private double _tolAngular;
    private double _angMin;
    private double _angMax;
    private ApproxStyle _approxStyle;
    private Continuity _continuity;
    private int _degmax;
    private int _segmax;
    private bool _forceApproxC1;
    private Shape _shape = new();
    private readonly LocationLaw _loc;
    private readonly SectionLaw _sec;
    private NCollection_HArray2<Shape>? _uEdges;
    private NCollection_HArray2<Shape>? _vEdges;
    private readonly Dictionary<Shape, Shape> _vEdgesModified = new(ShapeHasher.Instance);
    private NCollection_HArray2<Shape>? _faces;
    private readonly NCollection_List<Shape> _auxShape = new();
    private NCollection_HArray1<Shape>? _tapes;
    private double _error;
    private Wire _firstShape = new();
    private Wire _lastShape = new();

    //=================================================================================================

    private static int NumberOfPoles(in Wire w)
    {
        int nbPoints = 0;

        Iterator iter = new(w);
        for (; iter.More(); iter.Next())
        {
            EdgeCurveAdaptor c = new(ShapeCasts.Edge(iter.Value()));

            double dfUf = c.FirstParameter();
            double dfUl = c.LastParameter();
            if (IsEqual(dfUf, dfUl))
            {
                // Degenerate
                continue;
            }

            switch (c.GetType())
            {
                case BezierCurve:
                {
                    // Put all poles for bezier
                    BezierCurve3D gc = c.Bezier();
                    int iNbPol = gc.PoleCount;
                    if (iNbPol >= 2)
                    {
                        nbPoints += iNbPol;
                    }
                    break;
                }
                case BSplineCurve:
                {
                    // Put all poles for bspline
                    BSplineCurve3D gc = c.BSpline();
                    int iNbPol = gc.PoleCount;
                    if (iNbPol >= 2)
                    {
                        nbPoints += iNbPol;
                    }
                    break;
                }
                case Line:
                {
                    nbPoints += 2;
                    break;
                }
                case Circle:
                case Ellipse:
                case Hyperbola:
                case Parabola:
                {
                    nbPoints += 4;
                    break;
                }
                default:
                    nbPoints += 15 + c.NbIntervals(C3);
                    break;
            } // switch (c.GetType()) ...
        } // for (; iter.More(); iter.Next())

        return nbPoints;
    }

    //=================================================================================================

    private static bool HasPCurves(in Edge e)
    {
        bool haspcurves = false;

        NCollection_List<CurveRepresentation>.Iterator itcr = new(((BrepTEdge)e.TShape()!).Curves());
        for (; itcr.More(); itcr.Next())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface())
            {
                haspcurves = true;
                break;
            }
        }
        return haspcurves;
    }

    //=======================================================================
    // function : Translate
    // purpose  : Copy a column from one table to another.
    //=======================================================================
    private static void Translate(NCollection_HArray2<Shape> arrayIn,
                                  int In,
                                  NCollection_HArray2<Shape> arrayOut,
                                  int Out)
    {
        int ii, nb;
        nb = arrayOut.ColLength();
        for (ii = 1; ii <= nb; ii++)
        {
            arrayOut.SetValue(ii, Out, arrayIn.Value(ii, In));
        }
    }

    //=================================================================================================

    private static void Box(SurfaceSectionLaw sec, double u, ref Box box)
    {
        box.SetVoid();
        sec.SectionShape(out int nbPoles, out _, out _);
        NCollection_Array1<Point3D> poles = new(1, nbPoles);
        NCollection_Array1<double> w = new(1, nbPoles);
        sec.D0(u, poles, w);
        for (int ii = 1; ii <= nbPoles; ii++)
        {
            box.Add(poles[ii]);
        }
    }

    //=======================================================================
    // function : Couture
    // purpose  : Check if E is an edge of sewing on S
    //           and make the representation HadHoc
    //=======================================================================
    private static Curve2D? Couture(in Edge e, Surface s, in Placement L)
    {
        Placement l = L.Predivided(e.Location());
        bool eisreversed = (e.Orientation() == Reversed);

        // find the representation (OCCT reads ChangeCurves(); the port only reads the list)
        NCollection_List<CurveRepresentation>.Iterator itcr = new(((BrepTEdge)e.TShape()!).Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(s, l))
            {
                GCurve gc = (GCurve)cr;
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
        Curve2D? pc = null;
        return pc;
    }

    //=======================================================================
    // function : CheckSameParameter
    // purpose  : Check a posteriori that sameparameter has worked correctly
    //=======================================================================

    private static bool CheckSameParameter(CurveAdaptorBase c3d,
                                           Curve2D pcurv,
                                           SurfaceAdaptorBase s,
                                           double tol3d,
                                           out double tolreached)
    {
        tolreached = 0.0;
        double f = c3d.FirstParameter();
        double l = c3d.LastParameter();
        int nbp = 45;
        double step = 1.0 / (nbp - 1);
        for (int i = 0; i < nbp; i++)
        {
            double t;
            t = step * i;
            t = (1 - t) * f + t * l;
            pcurv.Value(t).Coord(out double u, out double v);
            Point3D pS = s.Value(u, v);
            Point3D pC = c3d.Value(t);
            double d2 = pS.SquareDistance(pC);
            tolreached = Max(tolreached, d2);
        }
        tolreached = Math.Sqrt(tolreached);
        if (tolreached > tol3d)
        {
            tolreached *= 2.0;
            return false;
        }
        tolreached *= 2.0;
        tolreached = Max(tolreached, global::Kerf.Engine.Foundation.Precision.Confusion());
        return true;
    }

    //=======================================================================
    // function : CheckSameParameterExact
    // purpose  : Check a posteriori that sameparameter has worked correctly
    // with exact calculation method of edge tolerance
    //=======================================================================
    private static bool CheckSameParameterExact(CurveAdaptorBase c3d,
                                                CurveOnSurface curveOnSurface,
                                                double tol3d,
                                                out double tolreached)
    {
        CheckCurveOnSurface checkCurveOnSurface = new(c3d);
        checkCurveOnSurface.SetParallel(false);
        checkCurveOnSurface.Perform(curveOnSurface);

        tolreached = checkCurveOnSurface.MaxDistance();

        if (tolreached > tol3d)
        {
            return false;
        }
        else
        {
            tolreached = Max(tolreached, global::Kerf.Engine.Foundation.Precision.Confusion());
            tolreached *= 1.05;
        }
        return true;
    }

    //=======================================================================
    // function : SameParameter
    // purpose  : Encapsulation of Sameparameter
    // Boolean informs if the pcurve was computed or not...
    // The tolerance is always OK.
    //=======================================================================

    private static bool SameParameter(in Edge e,
                                      ref Curve2D pcurv,
                                      Surface surf,
                                      double tol3d,
                                      out double tolreached)
    {
        Curve3D? c3d = Tool.Curve(e, out double f, out double l);
        CurveAdaptor gac3d = new(c3d!, f, l);
        CurveAdaptor hc3d = gac3d; // OCCT copies the adaptor into a handle

        SurfaceAdaptor s = new(surf);

        if (CheckSameParameter(hc3d, pcurv, s, tol3d, out tolreached))
        {
            return true;
        }

        if (!HasPCurves(e))
        {
            CurveAdaptor2D hc2d = new(pcurv);
            CurveOnSurfaceApproximation appCurve = new(hc2d,
                                                 s,
                                                 hc2d.FirstParameter(),
                                                 hc2d.LastParameter(),
                                                 global::Kerf.Engine.Foundation.Precision.Confusion());
            appCurve.Perform(10, 10, C1, true);
            if (appCurve.IsDone && appCurve.HasResult)
            {
                c3d = appCurve.Curve3d();
                tolreached = appCurve.MaxError3d();
                BrepBuilder b = new();
                b.UpdateEdge(e, c3d, tolreached);
                return true;
            }
        }

        CurveAdaptorBase hCurve = hc3d; // to avoid ambiguity
        SameParameter sp = new(hCurve, pcurv, s, tol3d);
        if (sp.IsDone && !sp.IsSameParameter)
        {
            pcurv = sp.Curve2d()!;
        }
        else if (!sp.IsDone && !sp.IsSameParameter)
        {
            return false;
        }

        CurveAdaptorBase curve3d = sp.Curve3d()!;
        CurveOnSurface curveOnSurface = sp.CurveOnSurface()!;

        if (!CheckSameParameterExact(curve3d, curveOnSurface, tol3d, out double resTol) && resTol > tolreached)
        {
            return false;
        }
        else
        {
            tolreached = 1.1 * resTol;
            if (sp.IsDone && !sp.IsSameParameter)
            {
                pcurv = sp.Curve2d()!;
            }
        }
        return true;
    }

    private static void CorrectSameParameter(Edge edge, in Face face1, in Face face2)
    {
        if (Tool.Degenerated(edge))
        {
            return;
        }

        Curve3D curve = Tool.Curve(edge, out double fpar, out double lpar)!;

        bool[] pCurveExists = [false, false];
        EdgeCurveAdaptor[] bAcurve = [new(), new()];

        if (!face1.IsNull)
        {
            pCurveExists[0] = true;
            bAcurve[0].Initialize(edge, face1);
        }
        if (!face1.IsNull && face1.IsSame(face2))
        {
            edge.Reverse();
        }
        if (!face2.IsNull)
        {
            pCurveExists[1] = true;
            bAcurve[1].Initialize(edge, face2);
        }

        double maxSqDist = 0.0;
        const int ncontrol = 23;
        double delta = (lpar - fpar) / ncontrol;

        for (int i = 0; i <= ncontrol; i++)
        {
            double param = fpar + i * delta;
            Point3D pnt = curve.Value(param);
            for (int j = 0; j < 2; j++)
            {
                if (pCurveExists[j])
                {
                    Point3D pntFromFace = bAcurve[j].Value(param);
                    double sqDist = pnt.SquareDistance(pntFromFace);
                    if (sqDist > maxSqDist)
                    {
                        maxSqDist = sqDist;
                    }
                }
            }
        }

        double tol = Math.Sqrt(maxSqDist);
        BrepBuilder bb = new();
        bb.UpdateEdge(edge, tol);
    }

    //=======================================================================
    // Objet : Orientate an edge of natural restriction
    //      : General
    //=======================================================================
    private static void Oriente(Surface s, ref Edge e)
    {
        Vector2D uRef = new(1, 0), vRef = new(0, 1);
        bool isuiso, isfirst, isopposite;
        s.Bounds(out double uFirst, out _, out double vFirst, out _);
        Curve2D c;

        c = Tool.CurveOnSurface(e, s, new Placement(), out double f, out double l)!;
        c.D1((f + l) / 2, out Point2D p, out Vector2D d);

        isuiso = d.IsParallel(vRef, 0.1);

        if (isuiso)
        {
            isfirst = (Math.Abs(p.X() - uFirst) < global::Kerf.Engine.Foundation.Precision.Confusion());
            isopposite = d.IsOpposite(vRef, 0.1);
            e.Orientation(Reversed);
        }
        else
        {
            isfirst = (Math.Abs(p.Y() - vFirst) < global::Kerf.Engine.Foundation.Precision.Confusion());
            isopposite = d.IsOpposite(uRef, 0.1);
            e.Orientation(Forward);
        }

        if (!isfirst)
        {
            e.Reverse();
        }
        if (isopposite)
        {
            e.Reverse();
        }
    }

    // OCC500(apo)->
    private static void UpdateEdgeOnPlane(in Face f, in Edge e, BrepBuilder bb)
    {
        Curve2D? c2d = Tool.CurveOnSurface(e, f, out _, out _);
        Surface? s = Tool.Surface(f);
        Placement loc = new();
        double tol = Tool.Tolerance(e);
        bb.UpdateEdge(e, c2d, s, loc, tol);
        EdgeCheck check = new(e);
        tol = Max(tol, check.Tolerance());
        bb.UpdateEdge(e, tol);
        Vertex v;
        tol *= 1.01;
        v = Exploration.FirstVertex(e);
        if (Tool.Tolerance(v) < tol)
        {
            bb.UpdateVertex(v, tol);
        }
        v = Exploration.LastVertex(e);
        if (Tool.Tolerance(v) < tol)
        {
            bb.UpdateVertex(v, tol);
        }
    }

    // OCCT.NET: BuildFace's three repeated blocks. Adds E to the wire maker, then binds E to the edge actually added
    // (the last of the wire) when the maker replaced it.
    private static void AddAndBind(MakeWire b,
                                   in Edge e,
                                   Dictionary<Shape, Shape> eEmap)
    {
        b.Add(e);
        if (!Tool.Degenerated(e))
        {
            Wire ww = b.InternalWire();
            Shape newEdge = new();
            // take the last edge added to WW
            for (Iterator iter = new(ww); iter.More(); iter.Next())
            {
                newEdge = iter.Value();
            }
            if (!e.IsSame(newEdge))
            {
                eEmap[e] = newEdge;
            }
        }
    }

    //<-OCC500(apo)
    //=======================================================================
    // Function : BuildFace
    // Objet : Construct a Face via a surface and 4 Edges (natural borders)
    //      : Only one Hypothesis : isos u and v are switched :
    //        Edge1/3 are iso u (recp v)
    //        Edge2/4 are iso v (recp u)
    //=======================================================================
    private static void BuildFace(Surface s,
                                  in Edge E1,
                                  in Edge E2,
                                  in Edge e3,
                                  in Edge e4,
                                  Dictionary<Shape, Shape> eEmap,
                                  bool exchUV,
                                  bool uReverse,
                                  ref Face f)
    {
        double tol;
        Edge e1, e2, e;
        Wire ww;
        BrepBuilder bb = new();
        MakeWire b = new();

        // Is the surface planar ?
        double tol1, tol2, tol3, tol4;
        tol1 = Tool.Tolerance(E1);
        tol2 = Tool.Tolerance(E2);
        tol3 = Tool.Tolerance(e3);
        tol4 = Tool.Tolerance(e4);
        tol = Min(tol1, tol2);
        tol = Min(tol, Min(tol3, tol4));
        bool isPlan = false;
        PlaneSurface? thePlane = null;

        if (!E1.IsSame(e3) && !E2.IsSame(e4)) // exclude cases with seam edges: they are not planar
        {
            IsPlanarSurface isP = new(s, tol);
            if (isP.IsPlanar)
            {
                isPlan = true;
                thePlane = new PlaneSurface(isP.Plan());
            }
            else
            {
                BrepTEdge te1 = (BrepTEdge)E1.TShape()!;
                BrepTEdge te2 = (BrepTEdge)E2.TShape()!;
                BrepTEdge te3 = (BrepTEdge)e3.TShape()!;
                BrepTEdge te4 = (BrepTEdge)e4.TShape()!;
                te1.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                te2.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                te3.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                te4.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());

                Wire theWire = new LibraryMakeWire(E1, E2, e3, e4).InternalWire();
                int nbPoints = NumberOfPoles(theWire);
                if (nbPoints <= 100) // limitation for CPU
                {
                    FindSurface fs = new(theWire, -1, true);
                    if (fs.Found())
                    {
                        isPlan = true;
                        thePlane = fs.InternalSurface() as PlaneSurface;
                    }
                }
                bb.UpdateEdge(E1, tol1);
                bb.UpdateEdge(E2, tol2);
                bb.UpdateEdge(e3, tol3);
                bb.UpdateEdge(e4, tol4);
            }
        }

        // Construction of the wire
        e1 = E1;
        Oriente(s, ref e1);
        if (!isPlan || !Tool.Degenerated(e1))
        {
            b.Add(e1);
        }

        e2 = E2;
        Oriente(s, ref e2);
        if (!isPlan || !Tool.Degenerated(e2))
        {
            AddAndBind(b, e2, eEmap);
        }

        if (e3.IsSame(E1))
        {
            e = e1;
            e.Reverse();
        }
        else
        {
            e = e3;
            Oriente(s, ref e);
        }
        if (!isPlan || !Tool.Degenerated(e))
        {
            AddAndBind(b, e, eEmap);
        }

        if (e4.IsSame(E2))
        {
            e = e2;
            e.Reverse();
        }
        else
        {
            e = e4;
            Oriente(s, ref e);
        }
        if (!isPlan || !Tool.Degenerated(e))
        {
            AddAndBind(b, e, eEmap);
        }

        ww = b.InternalWire();

        // Construction of the face.
        if (isPlan)
        { // Suspend representation 2d
          // and construct face Plane
            Point3D pnt;
            Vector3D du, dv, ns, np;
            s.Bounds(out double ufirst, out double ulast, out double vfirst, out double vlast);
            s.D1((ufirst + ulast) / 2.0, (vfirst + vlast) / 2.0, out pnt, out du, out dv);
            ns = du ^ dv;
            np = new Vector3D(thePlane!.Pln().Axis().Direction());
            if (ns.Dot(np) < 0.0)
            {
                if (thePlane.IsFrozen)
                {
                    // OCCT.NET: BRepLib_FindSurface can return the plane of a face already built (the edges' pcurves
                    // lie on it), and OCCT reverses that shared plane in place, changing the other face too
                    // (occt-issues.md). The port reverses a copy (decision 929).
                    thePlane = (PlaneSurface)thePlane.Copy();
                }
                thePlane.UReverse();
            }
            LibraryMakeFace mkF = new(thePlane, ww);
            if (mkF.Error() != LibraryFaceError.FaceDone)
            {
            }
            else
            {
                Curve2D? nullC2d = null;
                Placement loc = new();
                bb.UpdateEdge(E1, nullC2d, s, loc, tol1);
                bb.UpdateEdge(E2, nullC2d, s, loc, tol2);
                bb.UpdateEdge(e3, nullC2d, s, loc, tol3);
                bb.UpdateEdge(e4, nullC2d, s, loc, tol4);

                f = mkF.InternalFace();
                UpdateEdgeOnPlane(f, E1, bb);
                UpdateEdgeOnPlane(f, E2, bb);
                UpdateEdgeOnPlane(f, e3, bb);
                UpdateEdgeOnPlane(f, e4, bb);
            }
        }

        if (!isPlan)
        { // Cas Standard : Ajout
            bb.MakeFace(ref f, s, global::Kerf.Engine.Foundation.Precision.Confusion());
            bb.Add(f, ww);
        }

        // Reorientation
        if (exchUV)
        {
            f.Reverse();
        }
        if (uReverse)
        {
            f.Reverse();
        }
    }

    //=================================================================================================

    private static Edge BuildEdge(Curve3D? c3d,
                                         Curve2D c2d,
                                         Surface s,
                                         in Vertex vf,
                                         in Vertex vl,
                                         double f,
                                         double l,
                                         double tol3d)
    {
        Point3D p;
        double tol1, tol2, tol, d;
        BrepBuilder b = new();
        Edge e = new();

        Point3D p1 = Tool.Pnt(vf);
        tol1 = Tool.Tolerance(vf);
        Point3D p2 = Tool.Pnt(vl);
        tol2 = Tool.Tolerance(vl);
        tol = Max(tol1, tol2);

        if (vf.IsSame(vl) || (p1.Distance(p2) < tol))
        {
            // Degenerated case
            c2d.D0(f, out Point2D p2d);
            s.D0(p2d.X(), p2d.Y(), out p);
            d = p1.Distance(p);
            if (d > tol)
            {
                tol = d;
            }
            c2d.D0(l, out p2d);
            s.D0(p2d.X(), p2d.Y(), out p);
            d = p2.Distance(p);
            if (d > tol)
            {
                tol = d;
            }

            b.UpdateVertex(vf, tol);
            b.UpdateVertex(vl, tol);

            b.MakeEdge(ref e);
            b.UpdateEdge(e, c2d, s, new Placement(), tol);
            b.Add(e, vf);
            b.Add(e, vl);
            b.Range(e, f, l);
            b.Degenerated(e, true);

            return e;
        }

        c3d!.D0(f, out p);
        d = p1.Distance(p);
        if (d > tol1)
        {
            b.UpdateVertex(vf, d);
        }

        c3d.D0(l, out p);
        d = p2.Distance(p);
        if (d > tol2)
        {
            b.UpdateVertex(vl, d);
        }

        LibraryMakeEdge mkE = new(c3d, vf, vl, f, l);
        if (!mkE.IsDone)
        { // Error of construction !!
        }

        e = mkE.InternalEdge();
        Placement loc = new();
        b.UpdateEdge(e, c2d, s, loc, tol3d);

        IntersectionContext? nullCtx = null;
        if (AlgoTools.IsMicroEdge(e, nullCtx))
        {
            Vertex v = vf;
            b.UpdateVertex(v, p1.Distance(p2));
            b.MakeEdge(ref e);
            b.UpdateEdge(e, c2d, s, new Placement(), tol);
            b.Add(e, ShapeCasts.Vertex(v.Oriented(Forward)));
            b.Add(e, ShapeCasts.Vertex(v.Oriented(Reversed)));
            b.Range(e, f, l);
            b.Degenerated(e, true);
        }

        return e;
    }

    //=======================================================================
    // Fonction : Filling
    // Objet : Construct the faces of filling
    //=======================================================================
    private static bool Filling(in Shape ef,
                                in Shape F1,
                                in Shape el,
                                in Shape F2,
                                Dictionary<Shape, Shape> eEmap,
                                double tol,
                                in Frame3D Axe,
                                in Vector3D tangentOnPart1,
                                ref Edge aux1,
                                ref Edge aux2,
                                ref Face result)
    {
        BrepBuilder b = new();
        bool withE3, withE4;

        // Return constraints
        Vertex V1, V2, vf, vl;
        Edge e1, e2, e3 = new(), e4 = new();
        e1 = ShapeCasts.Edge(ef);
        e2 = ShapeCasts.Edge(el);

        Exploration.Vertices(e1, out vf, out vl);
        vf.Orientation(Forward);
        vl.Orientation(Forward);

        Exploration.Vertices(e2, out V1, out V2);
        V1.Orientation(Reversed);
        V2.Orientation(Reversed);

        b.MakeEdge(ref e3);
        b.MakeEdge(ref e4);

        withE3 = withE4 = false;

        if ((!aux1.IsNull) && (!vf.IsSame(V1)))
        {
            e3 = aux1;
            withE3 = true;
        }

        if (vf.IsSame(vl))
        {
            e4 = e3;
            e4.Reverse();
            withE4 = withE3;
        }
        else if (!aux2.IsNull && (!vl.IsSame(V2)))
        {
            e4 = aux2;
            withE4 = true;
        }

        // Construction of a surface of revolution
        Curve3D prof1, prof2;
        double f1, f2, l1, l2, angle;
        prof1 = Tool.Curve(e1, out f1, out l1)!;
        prof2 = Tool.Curve(e2, out f2, out l2)!;

        // Indeed, both Prof1 and Prof2 are the same curves but in different positions
        // Prof1's param domain may equals to Prof2's param domain *(-1), which means EF.Orientation() ==
        // EL.Orientation()
        bool bSameCurveDomain = ef.Orientation() != el.Orientation();
        Point3D P1, P2, p;

        // Choose the angle of opening
        Transform3D tf = new Transform3D().WithTransformation(new CoordinateSystem(Axe));

        // Choose the furthest point from the "center of revolution"
        // to provide correct angle measurement.
        double[] prm = [f1, 0.5 * (f1 + l1), l1];
        Point3D[] p1 = [prof1.Value(prm[0]).Transformed(tf),
                        prof1.Value(prm[1]).Transformed(tf),
                        prof1.Value(prm[2]).Transformed(tf)];

        int maxIdx = -1;
        double maxDist = RealFirst();
        for (int i = 0; i < 3; i++)
        {
            double dist = p1[i].X() * p1[i].X() + p1[i].Z() * p1[i].Z();
            if (dist > maxDist)
            {
                maxDist = dist;
                maxIdx = i;
            }
        }

        double[] prm2 = [f2, 0.5 * (f2 + l2), l2];
        Point3D p2 = prof2.Value(prm2[bSameCurveDomain ? maxIdx : 2 - maxIdx]).Transformed(tf);
        Vector2D v1 = new(p1[maxIdx].Z(), p1[maxIdx].X());
        Vector2D v2 = new(p2.Z(), p2.X());
        if (v1.SquareMagnitude() <= Constants.Resolution() || v2.SquareMagnitude() <= Constants.Resolution())
        {
            return false;
        }

        angle = v1.Angle(v2);

        Axis3D axe = new(Axe.Location(), Axe.YDirection());

        if (angle < 0)
        {
            angle = -angle;
            axe = axe.Reversed();
        }

        RevolutionSurface rev = new(prof1, axe);

        Surface surf = new TrimmedSurface(rev, 0, angle, f1, l1);

        // Control the direction of the rotation
        bool toReverseResult = false;
        Vector3D d1u;
        d1u = surf.DN(0, prm[maxIdx], 1, 0);
        if (d1u.Angle(tangentOnPart1) > M_PI / 2)
        { // Invert everything
            toReverseResult = true;
        }

        Curve2D c1, c2, c4;
        Curve2D? c3 = null;
        LineCurve2D l;
        Point2D p2d = new(0.0, 0.0);

        l = new LineCurve2D(p2d, Constants.DY2d());
        c1 = new TrimmedCurve2D(l, f1, l1);

        p2d = p2d.WithCoord(angle, 0.0);
        l = new LineCurve2D(p2d, Constants.DY2d());
        c2 = new TrimmedCurve2D(l, f1, l1);

        // It is required to control the direction and the range.
        c2.D0(f1, out p2d);
        surf.D0(p2d.X(), p2d.Y(), out P1);
        c2.D0(l1, out p2d);
        surf.D0(p2d.X(), p2d.Y(), out P2);
        p = Tool.Pnt(V1);
        if (p.Distance(P2) + tol < p.Distance(P1))
        {
            // E2 is parsed in the direction opposite to E1
            c2.Reverse();
            Vertex aux;
            aux = V2;
            V2 = V1;
            V1 = aux;
        }
        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(),
                          c2,
                          c2.FirstParameter(),
                          c2.LastParameter(),
                          f2,
                          l2,
                          ref c3);
        c2 = c3!;

        p2d = p2d.WithCoord(0.0, f1);
        l = new LineCurve2D(p2d, Constants.DX2d());
        c3 = new TrimmedCurve2D(l, 0, angle);

        p2d = p2d.WithCoord(0.0, l1);
        l = new LineCurve2D(p2d, Constants.DX2d());
        c4 = new TrimmedCurve2D(l, 0, angle);
        // (OCCT's commented-out deformation by GeomPlate and its unused points are left out.)

        // Update des Edges
        Placement loc = new();
        Curve3D c3d;
        b.UpdateEdge(e1, c1, surf, loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
        b.UpdateEdge(e2, c2, surf, loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());

        if (e3.IsSame(e4))
        {
            if (!withE3)
            {
                c3d = surf.VIso(f1);
                e3 = BuildEdge(c3d, c3, surf, vf, V1, 0, angle, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                EdgeCurveAdaptor curve = new(e3);
                double angleOld = curve.LastParameter();
                if (angle > angleOld)
                {
                    b.Range(e3, 0, angle);
                    Vertex v = Exploration.LastVertex(e3);
                    BrepTVertex tVlast = (BrepTVertex)v.TShape()!;
                    tVlast.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                }
            }

            b.UpdateEdge(e3, c3, c4, surf, loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            e4 = e3;
            e4.Reverse();
        }
        else
        {
            if (!withE3)
            {
                c3d = surf.VIso(f1);
                e3 = BuildEdge(c3d, c3, surf, vf, V1, 0, angle, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                EdgeCurveAdaptor curve = new(e3);
                double angleOld = curve.LastParameter();
                if (angle > angleOld)
                {
                    b.Range(e3, 0, angle);
                    Vertex v = Exploration.LastVertex(e3);
                    BrepTVertex tVlast = (BrepTVertex)v.TShape()!;
                    tVlast.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                }

                b.UpdateEdge(e3, c3, surf, loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }

            if (!withE4)
            {
                c3d = surf.VIso(l1);
                e4 = BuildEdge(c3d, c4, surf, vl, V2, 0, angle, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                EdgeCurveAdaptor curve = new(e4);
                double angleOld = curve.LastParameter();
                if (angle > angleOld)
                {
                    b.Range(e4, 0, angle);
                    Vertex v = Exploration.LastVertex(e4);
                    BrepTVertex tVlast = (BrepTVertex)v.TShape()!;
                    tVlast.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                }

                b.UpdateEdge(e4, c4, surf, loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        // Construct face
        BuildFace(surf, e1, e3, e2, e4, eEmap, false, false, ref result);

        // Set the continuities.
        b.Continuity(e1, ShapeCasts.Face(F1), result, G1);
        b.Continuity(e2, ShapeCasts.Face(F2), result, G1);

        // Render the calculated borders.
        if (!Tool.Degenerated(e3))
        {
            aux1 = e3;
        }
        else
        {
            b.MakeEdge(ref aux1); // Nullify
        }

        if (!Tool.Degenerated(e4))
        {
            aux2 = e4;
        }
        else
        {
            b.MakeEdge(ref aux2);
        }

        // Set the orientation

        // Provide correct normals computation
        // (the normal will be computed not in
        // singularity point definitely).
        angle = RealFirst();
        for (int i = 0; i < 3; i++)
        {
            Vector3D d1U, d1V, n1, n2;
            c1.D0(prm[i], out p2d);
            surf.D1(p2d.X(), p2d.Y(), out p, out d1U, out d1V);
            n1 = d1U ^ d1V;

            if (n1.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                continue;
            }

            c1 = Tool.CurveOnSurface(e1, ShapeCasts.Face(F1), out f2, out l2)!;
            c1.D0(prm[i], out p2d);
            FaceSurfaceAdaptor AS = new(ShapeCasts.Face(F1));
            AS.D1(p2d.X(), p2d.Y(), out p, out d1U, out d1V);
            n2 = d1U ^ d1V;

            if (n2.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                continue;
            }

            angle = n1.Angle(n2);

            break;
        }

        if (angle == RealFirst())
        {
            return false;
        }

        if ((F1.Orientation() == Reversed) ^ (angle > M_PI / 2))
        {
            result.Orientation(Reversed);
        }
        else
        {
            result.Orientation(Forward);
        }

        if (toReverseResult)
        {
            result.Reverse();
        }

        return true;
    }

    //=================================================================================================

    private static void Substitute(Substitution aSubstitute, in Edge old, in Edge New)
    {
        NCollection_List<Shape> listShape = new();

        Exploration.Vertices(old, out Vertex oldV1, out Vertex oldV2);
        Exploration.Vertices(New, out Vertex newV1, out Vertex newV2);

        if (!aSubstitute.IsCopied(oldV1))
        {
            listShape.Append(newV1.Oriented(Forward));
            aSubstitute.Substitute(oldV1, listShape);
            listShape.Clear();
        }
        if (!aSubstitute.IsCopied(oldV2))
        {
            listShape.Append(newV2.Oriented(Forward));
            aSubstitute.Substitute(oldV2, listShape);
            listShape.Clear();
        }
        if (!aSubstitute.IsCopied(old))
        {
            listShape.Append(New.Oriented(Forward));
            aSubstitute.Substitute(old, listShape);
        }
    }

    //=======================================================================
    // Fonction : KeepEdge
    // Objet : Find edges of the face supported by the same Curve.
    //=======================================================================
    private static void KeepEdge(in Shape face, in Shape edge, NCollection_List<Shape> list)
    {
        list.Clear();
        Explorer exp = new(face, global::Kerf.Topology.ShapeType.Edge);
        Curve3D? cref, c;
        cref = Tool.Curve(ShapeCasts.Edge(edge), out Placement lref, out _, out _);

        for (; exp.More(); exp.Next())
        {
            c = Tool.Curve(ShapeCasts.Edge(exp.Current()), out Placement l, out _, out _);
            if (ReferenceEquals(cref, c) && (lref.IsEqual(l)))
            {
                list.Append(exp.Current());
            }
        }
    }

    //=================================================================================================

    private static void BuildVertex(Curve3D iso, bool isfirst, double first, double last, ref Shape vertex)
    {
        BrepBuilder b = new();
        double val;

        if (isfirst)
        {
            val = first;
        }
        else
        {
            val = last;
        }
        Vertex v = new();
        b.MakeVertex(ref v, iso.Value(val), global::Kerf.Engine.Foundation.Precision.Confusion());
        vertex = v;
    }

    //=================================================================================================

    private static Edge NullEdge(ref Shape vertex)
    {
        Edge e = new();
        BrepBuilder b = new();
        b.MakeEdge(ref e);
        vertex.Orientation(Forward);
        b.Add(e, vertex);
        b.Add(e, vertex.Reversed());
        b.Degenerated(e, true);
        return e;
    }

    //=================================================================================================

    private static Edge BuildEdge(Surface s,
                                         bool isUiso,
                                         double valIso,
                                         in Shape vFirst,
                                         in Shape vLast,
                                         double Tol)
    {
        Edge e;
        BrepBuilder b = new();
        Curve3D iso;
        bool sing = false;
        if (isUiso)
        {
            iso = s.UIso(valIso);
        }
        else
        {
            iso = s.VIso(valIso);
        }

        if (vFirst.IsSame(vLast))
        { // Singular case ?
            Vertex v = ShapeCasts.Vertex(vFirst);
            double tol = Tool.Tolerance(v);
            if (Tol > tol)
            {
                tol = Tol;
            }
            iso.D0((iso.FirstParameter() + iso.LastParameter()) / 2, out Point3D p);
            if (p.Distance(Tool.Pnt(v)) < tol)
            {
                CurveAdaptor ac = new(iso);
                sing = AbscissaPoint.Length(ac, tol / 4) < tol;
            }
        }

        if (sing)
        { // Singular case
            Shape v;
            v = vFirst;
            e = NullEdge(ref v);
            b.Degenerated(e, true);
        }

        else
        {
            // Construction Via 3d
            Point3D P1, P2;
            double p1, p2, p11, p12, p21, p22;
            bool fwd = false;
            p1 = iso.FirstParameter();
            p2 = iso.LastParameter();
            P1 = iso.Value(p1);
            P2 = iso.Value(p2);

            double t1 = Tool.Tolerance(ShapeCasts.Vertex(vFirst));
            double t2 = Tool.Tolerance(ShapeCasts.Vertex(vLast));

            BrepBuilder bb = new();

            p11 = P1.Distance(Tool.Pnt(ShapeCasts.Vertex(vFirst)));
            p22 = P2.Distance(Tool.Pnt(ShapeCasts.Vertex(vLast)));
            p12 = P1.Distance(Tool.Pnt(ShapeCasts.Vertex(vLast)));
            p21 = P2.Distance(Tool.Pnt(ShapeCasts.Vertex(vFirst)));

            if (p11 < p12 && p22 < p21)
            {
                fwd = true;
            }

            if (fwd)
            { // OCC500(apo)
                if (p11 >= t1)
                {
                    bb.UpdateVertex(ShapeCasts.Vertex(vFirst), 1.01 * p11);
                }
                if (p22 >= t2)
                {
                    bb.UpdateVertex(ShapeCasts.Vertex(vLast), 1.01 * p22);
                }
            }
            else
            {
                if (p12 >= t2)
                {
                    bb.UpdateVertex(ShapeCasts.Vertex(vLast), 1.01 * p12);
                }
                if (p21 >= t1)
                {
                    bb.UpdateVertex(ShapeCasts.Vertex(vFirst), 1.01 * p21);
                }
            }

            LibraryMakeEdge mkE = new();

            if (fwd)
            {
                mkE.Init(iso,
                         ShapeCasts.Vertex(vFirst),
                         ShapeCasts.Vertex(vLast),
                         iso.FirstParameter(),
                         iso.LastParameter());
            }
            else
            {
                mkE.Init(iso,
                         ShapeCasts.Vertex(vLast),
                         ShapeCasts.Vertex(vFirst),
                         iso.FirstParameter(),
                         iso.LastParameter());
            }

            if (!mkE.IsDone)
            { // Erreur de construction !!
                throw new ConstructionErrorException("BRepFill_Sweep::BuildEdge");
            }

            e = mkE.InternalEdge();
        }

        // Associate 2d
        LineCurve2D l;
        Placement loc = new();
        s.Bounds(out double umin, out _, out double vmin, out _);
        if (isUiso)
        {
            Point2D p = new(valIso, vmin - iso.FirstParameter());
            Vector2D v = new(0.0, 1.0);
            l = new LineCurve2D(p, new Direction2D(v));
        }
        else
        {
            Point2D p = new(umin - iso.FirstParameter(), valIso);
            Vector2D v = new(1.0, 0.0);
            l = new LineCurve2D(p, new Direction2D(v));
        }

        b.UpdateEdge(e, l, s, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (sing)
        {
            b.Range(e, s, loc, iso.FirstParameter(), iso.LastParameter());
        }

        double maxTol = 1.0e-4;
        CurveAdaptor gAiso = new(iso);
        CurveAdaptor gaHiso = gAiso;
        SurfaceAdaptor gAsurf = new(s);
        SurfaceAdaptor gaHsurf = gAsurf;
        CheckSameParameter(gaHiso, l, gaHsurf, maxTol, out double theTol);
        b.UpdateEdge(e, theTol);

        return e;
    }

    //=================================================================================================

    private static void UpdateEdge(in Edge E, Surface s, bool isUiso, double valIso)
    {
        BrepBuilder b = new();
        LineCurve2D l;
        Curve2D? pCurve;
        Curve2D cl;
        Placement loc = new();
        double f2d, l2d;
        s.Bounds(out double uFirst, out double uLast, out double vFirst, out double vLast);

        bool sing = false;
        Curve3D iso;
        if (isUiso)
        {
            iso = s.UIso(valIso);
        }
        else
        {
            iso = s.VIso(valIso);
        }

        Exploration.Vertices(E, out Vertex vf, out Vertex vl);
        if (vf.IsSame(vl))
        { // Singular case ?
            double tol = Tool.Tolerance(vf);
            iso.D0((iso.FirstParameter() + iso.LastParameter()) / 2, out Point3D pmid);
            if (pmid.Distance(Tool.Pnt(vf)) < tol)
            {
                CurveAdaptor ac = new(iso);
                sing = AbscissaPoint.Length(ac, tol / 4) < tol;
            }
        }

        if (isUiso)
        {
            Point2D p = new(valIso, 0);
            Vector2D v = new(0.0, 1.0);
            l = new LineCurve2D(p, new Direction2D(v));
            f2d = vFirst;
            l2d = vLast;
        }
        else
        {
            Point2D p = new(0.0, valIso);
            Vector2D v = new(1.0, 0.0);
            l = new LineCurve2D(p, new Direction2D(v));
            f2d = uFirst;
            l2d = uLast;
        }
        cl = new TrimmedCurve2D(l, f2d, l2d);

        // Control direction & Range
        double r, first, last, Tol = 1.0e-4;
        bool reverse = false;

        Point3D pOnS;
        Point2D p2d;
        Tool.Range(E, out first, out last);

        if (!vf.IsSame(vl))
        {
            // Test distances between "FirstPoint" and "Vertex"
            p2d = cl.Value(f2d);
            pOnS = s.Value(p2d.X(), p2d.Y());
            reverse = pOnS.Distance(Tool.Pnt(vl)) < pOnS.Distance(Tool.Pnt(vf));
        }
        else if (!sing)
        {
            // Test angle between "First Tangente"
            Vector3D v3d = new();
            EdgeCurveAdaptor c3d = new(E);

            c3d.D1(first, out pOnS, out Vector3D dC3d);
            cl.D1(f2d, out p2d, out Vector2D v2d);
            s.D1(p2d.X(), p2d.Y(), out pOnS, out Vector3D du, out Vector3D dv);
            v3d = Vector3D.LinearForm(v2d.X(), du, v2d.Y(), dv);
            reverse = (dC3d.Angle(v3d) > Tol);
        }
        if (reverse)
        { // Return curve 2d
            cl = new TrimmedCurve2D(l, f2d, l2d);
            cl.Reverse();
            f2d = cl.FirstParameter();
            l2d = cl.LastParameter();
        }

        if (sing)
        {
            Curve3D? nullCurve = null;
            b.UpdateEdge(E, nullCurve, 0.0);
            b.Degenerated(E, true);
            b.Range(E, f2d, l2d);
            first = f2d;
            last = l2d;
        }

        if (first != f2d || last != l2d)
        {
            Curve2D? c2d = null;
            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(), cl, f2d, l2d, first, last, ref c2d);
            cl = new TrimmedCurve2D(c2d!, first, last);
        }

        // Update des Vertex

        Vertex v2;

        p2d = cl.Value(first);
        pOnS = s.Value(p2d.X(), p2d.Y());
        v2 = Exploration.FirstVertex(E);
        r = pOnS.Distance(Tool.Pnt(v2));
        b.UpdateVertex(v2, r);

        p2d = cl.Value(last);
        pOnS = s.Value(p2d.X(), p2d.Y());
        v2 = Exploration.LastVertex(E);
        r = pOnS.Distance(Tool.Pnt(v2));
        b.UpdateVertex(v2, r);

        // Update Edge
        if (!sing && SameParameter(E, ref cl, s, Tol, out r))
        {
            b.UpdateEdge(E, r);
        }

        pCurve = Couture(E, s, loc);
        if (pCurve is null)
        {
            b.UpdateEdge(E, cl, s, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        { // Sewing edge
            Edge e = E;
            Oriente(s, ref e);
            if (e.Orientation() == Reversed)
            {
                b.UpdateEdge(E, cl, pCurve, s, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                b.UpdateEdge(E, pCurve, cl, s, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        // Attention to case not SameRange on its shapes (PRO13551)
        if (!Tool.SameRange(E))
        {
            b.Range(E, s, loc, first, last);
        }
    }

    //=======================================================================
    // Object : Check if a surface is degenerated
    //=======================================================================
    private static bool IsDegen(Surface s, double tol)
    {
        int nb = 5;
        bool b = true;
        double t, dt, l;
        int ii;
        Curve3D iso;
        Point3D p1, p2, p3;

        s.Bounds(out double umin, out double umax, out double vmin, out double vmax);

        // Check the length of Iso-U
        t = (umin + umax) / 2;
        s.D0(t, vmin, out p1);
        s.D0(t, (vmin + vmax) / 2, out p2);
        s.D0(t, vmax, out p3);
        b = ((p1.Distance(p2) + p2.Distance(p3)) < tol);

        for (ii = 1, dt = (umax - umin) / (nb + 1); b && (ii <= nb); ii++)
        {
            t = umin + ii * dt;
            iso = s.UIso(t);
            CurveAdaptor ac = new(iso);
            l = AbscissaPoint.Length(ac, tol / 4);
            b = (l <= tol);
        }

        if (b)
        {
            return true;
        }

        // Check the length of Iso-V
        t = (vmin + vmax) / 2;
        s.D0(umin, t, out p1);
        s.D0((umin + umax) / 2, t, out p2);
        s.D0(umax, t, out p3);
        b = ((p1.Distance(p2) + p2.Distance(p3)) < tol);

        for (ii = 1, dt = (vmax - vmin) / (nb + 1); b && (ii <= nb); ii++)
        {
            t = vmin + ii * dt;
            iso = s.VIso(t);
            CurveAdaptor ac = new(iso);
            l = AbscissaPoint.Length(ac, tol / 4);
            b = (l <= tol);
        }

        return b;
    }

    private static void ReverseEdgeInFirstOrLastWire(in Shape wire, in Shape edge)
    {
        Shape edgeToReverse = new();
        Iterator itw = new(wire);

        for (; itw.More(); itw.Next())
        {
            Shape anEdge = itw.Value();
            if (anEdge.IsSame(edge))
            {
                edgeToReverse = anEdge;
                break;
            }
        }

        if (!edgeToReverse.IsNull)
        {
            BrepBuilder bb = new();
            wire.Free(true);
            bb.Remove(wire, edgeToReverse);
            edgeToReverse.Reverse();
            bb.Add(wire, edgeToReverse);
        }
    }

    private static void ReverseModifiedEdges(in Wire wire, HashSet<Shape> emap)
    {
        if (emap.Count == 0)
        {
            return;
        }

        Iterator itw = new(wire);
        BrepBuilder bb = new();

        NCollection_List<Shape> ledges = new();
        for (; itw.More(); itw.Next())
        {
            ledges.Append(itw.Value());
        }

        wire.Free(true);
        NCollection_List<Shape>.Iterator itl = new(ledges);
        for (; itl.More(); itl.Next())
        {
            bb.Remove(wire, itl.Value());
        }

        for (itl.Initialize(ledges); itl.More(); itl.Next())
        {
            Shape edge = itl.Value();
            if (emap.Contains(edge))
            {
                edge.Reverse();
            }
            bb.Add(wire, edge);
        }
    }

    //=================================================================================================

    /// <summary>The sweep of <paramref name="section"/> along <paramref name="location"/>.</summary>
    public Sweep(SectionLaw section, LocationLaw location, bool withKPart)
    {
        _isDone = false;
        _kPart = withKPart;
        _sec = section;
        _loc = location;

        SetTolerance(1.0e-4);
        SetAngularControl();
        _auxShape.Clear();

        _approxStyle = ApproxStyle.Location;
        _continuity = C2;
        _degmax = 11;
        _segmax = 30;
        _forceApproxC1 = false;
    }

    //=================================================================================================

    /// <summary>Sets the wires bounding the sweep at its start and end (null wires for none).</summary>
    public void SetBounds(in Wire first, in Wire last)
    {
        _firstShape = first;
        _lastShape = last;

        // It is necessary to check the SameRange on its (PRO13551)
        BrepBuilder b = new();
        WireExplorer wexp = new();
        if (!_firstShape.IsNull)
        {
            for (wexp.Init(_firstShape); wexp.More(); wexp.Next())
            {
                if (!BrepLibrary.CheckSameRange(wexp.Current()))
                {
                    b.SameRange(wexp.Current(), false);
                    b.SameParameter(wexp.Current(), false);
                }
            }
        }

        if (!_lastShape.IsNull)
        {
            for (wexp.Init(_lastShape); wexp.More(); wexp.Next())
            {
                if (!BrepLibrary.CheckSameRange(wexp.Current()))
                {
                    b.SameRange(wexp.Current(), false);
                    b.SameParameter(wexp.Current(), false);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Sets the approximation tolerances: <paramref name="tol3d"/> for the surfaces, <paramref name="tol2d"/> for the
    /// curves, <paramref name="tolAngular"/> (radians) between the section law's tangents and the iso-v's.
    /// </summary>
    public void SetTolerance(double tol3d, double boundTol = 1.0, double tol2d = 1.0e-5, double tolAngular = 1.0e-2)
    {
        _tol3d = tol3d;
        _boundTol = boundTol;
        _tol2d = tol2d;
        _tolAngular = tolAngular;
    }

    //=================================================================================================

    /// <summary>
    /// The control of the corners: below <paramref name="minAngle"/> (radians) the transition is always "Modified".
    /// </summary>
    public void SetAngularControl(double minAngle = 0.01, double maxAngle = 6.0)
    {
        _angMin = Max(minAngle, global::Kerf.Engine.Foundation.Precision.Angular());
        _angMax = Min(maxAngle, 6.28);
    }

    //=======================================================================
    // function : SetForceApproxC1
    // purpose  : Set the flag that indicates attempt to approximate
    //           a C1-continuous surface if a swept surface proved
    //           to be C0.
    //=======================================================================
    /// <summary>Tries to approximate C1 surfaces where a swept surface is C0.</summary>
    public void SetForceApproxC1(bool forceApproxC1)
    {
        _forceApproxC1 = forceApproxC1;
    }

    ///=================================================================================================

    private bool CorrectApproxParameters()
    {
        Wire thePath = _loc.Wire();
        Continuity newCont = _continuity;
        int newSegmax = _segmax;

        Iterator iter = new(thePath);
        for (; iter.More(); iter.Next())
        {
            Edge edge = ShapeCasts.Edge(iter.Value());
            EdgeCurveAdaptor bAcurve = new(edge);
            Continuity continuity = bAcurve.Continuity();
            int nbInterv = bAcurve.NbIntervals(CN);
            if (continuity < newCont)
            {
                newCont = continuity;
            }
            if (nbInterv > newSegmax)
            {
                newSegmax = nbInterv;
            }
        }

        bool corrected = false;
        if (newCont != _continuity || newSegmax != _segmax)
        {
            corrected = true;
        }
        _continuity = newCont;
        _segmax = newSegmax;
        return corrected;
    }

    //=================================================================================================

    private bool BuildWire(TransitionStyle transition)
    {
        _ = transition;
        int ipath, isec = 1;
        Point3D p1;

        BrepBuilder b = new();
        int nbPath = _loc.LawCount;
        bool vclose;
        vclose = (_loc.IsClosed() && (_loc.IsG1(0, _tol3d) >= 0));
        _error = 0.0;
        Surface s;
        Curve3D iso;
        double val, bid, first, last, tol;

        Wire wire = new();
        Edge e;
        b.MakeWire(ref wire);

        // (1) Construction of all curves

        // (1.1) Construction of Tables
        _faces = new NCollection_HArray2<Shape>(1, 1, 1, nbPath);
        _uEdges = new NCollection_HArray2<Shape>(1, 2, 1, nbPath);
        _vEdges = new NCollection_HArray2<Shape>(1, 1, 1, nbPath + 1);

        // (1.2) Calculate curves / vertex / edge
        for (ipath = 1; ipath <= nbPath; ipath++)
        {
            // Curve by iso value
            SurfaceSweep sweep = new(_loc.Law(ipath), _kPart);
            sweep.SetTolerance(_tol3d, _boundTol, _tol2d, _tolAngular);
            sweep.SetForceApproxC1(_forceApproxC1);
            sweep.Build(_sec.Law(isec), _approxStyle, _continuity, _degmax, _segmax);
            if (!sweep.IsDone)
            {
                return false;
            }
            s = sweep.InternalSurface()!;
            if (sweep.ExchangeUV())
            {
                if (sweep.UReversed())
                {
                    s.Bounds(out first, out last, out bid, out val);
                }
                else
                {
                    s.Bounds(out first, out last, out val, out bid);
                }
                iso = s.VIso(val);
            }
            else
            {
                if (sweep.UReversed())
                {
                    s.Bounds(out bid, out val, out first, out last);
                }
                else
                {
                    s.Bounds(out val, out bid, out first, out last);
                }
                iso = s.UIso(val);
            }
            // Vertex by position
            if (ipath < nbPath)
            {
                BuildVertex(iso, false, first, last, ref _vEdges.ChangeValue(1, ipath + 1));
            }
            else
            {
                if (vclose)
                {
                    Vertex v = ShapeCasts.Vertex(_vEdges.Value(1, 1));
                    _vEdges.SetValue(1, ipath + 1, v);
                    iso.D0(last, out p1);
                    tol = p1.Distance(Tool.Pnt(v));
                    b.UpdateVertex(v, tol);
                }
                else
                {
                    if (!_lastShape.IsNull)
                    {
                        _vEdges.SetValue(1, nbPath, _firstShape); // OCCT: FirstShape (a wire) at NbPath, not the last vertex
                    }
                    else
                    {
                        BuildVertex(iso, false, first, last, ref _vEdges.ChangeValue(1, nbPath + 1));
                    }
                }
            }

            if (ipath > 1)
            {
                iso.D0(first, out p1);
                Vertex v = ShapeCasts.Vertex(_vEdges.Value(1, ipath));
                tol = p1.Distance(Tool.Pnt(v));
                b.UpdateVertex(v, tol);
            }
            if (ipath == 1)
            {
                if (!_firstShape.IsNull)
                {
                    _vEdges.SetValue(1, 1, _firstShape);
                }
                else
                {
                    BuildVertex(iso, true, first, last, ref _vEdges.ChangeValue(1, 1));
                }
            }

            // Construction of the edge
            LibraryMakeEdge mkE = new();
            mkE.Init(iso,
                     ShapeCasts.Vertex(_vEdges.Value(1, ipath)),
                     ShapeCasts.Vertex(_vEdges.Value(1, ipath + 1)),
                     iso.FirstParameter(),
                     iso.LastParameter());
            if (!mkE.IsDone)
            { // Error of construction !!
                return false;
            }
            e = mkE.InternalEdge();
            b.UpdateEdge(e, sweep.ErrorOnSurface());
            b.Add(wire, e);
            _faces.SetValue(1, ipath, e);
        }
        _shape = wire;
        return true;
    }

    //=================================================================================================

    private bool BuildShell(TransitionStyle transition,
                            int iFirst,
                            int iLast,
                            HashSet<Shape> reversedEdges,
                            Dictionary<Shape, NCollection_HArray2<Shape>> tapes,
                            Dictionary<Shape, NCollection_HArray2<Shape>> rails,
                            double extendFirst = 0.0,
                            double extendLast = 0.0)
    {
        _ = transition;
        int ipath, isec, iPath;
        BrepBuilder b = new();
        int nbPath = iLast - iFirst;
        int nbLaw = _sec.LawCount;
        bool uclose, vclose, global_vclose, constSection, hasdegen = false;
        constSection = _sec.IsConstant;
        uclose = _sec.IsUClosed;
        global_vclose = (_loc.IsClosed()) && (_loc.IsG1(0, _tol3d) >= 0);
        vclose = global_vclose && (_sec.IsVClosed) && (nbPath == _loc.LawCount);
        _error = 0.0;

        // (1) Construction of all surfaces

        // (1.1) Construction of Tables

        NCollection_Array2<bool> exchUV = new(1, nbLaw, 1, nbPath);
        NCollection_Array2<bool> uReverse = new(1, nbLaw, 1, nbPath);
        NCollection_Array2<bool> degenerated = new(1, nbLaw, 1, nbPath);
        degenerated.Init(false);
        // No VReverse for the moment...
        NCollection_Array2<double> tabErr = new(1, nbLaw, 1, nbPath);
        NCollection_Array2<Surface?> tabS = new(1, nbLaw, 1, nbPath);

        NCollection_Array2<Shape> uEdge = new(1, nbLaw + 1, 1, nbPath);
        NCollection_Array2<Shape> vEdge = new(1, nbLaw, 1, nbPath + 1);
        NCollection_Array2<Shape> vertex = new(1, nbLaw + 1, 1, nbPath + 1);

        Vertex vnull = new();
        vnull.Nullify();
        vertex.Init(vnull);

        NCollection_Array1<Shape> secVertex = new(1, nbLaw + 1);
        NCollection_Array1<double> vError = new(1, nbLaw + 1);
        NCollection_Array1<double> vi = new(1, nbPath + 1);

        // Initialization of management of parametric intervals
        //(Case of evolutionary sections)
        double length, secDom, secDeb;
        _loc.CurvilinearBounds(_loc.LawCount, out secDom, out length);
        _sec.Law(1).GetDomain(out secDeb, out secDom);
        secDom -= secDeb;
        if (iFirst > 1)
        {
            _loc.CurvilinearBounds(iFirst - 1, out _, out double Ll);
            vi[1] = secDeb + (Ll / length) * secDom;
        }
        else
        {
            vi[1] = secDeb;
        }

        // Error a priori on vertices
        if (constSection)
        {
            for (isec = 1; isec <= nbLaw + 1; isec++)
            {
                vError[isec] = _sec.VertexTol(isec - 1, 0.0);
                secVertex[isec] = _sec.Vertex(isec, 0.0);
            }
        }

        // (1.2) Calculate surfaces
        for (ipath = 1, iPath = iFirst; ipath <= nbPath; ipath++, iPath++)
        {
            SurfaceSweep sweep = new(_loc.Law(iPath), _kPart);
            sweep.SetTolerance(_tol3d, _boundTol, _tol2d, _tolAngular);
            sweep.SetForceApproxC1(_forceApproxC1);

            // Case of evolutionary section, definition of parametric correspondence
            if (!constSection)
            {
                _loc.Law(iPath).GetDomain(out double lf, out double ll);
                _loc.CurvilinearBounds(iPath, out _, out double Ll);
                vi[ipath + 1] = secDeb + (Ll / length) * secDom;
                sweep.SetDomain(lf, ll, vi[ipath], vi[ipath + 1]);
            }
            else // section is constant
            {
                _loc.Law(iPath).GetDomain(out _, out _);
                _loc.CurvilinearBounds(iPath, out _, out double Ll);
                vi[ipath + 1] = secDeb + (Ll / length) * secDom;
            }

            for (isec = 1; isec <= nbLaw; isec++)
            {
                sweep.Build(_sec.Law(isec), _approxStyle, _continuity, _degmax, _segmax);
                if (!sweep.IsDone)
                {
                    return false;
                }
                tabS[isec, ipath] = sweep.InternalSurface();
                tabErr[isec, ipath] = sweep.ErrorOnSurface();
                exchUV[isec, ipath] = sweep.ExchangeUV();
                uReverse[isec, ipath] = sweep.UReversed();
                if (sweep.ErrorOnSurface() > _error)
                {
                    _error = sweep.ErrorOnSurface();
                }

                if ((ipath == 1) && (extendFirst > 0))
                {
                    BoundedSurface bndS;
                    bndS = (BoundedSurface)tabS[isec, ipath]!;
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref bndS, extendFirst, 1, sweep.ExchangeUV(), false);
                    tabS[isec, ipath] = bndS;
                }
                if ((ipath == nbPath) && (extendLast > 0))
                {
                    BoundedSurface bndS;
                    bndS = (BoundedSurface)tabS[isec, ipath]!;
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref bndS, extendLast, 1, sweep.ExchangeUV(), true);
                    tabS[isec, ipath] = bndS;
                }
            }
        }

        // (2) Construction of Edges
        double uFirst, uLast, vFirst, vLast;
        bool exuv, singu, singv;
        Surface s;

        // Correct <FirstShape> and <LastShape>: reverse modified edges
        ReverseModifiedEdges(_firstShape, reversedEdges);
        ReverseModifiedEdges(_lastShape, reversedEdges);

        // (2.0) return preexisting Edges and vertices
        Edge e;
        NCollection_Array1<bool> isBuilt = new(1, nbLaw);
        isBuilt.Init(false);
        NCollection_Array1<Shape> startEdges = new(1, nbLaw);
        if (!_firstShape.IsNull && (iFirst == 1))
        {
            _sec.Init(_firstShape);
            for (isec = 1; isec <= nbLaw; isec++)
            {
                e = _sec.CurrentEdge();
                Exploration.Vertices(e, out Vertex vfirst, out Vertex vlast);
                vEdge[isec, 1] = e;
                if (e.Orientation() == Reversed)
                {
                    vertex[isec + 1, 1] = vfirst; // TopExp::FirstVertex(E);
                }
                else
                {
                    vertex[isec + 1, 1] = vlast; // TopExp::LastVertex(E);
                }
                UpdateVertex(iFirst - 1, isec + 1, tabErr[isec, 1], vi[1], vertex[isec + 1, 1]);

                startEdges[isec] = e;
                if (tapes.ContainsKey(e))
                {
                    isBuilt[isec] = true;

                    // Initialize VEdge, UEdge, Vertex and myFaces
                    int j;
                    NCollection_HArray2<Shape> tape = tapes.GetValue(e);
                    for (j = 1; j <= nbPath + 1; j++)
                    {
                        vEdge[isec, j] = tape.Value(1, j);
                        vEdge[isec, j].Reverse(); // direction of round is reversed
                    }
                    int ifirst = isec + 1, ilast = isec; // direction of round is reversed
                    for (j = 1; j <= nbPath; j++)
                    {
                        uEdge[ifirst, j] = tape.Value(2, j);
                    }
                    for (j = 1; j <= nbPath; j++)
                    {
                        uEdge[ilast, j] = tape.Value(3, j);
                    }
                    for (j = 1; j <= nbPath + 1; j++)
                    {
                        vertex[ifirst, j] = tape.Value(4, j);
                    }
                    for (j = 1; j <= nbPath + 1; j++)
                    {
                        vertex[ilast, j] = tape.Value(5, j);
                    }
                    for (j = 1; j <= nbPath; j++)
                    {
                        _faces!.SetValue(isec, j, tape.Value(6, j));
                    }

                    if (uclose && isec == 1)
                    {
                        for (j = 1; j <= nbPath; j++)
                        {
                            uEdge[nbLaw + 1, j] = uEdge[1, j];
                        }
                        for (j = 1; j <= nbPath + 1; j++)
                        {
                            vertex[nbLaw + 1, j] = vertex[1, j];
                        }
                    }
                    if (uclose && isec == nbLaw)
                    {
                        for (j = 1; j <= nbPath; j++)
                        {
                            uEdge[1, j] = uEdge[nbLaw + 1, j];
                        }
                        for (j = 1; j <= nbPath + 1; j++)
                        {
                            vertex[1, j] = vertex[nbLaw + 1, j];
                        }
                    }
                }
                else
                {
                    NCollection_HArray2<Shape> emptyArray = new(1, 6, 1, nbPath + 1);
                    tapes[e] = emptyArray;
                    int j;
                    if (rails.ContainsKey(vfirst))
                    {
                        int ind = (e.Orientation() == Reversed) ? isec + 1 : isec;
                        NCollection_HArray2<Shape> rail = rails.GetValue(vfirst);
                        for (j = 1; j <= nbPath; j++)
                        {
                            uEdge[ind, j] = rail.Value(1, j);
                        }
                        for (j = 1; j <= nbPath + 1; j++)
                        {
                            vertex[ind, j] = rail.Value(2, j);
                        }
                    }
                    if (rails.ContainsKey(vlast))
                    {
                        int ind = (e.Orientation() == Forward) ? isec + 1 : isec;
                        NCollection_HArray2<Shape> rail = rails.GetValue(vlast);
                        for (j = 1; j <= nbPath; j++)
                        {
                            uEdge[ind, j] = rail.Value(1, j);
                        }
                        for (j = 1; j <= nbPath + 1; j++)
                        {
                            vertex[ind, j] = rail.Value(2, j);
                        }
                    }
                }
            }

            if (vEdge[1, 1].Orientation() == Reversed)
            {
                vertex[1, 1] = Exploration.LastVertex(ShapeCasts.Edge(vEdge[1, 1]));
            }
            else
            {
                vertex[1, 1] = Exploration.FirstVertex(ShapeCasts.Edge(vEdge[1, 1]));
            }
            UpdateVertex(iFirst - 1, 1, tabErr[1, 1], vi[1], vertex[1, 1]);
        }

        double u, v, aux;
        bool ureverse;
        for (isec = 1; isec <= nbLaw + 1; isec++)
        {
            // Return data
            if (isec > nbLaw)
            {
                s = tabS[nbLaw, 1]!;
                ureverse = uReverse[nbLaw, 1];
                exuv = exchUV[nbLaw, 1];
            }
            else
            {
                s = tabS[isec, 1]!;
                ureverse = uReverse[isec, 1];
                exuv = exchUV[isec, 1];
            }
            s.Bounds(out uFirst, out uLast, out vFirst, out vLast);

            // Choice of parameters
            if (ureverse)
            {
                if (exuv)
                {
                    aux = vFirst;
                    vFirst = vLast;
                    vLast = aux;
                }
                else
                {
                    aux = uFirst;
                    uFirst = uLast;
                    uLast = aux;
                }
            }
            if (isec != nbLaw + 1)
            {
                u = uFirst;
                v = vFirst;
            }
            else
            {
                if (exuv)
                {
                    u = uFirst;
                    v = vLast;
                }
                else
                {
                    u = uLast;
                    v = vFirst;
                }
            }

            // construction of vertices
            if (vertex[isec, 1].IsNull)
            {
                Vertex aV = new();
                b.MakeVertex(ref aV, s.Value(u, v), _sec.VertexTol(isec - 1, vi[1]));
                vertex[isec, 1] = aV;
            }
            else
            {
                Placement identity = new();
                vertex[isec, 1].Location(identity);
                b.UpdateVertex(ShapeCasts.Vertex(vertex[isec, 1]), s.Value(u, v), _sec.VertexTol(isec - 1, vi[1]));
            }
        } // end of for (isec=1; isec<=NbLaw+1; isec++)

        if (!_lastShape.IsNull && (iLast == _loc.LawCount + 1))
        {
            _sec.Init(_lastShape);
            for (isec = 1; isec <= nbLaw; isec++)
            {
                e = _sec.CurrentEdge();
                if (vEdge[isec, nbPath + 1].IsNull)
                {
                    vEdge[isec, nbPath + 1] = e;
                }

                if (vertex[isec + 1, nbPath + 1].IsNull)
                {
                    if (vEdge[isec, nbPath + 1].Orientation() == Reversed)
                    {
                        vertex[isec + 1, nbPath + 1] = Exploration.FirstVertex(ShapeCasts.Edge(vEdge[isec, nbPath + 1]));
                    }
                    else
                    {
                        vertex[isec + 1, nbPath + 1] = Exploration.LastVertex(ShapeCasts.Edge(vEdge[isec, nbPath + 1]));
                    }
                }
                UpdateVertex(iLast - 1, isec + 1, tabErr[isec, nbPath], vi[nbPath + 1], vertex[isec + 1, nbPath + 1]);
            }

            if (vertex[1, nbPath + 1].IsNull)
            {
                if (vEdge[1, nbPath + 1].Orientation() == Reversed)
                {
                    vertex[1, nbPath + 1] = Exploration.LastVertex(ShapeCasts.Edge(vEdge[1, nbPath + 1]));
                }
                else
                {
                    vertex[1, nbPath + 1] = Exploration.FirstVertex(ShapeCasts.Edge(vEdge[1, nbPath + 1]));
                }
            }
            UpdateVertex(iLast - 1, 1, tabErr[1, nbPath], vi[nbPath + 1], vertex[1, nbPath + 1]);
        }

        for (isec = 1; isec <= nbLaw + 1; isec++)
        {
            // Return data
            if (isec > nbLaw)
            {
                s = tabS[nbLaw, nbPath]!;
                ureverse = uReverse[nbLaw, nbPath];
                exuv = exchUV[nbLaw, nbPath];
            }
            else
            {
                s = tabS[isec, nbPath]!;
                ureverse = uReverse[isec, nbPath];
                exuv = exchUV[isec, nbPath];
            }
            s.Bounds(out uFirst, out uLast, out vFirst, out vLast);

            // Choice of parametres
            if (ureverse)
            {
                if (exuv)
                {
                    aux = vFirst;
                    vFirst = vLast;
                    vLast = aux;
                }
                else
                {
                    aux = uFirst;
                    uFirst = uLast;
                    uLast = aux;
                }
            }
            if (isec == nbLaw + 1)
            {
                u = uLast;
                v = vLast;
            }
            else
            {
                if (exuv)
                {
                    u = uLast;
                    v = vFirst;
                }
                else
                {
                    u = uFirst;
                    v = vLast;
                }
            }

            // construction of vertex
            if (vertex[isec, nbPath + 1].IsNull)
            {
                Vertex aV = new();
                b.MakeVertex(ref aV, s.Value(u, v), _sec.VertexTol(isec - 1, vi[nbPath + 1]));
                vertex[isec, nbPath + 1] = aV;
            }
            else
            {
                Placement identity = new();
                vertex[isec, nbPath + 1].Location(identity);
                b.UpdateVertex(ShapeCasts.Vertex(vertex[isec, nbPath + 1]),
                               s.Value(u, v),
                               _sec.VertexTol(isec - 1, vi[nbPath + 1]));
            }
        } // end of for (isec=1; isec<=NbLaw+1; isec++)

        // ---------- Creation of Vertex and edge ------------
        for (ipath = 1, iPath = iFirst; ipath <= nbPath; ipath++, iPath++)
        {
            for (isec = 1; isec <= nbLaw; isec++)
            {
                if (isBuilt[isec])
                {
                    continue;
                }

                s = tabS[isec, ipath]!;
                exuv = exchUV[isec, ipath];
                s.Bounds(out uFirst, out uLast, out vFirst, out vLast);
                if (uReverse[isec, ipath])
                {
                    double aux2;
                    if (exuv)
                    {
                        aux2 = vFirst;
                        vFirst = vLast;
                        vLast = aux2;
                    }
                    else
                    {
                        aux2 = uFirst;
                        uFirst = uLast;
                        uLast = aux2;
                    }
                }

                // (2.1) Construction of new vertices
                if (isec == 1)
                {
                    if (ipath == 1 && vertex[1, 1].IsNull)
                    {
                        // All first
                        Vertex aV;
                        if (constSection)
                        {
                            _loc.InternalPerformVertex(iPath - 1, ShapeCasts.Vertex(secVertex[1]), vError[1], out aV);
                        }
                        else
                        {
                            _loc.InternalPerformVertex(iPath - 1,
                                                        _sec.Vertex(1, vi[1]),
                                                        _sec.VertexTol(0, vi[1]),
                                                        out aV);
                        }
                        vertex[1, 1] = aV;
                    }
                    // the first and the next column
                    if (vclose && (ipath == nbPath))
                    {
                        vertex[1, ipath + 1] = vertex[1, 1];
                    }
                    else if (vertex[1, ipath + 1].IsNull)
                    {
                        Vertex aV;
                        if (constSection)
                        {
                            _loc.InternalPerformVertex(iPath,
                                                        ShapeCasts.Vertex(secVertex[1]),
                                                        tabErr[1, ipath] + vError[1],
                                                        out aV);
                        }
                        else
                        {
                            _loc.InternalPerformVertex(iPath,
                                                        _sec.Vertex(1, vi[ipath + 1]),
                                                        tabErr[1, ipath] + _sec.VertexTol(0, vi[ipath + 1]),
                                                        out aV);
                        }
                        vertex[1, ipath + 1] = aV;

                        if (MergeVertex(vertex[1, ipath], ref vertex[1, ipath + 1]))
                        {
                            uEdge[1, ipath] = NullEdge(ref vertex[1, ipath]);
                        }
                    }
                }

                if (ipath == 1)
                {
                    if (uclose && (isec == nbLaw))
                    {
                        vertex[isec + 1, 1] = vertex[1, 1];
                    }
                    else if (vertex[isec + 1, 1].IsNull)
                    {
                        Vertex aV;
                        if (constSection)
                        {
                            _loc.InternalPerformVertex(iPath - 1,
                                                        ShapeCasts.Vertex(secVertex[isec + 1]),
                                                        tabErr[isec, 1] + vError[isec + 1],
                                                        out aV);
                        }
                        else
                        {
                            _loc.InternalPerformVertex(iPath - 1,
                                                        _sec.Vertex(isec + 1, vi[1]),
                                                        tabErr[isec, 1] + _sec.VertexTol(isec, vi[1]),
                                                        out aV);
                        }
                        vertex[isec + 1, 1] = aV;

                        if (MergeVertex(vertex[isec, 1], ref vertex[isec + 1, 1]))
                        {
                            vEdge[isec, 1] = NullEdge(ref vertex[isec, 1]);
                        }
                    }
                }

                if (uclose && (isec == nbLaw))
                {
                    vertex[isec + 1, ipath + 1] = vertex[1, ipath + 1];
                }
                else if (vclose && (ipath == nbPath))
                {
                    vertex[isec + 1, ipath + 1] = vertex[isec + 1, 1];
                }
                else if (vertex[isec + 1, ipath + 1].IsNull)
                {
                    Vertex aV;
                    if (constSection)
                    {
                        _loc.InternalPerformVertex(iPath,
                                                    ShapeCasts.Vertex(secVertex[isec + 1]),
                                                    tabErr[isec, ipath] + vError[isec + 1],
                                                    out aV);
                    }
                    else
                    {
                        _loc.InternalPerformVertex(iPath,
                                                    _sec.Vertex(isec + 1, vi[ipath + 1]),
                                                    tabErr[isec, ipath] + _sec.VertexTol(isec, vi[ipath + 1]),
                                                    out aV);
                    }
                    vertex[isec + 1, ipath + 1] = aV;
                }

                // Singular cases
                singv = MergeVertex(vertex[isec, ipath + 1], ref vertex[isec + 1, ipath + 1]);
                singu = MergeVertex(vertex[isec + 1, ipath], ref vertex[isec + 1, ipath + 1]);

                if (singu || singv)
                {
                    degenerated[isec, ipath] = IsDegen(tabS[isec, ipath]!, Max(_tol3d, tabErr[isec, ipath]));
                }
                if (degenerated[isec, ipath])
                {
                    hasdegen = true;
                    // Particular construction of edges
                    if (uEdge[isec + 1, ipath].IsNull)
                    {
                        if (singu)
                        {
                            // Degenerated edge
                            uEdge[isec + 1, ipath] = NullEdge(ref vertex[isec + 1, ipath]);
                        }
                        else
                        { // Copy the previous edge
                            uEdge[isec + 1, ipath] = uEdge[isec, ipath];
                        }
                    }
                    if (vEdge[isec, ipath + 1].IsNull)
                    {
                        if (singv)
                        {
                            // Degenerated Edge
                            vEdge[isec, ipath + 1] = NullEdge(ref vertex[isec, ipath + 1]);
                        }
                        else
                        { // Copy the previous edge
                            vEdge[isec, ipath + 1] = vEdge[isec, ipath];
                        }
                    }
                }
                else
                { // Construction of edges by isos
                    if (exuv)
                    {
                        double uv;
                        uv = uFirst;
                        uFirst = vFirst;
                        vFirst = uv;
                        uv = uLast;
                        uLast = vLast;
                        vLast = uv;
                    }

                    // (2.2) Iso-u
                    if (isec == 1 && uEdge[1, ipath].IsNull)
                    {
                        if (!vertex[1, ipath].IsSame(vertex[1, ipath + 1]))
                        {
                            Point3D p1 = Tool.Pnt(ShapeCasts.Vertex(vertex[1, ipath]));
                            Point3D p2 = Tool.Pnt(ShapeCasts.Vertex(vertex[1, ipath + 1]));
                            if (p1.Distance(p2) <= _tol3d)
                            {
                                vertex[1, ipath + 1] = vertex[1, ipath];
                            }
                        }
                        uEdge[1, ipath] = BuildEdge(s, !exuv, uFirst, vertex[1, ipath], vertex[1, ipath + 1], _tol3d);
                    }
                    else
                    {
                        if (uEdge[isec, ipath].IsNull)
                        { // sweep failed
                            return false;
                        }
                        UpdateEdge(ShapeCasts.Edge(uEdge[isec, ipath]), s, !exuv, uFirst);
                    }

                    if (uclose && (isec == nbLaw))
                    {
                        if (uEdge[1, ipath].IsNull) // degenerated case
                        {
                            uEdge[isec + 1, ipath] = BuildEdge(s,
                                                               !exuv,
                                                               uLast,
                                                               vertex[isec + 1, ipath],
                                                               vertex[isec + 1, ipath + 1],
                                                               _tol3d);
                        }
                        else
                        {
                            UpdateEdge(ShapeCasts.Edge(uEdge[1, ipath]), s, !exuv, uLast);
                            uEdge[isec + 1, ipath] = uEdge[1, ipath];
                        }
                    }
                    else
                    {
                        if (uEdge[isec + 1, ipath].IsNull)
                        {
                            uEdge[isec + 1, ipath] = BuildEdge(s,
                                                               !exuv,
                                                               uLast,
                                                               vertex[isec + 1, ipath],
                                                               vertex[isec + 1, ipath + 1],
                                                               _tol3d);
                        }
                        else
                        {
                            UpdateEdge(ShapeCasts.Edge(uEdge[isec + 1, ipath]), s, !exuv, uLast);
                        }
                    }

                    // (2.3) Iso-v
                    if (ipath == 1)
                    {
                        Edge newFirstEdge = BuildEdge(s, exuv, vFirst, vertex[isec, 1], vertex[isec + 1, 1], _tol3d);
                        if (vEdge[isec, ipath].IsNull)
                        {
                            vEdge[isec, ipath] = newFirstEdge;
                        }
                        else // rebuild first edge
                        {
                            Edge edge = ShapeCasts.Edge(vEdge[isec, ipath]);
                            RebuildTopOrBottomEdge(newFirstEdge, ref edge, reversedEdges);
                            vEdge[isec, ipath] = edge;
                            if (reversedEdges.Contains(vEdge[isec, ipath]))
                            {
                                ReverseEdgeInFirstOrLastWire(_firstShape, vEdge[isec, ipath]);
                                startEdges[isec].Reverse();
                            }
                        }
                    }

                    else
                    {
                        UpdateEdge(ShapeCasts.Edge(vEdge[isec, ipath]), s, exuv, vFirst);
                    }

                    if (vclose && (ipath == nbPath))
                    {
                        if (vEdge[isec, 1].IsNull) // degenerated case
                        {
                            vEdge[isec, ipath + 1] = BuildEdge(s,
                                                               exuv,
                                                               vLast,
                                                               vertex[isec, ipath + 1],
                                                               vertex[isec + 1, ipath + 1],
                                                               _tol3d);
                        }
                        else
                        {
                            UpdateEdge(ShapeCasts.Edge(vEdge[isec, 1]), s, exuv, vLast);
                            vEdge[isec, ipath + 1] = vEdge[isec, 1];
                        }
                    }
                    else if (vEdge[isec, ipath + 1].IsNull)
                    {
                        vEdge[isec, ipath + 1] = BuildEdge(s,
                                                           exuv,
                                                           vLast,
                                                           vertex[isec, ipath + 1],
                                                           vertex[isec + 1, ipath + 1],
                                                           _tol3d);
                    }
                    else
                    {
                        if (ipath != nbPath || vclose || (global_vclose && iLast == _loc.LawCount + 1))
                        {
                            UpdateEdge(ShapeCasts.Edge(vEdge[isec, ipath + 1]), s, exuv, vLast);
                        }
                        else // ipath == NbPath && !vclose => rebuild last edge
                        {
                            Edge newLastEdge = BuildEdge(s,
                                                                 exuv,
                                                                 vLast,
                                                                 vertex[isec, ipath + 1],
                                                                 vertex[isec + 1, ipath + 1],
                                                                 _tol3d);
                            Edge edge = ShapeCasts.Edge(vEdge[isec, ipath + 1]);
                            RebuildTopOrBottomEdge(newLastEdge, ref edge, reversedEdges);
                            vEdge[isec, ipath + 1] = edge;
                            if (reversedEdges.Contains(vEdge[isec, ipath + 1]))
                            {
                                ReverseEdgeInFirstOrLastWire(_lastShape, vEdge[isec, ipath + 1]);
                            }
                        }
                    }
                }
            } // End of construction of edges
        }

        // (3) Construction of Faces
        Face face = new();

        for (ipath = 1, iPath = iFirst; ipath <= nbPath; ipath++, iPath++)
        {
            for (isec = 1; isec <= nbLaw; isec++)
            {
                if (degenerated[isec, ipath])
                {
                    if (uEdge[isec, ipath].IsSame(uEdge[isec + 1, ipath]))
                    {
                        _faces!.SetValue(isec, iPath, uEdge[isec, ipath]);
                    }
                    else
                    {
                        _faces!.SetValue(isec, iPath, vEdge[isec, ipath]);
                    }
                }
                else if (_faces!.Value(isec, iPath).IsNull)
                {
                    BuildFace(tabS[isec, ipath]!,
                              ShapeCasts.Edge(uEdge[isec, ipath]),
                              ShapeCasts.Edge(vEdge[isec, ipath]),
                              ShapeCasts.Edge(uEdge[isec + 1, ipath]),
                              ShapeCasts.Edge(vEdge[isec, ipath + 1]),
                              _vEdgesModified,
                              exchUV[isec, ipath],
                              uReverse[isec, ipath],
                              ref face);
                    _faces.SetValue(isec, iPath, face);
                }
            }
        }

        // (3.1) Reverse the faces that have been built earlier
        for (ipath = 1; ipath <= nbPath; ipath++)
        {
            for (isec = 1; isec <= nbLaw; isec++)
            {
                if (isBuilt[isec])
                {
                    _faces!.ChangeValue(isec, ipath).Reverse();
                }
            }
        }

        // (4) History and Continuity

        if (hasdegen)
        {
            //(4.1) // Degenerated case => Sledgehammer
            Compound comp = new();
            b.MakeCompound(ref comp);
            for (isec = 1; isec <= nbLaw + 1; isec++)
            {
                for (ipath = 1, iPath = iFirst; ipath <= nbPath + 1; ipath++, iPath++)
                {
                    if (ipath <= nbPath)
                    {
                        _uEdges!.SetValue(isec, iPath, uEdge[isec, ipath]);
                    }
                    if (isec <= nbLaw)
                    {
                        _vEdges!.SetValue(isec, iPath, vEdge[isec, ipath]);
                    }
                    if ((ipath <= nbPath) && (isec <= nbLaw) && !_faces!.Value(isec, iPath).IsNull
                        && _faces.Value(isec, iPath).ShapeType() == ShapeType.Face)
                    {
                        b.Add(comp, _faces.Value(isec, iPath));
                    }
                }
            }
            BrepLibrary.EncodeRegularity(comp, _tolAngular);
        }
        else
        {
            //(4.2) // General case => Tweezers
            bool isG1;
            Face ff;
            Edge edge;

            for (isec = 1; isec <= nbLaw + 1; isec++)
            {
                if (isec > 1)
                {
                    isG1 = (_sec.Continuity(isec - 1, _tolAngular) >= G1);
                }
                else
                {
                    isG1 = false;
                }
                for (ipath = 1, iPath = iFirst; ipath <= nbPath; ipath++, iPath++)
                {
                    _uEdges!.SetValue(isec, iPath, uEdge[isec, ipath]);
                    if (isG1)
                    {
                        if (isec == nbLaw + 1)
                        {
                            ff = ShapeCasts.Face(_faces!.Value(1, iPath));
                        }
                        else
                        {
                            ff = ShapeCasts.Face(_faces!.Value(isec, iPath));
                        }
                        b.Continuity(ShapeCasts.Edge(_uEdges.Value(isec, iPath)),
                                     ShapeCasts.Face(_faces.Value(isec - 1, iPath)),
                                     ff,
                                     G1);
                    }
                }
            }

            int nbpath = nbPath;
            if (vclose)
            {
                nbpath++; // Another test G1
            }
            for (ipath = 1, iPath = iFirst; ipath <= nbPath + 1; ipath++, iPath++)
            {
                if ((ipath > 1) && (ipath <= nbpath))
                {
                    isG1 = (_loc.IsG1(iPath - 1, _tol3d, _tolAngular) >= 0);
                }
                else
                {
                    isG1 = false;
                }
                for (isec = 1; isec <= nbLaw; isec++)
                {
                    _vEdges!.SetValue(isec, iPath, vEdge[isec, ipath]);
                    if (isG1)
                    {
                        if (ipath == nbPath + 1)
                        {
                            ff = ShapeCasts.Face(_faces!.Value(isec, 1));
                        }
                        else
                        {
                            ff = ShapeCasts.Face(_faces!.Value(isec, iPath));
                        }
                        edge = ShapeCasts.Edge(_vEdges.Value(isec, iPath));
                        BrepLibrary.EncodeRegularity(edge, ff, ShapeCasts.Face(_faces.Value(isec, iPath - 1)), _tolAngular);
                    }
                }
            }
        }

        // (5) Update Tapes and Rails
        int jj;
        if (iFirst == 1 && tapes.Count != 0) // works only in case of single shell
        {
            for (isec = 1; isec <= nbLaw; isec++)
            {
                NCollection_HArray2<Shape> tape = tapes.GetValue(startEdges[isec]);
                for (jj = 1; jj <= nbPath + 1; jj++)
                {
                    tape.SetValue(1, jj, _vEdges!.Value(isec, jj));
                }
                for (jj = 1; jj <= nbPath; jj++)
                {
                    tape.SetValue(2, jj, _uEdges!.Value(isec, jj));
                }
                for (jj = 1; jj <= nbPath; jj++)
                {
                    tape.SetValue(3, jj, _uEdges!.Value(isec + 1, jj));
                }
                for (jj = 1; jj <= nbPath + 1; jj++)
                {
                    tape.SetValue(4, jj, vertex[isec, jj]);
                }
                for (jj = 1; jj <= nbPath + 1; jj++)
                {
                    tape.SetValue(5, jj, vertex[isec + 1, jj]);
                }
                for (jj = 1; jj <= nbPath; jj++)
                {
                    tape.SetValue(6, jj, _faces!.Value(isec, jj));
                }
                Exploration.Vertices(ShapeCasts.Edge(startEdges[isec]), out Vertex vfirst, out Vertex vlast, true); //with orientation
                if (!rails.ContainsKey(vfirst))
                {
                    NCollection_HArray2<Shape> array = new(1, 2, 1, nbPath + 1);
                    for (jj = 1; jj <= nbPath; jj++)
                    {
                        array.SetValue(1, jj, _uEdges!.Value(isec, jj));
                    }
                    for (jj = 1; jj <= nbPath + 1; jj++)
                    {
                        array.SetValue(2, jj, vertex[isec, jj]);
                    }
                    rails[vfirst] = array;
                }
                if (!rails.ContainsKey(vlast))
                {
                    NCollection_HArray2<Shape> array = new(1, 2, 1, nbPath + 1);
                    for (jj = 1; jj <= nbPath; jj++)
                    {
                        array.SetValue(1, jj, _uEdges!.Value(isec + 1, jj));
                    }
                    for (jj = 1; jj <= nbPath + 1; jj++)
                    {
                        array.SetValue(2, jj, vertex[isec + 1, jj]);
                    }
                    rails[vlast] = array;
                }
            }
        }

        return true;
    }

    //=======================================================================
    // function : Build
    // purpose  : Construct the result of sweeping
    //======================================================================
    /// <summary>
    /// Builds the sweep: <paramref name="transition"/> treats the corners, <paramref name="approx"/> chooses what is
    /// approximated (the composed function, or the location law only), <paramref name="continuity"/>,
    /// <paramref name="degmax"/> and <paramref name="segmax"/> bound the surfaces in v. <paramref name="reversedEdges"/>,
    /// <paramref name="tapes"/> and <paramref name="rails"/> carry what earlier sweeps of the same pipe built.
    /// </summary>
    public void Build(HashSet<Shape> reversedEdges,
                      Dictionary<Shape, NCollection_HArray2<Shape>> tapes,
                      Dictionary<Shape, NCollection_HArray2<Shape>> rails,
                      TransitionStyle transition = TransitionStyle.Modified,
                      Continuity continuity = C2,
                      ApproxStyle approx = ApproxStyle.Location,
                      int degmax = 11,
                      int segmax = 30)
    {
        _continuity = continuity;
        _approxStyle = approx;
        _degmax = degmax;
        _segmax = segmax;

        CorrectApproxParameters();

        // Wire
        if (_sec.IsVertex)
        {
            _isDone = BuildWire(transition);
        }
        else
        { // Shell
            int nbTrous = _loc.NbHoles(_tol3d), nbPath = _loc.LawCount, nbLaw = _sec.LawCount, ii, jj, nbPart = 1;
            int ipath, isec;
            BrepBuilder b = new();
            _uEdges = new NCollection_HArray2<Shape>(1, nbLaw + 1, 1, nbPath);
            _vEdges = new NCollection_HArray2<Shape>(1, nbLaw, 1, nbPath + 1);
            _faces = new NCollection_HArray2<Shape>(1, nbLaw, 1, nbPath);
            _tapes = new NCollection_HArray1<Shape>(1, nbLaw);
            BrepBuilder bb = new();
            for (int i = 1; i <= nbLaw; i++)
            {
                Shell aShell = new();
                bb.MakeShell(ref aShell);
                _tapes[i] = aShell;
            }
            NCollection_HArray2<Shape> bounds = new(1, nbLaw, 1, 2);

            NCollection_HArray1<int>? trous = null;

            if (nbTrous > 0)
            { // How many sub-parts ?
                trous = new NCollection_HArray1<int>(1, nbTrous);
                _loc.Holes(trous.ChangeArray1());
                nbPart += nbTrous;
                if (trous.Value(nbTrous) == nbPath + 1)
                {
                    nbPart--;
                }
            }
            if (nbPart == 1)
            { // This is done at once
                double extend = 0.0;
                if (nbTrous == 1)
                {
                    extend = EvalExtrapol(1, transition);
                }
                _isDone = BuildShell(transition, 1, nbPath + 1, reversedEdges, tapes, rails, extend, extend);
            }
            else
            { //  This is done piece by piece
                int iFirst = 1, iLast;
                for (ii = 1, _isDone = true; ii <= nbPart && _isDone; ii++)
                {
                    if (ii > nbTrous)
                    {
                        iLast = nbPath + 1;
                    }
                    else
                    {
                        iLast = trous!.Value(ii);
                    }
                    _isDone = BuildShell(transition,
                                        iFirst,
                                        iLast,
                                        reversedEdges,
                                        tapes,
                                        rails,
                                        EvalExtrapol(iFirst, transition),
                                        EvalExtrapol(iLast, transition));
                    if (iFirst > 1)
                    {
                        Translate(_vEdges, iFirst, bounds, 2);
                        if (!PerformCorner(iFirst, transition, bounds))
                        {
                            _isDone = false;
                            return;
                        }
                    }
                    iFirst = iLast;
                    Translate(_vEdges, iFirst, bounds, 1);
                }
            }
            // Management of looping ends
            if ((nbTrous > 0) && (_loc.IsClosed()) && (trous!.Value(nbTrous) == nbPath + 1))
            {
                Translate(_vEdges, nbPath + 1, bounds, 1);
                Translate(_vEdges, 1, bounds, 2);
                if (!PerformCorner(1, transition, bounds))
                {
                    _isDone = false;
                    return;
                }
                Translate(_vEdges, 1, _vEdges, nbPath + 1);
            }

            // Construction of the shell
            Shell shell = new();
            b.MakeShell(ref shell);
            int nbFaces = 0;
            for (ipath = 1; ipath <= nbPath; ipath++)
            {
                for (isec = 1; isec <= nbLaw; isec++)
                {
                    Shape face = _faces.Value(isec, ipath);
                    if (!face.IsNull && (face.ShapeType() == ShapeType.Face))
                    {
                        b.Add(shell, face);
                        nbFaces++;
                    }
                }
            }

            if ((nbTrous > 0) ? (nbFaces < nbLaw) : (nbFaces == 0))
            {
                _isDone = false;
                return;
            }

            NCollection_List<Shape>.Iterator it = new(_auxShape);
            for (; it.More(); it.Next())
            {
                Shape face = it.Value();
                if (!face.IsNull && (face.ShapeType() == ShapeType.Face))
                {
                    b.Add(shell, face);
                }
            }
            // Set common Uedges to faces
            Substitution substitute = new();
            // (OCCT's commented-out SetCommonEdgeInFace calls are left out.)
            // OCCT iterates myVEdgesModified in address order; the port in binding order (occt-issues.md)
            foreach (KeyValuePair<Shape, Shape> mapit in _vEdgesModified)
            {
                Edge oldEdge = ShapeCasts.Edge(mapit.Key);
                Edge newEdge = ShapeCasts.Edge(mapit.Value);
                Substitute(substitute, oldEdge, newEdge);
            }
            substitute.Build(shell);
            if (substitute.IsCopied(shell))
            {
                NCollection_List<Shape> listSh = substitute.InternalCopy(shell);
                shell = ShapeCasts.Shell(listSh.First());
            }

            for (ii = _faces.LowerRow(); ii <= _faces.UpperRow(); ii++)
            {
                for (jj = _faces.LowerCol(); jj <= _faces.UpperCol(); jj++)
                {
                    Shape localShape = _faces.Value(ii, jj);

                    if (!localShape.IsNull && substitute.IsCopied(localShape))
                    {
                        NCollection_List<Shape> list = substitute.InternalCopy(localShape);

                        if (!list.IsEmpty())
                        {
                            _faces[ii, jj] = list.First();
                        }
                    }
                }
            }

            for (ii = _vEdges.LowerRow(); ii <= _vEdges.UpperRow(); ii++)
            {
                for (jj = _vEdges.LowerCol(); jj <= _vEdges.UpperCol(); jj++)
                {
                    Shape localShape = _vEdges.Value(ii, jj);

                    if (!localShape.IsNull && substitute.IsCopied(localShape))
                    {
                        NCollection_List<Shape> list = substitute.InternalCopy(localShape);

                        if (!list.IsEmpty())
                        {
                            _vEdges[ii, jj] = list.First();
                        }
                    }
                }
            }

            for (ii = _uEdges.LowerRow(); ii <= _uEdges.UpperRow(); ii++)
            {
                for (jj = _uEdges.LowerCol(); jj <= _uEdges.UpperCol(); jj++)
                {
                    Shape localShape = _uEdges.Value(ii, jj);

                    if (!localShape.IsNull && substitute.IsCopied(localShape))
                    {
                        NCollection_List<Shape> list = substitute.InternalCopy(localShape);

                        if (!list.IsEmpty())
                        {
                            _uEdges[ii, jj] = list.First();
                        }
                    }
                }
            }

            // Ensure Same Parameter on U-edges
            for (ii = _uEdges.LowerRow(); ii <= _uEdges.UpperRow(); ii++)
            {
                if (_sec.IsUClosed && ii == _uEdges.UpperRow())
                {
                    continue;
                }
                for (jj = _uEdges.LowerCol(); jj <= _uEdges.UpperCol(); jj++)
                {
                    Edge edge = ShapeCasts.Edge(_uEdges.Value(ii, jj));
                    if (edge.IsNull || Tool.Degenerated(edge))
                    {
                        continue;
                    }
                    Face face1 = new(), face2 = new();
                    int i1 = ii - 1, i2 = ii;
                    if (i1 == 0 && _sec.IsUClosed)
                    {
                        i1 = _faces.UpperRow();
                    }
                    if (i2 > _faces.UpperRow())
                    {
                        i2 = 0;
                    }
                    if (i1 != 0)
                    {
                        Shape shape1 = _faces.Value(i1, jj);
                        if (shape1.ShapeType() == ShapeType.Face)
                        {
                            face1 = ShapeCasts.Face(shape1);
                        }
                    }
                    if (i2 != 0)
                    {
                        Shape shape2 = _faces.Value(i2, jj);
                        if (shape2.ShapeType() == ShapeType.Face)
                        {
                            face2 = ShapeCasts.Face(shape2);
                        }
                    }
                    if (!face1.IsNull && !face2.IsNull)
                    {
                        CorrectSameParameter(edge, face1, face2);
                    }
                }
            }

            for (ii = 1; ii <= nbLaw; ii++)
            {
                for (jj = 1; jj <= nbPath; jj++)
                {
                    Shape aFace = _faces.Value(ii, jj);
                    if (!aFace.IsNull && aFace.ShapeType() == ShapeType.Face)
                    {
                        bb.Add(_tapes[ii], aFace);
                    }
                }
            }

            // Is it Closed ?
            if (_loc.IsClosed() && _sec.IsUClosed)
            {
                // Check
                bool closed = true;
                int iedge;
                IndexedMap<Shape, NCollection_List<Shape>> eFmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(shell, ShapeType.Edge, ShapeType.Face, eFmap);

                for (iedge = 1; iedge <= eFmap.Count && closed; iedge++)
                {
                    Edge theEdge = ShapeCasts.Edge(eFmap.KeyAt(iedge - 1));
                    if (Tool.Degenerated(theEdge))
                    {
                        continue;
                    }
                    closed = (eFmap.ValueAt(iedge - 1).Extent() > 1);
                }
                shell.Closed(closed);
            }
            _shape = shell;
        }
    }

    //=================================================================================================

    /// <summary>True if the sweep is built.</summary>
    public bool IsDone => _isDone;

    //=================================================================================================

    /// <summary>The swept shape (unfrozen: the sweep is a working object of the pipes).</summary>
    public Shape Shape() => _shape;

    //=================================================================================================

    /// <summary>The approximation error.</summary>
    public double ErrorOnSurface() => _error;

    //=================================================================================================

    /// <summary>The faces (or degenerated edges) of the sweep: row the section's edge, column the path's.</summary>
    public NCollection_HArray2<Shape>? SubShape() => _faces;

    //=================================================================================================

    /// <summary>The edges between the faces along the path (u-edges).</summary>
    public NCollection_HArray2<Shape>? InterFaces() => _uEdges;

    //=======================================================================
    // function : Sections
    // purpose  : Edges or Face (or compound of 2) Transition between 2 sweepings
    //=======================================================================
    /// <summary>The section edges (or faces, or compounds of both, at corners) between the sweeps.</summary>
    public NCollection_HArray2<Shape>? Sections() => _vEdges;

    //=======================================================================
    // function : Tape
    // purpose  : returns the Tape corresponding to Index-th edge of section
    //=======================================================================
    /// <summary>The shell of the faces swept by the section's edge of rank <paramref name="index"/>.</summary>
    public Shape Tape(int index) => _tapes!.Value(index);

    //=================================================================================================

    private bool PerformCorner(int index, TransitionStyle transition, NCollection_HArray2<Shape> bounds)
    {
        if (transition == TransitionStyle.Modified)
        {
            return true; // Do nothing.
        }

        const double angularTol = 0.025;

        TransitionStyle theTransition = transition;
        bool isTangent = false;
        double f, l;
        int i1, i2, ii;
        Point3D p1, p2;
        Vector3D T1, T2, tang, sortant = new();

        if (index > 1)
        {
            i1 = index - 1;
            i2 = index;
        }
        else
        {
            i1 = _loc.LawCount;
            i2 = 1;
        }

        // Construct an axis supported by the bissectrice
        _loc.Law(i1).GetDomain(out f, out l);
        _loc.Law(i1).GetCurve()!.D1(l, out p1, out T1);
        T1 = T1.Normalized();

        _loc.Law(i2).GetDomain(out f, out l);
        _loc.Law(i2).GetCurve()!.D1(f, out p2, out T2);
        T2 = T2.Normalized();

        if (T1.Angle(T2) < _angMin)
        {
            isTangent = true;
            Vector3D t1, t2, v = new();
            Matrix3x3 m = new();
            _loc.Law(i1).GetDomain(out f, out l);
            _loc.Law(i1).D0(l, ref m, ref v);
            t1 = new Vector3D(m.Column(3));
            _loc.Law(i2).GetDomain(out f, out l);
            _loc.Law(i2).D0(l, ref m, ref v); // OCCT: the next law's frame at its end, not its start
            t2 = new Vector3D(m.Column(3));

            if (t1.Angle(t2) < _angMin)
            {
                return true;
            }
            sortant = t2 - t1;
        }

        if (T1.Angle(T2) >= M_PI - angularTol)
        {
            return false;
        }
        if ((theTransition == TransitionStyle.Right) && (T1.Angle(T2) > _angMax))
        {
            theTransition = TransitionStyle.Round;
        }

        tang = T1 + T2; // Average direction
        Direction3D normalOfBisPlane = new(tang);
        Vector3D intersectPointCrossDirection = T1.Crossed(T2);
        if (isTangent)
        {
            sortant -= tang.Dot(tang) * tang;
        }
        else
        {
            sortant = T2 - T1; // Direction input
            sortant *= -1; //   "   "   output
            tang -= (tang.Dot(T2)) * T2;
        }

        p1 = p1.BaryCentered(0.5, p2, 0.5);
        Direction3D n = new(sortant);
        Direction3D dx = new(tang);

        Frame3D axe = new(p1, n, dx);
        Frame3D axeOfBisPlane = new(p1, normalOfBisPlane);

        // Construct 2 intersecting Shells
        NCollection_HArray2<Shape> UEdges = new(1, _sec.LawCount + 1, 1, _loc.LawCount);
        UEdges.ChangeArray2().Assign(_uEdges!.Array2());

        // modified by NIZHNY-MKK  Wed Oct 29 18:31:47 2003.BEGIN
        NCollection_HArray2<Shape> faces = new(_faces!.LowerRow(), _faces.UpperRow(), 1, 2);
        Translate(_faces, i1, faces, 1);
        Translate(_faces, i2, faces, 2);

        NCollection_HArray2<Shape> uEdges = new(_uEdges.LowerRow(), _uEdges.UpperRow(), 1, 2);
        Translate(_uEdges, i1, uEdges, 1);
        Translate(_uEdges, i2, uEdges, 2);

        Vector3D normal = T2 + T1;
        Face planeF = new();

        if (normal.Magnitude() > Constants.Resolution())
        {
            Plane3D pl = new(p1, new Direction3D(normal));
            LibraryMakeFace fMaker = new(pl);

            if (fMaker.Error() == LibraryFaceError.FaceDone)
            {
                planeF = fMaker.InternalFace();
                BrepBuilder aBB = new();
                aBB.UpdateFace(planeF, global::Kerf.Engine.Foundation.Precision.Confusion() * 10.0);
            }
        }

        TrimShellCorner trim = new(faces, transition, axeOfBisPlane, intersectPointCrossDirection);
        trim.AddBounds(bounds);
        trim.AddUEdges(uEdges);
        trim.AddVEdges(_vEdges!, index);
        trim.Perform();

        if (trim.IsDone)
        {
            NCollection_List<Shape> listmodif = new();
            for (ii = 1; ii <= _sec.LawCount; ii++)
            {
                listmodif.Clear();
                trim.Modified(_vEdges!.Value(ii, index), listmodif);

                if (listmodif.IsEmpty())
                {
                    Edge nullEdge = new();
                    _vEdges.SetValue(ii, index, nullEdge);
                }
                else
                {
                    _vEdges.SetValue(ii, index, listmodif.First());
                }
            }

            listmodif.Clear();
            int iit = 0;

            for (iit = 0; iit < 2; iit++)
            {
                int II = (iit == 0) ? i1 : i2;

                for (ii = 1; ii <= _sec.LawCount; ii++)
                {
                    trim.Modified(_faces.Value(ii, II), listmodif);

                    if (!listmodif.IsEmpty())
                    {
                        _faces.SetValue(ii, II, listmodif.First());
                    }
                }

                for (ii = _uEdges.LowerRow(); ii <= _uEdges.UpperRow(); ii++)
                {
                    trim.Modified(_uEdges.Value(ii, II), listmodif);

                    if (!listmodif.IsEmpty())
                    {
                        _uEdges.SetValue(ii, II, listmodif.First());
                    }
                }
            }
        }
        else if ((theTransition == TransitionStyle.Right) || trim.HasSection)
        {
            return true; // Nothing is touched
        }

        if (_sec.IsUClosed)
        {
            _uEdges.SetValue(1, i1, _uEdges.Value(_sec.LawCount + 1, i1));
            _uEdges.SetValue(1, i2, _uEdges.Value(_sec.LawCount + 1, i2));
        }

        if (theTransition == TransitionStyle.Round)
        {
            // Filling
            NCollection_List<Shape> list1 = new(), list2 = new();
            Edge bord1 = new(), bord2 = new(), bordFirst = new();
            bordFirst.Nullify();
            bord1.Nullify();
            bord2.Nullify();
            bool hasFilling = false;
            Face ff = new();
            for (ii = 1; ii <= _sec.LawCount; ii++)
            {
                KeepEdge(_faces.Value(ii, i1), bounds.Value(ii, 1), list1);
                KeepEdge(_faces.Value(ii, i2), bounds.Value(ii, 2), list2);
                if (list1.Extent() == list2.Extent())
                {
                    NCollection_List<Shape>.Iterator it1 = new(list1);
                    NCollection_List<Shape>.Iterator it2 = new(list2);
                    bool b;
                    for (; it1.More(); it1.Next(), it2.Next())
                    {
                        if (hasFilling)
                        { // Transversal choice of constraints
                            Vertex vc = new();
                            Edge e = ShapeCasts.Edge(it1.Value());
                            Edge e1 = new(), e2 = new();
                            e1.Nullify();
                            e2.Nullify();
                            Exploration.Vertices(e, out Vertex vf, out _);
                            if (!bord1.IsNull && Exploration.CommonVertex(e, bord1, ref vc))
                            {
                                if (vc.IsSame(vf))
                                {
                                    e1 = bord1;
                                }
                                else
                                {
                                    e2 = bord1;
                                }
                            }
                            if (!bord2.IsNull && Exploration.CommonVertex(e, bord2, ref vc))
                            {
                                if (vc.IsSame(vf))
                                {
                                    e1 = bord2;
                                }
                                else
                                {
                                    e2 = bord2;
                                }
                            }
                            if (!bordFirst.IsNull && Exploration.CommonVertex(e, bordFirst, ref vc))
                            {
                                if (vc.IsSame(vf))
                                {
                                    e1 = bordFirst;
                                }
                                else
                                {
                                    e2 = bordFirst;
                                }
                            }
                            bord1 = e1;
                            bord2 = e2;
                        }

                        // Filling
                        b = Filling(it1.Value(),
                                    _faces.Value(ii, i1),
                                    it2.Value(),
                                    _faces.Value(ii, i2),
                                    _vEdgesModified,
                                    _tol3d,
                                    axe,
                                    T1,
                                    ref bord1,
                                    ref bord2,
                                    ref ff);

                        if (b)
                        {
                            _auxShape.Append(ff);
                            BrepBuilder bb = new();
                            Shape vshape = _vEdges!.Value(ii, i2);
                            Compound compound = new();
                            bb.MakeCompound(ref compound);
                            if (!vshape.IsNull)
                            {
                                bb.Add(compound, vshape);
                            }
                            bb.Add(compound, ff);
                            _vEdges[ii, i2] = compound;

                            bb.Add(_tapes![ii], ff);
                            hasFilling = true;
                        }
                        if (ii == 1)
                        {
                            bordFirst = bord1;
                        }
                    }
                }
            }
        }
        return true;
    }

    //=================================================================================================

    private double EvalExtrapol(int index, TransitionStyle transition)
    {
        double extrap = 0.0;
        if (transition == TransitionStyle.Right)
        {
            int i1, i2;
            if ((index == 1) || (index == _loc.LawCount + 1))
            {
                if (!_loc.IsClosed() || !_sec.IsVClosed)
                {
                    return extrap;
                }
                i1 = _loc.LawCount;
                i2 = 1;
            }
            else
            {
                i1 = index - 1;
                i2 = index;
            }

            Vector3D v1 = new(), v2 = new(), t1 = new(), t2 = new();
            Matrix3x3 m1 = new(), m2 = new();
            double r, f, l;

            _loc.Law(i1).GetDomain(out f, out l);
            _loc.Law(i1).D0(l, ref m1, ref v1);
            t1 = t1.WithXYZ(m1.Column(3));
            _loc.Law(i2).GetDomain(out f, out l);
            _loc.Law(i2).D0(f, ref m2, ref v2);
            t2 = t2.WithXYZ(m2.Column(3));

            double alpha = t1.Angle(t2);
            if ((alpha > _angMax) || (alpha < _angMin))
            {
                // Angle too great => No "straight" connection
                // Angle too small => No connection
                return extrap; // = 0.0
            }

            SurfaceSectionLaw sec;
            sec = _sec.ConcatenedLaw()!;

            // Calculating parameter U
            double u, length, secFirst, secLen, ll;
            _loc.CurvilinearBounds(_loc.LawCount, out _, out length);
            _sec.Law(1).GetDomain(out secFirst, out secLen);
            secLen -= secFirst;
            _loc.CurvilinearBounds(i1, out _, out ll);
            u = secFirst + (ll / length) * secLen;

            Box box = new();
            Box(sec, u, ref box);
            box.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);

            r = Max(Max(Math.Abs(xmin), Math.Abs(xmax)), Max(Math.Abs(ymin), Math.Abs(ymax)));
            //  modified by NIZHNY-MKK  Fri Oct 31 18:57:51 2003
            double coef = 2.0;
            r *= coef;
            extrap = Max(Math.Abs(zmin), Math.Abs(zmax)) + 100 * _tol3d;
            extrap += r * Math.Tan(alpha / 2);
        }
        return extrap;
    }

    //=======================================================================
    // function : MergeVertex
    // purpose  : Make V2 = V1 if V2 is too close to V1
    //======================================================================
    private bool MergeVertex(in Shape V1, ref Shape V2)
    {
        Vertex v1 = ShapeCasts.Vertex(V1);
        Vertex v2 = ShapeCasts.Vertex(V2);
        double tol;
        tol = Max(Tool.Tolerance(v1), Tool.Tolerance(v2));
        if (tol < _tol3d)
        {
            tol = _tol3d;
        }
        if (Tool.Pnt(v1).Distance(Tool.Pnt(v2)) <= tol)
        {
            V2 = V1;
            return true;
        }
        return false;
    }

    //=======================================================================
    // function : UpdateVertex
    // purpose  : Update the Tolerance of Vertices depending on Laws.
    //======================================================================
    private void UpdateVertex(int ipath, int isec, double errApp, double param, in Shape v)
    {
        Vertex theV;
        theV = ShapeCasts.Vertex(v);
        _loc.InternalPerformVertex(ipath,
                                    _sec.Vertex(isec, param),
                                    errApp + _sec.VertexTol(isec - 1, param),
                                    out Vertex vv);
        Point3D p1, p2;
        p1 = Tool.Pnt(vv);
        p2 = Tool.Pnt(theV);

        double tol = Tool.Tolerance(vv);
        tol += p1.Distance(p2);

        if (tol > Tool.Tolerance(theV))
        {
            BrepBuilder b = new();
            b.UpdateVertex(theV, tol);
        }
    }

    //=======================================================================
    // function : RebuildTopOrBottomEdge
    // purpose  : Rebuild v-iso edge of top or bottom section
    //           inserting new 3d and 2d curves taken from swept surfaces
    //======================================================================
    private void RebuildTopOrBottomEdge(in Edge aNewEdge, ref Edge anEdge, HashSet<Shape> reversedEdges)
    {
        Curve3D? newCurve = Tool.Curve(aNewEdge, out double fpar, out double lpar);
        Placement identity = new();

        bool toReverse = false;
        bool isDegen = Tool.Degenerated(aNewEdge);
        if (isDegen)
        {
            Tool.Range(aNewEdge, out fpar, out lpar);
        }
        else
        {
            Exploration.Vertices(anEdge, out Vertex v1, out Vertex v2);
            if (!v1.IsSame(v2))
            {
                Exploration.Vertices(aNewEdge, out Vertex newV1, out Vertex newV2);
                v1.Location(identity);
                if (!v1.IsSame(newV1))
                {
                    if (v1.IsSame(newV2))
                    {
                        toReverse = true;
                    }
                    else
                    {
                        Point3D pnt1 = Tool.Pnt(v1);
                        Point3D newPnt1 = Tool.Pnt(newV1);
                        double tolSum = Tool.Tolerance(v1) + Tool.Tolerance(newV1);
                        if (!pnt1.IsEqual(newPnt1, tolSum))
                        {
                            toReverse = true;
                        }
                    }
                }
            }
            else
            {
                Curve3D oldCurve = Tool.Curve(anEdge, out double oldFirst, out double oldLast)!;
                oldCurve.D1(0.5 * (oldFirst + oldLast), out _, out Vector3D oldD1);
                newCurve!.D1(0.5 * (fpar + lpar), out _, out Vector3D newD1);
                if (oldD1 * newD1 < 0.0)
                {
                    toReverse = true;
                }
            }
        }

        anEdge.Location(identity);
        BrepTEdge tEdge = (BrepTEdge)anEdge.TShape()!;
        tEdge.Tolerance(Tool.Tolerance(aNewEdge));
        BrepBuilder bb = new();
        bb.Range(anEdge, fpar, lpar);
        bb.UpdateEdge(anEdge, newCurve, global::Kerf.Engine.Foundation.Precision.Confusion());
        BrepTEdge te = (BrepTEdge)aNewEdge.TShape()!;
        NCollection_List<CurveRepresentation> lcr = te.Curves();
        NCollection_List<CurveRepresentation>.Iterator itrep = new(lcr);
        for (; itrep.More(); itrep.Next())
        {
            CurveRepresentation curveRep = itrep.Value();
            if (curveRep.IsCurveOnSurface())
            {
                GCurve gc = (GCurve)curveRep;
                Curve2D? pCurve = gc.PCurve();
                Surface? surf = gc.Surface();
                Placement loc = aNewEdge.Location() * gc.Location();
                bb.UpdateEdge(anEdge, pCurve, surf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        anEdge.Free(true);
        Exploration.Vertices(anEdge, out Vertex w1, out Vertex w2);

        Shape edgeFORWARD = anEdge.Oriented(Forward);

        bb.Remove(edgeFORWARD, w1);
        bb.Remove(edgeFORWARD, w2);

        w1.Location(identity);
        w2.Location(identity);
        if (toReverse)
        {
            w2.Orientation(Forward);
            w1.Orientation(Reversed);
        }
        bb.Add(edgeFORWARD, w1);
        bb.Add(edgeFORWARD, w2);

        if (toReverse)
        {
            anEdge.Reverse();
            reversedEdges.Add(anEdge);
        }

        bb.Degenerated(anEdge, isDegen);
    }
}
