// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Sweep.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Sweep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepFill_TransitionStyle;
using static Occt.BRepLib_FaceError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Occt.GeomFill_ApproxStyle;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The topological sweep of a section law along a location law: a wire (the section is a vertex) or a shell of faces
/// built from GeomFill_Sweep's surfaces, with the corners of a path that is not G1 treated by the transition mode.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21) used by <see cref="BRepFill_Pipe"/> and <see cref="BRepFill_PipeShell"/>, as in
/// OCCT: it writes into the section edges and vertices it is given (the bounds, the section law's shapes), builds on
/// them, and hands out the unfrozen result and tables. Its callers give it shapes they own (decision 924); a frozen
/// shape reaching a write throws <see cref="ImmutableObjectException"/>. It calls <c>SetInterval</c> and
/// <c>SetTolerance</c> on the location law's elementary laws, which are BRepFill's per-edge copies (decision 900).
/// </para>
/// </remarks>
internal sealed class BRepFill_Sweep
{
    private bool isDone;
    private readonly bool KPart;
    private double myTol3d;
    private double myBoundTol;
    private double myTol2d;
    private double myTolAngular;
    private double myAngMin;
    private double myAngMax;
    private GeomFill_ApproxStyle myApproxStyle;
    private Continuity myContinuity;
    private int myDegmax;
    private int mySegmax;
    private bool myForceApproxC1;
    private TopoDS_Shape myShape = new();
    private readonly BRepFill_LocationLaw myLoc;
    private readonly BRepFill_SectionLaw mySec;
    private NCollection_HArray2<TopoDS_Shape>? myUEdges;
    private NCollection_HArray2<TopoDS_Shape>? myVEdges;
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myVEdgesModified = new(TopTools_ShapeMapHasher.Instance);
    private NCollection_HArray2<TopoDS_Shape>? myFaces;
    private readonly NCollection_List<TopoDS_Shape> myAuxShape = new();
    private NCollection_HArray1<TopoDS_Shape>? myTapes;
    private double Error;
    private TopoDS_Wire FirstShape = new();
    private TopoDS_Wire LastShape = new();

    //=================================================================================================

    private static int NumberOfPoles(in TopoDS_Wire W)
    {
        int NbPoints = 0;

        TopoDS_Iterator iter = new(W);
        for (; iter.More(); iter.Next())
        {
            BRepAdaptor_Curve c = new(TopoDS.Edge(iter.Value()));

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
                    Geom_BezierCurve GC = c.Bezier();
                    int iNbPol = GC.NbPoles();
                    if (iNbPol >= 2)
                    {
                        NbPoints += iNbPol;
                    }
                    break;
                }
                case BSplineCurve:
                {
                    // Put all poles for bspline
                    Geom_BSplineCurve GC = c.BSpline();
                    int iNbPol = GC.NbPoles();
                    if (iNbPol >= 2)
                    {
                        NbPoints += iNbPol;
                    }
                    break;
                }
                case Line:
                {
                    NbPoints += 2;
                    break;
                }
                case Circle:
                case Ellipse:
                case Hyperbola:
                case Parabola:
                {
                    NbPoints += 4;
                    break;
                }
                default:
                    NbPoints += 15 + c.NbIntervals(C3);
                    break;
            } // switch (c.GetType()) ...
        } // for (; iter.More(); iter.Next())

        return NbPoints;
    }

    //=================================================================================================

    private static bool HasPCurves(in TopoDS_Edge E)
    {
        bool haspcurves = false;

        NCollection_List<BRep_CurveRepresentation>.Iterator itcr = new(((BRep_TEdge)E.TShape()!).Curves());
        for (; itcr.More(); itcr.Next())
        {
            BRep_CurveRepresentation cr = itcr.Value();
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
    private static void Translate(NCollection_HArray2<TopoDS_Shape> ArrayIn,
                                  int In,
                                  NCollection_HArray2<TopoDS_Shape> ArrayOut,
                                  int Out)
    {
        int ii, Nb;
        Nb = ArrayOut.ColLength();
        for (ii = 1; ii <= Nb; ii++)
        {
            ArrayOut.SetValue(ii, Out, ArrayIn.Value(ii, In));
        }
    }

    //=================================================================================================

    private static void Box(GeomFill_SectionLaw Sec, double U, ref Box Box)
    {
        Box.SetVoid();
        Sec.SectionShape(out int NbPoles, out _, out _);
        NCollection_Array1<Point3D> Poles = new(1, NbPoles);
        NCollection_Array1<double> W = new(1, NbPoles);
        Sec.D0(U, Poles, W);
        for (int ii = 1; ii <= NbPoles; ii++)
        {
            Box.Add(Poles[ii]);
        }
    }

    //=======================================================================
    // function : Couture
    // purpose  : Check if E is an edge of sewing on S
    //           and make the representation HadHoc
    //=======================================================================
    private static Geom2d_Curve? Couture(in TopoDS_Edge E, Geom_Surface S, in Placement L)
    {
        Placement l = L.Predivided(E.Location());
        bool Eisreversed = (E.Orientation() == TopAbs_REVERSED);

        // find the representation (OCCT reads ChangeCurves(); the port only reads the list)
        NCollection_List<BRep_CurveRepresentation>.Iterator itcr = new(((BRep_TEdge)E.TShape()!).Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(S, l))
            {
                BRep_GCurve GC = (BRep_GCurve)cr;
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
        Geom2d_Curve? pc = null;
        return pc;
    }

    //=======================================================================
    // function : CheckSameParameter
    // purpose  : Check a posteriori that sameparameter has worked correctly
    //=======================================================================

    private static bool CheckSameParameter(Adaptor3d_Curve C3d,
                                           Geom2d_Curve Pcurv,
                                           Adaptor3d_Surface S,
                                           double tol3d,
                                           out double tolreached)
    {
        tolreached = 0.0;
        double f = C3d.FirstParameter();
        double l = C3d.LastParameter();
        int nbp = 45;
        double step = 1.0 / (nbp - 1);
        for (int i = 0; i < nbp; i++)
        {
            double t;
            t = step * i;
            t = (1 - t) * f + t * l;
            Pcurv.Value(t).Coord(out double u, out double v);
            Point3D pS = S.Value(u, v);
            Point3D pC = C3d.Value(t);
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
    private static bool CheckSameParameterExact(Adaptor3d_Curve C3d,
                                                Adaptor3d_CurveOnSurface curveOnSurface,
                                                double tol3d,
                                                out double tolreached)
    {
        GeomLib_CheckCurveOnSurface aCheckCurveOnSurface = new(C3d);
        aCheckCurveOnSurface.SetParallel(false);
        aCheckCurveOnSurface.Perform(curveOnSurface);

        tolreached = aCheckCurveOnSurface.MaxDistance();

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

    private static bool SameParameter(in TopoDS_Edge E,
                                      ref Geom2d_Curve Pcurv,
                                      Geom_Surface Surf,
                                      double tol3d,
                                      out double tolreached)
    {
        Geom_Curve? C3d = BRep_Tool.Curve(E, out double f, out double l);
        GeomAdaptor_Curve GAC3d = new(C3d!, f, l);
        GeomAdaptor_Curve HC3d = GAC3d; // OCCT copies the adaptor into a handle

        GeomAdaptor_Surface S = new(Surf);

        if (CheckSameParameter(HC3d, Pcurv, S, tol3d, out tolreached))
        {
            return true;
        }

        if (!HasPCurves(E))
        {
            Geom2dAdaptor_Curve HC2d = new(Pcurv);
            Approx_CurveOnSurface AppCurve = new(HC2d,
                                                 S,
                                                 HC2d.FirstParameter(),
                                                 HC2d.LastParameter(),
                                                 global::Kerf.Engine.Foundation.Precision.Confusion());
            AppCurve.Perform(10, 10, C1, true);
            if (AppCurve.IsDone() && AppCurve.HasResult())
            {
                C3d = AppCurve.Curve3d();
                tolreached = AppCurve.MaxError3d();
                BRep_Builder B = new();
                B.UpdateEdge(E, C3d, tolreached);
                return true;
            }
        }

        Adaptor3d_Curve aHCurve = HC3d; // to avoid ambiguity
        Approx_SameParameter sp = new(aHCurve, Pcurv, S, tol3d);
        if (sp.IsDone() && !sp.IsSameParameter())
        {
            Pcurv = sp.Curve2d()!;
        }
        else if (!sp.IsDone() && !sp.IsSameParameter())
        {
            return false;
        }

        Adaptor3d_Curve curve3d = sp.Curve3d()!;
        Adaptor3d_CurveOnSurface curveOnSurface = sp.CurveOnSurface()!;

        if (!CheckSameParameterExact(curve3d, curveOnSurface, tol3d, out double ResTol) && ResTol > tolreached)
        {
            return false;
        }
        else
        {
            tolreached = 1.1 * ResTol;
            if (sp.IsDone() && !sp.IsSameParameter())
            {
                Pcurv = sp.Curve2d()!;
            }
        }
        return true;
    }

    private static void CorrectSameParameter(TopoDS_Edge theEdge, in TopoDS_Face theFace1, in TopoDS_Face theFace2)
    {
        if (BRep_Tool.Degenerated(theEdge))
        {
            return;
        }

        Geom_Curve aCurve = BRep_Tool.Curve(theEdge, out double fpar, out double lpar)!;

        bool[] PCurveExists = [false, false];
        BRepAdaptor_Curve[] BAcurve = [new(), new()];

        if (!theFace1.IsNull())
        {
            PCurveExists[0] = true;
            BAcurve[0].Initialize(theEdge, theFace1);
        }
        if (!theFace1.IsNull() && theFace1.IsSame(theFace2))
        {
            theEdge.Reverse();
        }
        if (!theFace2.IsNull())
        {
            PCurveExists[1] = true;
            BAcurve[1].Initialize(theEdge, theFace2);
        }

        double MaxSqDist = 0.0;
        const int NCONTROL = 23;
        double delta = (lpar - fpar) / NCONTROL;

        for (int i = 0; i <= NCONTROL; i++)
        {
            double aParam = fpar + i * delta;
            Point3D aPnt = aCurve.Value(aParam);
            for (int j = 0; j < 2; j++)
            {
                if (PCurveExists[j])
                {
                    Point3D aPntFromFace = BAcurve[j].Value(aParam);
                    double aSqDist = aPnt.SquareDistance(aPntFromFace);
                    if (aSqDist > MaxSqDist)
                    {
                        MaxSqDist = aSqDist;
                    }
                }
            }
        }

        double aTol = Math.Sqrt(MaxSqDist);
        BRep_Builder BB = new();
        BB.UpdateEdge(theEdge, aTol);
    }

    //=======================================================================
    // Objet : Orientate an edge of natural restriction
    //      : General
    //=======================================================================
    private static void Oriente(Geom_Surface S, ref TopoDS_Edge E)
    {
        Vector2D URef = new(1, 0), VRef = new(0, 1);
        bool isuiso, isfirst, isopposite;
        S.Bounds(out double UFirst, out _, out double VFirst, out _);
        Geom2d_Curve C;

        C = BRep_Tool.CurveOnSurface(E, S, new Placement(), out double f, out double l)!;
        C.D1((f + l) / 2, out Point2D P, out Vector2D D);

        isuiso = D.IsParallel(VRef, 0.1);

        if (isuiso)
        {
            isfirst = (Math.Abs(P.X() - UFirst) < global::Kerf.Engine.Foundation.Precision.Confusion());
            isopposite = D.IsOpposite(VRef, 0.1);
            E.Orientation(TopAbs_REVERSED);
        }
        else
        {
            isfirst = (Math.Abs(P.Y() - VFirst) < global::Kerf.Engine.Foundation.Precision.Confusion());
            isopposite = D.IsOpposite(URef, 0.1);
            E.Orientation(TopAbs_FORWARD);
        }

        if (!isfirst)
        {
            E.Reverse();
        }
        if (isopposite)
        {
            E.Reverse();
        }
    }

    // OCC500(apo)->
    private static void UpdateEdgeOnPlane(in TopoDS_Face F, in TopoDS_Edge E, BRep_Builder BB)
    {
        Geom2d_Curve? C2d = BRep_Tool.CurveOnSurface(E, F, out _, out _);
        Geom_Surface? S = BRep_Tool.Surface(F);
        Placement Loc = new();
        double Tol = BRep_Tool.Tolerance(E);
        BB.UpdateEdge(E, C2d, S, Loc, Tol);
        BRepCheck_Edge Check = new(E);
        Tol = Max(Tol, Check.Tolerance());
        BB.UpdateEdge(E, Tol);
        TopoDS_Vertex V;
        Tol *= 1.01;
        V = TopExp.FirstVertex(E);
        if (BRep_Tool.Tolerance(V) < Tol)
        {
            BB.UpdateVertex(V, Tol);
        }
        V = TopExp.LastVertex(E);
        if (BRep_Tool.Tolerance(V) < Tol)
        {
            BB.UpdateVertex(V, Tol);
        }
    }

    // OCCT.NET: BuildFace's three repeated blocks. Adds E to the wire maker, then binds E to the edge actually added
    // (the last of the wire) when the maker replaced it.
    private static void AddAndBind(BRepBuilderAPI_MakeWire B,
                                   in TopoDS_Edge E,
                                   NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> EEmap)
    {
        B.Add(E);
        if (!BRep_Tool.Degenerated(E))
        {
            TopoDS_Wire WW = B.InternalWire();
            TopoDS_Shape NewEdge = new();
            // take the last edge added to WW
            for (TopoDS_Iterator Iter = new(WW); Iter.More(); Iter.Next())
            {
                NewEdge = Iter.Value();
            }
            if (!E.IsSame(NewEdge))
            {
                EEmap.Bind(E, NewEdge);
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
    private static void BuildFace(Geom_Surface S,
                                  in TopoDS_Edge E1,
                                  in TopoDS_Edge E2,
                                  in TopoDS_Edge E3,
                                  in TopoDS_Edge E4,
                                  NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> EEmap,
                                  bool ExchUV,
                                  bool UReverse,
                                  ref TopoDS_Face F)
    {
        double Tol;
        TopoDS_Edge e1, e2, E;
        TopoDS_Wire WW;
        BRep_Builder BB = new();
        BRepBuilderAPI_MakeWire B = new();

        // Is the surface planar ?
        double Tol1, Tol2, Tol3, Tol4;
        Tol1 = BRep_Tool.Tolerance(E1);
        Tol2 = BRep_Tool.Tolerance(E2);
        Tol3 = BRep_Tool.Tolerance(E3);
        Tol4 = BRep_Tool.Tolerance(E4);
        Tol = Min(Tol1, Tol2);
        Tol = Min(Tol, Min(Tol3, Tol4));
        bool IsPlan = false;
        Geom_Plane? thePlane = null;

        if (!E1.IsSame(E3) && !E2.IsSame(E4)) // exclude cases with seam edges: they are not planar
        {
            GeomLib_IsPlanarSurface IsP = new(S, Tol);
            if (IsP.IsPlanar())
            {
                IsPlan = true;
                thePlane = new Geom_Plane(IsP.Plan());
            }
            else
            {
                BRep_TEdge TE1 = (BRep_TEdge)E1.TShape()!;
                BRep_TEdge TE2 = (BRep_TEdge)E2.TShape()!;
                BRep_TEdge TE3 = (BRep_TEdge)E3.TShape()!;
                BRep_TEdge TE4 = (BRep_TEdge)E4.TShape()!;
                TE1.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                TE2.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                TE3.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                TE4.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());

                TopoDS_Wire theWire = new BRepLib_MakeWire(E1, E2, E3, E4).InternalWire();
                int NbPoints = NumberOfPoles(theWire);
                if (NbPoints <= 100) // limitation for CPU
                {
                    BRepLib_FindSurface FS = new(theWire, -1, true);
                    if (FS.Found())
                    {
                        IsPlan = true;
                        thePlane = FS.InternalSurface() as Geom_Plane;
                    }
                }
                BB.UpdateEdge(E1, Tol1);
                BB.UpdateEdge(E2, Tol2);
                BB.UpdateEdge(E3, Tol3);
                BB.UpdateEdge(E4, Tol4);
            }
        }

        // Construction of the wire
        e1 = E1;
        Oriente(S, ref e1);
        if (!IsPlan || !BRep_Tool.Degenerated(e1))
        {
            B.Add(e1);
        }

        e2 = E2;
        Oriente(S, ref e2);
        if (!IsPlan || !BRep_Tool.Degenerated(e2))
        {
            AddAndBind(B, e2, EEmap);
        }

        if (E3.IsSame(E1))
        {
            E = e1;
            E.Reverse();
        }
        else
        {
            E = E3;
            Oriente(S, ref E);
        }
        if (!IsPlan || !BRep_Tool.Degenerated(E))
        {
            AddAndBind(B, E, EEmap);
        }

        if (E4.IsSame(E2))
        {
            E = e2;
            E.Reverse();
        }
        else
        {
            E = E4;
            Oriente(S, ref E);
        }
        if (!IsPlan || !BRep_Tool.Degenerated(E))
        {
            AddAndBind(B, E, EEmap);
        }

        WW = B.InternalWire();

        // Construction of the face.
        if (IsPlan)
        { // Suspend representation 2d
          // and construct face Plane
            Point3D aPnt;
            Vector3D DU, DV, NS, NP;
            S.Bounds(out double Ufirst, out double Ulast, out double Vfirst, out double Vlast);
            S.D1((Ufirst + Ulast) / 2.0, (Vfirst + Vlast) / 2.0, out aPnt, out DU, out DV);
            NS = DU ^ DV;
            NP = new Vector3D(thePlane!.Pln().Axis().Direction());
            if (NS.Dot(NP) < 0.0)
            {
                if (thePlane.IsFrozen)
                {
                    // OCCT.NET: BRepLib_FindSurface can return the plane of a face already built (the edges' pcurves
                    // lie on it), and OCCT reverses that shared plane in place, changing the other face too
                    // (occt-issues.md). The port reverses a copy (decision 929).
                    thePlane = (Geom_Plane)thePlane.Copy();
                }
                thePlane.UReverse();
            }
            BRepLib_MakeFace MkF = new(thePlane, WW);
            if (MkF.Error() != BRepLib_FaceDone)
            {
            }
            else
            {
                Geom2d_Curve? NullC2d = null;
                Placement Loc = new();
                BB.UpdateEdge(E1, NullC2d, S, Loc, Tol1);
                BB.UpdateEdge(E2, NullC2d, S, Loc, Tol2);
                BB.UpdateEdge(E3, NullC2d, S, Loc, Tol3);
                BB.UpdateEdge(E4, NullC2d, S, Loc, Tol4);

                F = MkF.InternalFace();
                UpdateEdgeOnPlane(F, E1, BB);
                UpdateEdgeOnPlane(F, E2, BB);
                UpdateEdgeOnPlane(F, E3, BB);
                UpdateEdgeOnPlane(F, E4, BB);
            }
        }

        if (!IsPlan)
        { // Cas Standard : Ajout
            BB.MakeFace(ref F, S, global::Kerf.Engine.Foundation.Precision.Confusion());
            BB.Add(F, WW);
        }

        // Reorientation
        if (ExchUV)
        {
            F.Reverse();
        }
        if (UReverse)
        {
            F.Reverse();
        }
    }

    //=================================================================================================

    private static TopoDS_Edge BuildEdge(Geom_Curve? C3d,
                                         Geom2d_Curve C2d,
                                         Geom_Surface S,
                                         in TopoDS_Vertex VF,
                                         in TopoDS_Vertex VL,
                                         double f,
                                         double l,
                                         double Tol3d)
    {
        Point3D P;
        double Tol1, Tol2, Tol, d;
        BRep_Builder B = new();
        TopoDS_Edge E = new();

        Point3D P1 = BRep_Tool.Pnt(VF);
        Tol1 = BRep_Tool.Tolerance(VF);
        Point3D P2 = BRep_Tool.Pnt(VL);
        Tol2 = BRep_Tool.Tolerance(VL);
        Tol = Max(Tol1, Tol2);

        if (VF.IsSame(VL) || (P1.Distance(P2) < Tol))
        {
            // Degenerated case
            C2d.D0(f, out Point2D P2d);
            S.D0(P2d.X(), P2d.Y(), out P);
            d = P1.Distance(P);
            if (d > Tol)
            {
                Tol = d;
            }
            C2d.D0(l, out P2d);
            S.D0(P2d.X(), P2d.Y(), out P);
            d = P2.Distance(P);
            if (d > Tol)
            {
                Tol = d;
            }

            B.UpdateVertex(VF, Tol);
            B.UpdateVertex(VL, Tol);

            B.MakeEdge(ref E);
            B.UpdateEdge(E, C2d, S, new Placement(), Tol);
            B.Add(E, VF);
            B.Add(E, VL);
            B.Range(E, f, l);
            B.Degenerated(E, true);

            return E;
        }

        C3d!.D0(f, out P);
        d = P1.Distance(P);
        if (d > Tol1)
        {
            B.UpdateVertex(VF, d);
        }

        C3d.D0(l, out P);
        d = P2.Distance(P);
        if (d > Tol2)
        {
            B.UpdateVertex(VL, d);
        }

        BRepLib_MakeEdge MkE = new(C3d, VF, VL, f, l);
        if (!MkE.IsDone())
        { // Error of construction !!
        }

        E = MkE.InternalEdge();
        Placement Loc = new();
        B.UpdateEdge(E, C2d, S, Loc, Tol3d);

        IntTools_Context? aNullCtx = null;
        if (BOPTools_AlgoTools.IsMicroEdge(E, aNullCtx))
        {
            TopoDS_Vertex aV = VF;
            B.UpdateVertex(aV, P1.Distance(P2));
            B.MakeEdge(ref E);
            B.UpdateEdge(E, C2d, S, new Placement(), Tol);
            B.Add(E, TopoDS.Vertex(aV.Oriented(TopAbs_FORWARD)));
            B.Add(E, TopoDS.Vertex(aV.Oriented(TopAbs_REVERSED)));
            B.Range(E, f, l);
            B.Degenerated(E, true);
        }

        return E;
    }

    //=======================================================================
    // Fonction : Filling
    // Objet : Construct the faces of filling
    //=======================================================================
    private static bool Filling(in TopoDS_Shape EF,
                                in TopoDS_Shape F1,
                                in TopoDS_Shape EL,
                                in TopoDS_Shape F2,
                                NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> EEmap,
                                double Tol,
                                in Frame3D Axe,
                                in Vector3D TangentOnPart1,
                                ref TopoDS_Edge Aux1,
                                ref TopoDS_Edge Aux2,
                                ref TopoDS_Face Result)
    {
        BRep_Builder B = new();
        bool WithE3, WithE4;

        // Return constraints
        TopoDS_Vertex V1, V2, Vf, Vl;
        TopoDS_Edge E1, E2, E3 = new(), E4 = new();
        E1 = TopoDS.Edge(EF);
        E2 = TopoDS.Edge(EL);

        TopExp.Vertices(E1, out Vf, out Vl);
        Vf.Orientation(TopAbs_FORWARD);
        Vl.Orientation(TopAbs_FORWARD);

        TopExp.Vertices(E2, out V1, out V2);
        V1.Orientation(TopAbs_REVERSED);
        V2.Orientation(TopAbs_REVERSED);

        B.MakeEdge(ref E3);
        B.MakeEdge(ref E4);

        WithE3 = WithE4 = false;

        if ((!Aux1.IsNull()) && (!Vf.IsSame(V1)))
        {
            E3 = Aux1;
            WithE3 = true;
        }

        if (Vf.IsSame(Vl))
        {
            E4 = E3;
            E4.Reverse();
            WithE4 = WithE3;
        }
        else if (!Aux2.IsNull() && (!Vl.IsSame(V2)))
        {
            E4 = Aux2;
            WithE4 = true;
        }

        // Construction of a surface of revolution
        Geom_Curve Prof1, Prof2;
        double f1, f2, l1, l2, Angle;
        Prof1 = BRep_Tool.Curve(E1, out f1, out l1)!;
        Prof2 = BRep_Tool.Curve(E2, out f2, out l2)!;

        // Indeed, both Prof1 and Prof2 are the same curves but in different positions
        // Prof1's param domain may equals to Prof2's param domain *(-1), which means EF.Orientation() ==
        // EL.Orientation()
        bool bSameCurveDomain = EF.Orientation() != EL.Orientation();
        Point3D P1, P2, P;

        // Choose the angle of opening
        Transform3D aTf = new();
        aTf.SetTransformation(new CoordinateSystem(Axe));

        // Choose the furthest point from the "center of revolution"
        // to provide correct angle measurement.
        double[] aPrm = [f1, 0.5 * (f1 + l1), l1];
        Point3D[] aP1 = [Prof1.Value(aPrm[0]).Transformed(aTf),
                        Prof1.Value(aPrm[1]).Transformed(aTf),
                        Prof1.Value(aPrm[2]).Transformed(aTf)];

        int aMaxIdx = -1;
        double aMaxDist = RealFirst();
        for (int i = 0; i < 3; i++)
        {
            double aDist = aP1[i].X() * aP1[i].X() + aP1[i].Z() * aP1[i].Z();
            if (aDist > aMaxDist)
            {
                aMaxDist = aDist;
                aMaxIdx = i;
            }
        }

        double[] aPrm2 = [f2, 0.5 * (f2 + l2), l2];
        Point3D aP2 = Prof2.Value(aPrm2[bSameCurveDomain ? aMaxIdx : 2 - aMaxIdx]).Transformed(aTf);
        Vector2D aV1 = new(aP1[aMaxIdx].Z(), aP1[aMaxIdx].X());
        Vector2D aV2 = new(aP2.Z(), aP2.X());
        if (aV1.SquareMagnitude() <= Constants.Resolution() || aV2.SquareMagnitude() <= Constants.Resolution())
        {
            return false;
        }

        Angle = aV1.Angle(aV2);

        Axis3D axe = new(Axe.Location(), Axe.YDirection());

        if (Angle < 0)
        {
            Angle = -Angle;
            axe.Reverse();
        }

        Geom_SurfaceOfRevolution Rev = new(Prof1, axe);

        Geom_Surface Surf = new Geom_RectangularTrimmedSurface(Rev, 0, Angle, f1, l1);

        // Control the direction of the rotation
        bool ToReverseResult = false;
        Vector3D d1u;
        d1u = Surf.DN(0, aPrm[aMaxIdx], 1, 0);
        if (d1u.Angle(TangentOnPart1) > M_PI / 2)
        { // Invert everything
            ToReverseResult = true;
        }

        Geom2d_Curve C1, C2, C4;
        Geom2d_Curve? C3 = null;
        Geom2d_Line L;
        Point2D P2d = new(0.0, 0.0);

        L = new Geom2d_Line(P2d, Constants.DY2d());
        C1 = new Geom2d_TrimmedCurve(L, f1, l1);

        P2d.SetCoord(Angle, 0.0);
        L = new Geom2d_Line(P2d, Constants.DY2d());
        C2 = new Geom2d_TrimmedCurve(L, f1, l1);

        // It is required to control the direction and the range.
        C2.D0(f1, out P2d);
        Surf.D0(P2d.X(), P2d.Y(), out P1);
        C2.D0(l1, out P2d);
        Surf.D0(P2d.X(), P2d.Y(), out P2);
        P = BRep_Tool.Pnt(V1);
        if (P.Distance(P2) + Tol < P.Distance(P1))
        {
            // E2 is parsed in the direction opposite to E1
            C2.Reverse();
            TopoDS_Vertex aux;
            aux = V2;
            V2 = V1;
            V1 = aux;
        }
        GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(),
                          C2,
                          C2.FirstParameter(),
                          C2.LastParameter(),
                          f2,
                          l2,
                          ref C3);
        C2 = C3!;

        P2d.SetCoord(0.0, f1);
        L = new Geom2d_Line(P2d, Constants.DX2d());
        C3 = new Geom2d_TrimmedCurve(L, 0, Angle);

        P2d.SetCoord(0.0, l1);
        L = new Geom2d_Line(P2d, Constants.DX2d());
        C4 = new Geom2d_TrimmedCurve(L, 0, Angle);
        // (OCCT's commented-out deformation by GeomPlate and its unused points are left out.)

        // Update des Edges
        Placement Loc = new();
        Geom_Curve C3d;
        B.UpdateEdge(E1, C1, Surf, Loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
        B.UpdateEdge(E2, C2, Surf, Loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());

        if (E3.IsSame(E4))
        {
            if (!WithE3)
            {
                C3d = Surf.VIso(f1);
                E3 = BuildEdge(C3d, C3, Surf, Vf, V1, 0, Angle, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                BRepAdaptor_Curve aCurve = new(E3);
                double AngleOld = aCurve.LastParameter();
                if (Angle > AngleOld)
                {
                    B.Range(E3, 0, Angle);
                    TopoDS_Vertex V = TopExp.LastVertex(E3);
                    BRep_TVertex TVlast = (BRep_TVertex)V.TShape()!;
                    TVlast.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                }
            }

            B.UpdateEdge(E3, C3, C4, Surf, Loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            E4 = E3;
            E4.Reverse();
        }
        else
        {
            if (!WithE3)
            {
                C3d = Surf.VIso(f1);
                E3 = BuildEdge(C3d, C3, Surf, Vf, V1, 0, Angle, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                BRepAdaptor_Curve aCurve = new(E3);
                double AngleOld = aCurve.LastParameter();
                if (Angle > AngleOld)
                {
                    B.Range(E3, 0, Angle);
                    TopoDS_Vertex V = TopExp.LastVertex(E3);
                    BRep_TVertex TVlast = (BRep_TVertex)V.TShape()!;
                    TVlast.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                }

                B.UpdateEdge(E3, C3, Surf, Loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }

            if (!WithE4)
            {
                C3d = Surf.VIso(l1);
                E4 = BuildEdge(C3d, C4, Surf, Vl, V2, 0, Angle, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                BRepAdaptor_Curve aCurve = new(E4);
                double AngleOld = aCurve.LastParameter();
                if (Angle > AngleOld)
                {
                    B.Range(E4, 0, Angle);
                    TopoDS_Vertex V = TopExp.LastVertex(E4);
                    BRep_TVertex TVlast = (BRep_TVertex)V.TShape()!;
                    TVlast.Tolerance(global::Kerf.Engine.Foundation.Precision.Confusion());
                }

                B.UpdateEdge(E4, C4, Surf, Loc, /*Tol3d*/ global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        // Construct face
        BuildFace(Surf, E1, E3, E2, E4, EEmap, false, false, ref Result);

        // Set the continuities.
        B.Continuity(E1, TopoDS.Face(F1), Result, G1);
        B.Continuity(E2, TopoDS.Face(F2), Result, G1);

        // Render the calculated borders.
        if (!BRep_Tool.Degenerated(E3))
        {
            Aux1 = E3;
        }
        else
        {
            B.MakeEdge(ref Aux1); // Nullify
        }

        if (!BRep_Tool.Degenerated(E4))
        {
            Aux2 = E4;
        }
        else
        {
            B.MakeEdge(ref Aux2);
        }

        // Set the orientation

        // Provide correct normals computation
        // (the normal will be computed not in
        // singularity point definitely).
        Angle = RealFirst();
        for (int i = 0; i < 3; i++)
        {
            Vector3D D1U, D1V, N1, N2;
            C1.D0(aPrm[i], out P2d);
            Surf.D1(P2d.X(), P2d.Y(), out P, out D1U, out D1V);
            N1 = D1U ^ D1V;

            if (N1.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                continue;
            }

            C1 = BRep_Tool.CurveOnSurface(E1, TopoDS.Face(F1), out f2, out l2)!;
            C1.D0(aPrm[i], out P2d);
            BRepAdaptor_Surface AS = new(TopoDS.Face(F1));
            AS.D1(P2d.X(), P2d.Y(), out P, out D1U, out D1V);
            N2 = D1U ^ D1V;

            if (N2.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                continue;
            }

            Angle = N1.Angle(N2);

            break;
        }

        if (Angle == RealFirst())
        {
            return false;
        }

        if ((F1.Orientation() == TopAbs_REVERSED) ^ (Angle > M_PI / 2))
        {
            Result.Orientation(TopAbs_REVERSED);
        }
        else
        {
            Result.Orientation(TopAbs_FORWARD);
        }

        if (ToReverseResult)
        {
            Result.Reverse();
        }

        return true;
    }

    //=================================================================================================

    private static void Substitute(BRepTools_Substitution aSubstitute, in TopoDS_Edge Old, in TopoDS_Edge New)
    {
        NCollection_List<TopoDS_Shape> listShape = new();

        TopExp.Vertices(Old, out TopoDS_Vertex OldV1, out TopoDS_Vertex OldV2);
        TopExp.Vertices(New, out TopoDS_Vertex NewV1, out TopoDS_Vertex NewV2);

        if (!aSubstitute.IsCopied(OldV1))
        {
            listShape.Append(NewV1.Oriented(TopAbs_FORWARD));
            aSubstitute.Substitute(OldV1, listShape);
            listShape.Clear();
        }
        if (!aSubstitute.IsCopied(OldV2))
        {
            listShape.Append(NewV2.Oriented(TopAbs_FORWARD));
            aSubstitute.Substitute(OldV2, listShape);
            listShape.Clear();
        }
        if (!aSubstitute.IsCopied(Old))
        {
            listShape.Append(New.Oriented(TopAbs_FORWARD));
            aSubstitute.Substitute(Old, listShape);
        }
    }

    //=======================================================================
    // Fonction : KeepEdge
    // Objet : Find edges of the face supported by the same Curve.
    //=======================================================================
    private static void KeepEdge(in TopoDS_Shape Face, in TopoDS_Shape Edge, NCollection_List<TopoDS_Shape> List)
    {
        List.Clear();
        TopExp_Explorer Exp = new(Face, TopAbs_EDGE);
        Geom_Curve? Cref, C;
        Cref = BRep_Tool.Curve(TopoDS.Edge(Edge), out Placement Lref, out _, out _);

        for (; Exp.More(); Exp.Next())
        {
            C = BRep_Tool.Curve(TopoDS.Edge(Exp.Current()), out Placement L, out _, out _);
            if (ReferenceEquals(Cref, C) && (Lref.IsEqual(L)))
            {
                List.Append(Exp.Current());
            }
        }
    }

    //=================================================================================================

    private static void BuildVertex(Geom_Curve Iso, bool isfirst, double First, double Last, ref TopoDS_Shape Vertex)
    {
        BRep_Builder B = new();
        double val;

        if (isfirst)
        {
            val = First;
        }
        else
        {
            val = Last;
        }
        TopoDS_Vertex aV = new();
        B.MakeVertex(ref aV, Iso.Value(val), global::Kerf.Engine.Foundation.Precision.Confusion());
        Vertex = aV;
    }

    //=================================================================================================

    private static TopoDS_Edge NullEdge(ref TopoDS_Shape Vertex)
    {
        TopoDS_Edge E = new();
        BRep_Builder B = new();
        B.MakeEdge(ref E);
        Vertex.Orientation(TopAbs_FORWARD);
        B.Add(E, Vertex);
        B.Add(E, Vertex.Reversed());
        B.Degenerated(E, true);
        return E;
    }

    //=================================================================================================

    private static TopoDS_Edge BuildEdge(Geom_Surface S,
                                         bool isUiso,
                                         double ValIso,
                                         in TopoDS_Shape VFirst,
                                         in TopoDS_Shape VLast,
                                         double Tol)
    {
        TopoDS_Edge E;
        BRep_Builder B = new();
        Geom_Curve Iso;
        bool sing = false;
        if (isUiso)
        {
            Iso = S.UIso(ValIso);
        }
        else
        {
            Iso = S.VIso(ValIso);
        }

        if (VFirst.IsSame(VLast))
        { // Singular case ?
            TopoDS_Vertex V = TopoDS.Vertex(VFirst);
            double tol = BRep_Tool.Tolerance(V);
            if (Tol > tol)
            {
                tol = Tol;
            }
            Iso.D0((Iso.FirstParameter() + Iso.LastParameter()) / 2, out Point3D P);
            if (P.Distance(BRep_Tool.Pnt(V)) < tol)
            {
                GeomAdaptor_Curve AC = new(Iso);
                sing = GCPnts_AbscissaPoint.Length(AC, tol / 4) < tol;
            }
        }

        if (sing)
        { // Singular case
            TopoDS_Shape V;
            V = VFirst;
            E = NullEdge(ref V);
            B.Degenerated(E, true);
        }

        else
        {
            // Construction Via 3d
            Point3D P1, P2;
            double p1, p2, p11, p12, p21, p22;
            bool fwd = false;
            p1 = Iso.FirstParameter();
            p2 = Iso.LastParameter();
            P1 = Iso.Value(p1);
            P2 = Iso.Value(p2);

            double t1 = BRep_Tool.Tolerance(TopoDS.Vertex(VFirst));
            double t2 = BRep_Tool.Tolerance(TopoDS.Vertex(VLast));

            BRep_Builder BB = new();

            p11 = P1.Distance(BRep_Tool.Pnt(TopoDS.Vertex(VFirst)));
            p22 = P2.Distance(BRep_Tool.Pnt(TopoDS.Vertex(VLast)));
            p12 = P1.Distance(BRep_Tool.Pnt(TopoDS.Vertex(VLast)));
            p21 = P2.Distance(BRep_Tool.Pnt(TopoDS.Vertex(VFirst)));

            if (p11 < p12 && p22 < p21)
            {
                fwd = true;
            }

            if (fwd)
            { // OCC500(apo)
                if (p11 >= t1)
                {
                    BB.UpdateVertex(TopoDS.Vertex(VFirst), 1.01 * p11);
                }
                if (p22 >= t2)
                {
                    BB.UpdateVertex(TopoDS.Vertex(VLast), 1.01 * p22);
                }
            }
            else
            {
                if (p12 >= t2)
                {
                    BB.UpdateVertex(TopoDS.Vertex(VLast), 1.01 * p12);
                }
                if (p21 >= t1)
                {
                    BB.UpdateVertex(TopoDS.Vertex(VFirst), 1.01 * p21);
                }
            }

            BRepLib_MakeEdge MkE = new();

            if (fwd)
            {
                MkE.Init(Iso,
                         TopoDS.Vertex(VFirst),
                         TopoDS.Vertex(VLast),
                         Iso.FirstParameter(),
                         Iso.LastParameter());
            }
            else
            {
                MkE.Init(Iso,
                         TopoDS.Vertex(VLast),
                         TopoDS.Vertex(VFirst),
                         Iso.FirstParameter(),
                         Iso.LastParameter());
            }

            if (!MkE.IsDone())
            { // Erreur de construction !!
                throw new ConstructionErrorException("BRepFill_Sweep::BuildEdge");
            }

            E = MkE.InternalEdge();
        }

        // Associate 2d
        Geom2d_Line L;
        Placement Loc = new();
        S.Bounds(out double Umin, out _, out double Vmin, out _);
        if (isUiso)
        {
            Point2D P = new(ValIso, Vmin - Iso.FirstParameter());
            Vector2D V = new(0.0, 1.0);
            L = new Geom2d_Line(P, new Direction2D(V));
        }
        else
        {
            Point2D P = new(Umin - Iso.FirstParameter(), ValIso);
            Vector2D V = new(1.0, 0.0);
            L = new Geom2d_Line(P, new Direction2D(V));
        }

        B.UpdateEdge(E, L, S, Loc, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (sing)
        {
            B.Range(E, S, Loc, Iso.FirstParameter(), Iso.LastParameter());
        }

        double MaxTol = 1.0e-4;
        GeomAdaptor_Curve GAiso = new(Iso);
        GeomAdaptor_Curve GAHiso = GAiso;
        GeomAdaptor_Surface GAsurf = new(S);
        GeomAdaptor_Surface GAHsurf = GAsurf;
        CheckSameParameter(GAHiso, L, GAHsurf, MaxTol, out double theTol);
        B.UpdateEdge(E, theTol);

        return E;
    }

    //=================================================================================================

    private static void UpdateEdge(in TopoDS_Edge E, Geom_Surface S, bool isUiso, double ValIso)
    {
        BRep_Builder B = new();
        Geom2d_Line L;
        Geom2d_Curve? PCurve;
        Geom2d_Curve CL;
        Placement Loc = new();
        double F2d, L2d;
        S.Bounds(out double UFirst, out double ULast, out double VFirst, out double VLast);

        bool sing = false;
        Geom_Curve Iso;
        if (isUiso)
        {
            Iso = S.UIso(ValIso);
        }
        else
        {
            Iso = S.VIso(ValIso);
        }

        TopExp.Vertices(E, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
        if (Vf.IsSame(Vl))
        { // Singular case ?
            double tol = BRep_Tool.Tolerance(Vf);
            Iso.D0((Iso.FirstParameter() + Iso.LastParameter()) / 2, out Point3D Pmid);
            if (Pmid.Distance(BRep_Tool.Pnt(Vf)) < tol)
            {
                GeomAdaptor_Curve AC = new(Iso);
                sing = GCPnts_AbscissaPoint.Length(AC, tol / 4) < tol;
            }
        }

        if (isUiso)
        {
            Point2D P = new(ValIso, 0);
            Vector2D V = new(0.0, 1.0);
            L = new Geom2d_Line(P, new Direction2D(V));
            F2d = VFirst;
            L2d = VLast;
        }
        else
        {
            Point2D P = new(0.0, ValIso);
            Vector2D V = new(1.0, 0.0);
            L = new Geom2d_Line(P, new Direction2D(V));
            F2d = UFirst;
            L2d = ULast;
        }
        CL = new Geom2d_TrimmedCurve(L, F2d, L2d);

        // Control direction & Range
        double R, First, Last, Tol = 1.0e-4;
        bool reverse = false;

        Point3D POnS;
        Point2D P2d;
        BRep_Tool.Range(E, out First, out Last);

        if (!Vf.IsSame(Vl))
        {
            // Test distances between "FirstPoint" and "Vertex"
            P2d = CL.Value(F2d);
            POnS = S.Value(P2d.X(), P2d.Y());
            reverse = POnS.Distance(BRep_Tool.Pnt(Vl)) < POnS.Distance(BRep_Tool.Pnt(Vf));
        }
        else if (!sing)
        {
            // Test angle between "First Tangente"
            Vector3D V3d = new();
            BRepAdaptor_Curve C3d = new(E);

            C3d.D1(First, out POnS, out Vector3D dC3d);
            CL.D1(F2d, out P2d, out Vector2D V2d);
            S.D1(P2d.X(), P2d.Y(), out POnS, out Vector3D du, out Vector3D dv);
            V3d.SetLinearForm(V2d.X(), du, V2d.Y(), dv);
            reverse = (dC3d.Angle(V3d) > Tol);
        }
        if (reverse)
        { // Return curve 2d
            CL = new Geom2d_TrimmedCurve(L, F2d, L2d);
            CL.Reverse();
            F2d = CL.FirstParameter();
            L2d = CL.LastParameter();
        }

        if (sing)
        {
            Geom_Curve? NullCurve = null;
            B.UpdateEdge(E, NullCurve, 0.0);
            B.Degenerated(E, true);
            B.Range(E, F2d, L2d);
            First = F2d;
            Last = L2d;
        }

        if (First != F2d || Last != L2d)
        {
            Geom2d_Curve? C2d = null;
            GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(), CL, F2d, L2d, First, Last, ref C2d);
            CL = new Geom2d_TrimmedCurve(C2d!, First, Last);
        }

        // Update des Vertex

        TopoDS_Vertex V2;

        P2d = CL.Value(First);
        POnS = S.Value(P2d.X(), P2d.Y());
        V2 = TopExp.FirstVertex(E);
        R = POnS.Distance(BRep_Tool.Pnt(V2));
        B.UpdateVertex(V2, R);

        P2d = CL.Value(Last);
        POnS = S.Value(P2d.X(), P2d.Y());
        V2 = TopExp.LastVertex(E);
        R = POnS.Distance(BRep_Tool.Pnt(V2));
        B.UpdateVertex(V2, R);

        // Update Edge
        if (!sing && SameParameter(E, ref CL, S, Tol, out R))
        {
            B.UpdateEdge(E, R);
        }

        PCurve = Couture(E, S, Loc);
        if (PCurve is null)
        {
            B.UpdateEdge(E, CL, S, Loc, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        { // Sewing edge
            TopoDS_Edge e = E;
            Oriente(S, ref e);
            if (e.Orientation() == TopAbs_REVERSED)
            {
                B.UpdateEdge(E, CL, PCurve, S, Loc, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                B.UpdateEdge(E, PCurve, CL, S, Loc, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        // Attention to case not SameRange on its shapes (PRO13551)
        if (!BRep_Tool.SameRange(E))
        {
            B.Range(E, S, Loc, First, Last);
        }
    }

    //=======================================================================
    // Object : Check if a surface is degenerated
    //=======================================================================
    private static bool IsDegen(Geom_Surface S, double Tol)
    {
        int Nb = 5;
        bool B = true;
        double t, dt, l;
        int ii;
        Geom_Curve Iso;
        Point3D P1, P2, P3;

        S.Bounds(out double Umin, out double Umax, out double Vmin, out double Vmax);

        // Check the length of Iso-U
        t = (Umin + Umax) / 2;
        S.D0(t, Vmin, out P1);
        S.D0(t, (Vmin + Vmax) / 2, out P2);
        S.D0(t, Vmax, out P3);
        B = ((P1.Distance(P2) + P2.Distance(P3)) < Tol);

        for (ii = 1, dt = (Umax - Umin) / (Nb + 1); B && (ii <= Nb); ii++)
        {
            t = Umin + ii * dt;
            Iso = S.UIso(t);
            GeomAdaptor_Curve AC = new(Iso);
            l = GCPnts_AbscissaPoint.Length(AC, Tol / 4);
            B = (l <= Tol);
        }

        if (B)
        {
            return true;
        }

        // Check the length of Iso-V
        t = (Vmin + Vmax) / 2;
        S.D0(Umin, t, out P1);
        S.D0((Umin + Umax) / 2, t, out P2);
        S.D0(Umax, t, out P3);
        B = ((P1.Distance(P2) + P2.Distance(P3)) < Tol);

        for (ii = 1, dt = (Vmax - Vmin) / (Nb + 1); B && (ii <= Nb); ii++)
        {
            t = Vmin + ii * dt;
            Iso = S.VIso(t);
            GeomAdaptor_Curve AC = new(Iso);
            l = GCPnts_AbscissaPoint.Length(AC, Tol / 4);
            B = (l <= Tol);
        }

        return B;
    }

    private static void ReverseEdgeInFirstOrLastWire(in TopoDS_Shape theWire, in TopoDS_Shape theEdge)
    {
        TopoDS_Shape EdgeToReverse = new();
        TopoDS_Iterator itw = new(theWire);

        for (; itw.More(); itw.Next())
        {
            TopoDS_Shape anEdge = itw.Value();
            if (anEdge.IsSame(theEdge))
            {
                EdgeToReverse = anEdge;
                break;
            }
        }

        if (!EdgeToReverse.IsNull())
        {
            BRep_Builder BB = new();
            theWire.Free(true);
            BB.Remove(theWire, EdgeToReverse);
            EdgeToReverse.Reverse();
            BB.Add(theWire, EdgeToReverse);
        }
    }

    private static void ReverseModifiedEdges(in TopoDS_Wire theWire, NCollection_Map<TopoDS_Shape> theEmap)
    {
        if (theEmap.IsEmpty())
        {
            return;
        }

        TopoDS_Iterator itw = new(theWire);
        BRep_Builder BB = new();

        NCollection_List<TopoDS_Shape> Ledges = new();
        for (; itw.More(); itw.Next())
        {
            Ledges.Append(itw.Value());
        }

        theWire.Free(true);
        NCollection_List<TopoDS_Shape>.Iterator itl = new(Ledges);
        for (; itl.More(); itl.Next())
        {
            BB.Remove(theWire, itl.Value());
        }

        for (itl.Initialize(Ledges); itl.More(); itl.Next())
        {
            TopoDS_Shape anEdge = itl.Value();
            if (theEmap.Contains(anEdge))
            {
                anEdge.Reverse();
            }
            BB.Add(theWire, anEdge);
        }
    }

    //=================================================================================================

    /// <summary>The sweep of <paramref name="Section"/> along <paramref name="Location"/>.</summary>
    public BRepFill_Sweep(BRepFill_SectionLaw Section, BRepFill_LocationLaw Location, bool WithKPart)
    {
        isDone = false;
        KPart = WithKPart;
        mySec = Section;
        myLoc = Location;

        SetTolerance(1.0e-4);
        SetAngularControl();
        myAuxShape.Clear();

        myApproxStyle = GeomFill_Location;
        myContinuity = C2;
        myDegmax = 11;
        mySegmax = 30;
        myForceApproxC1 = false;
    }

    //=================================================================================================

    /// <summary>Sets the wires bounding the sweep at its start and end (null wires for none).</summary>
    public void SetBounds(in TopoDS_Wire First, in TopoDS_Wire Last)
    {
        FirstShape = First;
        LastShape = Last;

        // It is necessary to check the SameRange on its (PRO13551)
        BRep_Builder B = new();
        BRepTools_WireExplorer wexp = new();
        if (!FirstShape.IsNull())
        {
            for (wexp.Init(FirstShape); wexp.More(); wexp.Next())
            {
                if (!BRepLib.CheckSameRange(wexp.Current()))
                {
                    B.SameRange(wexp.Current(), false);
                    B.SameParameter(wexp.Current(), false);
                }
            }
        }

        if (!LastShape.IsNull())
        {
            for (wexp.Init(LastShape); wexp.More(); wexp.Next())
            {
                if (!BRepLib.CheckSameRange(wexp.Current()))
                {
                    B.SameRange(wexp.Current(), false);
                    B.SameParameter(wexp.Current(), false);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Sets the approximation tolerances: <paramref name="Tol3d"/> for the surfaces, <paramref name="Tol2d"/> for the
    /// curves, <paramref name="TolAngular"/> (radians) between the section law's tangents and the iso-v's.
    /// </summary>
    public void SetTolerance(double Tol3d, double BoundTol = 1.0, double Tol2d = 1.0e-5, double TolAngular = 1.0e-2)
    {
        myTol3d = Tol3d;
        myBoundTol = BoundTol;
        myTol2d = Tol2d;
        myTolAngular = TolAngular;
    }

    //=================================================================================================

    /// <summary>
    /// The control of the corners: below <paramref name="MinAngle"/> (radians) the transition is always "Modified".
    /// </summary>
    public void SetAngularControl(double MinAngle = 0.01, double MaxAngle = 6.0)
    {
        myAngMin = Max(MinAngle, global::Kerf.Engine.Foundation.Precision.Angular());
        myAngMax = Min(MaxAngle, 6.28);
    }

    //=======================================================================
    // function : SetForceApproxC1
    // purpose  : Set the flag that indicates attempt to approximate
    //           a C1-continuous surface if a swept surface proved
    //           to be C0.
    //=======================================================================
    /// <summary>Tries to approximate C1 surfaces where a swept surface is C0.</summary>
    public void SetForceApproxC1(bool ForceApproxC1)
    {
        myForceApproxC1 = ForceApproxC1;
    }

    ///=================================================================================================

    private bool CorrectApproxParameters()
    {
        TopoDS_Wire thePath = myLoc.Wire();
        Continuity NewCont = myContinuity;
        int NewSegmax = mySegmax;

        TopoDS_Iterator iter = new(thePath);
        for (; iter.More(); iter.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(iter.Value());
            BRepAdaptor_Curve aBAcurve = new(anEdge);
            Continuity aContinuity = aBAcurve.Continuity();
            int aNbInterv = aBAcurve.NbIntervals(CN);
            if (aContinuity < NewCont)
            {
                NewCont = aContinuity;
            }
            if (aNbInterv > NewSegmax)
            {
                NewSegmax = aNbInterv;
            }
        }

        bool Corrected = false;
        if (NewCont != myContinuity || NewSegmax != mySegmax)
        {
            Corrected = true;
        }
        myContinuity = NewCont;
        mySegmax = NewSegmax;
        return Corrected;
    }

    //=================================================================================================

    private bool BuildWire(BRepFill_TransitionStyle Transition)
    {
        _ = Transition;
        int ipath, isec = 1;
        Point3D P1;

        BRep_Builder B = new();
        int NbPath = myLoc.NbLaw();
        bool vclose;
        vclose = (myLoc.IsClosed() && (myLoc.IsG1(0, myTol3d) >= 0));
        Error = 0.0;
        Geom_Surface S;
        Geom_Curve Iso;
        double val, bid, First, Last, Tol;

        TopoDS_Wire wire = new();
        TopoDS_Edge E;
        B.MakeWire(ref wire);

        // (1) Construction of all curves

        // (1.1) Construction of Tables
        myFaces = new NCollection_HArray2<TopoDS_Shape>(1, 1, 1, NbPath);
        myUEdges = new NCollection_HArray2<TopoDS_Shape>(1, 2, 1, NbPath);
        myVEdges = new NCollection_HArray2<TopoDS_Shape>(1, 1, 1, NbPath + 1);

        // (1.2) Calculate curves / vertex / edge
        for (ipath = 1; ipath <= NbPath; ipath++)
        {
            // Curve by iso value
            GeomFill_Sweep Sweep = new(myLoc.Law(ipath), KPart);
            Sweep.SetTolerance(myTol3d, myBoundTol, myTol2d, myTolAngular);
            Sweep.SetForceApproxC1(myForceApproxC1);
            Sweep.Build(mySec.Law(isec), myApproxStyle, myContinuity, myDegmax, mySegmax);
            if (!Sweep.IsDone())
            {
                return false;
            }
            S = Sweep.InternalSurface()!;
            if (Sweep.ExchangeUV())
            {
                if (Sweep.UReversed())
                {
                    S.Bounds(out First, out Last, out bid, out val);
                }
                else
                {
                    S.Bounds(out First, out Last, out val, out bid);
                }
                Iso = S.VIso(val);
            }
            else
            {
                if (Sweep.UReversed())
                {
                    S.Bounds(out bid, out val, out First, out Last);
                }
                else
                {
                    S.Bounds(out val, out bid, out First, out Last);
                }
                Iso = S.UIso(val);
            }
            // Vertex by position
            if (ipath < NbPath)
            {
                BuildVertex(Iso, false, First, Last, ref myVEdges.ChangeValue(1, ipath + 1));
            }
            else
            {
                if (vclose)
                {
                    TopoDS_Vertex V = TopoDS.Vertex(myVEdges.Value(1, 1));
                    myVEdges.SetValue(1, ipath + 1, V);
                    Iso.D0(Last, out P1);
                    Tol = P1.Distance(BRep_Tool.Pnt(V));
                    B.UpdateVertex(V, Tol);
                }
                else
                {
                    if (!LastShape.IsNull())
                    {
                        myVEdges.SetValue(1, NbPath, FirstShape); // OCCT: FirstShape (a wire) at NbPath, not the last vertex
                    }
                    else
                    {
                        BuildVertex(Iso, false, First, Last, ref myVEdges.ChangeValue(1, NbPath + 1));
                    }
                }
            }

            if (ipath > 1)
            {
                Iso.D0(First, out P1);
                TopoDS_Vertex V = TopoDS.Vertex(myVEdges.Value(1, ipath));
                Tol = P1.Distance(BRep_Tool.Pnt(V));
                B.UpdateVertex(V, Tol);
            }
            if (ipath == 1)
            {
                if (!FirstShape.IsNull())
                {
                    myVEdges.SetValue(1, 1, FirstShape);
                }
                else
                {
                    BuildVertex(Iso, true, First, Last, ref myVEdges.ChangeValue(1, 1));
                }
            }

            // Construction of the edge
            BRepLib_MakeEdge MkE = new();
            MkE.Init(Iso,
                     TopoDS.Vertex(myVEdges.Value(1, ipath)),
                     TopoDS.Vertex(myVEdges.Value(1, ipath + 1)),
                     Iso.FirstParameter(),
                     Iso.LastParameter());
            if (!MkE.IsDone())
            { // Error of construction !!
                return false;
            }
            E = MkE.InternalEdge();
            B.UpdateEdge(E, Sweep.ErrorOnSurface());
            B.Add(wire, E);
            myFaces.SetValue(1, ipath, E);
        }
        myShape = wire;
        return true;
    }

    //=================================================================================================

    private bool BuildShell(BRepFill_TransitionStyle Transition,
                            int IFirst,
                            int ILast,
                            NCollection_Map<TopoDS_Shape> ReversedEdges,
                            NCollection_DataMap<TopoDS_Shape, NCollection_HArray2<TopoDS_Shape>> Tapes,
                            NCollection_DataMap<TopoDS_Shape, NCollection_HArray2<TopoDS_Shape>> Rails,
                            double ExtendFirst = 0.0,
                            double ExtendLast = 0.0)
    {
        _ = Transition;
        int ipath, isec, IPath;
        BRep_Builder B = new();
        int NbPath = ILast - IFirst;
        int NbLaw = mySec.NbLaw();
        bool uclose, vclose, global_vclose, constSection, hasdegen = false;
        constSection = mySec.IsConstant();
        uclose = mySec.IsUClosed();
        global_vclose = (myLoc.IsClosed()) && (myLoc.IsG1(0, myTol3d) >= 0);
        vclose = global_vclose && (mySec.IsVClosed()) && (NbPath == myLoc.NbLaw());
        Error = 0.0;

        // (1) Construction of all surfaces

        // (1.1) Construction of Tables

        NCollection_Array2<bool> ExchUV = new(1, NbLaw, 1, NbPath);
        NCollection_Array2<bool> UReverse = new(1, NbLaw, 1, NbPath);
        NCollection_Array2<bool> Degenerated = new(1, NbLaw, 1, NbPath);
        Degenerated.Init(false);
        // No VReverse for the moment...
        NCollection_Array2<double> TabErr = new(1, NbLaw, 1, NbPath);
        NCollection_Array2<Geom_Surface?> TabS = new(1, NbLaw, 1, NbPath);

        NCollection_Array2<TopoDS_Shape> UEdge = new(1, NbLaw + 1, 1, NbPath);
        NCollection_Array2<TopoDS_Shape> VEdge = new(1, NbLaw, 1, NbPath + 1);
        NCollection_Array2<TopoDS_Shape> Vertex = new(1, NbLaw + 1, 1, NbPath + 1);

        TopoDS_Vertex VNULL = new();
        VNULL.Nullify();
        Vertex.Init(VNULL);

        NCollection_Array1<TopoDS_Shape> SecVertex = new(1, NbLaw + 1);
        NCollection_Array1<double> VError = new(1, NbLaw + 1);
        NCollection_Array1<double> Vi = new(1, NbPath + 1);

        // Initialization of management of parametric intervals
        //(Case of evolutionary sections)
        double Length, SecDom, SecDeb;
        myLoc.CurvilinearBounds(myLoc.NbLaw(), out SecDom, out Length);
        mySec.Law(1).GetDomain(out SecDeb, out SecDom);
        SecDom -= SecDeb;
        if (IFirst > 1)
        {
            myLoc.CurvilinearBounds(IFirst - 1, out _, out double Ll);
            Vi[1] = SecDeb + (Ll / Length) * SecDom;
        }
        else
        {
            Vi[1] = SecDeb;
        }

        // Error a priori on vertices
        if (constSection)
        {
            for (isec = 1; isec <= NbLaw + 1; isec++)
            {
                VError[isec] = mySec.VertexTol(isec - 1, 0.0);
                SecVertex[isec] = mySec.Vertex(isec, 0.0);
            }
        }

        // (1.2) Calculate surfaces
        for (ipath = 1, IPath = IFirst; ipath <= NbPath; ipath++, IPath++)
        {
            GeomFill_Sweep Sweep = new(myLoc.Law(IPath), KPart);
            Sweep.SetTolerance(myTol3d, myBoundTol, myTol2d, myTolAngular);
            Sweep.SetForceApproxC1(myForceApproxC1);

            // Case of evolutionary section, definition of parametric correspondence
            if (!constSection)
            {
                myLoc.Law(IPath).GetDomain(out double lf, out double ll);
                myLoc.CurvilinearBounds(IPath, out _, out double Ll);
                Vi[ipath + 1] = SecDeb + (Ll / Length) * SecDom;
                Sweep.SetDomain(lf, ll, Vi[ipath], Vi[ipath + 1]);
            }
            else // section is constant
            {
                myLoc.Law(IPath).GetDomain(out _, out _);
                myLoc.CurvilinearBounds(IPath, out _, out double Ll);
                Vi[ipath + 1] = SecDeb + (Ll / Length) * SecDom;
            }

            for (isec = 1; isec <= NbLaw; isec++)
            {
                Sweep.Build(mySec.Law(isec), myApproxStyle, myContinuity, myDegmax, mySegmax);
                if (!Sweep.IsDone())
                {
                    return false;
                }
                TabS[isec, ipath] = Sweep.InternalSurface();
                TabErr[isec, ipath] = Sweep.ErrorOnSurface();
                ExchUV[isec, ipath] = Sweep.ExchangeUV();
                UReverse[isec, ipath] = Sweep.UReversed();
                if (Sweep.ErrorOnSurface() > Error)
                {
                    Error = Sweep.ErrorOnSurface();
                }

                if ((ipath == 1) && (ExtendFirst > 0))
                {
                    Geom_BoundedSurface BndS;
                    BndS = (Geom_BoundedSurface)TabS[isec, ipath]!;
                    GeomLib.ExtendSurfByLength(ref BndS, ExtendFirst, 1, Sweep.ExchangeUV(), false);
                    TabS[isec, ipath] = BndS;
                }
                if ((ipath == NbPath) && (ExtendLast > 0))
                {
                    Geom_BoundedSurface BndS;
                    BndS = (Geom_BoundedSurface)TabS[isec, ipath]!;
                    GeomLib.ExtendSurfByLength(ref BndS, ExtendLast, 1, Sweep.ExchangeUV(), true);
                    TabS[isec, ipath] = BndS;
                }
            }
        }

        // (2) Construction of Edges
        double UFirst, ULast, VFirst, VLast;
        bool exuv, singu, singv;
        Geom_Surface S;

        // Correct <FirstShape> and <LastShape>: reverse modified edges
        ReverseModifiedEdges(FirstShape, ReversedEdges);
        ReverseModifiedEdges(LastShape, ReversedEdges);

        // (2.0) return preexisting Edges and vertices
        TopoDS_Edge E;
        NCollection_Array1<bool> IsBuilt = new(1, NbLaw);
        IsBuilt.Init(false);
        NCollection_Array1<TopoDS_Shape> StartEdges = new(1, NbLaw);
        if (!FirstShape.IsNull() && (IFirst == 1))
        {
            mySec.Init(FirstShape);
            for (isec = 1; isec <= NbLaw; isec++)
            {
                E = mySec.CurrentEdge();
                TopExp.Vertices(E, out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast);
                VEdge[isec, 1] = E;
                if (E.Orientation() == TopAbs_REVERSED)
                {
                    Vertex[isec + 1, 1] = Vfirst; // TopExp::FirstVertex(E);
                }
                else
                {
                    Vertex[isec + 1, 1] = Vlast; // TopExp::LastVertex(E);
                }
                UpdateVertex(IFirst - 1, isec + 1, TabErr[isec, 1], Vi[1], Vertex[isec + 1, 1]);

                StartEdges[isec] = E;
                if (Tapes.IsBound(E))
                {
                    IsBuilt[isec] = true;

                    // Initialize VEdge, UEdge, Vertex and myFaces
                    int j;
                    NCollection_HArray2<TopoDS_Shape> aTape = Tapes.Find(E);
                    for (j = 1; j <= NbPath + 1; j++)
                    {
                        VEdge[isec, j] = aTape.Value(1, j);
                        VEdge[isec, j].Reverse(); // direction of round is reversed
                    }
                    int ifirst = isec + 1, ilast = isec; // direction of round is reversed
                    for (j = 1; j <= NbPath; j++)
                    {
                        UEdge[ifirst, j] = aTape.Value(2, j);
                    }
                    for (j = 1; j <= NbPath; j++)
                    {
                        UEdge[ilast, j] = aTape.Value(3, j);
                    }
                    for (j = 1; j <= NbPath + 1; j++)
                    {
                        Vertex[ifirst, j] = aTape.Value(4, j);
                    }
                    for (j = 1; j <= NbPath + 1; j++)
                    {
                        Vertex[ilast, j] = aTape.Value(5, j);
                    }
                    for (j = 1; j <= NbPath; j++)
                    {
                        myFaces!.SetValue(isec, j, aTape.Value(6, j));
                    }

                    if (uclose && isec == 1)
                    {
                        for (j = 1; j <= NbPath; j++)
                        {
                            UEdge[NbLaw + 1, j] = UEdge[1, j];
                        }
                        for (j = 1; j <= NbPath + 1; j++)
                        {
                            Vertex[NbLaw + 1, j] = Vertex[1, j];
                        }
                    }
                    if (uclose && isec == NbLaw)
                    {
                        for (j = 1; j <= NbPath; j++)
                        {
                            UEdge[1, j] = UEdge[NbLaw + 1, j];
                        }
                        for (j = 1; j <= NbPath + 1; j++)
                        {
                            Vertex[1, j] = Vertex[NbLaw + 1, j];
                        }
                    }
                }
                else
                {
                    NCollection_HArray2<TopoDS_Shape> EmptyArray = new(1, 6, 1, NbPath + 1);
                    Tapes.Bind(E, EmptyArray);
                    int j;
                    if (Rails.IsBound(Vfirst))
                    {
                        int ind = (E.Orientation() == TopAbs_REVERSED) ? isec + 1 : isec;
                        NCollection_HArray2<TopoDS_Shape> aRail = Rails.Find(Vfirst);
                        for (j = 1; j <= NbPath; j++)
                        {
                            UEdge[ind, j] = aRail.Value(1, j);
                        }
                        for (j = 1; j <= NbPath + 1; j++)
                        {
                            Vertex[ind, j] = aRail.Value(2, j);
                        }
                    }
                    if (Rails.IsBound(Vlast))
                    {
                        int ind = (E.Orientation() == TopAbs_FORWARD) ? isec + 1 : isec;
                        NCollection_HArray2<TopoDS_Shape> aRail = Rails.Find(Vlast);
                        for (j = 1; j <= NbPath; j++)
                        {
                            UEdge[ind, j] = aRail.Value(1, j);
                        }
                        for (j = 1; j <= NbPath + 1; j++)
                        {
                            Vertex[ind, j] = aRail.Value(2, j);
                        }
                    }
                }
            }

            if (VEdge[1, 1].Orientation() == TopAbs_REVERSED)
            {
                Vertex[1, 1] = TopExp.LastVertex(TopoDS.Edge(VEdge[1, 1]));
            }
            else
            {
                Vertex[1, 1] = TopExp.FirstVertex(TopoDS.Edge(VEdge[1, 1]));
            }
            UpdateVertex(IFirst - 1, 1, TabErr[1, 1], Vi[1], Vertex[1, 1]);
        }

        double u, v, aux;
        bool ureverse;
        for (isec = 1; isec <= NbLaw + 1; isec++)
        {
            // Return data
            if (isec > NbLaw)
            {
                S = TabS[NbLaw, 1]!;
                ureverse = UReverse[NbLaw, 1];
                exuv = ExchUV[NbLaw, 1];
            }
            else
            {
                S = TabS[isec, 1]!;
                ureverse = UReverse[isec, 1];
                exuv = ExchUV[isec, 1];
            }
            S.Bounds(out UFirst, out ULast, out VFirst, out VLast);

            // Choice of parameters
            if (ureverse)
            {
                if (exuv)
                {
                    aux = VFirst;
                    VFirst = VLast;
                    VLast = aux;
                }
                else
                {
                    aux = UFirst;
                    UFirst = ULast;
                    ULast = aux;
                }
            }
            if (isec != NbLaw + 1)
            {
                u = UFirst;
                v = VFirst;
            }
            else
            {
                if (exuv)
                {
                    u = UFirst;
                    v = VLast;
                }
                else
                {
                    u = ULast;
                    v = VFirst;
                }
            }

            // construction of vertices
            if (Vertex[isec, 1].IsNull())
            {
                TopoDS_Vertex aV = new();
                B.MakeVertex(ref aV, S.Value(u, v), mySec.VertexTol(isec - 1, Vi[1]));
                Vertex[isec, 1] = aV;
            }
            else
            {
                Placement Identity = new();
                Vertex[isec, 1].Location(Identity);
                B.UpdateVertex(TopoDS.Vertex(Vertex[isec, 1]), S.Value(u, v), mySec.VertexTol(isec - 1, Vi[1]));
            }
        } // end of for (isec=1; isec<=NbLaw+1; isec++)

        if (!LastShape.IsNull() && (ILast == myLoc.NbLaw() + 1))
        {
            mySec.Init(LastShape);
            for (isec = 1; isec <= NbLaw; isec++)
            {
                E = mySec.CurrentEdge();
                if (VEdge[isec, NbPath + 1].IsNull())
                {
                    VEdge[isec, NbPath + 1] = E;
                }

                if (Vertex[isec + 1, NbPath + 1].IsNull())
                {
                    if (VEdge[isec, NbPath + 1].Orientation() == TopAbs_REVERSED)
                    {
                        Vertex[isec + 1, NbPath + 1] = TopExp.FirstVertex(TopoDS.Edge(VEdge[isec, NbPath + 1]));
                    }
                    else
                    {
                        Vertex[isec + 1, NbPath + 1] = TopExp.LastVertex(TopoDS.Edge(VEdge[isec, NbPath + 1]));
                    }
                }
                UpdateVertex(ILast - 1, isec + 1, TabErr[isec, NbPath], Vi[NbPath + 1], Vertex[isec + 1, NbPath + 1]);
            }

            if (Vertex[1, NbPath + 1].IsNull())
            {
                if (VEdge[1, NbPath + 1].Orientation() == TopAbs_REVERSED)
                {
                    Vertex[1, NbPath + 1] = TopExp.LastVertex(TopoDS.Edge(VEdge[1, NbPath + 1]));
                }
                else
                {
                    Vertex[1, NbPath + 1] = TopExp.FirstVertex(TopoDS.Edge(VEdge[1, NbPath + 1]));
                }
            }
            UpdateVertex(ILast - 1, 1, TabErr[1, NbPath], Vi[NbPath + 1], Vertex[1, NbPath + 1]);
        }

        for (isec = 1; isec <= NbLaw + 1; isec++)
        {
            // Return data
            if (isec > NbLaw)
            {
                S = TabS[NbLaw, NbPath]!;
                ureverse = UReverse[NbLaw, NbPath];
                exuv = ExchUV[NbLaw, NbPath];
            }
            else
            {
                S = TabS[isec, NbPath]!;
                ureverse = UReverse[isec, NbPath];
                exuv = ExchUV[isec, NbPath];
            }
            S.Bounds(out UFirst, out ULast, out VFirst, out VLast);

            // Choice of parametres
            if (ureverse)
            {
                if (exuv)
                {
                    aux = VFirst;
                    VFirst = VLast;
                    VLast = aux;
                }
                else
                {
                    aux = UFirst;
                    UFirst = ULast;
                    ULast = aux;
                }
            }
            if (isec == NbLaw + 1)
            {
                u = ULast;
                v = VLast;
            }
            else
            {
                if (exuv)
                {
                    u = ULast;
                    v = VFirst;
                }
                else
                {
                    u = UFirst;
                    v = VLast;
                }
            }

            // construction of vertex
            if (Vertex[isec, NbPath + 1].IsNull())
            {
                TopoDS_Vertex aV = new();
                B.MakeVertex(ref aV, S.Value(u, v), mySec.VertexTol(isec - 1, Vi[NbPath + 1]));
                Vertex[isec, NbPath + 1] = aV;
            }
            else
            {
                Placement Identity = new();
                Vertex[isec, NbPath + 1].Location(Identity);
                B.UpdateVertex(TopoDS.Vertex(Vertex[isec, NbPath + 1]),
                               S.Value(u, v),
                               mySec.VertexTol(isec - 1, Vi[NbPath + 1]));
            }
        } // end of for (isec=1; isec<=NbLaw+1; isec++)

        // ---------- Creation of Vertex and edge ------------
        for (ipath = 1, IPath = IFirst; ipath <= NbPath; ipath++, IPath++)
        {
            for (isec = 1; isec <= NbLaw; isec++)
            {
                if (IsBuilt[isec])
                {
                    continue;
                }

                S = TabS[isec, ipath]!;
                exuv = ExchUV[isec, ipath];
                S.Bounds(out UFirst, out ULast, out VFirst, out VLast);
                if (UReverse[isec, ipath])
                {
                    double aux2;
                    if (exuv)
                    {
                        aux2 = VFirst;
                        VFirst = VLast;
                        VLast = aux2;
                    }
                    else
                    {
                        aux2 = UFirst;
                        UFirst = ULast;
                        ULast = aux2;
                    }
                }

                // (2.1) Construction of new vertices
                if (isec == 1)
                {
                    if (ipath == 1 && Vertex[1, 1].IsNull())
                    {
                        // All first
                        TopoDS_Vertex aV;
                        if (constSection)
                        {
                            myLoc.InternalPerformVertex(IPath - 1, TopoDS.Vertex(SecVertex[1]), VError[1], out aV);
                        }
                        else
                        {
                            myLoc.InternalPerformVertex(IPath - 1,
                                                        mySec.Vertex(1, Vi[1]),
                                                        mySec.VertexTol(0, Vi[1]),
                                                        out aV);
                        }
                        Vertex[1, 1] = aV;
                    }
                    // the first and the next column
                    if (vclose && (ipath == NbPath))
                    {
                        Vertex[1, ipath + 1] = Vertex[1, 1];
                    }
                    else if (Vertex[1, ipath + 1].IsNull())
                    {
                        TopoDS_Vertex aV;
                        if (constSection)
                        {
                            myLoc.InternalPerformVertex(IPath,
                                                        TopoDS.Vertex(SecVertex[1]),
                                                        TabErr[1, ipath] + VError[1],
                                                        out aV);
                        }
                        else
                        {
                            myLoc.InternalPerformVertex(IPath,
                                                        mySec.Vertex(1, Vi[ipath + 1]),
                                                        TabErr[1, ipath] + mySec.VertexTol(0, Vi[ipath + 1]),
                                                        out aV);
                        }
                        Vertex[1, ipath + 1] = aV;

                        if (MergeVertex(Vertex[1, ipath], ref Vertex[1, ipath + 1]))
                        {
                            UEdge[1, ipath] = NullEdge(ref Vertex[1, ipath]);
                        }
                    }
                }

                if (ipath == 1)
                {
                    if (uclose && (isec == NbLaw))
                    {
                        Vertex[isec + 1, 1] = Vertex[1, 1];
                    }
                    else if (Vertex[isec + 1, 1].IsNull())
                    {
                        TopoDS_Vertex aV;
                        if (constSection)
                        {
                            myLoc.InternalPerformVertex(IPath - 1,
                                                        TopoDS.Vertex(SecVertex[isec + 1]),
                                                        TabErr[isec, 1] + VError[isec + 1],
                                                        out aV);
                        }
                        else
                        {
                            myLoc.InternalPerformVertex(IPath - 1,
                                                        mySec.Vertex(isec + 1, Vi[1]),
                                                        TabErr[isec, 1] + mySec.VertexTol(isec, Vi[1]),
                                                        out aV);
                        }
                        Vertex[isec + 1, 1] = aV;

                        if (MergeVertex(Vertex[isec, 1], ref Vertex[isec + 1, 1]))
                        {
                            VEdge[isec, 1] = NullEdge(ref Vertex[isec, 1]);
                        }
                    }
                }

                if (uclose && (isec == NbLaw))
                {
                    Vertex[isec + 1, ipath + 1] = Vertex[1, ipath + 1];
                }
                else if (vclose && (ipath == NbPath))
                {
                    Vertex[isec + 1, ipath + 1] = Vertex[isec + 1, 1];
                }
                else if (Vertex[isec + 1, ipath + 1].IsNull())
                {
                    TopoDS_Vertex aV;
                    if (constSection)
                    {
                        myLoc.InternalPerformVertex(IPath,
                                                    TopoDS.Vertex(SecVertex[isec + 1]),
                                                    TabErr[isec, ipath] + VError[isec + 1],
                                                    out aV);
                    }
                    else
                    {
                        myLoc.InternalPerformVertex(IPath,
                                                    mySec.Vertex(isec + 1, Vi[ipath + 1]),
                                                    TabErr[isec, ipath] + mySec.VertexTol(isec, Vi[ipath + 1]),
                                                    out aV);
                    }
                    Vertex[isec + 1, ipath + 1] = aV;
                }

                // Singular cases
                singv = MergeVertex(Vertex[isec, ipath + 1], ref Vertex[isec + 1, ipath + 1]);
                singu = MergeVertex(Vertex[isec + 1, ipath], ref Vertex[isec + 1, ipath + 1]);

                if (singu || singv)
                {
                    Degenerated[isec, ipath] = IsDegen(TabS[isec, ipath]!, Max(myTol3d, TabErr[isec, ipath]));
                }
                if (Degenerated[isec, ipath])
                {
                    hasdegen = true;
                    // Particular construction of edges
                    if (UEdge[isec + 1, ipath].IsNull())
                    {
                        if (singu)
                        {
                            // Degenerated edge
                            UEdge[isec + 1, ipath] = NullEdge(ref Vertex[isec + 1, ipath]);
                        }
                        else
                        { // Copy the previous edge
                            UEdge[isec + 1, ipath] = UEdge[isec, ipath];
                        }
                    }
                    if (VEdge[isec, ipath + 1].IsNull())
                    {
                        if (singv)
                        {
                            // Degenerated Edge
                            VEdge[isec, ipath + 1] = NullEdge(ref Vertex[isec, ipath + 1]);
                        }
                        else
                        { // Copy the previous edge
                            VEdge[isec, ipath + 1] = VEdge[isec, ipath];
                        }
                    }
                }
                else
                { // Construction of edges by isos
                    if (exuv)
                    {
                        double UV;
                        UV = UFirst;
                        UFirst = VFirst;
                        VFirst = UV;
                        UV = ULast;
                        ULast = VLast;
                        VLast = UV;
                    }

                    // (2.2) Iso-u
                    if (isec == 1 && UEdge[1, ipath].IsNull())
                    {
                        if (!Vertex[1, ipath].IsSame(Vertex[1, ipath + 1]))
                        {
                            Point3D P1 = BRep_Tool.Pnt(TopoDS.Vertex(Vertex[1, ipath]));
                            Point3D P2 = BRep_Tool.Pnt(TopoDS.Vertex(Vertex[1, ipath + 1]));
                            if (P1.Distance(P2) <= myTol3d)
                            {
                                Vertex[1, ipath + 1] = Vertex[1, ipath];
                            }
                        }
                        UEdge[1, ipath] = BuildEdge(S, !exuv, UFirst, Vertex[1, ipath], Vertex[1, ipath + 1], myTol3d);
                    }
                    else
                    {
                        if (UEdge[isec, ipath].IsNull())
                        { // sweep failed
                            return false;
                        }
                        UpdateEdge(TopoDS.Edge(UEdge[isec, ipath]), S, !exuv, UFirst);
                    }

                    if (uclose && (isec == NbLaw))
                    {
                        if (UEdge[1, ipath].IsNull()) // degenerated case
                        {
                            UEdge[isec + 1, ipath] = BuildEdge(S,
                                                               !exuv,
                                                               ULast,
                                                               Vertex[isec + 1, ipath],
                                                               Vertex[isec + 1, ipath + 1],
                                                               myTol3d);
                        }
                        else
                        {
                            UpdateEdge(TopoDS.Edge(UEdge[1, ipath]), S, !exuv, ULast);
                            UEdge[isec + 1, ipath] = UEdge[1, ipath];
                        }
                    }
                    else
                    {
                        if (UEdge[isec + 1, ipath].IsNull())
                        {
                            UEdge[isec + 1, ipath] = BuildEdge(S,
                                                               !exuv,
                                                               ULast,
                                                               Vertex[isec + 1, ipath],
                                                               Vertex[isec + 1, ipath + 1],
                                                               myTol3d);
                        }
                        else
                        {
                            UpdateEdge(TopoDS.Edge(UEdge[isec + 1, ipath]), S, !exuv, ULast);
                        }
                    }

                    // (2.3) Iso-v
                    if (ipath == 1)
                    {
                        TopoDS_Edge aNewFirstEdge = BuildEdge(S, exuv, VFirst, Vertex[isec, 1], Vertex[isec + 1, 1], myTol3d);
                        if (VEdge[isec, ipath].IsNull())
                        {
                            VEdge[isec, ipath] = aNewFirstEdge;
                        }
                        else // rebuild first edge
                        {
                            TopoDS_Edge anEdge = TopoDS.Edge(VEdge[isec, ipath]);
                            RebuildTopOrBottomEdge(aNewFirstEdge, ref anEdge, ReversedEdges);
                            VEdge[isec, ipath] = anEdge;
                            if (ReversedEdges.Contains(VEdge[isec, ipath]))
                            {
                                ReverseEdgeInFirstOrLastWire(FirstShape, VEdge[isec, ipath]);
                                StartEdges[isec].Reverse();
                            }
                        }
                    }

                    else
                    {
                        UpdateEdge(TopoDS.Edge(VEdge[isec, ipath]), S, exuv, VFirst);
                    }

                    if (vclose && (ipath == NbPath))
                    {
                        if (VEdge[isec, 1].IsNull()) // degenerated case
                        {
                            VEdge[isec, ipath + 1] = BuildEdge(S,
                                                               exuv,
                                                               VLast,
                                                               Vertex[isec, ipath + 1],
                                                               Vertex[isec + 1, ipath + 1],
                                                               myTol3d);
                        }
                        else
                        {
                            UpdateEdge(TopoDS.Edge(VEdge[isec, 1]), S, exuv, VLast);
                            VEdge[isec, ipath + 1] = VEdge[isec, 1];
                        }
                    }
                    else if (VEdge[isec, ipath + 1].IsNull())
                    {
                        VEdge[isec, ipath + 1] = BuildEdge(S,
                                                           exuv,
                                                           VLast,
                                                           Vertex[isec, ipath + 1],
                                                           Vertex[isec + 1, ipath + 1],
                                                           myTol3d);
                    }
                    else
                    {
                        if (ipath != NbPath || vclose || (global_vclose && ILast == myLoc.NbLaw() + 1))
                        {
                            UpdateEdge(TopoDS.Edge(VEdge[isec, ipath + 1]), S, exuv, VLast);
                        }
                        else // ipath == NbPath && !vclose => rebuild last edge
                        {
                            TopoDS_Edge aNewLastEdge = BuildEdge(S,
                                                                 exuv,
                                                                 VLast,
                                                                 Vertex[isec, ipath + 1],
                                                                 Vertex[isec + 1, ipath + 1],
                                                                 myTol3d);
                            TopoDS_Edge anEdge = TopoDS.Edge(VEdge[isec, ipath + 1]);
                            RebuildTopOrBottomEdge(aNewLastEdge, ref anEdge, ReversedEdges);
                            VEdge[isec, ipath + 1] = anEdge;
                            if (ReversedEdges.Contains(VEdge[isec, ipath + 1]))
                            {
                                ReverseEdgeInFirstOrLastWire(LastShape, VEdge[isec, ipath + 1]);
                            }
                        }
                    }
                }
            } // End of construction of edges
        }

        // (3) Construction of Faces
        TopoDS_Face face = new();

        for (ipath = 1, IPath = IFirst; ipath <= NbPath; ipath++, IPath++)
        {
            for (isec = 1; isec <= NbLaw; isec++)
            {
                if (Degenerated[isec, ipath])
                {
                    if (UEdge[isec, ipath].IsSame(UEdge[isec + 1, ipath]))
                    {
                        myFaces!.SetValue(isec, IPath, UEdge[isec, ipath]);
                    }
                    else
                    {
                        myFaces!.SetValue(isec, IPath, VEdge[isec, ipath]);
                    }
                }
                else if (myFaces!.Value(isec, IPath).IsNull())
                {
                    BuildFace(TabS[isec, ipath]!,
                              TopoDS.Edge(UEdge[isec, ipath]),
                              TopoDS.Edge(VEdge[isec, ipath]),
                              TopoDS.Edge(UEdge[isec + 1, ipath]),
                              TopoDS.Edge(VEdge[isec, ipath + 1]),
                              myVEdgesModified,
                              ExchUV[isec, ipath],
                              UReverse[isec, ipath],
                              ref face);
                    myFaces.SetValue(isec, IPath, face);
                }
            }
        }

        // (3.1) Reverse the faces that have been built earlier
        for (ipath = 1; ipath <= NbPath; ipath++)
        {
            for (isec = 1; isec <= NbLaw; isec++)
            {
                if (IsBuilt[isec])
                {
                    myFaces!.ChangeValue(isec, ipath).Reverse();
                }
            }
        }

        // (4) History and Continuity

        if (hasdegen)
        {
            //(4.1) // Degenerated case => Sledgehammer
            TopoDS_Compound Comp = new();
            B.MakeCompound(ref Comp);
            for (isec = 1; isec <= NbLaw + 1; isec++)
            {
                for (ipath = 1, IPath = IFirst; ipath <= NbPath + 1; ipath++, IPath++)
                {
                    if (ipath <= NbPath)
                    {
                        myUEdges!.SetValue(isec, IPath, UEdge[isec, ipath]);
                    }
                    if (isec <= NbLaw)
                    {
                        myVEdges!.SetValue(isec, IPath, VEdge[isec, ipath]);
                    }
                    if ((ipath <= NbPath) && (isec <= NbLaw) && !myFaces!.Value(isec, IPath).IsNull()
                        && myFaces.Value(isec, IPath).ShapeType() == TopAbs_FACE)
                    {
                        B.Add(Comp, myFaces.Value(isec, IPath));
                    }
                }
            }
            BRepLib.EncodeRegularity(Comp, myTolAngular);
        }
        else
        {
            //(4.2) // General case => Tweezers
            bool isG1;
            TopoDS_Face FF;
            TopoDS_Edge anEdge;

            for (isec = 1; isec <= NbLaw + 1; isec++)
            {
                if (isec > 1)
                {
                    isG1 = (mySec.Continuity(isec - 1, myTolAngular) >= G1);
                }
                else
                {
                    isG1 = false;
                }
                for (ipath = 1, IPath = IFirst; ipath <= NbPath; ipath++, IPath++)
                {
                    myUEdges!.SetValue(isec, IPath, UEdge[isec, ipath]);
                    if (isG1)
                    {
                        if (isec == NbLaw + 1)
                        {
                            FF = TopoDS.Face(myFaces!.Value(1, IPath));
                        }
                        else
                        {
                            FF = TopoDS.Face(myFaces!.Value(isec, IPath));
                        }
                        B.Continuity(TopoDS.Edge(myUEdges.Value(isec, IPath)),
                                     TopoDS.Face(myFaces.Value(isec - 1, IPath)),
                                     FF,
                                     G1);
                    }
                }
            }

            int nbpath = NbPath;
            if (vclose)
            {
                nbpath++; // Another test G1
            }
            for (ipath = 1, IPath = IFirst; ipath <= NbPath + 1; ipath++, IPath++)
            {
                if ((ipath > 1) && (ipath <= nbpath))
                {
                    isG1 = (myLoc.IsG1(IPath - 1, myTol3d, myTolAngular) >= 0);
                }
                else
                {
                    isG1 = false;
                }
                for (isec = 1; isec <= NbLaw; isec++)
                {
                    myVEdges!.SetValue(isec, IPath, VEdge[isec, ipath]);
                    if (isG1)
                    {
                        if (ipath == NbPath + 1)
                        {
                            FF = TopoDS.Face(myFaces!.Value(isec, 1));
                        }
                        else
                        {
                            FF = TopoDS.Face(myFaces!.Value(isec, IPath));
                        }
                        anEdge = TopoDS.Edge(myVEdges.Value(isec, IPath));
                        BRepLib.EncodeRegularity(anEdge, FF, TopoDS.Face(myFaces.Value(isec, IPath - 1)), myTolAngular);
                    }
                }
            }
        }

        // (5) Update Tapes and Rails
        int jj;
        if (IFirst == 1 && !Tapes.IsEmpty()) // works only in case of single shell
        {
            for (isec = 1; isec <= NbLaw; isec++)
            {
                NCollection_HArray2<TopoDS_Shape> aTape = Tapes.Find(StartEdges[isec]);
                for (jj = 1; jj <= NbPath + 1; jj++)
                {
                    aTape.SetValue(1, jj, myVEdges!.Value(isec, jj));
                }
                for (jj = 1; jj <= NbPath; jj++)
                {
                    aTape.SetValue(2, jj, myUEdges!.Value(isec, jj));
                }
                for (jj = 1; jj <= NbPath; jj++)
                {
                    aTape.SetValue(3, jj, myUEdges!.Value(isec + 1, jj));
                }
                for (jj = 1; jj <= NbPath + 1; jj++)
                {
                    aTape.SetValue(4, jj, Vertex[isec, jj]);
                }
                for (jj = 1; jj <= NbPath + 1; jj++)
                {
                    aTape.SetValue(5, jj, Vertex[isec + 1, jj]);
                }
                for (jj = 1; jj <= NbPath; jj++)
                {
                    aTape.SetValue(6, jj, myFaces!.Value(isec, jj));
                }
                TopExp.Vertices(TopoDS.Edge(StartEdges[isec]), out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast, true); //with orientation
                if (!Rails.IsBound(Vfirst))
                {
                    NCollection_HArray2<TopoDS_Shape> anArray = new(1, 2, 1, NbPath + 1);
                    for (jj = 1; jj <= NbPath; jj++)
                    {
                        anArray.SetValue(1, jj, myUEdges!.Value(isec, jj));
                    }
                    for (jj = 1; jj <= NbPath + 1; jj++)
                    {
                        anArray.SetValue(2, jj, Vertex[isec, jj]);
                    }
                    Rails.Bind(Vfirst, anArray);
                }
                if (!Rails.IsBound(Vlast))
                {
                    NCollection_HArray2<TopoDS_Shape> anArray = new(1, 2, 1, NbPath + 1);
                    for (jj = 1; jj <= NbPath; jj++)
                    {
                        anArray.SetValue(1, jj, myUEdges!.Value(isec + 1, jj));
                    }
                    for (jj = 1; jj <= NbPath + 1; jj++)
                    {
                        anArray.SetValue(2, jj, Vertex[isec + 1, jj]);
                    }
                    Rails.Bind(Vlast, anArray);
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
    /// Builds the sweep: <paramref name="Transition"/> treats the corners, <paramref name="Approx"/> chooses what is
    /// approximated (the composed function, or the location law only), <paramref name="Continuity"/>,
    /// <paramref name="Degmax"/> and <paramref name="Segmax"/> bound the surfaces in v. <paramref name="ReversedEdges"/>,
    /// <paramref name="Tapes"/> and <paramref name="Rails"/> carry what earlier sweeps of the same pipe built.
    /// </summary>
    public void Build(NCollection_Map<TopoDS_Shape> ReversedEdges,
                      NCollection_DataMap<TopoDS_Shape, NCollection_HArray2<TopoDS_Shape>> Tapes,
                      NCollection_DataMap<TopoDS_Shape, NCollection_HArray2<TopoDS_Shape>> Rails,
                      BRepFill_TransitionStyle Transition = BRepFill_Modified,
                      Continuity Continuity = C2,
                      GeomFill_ApproxStyle Approx = GeomFill_Location,
                      int Degmax = 11,
                      int Segmax = 30)
    {
        myContinuity = Continuity;
        myApproxStyle = Approx;
        myDegmax = Degmax;
        mySegmax = Segmax;

        CorrectApproxParameters();

        // Wire
        if (mySec.IsVertex())
        {
            isDone = BuildWire(Transition);
        }
        else
        { // Shell
            int NbTrous = myLoc.NbHoles(myTol3d), NbPath = myLoc.NbLaw(), NbLaw = mySec.NbLaw(), ii, jj, NbPart = 1;
            int ipath, isec;
            BRep_Builder B = new();
            myUEdges = new NCollection_HArray2<TopoDS_Shape>(1, NbLaw + 1, 1, NbPath);
            myVEdges = new NCollection_HArray2<TopoDS_Shape>(1, NbLaw, 1, NbPath + 1);
            myFaces = new NCollection_HArray2<TopoDS_Shape>(1, NbLaw, 1, NbPath);
            myTapes = new NCollection_HArray1<TopoDS_Shape>(1, NbLaw);
            BRep_Builder BB = new();
            for (int i = 1; i <= NbLaw; i++)
            {
                TopoDS_Shell aShell = new();
                BB.MakeShell(ref aShell);
                myTapes[i] = aShell;
            }
            NCollection_HArray2<TopoDS_Shape> Bounds = new(1, NbLaw, 1, 2);

            NCollection_HArray1<int>? Trous = null;

            if (NbTrous > 0)
            { // How many sub-parts ?
                Trous = new NCollection_HArray1<int>(1, NbTrous);
                myLoc.Holes(Trous.ChangeArray1());
                NbPart += NbTrous;
                if (Trous.Value(NbTrous) == NbPath + 1)
                {
                    NbPart--;
                }
            }
            if (NbPart == 1)
            { // This is done at once
                double Extend = 0.0;
                if (NbTrous == 1)
                {
                    Extend = EvalExtrapol(1, Transition);
                }
                isDone = BuildShell(Transition, 1, NbPath + 1, ReversedEdges, Tapes, Rails, Extend, Extend);
            }
            else
            { //  This is done piece by piece
                int IFirst = 1, ILast;
                for (ii = 1, isDone = true; ii <= NbPart && isDone; ii++)
                {
                    if (ii > NbTrous)
                    {
                        ILast = NbPath + 1;
                    }
                    else
                    {
                        ILast = Trous!.Value(ii);
                    }
                    isDone = BuildShell(Transition,
                                        IFirst,
                                        ILast,
                                        ReversedEdges,
                                        Tapes,
                                        Rails,
                                        EvalExtrapol(IFirst, Transition),
                                        EvalExtrapol(ILast, Transition));
                    if (IFirst > 1)
                    {
                        Translate(myVEdges, IFirst, Bounds, 2);
                        if (!PerformCorner(IFirst, Transition, Bounds))
                        {
                            isDone = false;
                            return;
                        }
                    }
                    IFirst = ILast;
                    Translate(myVEdges, IFirst, Bounds, 1);
                }
            }
            // Management of looping ends
            if ((NbTrous > 0) && (myLoc.IsClosed()) && (Trous!.Value(NbTrous) == NbPath + 1))
            {
                Translate(myVEdges, NbPath + 1, Bounds, 1);
                Translate(myVEdges, 1, Bounds, 2);
                if (!PerformCorner(1, Transition, Bounds))
                {
                    isDone = false;
                    return;
                }
                Translate(myVEdges, 1, myVEdges, NbPath + 1);
            }

            // Construction of the shell
            TopoDS_Shell shell = new();
            B.MakeShell(ref shell);
            int aNbFaces = 0;
            for (ipath = 1; ipath <= NbPath; ipath++)
            {
                for (isec = 1; isec <= NbLaw; isec++)
                {
                    TopoDS_Shape face = myFaces.Value(isec, ipath);
                    if (!face.IsNull() && (face.ShapeType() == TopAbs_FACE))
                    {
                        B.Add(shell, face);
                        aNbFaces++;
                    }
                }
            }

            if ((NbTrous > 0) ? (aNbFaces < NbLaw) : (aNbFaces == 0))
            {
                isDone = false;
                return;
            }

            NCollection_List<TopoDS_Shape>.Iterator It = new(myAuxShape);
            for (; It.More(); It.Next())
            {
                TopoDS_Shape face = It.Value();
                if (!face.IsNull() && (face.ShapeType() == TopAbs_FACE))
                {
                    B.Add(shell, face);
                }
            }
            // Set common Uedges to faces
            BRepTools_Substitution aSubstitute = new();
            // (OCCT's commented-out SetCommonEdgeInFace calls are left out.)
            // OCCT iterates myVEdgesModified in address order; the port in binding order (occt-issues.md)
            foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> mapit in myVEdgesModified)
            {
                TopoDS_Edge OldEdge = TopoDS.Edge(mapit.Key);
                TopoDS_Edge NewEdge = TopoDS.Edge(mapit.Value);
                Substitute(aSubstitute, OldEdge, NewEdge);
            }
            aSubstitute.Build(shell);
            if (aSubstitute.IsCopied(shell))
            {
                NCollection_List<TopoDS_Shape> listSh = aSubstitute.InternalCopy(shell);
                shell = TopoDS.Shell(listSh.First());
            }

            for (ii = myFaces.LowerRow(); ii <= myFaces.UpperRow(); ii++)
            {
                for (jj = myFaces.LowerCol(); jj <= myFaces.UpperCol(); jj++)
                {
                    TopoDS_Shape aLocalShape = myFaces.Value(ii, jj);

                    if (!aLocalShape.IsNull() && aSubstitute.IsCopied(aLocalShape))
                    {
                        NCollection_List<TopoDS_Shape> aList = aSubstitute.InternalCopy(aLocalShape);

                        if (!aList.IsEmpty())
                        {
                            myFaces[ii, jj] = aList.First();
                        }
                    }
                }
            }

            for (ii = myVEdges.LowerRow(); ii <= myVEdges.UpperRow(); ii++)
            {
                for (jj = myVEdges.LowerCol(); jj <= myVEdges.UpperCol(); jj++)
                {
                    TopoDS_Shape aLocalShape = myVEdges.Value(ii, jj);

                    if (!aLocalShape.IsNull() && aSubstitute.IsCopied(aLocalShape))
                    {
                        NCollection_List<TopoDS_Shape> aList = aSubstitute.InternalCopy(aLocalShape);

                        if (!aList.IsEmpty())
                        {
                            myVEdges[ii, jj] = aList.First();
                        }
                    }
                }
            }

            for (ii = myUEdges.LowerRow(); ii <= myUEdges.UpperRow(); ii++)
            {
                for (jj = myUEdges.LowerCol(); jj <= myUEdges.UpperCol(); jj++)
                {
                    TopoDS_Shape aLocalShape = myUEdges.Value(ii, jj);

                    if (!aLocalShape.IsNull() && aSubstitute.IsCopied(aLocalShape))
                    {
                        NCollection_List<TopoDS_Shape> aList = aSubstitute.InternalCopy(aLocalShape);

                        if (!aList.IsEmpty())
                        {
                            myUEdges[ii, jj] = aList.First();
                        }
                    }
                }
            }

            // Ensure Same Parameter on U-edges
            for (ii = myUEdges.LowerRow(); ii <= myUEdges.UpperRow(); ii++)
            {
                if (mySec.IsUClosed() && ii == myUEdges.UpperRow())
                {
                    continue;
                }
                for (jj = myUEdges.LowerCol(); jj <= myUEdges.UpperCol(); jj++)
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(myUEdges.Value(ii, jj));
                    if (anEdge.IsNull() || BRep_Tool.Degenerated(anEdge))
                    {
                        continue;
                    }
                    TopoDS_Face Face1 = new(), Face2 = new();
                    int i1 = ii - 1, i2 = ii;
                    if (i1 == 0 && mySec.IsUClosed())
                    {
                        i1 = myFaces.UpperRow();
                    }
                    if (i2 > myFaces.UpperRow())
                    {
                        i2 = 0;
                    }
                    if (i1 != 0)
                    {
                        TopoDS_Shape aShape1 = myFaces.Value(i1, jj);
                        if (aShape1.ShapeType() == TopAbs_FACE)
                        {
                            Face1 = TopoDS.Face(aShape1);
                        }
                    }
                    if (i2 != 0)
                    {
                        TopoDS_Shape aShape2 = myFaces.Value(i2, jj);
                        if (aShape2.ShapeType() == TopAbs_FACE)
                        {
                            Face2 = TopoDS.Face(aShape2);
                        }
                    }
                    if (!Face1.IsNull() && !Face2.IsNull())
                    {
                        CorrectSameParameter(anEdge, Face1, Face2);
                    }
                }
            }

            for (ii = 1; ii <= NbLaw; ii++)
            {
                for (jj = 1; jj <= NbPath; jj++)
                {
                    TopoDS_Shape aFace = myFaces.Value(ii, jj);
                    if (!aFace.IsNull() && aFace.ShapeType() == TopAbs_FACE)
                    {
                        BB.Add(myTapes[ii], aFace);
                    }
                }
            }

            // Is it Closed ?
            if (myLoc.IsClosed() && mySec.IsUClosed())
            {
                // Check
                bool closed = true;
                int iedge;
                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> EFmap = new(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapesAndAncestors(shell, TopAbs_EDGE, TopAbs_FACE, EFmap);

                for (iedge = 1; iedge <= EFmap.Extent() && closed; iedge++)
                {
                    TopoDS_Edge theEdge = TopoDS.Edge(EFmap.FindKey(iedge));
                    if (BRep_Tool.Degenerated(theEdge))
                    {
                        continue;
                    }
                    closed = (EFmap[iedge].Extent() > 1);
                }
                shell.Closed(closed);
            }
            myShape = shell;
        }
    }

    //=================================================================================================

    /// <summary>True if the sweep is built.</summary>
    public bool IsDone() => isDone;

    //=================================================================================================

    /// <summary>The swept shape (unfrozen: the sweep is a working object of the pipes).</summary>
    public TopoDS_Shape Shape() => myShape;

    //=================================================================================================

    /// <summary>The approximation error.</summary>
    public double ErrorOnSurface() => Error;

    //=================================================================================================

    /// <summary>The faces (or degenerated edges) of the sweep: row the section's edge, column the path's.</summary>
    public NCollection_HArray2<TopoDS_Shape>? SubShape() => myFaces;

    //=================================================================================================

    /// <summary>The edges between the faces along the path (u-edges).</summary>
    public NCollection_HArray2<TopoDS_Shape>? InterFaces() => myUEdges;

    //=======================================================================
    // function : Sections
    // purpose  : Edges or Face (or compound of 2) Transition between 2 sweepings
    //=======================================================================
    /// <summary>The section edges (or faces, or compounds of both, at corners) between the sweeps.</summary>
    public NCollection_HArray2<TopoDS_Shape>? Sections() => myVEdges;

    //=======================================================================
    // function : Tape
    // purpose  : returns the Tape corresponding to Index-th edge of section
    //=======================================================================
    /// <summary>The shell of the faces swept by the section's edge of rank <paramref name="Index"/>.</summary>
    public TopoDS_Shape Tape(int Index) => myTapes!.Value(Index);

    //=================================================================================================

    private bool PerformCorner(int Index, BRepFill_TransitionStyle Transition, NCollection_HArray2<TopoDS_Shape> Bounds)
    {
        if (Transition == BRepFill_Modified)
        {
            return true; // Do nothing.
        }

        const double anAngularTol = 0.025;

        BRepFill_TransitionStyle TheTransition = Transition;
        bool isTangent = false;
        double F, L;
        int I1, I2, ii;
        Point3D P1, P2;
        Vector3D T1, T2, Tang, Sortant = new();

        if (Index > 1)
        {
            I1 = Index - 1;
            I2 = Index;
        }
        else
        {
            I1 = myLoc.NbLaw();
            I2 = 1;
        }

        // Construct an axis supported by the bissectrice
        myLoc.Law(I1).GetDomain(out F, out L);
        myLoc.Law(I1).GetCurve()!.D1(L, out P1, out T1);
        T1.Normalize();

        myLoc.Law(I2).GetDomain(out F, out L);
        myLoc.Law(I2).GetCurve()!.D1(F, out P2, out T2);
        T2.Normalize();

        if (T1.Angle(T2) < myAngMin)
        {
            isTangent = true;
            Vector3D t1, t2, V = new();
            Matrix3x3 M = new();
            myLoc.Law(I1).GetDomain(out F, out L);
            myLoc.Law(I1).D0(L, ref M, ref V);
            t1 = new Vector3D(M.Column(3));
            myLoc.Law(I2).GetDomain(out F, out L);
            myLoc.Law(I2).D0(L, ref M, ref V); // OCCT: the next law's frame at its end, not its start
            t2 = new Vector3D(M.Column(3));

            if (t1.Angle(t2) < myAngMin)
            {
                return true;
            }
            Sortant = t2 - t1;
        }

        if (T1.Angle(T2) >= M_PI - anAngularTol)
        {
            return false;
        }
        if ((TheTransition == BRepFill_Right) && (T1.Angle(T2) > myAngMax))
        {
            TheTransition = BRepFill_Round;
        }

        Tang = T1 + T2; // Average direction
        Direction3D NormalOfBisPlane = new(Tang);
        Vector3D anIntersectPointCrossDirection = T1.Crossed(T2);
        if (isTangent)
        {
            Sortant -= Tang.Dot(Tang) * Tang;
        }
        else
        {
            Sortant = T2 - T1; // Direction input
            Sortant *= -1; //   "   "   output
            Tang -= (Tang.Dot(T2)) * T2;
        }

        P1.BaryCenter(0.5, P2, 0.5);
        Direction3D N = new(Sortant);
        Direction3D Dx = new(Tang);

        Frame3D Axe = new(P1, N, Dx);
        Frame3D AxeOfBisPlane = new(P1, NormalOfBisPlane);

        // Construct 2 intersecting Shells
        NCollection_HArray2<TopoDS_Shape> UEdges = new(1, mySec.NbLaw() + 1, 1, myLoc.NbLaw());
        UEdges.ChangeArray2().Assign(myUEdges!.Array2());

        // modified by NIZHNY-MKK  Wed Oct 29 18:31:47 2003.BEGIN
        NCollection_HArray2<TopoDS_Shape> aFaces = new(myFaces!.LowerRow(), myFaces.UpperRow(), 1, 2);
        Translate(myFaces, I1, aFaces, 1);
        Translate(myFaces, I2, aFaces, 2);

        NCollection_HArray2<TopoDS_Shape> aUEdges = new(myUEdges.LowerRow(), myUEdges.UpperRow(), 1, 2);
        Translate(myUEdges, I1, aUEdges, 1);
        Translate(myUEdges, I2, aUEdges, 2);

        Vector3D aNormal = T2 + T1;
        TopoDS_Face aPlaneF = new();

        if (aNormal.Magnitude() > Constants.Resolution())
        {
            Plane3D pl = new(P1, new Direction3D(aNormal));
            BRepLib_MakeFace aFMaker = new(pl);

            if (aFMaker.Error() == BRepLib_FaceDone)
            {
                aPlaneF = aFMaker.InternalFace();
                BRep_Builder aBB = new();
                aBB.UpdateFace(aPlaneF, global::Kerf.Engine.Foundation.Precision.Confusion() * 10.0);
            }
        }

        BRepFill_TrimShellCorner aTrim = new(aFaces, Transition, AxeOfBisPlane, anIntersectPointCrossDirection);
        aTrim.AddBounds(Bounds);
        aTrim.AddUEdges(aUEdges);
        aTrim.AddVEdges(myVEdges!, Index);
        aTrim.Perform();

        if (aTrim.IsDone())
        {
            NCollection_List<TopoDS_Shape> listmodif = new();
            for (ii = 1; ii <= mySec.NbLaw(); ii++)
            {
                listmodif.Clear();
                aTrim.Modified(myVEdges!.Value(ii, Index), listmodif);

                if (listmodif.IsEmpty())
                {
                    TopoDS_Edge NullEdge = new();
                    myVEdges.SetValue(ii, Index, NullEdge);
                }
                else
                {
                    myVEdges.SetValue(ii, Index, listmodif.First());
                }
            }

            listmodif.Clear();
            int iit = 0;

            for (iit = 0; iit < 2; iit++)
            {
                int II = (iit == 0) ? I1 : I2;

                for (ii = 1; ii <= mySec.NbLaw(); ii++)
                {
                    aTrim.Modified(myFaces.Value(ii, II), listmodif);

                    if (!listmodif.IsEmpty())
                    {
                        myFaces.SetValue(ii, II, listmodif.First());
                    }
                }

                for (ii = myUEdges.LowerRow(); ii <= myUEdges.UpperRow(); ii++)
                {
                    aTrim.Modified(myUEdges.Value(ii, II), listmodif);

                    if (!listmodif.IsEmpty())
                    {
                        myUEdges.SetValue(ii, II, listmodif.First());
                    }
                }
            }
        }
        else if ((TheTransition == BRepFill_Right) || aTrim.HasSection())
        {
            return true; // Nothing is touched
        }

        if (mySec.IsUClosed())
        {
            myUEdges.SetValue(1, I1, myUEdges.Value(mySec.NbLaw() + 1, I1));
            myUEdges.SetValue(1, I2, myUEdges.Value(mySec.NbLaw() + 1, I2));
        }

        if (TheTransition == BRepFill_Round)
        {
            // Filling
            NCollection_List<TopoDS_Shape> list1 = new(), list2 = new();
            TopoDS_Edge Bord1 = new(), Bord2 = new(), BordFirst = new();
            BordFirst.Nullify();
            Bord1.Nullify();
            Bord2.Nullify();
            bool HasFilling = false;
            TopoDS_Face FF = new();
            for (ii = 1; ii <= mySec.NbLaw(); ii++)
            {
                KeepEdge(myFaces.Value(ii, I1), Bounds.Value(ii, 1), list1);
                KeepEdge(myFaces.Value(ii, I2), Bounds.Value(ii, 2), list2);
                if (list1.Extent() == list2.Extent())
                {
                    NCollection_List<TopoDS_Shape>.Iterator It1 = new(list1);
                    NCollection_List<TopoDS_Shape>.Iterator It2 = new(list2);
                    bool B;
                    for (; It1.More(); It1.Next(), It2.Next())
                    {
                        if (HasFilling)
                        { // Transversal choice of constraints
                            TopoDS_Vertex VC = new();
                            TopoDS_Edge E = TopoDS.Edge(It1.Value());
                            TopoDS_Edge E1 = new(), E2 = new();
                            E1.Nullify();
                            E2.Nullify();
                            TopExp.Vertices(E, out TopoDS_Vertex VF, out _);
                            if (!Bord1.IsNull() && TopExp.CommonVertex(E, Bord1, ref VC))
                            {
                                if (VC.IsSame(VF))
                                {
                                    E1 = Bord1;
                                }
                                else
                                {
                                    E2 = Bord1;
                                }
                            }
                            if (!Bord2.IsNull() && TopExp.CommonVertex(E, Bord2, ref VC))
                            {
                                if (VC.IsSame(VF))
                                {
                                    E1 = Bord2;
                                }
                                else
                                {
                                    E2 = Bord2;
                                }
                            }
                            if (!BordFirst.IsNull() && TopExp.CommonVertex(E, BordFirst, ref VC))
                            {
                                if (VC.IsSame(VF))
                                {
                                    E1 = BordFirst;
                                }
                                else
                                {
                                    E2 = BordFirst;
                                }
                            }
                            Bord1 = E1;
                            Bord2 = E2;
                        }

                        // Filling
                        B = Filling(It1.Value(),
                                    myFaces.Value(ii, I1),
                                    It2.Value(),
                                    myFaces.Value(ii, I2),
                                    myVEdgesModified,
                                    myTol3d,
                                    Axe,
                                    T1,
                                    ref Bord1,
                                    ref Bord2,
                                    ref FF);

                        if (B)
                        {
                            myAuxShape.Append(FF);
                            BRep_Builder BB = new();
                            TopoDS_Shape aVshape = myVEdges!.Value(ii, I2);
                            TopoDS_Compound aCompound = new();
                            BB.MakeCompound(ref aCompound);
                            if (!aVshape.IsNull())
                            {
                                BB.Add(aCompound, aVshape);
                            }
                            BB.Add(aCompound, FF);
                            myVEdges[ii, I2] = aCompound;

                            BB.Add(myTapes![ii], FF);
                            HasFilling = true;
                        }
                        if (ii == 1)
                        {
                            BordFirst = Bord1;
                        }
                    }
                }
            }
        }
        return true;
    }

    //=================================================================================================

    private double EvalExtrapol(int Index, BRepFill_TransitionStyle Transition)
    {
        double Extrap = 0.0;
        if (Transition == BRepFill_Right)
        {
            int I1, I2;
            if ((Index == 1) || (Index == myLoc.NbLaw() + 1))
            {
                if (!myLoc.IsClosed() || !mySec.IsVClosed())
                {
                    return Extrap;
                }
                I1 = myLoc.NbLaw();
                I2 = 1;
            }
            else
            {
                I1 = Index - 1;
                I2 = Index;
            }

            Vector3D V1 = new(), V2 = new(), T1 = new(), T2 = new();
            Matrix3x3 M1 = new(), M2 = new();
            double R, f, l;

            myLoc.Law(I1).GetDomain(out f, out l);
            myLoc.Law(I1).D0(l, ref M1, ref V1);
            T1.SetXYZ(M1.Column(3));
            myLoc.Law(I2).GetDomain(out f, out l);
            myLoc.Law(I2).D0(f, ref M2, ref V2);
            T2.SetXYZ(M2.Column(3));

            double alpha = T1.Angle(T2);
            if ((alpha > myAngMax) || (alpha < myAngMin))
            {
                // Angle too great => No "straight" connection
                // Angle too small => No connection
                return Extrap; // = 0.0
            }

            GeomFill_SectionLaw Sec;
            Sec = mySec.ConcatenedLaw()!;

            // Calculating parameter U
            double U, Length, SecFirst, SecLen, Ll;
            myLoc.CurvilinearBounds(myLoc.NbLaw(), out _, out Length);
            mySec.Law(1).GetDomain(out SecFirst, out SecLen);
            SecLen -= SecFirst;
            myLoc.CurvilinearBounds(I1, out _, out Ll);
            U = SecFirst + (Ll / Length) * SecLen;

            Box box = new();
            Box(Sec, U, ref box);
            box.Get(out double Xmin, out double Ymin, out double Zmin, out double Xmax, out double Ymax, out double Zmax);

            R = Max(Max(Math.Abs(Xmin), Math.Abs(Xmax)), Max(Math.Abs(Ymin), Math.Abs(Ymax)));
            //  modified by NIZHNY-MKK  Fri Oct 31 18:57:51 2003
            double coef = 2.0;
            R *= coef;
            Extrap = Max(Math.Abs(Zmin), Math.Abs(Zmax)) + 100 * myTol3d;
            Extrap += R * Math.Tan(alpha / 2);
        }
        return Extrap;
    }

    //=======================================================================
    // function : MergeVertex
    // purpose  : Make V2 = V1 if V2 is too close to V1
    //======================================================================
    private bool MergeVertex(in TopoDS_Shape V1, ref TopoDS_Shape V2)
    {
        TopoDS_Vertex v1 = TopoDS.Vertex(V1);
        TopoDS_Vertex v2 = TopoDS.Vertex(V2);
        double tol;
        tol = Max(BRep_Tool.Tolerance(v1), BRep_Tool.Tolerance(v2));
        if (tol < myTol3d)
        {
            tol = myTol3d;
        }
        if (BRep_Tool.Pnt(v1).Distance(BRep_Tool.Pnt(v2)) <= tol)
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
    private void UpdateVertex(int ipath, int isec, double ErrApp, double Param, in TopoDS_Shape V)
    {
        TopoDS_Vertex TheV;
        TheV = TopoDS.Vertex(V);
        myLoc.InternalPerformVertex(ipath,
                                    mySec.Vertex(isec, Param),
                                    ErrApp + mySec.VertexTol(isec - 1, Param),
                                    out TopoDS_Vertex vv);
        Point3D P1, P2;
        P1 = BRep_Tool.Pnt(vv);
        P2 = BRep_Tool.Pnt(TheV);

        double Tol = BRep_Tool.Tolerance(vv);
        Tol += P1.Distance(P2);

        if (Tol > BRep_Tool.Tolerance(TheV))
        {
            BRep_Builder B = new();
            B.UpdateVertex(TheV, Tol);
        }
    }

    //=======================================================================
    // function : RebuildTopOrBottomEdge
    // purpose  : Rebuild v-iso edge of top or bottom section
    //           inserting new 3d and 2d curves taken from swept surfaces
    //======================================================================
    private void RebuildTopOrBottomEdge(in TopoDS_Edge aNewEdge, ref TopoDS_Edge anEdge, NCollection_Map<TopoDS_Shape> ReversedEdges)
    {
        Geom_Curve? aNewCurve = BRep_Tool.Curve(aNewEdge, out double fpar, out double lpar);
        Placement Identity = new();

        bool ToReverse = false;
        bool IsDegen = BRep_Tool.Degenerated(aNewEdge);
        if (IsDegen)
        {
            BRep_Tool.Range(aNewEdge, out fpar, out lpar);
        }
        else
        {
            TopExp.Vertices(anEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            if (!V1.IsSame(V2))
            {
                TopExp.Vertices(aNewEdge, out TopoDS_Vertex NewV1, out TopoDS_Vertex NewV2);
                V1.Location(Identity);
                if (!V1.IsSame(NewV1))
                {
                    if (V1.IsSame(NewV2))
                    {
                        ToReverse = true;
                    }
                    else
                    {
                        Point3D Pnt1 = BRep_Tool.Pnt(V1);
                        Point3D NewPnt1 = BRep_Tool.Pnt(NewV1);
                        double TolSum = BRep_Tool.Tolerance(V1) + BRep_Tool.Tolerance(NewV1);
                        if (!Pnt1.IsEqual(NewPnt1, TolSum))
                        {
                            ToReverse = true;
                        }
                    }
                }
            }
            else
            {
                Geom_Curve OldCurve = BRep_Tool.Curve(anEdge, out double OldFirst, out double OldLast)!;
                OldCurve.D1(0.5 * (OldFirst + OldLast), out _, out Vector3D OldD1);
                aNewCurve!.D1(0.5 * (fpar + lpar), out _, out Vector3D NewD1);
                if (OldD1 * NewD1 < 0.0)
                {
                    ToReverse = true;
                }
            }
        }

        anEdge.Location(Identity);
        BRep_TEdge TEdge = (BRep_TEdge)anEdge.TShape()!;
        TEdge.Tolerance(BRep_Tool.Tolerance(aNewEdge));
        BRep_Builder BB = new();
        BB.Range(anEdge, fpar, lpar);
        BB.UpdateEdge(anEdge, aNewCurve, global::Kerf.Engine.Foundation.Precision.Confusion());
        BRep_TEdge TE = (BRep_TEdge)aNewEdge.TShape()!;
        NCollection_List<BRep_CurveRepresentation> lcr = TE.Curves();
        NCollection_List<BRep_CurveRepresentation>.Iterator itrep = new(lcr);
        for (; itrep.More(); itrep.Next())
        {
            BRep_CurveRepresentation CurveRep = itrep.Value();
            if (CurveRep.IsCurveOnSurface())
            {
                BRep_GCurve GC = (BRep_GCurve)CurveRep;
                Geom2d_Curve? aPCurve = GC.PCurve();
                Geom_Surface? aSurf = GC.Surface();
                Placement aLoc = aNewEdge.Location() * GC.Location();
                BB.UpdateEdge(anEdge, aPCurve, aSurf, aLoc, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        anEdge.Free(true);
        TopExp.Vertices(anEdge, out TopoDS_Vertex W1, out TopoDS_Vertex W2);

        TopoDS_Shape anEdgeFORWARD = anEdge.Oriented(TopAbs_FORWARD);

        BB.Remove(anEdgeFORWARD, W1);
        BB.Remove(anEdgeFORWARD, W2);

        W1.Location(Identity);
        W2.Location(Identity);
        if (ToReverse)
        {
            W2.Orientation(TopAbs_FORWARD);
            W1.Orientation(TopAbs_REVERSED);
        }
        BB.Add(anEdgeFORWARD, W1);
        BB.Add(anEdgeFORWARD, W2);

        if (ToReverse)
        {
            anEdge.Reverse();
            ReversedEdges.Add(anEdge);
        }

        BB.Degenerated(anEdge, IsDegen);
    }
}
