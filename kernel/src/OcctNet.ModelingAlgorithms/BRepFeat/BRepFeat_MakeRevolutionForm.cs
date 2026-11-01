// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevolutionForm.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevolutionForm.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevolutionForm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Builds a rib or a groove along a surface of revolution: a contour (an open wire) in a plane through the axis,
/// revolved and thickened on both sides of the plane until it meets the basis shape.
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a groove (a cut), 1 for a rib (a fusion). With sliding, the rib slides on the faces of
/// revolution it meets.</para>
/// <para>The basis shape and the contour are read, never written (LocOpe_RevolutionForm sweeps a copy of a frozen
/// profile, decision 1014; the gluing and the same-parameter pass, BRepFeat_RibSlot's).</para>
/// </remarks>
public sealed class BRepFeat_MakeRevolutionForm : BRepFeat_RibSlot
{
    private gp_Ax1 myAxe = new();
    private double myHeight1;
    private double myHeight2;
    // OCCT: the class declares its own mySliding, hiding BRepFeat_RibSlot's, which stays false
    private new bool mySliding;
    private Geom_Plane? myPln;
    private double myBnd;
    private NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySlface = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_List<TopoDS_Shape> myListOfEdges = new();
    private double myTol;
    private double myAngle1;
    private double myAngle2;

    /// <summary>An empty revolution form.</summary>
    public BRepFeat_MakeRevolutionForm()
    {
        myHeight1 = 0.0;
        myHeight2 = 0.0;
        mySliding = false;
        myBnd = 0.0;
        myTol = 0.0;
        myAngle1 = RealLast();
        myAngle2 = RealLast();
    }

    /// <summary>
    /// A rib or groove of the contour <paramref name="W"/> in the plane <paramref name="Plane"/> on
    /// <paramref name="Sbase"/>, revolved around <paramref name="Axis"/>, of the thicknesses <paramref name="Height1"/>
    /// and <paramref name="Height2"/> on each side of the plane; <paramref name="Fuse"/> 0 cuts, 1 fuses;
    /// <paramref name="Sliding"/> asks for sliding (OCCT takes it by reference and never writes it).
    /// </summary>
    public BRepFeat_MakeRevolutionForm(in TopoDS_Shape Sbase, in TopoDS_Wire W, Geom_Plane Plane, in gp_Ax1 Axis,
                                       double Height1, double Height2, int Fuse, bool Sliding)
    {
        Init(Sbase, W, Plane, Axis, Height1, Height2, Fuse, Sliding);
    }

    /// <summary>Initializes the revolution form (see the constructor).</summary>
    public void Init(in TopoDS_Shape Sbase, in TopoDS_Wire W, Geom_Plane Plane, in gp_Ax1 Axis, double H1, double H2,
                     int Mode, bool Modify)
    {
        bool RevolRib = true;
        Done();

        // modify = 0 if it is not required to make sliding
        //        = 1 if it is intended to try to make sliding
        bool Sliding = Modify;

        myAxe = Axis;
        Geom_Line Line = new(Axis);
        double LineFirst, LineLast;

        LocOpe_CSIntersector ASI = new(Sbase);
        NCollection_Sequence<Geom_Curve?> scur = new();
        scur.Clear();
        scur.Append(Line);
        ASI.Perform(scur);
        if (ASI.IsDone() && ASI.NbPoints(1) >= 2)
        {
            LineLast = ASI.Point(1, ASI.NbPoints(1)).Parameter();
            LineFirst = ASI.Point(1, 1).Parameter();
        }
        else
        {
            LineFirst = RealFirst();
            LineLast = RealLast();
        }

        Geom2d_Curve ln2d = GeomAPI.To2d(Line, Plane.Pln())!;

        TopExp_Explorer exx = new();
        double Rad = RealLast();

        exx.Init(W, TopAbs_EDGE);
        for (; exx.More(); exx.Next())
        {
            TopoDS_Edge e = TopoDS.Edge(exx.Current());
            Geom_Curve c = BRep_Tool.Curve(e, out double f, out double l)!;
            Geom2d_Curve c2d = GeomAPI.To2d(c, Plane.Pln())!;
            Geom2dAPI_ExtremaCurveCurve extr = new(ln2d, c2d, LineFirst, LineLast, f, l);
            double L = RealLast();
            if (extr.NbExtrema() >= 1)
            {
                L = extr.LowerDistance();
            }
            gp_Pnt p1 = c.Value(f);
            gp_Pnt p2 = c.Value(l);
            GeomAPI_ProjectPointOnCurve proj1 = new(p1, Line);
            GeomAPI_ProjectPointOnCurve proj2 = new(p2, Line);
            if (proj1.NbPoints() < 1 || proj2.NbPoints() < 1)
            {
                myStatusError = BRepFeat_NoProjPt;
                NotDone();
                return;
            }
            double par1 = proj1.Distance(1);
            double par2 = proj2.Distance(1);
            double Par = Min(par1, par2);
            if (Par < L)
            {
                L = Par;
            }
            if (L < Rad && L > 0.0)
            {
                Rad = L;
            }
        }

        double height = Min(H1, H2);

        if (Rad <= height)
        {
            Rad = height + 0.01 * height;
        }

        myAngle1 = Math.Asin(H1 / Rad) + M_PI / 10.0;
        myAngle2 = Math.Asin(H2 / Rad) + M_PI / 10.0;

        if ((myAngle1 - M_PI / 2) > Precision.Confusion())
        {
            myAngle1 = M_PI / 2;
        }
        if ((myAngle2 - M_PI / 2) > Precision.Confusion())
        {
            myAngle2 = M_PI / 2;
        }

        mySkface.Nullify();
        myPbase.Nullify();

        myFuse = Mode != 0;

        // ---Determination Tolerance : tolerance max on parameters
        myTol = Precision.Confusion();

        exx.Init(W, TopAbs_VERTEX);
        for (; exx.More(); exx.Next())
        {
            double tol = BRep_Tool.Tolerance(TopoDS.Vertex(exx.Current()));
            if (tol > myTol)
            {
                myTol = tol;
            }
        }

        exx.Init(Sbase, TopAbs_VERTEX);
        for (; exx.More(); exx.Next())
        {
            double tol = BRep_Tool.Tolerance(TopoDS.Vertex(exx.Current()));
            if (tol > myTol)
            {
                myTol = tol;
            }
        }

        TopoDS_Shape aLocalShapeW = W.Oriented(TopAbs_FORWARD);
        myWire = TopoDS.Wire(aLocalShapeW);
        myPln = Plane;
        myHeight1 = H1;
        myHeight2 = H2;

        mySbase = Sbase;
        mySlface.Clear();
        myShape.Nullify();
        myMap.Clear();
        myFShape.Nullify();
        myLShape.Nullify();

        // ---Calculate bounding box
        BRep_Builder BB = new();

        NCollection_List<TopoDS_Shape> theList = new();

        TopoDS_Shape U = new();
        U.Nullify();
        gp_Pnt FirstCorner = new(), LastCorner = new();
        double bnd = HeightMax(mySbase, U, ref FirstCorner, ref LastCorner);
        myBnd = bnd;

        BRepPrimAPI_MakeBox Bndbox = new(FirstCorner, LastCorner);
        TopoDS_Solid BndBox = Bndbox.InternalSolid();

        // ---Construction of the working plane face (section bounding box)
        BRepLib_MakeFace PlaneF = new(myPln.Pln(), -6.0 * myBnd, 6.0 * myBnd, -6.0 * myBnd, 6.0 * myBnd);
        TopoDS_Face PlaneFace = TopoDS.Face(PlaneF.InternalShape());

        BRepAlgoAPI_Common PlaneS = new(BndBox, PlaneFace);
        TopExp_Explorer EXP = new();
        TopoDS_Shape PlaneSect = PlaneS.InternalShape();
        EXP.Init(PlaneSect, TopAbs_WIRE);
        TopoDS_Wire www = TopoDS.Wire(EXP.Current());
        BRepLib_MakeFace Bndface = new(myPln.Pln(), www, true);
        TopoDS_Face BndFace = TopoDS.Face(Bndface.InternalShape());

        // ---Find base faces of the rib
        TopoDS_Edge FirstEdge = new(), LastEdge = new();
        TopoDS_Face FirstFace = new(), LastFace = new();
        TopoDS_Vertex FirstVertex = new(), LastVertex = new();

        bool OnFirstFace = false;
        bool OnLastFace = false;
        bool PtOnFirstEdge = false;
        bool PtOnLastEdge = false;
        TopoDS_Edge OnFirstEdge = new(), OnLastEdge = new();
        OnFirstEdge.Nullify();
        OnLastEdge.Nullify();

        bool Data = ExtremeFaces(RevolRib, myBnd, myPln, ref FirstEdge, ref LastEdge, ref FirstFace, ref LastFace,
                                 ref FirstVertex, ref LastVertex, ref OnFirstFace, ref OnLastFace, ref PtOnFirstEdge,
                                 ref PtOnLastEdge, ref OnFirstEdge, ref OnLastEdge);

        if (!Data)
        {
            myStatusError = BRepFeat_NoExtFace;
            NotDone();
            return;
        }

        // ---Proofing Point for the side of the wire to be filled - material side
        gp_Pnt CheckPnt = CheckPoint(FirstEdge, bnd / 10.0, myPln);

        // ---Control sliding valid
        // Many cases when the sliding is abandoned
        int Concavite = 3; // a priori the profile is not concave

        myFirstPnt = BRep_Tool.Pnt(FirstVertex);
        myLastPnt = BRep_Tool.Pnt(LastVertex);

        // SliList : list of faces concerned by the rib
        NCollection_List<TopoDS_Shape> SliList = new();
        SliList.Append(FirstFace);

        if (Sliding)
        { // sliding
            Geom_Surface s = BRep_Tool.Surface(FirstFace)!;
            if (s.GetType() == typeof(Geom_RectangularTrimmedSurface))
            {
                s = ((Geom_RectangularTrimmedSurface)s).BasisSurface();
            }
            if (s.GetType() != typeof(Geom_Plane) && s.GetType() != typeof(Geom_CylindricalSurface)
                && s.GetType() != typeof(Geom_ConicalSurface) && s.GetType() != typeof(Geom_ToroidalSurface))
            {
                Sliding = false;
            }
        }

        if (Sliding)
        { // sliding
            Geom_Surface ss = BRep_Tool.Surface(LastFace)!;
            if (ss.GetType() == typeof(Geom_RectangularTrimmedSurface))
            {
                ss = ((Geom_RectangularTrimmedSurface)ss).BasisSurface();
            }
            if (ss.GetType() != typeof(Geom_Plane) && ss.GetType() != typeof(Geom_CylindricalSurface)
                && ss.GetType() != typeof(Geom_ConicalSurface) && ss.GetType() != typeof(Geom_ToroidalSurface))
            {
                Sliding = false;
            }
        }

        // Control only start and end points no control at the middle to improve
        // If make a control between Surface and segment 2 points limited
        // -> too expensive - to improve
        gp_Circ FirstCircle, LastCircle;
        Geom_Curve FirstCrv, LastCrv;

        if (Sliding)
        { // sliding
            GeomAPI_ProjectPointOnCurve proj = new(myFirstPnt, Line);
            if (proj.NbPoints() < 1)
            {
                myStatusError = BRepFeat_NoProjPt;
                NotDone();
                return;
            }
            double FirstRayon = proj.Distance(1);
            gp_Pnt FirstCenter = proj.Point(1);

            GeomAPI_ProjectPointOnCurve proj1 = new(myLastPnt, Line);
            if (proj.NbPoints() < 1) // OCCT: tests proj again, not proj1
            {
                myStatusError = BRepFeat_NoProjPt;
                NotDone();
                return;
            }
            double LastRayon = proj1.Distance(1);
            gp_Pnt LastCenter = proj1.Point(1);

            gp_Vec axv = new(myAxe.Direction());
            gp_Ax2 ax2 = new(FirstCenter, new gp_Dir(axv));
            gp_Ax2 ax2p = new(LastCenter, new gp_Dir(axv));
            gp_Circ theFC = new(ax2, FirstRayon);
            gp_Circ theLC = new(ax2p, LastRayon);

            gp_Pnt RFirstPnt1 = myFirstPnt.Rotated(myAxe, myAngle1);
            gp_Pnt RLastPnt1 = myLastPnt.Rotated(myAxe, myAngle1);
            gp_Pnt RFirstPnt2 = myFirstPnt.Rotated(myAxe, -myAngle2);
            gp_Pnt RLastPnt2 = myLastPnt.Rotated(myAxe, -myAngle2);

            BRep_Builder Bu = new();
            TopoDS_Vertex v1 = new(), v2 = new(), v3 = new(), v4 = new();
            Bu.MakeVertex(ref v1, RFirstPnt2, Precision.Confusion());
            Bu.MakeVertex(ref v2, RFirstPnt1, Precision.Confusion());
            Bu.MakeVertex(ref v3, RLastPnt2, Precision.Confusion());
            Bu.MakeVertex(ref v4, RLastPnt1, Precision.Confusion());

            BRepLib_MakeEdge ee1 = new(theFC, v1, v2);
            BRepLib_MakeEdge ee2 = new(theLC, v3, v4);

            if (Sliding && !PtOnFirstEdge)
            {
                BRepExtrema_ExtCF ext1 = new(TopoDS.Edge(ee1.InternalShape()), FirstFace);
                if (ext1.NbExt() < 1 || ext1.SquareDistance(1) > Precision.SquareConfusion())
                {
                    Sliding = false;
                }
            }
            if (Sliding && !PtOnLastEdge)
            {
                BRepExtrema_ExtCF ext2 = new(ee2.InternalEdge(), LastFace); // ExtCF : curves and surfaces
                if (ext2.NbExt() < 1 || ext2.SquareDistance(1) > Precision.SquareConfusion())
                {
                    Sliding = false;
                }
            }
            if (Sliding && PtOnFirstEdge)
            {
                FirstCrv = BRep_Tool.Curve(OnFirstEdge, out double f, out double l)!;
                if (FirstCrv.GetType() != typeof(Geom_Circle))
                {
                    Sliding = false;
                }
                else
                {
                    Geom_Circle C1 = (Geom_Circle)FirstCrv;
                    gp_Circ Circ = C1.Circ();
                    FirstCircle = Circ;
                    gp_Ax1 circax = FirstCircle.Axis();
                    if (!circax.IsCoaxial(myAxe, Precision.Confusion(), Precision.Confusion()))
                    {
                        Sliding = false;
                    }
                    else
                    {
                        if (Math.Abs(FirstCircle.Radius() - FirstRayon) >= Precision.Confusion())
                        {
                            Sliding = false;
                        }
                    }
                }
            }

            if (Sliding && PtOnLastEdge)
            {
                LastCrv = BRep_Tool.Curve(OnLastEdge, out double f, out double l)!;
                if (LastCrv.GetType() != typeof(Geom_Circle))
                {
                    Sliding = false;
                }
                else
                {
                    Geom_Circle C1 = (Geom_Circle)LastCrv;
                    gp_Circ Circ = C1.Circ();
                    LastCircle = Circ;
                    gp_Ax1 circax = LastCircle.Axis();
                    if (!circax.IsCoaxial(myAxe, Precision.Confusion(), Precision.Confusion()))
                    {
                        Sliding = false;
                    }
                    else
                    {
                        double rad = LastCircle.Radius();
                        if (Math.Abs(rad - LastRayon) >= Precision.Confusion())
                        {
                            Sliding = false;
                        }
                    }
                }
            }
        }

        // Construct a great profile that goes till the bounding box
        // -> by tangency with first and last edge of the Wire
        // -> by normals to base faces : statistically better
        // Intersect everything to find the final profile

        // ---case of sliding : construction of the face profile
        if (Sliding)
        {
            TopoDS_Face Prof = new();
            bool ProfileOK;
            ProfileOK = SlidingProfile(ref Prof, RevolRib, myTol, ref Concavite, myPln, BndFace, CheckPnt, FirstFace,
                                       LastFace, FirstVertex, LastVertex, FirstEdge, LastEdge);

            if (!ProfileOK)
            {
                myStatusError = BRepFeat_NoFaceProf;
                NotDone();
                return;
            }

            // ---Propagation on faces of the initial shape
            // to find the faces concerned by the rib
            bool falseside = true;
            Sliding = Propagate(SliList, Prof, myFirstPnt, myLastPnt, ref falseside);
            // Control if there is everything required to have the material at the proper side
            if (!falseside)
            {
                myStatusError = BRepFeat_FalseSide;
                NotDone();
                return;
            }
        }

        // ---Generation of the base profile of the rib

        TopoDS_Wire w = new();
        BB.MakeWire(ref w);
        TopoDS_Edge thePreviousEdge = new();
        TopoDS_Vertex theFV = new();
        thePreviousEdge.Nullify();

        // counter of the number of edges to fill the map
        int counter = 1;

        // ---case of sliding
        if (Sliding && !myListOfEdges.IsEmpty())
        {
            BRepTools_WireExplorer EX1 = new(myWire);
            for (; EX1.More(); EX1.Next())
            {
                TopoDS_Edge E = EX1.Current();
                if (!myLFMap.IsBound(E))
                {
                    NCollection_List<TopoDS_Shape> theTmpList = new();
                    myLFMap.Bind(E, theTmpList);
                }
                if (E.IsSame(FirstEdge))
                {
                    Geom_Curve cc = BRep_Tool.Curve(E, out double f, out double l)!;
                    gp_Pnt pt;
                    if (!FirstEdge.IsSame(LastEdge))
                    {
                        pt = BRep_Tool.Pnt(TopExp.LastVertex(E, true));
                    }
                    else
                    {
                        pt = myLastPnt;
                        double fpar = IntPar(cc, myFirstPnt);
                        double lpar = IntPar(cc, pt);
                        if (fpar > lpar)
                        {
                            cc = cc.Reversed();
                        }
                    }
                    TopoDS_Edge ee1;
                    if (thePreviousEdge.IsNull())
                    {
                        BRepLib_MakeVertex v1 = new(myFirstPnt);
                        BRepLib_MakeVertex v2 = new(pt);
                        BRepLib_MakeEdge e = new(cc, v1.InternalVertex(), v2.InternalVertex());
                        ee1 = TopoDS.Edge(e.InternalShape());
                    }
                    else
                    {
                        TopoDS_Vertex v1 = TopExp.LastVertex(thePreviousEdge, true);
                        BRepLib_MakeVertex v2 = new(pt);
                        BRepLib_MakeEdge e = new(cc, v1, v2.InternalVertex());
                        ee1 = TopoDS.Edge(e.InternalShape());
                    }
                    TopoDS_Shape aLocalShape = ee1.Oriented(E.Orientation());
                    ee1 = TopoDS.Edge(aLocalShape);
                    if (counter == 1)
                    {
                        theFV = TopExp.FirstVertex(ee1, true);
                    }
                    myLFMap[E].Append(ee1);
                    BB.Add(w, ee1);
                    thePreviousEdge = ee1;
                    counter++;
                    EX1.Next();
                    break;
                }
            }

            // Case of several edges
            if (!FirstEdge.IsSame(LastEdge))
            {
                for (; EX1.More(); EX1.Next())
                {
                    TopoDS_Edge E = EX1.Current();
                    if (!myLFMap.IsBound(E))
                    {
                        NCollection_List<TopoDS_Shape> thelist1 = new();
                        myLFMap.Bind(E, thelist1);
                    }
                    theList.Append(E);
                    if (!E.IsSame(LastEdge))
                    {
                        Geom_Curve ccc = BRep_Tool.Curve(E, out double f, out double l)!;
                        TopoDS_Vertex v1, v2;
                        if (!thePreviousEdge.IsNull())
                        {
                            v1 = TopExp.LastVertex(thePreviousEdge, true);
                            v2 = TopExp.LastVertex(E, true);
                        }
                        else
                        {
                            v1 = TopExp.FirstVertex(E, true);
                            v2 = TopExp.LastVertex(E, true);
                        }
                        BRepLib_MakeEdge E1 = new(ccc, v1, v2);
                        TopoDS_Edge E11 = TopoDS.Edge(E1.InternalShape());
                        TopoDS_Shape aLocalShape = E11.Oriented(E.Orientation());
                        E11 = TopoDS.Edge(aLocalShape);
                        thePreviousEdge = E11;
                        myLFMap[E].Append(E11);
                        BB.Add(w, E11);
                        if (counter == 1)
                        {
                            theFV = TopExp.FirstVertex(E11, true);
                        }
                        counter++;
                    }
                    else
                    {
                        Geom_Curve cc = BRep_Tool.Curve(E, out double f, out double l)!;
                        gp_Pnt pf = BRep_Tool.Pnt(TopExp.FirstVertex(E, true));
                        gp_Pnt pl = myLastPnt;
                        TopoDS_Edge ee;
                        if (thePreviousEdge.IsNull())
                        {
                            BRepLib_MakeEdge e = new(cc, pf, pl);
                            ee = TopoDS.Edge(e.InternalShape());
                        }
                        else
                        {
                            TopoDS_Vertex v1 = TopExp.LastVertex(thePreviousEdge, true);
                            BRepLib_MakeVertex v2 = new(pl);
                            BRepLib_MakeEdge e = new(cc, v1, v2.InternalVertex());
                            ee = TopoDS.Edge(e.InternalShape());
                        }
                        TopoDS_Shape aLocalShape = ee.Oriented(E.Orientation());
                        ee = TopoDS.Edge(aLocalShape);
                        BB.Add(w, ee);
                        myLFMap[E].Append(ee);
                        if (counter == 1)
                        {
                            theFV = TopExp.FirstVertex(ee, true);
                        }
                        thePreviousEdge = ee;
                        counter++;
                        break;
                    }
                }
            }

            NCollection_List<TopoDS_Shape>.Iterator it = new(myListOfEdges);
            bool FirstOK = false;
            bool LastOK = false;

            gp_Pnt theLastPnt = myLastPnt;
            int sens = 0;
            TopoDS_Edge theEdge = new(), theLEdge = new(), theFEdge = new();
            int counter1 = counter;
            NCollection_List<TopoDS_Shape> NewListOfEdges = new();
            NewListOfEdges.Clear();
            while (!FirstOK)
            {
                TopoDS_Edge edg = TopoDS.Edge(it.Value());
                gp_Pnt fp, lp;
                Geom_Curve ccc = BRep_Tool.Curve(edg, out double f, out double l)!;
                Geom_TrimmedCurve cc = new(ccc, f, l);
                if (edg.Orientation() == TopAbs_REVERSED)
                {
                    cc.Reverse();
                }

                fp = cc.Value(cc.FirstParameter());
                lp = cc.Value(cc.LastParameter());
                double dist = fp.Distance(theLastPnt);
                if (dist <= myTol)
                {
                    sens = 1;
                    LastOK = true;
                }
                else
                {
                    dist = lp.Distance(theLastPnt);
                    if (dist <= myTol)
                    {
                        sens = 2;
                        LastOK = true;
                        cc.Reverse();
                    }
                }
                int FirstFlag = 0;
                if (sens == 1 && lp.Distance(myFirstPnt) <= myTol)
                {
                    FirstOK = true;
                    FirstFlag = 1;
                }
                else if (sens == 2 && fp.Distance(myFirstPnt) <= myTol)
                {
                    FirstOK = true;
                    FirstFlag = 2;
                }
                if (LastOK)
                {
                    TopoDS_Edge eeee;
                    double fpar = cc.FirstParameter();
                    double lpar = cc.LastParameter();
                    if (!FirstOK)
                    {
                        if (thePreviousEdge.IsNull())
                        {
                            BRepLib_MakeEdge e = new(cc, fpar, lpar);
                            eeee = TopoDS.Edge(e.InternalShape());
                        }
                        else
                        {
                            TopoDS_Vertex v1 = TopExp.LastVertex(thePreviousEdge, true);
                            BRepLib_MakeVertex v2 = new(cc.Value(lpar));
                            BRepLib_MakeEdge e = new(cc, v1, v2.InternalVertex());
                            eeee = TopoDS.Edge(e.InternalShape());
                        }
                    }
                    else
                    {
                        if (thePreviousEdge.IsNull())
                        {
                            BRepLib_MakeVertex v1 = new(cc.Value(fpar));
                            BRepLib_MakeEdge e = new(cc, v1.InternalVertex(), theFV);
                            eeee = TopoDS.Edge(e.InternalShape());
                        }
                        else
                        {
                            TopoDS_Vertex v1 = TopExp.LastVertex(thePreviousEdge, true);
                            BRepLib_MakeEdge e = new(cc, v1, theFV);
                            eeee = TopoDS.Edge(e.InternalShape());
                        }
                    }

                    thePreviousEdge = eeee;
                    BB.Add(w, eeee);
                    if (counter == 1)
                    {
                        theFV = TopExp.FirstVertex(eeee, true);
                    }
                    counter1++;
                    NewListOfEdges.Append(edg);
                    theEdge = eeee;

                    if (dist <= myTol)
                    {
                        theFEdge = edg;
                    }
                    theLastPnt = BRep_Tool.Pnt(TopExp.LastVertex(theEdge, true));
                }
                if (FirstFlag == 1)
                {
                    theLEdge = edg;
                }
                else if (FirstFlag == 2)
                {
                    theLEdge = theEdge;
                }

                if (LastOK)
                {
                    it.Initialize(myListOfEdges);
                    LastOK = false;
                }
                else if (it.More())
                {
                    it.Next();
                }
                else
                {
                    Sliding = false;
                    break;
                }
                sens = 0;
            }

            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> SlidMap = new(TopTools_ShapeMapHasher.Instance);
            SlidMap.Clear();

            if (Sliding && counter1 > counter)
            {
                TopExp_Explorer EX2 = new(w, TopAbs_EDGE);
                int ii = 0;
                for (; EX2.More(); EX2.Next())
                {
                    TopoDS_Edge E = TopoDS.Edge(EX2.Current());
                    ii++;
                    if (ii >= counter && ii <= counter1)
                    {
                        it.Initialize(NewListOfEdges);
                        int jj = 0;
                        for (; it.More(); it.Next())
                        {
                            TopoDS_Edge e2 = TopoDS.Edge(it.Value());
                            jj++;
                            if (jj == (ii - counter + 1))
                            {
                                foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itm in mySlface)
                                {
                                    TopoDS_Face fac = TopoDS.Face(itm.Key);
                                    NCollection_List<TopoDS_Shape> ledg = itm.Value;
                                    NCollection_List<TopoDS_Shape>.Iterator itedg = new(ledg);
                                    for (; itedg.More(); itedg.Next())
                                    {
                                        TopoDS_Edge e1 = TopoDS.Edge(itedg.Value());
                                        if (e1.IsSame(e2))
                                        {
                                            if (!SlidMap.IsBound(fac))
                                            {
                                                NCollection_List<TopoDS_Shape> thelist2 = new();
                                                SlidMap.Bind(fac, thelist2);
                                            }
                                            SlidMap[fac].Append(E);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            mySlface.Clear();
            mySlface = SlidMap;
        }

        // ---Arguments of LocOpe_LinearForm : arguments of the prism
        // sliding
        if (Sliding)
        {
            TopoDS_Face F = new();
            BB.MakeFace(ref F, myPln, myTol);
            w.Closed(BRep_Tool.IsClosed(w));
            BB.Add(F, w);
            mySkface = F;
            myPbase = mySkface;
            mySUntil.Nullify();
        }

        // ---Case without sliding : construction of the face profile
        if (!Sliding)
        {
            TopExp_Explorer explo1 = new(BndFace, TopAbs_WIRE);
            TopoDS_Wire WWW = TopoDS.Wire(explo1.Current());
            BRepTools_WireExplorer explo = new(WWW);
            BRep_Builder Bu = new();
            TopoDS_Wire Wiwiwi = new();
            Bu.MakeWire(ref Wiwiwi);
            TopoDS_Vertex NewV1 = new(), NewV2 = new(), LastV = new();
            NewV1.Nullify();
            NewV2.Nullify();
            LastV.Nullify();

            for (; explo.More(); explo.Next())
            {
                TopoDS_Edge e = TopoDS.Edge(explo.Current());
                TopoDS_Vertex v1 = TopExp.FirstVertex(e, true);
                TopoDS_Vertex v2 = TopExp.LastVertex(e, true);

                Geom_Curve ln = BRep_Tool.Curve(e, out double f, out double l)!;
                Geom2d_Curve l2d = GeomAPI.To2d(ln, Plane.Pln())!;
                Geom2dAPI_InterCurveCurve intcc = new(l2d, ln2d, Precision.Confusion());
                TopoDS_Vertex VV = new();
                VV.Nullify();

                if (intcc.NbPoints() > 0)
                {
                    gp_Pnt2d P = intcc.Point(1);
                    myPln.D0(P.X(), P.Y(), out gp_Pnt point);
                    double par = IntPar(ln, point);
                    if (f <= par && l >= par)
                    {
                        Bu.MakeVertex(ref VV, point, Precision.Confusion());
                    }
                }

                if (VV.IsNull() && NewV1.IsNull())
                {
                    continue;
                }

                if (!VV.IsNull() && NewV1.IsNull())
                {
                    NewV1 = VV;
                    LastV = v2;
                    BRepLib_MakeEdge ee1 = new(NewV1, LastV);
                    Bu.Add(Wiwiwi, ee1.InternalEdge());
                    continue;
                }

                if (VV.IsNull() && !NewV1.IsNull())
                {
                    BRepLib_MakeEdge ee1 = new(LastV, v2);
                    LastV = v2;
                    Bu.Add(Wiwiwi, e);
                    continue;
                }

                if (!VV.IsNull() && !NewV1.IsNull())
                {
                    NewV2 = VV;
                    BRepLib_MakeEdge ee1 = new(LastV, NewV2);
                    LastV = NewV2;
                    Bu.Add(Wiwiwi, ee1.InternalEdge());
                    BRepLib_MakeEdge ee2 = new(LastV, NewV1);
                    Bu.Add(Wiwiwi, ee2.InternalEdge());
                    break;
                }
            }

            Wiwiwi.Closed(BRep_Tool.IsClosed(Wiwiwi));
            BRepLib_MakeFace newbndface = new(myPln.Pln(), Wiwiwi, true);
            TopoDS_Face NewBndFace = TopoDS.Face(newbndface.InternalShape());

            BRepTopAdaptor_FClass2d Cl = new(NewBndFace, Precision.Confusion());
            ElSLib.Parameters(myPln.Pln(), CheckPnt, out double paru, out double parv);
            gp_Pnt2d checkpnt2d = new(paru, parv);
            if (Cl.Perform(checkpnt2d, true) == TopAbs_OUT)
            {
                BRepAlgoAPI_Cut c = new(BndFace, NewBndFace);
                TopExp_Explorer exp = new(c.InternalShape(), TopAbs_WIRE);
                TopoDS_Wire aCurWire = TopoDS.Wire(exp.Current());
                BRepLib_MakeFace ff = new(myPln.Pln(), aCurWire, true);
                NewBndFace = TopoDS.Face(ff.InternalShape());
            }

            if (!BRepAlgo.IsValid(NewBndFace))
            {
                myStatusError = BRepFeat_InvShape;
                NotDone();
                return;
            }

            BndFace = NewBndFace;

            TopoDS_Face Prof = new();
            bool ProfileOK;
            ProfileOK = NoSlidingProfile(ref Prof, RevolRib, myTol, ref Concavite, myPln, bnd, BndFace, CheckPnt,
                                         FirstFace, LastFace, FirstVertex, LastVertex, FirstEdge, LastEdge,
                                         OnFirstFace, OnLastFace);

            if (!ProfileOK)
            {
                myStatusError = BRepFeat_NoFaceProf;
                NotDone();
                return;
            }

            // ---Propagation on the faces of the initial shape
            // to find the faces concerned by the rib
            bool falseside = true;
            Propagate(SliList, Prof, myFirstPnt, myLastPnt, ref falseside);
            // Control if there is everything required to have the material at the proper side
            if (!falseside)
            {
                myStatusError = BRepFeat_FalseSide;
                NotDone();
                return;
            }

            mySlface.Clear();

            NCollection_List<TopoDS_Shape>.Iterator it = new();
            it.Initialize(SliList);

            TopoDS_Shell comp = new();
            BB.MakeShell(ref comp);

            for (; it.More(); it.Next())
            {
                BB.Add(comp, it.Value());
            }
            comp.Closed(BRep_Tool.IsClosed(comp));

            mySUntil = comp;

            mySkface = Prof;
            myPbase = Prof;
        }

        mySliding = Sliding;

        TopExp_Explorer exp1 = new();
        for (exp1.Init(mySbase, TopAbs_FACE); exp1.More(); exp1.Next())
        {
            NCollection_List<TopoDS_Shape> thelist3 = new();
            myMap.Bind(exp1.Current(), thelist3);
            myMap[exp1.Current()].Append(exp1.Current());
        }
    }

    /// <summary>Adds an element of sliding: the edge <paramref name="E"/> slides on the face <paramref name="F"/>.</summary>
    /// <exception cref="Standard_ConstructionError">F isn't a face of the basis shape.</exception>
    public void Add(in TopoDS_Edge E, in TopoDS_Face F)
    {
        if (mySlface.IsEmpty())
        {
            TopExp_Explorer exp = new();
            for (exp.Init(mySbase, TopAbs_FACE); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(F))
                {
                    break;
                }
            }
            if (!exp.More())
            {
                throw new Standard_ConstructionError();
            }

            if (!mySlface.IsBound(F))
            {
                NCollection_List<TopoDS_Shape> thelist = new();
                mySlface.Bind(F, thelist);
            }
            NCollection_List<TopoDS_Shape>.Iterator itl = new(mySlface[F]);
            for (; itl.More(); itl.Next())
            {
                if (itl.Value().IsSame(E))
                {
                    break;
                }
            }
            if (!itl.More())
            {
                mySlface[F].Append(E);
            }
        }
    }

    /// <summary>Builds the rib or groove.</summary>
    public void Perform()
    {
        if (mySbase.IsNull() || mySkface.IsNull() || myPbase.IsNull())
        {
            myStatusError = BRepFeat_NotInitialized;
            NotDone();
            return;
        }

        gp_Pnt Pt = new();

        TopExp_Explorer exx = new(myPbase, TopAbs_VERTEX);
        for (; exx.More(); exx.Next())
        {
            TopoDS_Vertex vv = TopoDS.Vertex(exx.Current());
            if (!vv.IsNull())
            {
                Pt = BRep_Tool.Pnt(vv);
                break;
            }
        }

        if (myAngle2 != 0)
        {
            gp_Trsf T = new();
            T.SetRotation(myAxe, -myAngle2);
            BRepBuilderAPI_Transform trsf = new(T);
            trsf.Perform(myPbase, false);
            TopoDS_Face Pbase = TopoDS.Face(trsf.InternalShape());
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> iter in myLFMap)
            {
                TopoDS_Shape e1 = iter.Value.First();
                TopExp_Explorer ex1 = new(myPbase, TopAbs_EDGE);
                TopExp_Explorer ex2 = new(Pbase, TopAbs_EDGE);
                for (; ex1.More(); ex1.Next())
                {
                    if (ex1.Current().IsSame(e1))
                    {
                        myLFMap[iter.Key].Clear();
                        myLFMap[iter.Key].Append(ex2.Current());
                        break; // break the cycle (e1 became a dead reference)
                    }
                    ex2.Next();
                }
            }

            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> iter1 in mySlface)
            {
                TopoDS_Shape e1 = iter1.Value.First();
                TopExp_Explorer ex1 = new(myPbase, TopAbs_EDGE);
                TopExp_Explorer ex2 = new(Pbase, TopAbs_EDGE);
                for (; ex1.More(); ex1.Next())
                {
                    if (ex1.Current().IsSame(e1))
                    {
                        mySlface[iter1.Key].Clear();
                        mySlface[iter1.Key].Append(ex2.Current());
                        break; // break the cycle (e1 became a dead reference)
                    }
                    ex2.Next();
                }
            }
            myPbase = Pbase;
            trsf.Perform(mySkface, false);
            mySkface = TopoDS.Face(trsf.InternalShape());
        }

        LocOpe_RevolutionForm theForm = new();
        theForm.Perform(myPbase, myAxe, myAngle1 + myAngle2);
        TopoDS_Shape VraiForm = theForm.InternalShape(); // uncut  primitive

        // management of descendants
        MajMap(myPbase, theForm, myMap, ref myFShape, ref myLShape);

        myGluedF.Clear();

        gp_Pln Pln0 = myPln!.Pln();
        BRepLib_MakeFace f = new(Pln0);

        gp_Vec vec1 = myHeight1 * new gp_Vec(Normal(f.InternalFace(), Pt));
        gp_Vec vec2 = -myHeight2 * new gp_Vec(Normal(f.InternalFace(), Pt));

        gp_Pln Pln1 = Pln0.Translated(vec1);
        gp_Pln Pln2 = Pln0.Translated(vec2);

        BRepLib_MakeFace ff1 = new(Pln1);
        BRepLib_MakeFace ff2 = new(Pln2);
        TopoDS_Face f1 = TopoDS.Face(ff1.InternalShape());
        TopoDS_Face f2 = TopoDS.Face(ff2.InternalShape());
        BRepFeat.FaceUntil(mySbase, ref f1);
        BRepFeat.FaceUntil(mySbase, ref f2);

        LocOpe_CSIntersector ASI1 = new(f1);
        LocOpe_CSIntersector ASI2 = new(f2);

        Geom_Line normale = new(Pt, new gp_Dir(vec1));
        NCollection_Sequence<Geom_Curve?> scur = new();
        scur.Append(normale);

        ASI1.Perform(scur);
        ASI2.Perform(scur);

        if (!ASI1.IsDone() || !ASI2.IsDone() || ASI1.NbPoints(1) != 1 || ASI2.NbPoints(1) != 1)
        {
            myStatusError = BRepFeat_BadIntersect;
            NotDone();
            return;
        }

        TopAbs_Orientation Ori1 = ASI1.Point(1, 1).Orientation();
        TopAbs_Orientation Ori2 = TopAbs.Reverse(ASI2.Point(1, 1).Orientation());
        TopoDS_Face FF1 = ASI1.Point(1, 1).Face();
        TopoDS_Face FF2 = ASI2.Point(1, 1).Face();

        TopoDS_Compound Comp = new();
        BRep_Builder B = new();
        B.MakeCompound(ref Comp);
        TopoDS_Solid S1 = BRepFeat.Tool(f1, FF1, Ori1);
        TopoDS_Solid S2 = BRepFeat.Tool(f2, FF2, Ori2);
        if (!S1.IsNull())
        {
            B.Add(Comp, S1);
        }
        if (!S2.IsNull())
        {
            B.Add(Comp, S2);
        }

        BRepAlgoAPI_Cut trP = new(VraiForm, Comp);
        // coupe de la nervure par deux plans parallels
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> SlidingMap = new(TopTools_ShapeMapHasher.Instance);

        // management of descendants
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> it1 in myMap)
        {
            TopoDS_Shape orig = it1.Key;
            if (it1.Value.IsEmpty())
            {
                continue;
            }
            TopoDS_Shape sh = it1.Value.First();
            exx.Init(VraiForm, TopAbs_FACE);
            for (; exx.More(); exx.Next())
            {
                TopoDS_Face fac = TopoDS.Face(exx.Current());
                TopExp_Explorer exx1 = new(fac, TopAbs_WIRE);
                TopoDS_Wire thew = TopoDS.Wire(exx1.Current());
                if (thew.IsSame(myFShape))
                {
                    myMap[myFShape] = new NCollection_List<TopoDS_Shape>(trP.InternalModified(f2));
                    continue;
                }
                else if (thew.IsSame(myLShape))
                {
                    myMap[myLShape] = new NCollection_List<TopoDS_Shape>(trP.InternalModified(f1));
                    continue;
                }
                if (fac.IsSame(sh))
                {
                    if (!trP.IsDeleted(fac))
                    {
                        NCollection_List<TopoDS_Shape> desfaces = trP.InternalModified(fac);
                        if (!desfaces.IsEmpty())
                        {
                            myMap[orig].Clear();
                            myMap[orig] = new NCollection_List<TopoDS_Shape>(trP.InternalModified(fac));
                            break; // break the cycle (sh became a dead reference)
                        }
                    }
                }
            }
        }

        exx.Init(VraiForm, TopAbs_FACE);
        for (; exx.More(); exx.Next())
        {
            TopoDS_Face fac = TopoDS.Face(exx.Current());
            NCollection_List<TopoDS_Shape> thelist = new();
            SlidingMap.Bind(fac, thelist);
            if (trP.IsDeleted(fac))
            {
            }
            else
            {
                NCollection_List<TopoDS_Shape> desfaces = trP.InternalModified(fac);
                if (!desfaces.IsEmpty())
                {
                    SlidingMap[fac] = new NCollection_List<TopoDS_Shape>(desfaces);
                }
                else
                {
                    SlidingMap[fac].Append(fac);
                }
            }
        }

        // gestion of faces of sliding
        SetGluedFaces(mySlface, theForm, SlidingMap, myGluedF);

        VraiForm = trP.InternalShape(); // primitive cut

        if (!myGluedF.IsEmpty())
        {
            myPerfSelection = BRepFeat_NoSelection;
        }
        else
        {
            myPerfSelection = BRepFeat_SelectionSh;
        }

        exx.Init(myPbase, TopAbs_EDGE);
        for (; exx.More(); exx.Next())
        {
            TopoDS_Edge e = TopoDS.Edge(exx.Current());
            if (!myMap.IsBound(e))
            {
                myStatusError = BRepFeat_IncSlidFace;
                NotDone();
                return;
            }
        }

        myGShape = VraiForm;

        if (!myGluedF.IsEmpty() && !mySUntil.IsNull())
        {
            myStatusError = BRepFeat_InvShape;
            NotDone();
            return;
        }

        LFPerform(); // topological reconstruction
    }

    /// <summary>
    /// Propagates the profile <paramref name="fac"/> over the faces of the basis shape from the first face of
    /// <paramref name="SliList"/>, adding the faces the rib meets; <paramref name="falseside"/> becomes false when the
    /// material isn't on the profile's side.
    /// </summary>
    public bool Propagate(NCollection_List<TopoDS_Shape> SliList, in TopoDS_Face fac, in gp_Pnt Firstpnt,
                          in gp_Pnt Lastpnt, ref bool falseside)
    {
        gp_Pnt Firstpoint = Firstpnt;
        gp_Pnt Lastpoint = Lastpnt;

        bool result = true;
        TopoDS_Face CurrentFace, saveFace;
        CurrentFace = TopoDS.Face(SliList.First());
        saveFace = CurrentFace;

        bool LastOK = false, FirstOK = false;
        TopoDS_Vertex v1 = new(), v2 = new(), v3, v4, Vert;

        BRepAlgoAPI_Section sect = new(fac, CurrentFace, false);
        sect.Approximation(true);
        sect.Build();
        TopExp_Explorer Ex = new();
        TopoDS_Edge e = new(), e1 = new();
        gp_Pnt FP, LP;
        int ii = 0;
        for (Ex.Init(sect.InternalShape(), TopAbs_EDGE); Ex.More(); Ex.Next())
        {
            ii++;
            if (ii == 1)
            {
                e = TopoDS.Edge(Ex.Current());
            }
            else if (ii > 1)
            {
                e1 = TopoDS.Edge(Ex.Current());
                break;
            }
        }
        if (e.IsNull())
        {
            falseside = false;
            return false;
        }
        //
        if (!e1.IsNull())
        {
            double aTolV1, aTolV2;
            myListOfEdges.Clear();
            NCollection_List<TopoDS_Shape> thelist = new();
            mySlface.Bind(CurrentFace, thelist);
            mySlface[CurrentFace].Append(e1);

            myListOfEdges.Append(e1);

            v1 = TopExp.FirstVertex(e1, true);
            v2 = TopExp.LastVertex(e1, true);

            FP = BRep_Tool.Pnt(v1);
            LP = BRep_Tool.Pnt(v2);

            aTolV1 = BRep_Tool.Tolerance(v1);
            aTolV2 = BRep_Tool.Tolerance(v2);

            if (FP.Distance(Firstpoint) <= aTolV1 || FP.Distance(Lastpoint) <= aTolV1)
            {
                FirstOK = true;
            }
            if (LP.Distance(Firstpoint) <= aTolV2 || LP.Distance(Lastpoint) <= aTolV2)
            {
                LastOK = true;
            }

            if (LastOK && FirstOK)
            {
                return result;
            }
            else
            {
                myListOfEdges.Clear();
            }
        }
        //
        if (!e1.IsNull())
        {
            myListOfEdges.Clear();
            NCollection_List<TopoDS_Shape> thelist1 = new();
            mySlface.Bind(CurrentFace, thelist1);
            mySlface[CurrentFace].Append(e);

            myListOfEdges.Append(e);

            mySlface[CurrentFace].Append(e1);

            v1 = TopExp.FirstVertex(e, true);
            v2 = TopExp.LastVertex(e, true);
            v3 = TopExp.FirstVertex(e1, true);
            v4 = TopExp.LastVertex(e1, true);
            gp_Pnt p1, p2, p3, p4;
            p1 = BRep_Tool.Pnt(v1);
            FP = p1;
            p2 = BRep_Tool.Pnt(v2);
            LP = p2;
            p3 = BRep_Tool.Pnt(v3);
            p4 = BRep_Tool.Pnt(v4);
            if (p1.Distance(Firstpoint) <= BRep_Tool.Tolerance(v1))
            {
                if (p3.Distance(Lastpoint) <= BRep_Tool.Tolerance(v3))
                {
                    FirstOK = true;
                    Lastpoint = p4;
                }
                else if (p4.Distance(Lastpoint) <= BRep_Tool.Tolerance(v4))
                {
                    FirstOK = true;
                    Lastpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else if (p1.Distance(Lastpoint) <= BRep_Tool.Tolerance(v1))
            {
                if (p3.Distance(Firstpoint) <= BRep_Tool.Tolerance(v3))
                {
                    FirstOK = true;
                    Firstpoint = p4;
                }
                else if (p4.Distance(Firstpoint) <= BRep_Tool.Tolerance(v4))
                {
                    FirstOK = true;
                    Firstpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else if (p2.Distance(Firstpoint) <= BRep_Tool.Tolerance(v2))
            {
                if (p3.Distance(Lastpoint) <= BRep_Tool.Tolerance(v3))
                {
                    LastOK = true;
                    Lastpoint = p4;
                }
                else if (p4.Distance(Lastpoint) <= BRep_Tool.Tolerance(v4))
                {
                    LastOK = true;
                    Lastpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else if (p2.Distance(Lastpoint) <= BRep_Tool.Tolerance(v2))
            {
                if (p3.Distance(Firstpoint) <= BRep_Tool.Tolerance(v3))
                {
                    LastOK = true;
                    Firstpoint = p4;
                }
                else if (p4.Distance(Firstpoint) <= BRep_Tool.Tolerance(v4))
                {
                    LastOK = true;
                    Firstpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else
            {
                e = e1;
                e1.Nullify();
            }
        }
        if (e1.IsNull())
        {
            myListOfEdges.Clear();
            NCollection_List<TopoDS_Shape> thelist2 = new();
            mySlface.Bind(CurrentFace, thelist2);
            mySlface[CurrentFace].Append(e);

            myListOfEdges.Append(e);

            v1 = TopExp.FirstVertex(e, true);
            v2 = TopExp.LastVertex(e, true);

            FP = BRep_Tool.Pnt(v1);
            LP = BRep_Tool.Pnt(v2);

            if (FP.Distance(Firstpoint) <= BRep_Tool.Tolerance(v1) || FP.Distance(Lastpoint) <= BRep_Tool.Tolerance(v1))
            {
                FirstOK = true;
            }
            if (LP.Distance(Firstpoint) <= BRep_Tool.Tolerance(v2) || LP.Distance(Lastpoint) <= BRep_Tool.Tolerance(v2))
            {
                LastOK = true;
            }

            if (LastOK && FirstOK)
            {
                return result;
            }
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mapedges = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(mySbase, TopAbs_EDGE, TopAbs_FACE, mapedges);
        TopExp_Explorer ex = new();
        TopoDS_Edge FirstEdge = new();

        TopoDS_Vertex Vprevious = new();
        Vprevious.Nullify();
        TopoDS_Vertex Vpreprevious = new();
        Vpreprevious.Nullify();

        while (!FirstOK)
        {
            // find edge connected to v1:
            gp_Pnt pt = new();
            if (!v1.IsNull())
            {
                pt = BRep_Tool.Pnt(v1);
            }
            gp_Pnt ptprev = new();
            if (!Vprevious.IsNull())
            {
                ptprev = BRep_Tool.Pnt(Vprevious);
            }
            gp_Pnt ptpreprev = new();
            if (!Vpreprevious.IsNull())
            {
                ptpreprev = BRep_Tool.Pnt(Vpreprevious);
            }

            if ((!Vprevious.IsNull() && ptprev.Distance(pt) <= myTol)
                || (!Vpreprevious.IsNull() && ptpreprev.Distance(pt) <= myTol))
            {
                falseside = false;
                return false;
            }

            for (ex.Init(CurrentFace, TopAbs_EDGE); ex.More(); ex.Next())
            {
                TopoDS_Edge aCurEdge = TopoDS.Edge(ex.Current());

                BRepExtrema_ExtPC projF = new(v1, aCurEdge);

                if (projF.IsDone() && projF.NbExt() >= 1)
                {
                    double dist2min = RealLast();
                    int index = 0;
                    for (int sol = 1; sol <= projF.NbExt(); sol++)
                    {
                        if (projF.SquareDistance(sol) <= dist2min)
                        {
                            index = sol;
                            dist2min = projF.SquareDistance(sol);
                        }
                    }
                    if (index != 0)
                    {
                        if (dist2min <= BRep_Tool.Tolerance(aCurEdge) * BRep_Tool.Tolerance(aCurEdge))
                        {
                            FirstEdge = aCurEdge;
                            break;
                        }
                    }
                }
            }

            NCollection_List<TopoDS_Shape> L = mapedges.FindFromKey(FirstEdge);
            NCollection_List<TopoDS_Shape>.Iterator It = new(L);

            for (; It.More(); It.Next())
            {
                TopoDS_Face FF = TopoDS.Face(It.Value());
                if (!FF.IsSame(CurrentFace))
                {
                    CurrentFace = FF;
                    break;
                }
            }

            BRepAlgoAPI_Section sectf = new(fac, CurrentFace, false);
            sectf.Approximation(true);
            sectf.Build();

            TopoDS_Edge edg1 = new();
            for (Ex.Init(sectf.InternalShape(), TopAbs_EDGE); Ex.More(); Ex.Next())
            {
                edg1 = TopoDS.Edge(Ex.Current());
                gp_Pnt ppp1 = BRep_Tool.Pnt(TopExp.FirstVertex(edg1, true));
                gp_Pnt ppp2 = BRep_Tool.Pnt(TopExp.LastVertex(edg1, true));
                if (ppp1.Distance(BRep_Tool.Pnt(v1)) <= BRep_Tool.Tolerance(v1)
                    || ppp2.Distance(BRep_Tool.Pnt(v1)) <= BRep_Tool.Tolerance(v1))
                {
                    break;
                }
            }

            NCollection_List<TopoDS_Shape> thelist3 = new();
            mySlface.Bind(CurrentFace, thelist3);
            mySlface[CurrentFace].Append(edg1);
            myListOfEdges.Append(edg1);

            if (!edg1.IsNull())
            {
                SliList.Prepend(CurrentFace);
            }
            else
            {
                return false;
            }

            Vert = TopExp.FirstVertex(edg1, true);
            gp_Pnt PP = BRep_Tool.Pnt(Vert);
            FP = BRep_Tool.Pnt(v1);
            double tol = BRep_Tool.Tolerance(edg1);
            double tol1 = BRep_Tool.Tolerance(v1);
            if (tol1 > tol)
            {
                tol = tol1;
            }
            double dist = PP.Distance(FP);
            if (dist <= tol)
            {
                Vpreprevious = Vprevious;
                Vprevious = v1;
                v1 = TopExp.LastVertex(edg1, true);
            }
            else
            {
                Vpreprevious = Vprevious;
                Vprevious = v1;
                v1 = Vert;
            }

            FP = BRep_Tool.Pnt(v1);

            if (FP.Distance(Firstpoint) <= BRep_Tool.Tolerance(v1) || FP.Distance(Lastpoint) <= BRep_Tool.Tolerance(v1))
            {
                FirstOK = true;
            }
        }

        CurrentFace = saveFace;
        Vprevious.Nullify();
        Vpreprevious.Nullify();

        while (!LastOK)
        {
            // find edge connected to v2:
            gp_Pnt pt = new();
            if (!v2.IsNull())
            {
                pt = BRep_Tool.Pnt(v2);
            }
            gp_Pnt ptprev = new();
            if (!Vprevious.IsNull())
            {
                ptprev = BRep_Tool.Pnt(Vprevious);
            }
            gp_Pnt ptpreprev = new();
            if (!Vpreprevious.IsNull())
            {
                ptpreprev = BRep_Tool.Pnt(Vpreprevious);
            }

            if ((!Vprevious.IsNull() && ptprev.Distance(pt) <= myTol)
                || (!Vpreprevious.IsNull() && ptpreprev.Distance(pt) <= myTol))
            {
                falseside = false;
                return false;
            }

            for (ex.Init(CurrentFace, TopAbs_EDGE); ex.More(); ex.Next())
            {
                TopoDS_Edge aCurEdge = TopoDS.Edge(ex.Current());
                BRepExtrema_ExtPC projF = new(v2, aCurEdge);

                if (projF.IsDone() && projF.NbExt() >= 1)
                {
                    double dist2min = RealLast();
                    int index = 0;
                    for (int sol = 1; sol <= projF.NbExt(); sol++)
                    {
                        if (projF.SquareDistance(sol) <= dist2min)
                        {
                            index = sol;
                            dist2min = projF.SquareDistance(sol);
                        }
                    }
                    if (index != 0)
                    {
                        if (dist2min <= BRep_Tool.Tolerance(aCurEdge) * BRep_Tool.Tolerance(aCurEdge))
                        {
                            FirstEdge = aCurEdge;
                            break;
                        }
                    }
                }
            }

            NCollection_List<TopoDS_Shape> L = mapedges.FindFromKey(FirstEdge);
            NCollection_List<TopoDS_Shape>.Iterator It = new(L);

            for (; It.More(); It.Next())
            {
                TopoDS_Face FF = TopoDS.Face(It.Value());
                if (!FF.IsSame(CurrentFace))
                {
                    CurrentFace = FF;
                    break;
                }
            }

            ii = 0;

            BRepAlgoAPI_Section sectf = new(fac, CurrentFace, false);
            sectf.Approximation(true);
            sectf.Build();

            TopoDS_Edge edg2 = new();
            for (Ex.Init(sectf.InternalShape(), TopAbs_EDGE); Ex.More(); Ex.Next())
            {
                edg2 = TopoDS.Edge(Ex.Current());
                gp_Pnt ppp1 = BRep_Tool.Pnt(TopExp.FirstVertex(edg2, true));
                gp_Pnt ppp2 = BRep_Tool.Pnt(TopExp.LastVertex(edg2, true));
                if (ppp1.Distance(BRep_Tool.Pnt(v2)) <= BRep_Tool.Tolerance(v2)
                    || ppp2.Distance(BRep_Tool.Pnt(v2)) <= BRep_Tool.Tolerance(v2))
                {
                    break;
                }
            }
            NCollection_List<TopoDS_Shape> thelist4 = new();
            mySlface.Bind(CurrentFace, thelist4);
            mySlface[CurrentFace].Append(edg2);
            myListOfEdges.Append(edg2);

            if (!edg2.IsNull())
            {
                SliList.Append(CurrentFace);
            }
            else
            {
                return false;
            }

            Vert = TopExp.FirstVertex(edg2, true);
            gp_Pnt PP = BRep_Tool.Pnt(Vert);
            FP = BRep_Tool.Pnt(v2);
            if (PP.Distance(FP) <= BRep_Tool.Tolerance(v2))
            {
                Vpreprevious = Vprevious;
                Vprevious = v2;
                v2 = TopExp.LastVertex(edg2, true);
            }
            else
            {
                v2 = Vert;
            }
            FP = BRep_Tool.Pnt(v2);

            if (FP.Distance(Firstpoint) <= BRep_Tool.Tolerance(v2) || FP.Distance(Lastpoint) <= BRep_Tool.Tolerance(v2))
            {
                LastOK = true;
            }
        }
        if (!e1.IsNull())
        {
            myListOfEdges.Append(e1);
        }
        return result;
    }

    private static void MajMap(in TopoDS_Shape theB, LocOpe_RevolutionForm theP,
                               NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap,
                               ref TopoDS_Shape theFShape, ref TopoDS_Shape theLShape)
    {
        TopExp_Explorer exp = new(theP.InternalFirstShape(), TopAbs_WIRE);
        if (exp.More())
        {
            theFShape = exp.Current();
            NCollection_List<TopoDS_Shape> thelist = new();
            theMap.Bind(theFShape, thelist);
            for (exp.Init(theP.InternalFirstShape(), TopAbs_FACE); exp.More(); exp.Next())
            {
                TopoDS_Shape sh = exp.Current();
                theMap[theFShape].Append(sh);
            }
        }

        exp.Init(theP.InternalLastShape(), TopAbs_WIRE);
        if (exp.More())
        {
            theLShape = exp.Current();
            NCollection_List<TopoDS_Shape> thelist1 = new();
            theMap.Bind(theLShape, thelist1);
            for (exp.Init(theP.InternalLastShape(), TopAbs_FACE); exp.More(); exp.Next())
            {
                TopoDS_Shape sh = exp.Current();
                theMap[theLShape].Append(sh);
            }
        }

        for (exp.Init(theB, TopAbs_EDGE); exp.More(); exp.Next())
        {
            if (!theMap.IsBound(exp.Current()))
            {
                theMap.Bind(exp.Current(), new NCollection_List<TopoDS_Shape>(theP.InternalShapes(exp.Current())));
            }
        }
    }

    private static void SetGluedFaces(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theSlmap,
                                      LocOpe_RevolutionForm thePrism,
                                      NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> SlidingMap,
                                      NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theMap)
    {
        // Slidings
        if (!theSlmap.IsEmpty())
        {
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itm in theSlmap)
            {
                TopoDS_Face fac = TopoDS.Face(itm.Key);
                NCollection_List<TopoDS_Shape> ledg = itm.Value;
                NCollection_List<TopoDS_Shape>.Iterator it = new();
                for (it.Initialize(ledg); it.More(); it.Next())
                {
                    NCollection_List<TopoDS_Shape> gfac = thePrism.InternalShapes(it.Value());
                    foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> iterm in SlidingMap)
                    {
                        TopoDS_Face ff = TopoDS.Face(iterm.Key);
                        NCollection_List<TopoDS_Shape> lfaces = iterm.Value;
                        if (lfaces.IsEmpty())
                        {
                            continue;
                        }
                        TopoDS_Face fff = TopoDS.Face(lfaces.First());
                        if (gfac.First().IsSame(ff))
                        {
                            theMap.Bind(fff, fac);
                        }
                    }
                }
            }
        }
    }
}
