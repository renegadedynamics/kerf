// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevolutionForm.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevolutionForm.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevolutionForm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
    private Axis3D myAxe = new();
    private double myHeight1;
    private double myHeight2;
    // OCCT: the class declares its own mySliding, hiding BRepFeat_RibSlot's, which stays false
    private new bool mySliding;
    private PlaneSurface? myPln;
    private double myBnd;
    private NCollection_DataMap<Shape, NCollection_List<Shape>> mySlface = new(ShapeHasher.Instance);
    private readonly NCollection_List<Shape> myListOfEdges = new();
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
    public BRepFeat_MakeRevolutionForm(in Shape Sbase, in Wire W, PlaneSurface Plane, in Axis3D Axis,
                                       double Height1, double Height2, int Fuse, bool Sliding)
    {
        Init(Sbase, W, Plane, Axis, Height1, Height2, Fuse, Sliding);
    }

    /// <summary>Initializes the revolution form (see the constructor).</summary>
    public void Init(in Shape Sbase, in Wire W, PlaneSurface Plane, in Axis3D Axis, double H1, double H2,
                     int Mode, bool Modify)
    {
        bool RevolRib = true;
        Done();

        // modify = 0 if it is not required to make sliding
        //        = 1 if it is intended to try to make sliding
        bool Sliding = Modify;

        myAxe = Axis;
        LineCurve3D Line = new(Axis);
        double LineFirst, LineLast;

        LocOpe_CSIntersector ASI = new(Sbase);
        NCollection_Sequence<Curve3D?> scur = new();
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

        Curve2D ln2d = GeomAPI.To2d(Line, Plane.Pln())!;

        Explorer exx = new();
        double Rad = RealLast();

        exx.Init(W, ShapeType.Edge);
        for (; exx.More(); exx.Next())
        {
            Edge e = ShapeCasts.Edge(exx.Current());
            Curve3D c = Tool.Curve(e, out double f, out double l)!;
            Curve2D c2d = GeomAPI.To2d(c, Plane.Pln())!;
            Geom2dAPI_ExtremaCurveCurve extr = new(ln2d, c2d, LineFirst, LineLast, f, l);
            double L = RealLast();
            if (extr.NbExtrema() >= 1)
            {
                L = extr.LowerDistance();
            }
            Point3D p1 = c.Value(f);
            Point3D p2 = c.Value(l);
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

        if ((myAngle1 - M_PI / 2) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            myAngle1 = M_PI / 2;
        }
        if ((myAngle2 - M_PI / 2) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            myAngle2 = M_PI / 2;
        }

        mySkface.Nullify();
        myPbase.Nullify();

        myFuse = Mode != 0;

        // ---Determination Tolerance : tolerance max on parameters
        myTol = global::Kerf.Engine.Foundation.Precision.Confusion();

        exx.Init(W, ShapeType.Vertex);
        for (; exx.More(); exx.Next())
        {
            double tol = Tool.Tolerance(ShapeCasts.Vertex(exx.Current()));
            if (tol > myTol)
            {
                myTol = tol;
            }
        }

        exx.Init(Sbase, ShapeType.Vertex);
        for (; exx.More(); exx.Next())
        {
            double tol = Tool.Tolerance(ShapeCasts.Vertex(exx.Current()));
            if (tol > myTol)
            {
                myTol = tol;
            }
        }

        Shape aLocalShapeW = W.Oriented(Forward);
        myWire = ShapeCasts.Wire(aLocalShapeW);
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
        BrepBuilder BB = new();

        NCollection_List<Shape> theList = new();

        Shape U = new();
        U.Nullify();
        Point3D FirstCorner = new(), LastCorner = new();
        double bnd = HeightMax(mySbase, U, ref FirstCorner, ref LastCorner);
        myBnd = bnd;

        BRepPrimAPI_MakeBox Bndbox = new(FirstCorner, LastCorner);
        Solid BndBox = Bndbox.InternalSolid();

        // ---Construction of the working plane face (section bounding box)
        BRepLib_MakeFace PlaneF = new(myPln.Pln(), -6.0 * myBnd, 6.0 * myBnd, -6.0 * myBnd, 6.0 * myBnd);
        Face PlaneFace = ShapeCasts.Face(PlaneF.InternalShape());

        BRepAlgoAPI_Common PlaneS = new(BndBox, PlaneFace);
        Explorer EXP = new();
        Shape PlaneSect = PlaneS.InternalShape();
        EXP.Init(PlaneSect, ShapeType.Wire);
        Wire www = ShapeCasts.Wire(EXP.Current());
        BRepLib_MakeFace Bndface = new(myPln.Pln(), www, true);
        Face BndFace = ShapeCasts.Face(Bndface.InternalShape());

        // ---Find base faces of the rib
        Edge FirstEdge = new(), LastEdge = new();
        Face FirstFace = new(), LastFace = new();
        Vertex FirstVertex = new(), LastVertex = new();

        bool OnFirstFace = false;
        bool OnLastFace = false;
        bool PtOnFirstEdge = false;
        bool PtOnLastEdge = false;
        Edge OnFirstEdge = new(), OnLastEdge = new();
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
        Point3D CheckPnt = CheckPoint(FirstEdge, bnd / 10.0, myPln);

        // ---Control sliding valid
        // Many cases when the sliding is abandoned
        int Concavite = 3; // a priori the profile is not concave

        myFirstPnt = Tool.Pnt(FirstVertex);
        myLastPnt = Tool.Pnt(LastVertex);

        // SliList : list of faces concerned by the rib
        NCollection_List<Shape> SliList = new();
        SliList.Append(FirstFace);

        if (Sliding)
        { // sliding
            Surface s = Tool.Surface(FirstFace)!;
            if (s.GetType() == typeof(TrimmedSurface))
            {
                s = ((TrimmedSurface)s).BasisSurface();
            }
            if (s.GetType() != typeof(PlaneSurface) && s.GetType() != typeof(CylindricalSurface)
                && s.GetType() != typeof(ConicalSurface) && s.GetType() != typeof(ToroidalSurface))
            {
                Sliding = false;
            }
        }

        if (Sliding)
        { // sliding
            Surface ss = Tool.Surface(LastFace)!;
            if (ss.GetType() == typeof(TrimmedSurface))
            {
                ss = ((TrimmedSurface)ss).BasisSurface();
            }
            if (ss.GetType() != typeof(PlaneSurface) && ss.GetType() != typeof(CylindricalSurface)
                && ss.GetType() != typeof(ConicalSurface) && ss.GetType() != typeof(ToroidalSurface))
            {
                Sliding = false;
            }
        }

        // Control only start and end points no control at the middle to improve
        // If make a control between Surface and segment 2 points limited
        // -> too expensive - to improve
        Circle3D FirstCircle, LastCircle;
        Curve3D FirstCrv, LastCrv;

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
            Point3D FirstCenter = proj.Point(1);

            GeomAPI_ProjectPointOnCurve proj1 = new(myLastPnt, Line);
            if (proj.NbPoints() < 1) // OCCT: tests proj again, not proj1
            {
                myStatusError = BRepFeat_NoProjPt;
                NotDone();
                return;
            }
            double LastRayon = proj1.Distance(1);
            Point3D LastCenter = proj1.Point(1);

            Vector3D axv = new(myAxe.Direction());
            Frame3D ax2 = new(FirstCenter, new Direction3D(axv));
            Frame3D ax2p = new(LastCenter, new Direction3D(axv));
            Circle3D theFC = new(ax2, FirstRayon);
            Circle3D theLC = new(ax2p, LastRayon);

            Point3D RFirstPnt1 = myFirstPnt.Rotated(myAxe, myAngle1);
            Point3D RLastPnt1 = myLastPnt.Rotated(myAxe, myAngle1);
            Point3D RFirstPnt2 = myFirstPnt.Rotated(myAxe, -myAngle2);
            Point3D RLastPnt2 = myLastPnt.Rotated(myAxe, -myAngle2);

            BrepBuilder Bu = new();
            Vertex v1 = new(), v2 = new(), v3 = new(), v4 = new();
            Bu.MakeVertex(ref v1, RFirstPnt2, global::Kerf.Engine.Foundation.Precision.Confusion());
            Bu.MakeVertex(ref v2, RFirstPnt1, global::Kerf.Engine.Foundation.Precision.Confusion());
            Bu.MakeVertex(ref v3, RLastPnt2, global::Kerf.Engine.Foundation.Precision.Confusion());
            Bu.MakeVertex(ref v4, RLastPnt1, global::Kerf.Engine.Foundation.Precision.Confusion());

            BRepLib_MakeEdge ee1 = new(theFC, v1, v2);
            BRepLib_MakeEdge ee2 = new(theLC, v3, v4);

            if (Sliding && !PtOnFirstEdge)
            {
                BRepExtrema_ExtCF ext1 = new(ShapeCasts.Edge(ee1.InternalShape()), FirstFace);
                if (ext1.NbExt() < 1 || ext1.SquareDistance(1) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    Sliding = false;
                }
            }
            if (Sliding && !PtOnLastEdge)
            {
                BRepExtrema_ExtCF ext2 = new(ee2.InternalEdge(), LastFace); // ExtCF : curves and surfaces
                if (ext2.NbExt() < 1 || ext2.SquareDistance(1) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    Sliding = false;
                }
            }
            if (Sliding && PtOnFirstEdge)
            {
                FirstCrv = Tool.Curve(OnFirstEdge, out double f, out double l)!;
                if (FirstCrv.GetType() != typeof(CircleCurve3D))
                {
                    Sliding = false;
                }
                else
                {
                    CircleCurve3D C1 = (CircleCurve3D)FirstCrv;
                    Circle3D Circ = C1.Circ();
                    FirstCircle = Circ;
                    Axis3D circax = FirstCircle.Axis();
                    if (!circax.IsCoaxial(myAxe, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion()))
                    {
                        Sliding = false;
                    }
                    else
                    {
                        if (Math.Abs(FirstCircle.Radius() - FirstRayon) >= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            Sliding = false;
                        }
                    }
                }
            }

            if (Sliding && PtOnLastEdge)
            {
                LastCrv = Tool.Curve(OnLastEdge, out double f, out double l)!;
                if (LastCrv.GetType() != typeof(CircleCurve3D))
                {
                    Sliding = false;
                }
                else
                {
                    CircleCurve3D C1 = (CircleCurve3D)LastCrv;
                    Circle3D Circ = C1.Circ();
                    LastCircle = Circ;
                    Axis3D circax = LastCircle.Axis();
                    if (!circax.IsCoaxial(myAxe, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion()))
                    {
                        Sliding = false;
                    }
                    else
                    {
                        double rad = LastCircle.Radius();
                        if (Math.Abs(rad - LastRayon) >= global::Kerf.Engine.Foundation.Precision.Confusion())
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
            Face Prof = new();
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

        Wire w = new();
        BB.MakeWire(ref w);
        Edge thePreviousEdge = new();
        Vertex theFV = new();
        thePreviousEdge.Nullify();

        // counter of the number of edges to fill the map
        int counter = 1;

        // ---case of sliding
        if (Sliding && !myListOfEdges.IsEmpty())
        {
            WireExplorer EX1 = new(myWire);
            for (; EX1.More(); EX1.Next())
            {
                Edge E = EX1.Current();
                if (!myLFMap.IsBound(E))
                {
                    NCollection_List<Shape> theTmpList = new();
                    myLFMap.Bind(E, theTmpList);
                }
                if (E.IsSame(FirstEdge))
                {
                    Curve3D cc = Tool.Curve(E, out double f, out double l)!;
                    Point3D pt;
                    if (!FirstEdge.IsSame(LastEdge))
                    {
                        pt = Tool.Pnt(Exploration.LastVertex(E, true));
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
                    Edge ee1;
                    if (thePreviousEdge.IsNull)
                    {
                        BRepLib_MakeVertex v1 = new(myFirstPnt);
                        BRepLib_MakeVertex v2 = new(pt);
                        BRepLib_MakeEdge e = new(cc, v1.InternalVertex(), v2.InternalVertex());
                        ee1 = ShapeCasts.Edge(e.InternalShape());
                    }
                    else
                    {
                        Vertex v1 = Exploration.LastVertex(thePreviousEdge, true);
                        BRepLib_MakeVertex v2 = new(pt);
                        BRepLib_MakeEdge e = new(cc, v1, v2.InternalVertex());
                        ee1 = ShapeCasts.Edge(e.InternalShape());
                    }
                    Shape aLocalShape = ee1.Oriented(E.Orientation());
                    ee1 = ShapeCasts.Edge(aLocalShape);
                    if (counter == 1)
                    {
                        theFV = Exploration.FirstVertex(ee1, true);
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
                    Edge E = EX1.Current();
                    if (!myLFMap.IsBound(E))
                    {
                        NCollection_List<Shape> thelist1 = new();
                        myLFMap.Bind(E, thelist1);
                    }
                    theList.Append(E);
                    if (!E.IsSame(LastEdge))
                    {
                        Curve3D ccc = Tool.Curve(E, out double f, out double l)!;
                        Vertex v1, v2;
                        if (!thePreviousEdge.IsNull)
                        {
                            v1 = Exploration.LastVertex(thePreviousEdge, true);
                            v2 = Exploration.LastVertex(E, true);
                        }
                        else
                        {
                            v1 = Exploration.FirstVertex(E, true);
                            v2 = Exploration.LastVertex(E, true);
                        }
                        BRepLib_MakeEdge E1 = new(ccc, v1, v2);
                        Edge E11 = ShapeCasts.Edge(E1.InternalShape());
                        Shape aLocalShape = E11.Oriented(E.Orientation());
                        E11 = ShapeCasts.Edge(aLocalShape);
                        thePreviousEdge = E11;
                        myLFMap[E].Append(E11);
                        BB.Add(w, E11);
                        if (counter == 1)
                        {
                            theFV = Exploration.FirstVertex(E11, true);
                        }
                        counter++;
                    }
                    else
                    {
                        Curve3D cc = Tool.Curve(E, out double f, out double l)!;
                        Point3D pf = Tool.Pnt(Exploration.FirstVertex(E, true));
                        Point3D pl = myLastPnt;
                        Edge ee;
                        if (thePreviousEdge.IsNull)
                        {
                            BRepLib_MakeEdge e = new(cc, pf, pl);
                            ee = ShapeCasts.Edge(e.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(thePreviousEdge, true);
                            BRepLib_MakeVertex v2 = new(pl);
                            BRepLib_MakeEdge e = new(cc, v1, v2.InternalVertex());
                            ee = ShapeCasts.Edge(e.InternalShape());
                        }
                        Shape aLocalShape = ee.Oriented(E.Orientation());
                        ee = ShapeCasts.Edge(aLocalShape);
                        BB.Add(w, ee);
                        myLFMap[E].Append(ee);
                        if (counter == 1)
                        {
                            theFV = Exploration.FirstVertex(ee, true);
                        }
                        thePreviousEdge = ee;
                        counter++;
                        break;
                    }
                }
            }

            NCollection_List<Shape>.Iterator it = new(myListOfEdges);
            bool FirstOK = false;
            bool LastOK = false;

            Point3D theLastPnt = myLastPnt;
            int sens = 0;
            Edge theEdge = new(), theLEdge = new(), theFEdge = new();
            int counter1 = counter;
            NCollection_List<Shape> NewListOfEdges = new();
            NewListOfEdges.Clear();
            while (!FirstOK)
            {
                Edge edg = ShapeCasts.Edge(it.Value());
                Point3D fp, lp;
                Curve3D ccc = Tool.Curve(edg, out double f, out double l)!;
                TrimmedCurve3D cc = new(ccc, f, l);
                if (edg.Orientation() == Reversed)
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
                    Edge eeee;
                    double fpar = cc.FirstParameter();
                    double lpar = cc.LastParameter();
                    if (!FirstOK)
                    {
                        if (thePreviousEdge.IsNull)
                        {
                            BRepLib_MakeEdge e = new(cc, fpar, lpar);
                            eeee = ShapeCasts.Edge(e.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(thePreviousEdge, true);
                            BRepLib_MakeVertex v2 = new(cc.Value(lpar));
                            BRepLib_MakeEdge e = new(cc, v1, v2.InternalVertex());
                            eeee = ShapeCasts.Edge(e.InternalShape());
                        }
                    }
                    else
                    {
                        if (thePreviousEdge.IsNull)
                        {
                            BRepLib_MakeVertex v1 = new(cc.Value(fpar));
                            BRepLib_MakeEdge e = new(cc, v1.InternalVertex(), theFV);
                            eeee = ShapeCasts.Edge(e.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(thePreviousEdge, true);
                            BRepLib_MakeEdge e = new(cc, v1, theFV);
                            eeee = ShapeCasts.Edge(e.InternalShape());
                        }
                    }

                    thePreviousEdge = eeee;
                    BB.Add(w, eeee);
                    if (counter == 1)
                    {
                        theFV = Exploration.FirstVertex(eeee, true);
                    }
                    counter1++;
                    NewListOfEdges.Append(edg);
                    theEdge = eeee;

                    if (dist <= myTol)
                    {
                        theFEdge = edg;
                    }
                    theLastPnt = Tool.Pnt(Exploration.LastVertex(theEdge, true));
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

            NCollection_DataMap<Shape, NCollection_List<Shape>> SlidMap = new(ShapeHasher.Instance);
            SlidMap.Clear();

            if (Sliding && counter1 > counter)
            {
                Explorer EX2 = new(w, ShapeType.Edge);
                int ii = 0;
                for (; EX2.More(); EX2.Next())
                {
                    Edge E = ShapeCasts.Edge(EX2.Current());
                    ii++;
                    if (ii >= counter && ii <= counter1)
                    {
                        it.Initialize(NewListOfEdges);
                        int jj = 0;
                        for (; it.More(); it.Next())
                        {
                            Edge e2 = ShapeCasts.Edge(it.Value());
                            jj++;
                            if (jj == (ii - counter + 1))
                            {
                                foreach (KeyValuePair<Shape, NCollection_List<Shape>> itm in mySlface)
                                {
                                    Face fac = ShapeCasts.Face(itm.Key);
                                    NCollection_List<Shape> ledg = itm.Value;
                                    NCollection_List<Shape>.Iterator itedg = new(ledg);
                                    for (; itedg.More(); itedg.Next())
                                    {
                                        Edge e1 = ShapeCasts.Edge(itedg.Value());
                                        if (e1.IsSame(e2))
                                        {
                                            if (!SlidMap.IsBound(fac))
                                            {
                                                NCollection_List<Shape> thelist2 = new();
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
            Face F = new();
            BB.MakeFace(ref F, myPln, myTol);
            w.Closed(Tool.IsClosed(w));
            BB.Add(F, w);
            mySkface = F;
            myPbase = mySkface;
            mySUntil.Nullify();
        }

        // ---Case without sliding : construction of the face profile
        if (!Sliding)
        {
            Explorer explo1 = new(BndFace, ShapeType.Wire);
            Wire WWW = ShapeCasts.Wire(explo1.Current());
            WireExplorer explo = new(WWW);
            BrepBuilder Bu = new();
            Wire Wiwiwi = new();
            Bu.MakeWire(ref Wiwiwi);
            Vertex NewV1 = new(), NewV2 = new(), LastV = new();
            NewV1.Nullify();
            NewV2.Nullify();
            LastV.Nullify();

            for (; explo.More(); explo.Next())
            {
                Edge e = ShapeCasts.Edge(explo.Current());
                Vertex v1 = Exploration.FirstVertex(e, true);
                Vertex v2 = Exploration.LastVertex(e, true);

                Curve3D ln = Tool.Curve(e, out double f, out double l)!;
                Curve2D l2d = GeomAPI.To2d(ln, Plane.Pln())!;
                Geom2dAPI_InterCurveCurve intcc = new(l2d, ln2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                Vertex VV = new();
                VV.Nullify();

                if (intcc.NbPoints() > 0)
                {
                    Point2D P = intcc.Point(1);
                    myPln.D0(P.X(), P.Y(), out Point3D point);
                    double par = IntPar(ln, point);
                    if (f <= par && l >= par)
                    {
                        Bu.MakeVertex(ref VV, point, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                }

                if (VV.IsNull && NewV1.IsNull)
                {
                    continue;
                }

                if (!VV.IsNull && NewV1.IsNull)
                {
                    NewV1 = VV;
                    LastV = v2;
                    BRepLib_MakeEdge ee1 = new(NewV1, LastV);
                    Bu.Add(Wiwiwi, ee1.InternalEdge());
                    continue;
                }

                if (VV.IsNull && !NewV1.IsNull)
                {
                    BRepLib_MakeEdge ee1 = new(LastV, v2);
                    LastV = v2;
                    Bu.Add(Wiwiwi, e);
                    continue;
                }

                if (!VV.IsNull && !NewV1.IsNull)
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

            Wiwiwi.Closed(Tool.IsClosed(Wiwiwi));
            BRepLib_MakeFace newbndface = new(myPln.Pln(), Wiwiwi, true);
            Face NewBndFace = ShapeCasts.Face(newbndface.InternalShape());

            BRepTopAdaptor_FClass2d Cl = new(NewBndFace, global::Kerf.Engine.Foundation.Precision.Confusion());
            global::Kerf.Engine.Geometry.ElSLib.Parameters(myPln.Pln(), CheckPnt, out double paru, out double parv);
            Point2D checkpnt2d = new(paru, parv);
            if (Cl.Perform(checkpnt2d, true) == Out)
            {
                BRepAlgoAPI_Cut c = new(BndFace, NewBndFace);
                Explorer exp = new(c.InternalShape(), ShapeType.Wire);
                Wire aCurWire = ShapeCasts.Wire(exp.Current());
                BRepLib_MakeFace ff = new(myPln.Pln(), aCurWire, true);
                NewBndFace = ShapeCasts.Face(ff.InternalShape());
            }

            if (!BRepAlgo.IsValid(NewBndFace))
            {
                myStatusError = BRepFeat_InvShape;
                NotDone();
                return;
            }

            BndFace = NewBndFace;

            Face Prof = new();
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

            NCollection_List<Shape>.Iterator it = new();
            it.Initialize(SliList);

            Shell comp = new();
            BB.MakeShell(ref comp);

            for (; it.More(); it.Next())
            {
                BB.Add(comp, it.Value());
            }
            comp.Closed(Tool.IsClosed(comp));

            mySUntil = comp;

            mySkface = Prof;
            myPbase = Prof;
        }

        mySliding = Sliding;

        Explorer exp1 = new();
        for (exp1.Init(mySbase, ShapeType.Face); exp1.More(); exp1.Next())
        {
            NCollection_List<Shape> thelist3 = new();
            myMap.Bind(exp1.Current(), thelist3);
            myMap[exp1.Current()].Append(exp1.Current());
        }
    }

    /// <summary>Adds an element of sliding: the edge <paramref name="E"/> slides on the face <paramref name="F"/>.</summary>
    /// <exception cref="ConstructionErrorException">F isn't a face of the basis shape.</exception>
    public void Add(in Edge E, in Face F)
    {
        if (mySlface.IsEmpty())
        {
            Explorer exp = new();
            for (exp.Init(mySbase, ShapeType.Face); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(F))
                {
                    break;
                }
            }
            if (!exp.More())
            {
                throw new ConstructionErrorException();
            }

            if (!mySlface.IsBound(F))
            {
                NCollection_List<Shape> thelist = new();
                mySlface.Bind(F, thelist);
            }
            NCollection_List<Shape>.Iterator itl = new(mySlface[F]);
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
        if (mySbase.IsNull || mySkface.IsNull || myPbase.IsNull)
        {
            myStatusError = BRepFeat_NotInitialized;
            NotDone();
            return;
        }

        Point3D Pt = new();

        Explorer exx = new(myPbase, ShapeType.Vertex);
        for (; exx.More(); exx.Next())
        {
            Vertex vv = ShapeCasts.Vertex(exx.Current());
            if (!vv.IsNull)
            {
                Pt = Tool.Pnt(vv);
                break;
            }
        }

        if (myAngle2 != 0)
        {
            Transform3D T = new();
            T.SetRotation(myAxe, -myAngle2);
            BRepBuilderAPI_Transform trsf = new(T);
            trsf.Perform(myPbase, false);
            Face Pbase = ShapeCasts.Face(trsf.InternalShape());
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> iter in myLFMap)
            {
                Shape e1 = iter.Value.First();
                Explorer ex1 = new(myPbase, ShapeType.Edge);
                Explorer ex2 = new(Pbase, ShapeType.Edge);
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

            foreach (KeyValuePair<Shape, NCollection_List<Shape>> iter1 in mySlface)
            {
                Shape e1 = iter1.Value.First();
                Explorer ex1 = new(myPbase, ShapeType.Edge);
                Explorer ex2 = new(Pbase, ShapeType.Edge);
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
            mySkface = ShapeCasts.Face(trsf.InternalShape());
        }

        LocOpe_RevolutionForm theForm = new();
        theForm.Perform(myPbase, myAxe, myAngle1 + myAngle2);
        Shape VraiForm = theForm.InternalShape(); // uncut  primitive

        // management of descendants
        MajMap(myPbase, theForm, myMap, ref myFShape, ref myLShape);

        myGluedF.Clear();

        Plane3D Pln0 = myPln!.Pln();
        BRepLib_MakeFace f = new(Pln0);

        Vector3D vec1 = myHeight1 * new Vector3D(Normal(f.InternalFace(), Pt));
        Vector3D vec2 = -myHeight2 * new Vector3D(Normal(f.InternalFace(), Pt));

        Plane3D Pln1 = Pln0.Translated(vec1);
        Plane3D Pln2 = Pln0.Translated(vec2);

        BRepLib_MakeFace ff1 = new(Pln1);
        BRepLib_MakeFace ff2 = new(Pln2);
        Face f1 = ShapeCasts.Face(ff1.InternalShape());
        Face f2 = ShapeCasts.Face(ff2.InternalShape());
        BRepFeat.FaceUntil(mySbase, ref f1);
        BRepFeat.FaceUntil(mySbase, ref f2);

        LocOpe_CSIntersector ASI1 = new(f1);
        LocOpe_CSIntersector ASI2 = new(f2);

        LineCurve3D normale = new(Pt, new Direction3D(vec1));
        NCollection_Sequence<Curve3D?> scur = new();
        scur.Append(normale);

        ASI1.Perform(scur);
        ASI2.Perform(scur);

        if (!ASI1.IsDone() || !ASI2.IsDone() || ASI1.NbPoints(1) != 1 || ASI2.NbPoints(1) != 1)
        {
            myStatusError = BRepFeat_BadIntersect;
            NotDone();
            return;
        }

        Orientation Ori1 = ASI1.Point(1, 1).Orientation();
        Orientation Ori2 = global::Kerf.Engine.Topology.TopAbs.Reverse(ASI2.Point(1, 1).Orientation());
        Face FF1 = ASI1.Point(1, 1).Face();
        Face FF2 = ASI2.Point(1, 1).Face();

        Compound Comp = new();
        BrepBuilder B = new();
        B.MakeCompound(ref Comp);
        Solid S1 = BRepFeat.Tool(f1, FF1, Ori1);
        Solid S2 = BRepFeat.Tool(f2, FF2, Ori2);
        if (!S1.IsNull)
        {
            B.Add(Comp, S1);
        }
        if (!S2.IsNull)
        {
            B.Add(Comp, S2);
        }

        BRepAlgoAPI_Cut trP = new(VraiForm, Comp);
        // coupe de la nervure par deux plans parallels
        NCollection_DataMap<Shape, NCollection_List<Shape>> SlidingMap = new(ShapeHasher.Instance);

        // management of descendants
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it1 in myMap)
        {
            Shape orig = it1.Key;
            if (it1.Value.IsEmpty())
            {
                continue;
            }
            Shape sh = it1.Value.First();
            exx.Init(VraiForm, ShapeType.Face);
            for (; exx.More(); exx.Next())
            {
                Face fac = ShapeCasts.Face(exx.Current());
                Explorer exx1 = new(fac, ShapeType.Wire);
                Wire thew = ShapeCasts.Wire(exx1.Current());
                if (thew.IsSame(myFShape))
                {
                    myMap[myFShape] = new NCollection_List<Shape>(trP.InternalModified(f2));
                    continue;
                }
                else if (thew.IsSame(myLShape))
                {
                    myMap[myLShape] = new NCollection_List<Shape>(trP.InternalModified(f1));
                    continue;
                }
                if (fac.IsSame(sh))
                {
                    if (!trP.IsDeleted(fac))
                    {
                        NCollection_List<Shape> desfaces = trP.InternalModified(fac);
                        if (!desfaces.IsEmpty())
                        {
                            myMap[orig].Clear();
                            myMap[orig] = new NCollection_List<Shape>(trP.InternalModified(fac));
                            break; // break the cycle (sh became a dead reference)
                        }
                    }
                }
            }
        }

        exx.Init(VraiForm, ShapeType.Face);
        for (; exx.More(); exx.Next())
        {
            Face fac = ShapeCasts.Face(exx.Current());
            NCollection_List<Shape> thelist = new();
            SlidingMap.Bind(fac, thelist);
            if (trP.IsDeleted(fac))
            {
            }
            else
            {
                NCollection_List<Shape> desfaces = trP.InternalModified(fac);
                if (!desfaces.IsEmpty())
                {
                    SlidingMap[fac] = new NCollection_List<Shape>(desfaces);
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

        exx.Init(myPbase, ShapeType.Edge);
        for (; exx.More(); exx.Next())
        {
            Edge e = ShapeCasts.Edge(exx.Current());
            if (!myMap.IsBound(e))
            {
                myStatusError = BRepFeat_IncSlidFace;
                NotDone();
                return;
            }
        }

        myGShape = VraiForm;

        if (!myGluedF.IsEmpty() && !mySUntil.IsNull)
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
    public bool Propagate(NCollection_List<Shape> SliList, in Face fac, in Point3D Firstpnt,
                          in Point3D Lastpnt, ref bool falseside)
    {
        Point3D Firstpoint = Firstpnt;
        Point3D Lastpoint = Lastpnt;

        bool result = true;
        Face CurrentFace, saveFace;
        CurrentFace = ShapeCasts.Face(SliList.First());
        saveFace = CurrentFace;

        bool LastOK = false, FirstOK = false;
        Vertex v1 = new(), v2 = new(), v3, v4, Vert;

        BRepAlgoAPI_Section sect = new(fac, CurrentFace, false);
        sect.Approximation(true);
        sect.Build();
        Explorer Ex = new();
        Edge e = new(), e1 = new();
        Point3D FP, LP;
        int ii = 0;
        for (Ex.Init(sect.InternalShape(), ShapeType.Edge); Ex.More(); Ex.Next())
        {
            ii++;
            if (ii == 1)
            {
                e = ShapeCasts.Edge(Ex.Current());
            }
            else if (ii > 1)
            {
                e1 = ShapeCasts.Edge(Ex.Current());
                break;
            }
        }
        if (e.IsNull)
        {
            falseside = false;
            return false;
        }
        //
        if (!e1.IsNull)
        {
            double aTolV1, aTolV2;
            myListOfEdges.Clear();
            NCollection_List<Shape> thelist = new();
            mySlface.Bind(CurrentFace, thelist);
            mySlface[CurrentFace].Append(e1);

            myListOfEdges.Append(e1);

            v1 = Exploration.FirstVertex(e1, true);
            v2 = Exploration.LastVertex(e1, true);

            FP = Tool.Pnt(v1);
            LP = Tool.Pnt(v2);

            aTolV1 = Tool.Tolerance(v1);
            aTolV2 = Tool.Tolerance(v2);

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
        if (!e1.IsNull)
        {
            myListOfEdges.Clear();
            NCollection_List<Shape> thelist1 = new();
            mySlface.Bind(CurrentFace, thelist1);
            mySlface[CurrentFace].Append(e);

            myListOfEdges.Append(e);

            mySlface[CurrentFace].Append(e1);

            v1 = Exploration.FirstVertex(e, true);
            v2 = Exploration.LastVertex(e, true);
            v3 = Exploration.FirstVertex(e1, true);
            v4 = Exploration.LastVertex(e1, true);
            Point3D p1, p2, p3, p4;
            p1 = Tool.Pnt(v1);
            FP = p1;
            p2 = Tool.Pnt(v2);
            LP = p2;
            p3 = Tool.Pnt(v3);
            p4 = Tool.Pnt(v4);
            if (p1.Distance(Firstpoint) <= Tool.Tolerance(v1))
            {
                if (p3.Distance(Lastpoint) <= Tool.Tolerance(v3))
                {
                    FirstOK = true;
                    Lastpoint = p4;
                }
                else if (p4.Distance(Lastpoint) <= Tool.Tolerance(v4))
                {
                    FirstOK = true;
                    Lastpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else if (p1.Distance(Lastpoint) <= Tool.Tolerance(v1))
            {
                if (p3.Distance(Firstpoint) <= Tool.Tolerance(v3))
                {
                    FirstOK = true;
                    Firstpoint = p4;
                }
                else if (p4.Distance(Firstpoint) <= Tool.Tolerance(v4))
                {
                    FirstOK = true;
                    Firstpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else if (p2.Distance(Firstpoint) <= Tool.Tolerance(v2))
            {
                if (p3.Distance(Lastpoint) <= Tool.Tolerance(v3))
                {
                    LastOK = true;
                    Lastpoint = p4;
                }
                else if (p4.Distance(Lastpoint) <= Tool.Tolerance(v4))
                {
                    LastOK = true;
                    Lastpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else if (p2.Distance(Lastpoint) <= Tool.Tolerance(v2))
            {
                if (p3.Distance(Firstpoint) <= Tool.Tolerance(v3))
                {
                    LastOK = true;
                    Firstpoint = p4;
                }
                else if (p4.Distance(Firstpoint) <= Tool.Tolerance(v4))
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
        if (e1.IsNull)
        {
            myListOfEdges.Clear();
            NCollection_List<Shape> thelist2 = new();
            mySlface.Bind(CurrentFace, thelist2);
            mySlface[CurrentFace].Append(e);

            myListOfEdges.Append(e);

            v1 = Exploration.FirstVertex(e, true);
            v2 = Exploration.LastVertex(e, true);

            FP = Tool.Pnt(v1);
            LP = Tool.Pnt(v2);

            if (FP.Distance(Firstpoint) <= Tool.Tolerance(v1) || FP.Distance(Lastpoint) <= Tool.Tolerance(v1))
            {
                FirstOK = true;
            }
            if (LP.Distance(Firstpoint) <= Tool.Tolerance(v2) || LP.Distance(Lastpoint) <= Tool.Tolerance(v2))
            {
                LastOK = true;
            }

            if (LastOK && FirstOK)
            {
                return result;
            }
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapedges = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(mySbase, ShapeType.Edge, ShapeType.Face, mapedges);
        Explorer ex = new();
        Edge FirstEdge = new();

        Vertex Vprevious = new();
        Vprevious.Nullify();
        Vertex Vpreprevious = new();
        Vpreprevious.Nullify();

        while (!FirstOK)
        {
            // find edge connected to v1:
            Point3D pt = new();
            if (!v1.IsNull)
            {
                pt = Tool.Pnt(v1);
            }
            Point3D ptprev = new();
            if (!Vprevious.IsNull)
            {
                ptprev = Tool.Pnt(Vprevious);
            }
            Point3D ptpreprev = new();
            if (!Vpreprevious.IsNull)
            {
                ptpreprev = Tool.Pnt(Vpreprevious);
            }

            if ((!Vprevious.IsNull && ptprev.Distance(pt) <= myTol)
                || (!Vpreprevious.IsNull && ptpreprev.Distance(pt) <= myTol))
            {
                falseside = false;
                return false;
            }

            for (ex.Init(CurrentFace, ShapeType.Edge); ex.More(); ex.Next())
            {
                Edge aCurEdge = ShapeCasts.Edge(ex.Current());

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
                        if (dist2min <= Tool.Tolerance(aCurEdge) * Tool.Tolerance(aCurEdge))
                        {
                            FirstEdge = aCurEdge;
                            break;
                        }
                    }
                }
            }

            NCollection_List<Shape> L = mapedges.FindFromKey(FirstEdge);
            NCollection_List<Shape>.Iterator It = new(L);

            for (; It.More(); It.Next())
            {
                Face FF = ShapeCasts.Face(It.Value());
                if (!FF.IsSame(CurrentFace))
                {
                    CurrentFace = FF;
                    break;
                }
            }

            BRepAlgoAPI_Section sectf = new(fac, CurrentFace, false);
            sectf.Approximation(true);
            sectf.Build();

            Edge edg1 = new();
            for (Ex.Init(sectf.InternalShape(), ShapeType.Edge); Ex.More(); Ex.Next())
            {
                edg1 = ShapeCasts.Edge(Ex.Current());
                Point3D ppp1 = Tool.Pnt(Exploration.FirstVertex(edg1, true));
                Point3D ppp2 = Tool.Pnt(Exploration.LastVertex(edg1, true));
                if (ppp1.Distance(Tool.Pnt(v1)) <= Tool.Tolerance(v1)
                    || ppp2.Distance(Tool.Pnt(v1)) <= Tool.Tolerance(v1))
                {
                    break;
                }
            }

            NCollection_List<Shape> thelist3 = new();
            mySlface.Bind(CurrentFace, thelist3);
            mySlface[CurrentFace].Append(edg1);
            myListOfEdges.Append(edg1);

            if (!edg1.IsNull)
            {
                SliList.Prepend(CurrentFace);
            }
            else
            {
                return false;
            }

            Vert = Exploration.FirstVertex(edg1, true);
            Point3D PP = Tool.Pnt(Vert);
            FP = Tool.Pnt(v1);
            double tol = Tool.Tolerance(edg1);
            double tol1 = Tool.Tolerance(v1);
            if (tol1 > tol)
            {
                tol = tol1;
            }
            double dist = PP.Distance(FP);
            if (dist <= tol)
            {
                Vpreprevious = Vprevious;
                Vprevious = v1;
                v1 = Exploration.LastVertex(edg1, true);
            }
            else
            {
                Vpreprevious = Vprevious;
                Vprevious = v1;
                v1 = Vert;
            }

            FP = Tool.Pnt(v1);

            if (FP.Distance(Firstpoint) <= Tool.Tolerance(v1) || FP.Distance(Lastpoint) <= Tool.Tolerance(v1))
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
            Point3D pt = new();
            if (!v2.IsNull)
            {
                pt = Tool.Pnt(v2);
            }
            Point3D ptprev = new();
            if (!Vprevious.IsNull)
            {
                ptprev = Tool.Pnt(Vprevious);
            }
            Point3D ptpreprev = new();
            if (!Vpreprevious.IsNull)
            {
                ptpreprev = Tool.Pnt(Vpreprevious);
            }

            if ((!Vprevious.IsNull && ptprev.Distance(pt) <= myTol)
                || (!Vpreprevious.IsNull && ptpreprev.Distance(pt) <= myTol))
            {
                falseside = false;
                return false;
            }

            for (ex.Init(CurrentFace, ShapeType.Edge); ex.More(); ex.Next())
            {
                Edge aCurEdge = ShapeCasts.Edge(ex.Current());
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
                        if (dist2min <= Tool.Tolerance(aCurEdge) * Tool.Tolerance(aCurEdge))
                        {
                            FirstEdge = aCurEdge;
                            break;
                        }
                    }
                }
            }

            NCollection_List<Shape> L = mapedges.FindFromKey(FirstEdge);
            NCollection_List<Shape>.Iterator It = new(L);

            for (; It.More(); It.Next())
            {
                Face FF = ShapeCasts.Face(It.Value());
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

            Edge edg2 = new();
            for (Ex.Init(sectf.InternalShape(), ShapeType.Edge); Ex.More(); Ex.Next())
            {
                edg2 = ShapeCasts.Edge(Ex.Current());
                Point3D ppp1 = Tool.Pnt(Exploration.FirstVertex(edg2, true));
                Point3D ppp2 = Tool.Pnt(Exploration.LastVertex(edg2, true));
                if (ppp1.Distance(Tool.Pnt(v2)) <= Tool.Tolerance(v2)
                    || ppp2.Distance(Tool.Pnt(v2)) <= Tool.Tolerance(v2))
                {
                    break;
                }
            }
            NCollection_List<Shape> thelist4 = new();
            mySlface.Bind(CurrentFace, thelist4);
            mySlface[CurrentFace].Append(edg2);
            myListOfEdges.Append(edg2);

            if (!edg2.IsNull)
            {
                SliList.Append(CurrentFace);
            }
            else
            {
                return false;
            }

            Vert = Exploration.FirstVertex(edg2, true);
            Point3D PP = Tool.Pnt(Vert);
            FP = Tool.Pnt(v2);
            if (PP.Distance(FP) <= Tool.Tolerance(v2))
            {
                Vpreprevious = Vprevious;
                Vprevious = v2;
                v2 = Exploration.LastVertex(edg2, true);
            }
            else
            {
                v2 = Vert;
            }
            FP = Tool.Pnt(v2);

            if (FP.Distance(Firstpoint) <= Tool.Tolerance(v2) || FP.Distance(Lastpoint) <= Tool.Tolerance(v2))
            {
                LastOK = true;
            }
        }
        if (!e1.IsNull)
        {
            myListOfEdges.Append(e1);
        }
        return result;
    }

    private static void MajMap(in Shape theB, LocOpe_RevolutionForm theP,
                               NCollection_DataMap<Shape, NCollection_List<Shape>> theMap,
                               ref Shape theFShape, ref Shape theLShape)
    {
        Explorer exp = new(theP.InternalFirstShape(), ShapeType.Wire);
        if (exp.More())
        {
            theFShape = exp.Current();
            NCollection_List<Shape> thelist = new();
            theMap.Bind(theFShape, thelist);
            for (exp.Init(theP.InternalFirstShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                Shape sh = exp.Current();
                theMap[theFShape].Append(sh);
            }
        }

        exp.Init(theP.InternalLastShape(), ShapeType.Wire);
        if (exp.More())
        {
            theLShape = exp.Current();
            NCollection_List<Shape> thelist1 = new();
            theMap.Bind(theLShape, thelist1);
            for (exp.Init(theP.InternalLastShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                Shape sh = exp.Current();
                theMap[theLShape].Append(sh);
            }
        }

        for (exp.Init(theB, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!theMap.IsBound(exp.Current()))
            {
                theMap.Bind(exp.Current(), new NCollection_List<Shape>(theP.InternalShapes(exp.Current())));
            }
        }
    }

    private static void SetGluedFaces(NCollection_DataMap<Shape, NCollection_List<Shape>> theSlmap,
                                      LocOpe_RevolutionForm thePrism,
                                      NCollection_DataMap<Shape, NCollection_List<Shape>> SlidingMap,
                                      NCollection_DataMap<Shape, Shape> theMap)
    {
        // Slidings
        if (!theSlmap.IsEmpty())
        {
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> itm in theSlmap)
            {
                Face fac = ShapeCasts.Face(itm.Key);
                NCollection_List<Shape> ledg = itm.Value;
                NCollection_List<Shape>.Iterator it = new();
                for (it.Initialize(ledg); it.More(); it.Next())
                {
                    NCollection_List<Shape> gfac = thePrism.InternalShapes(it.Value());
                    foreach (KeyValuePair<Shape, NCollection_List<Shape>> iterm in SlidingMap)
                    {
                        Face ff = ShapeCasts.Face(iterm.Key);
                        NCollection_List<Shape> lfaces = iterm.Value;
                        if (lfaces.IsEmpty())
                        {
                            continue;
                        }
                        Face fff = ShapeCasts.Face(lfaces.First());
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
