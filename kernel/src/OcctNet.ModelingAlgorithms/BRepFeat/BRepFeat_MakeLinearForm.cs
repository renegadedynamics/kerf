// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeLinearForm.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeLinearForm.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeLinearForm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Builds a rib or a groove along a developable, planar surface: a contour (an open wire) in a plane, thickened along
/// one or two directions until it meets the basis shape.
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a groove (a cut), 1 for a rib (a fusion). With sliding (<c>Modify</c>), the rib slides on
/// the faces of the basis shape it meets.</para>
/// <para>The basis shape and the contour are read, never written. OCCT raises the tolerance of a vertex of the profile
/// it builds, which may be the contour's own, and translates the plane it is given: a frozen contour is worked on as a
/// copy (its unchanged sub-shapes come back in the result, its edges answer <see cref="BRepBuilderAPI_MakeShape.Generated"/>),
/// a frozen plane is translated on a copy (decision 1052).</para>
/// </remarks>
public sealed class BRepFeat_MakeLinearForm : BRepFeat_RibSlot
{
#pragma warning disable CS0169 // declared, never used: as in OCCT
    private Geom_Curve? myCrv;
#pragma warning restore CS0169
    private Vector3D myDir = new();
    private Vector3D myDir1 = new();
    private Geom_Plane? myPln;
    private double myBnd;
    private NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySlface = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_List<TopoDS_Shape> myListOfEdges = new();
    private double myTol;
    // OCCT.NET: the contour's copy for write (decision 1052), ended after the first Perform
    private BRepTools_CopyAtEntry? myWireCopy;

    /// <summary>An empty linear form.</summary>
    public BRepFeat_MakeLinearForm()
    {
        myBnd = 0.0;
        myTol = 0.0;
    }

    /// <summary>
    /// A rib or groove of the contour <paramref name="W"/> in the plane <paramref name="P"/> on
    /// <paramref name="Sbase"/>, thickened along <paramref name="Direction"/> and <paramref name="Direction1"/> (null
    /// for one side); <paramref name="Fuse"/> 0 cuts, 1 fuses; <paramref name="Modify"/> asks for sliding.
    /// </summary>
    public BRepFeat_MakeLinearForm(in TopoDS_Shape Sbase, in TopoDS_Wire W, Geom_Plane P, in Vector3D Direction,
                                   in Vector3D Direction1, int Fuse, bool Modify)
    {
        Init(Sbase, W, P, Direction, Direction1, Fuse, Modify);
    }

    /// <summary>Initializes the linear form (see the constructor).</summary>
    public void Init(in TopoDS_Shape Sbase, in TopoDS_Wire W, Geom_Plane Plane, in Vector3D Direc, in Vector3D Direc1,
                     int Mode, bool Modify)
    {
        bool RevolRib = false;
        Done();
        myGenerated.Clear();

        // modify = 0 if there is no intention to make sliding
        //        = 1 if one tries to make sliding
        bool Sliding = Modify;
        myLFMap.Clear();

        myShape.Nullify();
        myMap.Clear();
        myFShape.Nullify();
        myLShape.Nullify();
        mySbase = Sbase;
        mySkface.Nullify();
        myPbase.Nullify();

        myGShape.Nullify();
        mySUntil.Nullify();
        myListOfEdges.Clear();
        mySlface.Clear();

        // OCCT.NET: the profile is built of the contour's vertices, one of which may get a raised tolerance: a frozen
        // contour is worked on as a copy (decision 1052)
        myWireCopy = new BRepTools_CopyAtEntry(W);
        TopoDS_Shape aLocalShapeW = myWireCopy.Shape().Oriented(TopAbs_FORWARD);
        myWire = TopoDS.Wire(aLocalShapeW);
        myDir = Direc;
        myDir1 = Direc1;
        myPln = Plane;

        myFuse = Mode != 0;

        // ---Determine Tolerance : max tolerance on parameters
        myTol = global::Kerf.Engine.Foundation.Precision.Confusion();

        TopExp_Explorer exx = new();
        exx.Init(myWire, TopAbs_VERTEX);
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

        // ---Control of directions
        //    the wire should be in the rib
        Vector3D nulldir = new(0, 0, 0);
        if (!myDir1.IsEqual(nulldir, myTol, myTol))
        {
            double ang = myDir1.Angle(myDir);
            if (ang != M_PI)
            {
                myStatusError = BRepFeat_BadDirect;
                NotDone();
                return;
            }
        }
        else
        {
            // Rib is centre in the middle of translation
            Vector3D DirTranslation = (Direc + Direc1) * 0.5;
            Transform3D T = new();
            T.SetTranslation(DirTranslation);
            BRepBuilderAPI_Transform trf = new(T);
            trf.Perform(myWire);
            myWire = TopoDS.Wire(trf.InternalShape());
            myDir = Direc - DirTranslation;
            myDir1 = Direc1 - DirTranslation;
            // OCCT.NET: OCCT translates the caller's plane; a frozen one is translated on a copy (decision 974)
            if (myPln.IsFrozen)
            {
                myPln = (Geom_Plane)myPln.Copy();
            }
            myPln.Transform(T);
        }

        // ---Calculate bounding box
        BRep_Builder BB = new();

        NCollection_List<TopoDS_Shape> theList = new();

        TopoDS_Shape U = new();
        U.Nullify();
        Point3D FirstCorner = new(), LastCorner = new();
        double bnd = HeightMax(mySbase, U, ref FirstCorner, ref LastCorner);
        myBnd = bnd;

        BRepPrimAPI_MakeBox Bndbox = new(FirstCorner, LastCorner);
        TopoDS_Solid BndBox = Bndbox.InternalSolid();

        // ---Construction of the face workplane (section bounding box)
        BRepLib_MakeFace PlaneF = new(myPln.Pln(), -6.0 * myBnd, 6.0 * myBnd, -6.0 * myBnd, 6.0 * myBnd);
        TopoDS_Face PlaneFace = TopoDS.Face(PlaneF.InternalShape());

        BRepAlgoAPI_Common PlaneS = new(BndBox, PlaneFace);
        TopExp_Explorer EXP = new();
        TopoDS_Shape PlaneSect = PlaneS.InternalShape();
        EXP.Init(PlaneSect, TopAbs_WIRE);
        TopoDS_Wire www = TopoDS.Wire(EXP.Current());
        BRepLib_MakeFace Bndface = new(myPln.Pln(), www, true);
        TopoDS_Face BndFace = TopoDS.Face(Bndface.InternalShape());

        // ---Find support faces of the rib
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

        // ---Proofing Point for the side of the wire to be filled - side material
        Point3D CheckPnt = CheckPoint(FirstEdge, bnd / 10.0, myPln);

        // ---Control sliding valuable
        // Many cases when the sliding is abandoned
        int Concavite = 3; // a priori the profile is not concave

        myFirstPnt = BRep_Tool.Pnt(FirstVertex);
        myLastPnt = BRep_Tool.Pnt(LastVertex);

        // SliList : list of faces concerned by the rib
        NCollection_List<TopoDS_Shape> SliList = new();
        SliList.Append(FirstFace);

        if (Sliding)
        { // sliding
            Sliding = false;
            Geom_Surface s = BRep_Tool.Surface(FirstFace)!;
            if (s.GetType() == typeof(Geom_RectangularTrimmedSurface))
            {
                s = ((Geom_RectangularTrimmedSurface)s).BasisSurface();
            }
            if (s.GetType() == typeof(Geom_Plane) || s.GetType() == typeof(Geom_CylindricalSurface))
            {
                // if plane or cylinder : sliding is possible
                Sliding = true;
            }
        }

        // Control only start and end points
        // -> no control at the middle - improve
        // Controle between Surface and segment between 2 limit points
        // is too expensive - improve
        if (Sliding)
        {
            Point3D p1 = new(myFirstPnt.X() + myDir.X(), myFirstPnt.Y() + myDir.Y(), myFirstPnt.Z() + myDir.Z());
            BRepLib_MakeEdge ee1 = new(myFirstPnt, p1);
            BRepExtrema_ExtCF ext1 = new(ee1.InternalEdge(), FirstFace);
            if (ext1.NbExt() == 1
                && ext1.SquareDistance(1) <= BRep_Tool.Tolerance(FirstFace) * BRep_Tool.Tolerance(FirstFace))
            {
                Point3D p2 = new(myLastPnt.X() + myDir.X(), myLastPnt.Y() + myDir.Y(), myLastPnt.Z() + myDir.Z());
                BRepLib_MakeEdge ee2 = new(myLastPnt, p2);
                BRepExtrema_ExtCF ext2 = new(ee2.InternalEdge(), LastFace); // ExtCF : curves and surfaces
                Sliding = ext2.NbExt() == 1
                          && ext2.SquareDistance(1) <= BRep_Tool.Tolerance(LastFace) * BRep_Tool.Tolerance(LastFace);
            }
            else
            {
                Sliding = false;
            }
        }

        if (!myDir1.IsEqual(nulldir, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            if (Sliding)
            {
                Point3D p1 = new(myFirstPnt.X() + myDir1.X(), myFirstPnt.Y() + myDir1.Y(), myFirstPnt.Z() + myDir1.Z());
                BRepLib_MakeEdge ee1 = new(myFirstPnt, p1);
                BRepExtrema_ExtCF ext1 = new(ee1.InternalEdge(), FirstFace);
                if (ext1.NbExt() == 1
                    && ext1.SquareDistance(1) <= BRep_Tool.Tolerance(FirstFace) * BRep_Tool.Tolerance(FirstFace))
                {
                    Point3D p2 = new(myLastPnt.X() + myDir1.X(), myLastPnt.Y() + myDir1.Y(), myLastPnt.Z() + myDir1.Z());
                    BRepLib_MakeEdge ee2 = new(myLastPnt, p2);
                    BRepExtrema_ExtCF ext2 = new(ee2.InternalEdge(), LastFace);
                    Sliding = ext2.NbExt() == 1
                              && ext2.SquareDistance(1) <= BRep_Tool.Tolerance(LastFace) * BRep_Tool.Tolerance(LastFace);
                }
                else
                {
                    Sliding = false;
                }
            }
        }

        // Construct a great profile that goes till the bounding box
        // -> by tangency with the first and the last edge of the Wire
        // -> by normals to the support faces : statistically better
        // Intersect everything to find the final profile

        // ---case of sliding : construction of the profile face
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

        // ---Generation of the base of the rib profile

        TopoDS_Wire w = new();
        BB.MakeWire(ref w);
        TopoDS_Edge thePreviousEdge = new();
        TopoDS_Vertex theFV = new();
        thePreviousEdge.Nullify();

        // calculate the number of edges to fill the map
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
                    cc = new Geom_TrimmedCurve(cc, f, l);
                    Point3D pt;
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
                        Point3D pf = BRep_Tool.Pnt(TopExp.FirstVertex(E, true));
                        Point3D pl = myLastPnt;
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

            Point3D theLastPnt = myLastPnt;
            int sens = 0;
            TopoDS_Edge theEdge = new(), theLEdge = new(), theFEdge = new();
            int counter1 = counter;
            NCollection_List<TopoDS_Shape> NewListOfEdges = new();
            NewListOfEdges.Clear();
            while (!FirstOK)
            {
                TopoDS_Edge edg = TopoDS.Edge(it.Value());
                Point3D fp, lp;
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
                            BB.UpdateVertex(v1, dist);
                            BRepLib_MakeVertex v2 = new(cc.Value(lpar));
                            TopoDS_Vertex nv = v2.InternalVertex();
                            BRepLib_MakeEdge e = new(cc, v1, nv);
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
                    myListOfEdges.Remove(it);
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

        // ---Arguments of LocOpe_LinearForm : arguments of the prism sliding
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

        // ---Case without sliding : construction of the profile face
        if (!Sliding)
        {
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

            // ---Propagation on faces of the initial shape
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

        TopExp_Explorer exp = new();
        for (exp.Init(mySbase, TopAbs_FACE); exp.More(); exp.Next())
        {
            NCollection_List<TopoDS_Shape> thelist3 = new();
            myMap.Bind(exp.Current(), thelist3);
            myMap[exp.Current()].Append(exp.Current());
        }
    }

    /// <summary>Adds an element of sliding: the edge <paramref name="E"/> slides on the face <paramref name="F"/>.</summary>
    /// <exception cref="ConstructionErrorException">F isn't a face of the basis shape.</exception>
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
                throw new ConstructionErrorException();
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

        Vector3D nulldir = new(0, 0, 0);

        double Length = myDir.Magnitude() + myDir1.Magnitude();

        myGluedF.Clear();

        if (!mySUntil.IsNull())
        {
            myPerfSelection = BRepFeat_SelectionU;
        }
        else
        {
            myPerfSelection = BRepFeat_NoSelection;
        }

        Direction3D dir = new(myDir);
        Vector3D V = Length * new Vector3D(dir);

        LocOpe_LinearForm theForm = new();

        if (myDir1.IsEqual(nulldir, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            theForm.Perform(myPbase, V, myFirstPnt, myLastPnt);
        }
        else
        {
            theForm.Perform(myPbase, V, myDir1, myFirstPnt, myLastPnt);
        }

        TopoDS_Shape VraiForm = theForm.InternalShape(); // primitive of the rib

        myFacesForDraft.Append(theForm.InternalFirstShape());
        myFacesForDraft.Append(theForm.InternalLastShape());
        MajMap(myPbase, theForm, myMap, ref myFShape, ref myLShape); // management of descendants

        TopExp_Explorer exx = new(myPbase, TopAbs_EDGE);
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
        SetGluedFaces(mySlface, theForm, myGluedF); // management of sliding faces

        if (!myGluedF.IsEmpty() && !mySUntil.IsNull())
        {
            myStatusError = BRepFeat_InvShape;
            NotDone();
            return;
        }

        LFPerform();
        FinishTheContour();
    }

    // OCCT.NET: ends the contour's copy (decision 1052): the result and the history get the caller's sub-shapes back
    // where the construction left them unchanged
    private void FinishTheContour()
    {
        if (myWireCopy is null || !myWireCopy.IsCopied())
        {
            return;
        }
        BRepTools_CopyAtEntry aCopy = myWireCopy;
        myWireCopy = null;
        var anOthers = new List<TopoDS_Shape>();
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myMap)
        {
            anOthers.AddRange(anItem.Value);
        }
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myLFMap)
        {
            anOthers.AddRange(anItem.Value);
        }
        anOthers.AddRange(myNewEdges);
        anOthers.AddRange(myTgtEdges);
        anOthers.AddRange(myFacesForDraft);
        myShape = aCopy.Finish(myShape, anOthers);
        foreach (TopoDS_Shape aKey in myMap.Keys.ToList())
        {
            myMap[aKey] = Final(aCopy, myMap[aKey]);
        }
        // the contour's edges answer Generated: keyed by the caller's edges
        var aLFMap = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myLFMap)
        {
            aLFMap.Bind(aCopy.Original(anItem.Key), Final(aCopy, anItem.Value));
        }
        myLFMap = aLFMap;
        myNewEdges = Final(aCopy, myNewEdges);
        myTgtEdges = Final(aCopy, myTgtEdges);
        myFacesForDraft = Final(aCopy, myFacesForDraft);
    }

    private static NCollection_List<TopoDS_Shape> Final(BRepTools_CopyAtEntry theCopy, NCollection_List<TopoDS_Shape> theList)
    {
        NCollection_List<TopoDS_Shape> aList = new();
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            aList.Append(theCopy.Final(anIt.Value()));
        }
        return aList;
    }

    /// <summary>Limitation of the shape until the case of infinite faces: does nothing (as in OCCT).</summary>
    public void TransformShapeFU(int flag)
    {
    }

    /// <summary>
    /// Propagates the profile <paramref name="fac"/> over the faces of the basis shape from the first face of
    /// <paramref name="SliList"/>, adding the faces the rib meets; <paramref name="falseside"/> becomes false when the
    /// material isn't on the profile's side.
    /// </summary>
    public bool Propagate(NCollection_List<TopoDS_Shape> SliList, in TopoDS_Face fac, in Point3D Firstpnt,
                          in Point3D Lastpnt, ref bool falseside)
    {
        Point3D Firstpoint = Firstpnt;
        Point3D Lastpoint = Lastpnt;

        bool result = true;
        TopoDS_Face CurrentFace, saveFace;
        CurrentFace = TopoDS.Face(SliList.First());
        saveFace = CurrentFace;

        bool LastOK = false, FirstOK = false;
        bool v1OK = false, v2OK = false;
        TopoDS_Vertex v1 = new(), v2 = new(), ve1, ve2;

        BRepAlgoAPI_Section sect = new(fac, CurrentFace, false);

        sect.Approximation(true);
        sect.Build();

        TopExp_Explorer Ex = new();
        TopoDS_Edge eb = new(), ec;
        Point3D p1 = new(), p2 = new();
        double t1 = 0.0, t2 = 0.0;
        bool c1f, c2f, c1l, c2l;

        for (Ex.Init(sect.InternalShape(), TopAbs_EDGE); Ex.More(); Ex.Next())
        {
            ec = TopoDS.Edge(Ex.Current());
            v1 = TopExp.FirstVertex(ec, true);
            v2 = TopExp.LastVertex(ec, true);
            p1 = BRep_Tool.Pnt(v1);
            p2 = BRep_Tool.Pnt(v2);
            t1 = BRep_Tool.Tolerance(v1);
            t2 = BRep_Tool.Tolerance(v2);
            c1f = p1.Distance(Firstpoint) <= t1;
            c2f = p2.Distance(Firstpoint) <= t2;
            c1l = p1.Distance(Lastpoint) <= t1;
            c2l = p2.Distance(Lastpoint) <= t2;
            if (c1f || c2f || c1l || c2l)
            {
                eb = ec;
                if (c1f || c1l)
                {
                    v1OK = true;
                }
                if (c2f || c2l)
                {
                    v2OK = true;
                }
                if (c1f || c2f)
                {
                    FirstOK = true;
                }
                if (c1l || c2l)
                {
                    LastOK = true;
                }
                break;
            }
        }

        if (eb.IsNull())
        {
            falseside = false;
            return false;
        }
        NCollection_List<TopoDS_Shape> thelist = new();
        mySlface.Bind(CurrentFace, thelist);
        mySlface[CurrentFace].Append(eb);

        myListOfEdges.Clear();
        myListOfEdges.Append(eb);

        // two points are on the same face.
        if (LastOK && FirstOK)
        {
            return result;
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mapedges = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(mySbase, TopAbs_EDGE, TopAbs_FACE, mapedges);
        TopExp_Explorer ex = new();
        TopoDS_Edge FirstEdge = new();
        BRep_Builder BB = new();

        TopoDS_Vertex Vprevious;
        Point3D ptprev;
        double dp;

        while (!(LastOK && FirstOK))
        {
            if (v1OK)
            {
                Vprevious = v2;
                ptprev = p2;
            }
            else
            {
                Vprevious = v1;
                ptprev = p1;
            }

            // find edge connected to v1 or v2:
            for (ex.Init(CurrentFace, TopAbs_EDGE); ex.More(); ex.Next())
            {
                TopoDS_Edge rfe = TopoDS.Edge(ex.Current());

                BRepExtrema_ExtPC projF = new(Vprevious, rfe);

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
                        if (dist2min <= BRep_Tool.Tolerance(rfe) * BRep_Tool.Tolerance(rfe))
                        {
                            FirstEdge = rfe;
                            // If the edge is not perpendicular to the plane of the rib
                            // it is required to set Sliding(result) to false.
                            if (result)
                            {
                                result = false;
                                ve1 = TopExp.FirstVertex(rfe, true);
                                ve2 = TopExp.LastVertex(rfe, true);
                                BRepExtrema_ExtPF perp = new(ve1, fac);
                                if (perp.IsDone())
                                {
                                    Point3D pe1 = perp.Point(1);
                                    perp.Perform(ve2, fac);
                                    if (perp.IsDone())
                                    {
                                        Point3D pe2 = perp.Point(1);
                                        if (pe1.Distance(pe2) <= BRep_Tool.Tolerance(rfe))
                                        {
                                            result = true;
                                        }
                                    }
                                }
                            }
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
                v1 = TopExp.FirstVertex(edg1, true);
                v2 = TopExp.LastVertex(edg1, true);
                t1 = BRep_Tool.Tolerance(v1);
                t2 = BRep_Tool.Tolerance(v2);
                p1 = BRep_Tool.Pnt(v1);
                p2 = BRep_Tool.Pnt(v2);
                v1OK = p1.Distance(ptprev) <= t1;
                v2OK = p2.Distance(ptprev) <= t2;
                if (v1OK || v2OK)
                {
                    break;
                }
            }

            if (v1OK)
            {
                if (!FirstOK)
                {
                    dp = p2.Distance(Firstpoint);
                    if (dp <= 2 * t2)
                    {
                        FirstOK = true;
                        BB.UpdateVertex(v2, dp);
                    }
                }
                if (!LastOK)
                {
                    dp = p2.Distance(Lastpoint);
                    if (dp <= 2 * t2)
                    {
                        LastOK = true;
                        BB.UpdateVertex(v2, dp);
                    }
                }
            }
            else if (v2OK)
            {
                if (!FirstOK)
                {
                    dp = p1.Distance(Firstpoint);
                    if (dp <= 2 * t1)
                    {
                        FirstOK = true;
                        BB.UpdateVertex(v1, dp);
                    }
                }
                if (!LastOK)
                {
                    dp = p1.Distance(Lastpoint);
                    if (dp <= 2 * t1)
                    {
                        LastOK = true;
                        BB.UpdateVertex(v1, dp);
                    }
                }
            }
            else
            {
                // end by chaining the section
                return false;
            }
            NCollection_List<TopoDS_Shape> thelist1 = new();
            mySlface.Bind(CurrentFace, thelist1);
            mySlface[CurrentFace].Append(edg1);
            myListOfEdges.Append(edg1);
        }

        return result;
    }

    private static void MajMap(in TopoDS_Shape theB, LocOpe_LinearForm theP,
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
                theMap[theFShape].Append(exp.Current());
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
                theMap[theLShape].Append(exp.Current());
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
                                      LocOpe_LinearForm thePrism,
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
                    theMap.Bind(gfac.First(), fac);
                }
            }
        }
    }
}
