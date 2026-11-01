// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_RibSlot.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_RibSlot.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_RibSlot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.LocOpe_Operation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The root of the mechanical features (ribs and grooves) built from a contour: the profile computed from the contour
/// and the basis shape, the topological reconstruction of the result, and the history of the basis shape's faces.
/// </summary>
/// <remarks>
/// <para>A working object (decision 21): <see cref="BRepBuilderAPI_MakeShape.Shape"/>, the history and the lists
/// publish what they return (frozen); ported code reads the <c>Internal</c> forms.</para>
/// <para>The basis shape and the contour are read, never written: the gluing copies them (LocOpe_Gluer, decision 1013),
/// the Booleans are non-destructive; the same-parameter pass OCCT runs on the glued result works on its copy where it
/// holds the caller's shapes (decision 9501).</para>
/// </remarks>
public abstract class BRepFeat_RibSlot : BRepBuilderAPI_MakeShape
{
    protected gp_Pnt myFirstPnt = new();
    protected gp_Pnt myLastPnt = new();
    protected bool myFuse;
    protected bool mySliding;
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap = new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myLFMap = new(TopTools_ShapeMapHasher.Instance);
    protected TopoDS_Shape myFShape = new();
    protected TopoDS_Shape myLShape = new();
    protected BRepFeat_PerfSelection myPerfSelection;
    protected TopoDS_Wire myWire = new();
    protected TopoDS_Shape mySbase = new();
    protected TopoDS_Face mySkface = new();
    protected TopoDS_Face myPbase = new();
    protected TopoDS_Shape myGShape = new();
    protected TopoDS_Shape mySUntil = new();
    protected NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myGluedF = new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_List<TopoDS_Shape> myNewEdges = new();
    protected NCollection_List<TopoDS_Shape> myTgtEdges = new();
    protected NCollection_List<TopoDS_Shape> myFacesForDraft = new();
    protected BRepFeat_StatusError myStatusError;

    // OCCT's Modified and Generated return a function-static list
    private readonly NCollection_List<TopoDS_Shape> myList = new();

    protected BRepFeat_RibSlot()
    {
        myFuse = false;
        mySliding = false;
        myStatusError = BRepFeat_OK;
    }

    /// <summary>General perform method: the topological reconstruction of the result.</summary>
    protected void LFPerform()
    {
        if (mySbase.IsNull() || myPbase.IsNull() || mySkface.IsNull() || myGShape.IsNull() || myLFMap.IsEmpty())
        {
            myStatusError = BRepFeat_NotInitialized;
            NotDone();
            return;
        }

        TopExp_Explorer exp = new(), exp2 = new();
        int theOpe = 2;

        if (!myGluedF.IsEmpty())
        {
            theOpe = 1;
        }

        // Hope that there is just a solid in the result
        if (!mySUntil.IsNull())
        {
            for (exp2.Init(mySUntil, TopAbs_FACE); exp2.More(); exp2.Next())
            {
                TopoDS_Shape funtil = exp2.Current();
                for (exp.Init(mySbase, TopAbs_FACE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(funtil))
                    {
                        break;
                    }
                }
                if (!exp.More())
                {
                    break;
                }
            }
        }

        LocOpe_Gluer theGlue = new();

        // case of gluing
        if (theOpe == 1)
        {
            bool Collage = true;
            LocOpe_FindEdges theFE = new();
            theGlue.Init(mySbase, myGShape);
            foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> itm in myGluedF)
            {
                TopoDS_Face glface = TopoDS.Face(itm.Key);
                TopoDS_Face fac = TopoDS.Face(myGluedF[glface]);
                for (exp.Init(myGShape, TopAbs_FACE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(glface))
                    {
                        break;
                    }
                }
                if (exp.More())
                {
                    Collage = BRepFeat.IsInside(glface, fac);
                    if (!Collage)
                    {
                        theOpe = 2;
                        break;
                    }
                    else
                    {
                        theGlue.Bind(glface, fac);
                        theFE.Set(glface, fac);
                        for (theFE.InitIterator(); theFE.More(); theFE.Next())
                        {
                            theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                        }
                    }
                }
            }

            LocOpe_Operation ope = theGlue.OpeType();
            if (ope == LocOpe_INVALID || (myFuse && ope != LocOpe_FUSE) || (!myFuse && ope != LocOpe_CUT) || (!Collage))
            {
                theOpe = 2;
            }
        }

        // gluing is always applicable

        if (theOpe == 1)
        {
            theGlue.Perform();
            if (theGlue.IsDone())
            {
                UpdateDescendants(theGlue);
                myNewEdges = new NCollection_List<TopoDS_Shape>(theGlue.InternalEdges());
                myTgtEdges = new NCollection_List<TopoDS_Shape>(theGlue.InternalTgtEdges());
                //
                Done();
                myShape = theGlue.InternalResultingShape();
                SameParameterOfTheGluedResult();
            }
            else
            {
                theOpe = 2;
            }
        }

        // case without gluing
        if (theOpe == 2)
        {
            BRepFeat_Builder theBuilder = new();
            NCollection_List<TopoDS_Shape> partsoftool = new();
            BRepClass3d_SolidClassifier oussa = new();
            bool bFlag;
            NCollection_List<TopoDS_Shape>.Iterator aIt = new();

            bFlag = myPerfSelection != BRepFeat_NoSelection;
            //
            theBuilder.Init(mySbase, myGShape);
            theBuilder.SetOperation(myFuse ? 1 : 0, bFlag);
            //
            theBuilder.Perform();
            if (bFlag)
            {
                theBuilder.PartsOfTool(partsoftool);
                aIt.Initialize(partsoftool);
                if (aIt.More() && myPerfSelection != BRepFeat_NoSelection)
                {
                    double toler = BRep_Tool.Tolerance(myPbase) * 2;
                    //
                    for (; aIt.More(); aIt.Next())
                    {
                        oussa.Load(aIt.Value());
                        oussa.Perform(myFirstPnt, toler);
                        TopAbs_State sp1 = oussa.State();
                        oussa.Perform(myLastPnt, toler);
                        TopAbs_State sp2 = oussa.State();
                        if (sp1 != TopAbs_OUT && sp2 != TopAbs_OUT)
                        {
                            TopoDS_Shape S = aIt.Value();
                            theBuilder.KeepPart(S);
                        }
                    }
                }
                //
                theBuilder.PerformResult();
                myShape = theBuilder.InternalShape();
            }
            else
            {
                myShape = theBuilder.InternalShape();
            }
            Done();
        }
    }

    // OCCT runs BRepLib::SameParameter(myShape, 1.e-7, true) on the glued result in place. The result shares the
    // caller's frozen sub-shapes where the gluing left them unchanged: the pass runs on its copy for write, the
    // sub-shapes it leaves unchanged go back to the caller's, and the history follows (decision 9501).
    private void SameParameterOfTheGluedResult()
    {
        BRepTools_CopyAtEntry aCopy = new(myShape);
        BRepLib.SameParameter(aCopy.Shape(), 1.0e-7, true);
        if (!aCopy.IsCopied())
        {
            return;
        }
        myShape = aCopy.Finish(aCopy.Shape());
        foreach (TopoDS_Shape aKey in myMap.Keys.ToList())
        {
            myMap[aKey] = Final(aCopy, myMap[aKey]);
        }
        myNewEdges = Final(aCopy, myNewEdges);
        myTgtEdges = Final(aCopy, myTgtEdges);
    }

    private static NCollection_List<TopoDS_Shape> Final(BRepTools_CopyAtEntry theCopy, NCollection_List<TopoDS_Shape> theList)
    {
        NCollection_List<TopoDS_Shape> aList = new();
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            aList.Append(theCopy.Final(theCopy.Copied(anIt.Value())));
        }
        return aList;
    }

    /// <summary>True if the face <paramref name="F"/> has no descendant in the result.</summary>
    /// <exception cref="Standard_NoSuchObject"><paramref name="F"/> isn't a shape of the history.</exception>
    public override bool IsDeleted(in TopoDS_Shape F)
    {
        return myMap.Find(F).IsEmpty();
    }

    /// <summary>The faces generated from the face <paramref name="F"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape F)
    {
        if (myMap.IsBound(F))
        {
            myList.Clear();
            NCollection_List<TopoDS_Shape>.Iterator ite = new(myMap.Find(F));
            for (; ite.More(); ite.Next())
            {
                TopoDS_Shape sh = ite.Value();
                if (!sh.IsSame(F))
                {
                    myList.Append(sh);
                }
            }
            return myList;
        }
        return myGenerated; // empty list
    }

    /// <summary>The faces created from the shape <paramref name="S"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S)
    {
        if (S.ShapeType() != TopAbs_FACE)
        {
            myGenerated.Clear();
            if (myLFMap.IsEmpty() || !myLFMap.IsBound(S))
            {
                if (myMap.IsBound(S))
                { // check if filter on face or not
                    myList.Clear();
                    NCollection_List<TopoDS_Shape>.Iterator ite = new(myMap.Find(S));
                    for (; ite.More(); ite.Next())
                    {
                        TopoDS_Shape sh = ite.Value();
                        if (!sh.IsSame(S))
                        {
                            myList.Append(sh);
                        }
                    }
                    return myList;
                }
                else
                {
                    return myGenerated;
                }
            }
            else
            {
                myGenerated.Clear();
                NCollection_List<TopoDS_Shape>.Iterator it = new(myLFMap.Find(S));
                myList.Clear();
                for (; it.More(); it.Next())
                {
                    if (myMap.IsBound(it.Value()))
                    {
                        NCollection_List<TopoDS_Shape>.Iterator it1 = new(myMap.Find(it.Value()));
                        for (; it1.More(); it1.Next())
                        {
                            TopoDS_Shape sh = it1.Value();
                            if (!sh.IsSame(S))
                            {
                                myList.Append(sh);
                            }
                        }
                    }
                }
                return myList;
            }
        }
        else
        {
            return myGenerated;
        }
    }

    // The history's lists hold sub-shapes of the result: publishing them publishes the result (decision 414).
    private protected override void BeforePublishingLists()
    {
        myShape.TShape()?.FreezeGraph();
    }

    protected void UpdateDescendants(LocOpe_Gluer G)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new(), it2 = new();

        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itdm in myMap)
        {
            TopoDS_Shape orig = itdm.Key;
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 9503)
            NCollection_IndexedMap<TopoDS_Shape> newdsc = new(TopTools_ShapeMapHasher.Instance);
            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                TopoDS_Face fdsc = TopoDS.Face(it.Value());
                for (it2.Initialize(G.InternalDescendantFaces(fdsc)); it2.More(); it2.Next())
                {
                    newdsc.Add(it2.Value());
                }
            }
            myMap.ChangeFind(orig).Clear();
            foreach (TopoDS_Shape itm in newdsc)
            {
                myMap.ChangeFind(orig).Append(itm);
            }
        }
    }

    /// <summary>The shapes created at the bottom of the created form; it may be empty (frozen).</summary>
    public IBentoList<TopoDS_Shape> FirstShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalFirstShape());
    }

    /// <summary><see cref="FirstShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalFirstShape()
    {
        if (!myFShape.IsNull())
        {
            return myMap.Find(myFShape);
        }
        return myGenerated; // empty list
    }

    /// <summary>The shapes created at the top of the created form; it may be empty (frozen).</summary>
    public IBentoList<TopoDS_Shape> LastShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalLastShape());
    }

    /// <summary><see cref="LastShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalLastShape()
    {
        if (!myLShape.IsNull())
        {
            return myMap.Find(myLShape);
        }
        return myGenerated; // empty list
    }

    /// <summary>The faces on which drafts can be made afterwards (frozen).</summary>
    public IBentoList<TopoDS_Shape> FacesForDraft()
    {
        BeforePublishingLists();
        return PublishedList(myFacesForDraft);
    }

    /// <summary><see cref="FacesForDraft"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalFacesForDraft() => myFacesForDraft;

    /// <summary>The limiting and glueing edges generated by the feature (frozen).</summary>
    public IBentoList<TopoDS_Shape> NewEdges()
    {
        BeforePublishingLists();
        return PublishedList(myNewEdges);
    }

    /// <summary><see cref="NewEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalNewEdges() => myNewEdges;

    /// <summary>The tangent edges among <see cref="NewEdges"/> (frozen).</summary>
    public IBentoList<TopoDS_Shape> TgtEdges()
    {
        BeforePublishingLists();
        return PublishedList(myTgtEdges);
    }

    /// <summary><see cref="TgtEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalTgtEdges() => myTgtEdges;

    /// <summary>The status of the last construction.</summary>
    public BRepFeat_StatusError CurrentStatusError() => myStatusError;

    /// <summary>A proofing point somewhat inside the material side of the edge <paramref name="e"/>.</summary>
    protected gp_Pnt CheckPoint(in TopoDS_Edge e, double bnd, Geom_Plane Pln)
    {
        // Vector product : normal to plane X direction Wire
        // -> gives the material side
        // Proofing point somewhat inside the material side
        Geom_Curve cc = BRep_Tool.Curve(e, out double f, out double l)!;

        double par = (f + l) / 2.0;
        cc.D1(par, out gp_Pnt pp, out gp_Vec tgt);

        if (e.Orientation() == TopAbs_REVERSED)
        {
            tgt.Reverse();
        }

        gp_Vec D = -tgt.Crossed(new gp_Vec(Pln.Pln().Position().Direction())) / 10.0;
        pp.Translate(D);

        return pp;
    }

    /// <summary>The normal of the face <paramref name="F"/> at <paramref name="P"/>, reversed for a forward face.</summary>
    protected gp_Dir Normal(in TopoDS_Face F, in gp_Pnt P)
    {
        double U, V;
        BRepAdaptor_Surface AS = new(F, true);

        switch (AS.GetType())
        {
            case GeomAbs_Plane:
                ElSLib.Parameters(AS.Plane(), P, out U, out V);
                break;

            case GeomAbs_Cylinder:
                ElSLib.Parameters(AS.Cylinder(), P, out U, out V);
                break;

            case GeomAbs_Cone:
                ElSLib.Parameters(AS.Cone(), P, out U, out V);
                break;

            case GeomAbs_Torus:
                ElSLib.Parameters(AS.Torus(), P, out U, out V);
                break;

            default:
            {
                return new gp_Dir(gp_Dir.D.X);
            }
        }

        AS.D1(U, V, out gp_Pnt pt, out gp_Vec D1U, out gp_Vec D1V);
        gp_Dir N = new();
        CSLib.Normal(D1U, D1V, Precision.Confusion(), out CSLib_DerivativeStatus St, ref N);
        if (F.Orientation() == TopAbs_FORWARD)
        {
            N.Reverse();
        }
        return N;
    }

    /// <summary>The parameter of the point <paramref name="P"/> on the conic <paramref name="C"/> (0 for other curves).</summary>
    public static double IntPar(Geom_Curve? C, in gp_Pnt P)
    {
        if (C is null)
        {
            return 0.0;
        }

        GeomAdaptor_Curve AC = new(C);
        double U;

        switch (AC.GetType())
        {
            case GeomAbs_Line:
                U = ElCLib.Parameter(AC.Line(), P);
                break;

            case GeomAbs_Circle:
                U = ElCLib.Parameter(AC.Circle(), P);
                break;

            case GeomAbs_Ellipse:
                U = ElCLib.Parameter(AC.Ellipse(), P);
                break;

            case GeomAbs_Hyperbola:
                U = ElCLib.Parameter(AC.Hyperbola(), P);
                break;

            case GeomAbs_Parabola:
                U = ElCLib.Parameter(AC.Parabola(), P);
                break;

            default:
                U = 0.0;
                break;
        }

        return U;
    }

    /// <summary>Extends the edge <paramref name="e"/> at its start (or its end) by a tenth of <paramref name="bnd"/>.</summary>
    protected void EdgeExtention(ref TopoDS_Edge e, double bnd, bool FirstLast)
    {
        Geom_Curve cu = BRep_Tool.Curve(e, out double f, out double l)!;
        Geom_BoundedCurve C = new Geom_TrimmedCurve(cu, f, l);

        TopoDS_Edge E;

        if (cu.GetType() == typeof(Geom_Line) || cu.GetType() == typeof(Geom_Circle)
            || cu.GetType() == typeof(Geom_Ellipse) || cu.GetType() == typeof(Geom_Hyperbola)
            || cu.GetType() == typeof(Geom_Parabola))
        {
            if (FirstLast)
            {
                BRepLib_MakeEdge Edg = new(cu, f - bnd / 10.0, l);
                E = TopoDS.Edge(Edg.InternalShape());
            }
            else
            {
                BRepLib_MakeEdge Edg = new(cu, f, l + bnd / 10.0);
                E = TopoDS.Edge(Edg.InternalShape());
            }
        }
        else
        {
            Geom_Line ln;
            gp_Pnt Pt;
            if (FirstLast)
            {
                C.D1(f, out gp_Pnt pnt, out gp_Vec vct);
                ln = new Geom_Line(pnt, new gp_Dir(-vct));
                ln.D0(bnd / 1000.0, out Pt);
                GeomLib.ExtendCurveToPoint(ref C, Pt, (int)GeomAbs_G1, false);
                BRepLib_MakeEdge Edg = new(C, Pt, BRep_Tool.Pnt(TopExp.LastVertex(e, true)));
                E = TopoDS.Edge(Edg.InternalShape());
            }
            else
            {
                C.D1(l, out gp_Pnt pnt, out gp_Vec vct);
                ln = new Geom_Line(pnt, new gp_Dir(vct));
                ln.D0(bnd / 1000.0, out Pt);
                GeomLib.ExtendCurveToPoint(ref C, Pt, (int)GeomAbs_G1, true);
                BRepLib_MakeEdge Edg = new(C, BRep_Tool.Pnt(TopExp.FirstVertex(e, true)), Pt);
                E = TopoDS.Edge(Edg.InternalShape());
            }
        }
        e = E;
    }

    /// <summary>
    /// The face of <paramref name="faces"/> first met by lines through the point of <paramref name="cc"/> at
    /// <paramref name="par"/>, turned around the normal of <paramref name="Pln"/>.
    /// </summary>
    public static TopoDS_Face ChoiceOfFaces(NCollection_List<TopoDS_Shape> faces, Geom_Curve cc, double par, double bnd,
                                            Geom_Plane Pln)
    {
        TopoDS_Face FFF = new();

        cc.D1(par, out gp_Pnt pp, out gp_Vec tgt);

        Geom_Line l1 = new(pp, new gp_Dir(tgt));

        NCollection_Sequence<Geom_Curve?> scur = new();
        int Counter = 0;

        gp_Ax1 Axe = new(pp, Pln.Position().Direction());
        for (int i = 1; i <= 8; i++)
        {
            Geom_Curve L = (Geom_Curve)l1.Rotated(Axe, i * M_PI / 9.0);
            scur.Append(L);
            Counter++;
        }

        NCollection_List<TopoDS_Shape>.Iterator it = new();
        it.Initialize(faces);
        double Par = RealLast();
        for (; it.More(); it.Next())
        {
            TopoDS_Face f = TopoDS.Face(it.Value());
            LocOpe_CSIntersector ASI = new(f);
            ASI.Perform(scur);
            if (!ASI.IsDone())
            {
                continue;
            }
            for (int jj = 1; jj <= Counter; jj++)
            {
                if (ASI.NbPoints(jj) >= 1)
                {
                    double app = ASI.Point(jj, 1).Parameter();
                    if (app >= 0 && app < Par)
                    {
                        Par = app;
                        FFF = f;
                    }
                }
            }
        }

        return FFF;
    }

    /// <summary>The bound of the boxes of the basis shape and the until shape, and the box's corners pushed outwards.</summary>
    protected double HeightMax(in TopoDS_Shape theSbase, in TopoDS_Shape theSUntil, ref gp_Pnt p1, ref gp_Pnt p2)
    {
        Bnd_Box Box = new();
        BRepBndLib.Add(theSbase, ref Box);
        if (!theSUntil.IsNull())
        {
            BRepBndLib.Add(theSUntil, ref Box);
        }
        double[] c = new double[6];
        double bnd;
        Box.Get(out c[0], out c[2], out c[4], out c[1], out c[3], out c[5]);
        bnd = c[0];
        for (int i = 0; i < 6; i++)
        {
            if (c[i] > bnd)
            {
                bnd = c[i];
            }
        }
        p1.SetCoord(c[0] - 2.0 * bnd, c[1] - 2.0 * bnd, c[2] - 2.0 * bnd);
        p2.SetCoord(c[3] + 2.0 * bnd, c[4] + 2.0 * bnd, c[5] + 2.0 * bnd);
        return bnd;
    }

    /// <summary>The faces, edges and vertices of the basis shape where the contour starts and ends.</summary>
    protected bool ExtremeFaces(bool RevolRib, double bnd, Geom_Plane Pln, ref TopoDS_Edge FirstEdge,
                                ref TopoDS_Edge LastEdge, ref TopoDS_Face FirstFace, ref TopoDS_Face LastFace,
                                ref TopoDS_Vertex FirstVertex, ref TopoDS_Vertex LastVertex, ref bool OnFirstFace,
                                ref bool OnLastFace, ref bool PtOnFirstEdge, ref bool PtOnLastEdge,
                                ref TopoDS_Edge OnFirstEdge, ref TopoDS_Edge OnLastEdge)
    {
        bool Data = true;
        FirstFace.Nullify();
        LastFace.Nullify();
        FirstEdge.Nullify();
        LastEdge.Nullify();
        PtOnFirstEdge = false;
        PtOnLastEdge = false;
        OnFirstEdge.Nullify();
        OnLastEdge.Nullify();

        BRepIntCurveSurface_Inter inter = new();
        BRep_Builder B = new();
        TopExp_Explorer ex1 = new();

        bool FirstOK = false, LastOK = false;

        int NumberOfEdges = 0;
        TopExp_Explorer exp = new(myWire, TopAbs_EDGE);

        for (; exp.More(); exp.Next())
        {
            NumberOfEdges++;
        }

        // ---the wire includes only one edge
        if (NumberOfEdges == 1)
        {
            exp.ReInit();
            gp_Pnt firstpoint = new(), lastpoint = new();

            // Points limit the unique edge
            TopoDS_Edge E = TopoDS.Edge(exp.Current());
            Geom_Curve cc = BRep_Tool.Curve(E, out double f, out double l)!;
            gp_Pnt p1 = BRep_Tool.Pnt(TopExp.FirstVertex(E, true));
            gp_Pnt p2 = BRep_Tool.Pnt(TopExp.LastVertex(E, true));

            double FirstPar = f;
            double LastPar = l;

            // ---Find if 2 points limiting the unique edge of the wire
            //    are on an edge or a vertex of the base shape
            bool PtOnFirstVertex = false;
            bool PtOnLastVertex = false;
            TopoDS_Vertex OnFirstVertex = new(), OnLastVertex = new();
            PtOnEdgeVertex(RevolRib, mySbase, p1, FirstVertex, LastVertex, ref PtOnFirstEdge, ref OnFirstEdge,
                           ref PtOnFirstVertex, ref OnFirstVertex);
            PtOnEdgeVertex(RevolRib, mySbase, p2, FirstVertex, LastVertex, ref PtOnLastEdge, ref OnLastEdge,
                           ref PtOnLastVertex, ref OnLastVertex);

            NCollection_Map<TopoDS_Shape> Map = new(TopTools_ShapeMapHasher.Instance);

            if (PtOnFirstEdge)
            {
                if (!PtOnFirstVertex)
                {
                    // Find FirstFace : face of the base shape containing OnFirstEdge
                    //                  meeting ChoiceOfFaces
                    TopExp_Explorer ex4 = new(), ex5 = new();
                    ex4.Init(mySbase, TopAbs_FACE);
                    NCollection_List<TopoDS_Shape> faces = new();
                    faces.Clear();
                    Map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        TopoDS_Face fx = TopoDS.Face(ex4.Current());
                        if (!Map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), TopAbs_EDGE);
                        for (; ex5.More(); ex5.Next())
                        {
                            TopoDS_Edge ee = TopoDS.Edge(ex5.Current());
                            if (ee.IsSame(OnFirstEdge))
                            {
                                faces.Append(fx);
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        TopoDS_Face FFF = ChoiceOfFaces(faces, cc, FirstPar + bnd / 50.0, bnd / 50.0, Pln);
                        if (!FFF.IsNull())
                        {
                            FirstFace = FFF;
                        }
                    }
                }
                else if (PtOnFirstVertex)
                {
                    // Find FirstFace : face of the base shape containing OnFirstVertex
                    //                  meeting ChoiceOfFaces
                    TopExp_Explorer ex4 = new(), ex5 = new();
                    ex4.Init(mySbase, TopAbs_FACE);
                    NCollection_List<TopoDS_Shape> faces = new();
                    faces.Clear();
                    Map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        TopoDS_Face fx = TopoDS.Face(ex4.Current());
                        if (!Map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), TopAbs_VERTEX);
                        for (; ex5.More(); ex5.Next())
                        {
                            TopoDS_Vertex vv = TopoDS.Vertex(ex5.Current());
                            if (vv.IsSame(OnFirstVertex))
                            {
                                faces.Append(fx);
                                break;
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        TopoDS_Face FFF = ChoiceOfFaces(faces, cc, FirstPar + bnd / 50.0, bnd / 50.0, Pln);
                        if (!FFF.IsNull())
                        {
                            FirstFace = FFF;
                        }
                    }
                }
                FirstEdge = E;
                BRepLib_MakeVertex v = new(p1);
                FirstVertex = v.InternalVertex();
                OnFirstFace = true;
            }

            if (PtOnLastEdge)
            {
                if (!PtOnLastVertex)
                {
                    // Find LastFace : face of the base shape containing OnLastEdge
                    //                 meeting ChoiceOfFaces
                    TopExp_Explorer ex4 = new(), ex5 = new();
                    ex4.Init(mySbase, TopAbs_FACE);
                    NCollection_List<TopoDS_Shape> faces = new();
                    faces.Clear();
                    Map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        TopoDS_Face fx = TopoDS.Face(ex4.Current());
                        if (!Map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), TopAbs_EDGE);
                        for (; ex5.More(); ex5.Next())
                        {
                            TopoDS_Edge ee = TopoDS.Edge(ex5.Current());
                            if (ee.IsSame(OnLastEdge))
                            {
                                faces.Append(fx);
                                break;
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        TopoDS_Face FFF = ChoiceOfFaces(faces, cc, LastPar - bnd / 50.0, bnd / 50.0, Pln);
                        if (!FFF.IsNull())
                        {
                            LastFace = FFF;
                        }
                    }
                }
                else if (PtOnLastEdge && PtOnLastVertex)
                {
                    // Find LastFace : face of the base shape containing OnLastVertex
                    //                 meeting ChoiceOfFaces
                    TopExp_Explorer ex4 = new(), ex5 = new();
                    ex4.Init(mySbase, TopAbs_FACE);
                    NCollection_List<TopoDS_Shape> faces = new();
                    faces.Clear();
                    Map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        TopoDS_Face fx = TopoDS.Face(ex4.Current());
                        if (!Map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), TopAbs_VERTEX);
                        for (; ex5.More(); ex5.Next())
                        {
                            TopoDS_Vertex vv = TopoDS.Vertex(ex5.Current());
                            if (vv.IsSame(OnLastVertex))
                            {
                                faces.Append(fx);
                                break;
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        TopoDS_Face FFF = ChoiceOfFaces(faces, cc, LastPar - bnd / 50.0, bnd / 50.0, Pln);
                        if (!FFF.IsNull())
                        {
                            LastFace = FFF;
                        }
                    }
                }
                LastEdge = E;
                BRepLib_MakeVertex v = new(p2);
                LastVertex = v.InternalVertex();
                OnLastFace = true;
            }

            if (!FirstFace.IsNull() && !LastFace.IsNull())
            {
                return true;
            }

            //--- FirstFace or LastFace was not found
            LocOpe_CSIntersector ASI = new(mySbase);
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(cc);
            ASI.Perform(scur);
            double lastpar, firstpar;
            if (ASI.IsDone() && ASI.NbPoints(1) >= 2)
            {
                lastpar = ASI.Point(1, ASI.NbPoints(1)).Parameter();
                int lastindex = ASI.NbPoints(1);
                if (lastpar > l)
                {
                    for (int jj = ASI.NbPoints(1) - 1; jj >= 1; jj--)
                    {
                        double par = ASI.Point(1, jj).Parameter();
                        if (par <= l)
                        {
                            lastpar = par;
                            lastindex = jj;
                            break;
                        }
                    }
                }
                int firstindex = lastindex - 1;
                firstpar = ASI.Point(1, firstindex).Parameter();

                if (FirstFace.IsNull())
                {
                    FirstFace = ASI.Point(1, firstindex).Face();
                    cc.D0(firstpar, out firstpoint);
                    BRepLib_MakeVertex v1 = new(firstpoint);
                    FirstVertex = TopoDS.Vertex(v1.InternalShape());
                    FirstEdge = E;
                }

                if (LastFace.IsNull())
                {
                    LastFace = ASI.Point(1, lastindex).Face();
                    cc.D0(lastpar, out lastpoint);
                    BRepLib_MakeVertex v2 = new(lastpoint);
                    LastVertex = TopoDS.Vertex(v2.InternalShape());
                    LastEdge = E;
                }
            }
            else
            {
                Data = false;
                return Data;
            }

            if (!OnFirstFace)
            {
                OnFirstFace = p1.Distance(firstpoint) <= Precision.Confusion();
            }
            if (!OnLastFace)
            {
                OnLastFace = p2.Distance(lastpoint) <= Precision.Confusion();
            }

            Data = !(FirstFace.IsNull() || LastFace.IsNull());
            return Data;
        }

        // ---The wire consists of several edges
        else
        {
            BRepTools_WireExplorer ex = new(myWire);
            for (; ex.More(); ex.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(ex.Current());
                Geom_Curve Cur = BRep_Tool.Curve(E, out double f, out double l)!;
                f = f - bnd / 10000;
                l = l + bnd / 10000;
                Geom_TrimmedCurve curve;
                curve = new Geom_TrimmedCurve(Cur, f, l, true);
                gp_Pnt P2 = BRep_Tool.Pnt(TopExp.LastVertex(E, true));
                ex1.Init(mySbase, TopAbs_FACE);
                TopoDS_Vertex theVertex = new();
                TopoDS_Edge theEdge = new();
                TopoDS_Face theFace = new();
                bool PtOnEdge = false;
                bool PtOnVertex = false;
                TopoDS_Edge OnEdge = new();
                TopoDS_Vertex OnVertex = new();
                double intpar;
                for (; ex1.More(); ex1.Next())
                {
                    TopoDS_Face aCurFace = TopoDS.Face(ex1.Current());
                    GeomAdaptor_Curve aGAC = new(curve);
                    inter.Init(aCurFace, aGAC, BRep_Tool.Tolerance(aCurFace));
                    if (!inter.More())
                    {
                        continue;
                    }
                    for (; inter.More(); inter.Next())
                    {
                        gp_Pnt thePoint = inter.Pnt();
                        if (!FirstVertex.IsNull())
                        {
                            gp_Pnt point = BRep_Tool.Pnt(FirstVertex);
                            if (point.Distance(thePoint) <= BRep_Tool.Tolerance(aCurFace))
                            {
                                continue;
                            }
                        }
                        intpar = IntPar(curve, thePoint);
                        theEdge = E;
                        theFace = aCurFace;
                        B.MakeVertex(ref theVertex, thePoint, Precision.Confusion());
                        if (!FirstOK)
                        {
                            if (thePoint.Distance(P2) <= Precision.Confusion())
                            {
                                continue;
                            }
                        }

                        // ---Find thepoint on an edge or a vertex of face f
                        PtOnEdgeVertex(RevolRib, aCurFace, thePoint, FirstVertex, LastVertex, ref PtOnEdge, ref OnEdge,
                                       ref PtOnVertex, ref OnVertex);

                        if (FirstEdge.IsNull() && !theEdge.IsNull() && !theFace.IsNull() && !theVertex.IsNull())
                        {
                            FirstEdge = theEdge;
                            FirstFace = theFace;
                            FirstVertex = theVertex;
                            PtOnFirstEdge = PtOnEdge;
                            OnFirstEdge = OnEdge;
                            theEdge.Nullify();
                            theFace.Nullify();
                            theVertex.Nullify();
                            if (PtOnEdge && !PtOnVertex)
                            {
                                NCollection_List<TopoDS_Shape> faces = new();
                                faces.Clear();
                                faces.Append(FirstFace);
                                TopExp_Explorer ex2 = new();
                                ex2.Init(mySbase, TopAbs_FACE);
                                for (; ex2.More(); ex2.Next())
                                {
                                    TopoDS_Face fx = TopoDS.Face(ex2.Current());
                                    TopExp_Explorer ex3 = new();
                                    ex3.Init(fx, TopAbs_EDGE);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        TopoDS_Edge e = TopoDS.Edge(ex3.Current());
                                        if (e.IsSame(OnEdge) && !fx.IsSame(FirstFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                TopoDS_Face FFF = ChoiceOfFaces(faces, curve, intpar + bnd / 10.0, bnd / 10.0, Pln);
                                if (!FFF.IsNull())
                                {
                                    FirstFace = FFF;
                                }
                            }
                            else if (PtOnEdge && PtOnVertex)
                            {
                                NCollection_List<TopoDS_Shape> faces = new();
                                faces.Clear();
                                faces.Append(FirstFace);
                                TopExp_Explorer ex2 = new();
                                ex2.Init(mySbase, TopAbs_FACE);
                                for (; ex2.More(); ex2.Next())
                                {
                                    TopoDS_Face fx = TopoDS.Face(ex2.Current());
                                    TopExp_Explorer ex3 = new();
                                    ex3.Init(fx, TopAbs_VERTEX);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        TopoDS_Vertex v = TopoDS.Vertex(ex3.Current());
                                        if (v.IsSame(OnVertex) && !fx.IsSame(FirstFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                TopoDS_Face FFF = ChoiceOfFaces(faces, curve, intpar + bnd / 10.0, bnd / 10.0, Pln);
                                if (!FFF.IsNull())
                                {
                                    FirstFace = FFF;
                                }
                            }
                            if (!FirstEdge.IsNull() && !FirstFace.IsNull() && !FirstVertex.IsNull())
                            {
                                FirstOK = true;
                            }
                        }
                        if (LastEdge.IsNull() && !theEdge.IsNull() && !theFace.IsNull() && !theVertex.IsNull()
                            && !FirstEdge.IsNull())
                        {
                            LastEdge = theEdge;
                            LastFace = theFace;
                            LastVertex = theVertex;
                            PtOnLastEdge = PtOnEdge;
                            OnLastEdge = OnEdge;
                            if (PtOnEdge && !PtOnVertex)
                            {
                                NCollection_List<TopoDS_Shape> faces = new();
                                faces.Clear();
                                faces.Append(LastFace);
                                TopExp_Explorer ex2 = new();
                                ex2.Init(mySbase, TopAbs_FACE);
                                for (; ex2.More(); ex2.Next())
                                {
                                    TopoDS_Face fx = TopoDS.Face(ex2.Current());
                                    TopExp_Explorer ex3 = new();
                                    ex3.Init(fx, TopAbs_EDGE);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        TopoDS_Edge e = TopoDS.Edge(ex3.Current());
                                        if (e.IsSame(OnEdge) && !fx.IsSame(LastFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                TopoDS_Face FFF = ChoiceOfFaces(faces, curve, intpar - bnd / 10.0, bnd / 10.0, Pln);
                                if (!FFF.IsNull())
                                {
                                    LastFace = FFF;
                                }
                            }
                            else if (PtOnEdge && PtOnVertex)
                            {
                                NCollection_List<TopoDS_Shape> faces = new();
                                faces.Clear();
                                faces.Append(LastFace);
                                TopExp_Explorer ex2 = new();
                                ex2.Init(mySbase, TopAbs_FACE);
                                for (; ex2.More(); ex2.Next())
                                {
                                    TopoDS_Face fx = TopoDS.Face(ex2.Current());
                                    TopExp_Explorer ex3 = new();
                                    ex3.Init(fx, TopAbs_VERTEX);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        TopoDS_Vertex v = TopoDS.Vertex(ex3.Current());
                                        if (v.IsSame(OnVertex) && !fx.IsSame(LastFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                TopoDS_Face FFF = ChoiceOfFaces(faces, curve, intpar - bnd / 10.0, bnd / 10.0, Pln);
                                if (!FFF.IsNull())
                                {
                                    LastFace = FFF;
                                }
                            }
                            if (!LastEdge.IsNull() && !LastFace.IsNull() && !LastVertex.IsNull())
                            {
                                LastOK = true;
                            }
                            break;
                        }
                    }
                }
            }

            if (FirstOK && LastOK)
            {
                Data = true;
                gp_Pnt PP1 = BRep_Tool.Pnt(TopExp.FirstVertex(FirstEdge, true));
                gp_Pnt PP2 = BRep_Tool.Pnt(TopExp.LastVertex(LastEdge, true));
                gp_Pnt p1 = BRep_Tool.Pnt(FirstVertex);
                gp_Pnt p2 = BRep_Tool.Pnt(LastVertex);
                if (p1.Distance(PP1) <= BRep_Tool.Tolerance(FirstFace))
                {
                    OnFirstFace = true;
                }
                if (p2.Distance(PP2) <= BRep_Tool.Tolerance(LastFace))
                {
                    OnLastFace = true;
                }
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    /// <summary>Finds whether the point is on an edge or a vertex of the shape.</summary>
    protected void PtOnEdgeVertex(bool RevolRib, in TopoDS_Shape shape, in gp_Pnt point, in TopoDS_Vertex FirstVertex,
                                  in TopoDS_Vertex LastVertex, ref bool PtOnEdge, ref TopoDS_Edge OnEdge,
                                  ref bool PtOnVertex, ref TopoDS_Vertex OnVertex)
    {
        bool TestOK;

        TopExp_Explorer EXP = new();
        EXP.Init(shape, TopAbs_EDGE);
        NCollection_Map<TopoDS_Shape> Map = new(TopTools_ShapeMapHasher.Instance);
        for (; EXP.More(); EXP.Next())
        {
            TopoDS_Edge e = TopoDS.Edge(EXP.Current());
            if (!Map.Add(e))
            {
                continue;
            }
            if (!RevolRib)
            {
                if (BRep_Tool.Degenerated(e))
                {
                    continue;
                }
            }
            Geom_Curve ccc = BRep_Tool.Curve(e, out double fff, out double lll)!;
            if (!RevolRib)
            {
                ccc = new Geom_TrimmedCurve(ccc, fff, lll);
            }
            GeomAPI_ProjectPointOnCurve proj = new(point, ccc);
            TestOK = false;
            if (!RevolRib)
            {
                if (proj.NbPoints() == 1)
                {
                    TestOK = true;
                }
            }
            else
            {
                if (proj.NbPoints() >= 1)
                {
                    TestOK = true;
                }
            }
            if (TestOK && proj.Distance(1) <= BRep_Tool.Tolerance(e))
            {
                PtOnEdge = true;
                OnEdge = e;
                TopoDS_Vertex ev1 = TopExp.FirstVertex(e, true);
                TopoDS_Vertex ev2 = TopExp.LastVertex(e, true);
                gp_Pnt ep1 = BRep_Tool.Pnt(ev1);
                gp_Pnt ep2 = BRep_Tool.Pnt(ev2);
                if (point.Distance(ep1) <= BRep_Tool.Tolerance(ev1))
                {
                    PtOnVertex = true;
                    OnVertex = ev1;
                    break;
                }
                else if (point.Distance(ep2) <= BRep_Tool.Tolerance(ev1))
                {
                    PtOnVertex = true;
                    OnVertex = ev2;
                    break;
                }
                break;
            }
        }
    }

    /// <summary>The profile of a sliding rib or groove: part of the bounding box closed by the contour.</summary>
    protected bool SlidingProfile(ref TopoDS_Face Prof, bool RevolRib, double myTol, ref int Concavite, Geom_Plane myPln,
                                  in TopoDS_Face BndFace, in gp_Pnt CheckPnt, in TopoDS_Face FirstFace,
                                  in TopoDS_Face LastFace, in TopoDS_Vertex FirstVertex, in TopoDS_Vertex LastVertex,
                                  in TopoDS_Edge FirstEdge, in TopoDS_Edge LastEdge)
    {
        bool ProfileOK = true;
        // --case of sliding : construction of the wire of the profile
        // --> 1 part bounding box + 1 part wire
        //   attention to the compatibility of orientations

        gp_Dir FN, LN;
        BRepLib_MakeWire WW = new();

        FN = Normal(FirstFace, myFirstPnt);
        LN = Normal(LastFace, myLastPnt);

        // Case of the groove (cut) <> rib (fuse)
        // -> we are in the material
        // -> make everything in 2d in the working plane : easier
        if (!myFuse)
        {
            FN = -FN;
            LN = -LN;
        }

        Geom_Line ln1, ln2;
        gp_Pnt Pt = new();

        ln2 = new Geom_Line(myFirstPnt, FN);
        ln1 = new Geom_Line(myLastPnt, LN);

        Geom2d_Curve ln2d1 = GeomAPI.To2d(ln1, myPln.Pln())!;
        Geom2d_Curve ln2d2 = GeomAPI.To2d(ln2, myPln.Pln())!;

        Geom2dAPI_InterCurveCurve inter = new(ln2d1, ln2d2, Precision.Confusion());

        bool TestOK = true;
        if (RevolRib)
        {
            gp_Dir d1, d2;
            d1 = ln1.Position().Direction();
            d2 = ln2.Position().Direction();
            if (d1.IsOpposite(d2, myTol))
            {
                double par1 = ElCLib.Parameter(ln1.Lin(), myFirstPnt);
                double par2 = ElCLib.Parameter(ln2.Lin(), myLastPnt);
                if (par1 >= myTol || par2 >= myTol)
                {
                    Concavite = 2; // parallel and concave
                    BRepLib_MakeEdge e1 = new(myLastPnt, myFirstPnt);
                    WW.Add(e1.InternalEdge());
                }
            }
            if (d1.IsEqual(d2, myTol))
            {
                if (Concavite == 3)
                {
                    TestOK = false;
                }
            }
        }

        if (TestOK)
        {
            if (inter.NbPoints() > 0)
            {
                gp_Pnt2d P = inter.Point(1);
                myPln.D0(P.X(), P.Y(), out Pt);
                double par = IntPar(ln1, Pt);
                if (par > 0)
                {
                    Concavite = 1; // concave
                }
            }
        }

        // ---Construction of the profile face
        if (Concavite == 1)
        {
            // if concave : it is possible to extend first and last edges of the wire
            //              to the bounding box
            BRepLib_MakeEdge e1 = new(myLastPnt, Pt);
            WW.Add(e1.InternalEdge());
            BRepLib_MakeEdge e2 = new(Pt, myFirstPnt);
            WW.Add(e2.InternalEdge());
        }
        else if (Concavite == 3)
        {
            // BndEdge : edges of intersection with the bounding box
            TopoDS_Edge BndEdge1 = new(), BndEdge2 = new();
            // Points of intersection with the bounding box / Find Profile
            gp_Pnt BndPnt1 = new(), BndPnt2 = new(), LastPnt = new();
            TopExp_Explorer expl = new();
            expl.Init(BndFace, TopAbs_WIRE);
            BRepTools_WireExplorer explo = new();
            TopoDS_Wire BndWire = TopoDS.Wire(expl.Current());
            explo.Init(BndWire);
            for (; explo.More(); explo.Next())
            {
                TopoDS_Edge e = TopoDS.Edge(explo.Current());
                Geom_Curve c = BRep_Tool.Curve(e, out double first, out double last)!;
                Geom2d_Curve c2d = GeomAPI.To2d(c, myPln.Pln())!;
                Geom2dAPI_InterCurveCurve intcln1 = new(ln2d1, c2d, Precision.Confusion());
                if (intcln1.NbPoints() > 0)
                {
                    gp_Pnt2d p2d = intcln1.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out gp_Pnt p);
                    double parl = IntPar(ln1, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        BndEdge1 = e;
                        BndPnt1 = p;
                    }
                }

                Geom2dAPI_InterCurveCurve intcln2 = new(ln2d2, c2d, Precision.Confusion());
                if (intcln2.NbPoints() > 0)
                {
                    gp_Pnt2d p2d = intcln2.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out gp_Pnt p);
                    double parl = IntPar(ln2, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        BndEdge2 = e;
                        BndPnt2 = p;
                    }
                }
                if (!BndEdge1.IsNull() && !BndEdge2.IsNull())
                {
                    break;
                }
            }

            if (BndEdge1.IsNull() || BndEdge2.IsNull())
            {
                ProfileOK = false;
                return ProfileOK;
            }

            BRepLib_MakeEdge e1 = new(myLastPnt, BndPnt1);
            WW.Add(e1.InternalEdge());

            if (BndEdge1.IsSame(BndEdge2))
            {
                // Particular case : same edge -> simply determined path
                BRepLib_MakeEdge e2 = new(BndPnt1, BndPnt2);
                WW.Add(e2.InternalEdge());
                BRepLib_MakeEdge e3 = new(BndPnt2, myFirstPnt);
                WW.Add(e3.InternalEdge());
            }
            else
            {
                explo.Init(BndWire);
                for (; explo.More(); explo.Next())
                {
                    TopoDS_Edge e = TopoDS.Edge(explo.Current());
                    if (e.IsSame(BndEdge1))
                    {
                        gp_Pnt pp;
                        pp = BRep_Tool.Pnt(TopExp.LastVertex(e, true));
                        if (pp.Distance(BndPnt1) >= BRep_Tool.Tolerance(e))
                        {
                            LastPnt = pp;
                        }
                        BRepLib_MakeEdge e2 = new(BndPnt1, LastPnt);
                        WW.Add(e2.InternalEdge());
                        break;
                    }
                }

                if (explo.More())
                {
                    explo.Next();
                    if (explo.Current().IsNull())
                    {
                        explo.Init(BndWire);
                    }
                }
                else
                {
                    explo.Init(BndWire);
                }

                // Check if this is BndEdge2
                // -> if yes : it is required to turn to join FirstPnt
                // -> if no : add edges
                bool Fin = false;
                while (!Fin)
                {
                    TopoDS_Edge e = TopoDS.Edge(explo.Current());
                    if (!e.IsSame(BndEdge2))
                    {
                        gp_Pnt pp;
                        pp = BRep_Tool.Pnt(TopExp.LastVertex(e, true));
                        BRepLib_MakeEdge ee = new(LastPnt, pp);
                        WW.Add(ee.InternalEdge());
                        LastPnt = pp;
                    }
                    else
                    {
                        // the path is closed
                        // -> since met BndEdge2, end of borders on BndFace
                        Fin = true;
                        BRepLib_MakeEdge ee = new(LastPnt, BndPnt2);
                        WW.Add(ee.InternalEdge());
                        LastPnt = BndPnt2;
                    }
                    if (explo.More())
                    {
                        explo.Next();
                        if (explo.Current().IsNull())
                        {
                            explo.Init(BndWire);
                        }
                    }
                    else
                    {
                        explo.Init(BndWire);
                    }
                }

                BRepLib_MakeEdge e3 = new(BndPnt2, myFirstPnt);
                WW.Add(e3.InternalEdge());
            }
        }

        // ---Construction of the profile

        // Explore the wire provided by the user
        // BRepTools_WireExplorer : correct order - without repetition <> TopExp : non ordered
        BRepTools_WireExplorer EX = new(myWire);

        Geom_Curve FirstCurve = BRep_Tool.Curve(FirstEdge, out double ff, out double ll)!;

        if (!FirstEdge.IsSame(LastEdge))
        {
            TopoDS_Vertex FLVert = TopExp.LastVertex(FirstEdge, true);
            gp_Pnt FLPnt = BRep_Tool.Pnt(FLVert);
            BRepLib_MakeEdge ef = new(FirstCurve, myFirstPnt, FLPnt);
            WW.Add(ef.InternalEdge());
            for (; EX.More(); EX.Next())
            {
                TopoDS_Edge E = EX.Current();
                if (E.IsSame(FirstEdge))
                {
                    break;
                }
            }
            EX.Next();
            for (; EX.More(); EX.Next())
            {
                TopoDS_Edge E = EX.Current();
                if (!E.IsSame(LastEdge))
                {
                    WW.Add(E);
                }
                else
                {
                    break;
                }
            }
            Geom_Curve LastCurve = BRep_Tool.Curve(LastEdge, out ff, out ll)!;
            TopoDS_Vertex LFVert = TopExp.FirstVertex(LastEdge, true);
            gp_Pnt LFPnt = BRep_Tool.Pnt(LFVert);
            BRepLib_MakeEdge el = new(LastCurve, LFPnt, myLastPnt);
            WW.Add(el.InternalEdge());
        }
        else
        {
            // only one edge : particular processing
            double fpar = IntPar(FirstCurve, myFirstPnt);
            double lpar = IntPar(FirstCurve, myLastPnt);
            Geom_Curve c;
            if (fpar > lpar)
            {
                c = FirstCurve.Reversed();
            }
            else
            {
                c = FirstCurve;
            }

            BRepLib_MakeEdge ef = new(c, myFirstPnt, myLastPnt);
            WW.Add(ef.InternalEdge());
        }

        BRepLib_MakeFace f = new(myPln.Pln(), WW.InternalWire(), true);
        TopoDS_Face fac = TopoDS.Face(f.InternalShape());

        if (!BRepAlgo.IsValid(fac))
        {
            ProfileOK = false;
            return ProfileOK;
        }

        if (Concavite != 3)
        {
            // if concave : face is OK
            Prof = fac;
        }
        else
        {
            // if not concave
            // CheckPnt : point slightly inside the material side
            // Bndface  : face/cut of the bounding box in the plane of the profile
            BRepTopAdaptor_FClass2d Cl = new(fac, BRep_Tool.Tolerance(fac));
            ElSLib.Parameters(myPln.Pln(), CheckPnt, out double u, out double v);
            gp_Pnt2d checkpnt2d = new(u, v);
            if (Cl.Perform(checkpnt2d, true) == TopAbs_OUT)
            {
                // If face is not the correct part of BndFace take the complementary
                BRepAlgoAPI_Cut c = new(BndFace, fac);
                TopExp_Explorer exp = new(c.InternalShape(), TopAbs_WIRE);
                TopoDS_Wire w = TopoDS.Wire(exp.Current());
                BRepLib_MakeFace ffx = new(w);
                Prof = TopoDS.Face(ffx.InternalShape());
            }
            else
            {
                // If face is the correct part of BndFace  : face is OK
                Prof = fac;
            }
        }

        if (!BRepAlgo.IsValid(Prof))
        {
            ProfileOK = false;
            return ProfileOK;
        }
        return ProfileOK;
    }

    /// <summary>The profile of a rib or groove without sliding: the contour extended to the bounding box.</summary>
    protected bool NoSlidingProfile(ref TopoDS_Face Prof, bool RevolRib, double myTol, ref int Concavite,
                                    Geom_Plane myPln, double bnd, in TopoDS_Face BndFace, in gp_Pnt CheckPnt,
                                    in TopoDS_Face FirstFace, in TopoDS_Face LastFace, in TopoDS_Vertex FirstVertex,
                                    in TopoDS_Vertex LastVertex, in TopoDS_Edge FirstEdge, in TopoDS_Edge LastEdge,
                                    bool OnFirstFace, bool OnLastFace)
    {
        bool ProfileOK = true;

        TopoDS_Vertex theFV = new();
        theFV.Nullify();
        gp_Pnt theFirstpoint = new();
        TopoDS_Edge theLastEdge = new();
        theLastEdge.Nullify();

        gp_Pnt firstpoint, lastpoint;
        gp_Vec firstvect, lastvect;
        TopoDS_Wire w = new();
        BRep_Builder BB = new();
        BB.MakeWire(ref w);
        TopoDS_Edge FalseFirstEdge = new(), FalseLastEdge = new(), FalseOnlyOne = new();

        Geom_Curve FirstCurve = BRep_Tool.Curve(FirstEdge, out double f1, out double l1)!;
        Geom_Curve LastCurve = BRep_Tool.Curve(LastEdge, out double f2, out double l2)!;

        Geom_Line firstln, lastln;
        FirstCurve.D1(f1, out firstpoint, out firstvect);
        lastln = new Geom_Line(firstpoint, new gp_Dir(-firstvect));
        LastCurve.D1(l2, out lastpoint, out lastvect);
        firstln = new Geom_Line(lastpoint, new gp_Dir(lastvect));

        gp_Pnt Pt = new();

        Geom2d_Curve ln2d1 = GeomAPI.To2d(firstln, myPln.Pln())!;
        Geom2d_Curve ln2d2 = GeomAPI.To2d(lastln, myPln.Pln())!;

        Geom2dAPI_InterCurveCurve inter = new(ln2d1, ln2d2, Precision.Confusion());

        bool TestOK = true;
        if (RevolRib)
        {
            gp_Dir d1, d2;
            d1 = firstln.Position().Direction();
            d2 = lastln.Position().Direction();
            if (d1.IsOpposite(d2, myTol))
            {
                double par1 = ElCLib.Parameter(firstln.Lin(), myFirstPnt);
                double par2 = ElCLib.Parameter(lastln.Lin(), myLastPnt);
                if (par1 >= myTol || par2 >= myTol)
                {
                    Concavite = 2; // parallel and concave
                }
            }
            if (d1.IsEqual(d2, myTol))
            {
                if (Concavite == 3)
                {
                    TestOK = false;
                }
            }
        }

        if (TestOK)
        {
            if (inter.NbPoints() > 0)
            {
                gp_Pnt2d P = inter.Point(1);
                myPln.D0(P.X(), P.Y(), out Pt);
                double par = IntPar(firstln, Pt);
                if (par > 0)
                {
                    Concavite = 1; // concave
                }
            }
        }

        // ---Construction of the face profile
        if (Concavite == 3)
        {
            if (OnFirstFace)
            {
                FalseFirstEdge = FirstEdge;
                EdgeExtention(ref FalseFirstEdge, bnd, true);
                TopoDS_Vertex vv1 = TopExp.FirstVertex(FalseFirstEdge, true);
                firstpoint = BRep_Tool.Pnt(vv1);
                Geom_Curve cc = BRep_Tool.Curve(FalseFirstEdge, out double f, out double l)!;
                cc.D1(f, out firstpoint, out firstvect);
                lastln = new Geom_Line(firstpoint, new gp_Dir(-firstvect));
                if (FirstEdge.IsSame(LastEdge))
                {
                    FalseOnlyOne = FalseFirstEdge;
                }
                ln2d2 = GeomAPI.To2d(lastln, myPln.Pln())!;
            }
            if (OnLastFace)
            {
                if (!FirstEdge.IsSame(LastEdge))
                {
                    FalseLastEdge = LastEdge;
                }
                else
                {
                    if (FalseOnlyOne.IsNull())
                    {
                        FalseOnlyOne = LastEdge;
                    }
                    FalseLastEdge = FalseOnlyOne;
                }
                EdgeExtention(ref FalseLastEdge, bnd, false);
                if (FirstEdge.IsSame(LastEdge))
                {
                    FalseOnlyOne = FalseLastEdge;
                }
                TopoDS_Vertex vv2 = TopExp.LastVertex(FalseLastEdge, true);
                lastpoint = BRep_Tool.Pnt(vv2);
                Geom_Curve cc = BRep_Tool.Curve(FalseLastEdge, out double f, out double l)!;
                cc.D1(l, out lastpoint, out lastvect);
                lastpoint = BRep_Tool.Pnt(vv2);
                firstln = new Geom_Line(lastpoint, new gp_Dir(lastvect));
                ln2d1 = GeomAPI.To2d(firstln, myPln.Pln())!;
            }

            TopoDS_Edge BndEdge1 = new(), BndEdge2 = new();
            gp_Pnt BndPnt1 = new(), BndPnt2 = new(), LastPnt = new();
            TopExp_Explorer expl = new();
            expl.Init(BndFace, TopAbs_WIRE);
            BRepTools_WireExplorer explo = new();
            TopoDS_Wire BndWire = TopoDS.Wire(expl.Current());
            explo.Init(BndWire);
            for (; explo.More(); explo.Next())
            {
                TopoDS_Edge e = TopoDS.Edge(explo.Current());
                Geom_Curve c = BRep_Tool.Curve(e, out double first, out double last)!;
                Geom2d_Curve c2d = GeomAPI.To2d(c, myPln.Pln())!;
                Geom2dAPI_InterCurveCurve intcln1 = new(ln2d1, c2d, Precision.Confusion());
                if (intcln1.NbPoints() > 0)
                {
                    gp_Pnt2d p2d = intcln1.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out gp_Pnt p);
                    double parl = IntPar(firstln, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        BndEdge1 = e;
                        BndPnt1 = p;
                    }
                }

                Geom2dAPI_InterCurveCurve intcln2 = new(ln2d2, c2d, Precision.Confusion());
                if (intcln2.NbPoints() > 0)
                {
                    gp_Pnt2d p2d = intcln2.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out gp_Pnt p);
                    double parl = IntPar(lastln, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        BndEdge2 = e;
                        BndPnt2 = p;
                    }
                }
                if (!BndEdge1.IsNull() && !BndEdge2.IsNull())
                {
                    break;
                }
            }

            if (BndEdge1.IsNull() || BndEdge2.IsNull())
            {
                ProfileOK = false;
                return ProfileOK;
            }

            TopoDS_Edge ee1;
            if (theLastEdge.IsNull())
            {
                BRepLib_MakeEdge e1 = new(lastpoint, BndPnt1);
                ee1 = TopoDS.Edge(e1.InternalShape());
            }
            else
            {
                TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                BRepLib_MakeVertex v2 = new(BndPnt1);
                BRepLib_MakeEdge e1 = new(v1, v2.InternalVertex());
                ee1 = TopoDS.Edge(e1.InternalShape());
            }
            BB.Add(w, ee1);
            theLastEdge = ee1;
            if (theFV.IsNull())
            {
                theFV = TopExp.FirstVertex(ee1, true);
                theFirstpoint = BRep_Tool.Pnt(theFV);
            }

            if (BndEdge1.IsSame(BndEdge2))
            {
                TopoDS_Edge ee2, ee3;
                if (theLastEdge.IsNull())
                {
                    BRepLib_MakeEdge e2 = new(BndPnt1, BndPnt2);
                    ee2 = TopoDS.Edge(e2.InternalShape());
                }
                else
                {
                    TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                    BRepLib_MakeVertex v2 = new(BndPnt2);
                    BRepLib_MakeEdge e2 = new(v1, v2.InternalVertex());
                    ee2 = TopoDS.Edge(e2.InternalShape());
                }
                BB.Add(w, ee2);
                theLastEdge = ee2;
                if (theFV.IsNull())
                {
                    theFV = TopExp.FirstVertex(ee2, true);
                    theFirstpoint = BRep_Tool.Pnt(theFV);
                }
                if (theLastEdge.IsNull())
                {
                    BRepLib_MakeEdge e3 = new(BndPnt2, firstpoint);
                    ee3 = TopoDS.Edge(e3.InternalShape());
                }
                else
                {
                    TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                    BRepLib_MakeVertex v2 = new(firstpoint);
                    BRepLib_MakeEdge e3 = new(v1, v2.InternalVertex());
                    ee3 = TopoDS.Edge(e3.InternalShape());
                }
                BB.Add(w, ee3);
                theLastEdge = ee3;
                if (theFV.IsNull())
                {
                    theFV = TopExp.FirstVertex(ee3, true);
                    theFirstpoint = BRep_Tool.Pnt(theFV);
                }
            }
            else
            {
                explo.Init(BndWire);
                for (; explo.More(); explo.Next())
                {
                    TopoDS_Edge e = TopoDS.Edge(explo.Current());
                    if (e.IsSame(BndEdge1))
                    {
                        gp_Pnt pp;
                        pp = BRep_Tool.Pnt(TopExp.LastVertex(e, true));
                        if (pp.Distance(BndPnt1) > BRep_Tool.Tolerance(e))
                        {
                            LastPnt = pp;
                        }
                        TopoDS_Edge eee;
                        if (theLastEdge.IsNull())
                        {
                            BRepLib_MakeEdge e2 = new(BndPnt1, LastPnt);
                            eee = TopoDS.Edge(e2.InternalShape());
                        }
                        else
                        {
                            TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                            BRepLib_MakeVertex v2 = new(LastPnt);
                            BRepLib_MakeEdge e2 = new(v1, v2.InternalVertex());
                            eee = TopoDS.Edge(e2.InternalShape());
                        }
                        BB.Add(w, eee);
                        theLastEdge = eee;
                        if (theFV.IsNull())
                        {
                            theFV = TopExp.FirstVertex(eee, true);
                            theFirstpoint = BRep_Tool.Pnt(theFV);
                        }
                        break;
                    }
                }

                if (explo.More())
                {
                    explo.Next();
                    if (explo.Current().IsNull())
                    {
                        explo.Init(BndWire);
                    }
                }
                else
                {
                    explo.Init(BndWire);
                }
                bool Fin = false;
                while (!Fin)
                {
                    TopoDS_Edge e = TopoDS.Edge(explo.Current());
                    if (!e.IsSame(BndEdge2))
                    {
                        gp_Pnt pp;
                        pp = BRep_Tool.Pnt(TopExp.LastVertex(e, true));
                        TopoDS_Edge eee1;
                        if (theLastEdge.IsNull())
                        {
                            BRepLib_MakeEdge ee = new(LastPnt, pp);
                            eee1 = TopoDS.Edge(ee.InternalShape());
                        }
                        else
                        {
                            TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                            BRepLib_MakeVertex v2 = new(pp);
                            BRepLib_MakeEdge ee = new(v1, v2.InternalVertex());
                            eee1 = TopoDS.Edge(ee.InternalShape());
                        }
                        BB.Add(w, eee1);
                        theLastEdge = eee1;
                        if (theFV.IsNull())
                        {
                            theFV = TopExp.FirstVertex(eee1, true);
                            theFirstpoint = BRep_Tool.Pnt(theFV);
                        }
                        LastPnt = pp;
                    }
                    else
                    {
                        Fin = true;
                        TopoDS_Edge eee2;
                        if (theLastEdge.IsNull())
                        {
                            BRepLib_MakeEdge ee = new(LastPnt, BndPnt2);
                            eee2 = TopoDS.Edge(ee.InternalShape());
                        }
                        else
                        {
                            TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                            BRepLib_MakeVertex v2 = new(BndPnt2);
                            BRepLib_MakeEdge ee = new(v1, v2.InternalVertex());
                            eee2 = TopoDS.Edge(ee.InternalShape());
                        }
                        BB.Add(w, eee2);
                        theLastEdge = eee2;
                        if (theFV.IsNull())
                        {
                            theFV = TopExp.FirstVertex(eee2, true);
                            theFirstpoint = BRep_Tool.Pnt(theFV);
                        }
                        LastPnt = BndPnt2;
                    }
                    if (explo.More())
                    {
                        explo.Next();
                        if (explo.Current().IsNull())
                        {
                            explo.Init(BndWire);
                        }
                    }
                    else
                    {
                        explo.Init(BndWire);
                    }
                }

                TopoDS_Edge eee3;
                if (theLastEdge.IsNull())
                {
                    BRepLib_MakeEdge e3 = new(BndPnt2, firstpoint);
                    eee3 = TopoDS.Edge(e3.InternalShape());
                }
                else
                {
                    TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                    BRepLib_MakeVertex v2 = new(firstpoint);
                    BRepLib_MakeEdge e3 = new(v1, v2.InternalVertex());
                    eee3 = TopoDS.Edge(e3.InternalShape());
                }
                BB.Add(w, eee3);
                theLastEdge = eee3;
                if (theFV.IsNull())
                {
                    theFV = TopExp.FirstVertex(eee3, true);
                    theFirstpoint = BRep_Tool.Pnt(theFV);
                }
            }
        }

        if (Concavite == 1)
        {
            TopoDS_Edge eee4;
            if (theLastEdge.IsNull())
            {
                BRepLib_MakeEdge e = new(Pt, firstpoint);
                eee4 = TopoDS.Edge(e.InternalShape());
            }
            else
            {
                TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                BRepLib_MakeVertex v2 = new(firstpoint);
                BRepLib_MakeEdge e = new(v1, v2.InternalVertex());
                eee4 = TopoDS.Edge(e.InternalShape());
            }
            BB.Add(w, eee4);
            if (theFV.IsNull())
            {
                theFV = TopExp.FirstVertex(eee4, true);
                theFirstpoint = BRep_Tool.Pnt(theFV);
            }
            theLastEdge = eee4;
        }

        if (FirstEdge.IsSame(LastEdge))
        {
            if (!myLFMap.IsBound(FirstEdge))
            {
                NCollection_List<TopoDS_Shape> thelist = new();
                myLFMap.Bind(FirstEdge, thelist);
            }
            if (OnFirstFace || OnLastFace)
            {
                TopoDS_Edge theEdge;
                Geom_Curve cc = BRep_Tool.Curve(FalseOnlyOne, out double f, out double l)!;
                if (!theLastEdge.IsNull())
                {
                    TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                    TopoDS_Vertex v2;
                    gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(FalseOnlyOne, true));
                    if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = TopExp.LastVertex(FalseOnlyOne, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = TopoDS.Edge(e.InternalShape());
                }
                else
                {
                    TopoDS_Vertex v1 = TopExp.FirstVertex(FalseOnlyOne, true);
                    TopoDS_Vertex v2;
                    gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(FalseOnlyOne, true));
                    if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = TopExp.LastVertex(FalseOnlyOne, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = TopoDS.Edge(e.InternalShape());
                }
                myLFMap[FirstEdge].Append(theEdge);
                BB.Add(w, theEdge);
                if (theFV.IsNull())
                {
                    theFV = TopExp.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }
            else
            {
                Geom_Curve cc = BRep_Tool.Curve(FirstEdge, out double f, out double l)!;
                TopoDS_Edge theEdge;
                if (!theLastEdge.IsNull())
                {
                    TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                    TopoDS_Vertex v2;
                    // Attention case Wire Reversed -> LastVertex without true
                    gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(FirstEdge));
                    if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = TopExp.LastVertex(FirstEdge);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = TopoDS.Edge(e.InternalShape());
                }
                else
                {
                    TopoDS_Vertex v1 = TopExp.FirstVertex(FirstEdge, true);
                    TopoDS_Vertex v2;
                    gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(FirstEdge, true));
                    if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = TopExp.LastVertex(FirstEdge, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = TopoDS.Edge(e.InternalShape());
                }
                myLFMap[FirstEdge].Append(theEdge);
                BB.Add(w, theEdge);
                if (theFV.IsNull())
                {
                    theFV = TopExp.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }
        }
        else
        {
            if (!myLFMap.IsBound(FirstEdge))
            {
                NCollection_List<TopoDS_Shape> thelist1 = new();
                myLFMap.Bind(FirstEdge, thelist1);
            }
            if (!OnFirstFace)
            {
                TopoDS_Edge theEdge;
                Geom_Curve cc = BRep_Tool.Curve(FirstEdge, out double f, out double l)!;
                if (!theLastEdge.IsNull())
                {
                    TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                    TopoDS_Vertex v2 = TopExp.LastVertex(FirstEdge, true);
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = TopoDS.Edge(e.InternalShape());
                }
                else
                {
                    theEdge = FirstEdge;
                }
                myLFMap[FirstEdge].Append(theEdge);
                BB.Add(w, theEdge);
                if (theFV.IsNull())
                {
                    theFV = TopExp.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }
            else
            {
                TopoDS_Edge theEdge;
                Geom_Curve cc = BRep_Tool.Curve(FalseFirstEdge, out double f, out double l)!;
                if (!theLastEdge.IsNull())
                {
                    TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                    TopoDS_Vertex v2 = TopExp.LastVertex(FalseFirstEdge, true);
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = TopoDS.Edge(e.InternalShape());
                }
                else
                {
                    theEdge = FalseFirstEdge;
                }
                myLFMap[FirstEdge].Append(theEdge);
                BB.Add(w, theEdge);
                if (theFV.IsNull())
                {
                    theFV = TopExp.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }

            BRepTools_WireExplorer ex = new(myWire);
            for (; ex.More(); ex.Next())
            {
                TopoDS_Edge E = ex.Current();
                if (E.IsSame(FirstEdge))
                {
                    break;
                }
            }

            ex.Next();

            for (; ex.More(); ex.Next())
            {
                TopoDS_Edge E = ex.Current();
                if (!E.IsSame(LastEdge))
                {
                    if (!myLFMap.IsBound(E))
                    {
                        NCollection_List<TopoDS_Shape> thelist2 = new();
                        myLFMap.Bind(E, thelist2);
                    }
                    TopoDS_Edge eee;
                    Geom_Curve cc = BRep_Tool.Curve(E, out double f, out double l)!;
                    if (!theLastEdge.IsNull())
                    {
                        TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                        TopoDS_Vertex v2 = TopExp.LastVertex(E, true);
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = TopoDS.Edge(e.InternalShape());
                    }
                    else
                    {
                        eee = E;
                    }
                    myLFMap[E].Append(eee);
                    BB.Add(w, eee);
                    if (theFV.IsNull())
                    {
                        theFV = TopExp.FirstVertex(eee, true);
                    }
                    theLastEdge = eee;
                }
                else
                {
                    break;
                }
            }

            if (!OnLastFace)
            {
                if (!FirstEdge.IsSame(LastEdge))
                {
                    TopoDS_Edge edg = TopoDS.Edge(ex.Current());
                    if (!myLFMap.IsBound(edg))
                    {
                        NCollection_List<TopoDS_Shape> thelist3 = new();
                        myLFMap.Bind(edg, thelist3);
                    }
                    TopoDS_Edge eee;
                    Geom_Curve cc = BRep_Tool.Curve(edg, out double f, out double l)!;
                    if (!theLastEdge.IsNull())
                    {
                        TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                        TopoDS_Vertex v2;
                        gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(edg, true));
                        if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = TopExp.LastVertex(edg, true);
                        }
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = TopoDS.Edge(e.InternalShape());
                    }
                    else
                    {
                        TopoDS_Vertex v1 = TopExp.FirstVertex(edg, true);
                        TopoDS_Vertex v2;
                        gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(edg, true));
                        if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = TopExp.LastVertex(edg, true);
                        }
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = TopoDS.Edge(e.InternalShape());
                    }
                    myLFMap[edg].Append(eee);
                    BB.Add(w, eee);
                    if (theFV.IsNull())
                    {
                        theFV = TopExp.FirstVertex(eee, true);
                    }
                    theLastEdge = eee;
                }
                else
                {
                    TopoDS_Edge eee;
                    if (!myLFMap.IsBound(LastEdge))
                    {
                        NCollection_List<TopoDS_Shape> thelist4 = new();
                        myLFMap.Bind(LastEdge, thelist4);
                    }
                    Geom_Curve cc = BRep_Tool.Curve(FalseOnlyOne, out double f, out double l)!;
                    if (!theLastEdge.IsNull())
                    {
                        TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                        TopoDS_Vertex v2;
                        gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(FalseOnlyOne, true));
                        if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = TopExp.LastVertex(FalseOnlyOne, true);
                        }
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = TopoDS.Edge(e.InternalShape());
                    }
                    else
                    {
                        TopoDS_Vertex v1 = TopExp.FirstVertex(FalseOnlyOne, true);
                        TopoDS_Vertex v2;
                        gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(FalseOnlyOne, true));
                        if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = TopExp.LastVertex(FalseOnlyOne, true);
                        }
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = TopoDS.Edge(e.InternalShape());
                    }
                    myLFMap[LastEdge].Append(eee);
                    BB.Add(w, eee);
                    if (theFV.IsNull())
                    {
                        theFV = TopExp.FirstVertex(eee, true);
                    }
                    theLastEdge = eee;
                }
            }
            else
            {
                TopoDS_Edge eee;
                if (!myLFMap.IsBound(LastEdge))
                {
                    NCollection_List<TopoDS_Shape> thelist5 = new();
                    myLFMap.Bind(LastEdge, thelist5);
                }
                Geom_Curve cc = BRep_Tool.Curve(FalseLastEdge, out double f, out double l)!;
                if (!theLastEdge.IsNull())
                {
                    TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                    TopoDS_Vertex v2;
                    gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(FalseLastEdge, true));
                    if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = TopExp.LastVertex(FalseLastEdge, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    eee = TopoDS.Edge(e.InternalShape());
                }
                else
                {
                    TopoDS_Vertex v1 = TopExp.FirstVertex(FalseLastEdge, true);
                    TopoDS_Vertex v2;
                    gp_Pnt pp = BRep_Tool.Pnt(TopExp.LastVertex(FalseLastEdge, true));
                    if (!theFV.IsNull() && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = TopExp.LastVertex(FalseLastEdge, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    eee = TopoDS.Edge(e.InternalShape());
                }
                myLFMap[LastEdge].Append(eee);
                BB.Add(w, eee);
                if (theFV.IsNull())
                {
                    theFV = TopExp.FirstVertex(eee, true);
                }
                theLastEdge = eee;
            }
        }

        if (Concavite == 1)
        {
            TopoDS_Edge eef;
            if (theLastEdge.IsNull())
            {
                BRepLib_MakeEdge ef = new(lastpoint, Pt);
                eef = TopoDS.Edge(ef.InternalShape());
            }
            else
            {
                TopoDS_Vertex v1 = TopExp.LastVertex(theLastEdge, true);
                BRepLib_MakeVertex vv = new(Pt);
                TopoDS_Vertex v2 = TopoDS.Vertex(vv.InternalShape());
                if (!theFV.IsNull() && Pt.Distance(theFirstpoint) <= myTol)
                {
                    v2 = theFV;
                }
                BRepLib_MakeEdge ef = new(v1, v2);
                eef = TopoDS.Edge(ef.InternalShape());
            }
            BB.Add(w, eef);
            if (theFV.IsNull())
            {
                theFV = TopExp.FirstVertex(eef, true);
            }
            theLastEdge = eef;
        }

        if (Concavite == 2)
        {
            BRepLib_MakeEdge ee = new(lastpoint, firstpoint);
            TopoDS_Edge e = ee.InternalEdge();
            BB.Add(w, e);
        }

        w.Closed(BRep_Tool.IsClosed(w));
        BRepLib_MakeFace fa = new(myPln.Pln(), w, true);
        TopoDS_Face fac = TopoDS.Face(fa.InternalShape());

        if (!BRepAlgo.IsValid(fac))
        {
            ProfileOK = false;
            return ProfileOK;
        }

        if (Concavite == 3)
        {
            BRepTopAdaptor_FClass2d Cl = new(fac, BRep_Tool.Tolerance(fac));
            ElSLib.Parameters(myPln.Pln(), CheckPnt, out double u, out double v);
            gp_Pnt2d checkpnt2d = new(u, v);
            if (Cl.Perform(checkpnt2d, true) == TopAbs_OUT)
            {
                BRepAlgoAPI_Cut c = new(BndFace, fac);
                TopExp_Explorer exp = new(c.InternalShape(), TopAbs_WIRE);
                UpdateDescendants(c, c.InternalShape(), false);
                TopoDS_Wire ww = TopoDS.Wire(exp.Current());
                BRepLib_MakeFace ff = new(ww);
                Prof = TopoDS.Face(ff.InternalShape());
            }
            else
            {
                Prof = fac;
            }
        }
        else
        {
            Prof = fac;
        }

        if (!BRepAlgo.IsValid(Prof))
        {
            ProfileOK = false;
            return ProfileOK;
        }
        return ProfileOK;
    }

    protected void UpdateDescendants(BRepAlgoAPI_BooleanOperation aBOP, in TopoDS_Shape S, bool SkipFace = false)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new(), it2 = new();
        TopExp_Explorer exp = new();

        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itdm in myMap)
        {
            TopoDS_Shape orig = itdm.Key;
            if (SkipFace && orig.ShapeType() == TopAbs_FACE)
            {
                continue;
            }
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 9503)
            NCollection_IndexedMap<TopoDS_Shape> newdsc = new(TopTools_ShapeMapHasher.Instance);

            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                TopoDS_Shape sh = it.Value();
                if (sh.ShapeType() != TopAbs_FACE)
                {
                    continue;
                }
                TopoDS_Face fdsc = TopoDS.Face(it.Value());
                for (exp.Init(S, TopAbs_FACE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(fdsc))
                    { // preserved
                        newdsc.Add(fdsc);
                        break;
                    }
                }
                if (!exp.More())
                {
                    NCollection_List<TopoDS_Shape> aLM = aBOP.InternalModified(fdsc);
                    it2.Initialize(aLM);
                    for (; it2.More(); it2.Next())
                    {
                        TopoDS_Shape aS = it2.Value();
                        newdsc.Add(aS);
                    }
                }
            }
            myMap.ChangeFind(orig).Clear();
            foreach (TopoDS_Shape itm in newdsc)
            {
                // check the belonging to the shape...
                for (exp.Init(S, TopAbs_FACE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(itm))
                    {
                        myMap.ChangeFind(orig).Append(itm);
                        break;
                    }
                }
            }
        }
    }
}
