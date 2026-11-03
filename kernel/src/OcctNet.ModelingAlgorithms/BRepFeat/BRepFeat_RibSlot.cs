// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_RibSlot.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_RibSlot.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_RibSlot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.LocOpe_Operation;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
/// holds the caller's shapes (decision 1052).</para>
/// </remarks>
public abstract class BRepFeat_RibSlot : BRepBuilderAPI_MakeShape
{
    protected Point3D myFirstPnt = new();
    protected Point3D myLastPnt = new();
    protected bool myFuse;
    protected bool mySliding;
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> myMap = new(ShapeHasher.Instance);
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> myLFMap = new(ShapeHasher.Instance);
    protected Shape myFShape = new();
    protected Shape myLShape = new();
    protected BRepFeat_PerfSelection myPerfSelection;
    protected Wire myWire = new();
    protected Shape mySbase = new();
    protected Face mySkface = new();
    protected Face myPbase = new();
    protected Shape myGShape = new();
    protected Shape mySUntil = new();
    protected NCollection_DataMap<Shape, Shape> myGluedF = new(ShapeHasher.Instance);
    protected NCollection_List<Shape> myNewEdges = new();
    protected NCollection_List<Shape> myTgtEdges = new();
    protected NCollection_List<Shape> myFacesForDraft = new();
    protected BRepFeat_StatusError myStatusError;

    // OCCT's Modified and Generated return a function-static list
    private readonly NCollection_List<Shape> myList = new();

    protected BRepFeat_RibSlot()
    {
        myFuse = false;
        mySliding = false;
        myStatusError = BRepFeat_OK;
    }

    /// <summary>General perform method: the topological reconstruction of the result.</summary>
    protected void LFPerform()
    {
        if (mySbase.IsNull || myPbase.IsNull || mySkface.IsNull || myGShape.IsNull || myLFMap.IsEmpty())
        {
            myStatusError = BRepFeat_NotInitialized;
            NotDone();
            return;
        }

        Explorer exp = new(), exp2 = new();
        int theOpe = 2;

        if (!myGluedF.IsEmpty())
        {
            theOpe = 1;
        }

        // Hope that there is just a solid in the result
        if (!mySUntil.IsNull)
        {
            for (exp2.Init(mySUntil, ShapeType.Face); exp2.More(); exp2.Next())
            {
                Shape funtil = exp2.Current();
                for (exp.Init(mySbase, ShapeType.Face); exp.More(); exp.Next())
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
            foreach (KeyValuePair<Shape, Shape> itm in myGluedF)
            {
                Face glface = ShapeCasts.Face(itm.Key);
                Face fac = ShapeCasts.Face(myGluedF[glface]);
                for (exp.Init(myGShape, ShapeType.Face); exp.More(); exp.Next())
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
                myNewEdges = new NCollection_List<Shape>(theGlue.InternalEdges());
                myTgtEdges = new NCollection_List<Shape>(theGlue.InternalTgtEdges());
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
            NCollection_List<Shape> partsoftool = new();
            BRepClass3d_SolidClassifier oussa = new();
            bool bFlag;
            NCollection_List<Shape>.Iterator aIt = new();

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
                    double toler = Tool.Tolerance(myPbase) * 2;
                    //
                    for (; aIt.More(); aIt.Next())
                    {
                        oussa.Load(aIt.Value());
                        oussa.Perform(myFirstPnt, toler);
                        Containment sp1 = oussa.State();
                        oussa.Perform(myLastPnt, toler);
                        Containment sp2 = oussa.State();
                        if (sp1 != Out && sp2 != Out)
                        {
                            Shape S = aIt.Value();
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
    // sub-shapes it leaves unchanged go back to the caller's, and the history follows (decision 1052).
    private void SameParameterOfTheGluedResult()
    {
        CopyAtEntry aCopy = new(myShape);
        BRepLib.SameParameter(aCopy.Shape(), 1.0e-7, true);
        if (!aCopy.IsCopied)
        {
            return;
        }
        myShape = aCopy.Finish(aCopy.Shape());
        foreach (Shape aKey in myMap.Keys.ToList())
        {
            myMap[aKey] = Final(aCopy, myMap[aKey]);
        }
        myNewEdges = Final(aCopy, myNewEdges);
        myTgtEdges = Final(aCopy, myTgtEdges);
    }

    private static NCollection_List<Shape> Final(CopyAtEntry theCopy, NCollection_List<Shape> theList)
    {
        NCollection_List<Shape> aList = new();
        for (NCollection_List<Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            aList.Append(theCopy.Final(theCopy.Copied(anIt.Value())));
        }
        return aList;
    }

    /// <summary>True if the face <paramref name="F"/> has no descendant in the result.</summary>
    /// <exception cref="NoSuchObjectException"><paramref name="F"/> isn't a shape of the history.</exception>
    public override bool IsDeleted(in Shape F)
    {
        return myMap.Find(F).IsEmpty();
    }

    /// <summary>The faces generated from the face <paramref name="F"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape F)
    {
        if (myMap.IsBound(F))
        {
            myList.Clear();
            NCollection_List<Shape>.Iterator ite = new(myMap.Find(F));
            for (; ite.More(); ite.Next())
            {
                Shape sh = ite.Value();
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
    internal override NCollection_List<Shape> InternalGenerated(in Shape S)
    {
        if (S.ShapeType() != ShapeType.Face)
        {
            myGenerated.Clear();
            if (myLFMap.IsEmpty() || !myLFMap.IsBound(S))
            {
                if (myMap.IsBound(S))
                { // check if filter on face or not
                    myList.Clear();
                    NCollection_List<Shape>.Iterator ite = new(myMap.Find(S));
                    for (; ite.More(); ite.Next())
                    {
                        Shape sh = ite.Value();
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
                NCollection_List<Shape>.Iterator it = new(myLFMap.Find(S));
                myList.Clear();
                for (; it.More(); it.Next())
                {
                    if (myMap.IsBound(it.Value()))
                    {
                        NCollection_List<Shape>.Iterator it1 = new(myMap.Find(it.Value()));
                        for (; it1.More(); it1.Next())
                        {
                            Shape sh = it1.Value();
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
        NCollection_List<Shape>.Iterator it = new(), it2 = new();

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in myMap)
        {
            Shape orig = itdm.Key;
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 1054)
            NCollection_IndexedMap<Shape> newdsc = new(ShapeHasher.Instance);
            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                Face fdsc = ShapeCasts.Face(it.Value());
                for (it2.Initialize(G.InternalDescendantFaces(fdsc)); it2.More(); it2.Next())
                {
                    newdsc.Add(it2.Value());
                }
            }
            myMap.ChangeFind(orig).Clear();
            foreach (Shape itm in newdsc)
            {
                myMap.ChangeFind(orig).Append(itm);
            }
        }
    }

    /// <summary>The shapes created at the bottom of the created form; it may be empty (frozen).</summary>
    public ISchematiqList<Shape> FirstShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalFirstShape());
    }

    /// <summary><see cref="FirstShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalFirstShape()
    {
        if (!myFShape.IsNull)
        {
            return myMap.Find(myFShape);
        }
        return myGenerated; // empty list
    }

    /// <summary>The shapes created at the top of the created form; it may be empty (frozen).</summary>
    public ISchematiqList<Shape> LastShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalLastShape());
    }

    /// <summary><see cref="LastShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalLastShape()
    {
        if (!myLShape.IsNull)
        {
            return myMap.Find(myLShape);
        }
        return myGenerated; // empty list
    }

    /// <summary>The faces on which drafts can be made afterwards (frozen).</summary>
    public ISchematiqList<Shape> FacesForDraft()
    {
        BeforePublishingLists();
        return PublishedList(myFacesForDraft);
    }

    /// <summary><see cref="FacesForDraft"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalFacesForDraft() => myFacesForDraft;

    /// <summary>The limiting and glueing edges generated by the feature (frozen).</summary>
    public ISchematiqList<Shape> NewEdges()
    {
        BeforePublishingLists();
        return PublishedList(myNewEdges);
    }

    /// <summary><see cref="NewEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalNewEdges() => myNewEdges;

    /// <summary>The tangent edges among <see cref="NewEdges"/> (frozen).</summary>
    public ISchematiqList<Shape> TgtEdges()
    {
        BeforePublishingLists();
        return PublishedList(myTgtEdges);
    }

    /// <summary><see cref="TgtEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalTgtEdges() => myTgtEdges;

    /// <summary>The status of the last construction.</summary>
    public BRepFeat_StatusError CurrentStatusError() => myStatusError;

    /// <summary>A proofing point somewhat inside the material side of the edge <paramref name="e"/>.</summary>
    protected Point3D CheckPoint(in Edge e, double bnd, PlaneSurface Pln)
    {
        // Vector product : normal to plane X direction Wire
        // -> gives the material side
        // Proofing point somewhat inside the material side
        Curve3D cc = Tool.Curve(e, out double f, out double l)!;

        double par = (f + l) / 2.0;
        cc.D1(par, out Point3D pp, out Vector3D tgt);

        if (e.Orientation() == Reversed)
        {
            tgt.Reverse();
        }

        Vector3D D = -tgt.Crossed(new Vector3D(Pln.Pln().Position().Direction())) / 10.0;
        pp.Translate(D);

        return pp;
    }

    /// <summary>The normal of the face <paramref name="F"/> at <paramref name="P"/>, reversed for a forward face.</summary>
    protected Direction3D Normal(in Face F, in Point3D P)
    {
        double U, V;
        FaceSurfaceAdaptor AS = new(F, true);

        switch (AS.GetType())
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(AS.Plane(), P, out U, out V);
                break;

            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(AS.Cylinder(), P, out U, out V);
                break;

            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(AS.Cone(), P, out U, out V);
                break;

            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(AS.Torus(), P, out U, out V);
                break;

            default:
            {
                return new Direction3D(Direction3D.D.X);
            }
        }

        AS.D1(U, V, out Point3D pt, out Vector3D D1U, out Vector3D D1V);
        Direction3D N = new();
        global::Kerf.Engine.Geometry.CSLib.Normal(D1U, D1V, global::Kerf.Engine.Foundation.Precision.Confusion(), out DerivativeStatus St, ref N);
        if (F.Orientation() == Forward)
        {
            N.Reverse();
        }
        return N;
    }

    /// <summary>The parameter of the point <paramref name="P"/> on the conic <paramref name="C"/> (0 for other curves).</summary>
    public static double IntPar(Curve3D? C, in Point3D P)
    {
        if (C is null)
        {
            return 0.0;
        }

        CurveAdaptor AC = new(C);
        double U;

        switch (AC.GetType())
        {
            case Line:
                U = global::Kerf.Engine.Geometry.ElCLib.Parameter(AC.Line(), P);
                break;

            case Circle:
                U = global::Kerf.Engine.Geometry.ElCLib.Parameter(AC.Circle(), P);
                break;

            case Ellipse:
                U = global::Kerf.Engine.Geometry.ElCLib.Parameter(AC.Ellipse(), P);
                break;

            case Hyperbola:
                U = global::Kerf.Engine.Geometry.ElCLib.Parameter(AC.Hyperbola(), P);
                break;

            case Parabola:
                U = global::Kerf.Engine.Geometry.ElCLib.Parameter(AC.Parabola(), P);
                break;

            default:
                U = 0.0;
                break;
        }

        return U;
    }

    /// <summary>Extends the edge <paramref name="e"/> at its start (or its end) by a tenth of <paramref name="bnd"/>.</summary>
    protected void EdgeExtention(ref Edge e, double bnd, bool FirstLast)
    {
        Curve3D cu = Tool.Curve(e, out double f, out double l)!;
        BoundedCurve3D C = new TrimmedCurve3D(cu, f, l);

        Edge E;

        if (cu.GetType() == typeof(LineCurve3D) || cu.GetType() == typeof(CircleCurve3D)
            || cu.GetType() == typeof(EllipseCurve3D) || cu.GetType() == typeof(HyperbolaCurve3D)
            || cu.GetType() == typeof(ParabolaCurve3D))
        {
            if (FirstLast)
            {
                BRepLib_MakeEdge Edg = new(cu, f - bnd / 10.0, l);
                E = ShapeCasts.Edge(Edg.InternalShape());
            }
            else
            {
                BRepLib_MakeEdge Edg = new(cu, f, l + bnd / 10.0);
                E = ShapeCasts.Edge(Edg.InternalShape());
            }
        }
        else
        {
            LineCurve3D ln;
            Point3D Pt;
            if (FirstLast)
            {
                C.D1(f, out Point3D pnt, out Vector3D vct);
                ln = new LineCurve3D(pnt, new Direction3D(-vct));
                ln.D0(bnd / 1000.0, out Pt);
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref C, Pt, (int)G1, false);
                BRepLib_MakeEdge Edg = new(C, Pt, Tool.Pnt(Exploration.LastVertex(e, true)));
                E = ShapeCasts.Edge(Edg.InternalShape());
            }
            else
            {
                C.D1(l, out Point3D pnt, out Vector3D vct);
                ln = new LineCurve3D(pnt, new Direction3D(vct));
                ln.D0(bnd / 1000.0, out Pt);
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref C, Pt, (int)G1, true);
                BRepLib_MakeEdge Edg = new(C, Tool.Pnt(Exploration.FirstVertex(e, true)), Pt);
                E = ShapeCasts.Edge(Edg.InternalShape());
            }
        }
        e = E;
    }

    /// <summary>
    /// The face of <paramref name="faces"/> first met by lines through the point of <paramref name="cc"/> at
    /// <paramref name="par"/>, turned around the normal of <paramref name="Pln"/>.
    /// </summary>
    public static Face ChoiceOfFaces(NCollection_List<Shape> faces, Curve3D cc, double par, double bnd,
                                            PlaneSurface Pln)
    {
        Face FFF = new();

        cc.D1(par, out Point3D pp, out Vector3D tgt);

        LineCurve3D l1 = new(pp, new Direction3D(tgt));

        NCollection_Sequence<Curve3D?> scur = new();
        int Counter = 0;

        Axis3D Axe = new(pp, Pln.Position().Direction());
        for (int i = 1; i <= 8; i++)
        {
            Curve3D L = (Curve3D)l1.Rotated(Axe, i * M_PI / 9.0);
            scur.Append(L);
            Counter++;
        }

        NCollection_List<Shape>.Iterator it = new();
        it.Initialize(faces);
        double Par = RealLast();
        for (; it.More(); it.Next())
        {
            Face f = ShapeCasts.Face(it.Value());
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
    protected double HeightMax(in Shape theSbase, in Shape theSUntil, ref Point3D p1, ref Point3D p2)
    {
        Box Box = new();
        BRepBndLib.Add(theSbase, ref Box);
        if (!theSUntil.IsNull)
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
    protected bool ExtremeFaces(bool RevolRib, double bnd, PlaneSurface Pln, ref Edge FirstEdge,
                                ref Edge LastEdge, ref Face FirstFace, ref Face LastFace,
                                ref Vertex FirstVertex, ref Vertex LastVertex, ref bool OnFirstFace,
                                ref bool OnLastFace, ref bool PtOnFirstEdge, ref bool PtOnLastEdge,
                                ref Edge OnFirstEdge, ref Edge OnLastEdge)
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
        BrepBuilder B = new();
        Explorer ex1 = new();

        bool FirstOK = false, LastOK = false;

        int NumberOfEdges = 0;
        Explorer exp = new(myWire, ShapeType.Edge);

        for (; exp.More(); exp.Next())
        {
            NumberOfEdges++;
        }

        // ---the wire includes only one edge
        if (NumberOfEdges == 1)
        {
            exp.ReInit();
            Point3D firstpoint = new(), lastpoint = new();

            // Points limit the unique edge
            Edge E = ShapeCasts.Edge(exp.Current());
            Curve3D cc = Tool.Curve(E, out double f, out double l)!;
            Point3D p1 = Tool.Pnt(Exploration.FirstVertex(E, true));
            Point3D p2 = Tool.Pnt(Exploration.LastVertex(E, true));

            double FirstPar = f;
            double LastPar = l;

            // ---Find if 2 points limiting the unique edge of the wire
            //    are on an edge or a vertex of the base shape
            bool PtOnFirstVertex = false;
            bool PtOnLastVertex = false;
            Vertex OnFirstVertex = new(), OnLastVertex = new();
            PtOnEdgeVertex(RevolRib, mySbase, p1, FirstVertex, LastVertex, ref PtOnFirstEdge, ref OnFirstEdge,
                           ref PtOnFirstVertex, ref OnFirstVertex);
            PtOnEdgeVertex(RevolRib, mySbase, p2, FirstVertex, LastVertex, ref PtOnLastEdge, ref OnLastEdge,
                           ref PtOnLastVertex, ref OnLastVertex);

            NCollection_Map<Shape> Map = new(ShapeHasher.Instance);

            if (PtOnFirstEdge)
            {
                if (!PtOnFirstVertex)
                {
                    // Find FirstFace : face of the base shape containing OnFirstEdge
                    //                  meeting ChoiceOfFaces
                    Explorer ex4 = new(), ex5 = new();
                    ex4.Init(mySbase, ShapeType.Face);
                    NCollection_List<Shape> faces = new();
                    faces.Clear();
                    Map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        Face fx = ShapeCasts.Face(ex4.Current());
                        if (!Map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), ShapeType.Edge);
                        for (; ex5.More(); ex5.Next())
                        {
                            Edge ee = ShapeCasts.Edge(ex5.Current());
                            if (ee.IsSame(OnFirstEdge))
                            {
                                faces.Append(fx);
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        Face FFF = ChoiceOfFaces(faces, cc, FirstPar + bnd / 50.0, bnd / 50.0, Pln);
                        if (!FFF.IsNull)
                        {
                            FirstFace = FFF;
                        }
                    }
                }
                else if (PtOnFirstVertex)
                {
                    // Find FirstFace : face of the base shape containing OnFirstVertex
                    //                  meeting ChoiceOfFaces
                    Explorer ex4 = new(), ex5 = new();
                    ex4.Init(mySbase, ShapeType.Face);
                    NCollection_List<Shape> faces = new();
                    faces.Clear();
                    Map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        Face fx = ShapeCasts.Face(ex4.Current());
                        if (!Map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), ShapeType.Vertex);
                        for (; ex5.More(); ex5.Next())
                        {
                            Vertex vv = ShapeCasts.Vertex(ex5.Current());
                            if (vv.IsSame(OnFirstVertex))
                            {
                                faces.Append(fx);
                                break;
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        Face FFF = ChoiceOfFaces(faces, cc, FirstPar + bnd / 50.0, bnd / 50.0, Pln);
                        if (!FFF.IsNull)
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
                    Explorer ex4 = new(), ex5 = new();
                    ex4.Init(mySbase, ShapeType.Face);
                    NCollection_List<Shape> faces = new();
                    faces.Clear();
                    Map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        Face fx = ShapeCasts.Face(ex4.Current());
                        if (!Map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), ShapeType.Edge);
                        for (; ex5.More(); ex5.Next())
                        {
                            Edge ee = ShapeCasts.Edge(ex5.Current());
                            if (ee.IsSame(OnLastEdge))
                            {
                                faces.Append(fx);
                                break;
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        Face FFF = ChoiceOfFaces(faces, cc, LastPar - bnd / 50.0, bnd / 50.0, Pln);
                        if (!FFF.IsNull)
                        {
                            LastFace = FFF;
                        }
                    }
                }
                else if (PtOnLastEdge && PtOnLastVertex)
                {
                    // Find LastFace : face of the base shape containing OnLastVertex
                    //                 meeting ChoiceOfFaces
                    Explorer ex4 = new(), ex5 = new();
                    ex4.Init(mySbase, ShapeType.Face);
                    NCollection_List<Shape> faces = new();
                    faces.Clear();
                    Map.Clear();
                    for (; ex4.More(); ex4.Next())
                    {
                        Face fx = ShapeCasts.Face(ex4.Current());
                        if (!Map.Add(fx))
                        {
                            continue;
                        }
                        ex5.Init(ex4.Current(), ShapeType.Vertex);
                        for (; ex5.More(); ex5.Next())
                        {
                            Vertex vv = ShapeCasts.Vertex(ex5.Current());
                            if (vv.IsSame(OnLastVertex))
                            {
                                faces.Append(fx);
                                break;
                            }
                        }
                    }
                    if (!faces.IsEmpty())
                    {
                        Face FFF = ChoiceOfFaces(faces, cc, LastPar - bnd / 50.0, bnd / 50.0, Pln);
                        if (!FFF.IsNull)
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

            if (!FirstFace.IsNull && !LastFace.IsNull)
            {
                return true;
            }

            //--- FirstFace or LastFace was not found
            LocOpe_CSIntersector ASI = new(mySbase);
            NCollection_Sequence<Curve3D?> scur = new();
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

                if (FirstFace.IsNull)
                {
                    FirstFace = ASI.Point(1, firstindex).Face();
                    cc.D0(firstpar, out firstpoint);
                    BRepLib_MakeVertex v1 = new(firstpoint);
                    FirstVertex = ShapeCasts.Vertex(v1.InternalShape());
                    FirstEdge = E;
                }

                if (LastFace.IsNull)
                {
                    LastFace = ASI.Point(1, lastindex).Face();
                    cc.D0(lastpar, out lastpoint);
                    BRepLib_MakeVertex v2 = new(lastpoint);
                    LastVertex = ShapeCasts.Vertex(v2.InternalShape());
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
                OnFirstFace = p1.Distance(firstpoint) <= global::Kerf.Engine.Foundation.Precision.Confusion();
            }
            if (!OnLastFace)
            {
                OnLastFace = p2.Distance(lastpoint) <= global::Kerf.Engine.Foundation.Precision.Confusion();
            }

            Data = !(FirstFace.IsNull || LastFace.IsNull);
            return Data;
        }

        // ---The wire consists of several edges
        else
        {
            WireExplorer ex = new(myWire);
            for (; ex.More(); ex.Next())
            {
                Edge E = ShapeCasts.Edge(ex.Current());
                Curve3D Cur = Tool.Curve(E, out double f, out double l)!;
                f = f - bnd / 10000;
                l = l + bnd / 10000;
                TrimmedCurve3D curve;
                curve = new TrimmedCurve3D(Cur, f, l, true);
                Point3D P2 = Tool.Pnt(Exploration.LastVertex(E, true));
                ex1.Init(mySbase, ShapeType.Face);
                Vertex theVertex = new();
                Edge theEdge = new();
                Face theFace = new();
                bool PtOnEdge = false;
                bool PtOnVertex = false;
                Edge OnEdge = new();
                Vertex OnVertex = new();
                double intpar;
                for (; ex1.More(); ex1.Next())
                {
                    Face aCurFace = ShapeCasts.Face(ex1.Current());
                    CurveAdaptor aGAC = new(curve);
                    inter.Init(aCurFace, aGAC, Tool.Tolerance(aCurFace));
                    if (!inter.More())
                    {
                        continue;
                    }
                    for (; inter.More(); inter.Next())
                    {
                        Point3D thePoint = inter.Pnt();
                        if (!FirstVertex.IsNull)
                        {
                            Point3D point = Tool.Pnt(FirstVertex);
                            if (point.Distance(thePoint) <= Tool.Tolerance(aCurFace))
                            {
                                continue;
                            }
                        }
                        intpar = IntPar(curve, thePoint);
                        theEdge = E;
                        theFace = aCurFace;
                        B.MakeVertex(ref theVertex, thePoint, global::Kerf.Engine.Foundation.Precision.Confusion());
                        if (!FirstOK)
                        {
                            if (thePoint.Distance(P2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                continue;
                            }
                        }

                        // ---Find thepoint on an edge or a vertex of face f
                        PtOnEdgeVertex(RevolRib, aCurFace, thePoint, FirstVertex, LastVertex, ref PtOnEdge, ref OnEdge,
                                       ref PtOnVertex, ref OnVertex);

                        if (FirstEdge.IsNull && !theEdge.IsNull && !theFace.IsNull && !theVertex.IsNull)
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
                                NCollection_List<Shape> faces = new();
                                faces.Clear();
                                faces.Append(FirstFace);
                                Explorer ex2 = new();
                                ex2.Init(mySbase, ShapeType.Face);
                                for (; ex2.More(); ex2.Next())
                                {
                                    Face fx = ShapeCasts.Face(ex2.Current());
                                    Explorer ex3 = new();
                                    ex3.Init(fx, ShapeType.Edge);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        Edge e = ShapeCasts.Edge(ex3.Current());
                                        if (e.IsSame(OnEdge) && !fx.IsSame(FirstFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                Face FFF = ChoiceOfFaces(faces, curve, intpar + bnd / 10.0, bnd / 10.0, Pln);
                                if (!FFF.IsNull)
                                {
                                    FirstFace = FFF;
                                }
                            }
                            else if (PtOnEdge && PtOnVertex)
                            {
                                NCollection_List<Shape> faces = new();
                                faces.Clear();
                                faces.Append(FirstFace);
                                Explorer ex2 = new();
                                ex2.Init(mySbase, ShapeType.Face);
                                for (; ex2.More(); ex2.Next())
                                {
                                    Face fx = ShapeCasts.Face(ex2.Current());
                                    Explorer ex3 = new();
                                    ex3.Init(fx, ShapeType.Vertex);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        Vertex v = ShapeCasts.Vertex(ex3.Current());
                                        if (v.IsSame(OnVertex) && !fx.IsSame(FirstFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                Face FFF = ChoiceOfFaces(faces, curve, intpar + bnd / 10.0, bnd / 10.0, Pln);
                                if (!FFF.IsNull)
                                {
                                    FirstFace = FFF;
                                }
                            }
                            if (!FirstEdge.IsNull && !FirstFace.IsNull && !FirstVertex.IsNull)
                            {
                                FirstOK = true;
                            }
                        }
                        if (LastEdge.IsNull && !theEdge.IsNull && !theFace.IsNull && !theVertex.IsNull
                            && !FirstEdge.IsNull)
                        {
                            LastEdge = theEdge;
                            LastFace = theFace;
                            LastVertex = theVertex;
                            PtOnLastEdge = PtOnEdge;
                            OnLastEdge = OnEdge;
                            if (PtOnEdge && !PtOnVertex)
                            {
                                NCollection_List<Shape> faces = new();
                                faces.Clear();
                                faces.Append(LastFace);
                                Explorer ex2 = new();
                                ex2.Init(mySbase, ShapeType.Face);
                                for (; ex2.More(); ex2.Next())
                                {
                                    Face fx = ShapeCasts.Face(ex2.Current());
                                    Explorer ex3 = new();
                                    ex3.Init(fx, ShapeType.Edge);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        Edge e = ShapeCasts.Edge(ex3.Current());
                                        if (e.IsSame(OnEdge) && !fx.IsSame(LastFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                Face FFF = ChoiceOfFaces(faces, curve, intpar - bnd / 10.0, bnd / 10.0, Pln);
                                if (!FFF.IsNull)
                                {
                                    LastFace = FFF;
                                }
                            }
                            else if (PtOnEdge && PtOnVertex)
                            {
                                NCollection_List<Shape> faces = new();
                                faces.Clear();
                                faces.Append(LastFace);
                                Explorer ex2 = new();
                                ex2.Init(mySbase, ShapeType.Face);
                                for (; ex2.More(); ex2.Next())
                                {
                                    Face fx = ShapeCasts.Face(ex2.Current());
                                    Explorer ex3 = new();
                                    ex3.Init(fx, ShapeType.Vertex);
                                    for (; ex3.More(); ex3.Next())
                                    {
                                        Vertex v = ShapeCasts.Vertex(ex3.Current());
                                        if (v.IsSame(OnVertex) && !fx.IsSame(LastFace))
                                        {
                                            faces.Append(fx);
                                        }
                                    }
                                }
                                Face FFF = ChoiceOfFaces(faces, curve, intpar - bnd / 10.0, bnd / 10.0, Pln);
                                if (!FFF.IsNull)
                                {
                                    LastFace = FFF;
                                }
                            }
                            if (!LastEdge.IsNull && !LastFace.IsNull && !LastVertex.IsNull)
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
                Point3D PP1 = Tool.Pnt(Exploration.FirstVertex(FirstEdge, true));
                Point3D PP2 = Tool.Pnt(Exploration.LastVertex(LastEdge, true));
                Point3D p1 = Tool.Pnt(FirstVertex);
                Point3D p2 = Tool.Pnt(LastVertex);
                if (p1.Distance(PP1) <= Tool.Tolerance(FirstFace))
                {
                    OnFirstFace = true;
                }
                if (p2.Distance(PP2) <= Tool.Tolerance(LastFace))
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
    protected void PtOnEdgeVertex(bool RevolRib, in Shape shape, in Point3D point, in Vertex FirstVertex,
                                  in Vertex LastVertex, ref bool PtOnEdge, ref Edge OnEdge,
                                  ref bool PtOnVertex, ref Vertex OnVertex)
    {
        bool TestOK;

        Explorer EXP = new();
        EXP.Init(shape, ShapeType.Edge);
        NCollection_Map<Shape> Map = new(ShapeHasher.Instance);
        for (; EXP.More(); EXP.Next())
        {
            Edge e = ShapeCasts.Edge(EXP.Current());
            if (!Map.Add(e))
            {
                continue;
            }
            if (!RevolRib)
            {
                if (Tool.Degenerated(e))
                {
                    continue;
                }
            }
            Curve3D ccc = Tool.Curve(e, out double fff, out double lll)!;
            if (!RevolRib)
            {
                ccc = new TrimmedCurve3D(ccc, fff, lll);
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
            if (TestOK && proj.Distance(1) <= Tool.Tolerance(e))
            {
                PtOnEdge = true;
                OnEdge = e;
                Vertex ev1 = Exploration.FirstVertex(e, true);
                Vertex ev2 = Exploration.LastVertex(e, true);
                Point3D ep1 = Tool.Pnt(ev1);
                Point3D ep2 = Tool.Pnt(ev2);
                if (point.Distance(ep1) <= Tool.Tolerance(ev1))
                {
                    PtOnVertex = true;
                    OnVertex = ev1;
                    break;
                }
                else if (point.Distance(ep2) <= Tool.Tolerance(ev1))
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
    protected bool SlidingProfile(ref Face Prof, bool RevolRib, double myTol, ref int Concavite, PlaneSurface myPln,
                                  in Face BndFace, in Point3D CheckPnt, in Face FirstFace,
                                  in Face LastFace, in Vertex FirstVertex, in Vertex LastVertex,
                                  in Edge FirstEdge, in Edge LastEdge)
    {
        bool ProfileOK = true;
        // --case of sliding : construction of the wire of the profile
        // --> 1 part bounding box + 1 part wire
        //   attention to the compatibility of orientations

        Direction3D FN, LN;
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

        LineCurve3D ln1, ln2;
        Point3D Pt = new();

        ln2 = new LineCurve3D(myFirstPnt, FN);
        ln1 = new LineCurve3D(myLastPnt, LN);

        Curve2D ln2d1 = GeomAPI.To2d(ln1, myPln.Pln())!;
        Curve2D ln2d2 = GeomAPI.To2d(ln2, myPln.Pln())!;

        Geom2dAPI_InterCurveCurve inter = new(ln2d1, ln2d2, global::Kerf.Engine.Foundation.Precision.Confusion());

        bool TestOK = true;
        if (RevolRib)
        {
            Direction3D d1, d2;
            d1 = ln1.Position().Direction();
            d2 = ln2.Position().Direction();
            if (d1.IsOpposite(d2, myTol))
            {
                double par1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(ln1.Lin(), myFirstPnt);
                double par2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(ln2.Lin(), myLastPnt);
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
                Point2D P = inter.Point(1);
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
            Edge BndEdge1 = new(), BndEdge2 = new();
            // Points of intersection with the bounding box / Find Profile
            Point3D BndPnt1 = new(), BndPnt2 = new(), LastPnt = new();
            Explorer expl = new();
            expl.Init(BndFace, ShapeType.Wire);
            WireExplorer explo = new();
            Wire BndWire = ShapeCasts.Wire(expl.Current());
            explo.Init(BndWire);
            for (; explo.More(); explo.Next())
            {
                Edge e = ShapeCasts.Edge(explo.Current());
                Curve3D c = Tool.Curve(e, out double first, out double last)!;
                Curve2D c2d = GeomAPI.To2d(c, myPln.Pln())!;
                Geom2dAPI_InterCurveCurve intcln1 = new(ln2d1, c2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (intcln1.NbPoints() > 0)
                {
                    Point2D p2d = intcln1.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out Point3D p);
                    double parl = IntPar(ln1, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        BndEdge1 = e;
                        BndPnt1 = p;
                    }
                }

                Geom2dAPI_InterCurveCurve intcln2 = new(ln2d2, c2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (intcln2.NbPoints() > 0)
                {
                    Point2D p2d = intcln2.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out Point3D p);
                    double parl = IntPar(ln2, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        BndEdge2 = e;
                        BndPnt2 = p;
                    }
                }
                if (!BndEdge1.IsNull && !BndEdge2.IsNull)
                {
                    break;
                }
            }

            if (BndEdge1.IsNull || BndEdge2.IsNull)
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
                    Edge e = ShapeCasts.Edge(explo.Current());
                    if (e.IsSame(BndEdge1))
                    {
                        Point3D pp;
                        pp = Tool.Pnt(Exploration.LastVertex(e, true));
                        if (pp.Distance(BndPnt1) >= Tool.Tolerance(e))
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
                    if (explo.Current().IsNull)
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
                    Edge e = ShapeCasts.Edge(explo.Current());
                    if (!e.IsSame(BndEdge2))
                    {
                        Point3D pp;
                        pp = Tool.Pnt(Exploration.LastVertex(e, true));
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
                        if (explo.Current().IsNull)
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
        WireExplorer EX = new(myWire);

        Curve3D FirstCurve = Tool.Curve(FirstEdge, out double ff, out double ll)!;

        if (!FirstEdge.IsSame(LastEdge))
        {
            Vertex FLVert = Exploration.LastVertex(FirstEdge, true);
            Point3D FLPnt = Tool.Pnt(FLVert);
            BRepLib_MakeEdge ef = new(FirstCurve, myFirstPnt, FLPnt);
            WW.Add(ef.InternalEdge());
            for (; EX.More(); EX.Next())
            {
                Edge E = EX.Current();
                if (E.IsSame(FirstEdge))
                {
                    break;
                }
            }
            EX.Next();
            for (; EX.More(); EX.Next())
            {
                Edge E = EX.Current();
                if (!E.IsSame(LastEdge))
                {
                    WW.Add(E);
                }
                else
                {
                    break;
                }
            }
            Curve3D LastCurve = Tool.Curve(LastEdge, out ff, out ll)!;
            Vertex LFVert = Exploration.FirstVertex(LastEdge, true);
            Point3D LFPnt = Tool.Pnt(LFVert);
            BRepLib_MakeEdge el = new(LastCurve, LFPnt, myLastPnt);
            WW.Add(el.InternalEdge());
        }
        else
        {
            // only one edge : particular processing
            double fpar = IntPar(FirstCurve, myFirstPnt);
            double lpar = IntPar(FirstCurve, myLastPnt);
            Curve3D c;
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
        Face fac = ShapeCasts.Face(f.InternalShape());

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
            BRepTopAdaptor_FClass2d Cl = new(fac, Tool.Tolerance(fac));
            global::Kerf.Engine.Geometry.ElSLib.Parameters(myPln.Pln(), CheckPnt, out double u, out double v);
            Point2D checkpnt2d = new(u, v);
            if (Cl.Perform(checkpnt2d, true) == Out)
            {
                // If face is not the correct part of BndFace take the complementary
                BRepAlgoAPI_Cut c = new(BndFace, fac);
                Explorer exp = new(c.InternalShape(), ShapeType.Wire);
                Wire w = ShapeCasts.Wire(exp.Current());
                BRepLib_MakeFace ffx = new(w);
                Prof = ShapeCasts.Face(ffx.InternalShape());
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
    protected bool NoSlidingProfile(ref Face Prof, bool RevolRib, double myTol, ref int Concavite,
                                    PlaneSurface myPln, double bnd, in Face BndFace, in Point3D CheckPnt,
                                    in Face FirstFace, in Face LastFace, in Vertex FirstVertex,
                                    in Vertex LastVertex, in Edge FirstEdge, in Edge LastEdge,
                                    bool OnFirstFace, bool OnLastFace)
    {
        bool ProfileOK = true;

        Vertex theFV = new();
        theFV.Nullify();
        Point3D theFirstpoint = new();
        Edge theLastEdge = new();
        theLastEdge.Nullify();

        Point3D firstpoint, lastpoint;
        Vector3D firstvect, lastvect;
        Wire w = new();
        BrepBuilder BB = new();
        BB.MakeWire(ref w);
        Edge FalseFirstEdge = new(), FalseLastEdge = new(), FalseOnlyOne = new();

        Curve3D FirstCurve = Tool.Curve(FirstEdge, out double f1, out double l1)!;
        Curve3D LastCurve = Tool.Curve(LastEdge, out double f2, out double l2)!;

        LineCurve3D firstln, lastln;
        FirstCurve.D1(f1, out firstpoint, out firstvect);
        lastln = new LineCurve3D(firstpoint, new Direction3D(-firstvect));
        LastCurve.D1(l2, out lastpoint, out lastvect);
        firstln = new LineCurve3D(lastpoint, new Direction3D(lastvect));

        Point3D Pt = new();

        Curve2D ln2d1 = GeomAPI.To2d(firstln, myPln.Pln())!;
        Curve2D ln2d2 = GeomAPI.To2d(lastln, myPln.Pln())!;

        Geom2dAPI_InterCurveCurve inter = new(ln2d1, ln2d2, global::Kerf.Engine.Foundation.Precision.Confusion());

        bool TestOK = true;
        if (RevolRib)
        {
            Direction3D d1, d2;
            d1 = firstln.Position().Direction();
            d2 = lastln.Position().Direction();
            if (d1.IsOpposite(d2, myTol))
            {
                double par1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(firstln.Lin(), myFirstPnt);
                double par2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lastln.Lin(), myLastPnt);
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
                Point2D P = inter.Point(1);
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
                Vertex vv1 = Exploration.FirstVertex(FalseFirstEdge, true);
                firstpoint = Tool.Pnt(vv1);
                Curve3D cc = Tool.Curve(FalseFirstEdge, out double f, out double l)!;
                cc.D1(f, out firstpoint, out firstvect);
                lastln = new LineCurve3D(firstpoint, new Direction3D(-firstvect));
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
                    if (FalseOnlyOne.IsNull)
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
                Vertex vv2 = Exploration.LastVertex(FalseLastEdge, true);
                lastpoint = Tool.Pnt(vv2);
                Curve3D cc = Tool.Curve(FalseLastEdge, out double f, out double l)!;
                cc.D1(l, out lastpoint, out lastvect);
                lastpoint = Tool.Pnt(vv2);
                firstln = new LineCurve3D(lastpoint, new Direction3D(lastvect));
                ln2d1 = GeomAPI.To2d(firstln, myPln.Pln())!;
            }

            Edge BndEdge1 = new(), BndEdge2 = new();
            Point3D BndPnt1 = new(), BndPnt2 = new(), LastPnt = new();
            Explorer expl = new();
            expl.Init(BndFace, ShapeType.Wire);
            WireExplorer explo = new();
            Wire BndWire = ShapeCasts.Wire(expl.Current());
            explo.Init(BndWire);
            for (; explo.More(); explo.Next())
            {
                Edge e = ShapeCasts.Edge(explo.Current());
                Curve3D c = Tool.Curve(e, out double first, out double last)!;
                Curve2D c2d = GeomAPI.To2d(c, myPln.Pln())!;
                Geom2dAPI_InterCurveCurve intcln1 = new(ln2d1, c2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (intcln1.NbPoints() > 0)
                {
                    Point2D p2d = intcln1.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out Point3D p);
                    double parl = IntPar(firstln, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        BndEdge1 = e;
                        BndPnt1 = p;
                    }
                }

                Geom2dAPI_InterCurveCurve intcln2 = new(ln2d2, c2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (intcln2.NbPoints() > 0)
                {
                    Point2D p2d = intcln2.Point(1);
                    myPln.D0(p2d.X(), p2d.Y(), out Point3D p);
                    double parl = IntPar(lastln, p);
                    double parc = IntPar(c, p);
                    if (parc >= first && parc <= last && parl >= 0)
                    {
                        BndEdge2 = e;
                        BndPnt2 = p;
                    }
                }
                if (!BndEdge1.IsNull && !BndEdge2.IsNull)
                {
                    break;
                }
            }

            if (BndEdge1.IsNull || BndEdge2.IsNull)
            {
                ProfileOK = false;
                return ProfileOK;
            }

            Edge ee1;
            if (theLastEdge.IsNull)
            {
                BRepLib_MakeEdge e1 = new(lastpoint, BndPnt1);
                ee1 = ShapeCasts.Edge(e1.InternalShape());
            }
            else
            {
                Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                BRepLib_MakeVertex v2 = new(BndPnt1);
                BRepLib_MakeEdge e1 = new(v1, v2.InternalVertex());
                ee1 = ShapeCasts.Edge(e1.InternalShape());
            }
            BB.Add(w, ee1);
            theLastEdge = ee1;
            if (theFV.IsNull)
            {
                theFV = Exploration.FirstVertex(ee1, true);
                theFirstpoint = Tool.Pnt(theFV);
            }

            if (BndEdge1.IsSame(BndEdge2))
            {
                Edge ee2, ee3;
                if (theLastEdge.IsNull)
                {
                    BRepLib_MakeEdge e2 = new(BndPnt1, BndPnt2);
                    ee2 = ShapeCasts.Edge(e2.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    BRepLib_MakeVertex v2 = new(BndPnt2);
                    BRepLib_MakeEdge e2 = new(v1, v2.InternalVertex());
                    ee2 = ShapeCasts.Edge(e2.InternalShape());
                }
                BB.Add(w, ee2);
                theLastEdge = ee2;
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(ee2, true);
                    theFirstpoint = Tool.Pnt(theFV);
                }
                if (theLastEdge.IsNull)
                {
                    BRepLib_MakeEdge e3 = new(BndPnt2, firstpoint);
                    ee3 = ShapeCasts.Edge(e3.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    BRepLib_MakeVertex v2 = new(firstpoint);
                    BRepLib_MakeEdge e3 = new(v1, v2.InternalVertex());
                    ee3 = ShapeCasts.Edge(e3.InternalShape());
                }
                BB.Add(w, ee3);
                theLastEdge = ee3;
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(ee3, true);
                    theFirstpoint = Tool.Pnt(theFV);
                }
            }
            else
            {
                explo.Init(BndWire);
                for (; explo.More(); explo.Next())
                {
                    Edge e = ShapeCasts.Edge(explo.Current());
                    if (e.IsSame(BndEdge1))
                    {
                        Point3D pp;
                        pp = Tool.Pnt(Exploration.LastVertex(e, true));
                        if (pp.Distance(BndPnt1) > Tool.Tolerance(e))
                        {
                            LastPnt = pp;
                        }
                        Edge eee;
                        if (theLastEdge.IsNull)
                        {
                            BRepLib_MakeEdge e2 = new(BndPnt1, LastPnt);
                            eee = ShapeCasts.Edge(e2.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                            BRepLib_MakeVertex v2 = new(LastPnt);
                            BRepLib_MakeEdge e2 = new(v1, v2.InternalVertex());
                            eee = ShapeCasts.Edge(e2.InternalShape());
                        }
                        BB.Add(w, eee);
                        theLastEdge = eee;
                        if (theFV.IsNull)
                        {
                            theFV = Exploration.FirstVertex(eee, true);
                            theFirstpoint = Tool.Pnt(theFV);
                        }
                        break;
                    }
                }

                if (explo.More())
                {
                    explo.Next();
                    if (explo.Current().IsNull)
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
                    Edge e = ShapeCasts.Edge(explo.Current());
                    if (!e.IsSame(BndEdge2))
                    {
                        Point3D pp;
                        pp = Tool.Pnt(Exploration.LastVertex(e, true));
                        Edge eee1;
                        if (theLastEdge.IsNull)
                        {
                            BRepLib_MakeEdge ee = new(LastPnt, pp);
                            eee1 = ShapeCasts.Edge(ee.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                            BRepLib_MakeVertex v2 = new(pp);
                            BRepLib_MakeEdge ee = new(v1, v2.InternalVertex());
                            eee1 = ShapeCasts.Edge(ee.InternalShape());
                        }
                        BB.Add(w, eee1);
                        theLastEdge = eee1;
                        if (theFV.IsNull)
                        {
                            theFV = Exploration.FirstVertex(eee1, true);
                            theFirstpoint = Tool.Pnt(theFV);
                        }
                        LastPnt = pp;
                    }
                    else
                    {
                        Fin = true;
                        Edge eee2;
                        if (theLastEdge.IsNull)
                        {
                            BRepLib_MakeEdge ee = new(LastPnt, BndPnt2);
                            eee2 = ShapeCasts.Edge(ee.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                            BRepLib_MakeVertex v2 = new(BndPnt2);
                            BRepLib_MakeEdge ee = new(v1, v2.InternalVertex());
                            eee2 = ShapeCasts.Edge(ee.InternalShape());
                        }
                        BB.Add(w, eee2);
                        theLastEdge = eee2;
                        if (theFV.IsNull)
                        {
                            theFV = Exploration.FirstVertex(eee2, true);
                            theFirstpoint = Tool.Pnt(theFV);
                        }
                        LastPnt = BndPnt2;
                    }
                    if (explo.More())
                    {
                        explo.Next();
                        if (explo.Current().IsNull)
                        {
                            explo.Init(BndWire);
                        }
                    }
                    else
                    {
                        explo.Init(BndWire);
                    }
                }

                Edge eee3;
                if (theLastEdge.IsNull)
                {
                    BRepLib_MakeEdge e3 = new(BndPnt2, firstpoint);
                    eee3 = ShapeCasts.Edge(e3.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    BRepLib_MakeVertex v2 = new(firstpoint);
                    BRepLib_MakeEdge e3 = new(v1, v2.InternalVertex());
                    eee3 = ShapeCasts.Edge(e3.InternalShape());
                }
                BB.Add(w, eee3);
                theLastEdge = eee3;
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(eee3, true);
                    theFirstpoint = Tool.Pnt(theFV);
                }
            }
        }

        if (Concavite == 1)
        {
            Edge eee4;
            if (theLastEdge.IsNull)
            {
                BRepLib_MakeEdge e = new(Pt, firstpoint);
                eee4 = ShapeCasts.Edge(e.InternalShape());
            }
            else
            {
                Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                BRepLib_MakeVertex v2 = new(firstpoint);
                BRepLib_MakeEdge e = new(v1, v2.InternalVertex());
                eee4 = ShapeCasts.Edge(e.InternalShape());
            }
            BB.Add(w, eee4);
            if (theFV.IsNull)
            {
                theFV = Exploration.FirstVertex(eee4, true);
                theFirstpoint = Tool.Pnt(theFV);
            }
            theLastEdge = eee4;
        }

        if (FirstEdge.IsSame(LastEdge))
        {
            if (!myLFMap.IsBound(FirstEdge))
            {
                NCollection_List<Shape> thelist = new();
                myLFMap.Bind(FirstEdge, thelist);
            }
            if (OnFirstFace || OnLastFace)
            {
                Edge theEdge;
                Curve3D cc = Tool.Curve(FalseOnlyOne, out double f, out double l)!;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(FalseOnlyOne, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(FalseOnlyOne, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.FirstVertex(FalseOnlyOne, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(FalseOnlyOne, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(FalseOnlyOne, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                myLFMap[FirstEdge].Append(theEdge);
                BB.Add(w, theEdge);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }
            else
            {
                Curve3D cc = Tool.Curve(FirstEdge, out double f, out double l)!;
                Edge theEdge;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2;
                    // Attention case Wire Reversed -> LastVertex without true
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(FirstEdge));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(FirstEdge);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.FirstVertex(FirstEdge, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(FirstEdge, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(FirstEdge, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                myLFMap[FirstEdge].Append(theEdge);
                BB.Add(w, theEdge);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }
        }
        else
        {
            if (!myLFMap.IsBound(FirstEdge))
            {
                NCollection_List<Shape> thelist1 = new();
                myLFMap.Bind(FirstEdge, thelist1);
            }
            if (!OnFirstFace)
            {
                Edge theEdge;
                Curve3D cc = Tool.Curve(FirstEdge, out double f, out double l)!;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2 = Exploration.LastVertex(FirstEdge, true);
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    theEdge = FirstEdge;
                }
                myLFMap[FirstEdge].Append(theEdge);
                BB.Add(w, theEdge);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }
            else
            {
                Edge theEdge;
                Curve3D cc = Tool.Curve(FalseFirstEdge, out double f, out double l)!;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2 = Exploration.LastVertex(FalseFirstEdge, true);
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    theEdge = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    theEdge = FalseFirstEdge;
                }
                myLFMap[FirstEdge].Append(theEdge);
                BB.Add(w, theEdge);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(theEdge, true);
                }
                theLastEdge = theEdge;
            }

            WireExplorer ex = new(myWire);
            for (; ex.More(); ex.Next())
            {
                Edge E = ex.Current();
                if (E.IsSame(FirstEdge))
                {
                    break;
                }
            }

            ex.Next();

            for (; ex.More(); ex.Next())
            {
                Edge E = ex.Current();
                if (!E.IsSame(LastEdge))
                {
                    if (!myLFMap.IsBound(E))
                    {
                        NCollection_List<Shape> thelist2 = new();
                        myLFMap.Bind(E, thelist2);
                    }
                    Edge eee;
                    Curve3D cc = Tool.Curve(E, out double f, out double l)!;
                    if (!theLastEdge.IsNull)
                    {
                        Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                        Vertex v2 = Exploration.LastVertex(E, true);
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    else
                    {
                        eee = E;
                    }
                    myLFMap[E].Append(eee);
                    BB.Add(w, eee);
                    if (theFV.IsNull)
                    {
                        theFV = Exploration.FirstVertex(eee, true);
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
                    Edge edg = ShapeCasts.Edge(ex.Current());
                    if (!myLFMap.IsBound(edg))
                    {
                        NCollection_List<Shape> thelist3 = new();
                        myLFMap.Bind(edg, thelist3);
                    }
                    Edge eee;
                    Curve3D cc = Tool.Curve(edg, out double f, out double l)!;
                    if (!theLastEdge.IsNull)
                    {
                        Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                        Vertex v2;
                        Point3D pp = Tool.Pnt(Exploration.LastVertex(edg, true));
                        if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = Exploration.LastVertex(edg, true);
                        }
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    else
                    {
                        Vertex v1 = Exploration.FirstVertex(edg, true);
                        Vertex v2;
                        Point3D pp = Tool.Pnt(Exploration.LastVertex(edg, true));
                        if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = Exploration.LastVertex(edg, true);
                        }
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    myLFMap[edg].Append(eee);
                    BB.Add(w, eee);
                    if (theFV.IsNull)
                    {
                        theFV = Exploration.FirstVertex(eee, true);
                    }
                    theLastEdge = eee;
                }
                else
                {
                    Edge eee;
                    if (!myLFMap.IsBound(LastEdge))
                    {
                        NCollection_List<Shape> thelist4 = new();
                        myLFMap.Bind(LastEdge, thelist4);
                    }
                    Curve3D cc = Tool.Curve(FalseOnlyOne, out double f, out double l)!;
                    if (!theLastEdge.IsNull)
                    {
                        Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                        Vertex v2;
                        Point3D pp = Tool.Pnt(Exploration.LastVertex(FalseOnlyOne, true));
                        if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = Exploration.LastVertex(FalseOnlyOne, true);
                        }
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    else
                    {
                        Vertex v1 = Exploration.FirstVertex(FalseOnlyOne, true);
                        Vertex v2;
                        Point3D pp = Tool.Pnt(Exploration.LastVertex(FalseOnlyOne, true));
                        if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                        {
                            v2 = theFV;
                        }
                        else
                        {
                            v2 = Exploration.LastVertex(FalseOnlyOne, true);
                        }
                        BRepLib_MakeEdge e = new(cc, v1, v2);
                        eee = ShapeCasts.Edge(e.InternalShape());
                    }
                    myLFMap[LastEdge].Append(eee);
                    BB.Add(w, eee);
                    if (theFV.IsNull)
                    {
                        theFV = Exploration.FirstVertex(eee, true);
                    }
                    theLastEdge = eee;
                }
            }
            else
            {
                Edge eee;
                if (!myLFMap.IsBound(LastEdge))
                {
                    NCollection_List<Shape> thelist5 = new();
                    myLFMap.Bind(LastEdge, thelist5);
                }
                Curve3D cc = Tool.Curve(FalseLastEdge, out double f, out double l)!;
                if (!theLastEdge.IsNull)
                {
                    Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(FalseLastEdge, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(FalseLastEdge, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    eee = ShapeCasts.Edge(e.InternalShape());
                }
                else
                {
                    Vertex v1 = Exploration.FirstVertex(FalseLastEdge, true);
                    Vertex v2;
                    Point3D pp = Tool.Pnt(Exploration.LastVertex(FalseLastEdge, true));
                    if (!theFV.IsNull && theFirstpoint.Distance(pp) <= myTol)
                    {
                        v2 = theFV;
                    }
                    else
                    {
                        v2 = Exploration.LastVertex(FalseLastEdge, true);
                    }
                    BRepLib_MakeEdge e = new(cc, v1, v2);
                    eee = ShapeCasts.Edge(e.InternalShape());
                }
                myLFMap[LastEdge].Append(eee);
                BB.Add(w, eee);
                if (theFV.IsNull)
                {
                    theFV = Exploration.FirstVertex(eee, true);
                }
                theLastEdge = eee;
            }
        }

        if (Concavite == 1)
        {
            Edge eef;
            if (theLastEdge.IsNull)
            {
                BRepLib_MakeEdge ef = new(lastpoint, Pt);
                eef = ShapeCasts.Edge(ef.InternalShape());
            }
            else
            {
                Vertex v1 = Exploration.LastVertex(theLastEdge, true);
                BRepLib_MakeVertex vv = new(Pt);
                Vertex v2 = ShapeCasts.Vertex(vv.InternalShape());
                if (!theFV.IsNull && Pt.Distance(theFirstpoint) <= myTol)
                {
                    v2 = theFV;
                }
                BRepLib_MakeEdge ef = new(v1, v2);
                eef = ShapeCasts.Edge(ef.InternalShape());
            }
            BB.Add(w, eef);
            if (theFV.IsNull)
            {
                theFV = Exploration.FirstVertex(eef, true);
            }
            theLastEdge = eef;
        }

        if (Concavite == 2)
        {
            BRepLib_MakeEdge ee = new(lastpoint, firstpoint);
            Edge e = ee.InternalEdge();
            BB.Add(w, e);
        }

        w.Closed(Tool.IsClosed(w));
        BRepLib_MakeFace fa = new(myPln.Pln(), w, true);
        Face fac = ShapeCasts.Face(fa.InternalShape());

        if (!BRepAlgo.IsValid(fac))
        {
            ProfileOK = false;
            return ProfileOK;
        }

        if (Concavite == 3)
        {
            BRepTopAdaptor_FClass2d Cl = new(fac, Tool.Tolerance(fac));
            global::Kerf.Engine.Geometry.ElSLib.Parameters(myPln.Pln(), CheckPnt, out double u, out double v);
            Point2D checkpnt2d = new(u, v);
            if (Cl.Perform(checkpnt2d, true) == Out)
            {
                BRepAlgoAPI_Cut c = new(BndFace, fac);
                Explorer exp = new(c.InternalShape(), ShapeType.Wire);
                UpdateDescendants(c, c.InternalShape(), false);
                Wire ww = ShapeCasts.Wire(exp.Current());
                BRepLib_MakeFace ff = new(ww);
                Prof = ShapeCasts.Face(ff.InternalShape());
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

    protected void UpdateDescendants(BRepAlgoAPI_BooleanOperation aBOP, in Shape S, bool SkipFace = false)
    {
        NCollection_List<Shape>.Iterator it = new(), it2 = new();
        Explorer exp = new();

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in myMap)
        {
            Shape orig = itdm.Key;
            if (SkipFace && orig.ShapeType() == ShapeType.Face)
            {
                continue;
            }
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 1054)
            NCollection_IndexedMap<Shape> newdsc = new(ShapeHasher.Instance);

            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                Shape sh = it.Value();
                if (sh.ShapeType() != ShapeType.Face)
                {
                    continue;
                }
                Face fdsc = ShapeCasts.Face(it.Value());
                for (exp.Init(S, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(fdsc))
                    { // preserved
                        newdsc.Add(fdsc);
                        break;
                    }
                }
                if (!exp.More())
                {
                    NCollection_List<Shape> aLM = aBOP.InternalModified(fdsc);
                    it2.Initialize(aLM);
                    for (; it2.More(); it2.Next())
                    {
                        Shape aS = it2.Value();
                        newdsc.Add(aS);
                    }
                }
            }
            myMap.ChangeFind(orig).Clear();
            foreach (Shape itm in newdsc)
            {
                // check the belonging to the shape...
                for (exp.Init(S, ShapeType.Face); exp.More(); exp.Next())
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
