// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Generator.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Generator.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Generator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Glues a generated shape (<see cref="LocOpe_GeneratedShape"/>) on a shape: fuses the coplanar faces and the
/// collinear edges along the generating edges, and rebuilds the solid.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, it writes into the shapes it works with: point representations of the shape's vertices on the fused
/// edges, pcurves of the generated shape's edges on the rebuilt faces. So the shape and the generated shape's
/// sub-shapes are worked on as copies when frozen (decision 8901), and the result shares the caller's unchanged
/// sub-shapes. <see cref="ResultingShape"/> and <see cref="DescendantFace"/> freeze what they hand out; LocOpe_Gluer
/// reads the <c>Internal</c> forms and ends the working shapes itself.
/// </para>
/// <para>
/// OCCT iterates its maps of faces to fuse and edges to fuse keyed by the shapes' addresses, so the order in which
/// faces are rebuilt (and with it the order of the result's faces) depends on the run natively; the port takes them in
/// the order they are found.
/// </para>
/// </remarks>
public sealed class LocOpe_Generator
{
    private TopoDS_Shape myShape = new();
    private LocOpe_GeneratedShape? myGen;
    private bool myDone;
    private TopoDS_Shape myRes = new();
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myModShapes = new(TopTools_ShapeMapHasher.Instance);
    private LocOpe_WorkingShapes myWork = new();
    private bool myOwnsWork = true;
    // the generated shape answers for its own shapes (a caller's), not the working copies
    private bool myMapsG = true;

    /// <summary>Empty constructor.</summary>
    public LocOpe_Generator()
    {
        myDone = false;
    }

    /// <summary>Creates the algorithm on the shape <paramref name="S"/>.</summary>
    public LocOpe_Generator(in TopoDS_Shape S)
    {
        myShape = S;
        myDone = false;
    }

    /// <summary>
    /// For LocOpe_Gluer: the algorithm on its working shape, sharing its working shapes (it ends them), with a generated
    /// shape built on working shapes.
    /// </summary>
    internal LocOpe_Generator(in TopoDS_Shape S, LocOpe_WorkingShapes theWork)
    {
        myWork = theWork;
        myOwnsWork = false;
        myMapsG = false;
        myShape = S;
        myDone = false;
    }

    /// <summary>Initializes the algorithm on the shape <paramref name="S"/>.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myShape = S;
        myDone = false;
    }

    public bool IsDone() => myDone;

    /// <summary>Returns the new shape (frozen).</summary>
    public TopoDS_Shape ResultingShape() => LocOpe_WorkingShapes.Published(InternalResultingShape());

    /// <summary><see cref="ResultingShape"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalResultingShape()
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        return myRes;
    }

    /// <summary>Returns the initial shape (the caller's).</summary>
    public TopoDS_Shape Shape() => myWork.Original(myShape);

    // The generated shape's answers, as working shapes
    private NCollection_List<TopoDS_Shape> GeneratingEdges(LocOpe_GeneratedShape G) => WorkingList(G.GeneratingEdges());

    private TopoDS_Edge Generated(LocOpe_GeneratedShape G, in TopoDS_Vertex V) =>
        TopoDS.Edge(myWork.Working(G.Generated(TopoDS.Vertex(GOriginal(V)))));

    private TopoDS_Face Generated(LocOpe_GeneratedShape G, in TopoDS_Edge E) =>
        TopoDS.Face(myWork.Working(G.Generated(TopoDS.Edge(GOriginal(E)))));

    private NCollection_List<TopoDS_Shape> OrientedFaces(LocOpe_GeneratedShape G) => WorkingList(G.OrientedFaces());

    private TopoDS_Shape GOriginal(in TopoDS_Shape theShape) => myMapsG ? myWork.Original(theShape) : theShape;

    private NCollection_List<TopoDS_Shape> WorkingList(NCollection_List<TopoDS_Shape> theList)
    {
        if (!myMapsG)
        {
            return theList;
        }
        var aList = new NCollection_List<TopoDS_Shape>();
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            aList.Append(myWork.Working(anIt.Value()));
        }
        return aList;
    }

    /// <summary>
    /// Glues <paramref name="G"/> on the shape. Ends the working shapes: the result shares the caller's unchanged
    /// sub-shapes.
    /// </summary>
    public void Perform(LocOpe_GeneratedShape G)
    {
        if (myShape.IsNull())
        {
            throw new Standard_NullObject();
        }
        if (myOwnsWork)
        {
            myWork = new LocOpe_WorkingShapes();
            myShape = myWork.Working(myShape);
        }
        InternalPerform(G);
        if (!myOwnsWork || !myDone || !myWork.IsCopied())
        {
            return;
        }

        var aShapes = new List<TopoDS_Shape> { myRes };
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myModShapes)
        {
            aShapes.AddRange(anItem.Value);
        }
        myWork.Finish(aShapes);
        myRes = myWork.Final(myRes);
        foreach (TopoDS_Shape aKey in myModShapes.Keys.ToList())
        {
            myModShapes[aKey] = myWork.Final(myModShapes[aKey]);
        }
    }

    // Perform on the working shapes
    internal void InternalPerform(LocOpe_GeneratedShape G)
    {
        if (myShape.IsNull())
        {
            throw new Standard_NullObject();
        }
        myDone = false;
        myRes.Nullify();
        myModShapes.Clear();
        myGen = G;

        NCollection_List<TopoDS_Shape> ledges = GeneratingEdges(G);

        // On genere une liste des faces a gauche du wire. Equivalent du LeftOf.
        // Attention : il faudra bien propager pour ne pas oublier des faces
        // a l`interieur

        TopExp_Explorer exp = new(), exp2 = new(), exp3 = new();
        NCollection_Map<TopoDS_Shape> theLeft = new(TopTools_ShapeMapHasher.Instance); // Faces a gauche

        // Edges et vertex generateurs; OCCT iterates GEdg keyed by the edges' addresses, the port in their order
        NCollection_IndexedMap<TopoDS_Shape> GEdg = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> GVtx = new(TopTools_ShapeMapHasher.Instance);

        NCollection_List<TopoDS_Shape>.Iterator itl = new(), itl2 = new();

        for (itl.Initialize(ledges); itl.More(); itl.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(itl.Value());

            GEdg.Add(edg);
            for (exp2.Init(edg, TopAbs_VERTEX); exp2.More(); exp2.Next())
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(exp2.Current());
                if (!GVtx.Contains(vtx))
                {
                    GVtx.Add(vtx);
                }
            }
            for (exp2.Init(myShape, TopAbs_FACE); exp2.More(); exp2.Next())
            {
                TopoDS_Face fac = TopoDS.Face(exp2.Current());
                for (exp3.Init(fac, TopAbs_EDGE); exp3.More(); exp3.Next())
                {
                    if (exp3.Current().IsSame(edg) && exp3.Current().Orientation() == edg.Orientation())
                    {
                        theLeft.Add(fac);
                        NCollection_List<TopoDS_Shape> emptylist = new();
                        if (!myModShapes.IsBound(fac))
                        {
                            myModShapes.Bind(fac, emptylist);
                        }
                        break;
                    }
                }
                if (exp3.More())
                {
                    break;
                }
            }
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEFMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, theEFMap);

        // OCCT iterates these maps keyed by the shapes' addresses; the port in insertion order
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEEMap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theFFMap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> toRemove = new(TopTools_ShapeMapHasher.Instance);

        // search for face fusions
        for (int itm = 1; itm <= GEdg.Extent(); itm++)
        {
            TopoDS_Edge edg = TopoDS.Edge(GEdg[itm]);
            if (!theEFMap.Contains(edg))
            {
                continue;
            }
            for (itl2.Initialize(theEFMap.FindFromKey(edg)); itl2.More(); itl2.Next())
            {
                if (!theLeft.Contains(itl2.Value()))
                {
                    break;
                }
            }
            if (!itl2.More())
            { // edge "interne" au shell, ou bord libre
            }
            else
            {
                TopoDS_Face fac = TopoDS.Face(itl2.Value());
                TopoDS_Face facbis = Generated(G, edg);
                if (ToFuse(fac, facbis))
                {
                    // On recherche si une face a deja fusionne avec facbis
                    bool facbisfound = false;
                    foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itf in theFFMap)
                    {
                        if (itf.Key.IsSame(fac))
                        {
                            continue;
                        }
                        for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                        {
                            if (itl.Value().IsSame(facbis))
                            {
                                facbisfound = true;
                                break;
                            }
                        }
                        if (facbisfound)
                        {
                            theFFMap[itf.Key].Append(fac);
                            toRemove.Add(fac);
                            toRemove.Add(edg);
                            break;
                        }
                    }

                    if (!facbisfound)
                    {
                        if (!theFFMap.IsBound(fac))
                        {
                            NCollection_List<TopoDS_Shape> thelist = new();
                            theFFMap.Bind(fac, thelist);
                        }
                        for (itl.Initialize(theFFMap[fac]); itl.More(); itl.Next())
                        {
                            if (itl.Value().IsSame(facbis))
                            {
                                break;
                            }
                        }
                        if (!itl.More())
                        {
                            theFFMap[fac].Append(facbis);
                        }
                        toRemove.Add(edg);
                        toRemove.Add(facbis);
                    }
                }
                else
                { // face generee par edg : on la marque.
                    //	myFFromE.Bind(edg,facbis);
                }
            }
        }

        // Il faut ici ajouter dans toRemove les edges de connexites entre faces
        // a fusionner avec une meme face de base

        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itf in theFFMap)
        {
            for (itl.Initialize(itf.Value); itl.More(); itl.Next())
            {
                for (exp.Init(itl.Value(), TopAbs_EDGE); exp.More(); exp.Next())
                {
                    TopoDS_Edge ed = TopoDS.Edge(exp.Current());
                    if (toRemove.Contains(ed))
                    {
                        continue;
                    }
                    for (itl2.Initialize(itf.Value); itl2.More(); itl2.Next())
                    {
                        if (!itl2.Value().IsSame(itl.Value()))
                        {
                            for (exp2.Init(itl2.Value(), TopAbs_EDGE); exp2.More(); exp2.Next())
                            {
                                if (ed.IsSame(exp2.Current()))
                                {
                                    toRemove.Add(ed);
                                    break;
                                }
                            }
                            if (exp2.More())
                            {
                                break;
                            }
                        }
                    }
                }
            }
        }

        NCollection_List<TopoDS_Shape> RebuildFace = new();
        NCollection_Map<TopoDS_Shape> mapTreated = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> DontFuse = new(TopTools_ShapeMapHasher.Instance);
        TopAbs_Orientation orient, orface;

        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itf in theFFMap)
        {
            TopoDS_Face fac = TopoDS.Face(itf.Key);
            for (exp.Init(fac, TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                if (mapTreated.Contains(edg))
                {
                    continue; // on saute l`edge
                }

                mapTreated.Add(edg);
                for (exp2.Init(edg, TopAbs_VERTEX); exp2.More(); exp2.Next())
                {
                    TopoDS_Vertex vtx = TopoDS.Vertex(exp2.Current());
                    if (GVtx.Contains(vtx))
                    {
                        TopoDS_Edge edgbis = Generated(G, vtx);

                        if ((edgbis.IsNull() || BRep_Tool.Degenerated(edgbis)) ||
                            //	      toRemove.Contains(edgbis) ||
                            !ToFuse(edg, edgbis))
                        {
                            continue;
                        }
                        // a voir
                        if (BRepTools.IsReallyClosed(edg, fac))
                        {
                            if (!theEEMap.IsBound(edg))
                            {
                                NCollection_List<TopoDS_Shape> thelist1 = new();
                                theEEMap.Bind(edg, thelist1);
                                theEEMap[edg].Append(edgbis);
                                toRemove.Add(edgbis); // toujours vrai pour edge double
                                bool FuseEdge = true;
                                TopExp.Vertices(edg, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
                                bool ConnectLast = (Vl.IsSame(vtx));
                                for (exp3.Init(fac.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp3.More(); exp3.Next())
                                {
                                    TopoDS_Edge eee = TopoDS.Edge(exp3.Current());
                                    orient = eee.Orientation();
                                    if (!eee.IsSame(edg))
                                    {
                                        TopExp.Vertices(eee, out Vf, out Vl);
                                        if ((Vf.IsSame(vtx) || Vl.IsSame(vtx)) && !toRemove.Contains(eee))
                                        {
                                            FuseEdge = false;
                                            // On recherche celui qu`il ne faut pas fusionner

                                            if ((Vf.IsSame(vtx) && orient == TopAbs_FORWARD)
                                                || (Vl.IsSame(vtx) && orient == TopAbs_REVERSED))
                                            {
                                                if (ConnectLast)
                                                {
                                                    DontFuse.Bind(edg, fac.Oriented(TopAbs_FORWARD));
                                                }
                                                else
                                                {
                                                    DontFuse.Bind(edg, fac.Oriented(TopAbs_REVERSED));
                                                }
                                            }
                                            else
                                            {
                                                if (ConnectLast)
                                                {
                                                    DontFuse.Bind(edg, fac.Oriented(TopAbs_REVERSED));
                                                }
                                                else
                                                {
                                                    DontFuse.Bind(edg, fac.Oriented(TopAbs_FORWARD));
                                                }
                                            }
                                            break;
                                        }
                                    }
                                }
                                if (FuseEdge)
                                {
                                    toRemove.Add(vtx);
                                }
                            }
                        }

                        else
                        {
                            /*  A VOIR
                                  if (!BRep_Tool::IsClosed(edg,fac)) {
                            */
                            if (!theEEMap.IsBound(edg))
                            {
                                NCollection_List<TopoDS_Shape> thelist2 = new();
                                theEEMap.Bind(edg, thelist2);
                            }
                            theEEMap[edg].Append(edgbis);
                            NCollection_List<TopoDS_Shape> L = theEEMap[edg];
                            NCollection_List<TopoDS_Shape>.Iterator Lit = new(L);
                            bool OK = true;
                            for (; Lit.More(); Lit.Next())
                            {
                                if (Lit.Value().IsSame(edgbis))
                                {
                                    OK = false;
                                    break;
                                }
                            }
                            if (OK)
                            {
                                theEEMap[edg].Append(edgbis);
                            }

                            itl.Initialize(theEFMap.FindFromKey(edg));
                            bool FuseEdge = ToFuse(edg, fac, vtx, toRemove);
                            if (!FuseEdge)
                            {
                                DontFuse.Bind(edg, fac);
                            }
                            else
                            {
                                for (; itl.More(); itl.Next())
                                {
                                    if (!itl.Value().IsSame(fac))
                                    {
                                        if (theFFMap.IsBound(itl.Value()))
                                        {
                                            FuseEdge = ToFuse(edg, TopoDS.Face(itl.Value()), vtx, toRemove);
                                            // edge a fusionner
                                            if (FuseEdge)
                                            {
                                                toRemove.Add(vtx);
                                            }
                                            else
                                            {
                                                if (toRemove.Contains(vtx))
                                                {
                                                    toRemove.Remove(vtx);
                                                }
                                                DontFuse.Bind(edg, itl.Value());
                                            }
                                        }
                                        else
                                        { // on marque comme face a reconstruire
                                            RebuildFace.Append(itl.Value());
                                            if (toRemove.Contains(vtx))
                                            {
                                                toRemove.Remove(vtx);
                                            }
                                            DontFuse.Bind(edg, itl.Value());
                                        }

                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        for (itl.Initialize(RebuildFace); itl.More(); itl.Next())
        {
            NCollection_List<TopoDS_Shape> thelist3 = new();
            theFFMap.Bind(itl.Value(), thelist3);
        }

        BRep_Builder B = new();
        TopoDS_Face newface;
        TopoDS_Edge newedg;
        TopoDS_Vertex newvtx = new();
        TopLoc_Location loc;
        double tol = 0.0, prm = 0.0, f, l, Uminc = 0.0, Umaxc = 0.0;
        gp_Pnt2d pf, pl;

        Geom_Surface S;
        Geom_Plane? P;
        Geom_Curve C = null!;

        // Fusion des edges
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> ite in theEEMap)
        {
            bool KeepNewEdge = false;
            TopoDS_Edge edg = TopoDS.Edge(ite.Key);
            BRep_Tool.Range(edg, out f, out l);
            TopoDS_Shape aLocalEdge = edg.EmptyCopied();
            newedg = TopoDS.Edge(aLocalEdge);
            newedg.Orientation(TopAbs_FORWARD);
            for (exp.Init(edg.Oriented(TopAbs_FORWARD), TopAbs_VERTEX); exp.More(); exp.Next())
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(exp.Current());
                prm = BRep_Tool.Parameter(vtx, edg);

                newvtx.Nullify();
                for (itl.Initialize(theEEMap[edg]); itl.More(); itl.Next())
                {
                    TopoDS_Edge edgbis = TopoDS.Edge(itl.Value());
                    for (exp2.Init(edgbis, TopAbs_VERTEX); exp2.More(); exp2.Next())
                    {
                        if (exp2.Current().IsSame(vtx))
                        {
                            break;
                        }
                    }
                    if (exp2.More())
                    {
                        for (exp2.ReInit(); exp2.More(); exp2.Next())
                        {
                            if (!exp2.Current().IsSame(vtx))
                            {
                                newvtx = TopoDS.Vertex(exp2.Current());
                                prm = NewParameter(edg, vtx, newedg, newvtx);
                                break;
                            }
                        }
                        break;
                    }
                }

                if (toRemove.Contains(vtx) || (prm < l && prm > f))
                {
                    B.Add(newedg, newvtx.Oriented(vtx.Orientation()));
                    tol = BRep_Tool.Tolerance(newvtx);
                    B.UpdateVertex(newvtx, prm, newedg, tol);
                    toRemove.Add(itl.Value()); // i-e edgbis
                    KeepNewEdge = true;
                }
                else
                {
                    B.Add(newedg, vtx.Oriented(vtx.Orientation()));
                    tol = BRep_Tool.Tolerance(vtx);
                    B.UpdateVertex(vtx, prm, newedg, tol);
                }
            }
            if (KeepNewEdge)
            {
                NCollection_List<TopoDS_Shape> emptylist = new();
                if (!myModShapes.IsBound(edg))
                {
                    myModShapes.Bind(edg, emptylist);
                }
                myModShapes[edg].Append(newedg);
                toRemove.Add(edg);
            }
        }

        NCollection_Map<TopoDS_Shape> EdgAdded = new(TopTools_ShapeMapHasher.Instance);

        // Fusion des faces, ou reconstruction
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itf in theFFMap)
        {
            TopoDS_Face fac = TopoDS.Face(itf.Key);
            bool ModFace = false;
            NCollection_List<TopoDS_Shape> listofedg = new();

            EdgAdded.Clear();

            for (exp.Init(myShape, TopAbs_FACE); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(fac))
                {
                    break;
                }
            }
            orface = exp.Current().Orientation();
            TopoDS_Shape aLocalFaceEmptyCopied = fac.EmptyCopied();
            newface = TopoDS.Face(aLocalFaceEmptyCopied);
            newface.Orientation(TopAbs_FORWARD);
            S = BRep_Tool.Surface(fac)!;
            if (S.GetType() == typeof(Geom_RectangularTrimmedSurface))
            {
                S = ((Geom_RectangularTrimmedSurface)S).BasisSurface();
            }
            P = S as Geom_Plane;
            TopoDS_Wire wir;
            for (exp.Init(fac.Oriented(TopAbs_FORWARD), TopAbs_WIRE); exp.More(); exp.Next())
            {
                wir = TopoDS.Wire(exp.Current());
                for (exp2.Init(wir, TopAbs_EDGE); exp2.More(); exp2.Next())
                {
                    TopoDS_Edge edg = TopoDS.Edge(exp2.Current());
                    if (toRemove.Contains(edg) || myModShapes.IsBound(edg))
                    {
                        break;
                    }
                }
                if (!exp2.More())
                { // wire non modifie
                    for (exp2.Init(wir, TopAbs_EDGE); exp2.More(); exp2.Next())
                    {
                        listofedg.Append(exp2.Current());
                    }
                }
                else
                {
                    if (!ModFace)
                    {
                        // Petit truc crad pour les p-curves sur les cylindres...
                        if (P is null)
                        {
                            BRepTools.UVBounds(fac, out Uminc, out Umaxc, out _, out _);
                        }

                        // premier passage : on met les wires non touches des faces
                        // en vis a vis

                        for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                        {
                            TopoDS_Face facbis = TopoDS.Face(itl.Value());
                            NCollection_List<TopoDS_Shape> aGFaces = OrientedFaces(G);
                            for (itl2.Initialize(aGFaces); itl2.More(); itl2.Next())
                            {
                                if (itl2.Value().IsSame(facbis))
                                {
                                    break;
                                }
                            }
                            if (itl2.More())
                            {
                                orient = itl2.Value().Orientation();
                                facbis.Orientation(orient);
                            }
                            else
                            { // on fusionne avec une autre face du shape...
                                for (exp2.Init(myShape, TopAbs_FACE); exp2.More(); exp2.Next())
                                {
                                    if (exp2.Current().IsSame(facbis))
                                    {
                                        facbis.Orientation(exp2.Current().Orientation());
                                        break;
                                    }
                                }
                            }

                            for (exp3.Init(facbis, TopAbs_WIRE); exp3.More(); exp3.Next())
                            {
                                for (exp2.Init(exp3.Current(), TopAbs_EDGE); exp2.More(); exp2.Next())
                                {
                                    if (toRemove.Contains(exp2.Current()))
                                    {
                                        break;
                                    }
                                }
                                if (!exp2.More())
                                {
                                    TopoDS_Wire theNew = new();
                                    B.MakeWire(ref theNew); // FORWARD
                                    for (exp2.ReInit(); exp2.More(); exp2.Next())
                                    {
                                        TopoDS_Edge edg = TopoDS.Edge(exp2.Current());

                                        orient = TopAbs.Compose(orface, edg.Orientation());
                                        listofedg.Append(edg.Oriented(orient));
                                        EdgAdded.Add(edg);
                                        if (P is null)
                                        {
                                            // on met les courbes 2d si on n`est pas sur un plan
                                            // on ne devrait pas avoir de pb d`edge de couture.
                                            tol = BRep_Tool.Tolerance(edg);
                                            C = BRep_Tool.Curve(edg, out loc, out f, out l)!;
                                            if (!loc.IsIdentity())
                                            {
                                                Geom_Geometry GG = C.Transformed(loc.Transformation());
                                                C = (Geom_Curve)GG;
                                            }
                                            if (C.GetType() == typeof(Geom_TrimmedCurve))
                                            {
                                                C = ((Geom_TrimmedCurve)C).BasisCurve();
                                            }

                                            Geom2d_Curve C2d0 = Translatable(GeomProjLib.Curve2d(C, f, l, S, ref tol)!);

                                            // Tentative de recalage dans la facette
                                            pf = C2d0.Value(f);
                                            pl = C2d0.Value(l);
                                            double tttol = Precision.Angular();
                                            while (Min(pf.X(), pl.X()) >= Umaxc - tttol)
                                            {
                                                C2d0.Translate(new gp_Vec2d(-2.0 * Math.PI, 0));
                                                pf = C2d0.Value(f);
                                                pl = C2d0.Value(l);
                                            }

                                            while (Max(pf.X(), pl.X()) <= Uminc + tttol)
                                            {
                                                C2d0.Translate(new gp_Vec2d(2.0 * Math.PI, 0));
                                                pf = C2d0.Value(f);
                                                pl = C2d0.Value(l);
                                            }

                                            if (!BRepTools.IsReallyClosed(edg, facbis))
                                            {
                                                B.UpdateEdge(edg, C2d0, newface, tol);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        ModFace = true;
                    }

                    // reconstruction du wire

                    Geom2d_Curve? C2d, C2d1 = null;

                    for (exp2.Init(wir, TopAbs_EDGE); exp2.More(); exp2.Next())
                    {
                        TopoDS_Edge edg = TopoDS.Edge(exp2.Current());
                        orient = edg.Orientation();
                        if (!toRemove.Contains(edg) && !theEEMap.IsBound(edg))
                        {
                            listofedg.Append(edg);
                        }
                        else if (myModShapes.IsBound(edg) || theEEMap.IsBound(edg))
                        {
                            if (myModShapes.IsBound(edg))
                            {
                                newedg = TopoDS.Edge(myModShapes[edg].First());
                            }
                            else
                            {
                                newedg = edg;
                            }
                            listofedg.Append(newedg.Oriented(orient));
                            C = BRep_Tool.Curve(newedg, out loc, out f, out l)!;
                            if (!loc.IsIdentity())
                            {
                                Geom_Geometry GG = C.Transformed(loc.Transformation());
                                C = (Geom_Curve)GG;
                            }
                            if (C.GetType() == typeof(Geom_TrimmedCurve))
                            {
                                C = ((Geom_TrimmedCurve)C).BasisCurve();
                            }
                            if (P is null)
                            { // on met les courbes 2d si on n`est pas
                                // sur un plan
                                TopAbs_Orientation oredonfafw = orient;
                                tol = BRep_Tool.Tolerance(newedg);
                                TopoDS_Shape aLocalEdge = edg.Oriented(oredonfafw);
                                TopoDS_Shape aLocalFace = fac.Oriented(TopAbs_FORWARD);
                                C2d = BRep_Tool.CurveOnSurface(TopoDS.Edge(aLocalEdge), TopoDS.Face(aLocalFace), out f, out l);

                                if (!BRepTools.IsReallyClosed(edg, fac))
                                {
                                    B.UpdateEdge(newedg, C2d, newface, tol);
                                }
                                else if (C2d1 is null)
                                {
                                    C2d1 = C2d;
                                }
                                else
                                {
                                    if (orient == TopAbs_FORWARD)
                                    {
                                        B.UpdateEdge(newedg, C2d, C2d1, newface, tol);
                                    }
                                    else
                                    {
                                        B.UpdateEdge(newedg, C2d1, C2d, newface, tol);
                                    }
                                }
                            }
                            bool AddPart = false;
                            if (DontFuse.IsBound(edg))
                            {
                                if (!BRepTools.IsReallyClosed(edg, fac))
                                {
                                    if (DontFuse[edg].IsSame(fac))
                                    {
                                        if (myModShapes.IsBound(edg))
                                        {
                                            AddPart = true;
                                        }
                                    }
                                    else if (!toRemove.Contains(edg))
                                    {
                                        AddPart = true;
                                    }
                                }
                                else
                                {
                                    if (myModShapes.IsBound(edg))
                                    { // edg raccourci
                                        if (orient == DontFuse[edg].Orientation())
                                        {
                                            AddPart = true;
                                        }
                                    }
                                    else
                                    {
                                        if (orient == TopAbs.Reverse(DontFuse[edg].Orientation()))
                                        {
                                            AddPart = true;
                                        }
                                    }
                                }
                            }
                            if (AddPart)
                            {
                                itl2.Initialize(theEEMap[edg]);
                                gp_Vec dir1, dir2;
                                for (; itl2.More(); itl2.Next())
                                {
                                    if (EdgAdded.Contains(itl2.Value()))
                                    {
                                        continue;
                                    }
                                    TopoDS_Edge edgbis = TopoDS.Edge(itl2.Value());
                                    TopoDS_Iterator it1 = new(newedg), it2 = new();
                                    for (; it1.More(); it1.Next())
                                    {
                                        for (it2.Initialize(edgbis); it2.More(); it2.Next())
                                        {
                                            if (it1.Value().IsSame(it2.Value()))
                                            {
                                                break;
                                            }
                                        }
                                        if (it2.More())
                                        {
                                            break;
                                        }
                                    }

                                    if (it1.More())
                                    {
                                        double prmvt = BRep_Tool.Parameter(TopoDS.Vertex(it1.Value()), newedg);
                                        C.D1(prmvt, out gp_Pnt ptbid, out dir1);

                                        C = BRep_Tool.Curve(edgbis, out loc, out f, out l)!;
                                        if (!loc.IsIdentity())
                                        {
                                            Geom_Geometry GG = C.Transformed(loc.Transformation());
                                            C = (Geom_Curve)GG;
                                        }
                                        if (C.GetType() == typeof(Geom_TrimmedCurve))
                                        {
                                            C = ((Geom_TrimmedCurve)C).BasisCurve();
                                        }
                                        prmvt = BRep_Tool.Parameter(TopoDS.Vertex(it1.Value()), edgbis);
                                        C.D1(prmvt, out ptbid, out dir2);
                                    }
                                    else
                                    {
                                        dir1 = dir2 = new gp_Vec(1, 0, 0);
                                    }
                                    EdgAdded.Add(edgbis);
                                    if (dir1.Dot(dir2) < 0.0)
                                    {
                                        listofedg.Append(edgbis.Oriented(TopAbs.Reverse(orient)));
                                    }
                                    else
                                    {
                                        listofedg.Append(edgbis.Oriented(orient));
                                    }

                                    if (P is null)
                                    {
                                        // on met les courbes 2d si on n`est pas sur un plan
                                        // C est la courbe de edgbis, f et l s`y rapportent
                                        Geom2d_Curve PTC = Translatable(GeomProjLib.Curve2d(C, f, l, S, ref tol)!);
                                        if (S.IsUPeriodic())
                                        {
                                            double Uref;
                                            if (DontFuse.IsBound(edg))
                                            {
                                                TopAbs_Orientation oredge = DontFuse[edg].Orientation();
                                                if (myModShapes.IsBound(edg))
                                                { // edge raccourci...
                                                    TopoDS_Shape aLocalShape = edg.Oriented(oredge);
                                                    TopoDS_Shape aLocalFace = fac.Oriented(TopAbs_FORWARD);
                                                    BRep_Tool.UVPoints(TopoDS.Edge(aLocalShape), TopoDS.Face(aLocalFace), out pf, out pl);
                                                }
                                                else
                                                {
                                                    TopoDS_Shape aLocalShape = edg.Oriented(TopAbs.Reverse(oredge));
                                                    TopoDS_Shape aLocalFace = fac.Oriented(TopAbs_FORWARD);
                                                    BRep_Tool.UVPoints(TopoDS.Edge(aLocalShape), TopoDS.Face(aLocalFace), out pf, out pl);
                                                }
                                                Uref = pf.X();
                                            }
                                            else
                                            {
                                                BRep_Tool.UVPoints(edg, fac, out pf, out pl);
                                                Uref = pf.X();
                                            }

                                            double NewU = (PTC.Value(f)).X();

                                            if (Math.Abs(NewU - Uref) > Epsilon(S.UPeriod()))
                                            {
                                                PTC.Translate(new gp_Vec2d((Uref - NewU), 0.0));
                                            }
                                        }

                                        B.UpdateEdge(edgbis, PTC, newface, tol);
                                    }
                                }
                            }
                        }
                    }

                    // Recuperation des edges sur les faces a fusionner.

                    // ICICICICI

                    bool includeinw = false;
                    for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                    {
                        TopoDS_Face facbis = TopoDS.Face(itl.Value());
                        bool genface = true;
                        NCollection_List<TopoDS_Shape> aGFaces = OrientedFaces(G);
                        for (itl2.Initialize(aGFaces); itl2.More(); itl2.Next())
                        {
                            if (itl2.Value().IsSame(facbis))
                            {
                                break;
                            }
                        }
                        if (itl2.More())
                        {
                            orient = itl2.Value().Orientation();
                            facbis.Orientation(orient);
                        }
                        else
                        { // on fusionne avec une autre face du shape...
                            genface = false;
                            for (exp2.Init(myShape, TopAbs_FACE); exp2.More(); exp2.Next())
                            {
                                if (exp2.Current().IsSame(facbis))
                                {
                                    facbis.Orientation(exp2.Current().Orientation());
                                    break;
                                }
                            }
                        }

                        for (exp3.Init(facbis, TopAbs_WIRE); exp3.More(); exp3.Next())
                        {
                            for (exp2.Init(exp3.Current(), TopAbs_EDGE); exp2.More(); exp2.Next())
                            {
                                if (toRemove.Contains(exp2.Current()))
                                {
                                    break;
                                }
                            }
                            if (genface)
                            {
                                includeinw = true;
                            }
                            else
                            {
                                bool isIncludedInW = false;
                                if (exp2.More())
                                {
                                    for (exp2.ReInit(); exp2.More(); exp2.Next())
                                    {
                                        if (!toRemove.Contains(exp2.Current()))
                                        {
                                            continue;
                                        }
                                        TopExp.Vertices(TopoDS.Edge(exp2.Current()), out TopoDS_Vertex VF, out TopoDS_Vertex VL);
                                        TopExp_Explorer exp4 = new();
                                        for (exp4.Init(wir, TopAbs_VERTEX); exp4.More(); exp4.Next())
                                        {
                                            if (exp4.Current().IsSame(VF) || exp4.Current().IsSame(VL))
                                            {
                                                isIncludedInW = true;
                                                break;
                                            }
                                        }
                                        if (isIncludedInW)
                                        {
                                            break;
                                        }
                                    }
                                }
                                // OCCT computes isIncludedInW and doesn't use it (includeinw stays as it was)
                            }

                            if (!includeinw)
                            {
                                continue;
                            }

                            for (exp2.ReInit(); exp2.More(); exp2.Next())
                            {
                                TopoDS_Edge edg = TopoDS.Edge(exp2.Current());
                                if (!toRemove.Contains(edg) && !EdgAdded.Contains(edg))
                                {
                                    orient = TopAbs.Compose(orface, edg.Orientation());
                                    listofedg.Append(edg.Oriented(orient));
                                    EdgAdded.Add(edg);
                                    if (P is null)
                                    {
                                        // on met les courbes 2d si on n`est pas sur un plan
                                        // on ne devrait pas avoir de pb d`edge de couture.
                                        tol = BRep_Tool.Tolerance(edg);
                                        C = BRep_Tool.Curve(edg, out loc, out f, out l)!;
                                        if (!loc.IsIdentity())
                                        {
                                            Geom_Geometry GG = C.Transformed(loc.Transformation());
                                            C = (Geom_Curve)GG;
                                        }
                                        if (C.GetType() == typeof(Geom_TrimmedCurve))
                                        {
                                            C = ((Geom_TrimmedCurve)C).BasisCurve();
                                        }

                                        C2d = Translatable(GeomProjLib.Curve2d(C, f, l, S, ref tol)!);

                                        // Tentative de recalage dans la facette
                                        pf = C2d.Value(f);
                                        pl = C2d.Value(l);
                                        double tttol = Precision.Angular();
                                        while (Min(pf.X(), pl.X()) >= Umaxc - tttol)
                                        {
                                            C2d.Translate(new gp_Vec2d(-2.0 * Math.PI, 0));
                                            pf = C2d.Value(f);
                                            pl = C2d.Value(l);
                                        }

                                        while (Max(pf.X(), pl.X()) <= Uminc + tttol)
                                        {
                                            C2d.Translate(new gp_Vec2d(2.0 * Math.PI, 0));
                                            pf = C2d.Value(f);
                                            pl = C2d.Value(l);
                                        }

                                        if (!BRepTools.IsReallyClosed(edg, facbis))
                                        {
                                            B.UpdateEdge(edg, C2d, newface, tol);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (!listofedg.IsEmpty())
            {
                BRepAlgo_Loop L = new();
                L.Init(newface);
                L.AddConstEdges(listofedg);
                L.Perform();
                L.WiresToFaces();
                NCollection_List<TopoDS_Shape> listoffaces = L.NewFaces();
                toRemove.Add(fac);
                myModShapes.Bind(fac, new NCollection_List<TopoDS_Shape>(listoffaces));
                for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                {
                    myModShapes.Bind(itl.Value(), new NCollection_List<TopoDS_Shape>(listoffaces));
                }
            }
        }

        // JAG 16.09.96 : on utilise LocOpe_BuildShape (OCCT's former construction of the shell is commented out)

        // 06.11.96
        // Debug temporaire. Il faudra prevoir un syntaxe de BuildShape
        // qui impose une ori de certaines faces.

        TopoDS_Face FaceRefOri = new();

        NCollection_List<TopoDS_Shape> lfres = new();
        for (exp.Init(myShape, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Face fac = TopoDS.Face(exp.Current());
            if (!theLeft.Contains(fac))
            {
                if (!toRemove.Contains(exp.Current()))
                {
                    lfres.Append(fac);
                    if (!myModShapes.IsBound(fac))
                    {
                        NCollection_List<TopoDS_Shape> emptylist = new();
                        myModShapes.Bind(fac, emptylist);
                    }
                    myModShapes[fac].Append(fac);
                    if (FaceRefOri.IsNull())
                    {
                        FaceRefOri = fac;
                    }
                }
                else if (myModShapes.IsBound(fac))
                {
                    lfres.Append(myModShapes[fac].First().Oriented(fac.Orientation()));
                }
            }
        }

        TopAbs_Orientation orsolid = myShape.Orientation();
        for (itl.Initialize(OrientedFaces(G)); itl.More(); itl.Next())
        {
            TopoDS_Face fac = TopoDS.Face(itl.Value());
            if (toRemove.Contains(fac))
            {
                continue;
            }

            if (orsolid == TopAbs_FORWARD)
            {
                lfres.Append(fac);
            }
            else
            {
                lfres.Append(fac.Reversed());
            }
            if (!myModShapes.IsBound(fac))
            {
                NCollection_List<TopoDS_Shape> emptylist = new();
                myModShapes.Bind(fac, emptylist);
            }
            myModShapes[fac].Append(fac);
        }

        LocOpe_BuildShape BS = new(lfres);
        myRes = BS.InternalShape();
        // Suite debug du 06.11.96
        if (myRes.ShapeType() == TopAbs_SOLID)
        {
            for (exp.Init(myRes, TopAbs_FACE); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(FaceRefOri))
                {
                    break;
                }
            }
            if (exp.More() && exp.Current().Orientation() != FaceRefOri.Orientation())
            {
                // Si un seul Shell, on change son orientation
                TopoDS_Solid NewSol = new();
                B.MakeSolid(ref NewSol);
                exp.Init(myRes, TopAbs_SHELL);
                B.Add(NewSol, exp.Current().Reversed());
                myRes.Nullify();
                myRes = NewSol;
            }
        }

        // recodage des regularites qui existaient sur le shape colle

        myDone = true;
    }

    /// <summary>
    /// Returns the descendant faces of <paramref name="F"/> (frozen). <paramref name="F"/> may belong to the original
    /// shape or to the "generated" shape. The list may hold a null face (when <paramref name="F"/> disappears).
    /// </summary>
    public NCollection_List<TopoDS_Shape> DescendantFace(in TopoDS_Face F) =>
        LocOpe_WorkingShapes.Published(InternalDescendantFace(F));

    /// <summary>
    /// <see cref="DescendantFace"/> without freezing, for ported code (of the caller's face or of its working copy).
    /// </summary>
    internal NCollection_List<TopoDS_Shape> InternalDescendantFace(in TopoDS_Face F)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        return myModShapes[myWork.Copied(F)];
    }

    // OCCT.NET: OCCT translates the projected pcurve in place; a frozen one (shared) is translated on a copy
    private static Geom2d_Curve Translatable(Geom2d_Curve theCurve) =>
        theCurve.IsFrozen() ? (Geom2d_Curve)theCurve.Copy() : theCurve;

    private static bool ToFuse(in TopoDS_Face F1, in TopoDS_Face F2)
    {
        if (F1.IsNull() || F2.IsNull())
        {
            return false;
        }

        double tollin = Precision.Confusion();
        double tolang = Precision.Angular();

        Geom_Surface S1 = BRep_Tool.Surface(F1, out TopLoc_Location loc1)!;
        Geom_Surface S2 = BRep_Tool.Surface(F2, out TopLoc_Location loc2)!;

        if (S1.GetType() == typeof(Geom_RectangularTrimmedSurface))
        {
            S1 = ((Geom_RectangularTrimmedSurface)S1).BasisSurface();
        }

        if (S2.GetType() == typeof(Geom_RectangularTrimmedSurface))
        {
            S2 = ((Geom_RectangularTrimmedSurface)S2).BasisSurface();
        }

        if (S1.GetType() != S2.GetType())
        {
            return false;
        }

        bool ValRet = false;
        if (S1.GetType() == typeof(Geom_Plane))
        {
            gp_Pln pl1 = ((Geom_Plane)S1).Pln();
            gp_Pln pl2 = ((Geom_Plane)S2).Pln();

            pl1.Transform(loc1.Transformation());
            pl2.Transform(loc2.Transformation());

            if (pl1.Position().IsCoplanar(pl2.Position(), tollin, tolang))
            {
                ValRet = true;
            }
        }

        return ValRet;
    }

    private static bool ToFuse(in TopoDS_Edge E1, in TopoDS_Edge E2)
    {
        if (E1.IsNull() || E2.IsNull())
        {
            return false;
        }

        double tollin = Precision.Confusion();
        double tolang = Precision.Angular();

        Geom_Curve C1 = BRep_Tool.Curve(E1, out TopLoc_Location loc1, out _, out _)!;
        if (!loc1.IsIdentity())
        {
            Geom_Geometry CC1 = C1.Transformed(loc1.Transformation());
            C1 = (Geom_Curve)CC1;
        }

        Geom_Curve C2 = BRep_Tool.Curve(E2, out TopLoc_Location loc2, out _, out _)!;
        if (!loc2.IsIdentity())
        {
            Geom_Geometry CC2 = C2.Transformed(loc2.Transformation());
            C2 = (Geom_Curve)CC2;
        }

        if (C1.GetType() == typeof(Geom_TrimmedCurve))
        {
            C1 = ((Geom_TrimmedCurve)C1).BasisCurve();
        }
        if (C2.GetType() == typeof(Geom_TrimmedCurve))
        {
            C2 = ((Geom_TrimmedCurve)C2).BasisCurve();
        }

        if (C1.GetType() != C2.GetType())
        {
            return false;
        }

        bool ValRet = false;
        if (C1.GetType() == typeof(Geom_Line))
        {
            gp_Lin li1 = ((Geom_Line)C1).Lin();
            gp_Lin li2 = ((Geom_Line)C2).Lin();

            if (li1.Position().IsCoaxial(li2.Position(), tolang, tollin))
            {
                ValRet = true;
            }
        }

        return ValRet;
    }

    private static bool ToFuse(in TopoDS_Edge E, in TopoDS_Face F, in TopoDS_Vertex V, NCollection_Map<TopoDS_Shape> toRemove)
    {
        TopExp_Explorer exp = new();
        for (exp.Init(F, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge eee = TopoDS.Edge(exp.Current());
            if (!eee.IsSame(E))
            {
                TopExp.Vertices(eee, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
                if ((Vf.IsSame(V) || Vl.IsSame(V)) && !toRemove.Contains(eee))
                {
                    return false;
                }
            }
        }
        return true;
    }

    private static double NewParameter(in TopoDS_Edge Edg, in TopoDS_Vertex Vtx, in TopoDS_Edge NewEdg, in TopoDS_Vertex NewVtx)
    {
        gp_Pnt P = BRep_Tool.Pnt(NewVtx);

        Geom_Curve C = BRep_Tool.Curve(Edg, out TopLoc_Location loc, out _, out _)!;
        if (!loc.IsIdentity())
        {
            Geom_Geometry GG = C.Transformed(loc.Transformation());
            C = (Geom_Curve)GG;
        }
        if (C.GetType() == typeof(Geom_TrimmedCurve))
        {
            C = ((Geom_TrimmedCurve)C).BasisCurve();
        }

        if (C.GetType() == typeof(Geom_Line))
        {
            return ElCLib.Parameter(((Geom_Line)C).Lin(), P);
        }
        else if (C.GetType() == typeof(Geom_Circle))
        {
            double prm = ElCLib.Parameter(((Geom_Circle)C).Circ(), P);
            // Vtx vient d`une exploration de Edg orientee FORWARD

            TopAbs_Orientation orient = TopAbs.Reverse(Vtx.Orientation());
            if (orient == TopAbs_FORWARD || orient == TopAbs_REVERSED)
            {
                TopExp_Explorer exp = new(NewEdg.Oriented(TopAbs_FORWARD), TopAbs_VERTEX);
                for (; exp.More(); exp.Next())
                {
                    if (exp.Current().Orientation() == orient)
                    {
                        break;
                    }
                }
                if (exp.More())
                {
                    double prmmax = BRep_Tool.Parameter(TopoDS.Vertex(exp.Current()), NewEdg);
                    if (Math.Abs(prmmax - prm) <= Epsilon(2.0 * Math.PI))
                    {
                        if (orient == TopAbs_REVERSED)
                        {
                            prm -= 2.0 * Math.PI;
                        }
                        else
                        {
                            prm += 2.0 * Math.PI;
                        }
                    }
                }
            }
            return prm;
        }
        return 0;
    }
}
