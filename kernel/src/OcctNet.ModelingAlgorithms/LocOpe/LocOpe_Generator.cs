// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Generator.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Generator.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Generator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Glues a generated shape (<see cref="LocOpe_GeneratedShape"/>) on a shape: fuses the coplanar faces and the
/// collinear edges along the generating edges, and rebuilds the solid.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, it writes into the shapes it works with: point representations of the shape's vertices on the fused
/// edges, pcurves of the generated shape's edges on the rebuilt faces. So the shape and the generated shape's
/// sub-shapes are worked on as copies when frozen (decision 1013), and the result shares the caller's unchanged
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
    private Shape myShape = new();
    private LocOpe_GeneratedShape? myGen;
    private bool myDone;
    private Shape myRes = new();
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myModShapes = new(ShapeHasher.Instance);
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
    public LocOpe_Generator(in Shape S)
    {
        myShape = S;
        myDone = false;
    }

    /// <summary>
    /// For LocOpe_Gluer: the algorithm on its working shape, sharing its working shapes (it ends them), with a generated
    /// shape built on working shapes.
    /// </summary>
    internal LocOpe_Generator(in Shape S, LocOpe_WorkingShapes theWork)
    {
        myWork = theWork;
        myOwnsWork = false;
        myMapsG = false;
        myShape = S;
        myDone = false;
    }

    /// <summary>Initializes the algorithm on the shape <paramref name="S"/>.</summary>
    public void Init(in Shape S)
    {
        myShape = S;
        myDone = false;
    }

    public bool IsDone() => myDone;

    /// <summary>Returns the new shape (frozen).</summary>
    public Shape ResultingShape() => LocOpe_WorkingShapes.Published(InternalResultingShape());

    /// <summary><see cref="ResultingShape"/> without freezing, for ported code.</summary>
    internal Shape InternalResultingShape()
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        return myRes;
    }

    /// <summary>Returns the initial shape (the caller's).</summary>
    public Shape Shape() => myWork.Original(myShape);

    // The generated shape's answers, as working shapes
    private NCollection_List<Shape> GeneratingEdges(LocOpe_GeneratedShape G) => WorkingList(G.GeneratingEdges());

    private Edge Generated(LocOpe_GeneratedShape G, in Vertex V) =>
        ShapeCasts.Edge(myWork.Working(G.Generated(ShapeCasts.Vertex(GOriginal(V)))));

    private Face Generated(LocOpe_GeneratedShape G, in Edge E) =>
        ShapeCasts.Face(myWork.Working(G.Generated(ShapeCasts.Edge(GOriginal(E)))));

    private NCollection_List<Shape> OrientedFaces(LocOpe_GeneratedShape G) => WorkingList(G.OrientedFaces());

    private Shape GOriginal(in Shape theShape) => myMapsG ? myWork.Original(theShape) : theShape;

    private NCollection_List<Shape> WorkingList(NCollection_List<Shape> theList)
    {
        if (!myMapsG)
        {
            return theList;
        }
        var aList = new NCollection_List<Shape>();
        for (NCollection_List<Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
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
        if (myShape.IsNull)
        {
            throw new NullObjectException();
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

        var aShapes = new List<Shape> { myRes };
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> anItem in myModShapes)
        {
            aShapes.AddRange(anItem.Value);
        }
        myWork.Finish(aShapes);
        myRes = myWork.Final(myRes);
        foreach (Shape aKey in myModShapes.Keys.ToList())
        {
            myModShapes[aKey] = myWork.Final(myModShapes[aKey]);
        }
    }

    // Perform on the working shapes
    internal void InternalPerform(LocOpe_GeneratedShape G)
    {
        if (myShape.IsNull)
        {
            throw new NullObjectException();
        }
        myDone = false;
        myRes.Nullify();
        myModShapes.Clear();
        myGen = G;

        NCollection_List<Shape> ledges = GeneratingEdges(G);

        // On genere une liste des faces a gauche du wire. Equivalent du LeftOf.
        // Attention : il faudra bien propager pour ne pas oublier des faces
        // a l`interieur

        Explorer exp = new(), exp2 = new(), exp3 = new();
        NCollection_Map<Shape> theLeft = new(ShapeHasher.Instance); // Faces a gauche

        // Edges et vertex generateurs; OCCT iterates GEdg keyed by the edges' addresses, the port in their order
        NCollection_IndexedMap<Shape> GEdg = new(ShapeHasher.Instance);
        NCollection_Map<Shape> GVtx = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator itl = new(), itl2 = new();

        for (itl.Initialize(ledges); itl.More(); itl.Next())
        {
            Edge edg = ShapeCasts.Edge(itl.Value());

            GEdg.Add(edg);
            for (exp2.Init(edg, ShapeType.Vertex); exp2.More(); exp2.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp2.Current());
                if (!GVtx.Contains(vtx))
                {
                    GVtx.Add(vtx);
                }
            }
            for (exp2.Init(myShape, ShapeType.Face); exp2.More(); exp2.Next())
            {
                Face fac = ShapeCasts.Face(exp2.Current());
                for (exp3.Init(fac, ShapeType.Edge); exp3.More(); exp3.Next())
                {
                    if (exp3.Current().IsSame(edg) && exp3.Current().Orientation() == edg.Orientation())
                    {
                        theLeft.Add(fac);
                        NCollection_List<Shape> emptylist = new();
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

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theEFMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(myShape, ShapeType.Edge, ShapeType.Face, theEFMap);

        // OCCT iterates these maps keyed by the shapes' addresses; the port in insertion order
        NCollection_DataMap<Shape, NCollection_List<Shape>> theEEMap = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_List<Shape>> theFFMap = new(ShapeHasher.Instance);
        NCollection_Map<Shape> toRemove = new(ShapeHasher.Instance);

        // search for face fusions
        for (int itm = 1; itm <= GEdg.Extent(); itm++)
        {
            Edge edg = ShapeCasts.Edge(GEdg[itm]);
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
                Face fac = ShapeCasts.Face(itl2.Value());
                Face facbis = Generated(G, edg);
                if (ToFuse(fac, facbis))
                {
                    // On recherche si une face a deja fusionne avec facbis
                    bool facbisfound = false;
                    foreach (KeyValuePair<Shape, NCollection_List<Shape>> itf in theFFMap)
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
                            NCollection_List<Shape> thelist = new();
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

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itf in theFFMap)
        {
            for (itl.Initialize(itf.Value); itl.More(); itl.Next())
            {
                for (exp.Init(itl.Value(), ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge ed = ShapeCasts.Edge(exp.Current());
                    if (toRemove.Contains(ed))
                    {
                        continue;
                    }
                    for (itl2.Initialize(itf.Value); itl2.More(); itl2.Next())
                    {
                        if (!itl2.Value().IsSame(itl.Value()))
                        {
                            for (exp2.Init(itl2.Value(), ShapeType.Edge); exp2.More(); exp2.Next())
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

        NCollection_List<Shape> RebuildFace = new();
        NCollection_Map<Shape> mapTreated = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, Shape> DontFuse = new(ShapeHasher.Instance);
        Orientation orient, orface;

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itf in theFFMap)
        {
            Face fac = ShapeCasts.Face(itf.Key);
            for (exp.Init(fac, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                if (mapTreated.Contains(edg))
                {
                    continue; // on saute l`edge
                }

                mapTreated.Add(edg);
                for (exp2.Init(edg, ShapeType.Vertex); exp2.More(); exp2.Next())
                {
                    Vertex vtx = ShapeCasts.Vertex(exp2.Current());
                    if (GVtx.Contains(vtx))
                    {
                        Edge edgbis = Generated(G, vtx);

                        if ((edgbis.IsNull || Tool.Degenerated(edgbis)) ||
                            //	      toRemove.Contains(edgbis) ||
                            !ToFuse(edg, edgbis))
                        {
                            continue;
                        }
                        // a voir
                        if (BrepTools.IsReallyClosed(edg, fac))
                        {
                            if (!theEEMap.IsBound(edg))
                            {
                                NCollection_List<Shape> thelist1 = new();
                                theEEMap.Bind(edg, thelist1);
                                theEEMap[edg].Append(edgbis);
                                toRemove.Add(edgbis); // toujours vrai pour edge double
                                bool FuseEdge = true;
                                Exploration.Vertices(edg, out Vertex Vf, out Vertex Vl);
                                bool ConnectLast = (Vl.IsSame(vtx));
                                for (exp3.Init(fac.Oriented(Forward), ShapeType.Edge); exp3.More(); exp3.Next())
                                {
                                    Edge eee = ShapeCasts.Edge(exp3.Current());
                                    orient = eee.Orientation();
                                    if (!eee.IsSame(edg))
                                    {
                                        Exploration.Vertices(eee, out Vf, out Vl);
                                        if ((Vf.IsSame(vtx) || Vl.IsSame(vtx)) && !toRemove.Contains(eee))
                                        {
                                            FuseEdge = false;
                                            // On recherche celui qu`il ne faut pas fusionner

                                            if ((Vf.IsSame(vtx) && orient == Forward)
                                                || (Vl.IsSame(vtx) && orient == Reversed))
                                            {
                                                if (ConnectLast)
                                                {
                                                    DontFuse.Bind(edg, fac.Oriented(Forward));
                                                }
                                                else
                                                {
                                                    DontFuse.Bind(edg, fac.Oriented(Reversed));
                                                }
                                            }
                                            else
                                            {
                                                if (ConnectLast)
                                                {
                                                    DontFuse.Bind(edg, fac.Oriented(Reversed));
                                                }
                                                else
                                                {
                                                    DontFuse.Bind(edg, fac.Oriented(Forward));
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
                                NCollection_List<Shape> thelist2 = new();
                                theEEMap.Bind(edg, thelist2);
                            }
                            theEEMap[edg].Append(edgbis);
                            NCollection_List<Shape> L = theEEMap[edg];
                            NCollection_List<Shape>.Iterator Lit = new(L);
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
                                            FuseEdge = ToFuse(edg, ShapeCasts.Face(itl.Value()), vtx, toRemove);
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
            NCollection_List<Shape> thelist3 = new();
            theFFMap.Bind(itl.Value(), thelist3);
        }

        BrepBuilder B = new();
        Face newface;
        Edge newedg;
        Vertex newvtx = new();
        Placement loc;
        double tol = 0.0, prm = 0.0, f, l, Uminc = 0.0, Umaxc = 0.0;
        Point2D pf, pl;

        Surface S;
        PlaneSurface? P;
        Curve3D C = null!;

        // Fusion des edges
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> ite in theEEMap)
        {
            bool KeepNewEdge = false;
            Edge edg = ShapeCasts.Edge(ite.Key);
            Tool.Range(edg, out f, out l);
            Shape aLocalEdge = edg.EmptyCopied();
            newedg = ShapeCasts.Edge(aLocalEdge);
            newedg.Orientation(Forward);
            for (exp.Init(edg.Oriented(Forward), ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                prm = Tool.Parameter(vtx, edg);

                newvtx.Nullify();
                for (itl.Initialize(theEEMap[edg]); itl.More(); itl.Next())
                {
                    Edge edgbis = ShapeCasts.Edge(itl.Value());
                    for (exp2.Init(edgbis, ShapeType.Vertex); exp2.More(); exp2.Next())
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
                                newvtx = ShapeCasts.Vertex(exp2.Current());
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
                    tol = Tool.Tolerance(newvtx);
                    B.UpdateVertex(newvtx, prm, newedg, tol);
                    toRemove.Add(itl.Value()); // i-e edgbis
                    KeepNewEdge = true;
                }
                else
                {
                    B.Add(newedg, vtx.Oriented(vtx.Orientation()));
                    tol = Tool.Tolerance(vtx);
                    B.UpdateVertex(vtx, prm, newedg, tol);
                }
            }
            if (KeepNewEdge)
            {
                NCollection_List<Shape> emptylist = new();
                if (!myModShapes.IsBound(edg))
                {
                    myModShapes.Bind(edg, emptylist);
                }
                myModShapes[edg].Append(newedg);
                toRemove.Add(edg);
            }
        }

        NCollection_Map<Shape> EdgAdded = new(ShapeHasher.Instance);

        // Fusion des faces, ou reconstruction
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itf in theFFMap)
        {
            Face fac = ShapeCasts.Face(itf.Key);
            bool ModFace = false;
            NCollection_List<Shape> listofedg = new();

            EdgAdded.Clear();

            for (exp.Init(myShape, ShapeType.Face); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(fac))
                {
                    break;
                }
            }
            orface = exp.Current().Orientation();
            Shape aLocalFaceEmptyCopied = fac.EmptyCopied();
            newface = ShapeCasts.Face(aLocalFaceEmptyCopied);
            newface.Orientation(Forward);
            S = Tool.Surface(fac)!;
            if (S.GetType() == typeof(TrimmedSurface))
            {
                S = ((TrimmedSurface)S).BasisSurface();
            }
            P = S as PlaneSurface;
            Wire wir;
            for (exp.Init(fac.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
            {
                wir = ShapeCasts.Wire(exp.Current());
                for (exp2.Init(wir, ShapeType.Edge); exp2.More(); exp2.Next())
                {
                    Edge edg = ShapeCasts.Edge(exp2.Current());
                    if (toRemove.Contains(edg) || myModShapes.IsBound(edg))
                    {
                        break;
                    }
                }
                if (!exp2.More())
                { // wire non modifie
                    for (exp2.Init(wir, ShapeType.Edge); exp2.More(); exp2.Next())
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
                            BrepTools.UVBounds(fac, out Uminc, out Umaxc, out _, out _);
                        }

                        // premier passage : on met les wires non touches des faces
                        // en vis a vis

                        for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                        {
                            Face facbis = ShapeCasts.Face(itl.Value());
                            NCollection_List<Shape> aGFaces = OrientedFaces(G);
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
                                for (exp2.Init(myShape, ShapeType.Face); exp2.More(); exp2.Next())
                                {
                                    if (exp2.Current().IsSame(facbis))
                                    {
                                        facbis.Orientation(exp2.Current().Orientation());
                                        break;
                                    }
                                }
                            }

                            for (exp3.Init(facbis, ShapeType.Wire); exp3.More(); exp3.Next())
                            {
                                for (exp2.Init(exp3.Current(), ShapeType.Edge); exp2.More(); exp2.Next())
                                {
                                    if (toRemove.Contains(exp2.Current()))
                                    {
                                        break;
                                    }
                                }
                                if (!exp2.More())
                                {
                                    Wire theNew = new();
                                    B.MakeWire(ref theNew); // FORWARD
                                    for (exp2.ReInit(); exp2.More(); exp2.Next())
                                    {
                                        Edge edg = ShapeCasts.Edge(exp2.Current());

                                        orient = global::Kerf.Engine.Topology.TopAbs.Compose(orface, edg.Orientation());
                                        listofedg.Append(edg.Oriented(orient));
                                        EdgAdded.Add(edg);
                                        if (P is null)
                                        {
                                            // on met les courbes 2d si on n`est pas sur un plan
                                            // on ne devrait pas avoir de pb d`edge de couture.
                                            tol = Tool.Tolerance(edg);
                                            C = Tool.Curve(edg, out loc, out f, out l)!;
                                            if (!loc.IsIdentity)
                                            {
                                                Geometry3D GG = C.Transformed(loc.Transformation());
                                                C = (Curve3D)GG;
                                            }
                                            if (C.GetType() == typeof(TrimmedCurve3D))
                                            {
                                                C = ((TrimmedCurve3D)C).BasisCurve();
                                            }

                                            Curve2D C2d0 = Translatable(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C, f, l, S, ref tol)!);

                                            // Tentative de recalage dans la facette
                                            pf = C2d0.Value(f);
                                            pl = C2d0.Value(l);
                                            double tttol = global::Kerf.Engine.Foundation.Precision.Angular();
                                            while (Min(pf.X(), pl.X()) >= Umaxc - tttol)
                                            {
                                                C2d0.Translate(new Vector2D(-2.0 * Math.PI, 0));
                                                pf = C2d0.Value(f);
                                                pl = C2d0.Value(l);
                                            }

                                            while (Max(pf.X(), pl.X()) <= Uminc + tttol)
                                            {
                                                C2d0.Translate(new Vector2D(2.0 * Math.PI, 0));
                                                pf = C2d0.Value(f);
                                                pl = C2d0.Value(l);
                                            }

                                            if (!BrepTools.IsReallyClosed(edg, facbis))
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

                    Curve2D? C2d, C2d1 = null;

                    for (exp2.Init(wir, ShapeType.Edge); exp2.More(); exp2.Next())
                    {
                        Edge edg = ShapeCasts.Edge(exp2.Current());
                        orient = edg.Orientation();
                        if (!toRemove.Contains(edg) && !theEEMap.IsBound(edg))
                        {
                            listofedg.Append(edg);
                        }
                        else if (myModShapes.IsBound(edg) || theEEMap.IsBound(edg))
                        {
                            if (myModShapes.IsBound(edg))
                            {
                                newedg = ShapeCasts.Edge(myModShapes[edg].First());
                            }
                            else
                            {
                                newedg = edg;
                            }
                            listofedg.Append(newedg.Oriented(orient));
                            C = Tool.Curve(newedg, out loc, out f, out l)!;
                            if (!loc.IsIdentity)
                            {
                                Geometry3D GG = C.Transformed(loc.Transformation());
                                C = (Curve3D)GG;
                            }
                            if (C.GetType() == typeof(TrimmedCurve3D))
                            {
                                C = ((TrimmedCurve3D)C).BasisCurve();
                            }
                            if (P is null)
                            { // on met les courbes 2d si on n`est pas
                                // sur un plan
                                Orientation oredonfafw = orient;
                                tol = Tool.Tolerance(newedg);
                                Shape aLocalEdge = edg.Oriented(oredonfafw);
                                Shape aLocalFace = fac.Oriented(Forward);
                                C2d = Tool.CurveOnSurface(ShapeCasts.Edge(aLocalEdge), ShapeCasts.Face(aLocalFace), out f, out l);

                                if (!BrepTools.IsReallyClosed(edg, fac))
                                {
                                    B.UpdateEdge(newedg, C2d, newface, tol);
                                }
                                else if (C2d1 is null)
                                {
                                    C2d1 = C2d;
                                }
                                else
                                {
                                    if (orient == Forward)
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
                                if (!BrepTools.IsReallyClosed(edg, fac))
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
                                        if (orient == global::Kerf.Engine.Topology.TopAbs.Reverse(DontFuse[edg].Orientation()))
                                        {
                                            AddPart = true;
                                        }
                                    }
                                }
                            }
                            if (AddPart)
                            {
                                itl2.Initialize(theEEMap[edg]);
                                Vector3D dir1, dir2;
                                for (; itl2.More(); itl2.Next())
                                {
                                    if (EdgAdded.Contains(itl2.Value()))
                                    {
                                        continue;
                                    }
                                    Edge edgbis = ShapeCasts.Edge(itl2.Value());
                                    Iterator it1 = new(newedg), it2 = new();
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
                                        double prmvt = Tool.Parameter(ShapeCasts.Vertex(it1.Value()), newedg);
                                        C.D1(prmvt, out Point3D ptbid, out dir1);

                                        C = Tool.Curve(edgbis, out loc, out f, out l)!;
                                        if (!loc.IsIdentity)
                                        {
                                            Geometry3D GG = C.Transformed(loc.Transformation());
                                            C = (Curve3D)GG;
                                        }
                                        if (C.GetType() == typeof(TrimmedCurve3D))
                                        {
                                            C = ((TrimmedCurve3D)C).BasisCurve();
                                        }
                                        prmvt = Tool.Parameter(ShapeCasts.Vertex(it1.Value()), edgbis);
                                        C.D1(prmvt, out ptbid, out dir2);
                                    }
                                    else
                                    {
                                        dir1 = dir2 = new Vector3D(1, 0, 0);
                                    }
                                    EdgAdded.Add(edgbis);
                                    if (dir1.Dot(dir2) < 0.0)
                                    {
                                        listofedg.Append(edgbis.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(orient)));
                                    }
                                    else
                                    {
                                        listofedg.Append(edgbis.Oriented(orient));
                                    }

                                    if (P is null)
                                    {
                                        // on met les courbes 2d si on n`est pas sur un plan
                                        // C est la courbe de edgbis, f et l s`y rapportent
                                        Curve2D PTC = Translatable(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C, f, l, S, ref tol)!);
                                        if (S.IsUPeriodic)
                                        {
                                            double Uref;
                                            if (DontFuse.IsBound(edg))
                                            {
                                                Orientation oredge = DontFuse[edg].Orientation();
                                                if (myModShapes.IsBound(edg))
                                                { // edge raccourci...
                                                    Shape aLocalShape = edg.Oriented(oredge);
                                                    Shape aLocalFace = fac.Oriented(Forward);
                                                    Tool.UVPoints(ShapeCasts.Edge(aLocalShape), ShapeCasts.Face(aLocalFace), out pf, out pl);
                                                }
                                                else
                                                {
                                                    Shape aLocalShape = edg.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(oredge));
                                                    Shape aLocalFace = fac.Oriented(Forward);
                                                    Tool.UVPoints(ShapeCasts.Edge(aLocalShape), ShapeCasts.Face(aLocalFace), out pf, out pl);
                                                }
                                                Uref = pf.X();
                                            }
                                            else
                                            {
                                                Tool.UVPoints(edg, fac, out pf, out pl);
                                                Uref = pf.X();
                                            }

                                            double NewU = (PTC.Value(f)).X();

                                            if (Math.Abs(NewU - Uref) > Epsilon(S.UPeriod()))
                                            {
                                                PTC.Translate(new Vector2D((Uref - NewU), 0.0));
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
                        Face facbis = ShapeCasts.Face(itl.Value());
                        bool genface = true;
                        NCollection_List<Shape> aGFaces = OrientedFaces(G);
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
                            for (exp2.Init(myShape, ShapeType.Face); exp2.More(); exp2.Next())
                            {
                                if (exp2.Current().IsSame(facbis))
                                {
                                    facbis.Orientation(exp2.Current().Orientation());
                                    break;
                                }
                            }
                        }

                        for (exp3.Init(facbis, ShapeType.Wire); exp3.More(); exp3.Next())
                        {
                            for (exp2.Init(exp3.Current(), ShapeType.Edge); exp2.More(); exp2.Next())
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
                                        Exploration.Vertices(ShapeCasts.Edge(exp2.Current()), out Vertex VF, out Vertex VL);
                                        Explorer exp4 = new();
                                        for (exp4.Init(wir, ShapeType.Vertex); exp4.More(); exp4.Next())
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
                                Edge edg = ShapeCasts.Edge(exp2.Current());
                                if (!toRemove.Contains(edg) && !EdgAdded.Contains(edg))
                                {
                                    orient = global::Kerf.Engine.Topology.TopAbs.Compose(orface, edg.Orientation());
                                    listofedg.Append(edg.Oriented(orient));
                                    EdgAdded.Add(edg);
                                    if (P is null)
                                    {
                                        // on met les courbes 2d si on n`est pas sur un plan
                                        // on ne devrait pas avoir de pb d`edge de couture.
                                        tol = Tool.Tolerance(edg);
                                        C = Tool.Curve(edg, out loc, out f, out l)!;
                                        if (!loc.IsIdentity)
                                        {
                                            Geometry3D GG = C.Transformed(loc.Transformation());
                                            C = (Curve3D)GG;
                                        }
                                        if (C.GetType() == typeof(TrimmedCurve3D))
                                        {
                                            C = ((TrimmedCurve3D)C).BasisCurve();
                                        }

                                        C2d = Translatable(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C, f, l, S, ref tol)!);

                                        // Tentative de recalage dans la facette
                                        pf = C2d.Value(f);
                                        pl = C2d.Value(l);
                                        double tttol = global::Kerf.Engine.Foundation.Precision.Angular();
                                        while (Min(pf.X(), pl.X()) >= Umaxc - tttol)
                                        {
                                            C2d.Translate(new Vector2D(-2.0 * Math.PI, 0));
                                            pf = C2d.Value(f);
                                            pl = C2d.Value(l);
                                        }

                                        while (Max(pf.X(), pl.X()) <= Uminc + tttol)
                                        {
                                            C2d.Translate(new Vector2D(2.0 * Math.PI, 0));
                                            pf = C2d.Value(f);
                                            pl = C2d.Value(l);
                                        }

                                        if (!BrepTools.IsReallyClosed(edg, facbis))
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
                NCollection_List<Shape> listoffaces = L.NewFaces();
                toRemove.Add(fac);
                myModShapes.Bind(fac, new NCollection_List<Shape>(listoffaces));
                for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                {
                    myModShapes.Bind(itl.Value(), new NCollection_List<Shape>(listoffaces));
                }
            }
        }

        // JAG 16.09.96 : on utilise LocOpe_BuildShape (OCCT's former construction of the shell is commented out)

        // 06.11.96
        // Debug temporaire. Il faudra prevoir un syntaxe de BuildShape
        // qui impose une ori de certaines faces.

        Face FaceRefOri = new();

        NCollection_List<Shape> lfres = new();
        for (exp.Init(myShape, ShapeType.Face); exp.More(); exp.Next())
        {
            Face fac = ShapeCasts.Face(exp.Current());
            if (!theLeft.Contains(fac))
            {
                if (!toRemove.Contains(exp.Current()))
                {
                    lfres.Append(fac);
                    if (!myModShapes.IsBound(fac))
                    {
                        NCollection_List<Shape> emptylist = new();
                        myModShapes.Bind(fac, emptylist);
                    }
                    myModShapes[fac].Append(fac);
                    if (FaceRefOri.IsNull)
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

        Orientation orsolid = myShape.Orientation();
        for (itl.Initialize(OrientedFaces(G)); itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());
            if (toRemove.Contains(fac))
            {
                continue;
            }

            if (orsolid == Forward)
            {
                lfres.Append(fac);
            }
            else
            {
                lfres.Append(fac.Reversed());
            }
            if (!myModShapes.IsBound(fac))
            {
                NCollection_List<Shape> emptylist = new();
                myModShapes.Bind(fac, emptylist);
            }
            myModShapes[fac].Append(fac);
        }

        LocOpe_BuildShape BS = new(lfres);
        myRes = BS.InternalShape();
        // Suite debug du 06.11.96
        if (myRes.ShapeType() == ShapeType.Solid)
        {
            for (exp.Init(myRes, ShapeType.Face); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(FaceRefOri))
                {
                    break;
                }
            }
            if (exp.More() && exp.Current().Orientation() != FaceRefOri.Orientation())
            {
                // Si un seul Shell, on change son orientation
                Solid NewSol = new();
                B.MakeSolid(ref NewSol);
                exp.Init(myRes, ShapeType.Shell);
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
    public NCollection_List<Shape> DescendantFace(in Face F) =>
        LocOpe_WorkingShapes.Published(InternalDescendantFace(F));

    /// <summary>
    /// <see cref="DescendantFace"/> without freezing, for ported code (of the caller's face or of its working copy).
    /// </summary>
    internal NCollection_List<Shape> InternalDescendantFace(in Face F)
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        return myModShapes[myWork.Copied(F)];
    }

    // OCCT.NET: OCCT translates the projected pcurve in place; a frozen one (shared) is translated on a copy
    private static Curve2D Translatable(Curve2D theCurve) =>
        theCurve.IsFrozen ? (Curve2D)theCurve.Copy() : theCurve;

    private static bool ToFuse(in Face F1, in Face F2)
    {
        if (F1.IsNull || F2.IsNull)
        {
            return false;
        }

        double tollin = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolang = global::Kerf.Engine.Foundation.Precision.Angular();

        Surface S1 = Tool.Surface(F1, out Placement loc1)!;
        Surface S2 = Tool.Surface(F2, out Placement loc2)!;

        if (S1.GetType() == typeof(TrimmedSurface))
        {
            S1 = ((TrimmedSurface)S1).BasisSurface();
        }

        if (S2.GetType() == typeof(TrimmedSurface))
        {
            S2 = ((TrimmedSurface)S2).BasisSurface();
        }

        if (S1.GetType() != S2.GetType())
        {
            return false;
        }

        bool ValRet = false;
        if (S1.GetType() == typeof(PlaneSurface))
        {
            Plane3D pl1 = ((PlaneSurface)S1).Pln();
            Plane3D pl2 = ((PlaneSurface)S2).Pln();

            pl1.Transform(loc1.Transformation());
            pl2.Transform(loc2.Transformation());

            if (pl1.Position().IsCoplanar(pl2.Position(), tollin, tolang))
            {
                ValRet = true;
            }
        }

        return ValRet;
    }

    private static bool ToFuse(in Edge E1, in Edge E2)
    {
        if (E1.IsNull || E2.IsNull)
        {
            return false;
        }

        double tollin = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolang = global::Kerf.Engine.Foundation.Precision.Angular();

        Curve3D C1 = Tool.Curve(E1, out Placement loc1, out _, out _)!;
        if (!loc1.IsIdentity)
        {
            Geometry3D CC1 = C1.Transformed(loc1.Transformation());
            C1 = (Curve3D)CC1;
        }

        Curve3D C2 = Tool.Curve(E2, out Placement loc2, out _, out _)!;
        if (!loc2.IsIdentity)
        {
            Geometry3D CC2 = C2.Transformed(loc2.Transformation());
            C2 = (Curve3D)CC2;
        }

        if (C1.GetType() == typeof(TrimmedCurve3D))
        {
            C1 = ((TrimmedCurve3D)C1).BasisCurve();
        }
        if (C2.GetType() == typeof(TrimmedCurve3D))
        {
            C2 = ((TrimmedCurve3D)C2).BasisCurve();
        }

        if (C1.GetType() != C2.GetType())
        {
            return false;
        }

        bool ValRet = false;
        if (C1.GetType() == typeof(LineCurve3D))
        {
            Line3D li1 = ((LineCurve3D)C1).Lin();
            Line3D li2 = ((LineCurve3D)C2).Lin();

            if (li1.Position().IsCoaxial(li2.Position(), tolang, tollin))
            {
                ValRet = true;
            }
        }

        return ValRet;
    }

    private static bool ToFuse(in Edge E, in Face F, in Vertex V, NCollection_Map<Shape> toRemove)
    {
        Explorer exp = new();
        for (exp.Init(F, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge eee = ShapeCasts.Edge(exp.Current());
            if (!eee.IsSame(E))
            {
                Exploration.Vertices(eee, out Vertex Vf, out Vertex Vl);
                if ((Vf.IsSame(V) || Vl.IsSame(V)) && !toRemove.Contains(eee))
                {
                    return false;
                }
            }
        }
        return true;
    }

    private static double NewParameter(in Edge Edg, in Vertex Vtx, in Edge NewEdg, in Vertex NewVtx)
    {
        Point3D P = Tool.Pnt(NewVtx);

        Curve3D C = Tool.Curve(Edg, out Placement loc, out _, out _)!;
        if (!loc.IsIdentity)
        {
            Geometry3D GG = C.Transformed(loc.Transformation());
            C = (Curve3D)GG;
        }
        if (C.GetType() == typeof(TrimmedCurve3D))
        {
            C = ((TrimmedCurve3D)C).BasisCurve();
        }

        if (C.GetType() == typeof(LineCurve3D))
        {
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(((LineCurve3D)C).Lin(), P);
        }
        else if (C.GetType() == typeof(CircleCurve3D))
        {
            double prm = global::Kerf.Engine.Geometry.ElCLib.Parameter(((CircleCurve3D)C).Circ(), P);
            // Vtx vient d`une exploration de Edg orientee FORWARD

            Orientation orient = global::Kerf.Engine.Topology.TopAbs.Reverse(Vtx.Orientation());
            if (orient == Forward || orient == Reversed)
            {
                Explorer exp = new(NewEdg.Oriented(Forward), ShapeType.Vertex);
                for (; exp.More(); exp.Next())
                {
                    if (exp.Current().Orientation() == orient)
                    {
                        break;
                    }
                }
                if (exp.More())
                {
                    double prmmax = Tool.Parameter(ShapeCasts.Vertex(exp.Current()), NewEdg);
                    if (Math.Abs(prmmax - prm) <= Epsilon(2.0 * Math.PI))
                    {
                        if (orient == Reversed)
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
