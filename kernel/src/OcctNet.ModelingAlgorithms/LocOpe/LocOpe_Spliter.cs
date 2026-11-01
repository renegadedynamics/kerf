// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Spliter.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Spliter.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Spliter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Splits a shape by the edges and wires of a <see cref="LocOpe_WiresOnShape"/> lying on it, and tells the faces on
/// their left.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, the splitting writes into the shape and the splitting edges (vertex tolerances and point
/// representations, continuities). It works on the wires-on-shape's working shapes, the copies of the frozen ones
/// (decision 8901); at the end of <see cref="Perform"/> the copies left unchanged are replaced by the caller's own
/// sub-shapes. The shapes it hands out are frozen; ported callers (LocOpe_Gluer) read the <c>Internal</c> forms, and
/// end the working shapes themselves.
/// </para>
/// </remarks>
public sealed class LocOpe_Spliter
{
    private TopoDS_Shape myShape = new();
    private bool myDone;
    private TopoDS_Shape myRes = new();
    // OCCT iterates this map keyed by the shapes' addresses; each entry is handled alone (the order isn't observable)
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_List<TopoDS_Shape> myDLeft = new();
    private readonly NCollection_List<TopoDS_Shape> myLeft = new();
    private LocOpe_WorkingShapes? myWork;
    private TopoDS_Shape myInput = new();

    /// <summary>Empty constructor.</summary>
    public LocOpe_Spliter()
    {
        myDone = false;
    }

    /// <summary>Creates the algorithm on the shape <paramref name="S"/>.</summary>
    public LocOpe_Spliter(in TopoDS_Shape S)
    {
        myInput = S;
        myShape = S;
        myDone = false;
    }

    /// <summary>Initializes the algorithm on the shape <paramref name="S"/>.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myInput = S;
        myShape = S;
        myDone = false;
    }

    /// <summary>
    /// Splits the shape by the edges of <paramref name="PW"/>, which must be on the same shape. Ends the working
    /// shapes: the result shares the caller's unchanged sub-shapes.
    /// </summary>
    public void Perform(LocOpe_WiresOnShape PW)
    {
        InternalPerform(PW);
        if (!myDone || myWork is null || !myWork.IsCopied())
        {
            return;
        }

        var aShapes = new List<TopoDS_Shape> { myRes };
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myMap)
        {
            aShapes.AddRange(anItem.Value);
        }
        aShapes.AddRange(myLeft);
        myWork.Finish(aShapes);
        myRes = myWork.Final(myRes);
        foreach (TopoDS_Shape aKey in myMap.Keys.ToList())
        {
            myMap[aKey] = myWork.Final(myMap[aKey]);
        }
        NCollection_List<TopoDS_Shape> aDLeft = myWork.Final(myDLeft), aLeft = myWork.Final(myLeft);
        myDLeft.Assign(aDLeft);
        myLeft.Assign(aLeft);
    }

    /// <summary>
    /// <see cref="Perform"/> for ported operations that own the working shapes (LocOpe_Gluer): the result is left
    /// unfrozen, on the working shapes, which the caller ends.
    /// </summary>
    internal void InternalPerform(LocOpe_WiresOnShape PW)
    {
        if (myShape.IsNull())
        {
            throw new Standard_NullObject();
        }
        myDone = false;
        myMap.Clear();
        myRes.Nullify();
        // OCCT.NET: the shape as the wires-on-shape works on it (its copy if it was frozen)
        myWork = PW.WorkingShapes();
        myShape = myWork.Working(myInput);

        Put(myShape, myMap);

        NCollection_Map<TopoDS_Shape> mapV = new(TopTools_ShapeMapHasher.Instance), mapE = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> EdgOnEdg = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mapFE = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new(), exp2 = new();

        // 1ere etape : substitution des vertex

        NCollection_List<TopoDS_Shape> lsubs = new();
        BRepTools_Substitution theSubs = new();
        BRep_Builder BB = new();

        for (PW.InitEdgeIterator(); PW.MoreEdge(); PW.NextEdge())
        {
            TopoDS_Edge edg = PW.InternalEdge();
            mapE.Add(edg);
            for (exp.Init(edg, TopAbs_VERTEX); exp.More(); exp.Next())
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(exp.Current());
                if (!mapV.Contains(vtx))
                {
                    if (PW.InternalOnVertex(vtx, out TopoDS_Vertex Vb))
                    {
                        mapV.Add(vtx);
                        if (vtx.IsSame(Vb))
                        {
                            continue;
                        }
                        lsubs.Clear();
                        TopoDS_Vertex vsub = TopoDS.Vertex(vtx.Oriented(TopAbs_FORWARD));
                        gp_Pnt p1 = BRep_Tool.Pnt(vsub), p2 = BRep_Tool.Pnt(Vb);
                        double d = p1.Distance(p2);
                        d = d + BRep_Tool.Tolerance(Vb);
                        BB.UpdateVertex(vsub, d);
                        lsubs.Append(vsub);
                        theSubs.Substitute(Vb.Oriented(TopAbs_FORWARD), lsubs);
                    }
                }
            }
        }

        theSubs.Build(myShape);
        if (theSubs.IsCopied(myShape))
        {
            // on n`a fait que des substitutions de vertex. Donc chaque element
            // est remplace par lui meme ou par un seul element du meme type.
            foreach (TopoDS_Shape aKey in myMap.Keys.ToList())
            {
                if (theSubs.IsCopied(aKey))
                {
                    NCollection_List<TopoDS_Shape> lsub = theSubs.InternalCopy(aKey);
                    // OCCT_DEBUG check left out (Standard_ConstructionError if lsub.Extent() != 1)
                    myMap[aKey].Clear();
                    myMap[aKey].Append(lsub.First());
                }
            }
        }

        myRes = myMap[myShape].First();
        LocOpe_SplitShape theCFace = new(myRes, myWork);

        // Adds every vertices lying on an edge of the shape, and prepares
        // work to rebuild wires on each face
        double prm;

        NCollection_Map<TopoDS_Shape> theFacesWithSection = new(TopTools_ShapeMapHasher.Instance);
        for (PW.InitEdgeIterator(); PW.MoreEdge(); PW.NextEdge())
        {
            TopoDS_Edge edg = PW.InternalEdge();
            for (exp.Init(edg, TopAbs_VERTEX); exp.More(); exp.Next())
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(exp.Current());
                if (!mapV.Contains(vtx))
                {
                    mapV.Add(vtx);
                    if (PW.InternalOnEdge(vtx, edg, out TopoDS_Edge Ed, out prm))
                    {
                        // on devrait verifier que le vtx n`existe pas deja sur l`edge
                        if (!myMap.IsBound(Ed))
                        {
                            continue;
                        }
                        Ed = TopoDS.Edge(myMap[Ed].First());
                        theCFace.InternalAdd(vtx, prm, Ed);
                    }
                }
            }
            if (PW.InternalOnEdge(out TopoDS_Edge Ebis))
            {
                EdgOnEdg.Bind(edg, Ebis);
            }
            else
            {
                TopoDS_Face fac = PW.InternalOnFace();
                if (!myMap.IsBound(fac))
                {
                    continue;
                }
                bool IsFaceWithSec = PW.InternalIsFaceWithSection(fac);
                fac = TopoDS.Face(myMap[fac].First());
                if (IsFaceWithSec)
                {
                    theFacesWithSection.Add(fac);
                }
                if (!mapFE.Contains(fac))
                {
                    NCollection_List<TopoDS_Shape> thelist = new();
                    mapFE.Add(fac, thelist);
                }
                mapFE.ChangeFromKey(fac).Append(edg);
            }
        }

        // Rebuilds wires on each face of the shape

        NCollection_List<TopoDS_Shape>.Iterator itl = new();
        for (int i1 = 1; i1 <= mapFE.Extent(); i1++)
        {
            TopoDS_Face fac = TopoDS.Face(mapFE.FindKey(i1));
            //  Modified by skv - Mon May 31 12:32:54 2004 OCC5865 Begin
            RebuildWires(ref mapFE[i1], PW);
            NCollection_List<TopoDS_Shape> ledges = mapFE[i1];
            //  Modified by skv - Mon May 31 12:32:54 2004 OCC5865 End
            if (theFacesWithSection.Contains(fac))
            {
                theCFace.InternalAdd(ledges, fac);
            }
            else
            {
                for (itl.Initialize(ledges); itl.More(); itl.Next())
                {
                    theCFace.InternalAdd(TopoDS.Wire(itl.Value()), fac);
                }
            }
        }

        // Mise a jour des descendants

        foreach (TopoDS_Shape sori in myMap.Keys.ToList())
        {
            TopoDS_Shape scib = myMap[sori].First();
            myMap[sori] = theCFace.InternalDescendantShapes(scib);
        }

        NCollection_List<TopoDS_Shape> lres = myMap[myShape];

        TopAbs_ShapeEnum typS = myShape.ShapeType();
        if (typS == TopAbs_FACE && lres.Extent() >= 2)
        {
            BRep_Builder B = new();
            myRes.Nullify();
            TopoDS_Shell aShell = new();
            B.MakeShell(ref aShell);
            myRes = aShell;
            myRes.Orientation(TopAbs_FORWARD);
            for (itl.Initialize(lres); itl.More(); itl.Next())
            {
                B.Add(myRes, itl.Value().Oriented(myShape.Orientation()));
            }
        }
        else if (typS == TopAbs_EDGE && lres.Extent() >= 2)
        {
            BRep_Builder B = new();
            myRes.Nullify();
            TopoDS_Wire aWire = new();
            B.MakeWire(ref aWire);
            myRes = aWire;
            myRes.Orientation(TopAbs_FORWARD);
            for (itl.Initialize(lres); itl.More(); itl.Next())
            {
                B.Add(myRes, itl.Value().Oriented(myShape.Orientation()));
            }
        }
        else
        {
            if (lres.Extent() != 1)
            {
                return;
            }
            myRes = lres.First();
        }

        theSubs.Clear();
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> itee in EdgOnEdg)
        {
            TopoDS_Edge e1 = TopoDS.Edge(itee.Key);
            // on recherche dans les descendants de e2 l`edge qui correspont a e1

            TopExp.Vertices(e1, out TopoDS_Vertex vf1, out TopoDS_Vertex vl1);
            lsubs.Clear();
            for (itl.Initialize(myMap[itee.Value]); itl.More(); itl.Next())
            {
                TopoDS_Edge e2 = TopoDS.Edge(itl.Value());
                TopExp.Vertices(e2, out TopoDS_Vertex vf2, out TopoDS_Vertex vl2);

                if (!vl1.IsSame(vf1))
                {
                    if (vf1.IsSame(vf2) && vl1.IsSame(vl2))
                    {
                        lsubs.Append(e2.Oriented(TopAbs_FORWARD));
                        //	break;
                    }
                    else if (vf1.IsSame(vl2) && vl1.IsSame(vf2))
                    {
                        lsubs.Append(e2.Oriented(TopAbs_REVERSED));
                        //	break;
                    }
                }
                else
                { // discrimination sur les tangentes
                    if (vf2.IsSame(vl2) && vl2.IsSame(vl1))
                    { // tout au meme point
                        Geom_Curve C = BRep_Tool.Curve(e1, out TopLoc_Location Loc, out double f, out _)!;
                        C.D1(f, out gp_Pnt pbid, out gp_Vec v1);
                        v1.Transform(Loc.Transformation());

                        C = BRep_Tool.Curve(e2, out Loc, out f, out _)!;
                        C.D1(f, out pbid, out gp_Vec v2);
                        v2.Transform(Loc.Transformation());
                        if (v1.Dot(v2) > 0.0)
                        {
                            lsubs.Append(e2.Oriented(TopAbs_FORWARD));
                        }
                        else
                        {
                            lsubs.Append(e2.Oriented(TopAbs_REVERSED));
                        }
                    }
                }
            }
            if (lsubs.Extent() >= 2)
            { // il faut faire un choix
                Select(e1, lsubs);
            }
            if (lsubs.Extent() == 1)
            {
                TopoDS_Shape ebase = lsubs.First();
                lsubs.Clear();
                lsubs.Append(e1.Oriented(ebase.Orientation()));
                theSubs.Substitute(ebase, lsubs);
            }
            else
            {
                // OCCT_DEBUG trace left out ("Pb pour substitution")
            }
        }

        theSubs.Build(myRes);

        foreach (TopoDS_Shape aKey in myMap.Keys.ToList())
        {
            NCollection_List<TopoDS_Shape> ldesc = myMap[aKey];
            NCollection_List<TopoDS_Shape> newdesc = new();
            for (itl.Initialize(ldesc); itl.More(); itl.Next())
            {
                if (theSubs.IsCopied(itl.Value()))
                {
                    NCollection_List<TopoDS_Shape> lsub = theSubs.InternalCopy(itl.Value());
                    // OCCT_DEBUG check left out (Standard_ConstructionError if lsub.Extent() != 1)
                    newdesc.Append(lsub.First());
                }
                else
                {
                    newdesc.Append(itl.Value());
                }
            }
            myMap[aKey] = newdesc;
        }

        if (theSubs.IsCopied(myRes))
        {
            myRes = theSubs.InternalCopy(myRes).First();
        }

        ////remove superfluous vertices on degenerated edges
        theSubs.Clear();
        NCollection_IndexedMap<TopoDS_Shape> Emap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(myRes, TopAbs_EDGE, Emap);
        NCollection_Sequence<TopoDS_Shape> DegEdges = new();
        int i, j;
        for (i = 1; i <= Emap.Extent(); i++)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(Emap[i]);
            if (BRep_Tool.Degenerated(anEdge))
            {
                DegEdges.Append(anEdge);
            }
        }

        NCollection_Sequence<TopoDS_Shape> DegWires = new();
        for (;;)
        {
            if (DegEdges.IsEmpty())
            {
                break;
            }
            TopoDS_Wire aDegWire = new();
            BB.MakeWire(ref aDegWire);
            BB.Add(aDegWire, DegEdges[1]);
            DegEdges.Remove(1);
            for (;;)
            {
                TopExp.Vertices(aDegWire, out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast);
                bool found = false;
                for (i = 1; i <= DegEdges.Length(); i++)
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(DegEdges[i]);
                    TopExp.Vertices(anEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    if (V1.IsSame(Vfirst) || V1.IsSame(Vlast) || V2.IsSame(Vfirst) || V2.IsSame(Vlast))
                    {
                        BB.Add(aDegWire, anEdge);
                        DegEdges.Remove(i);
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    break;
                }
            }
            DegWires.Append(aDegWire);
        }

        for (i = 1; i <= DegWires.Length(); i++)
        {
            NCollection_IndexedMap<TopoDS_Shape> Vmap = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(DegWires[i], TopAbs_VERTEX, Vmap);
            NCollection_List<TopoDS_Shape> LV = new();
            LV.Append(Vmap[1].Oriented(TopAbs_FORWARD));
            for (j = 2; j <= Vmap.Extent(); j++)
            {
                if (!Vmap[j].IsSame(Vmap[1]))
                {
                    theSubs.Substitute(Vmap[j], LV);
                }
            }
        }
        theSubs.Build(myRes);
        if (theSubs.IsCopied(myRes))
        {
            myRes = theSubs.InternalCopy(myRes).First();
        }
        ////

        myDLeft.Clear();
        myLeft.Clear();
        mapV.Clear();

        for (exp.Init(myRes, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Face fac = TopoDS.Face(exp.Current());
            bool isFound = false;
            for (exp2.Init(fac, TopAbs_EDGE); exp2.More(); exp2.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(exp2.Current());
                foreach (TopoDS_Shape itms in mapE)
                {
                    if (itms.IsSame(edg) && edg.Orientation() == itms.Orientation())
                    {
                        isFound = true;
                        break;
                    }
                }
                if (isFound)
                {
                    break;
                }
            }
            if (exp2.More())
            {
                myDLeft.Append(fac);
                myLeft.Append(fac);
            }
            else
            {
                mapV.Add(fac);
            }
        }

        // Map des edges ou les connexions sont possibles
        // OCCT takes the first edge of this map and the first face of mapV that has it, in the order of their
        // addresses: so the order of Left() natively depends on the run (the port: the maps' order)
        NCollection_Map<TopoDS_Shape> Mapebord = new(TopTools_ShapeMapHasher.Instance);
        for (itl.Initialize(myLeft); itl.More(); itl.Next())
        {
            for (exp.Init(itl.Value(), TopAbs_EDGE); exp.More(); exp.Next())
            {
                if (!mapE.Contains(exp.Current()))
                {
                    if (!Mapebord.Add(exp.Current()))
                    {
                        Mapebord.Remove(exp.Current());
                    }
                }
            }
        }

        while (Mapebord.Extent() != 0)
        {
            TopoDS_Shape edg = new();
            foreach (TopoDS_Shape aFirst in Mapebord)
            {
                edg = aFirst;
                break;
            }

            TopoDS_Shape aFound = new();
            foreach (TopoDS_Shape fac in mapV)
            {
                for (exp.Init(fac, TopAbs_EDGE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(edg))
                    {
                        break;
                    }
                }
                if (exp.More())
                {
                    aFound = fac;
                    break; // face a gauche
                }
            }
            if (!aFound.IsNull())
            {
                TopoDS_Shape fac = aFound;
                for (exp.Init(fac, TopAbs_EDGE); exp.More(); exp.Next())
                {
                    if (!Mapebord.Add(exp.Current()))
                    {
                        Mapebord.Remove(exp.Current());
                    }
                }
                mapV.Remove(fac);
                myLeft.Append(fac);
            }
            else
            {
                Mapebord.Remove(edg);
            }
        }

        myDone = true;
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
    public TopoDS_Shape Shape() => myInput;

    /// <summary>Returns the faces which are the left of the projected wires (frozen).</summary>
    public NCollection_List<TopoDS_Shape> DirectLeft() => LocOpe_WorkingShapes.Published(InternalDirectLeft());

    /// <summary><see cref="DirectLeft"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalDirectLeft()
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        return myDLeft;
    }

    /// <summary>
    /// Returns the faces of the "left" part on the shape, built from <see cref="DirectLeft"/> with the faces connected
    /// to this set, and so on (frozen).
    /// </summary>
    public NCollection_List<TopoDS_Shape> Left() => LocOpe_WorkingShapes.Published(InternalLeft());

    /// <summary><see cref="Left"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalLeft()
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        return myLeft;
    }

    /// <summary>Returns the list of descendant shapes of <paramref name="F"/>, a sub-shape of the caller's shape (frozen).</summary>
    public NCollection_List<TopoDS_Shape> DescendantShapes(in TopoDS_Shape F) =>
        LocOpe_WorkingShapes.Published(InternalDescendantShapes(F));

    /// <summary>
    /// <see cref="DescendantShapes"/> without freezing, for ported code (of the caller's shape or of its working copy).
    /// </summary>
    internal NCollection_List<TopoDS_Shape> InternalDescendantShapes(in TopoDS_Shape F)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        TopoDS_Shape aF = myWork is null ? F : myWork.Copied(F);
        if (myMap.IsBound(aF))
        {
            return myMap[aF];
        }
        else
        {
            return new NCollection_List<TopoDS_Shape>();
        }
    }

    //  Modified by skv - Mon May 31 12:31:39 2004 OCC5865 Begin
    private static void RebuildWires(ref NCollection_List<TopoDS_Shape> ledge, LocOpe_WiresOnShape PW)
    {
        LocOpe_BuildWires theBuild = new(ledge, PW);
        //  Modified by skv - Mon May 31 12:31:40 2004 OCC5865 End
        if (!theBuild.IsDone())
        {
            throw new Standard_ConstructionError();
        }
        ledge = new NCollection_List<TopoDS_Shape>(theBuild.InternalResult());
    }

    private static void Put(in TopoDS_Shape S, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap)
    {
        if (theMap.IsBound(S))
        {
            return;
        }
        NCollection_List<TopoDS_Shape> thelist = new();
        theMap.Bind(S, thelist);
        theMap[S].Append(S);
        for (TopoDS_Iterator it = new(S); it.More(); it.Next())
        {
            Put(it.Value(), theMap);
        }
    }

    private static void Select(in TopoDS_Edge Ebase, NCollection_List<TopoDS_Shape> lsubs)
    {
        // Choix d`un point

        double dmin = RealLast();
        int i = 0, imin = 0;

        Geom_Curve C = BRep_Tool.Curve(Ebase, out TopLoc_Location Loc, out double f, out double l)!;

        if (!Loc.IsIdentity())
        {
            Geom_Geometry GG = C.Transformed(Loc.Transformation());
            C = (Geom_Curve)GG;
        }
        gp_Pnt Pt = C.Value((f + l) / 2.0);

        GeomAPI_ProjectPointOnCurve proj = new();
        NCollection_List<TopoDS_Shape>.Iterator itl = new(lsubs);
        for (; itl.More(); itl.Next())
        {
            i++;
            TopoDS_Edge edg = TopoDS.Edge(itl.Value());
            C = BRep_Tool.Curve(edg, out Loc, out f, out l)!;
            if (!Loc.IsIdentity())
            {
                Geom_Geometry GG = C.Transformed(Loc.Transformation());
                C = (Geom_Curve)GG;
            }
            proj.Init(Pt, C, f, l);
            if (proj.NbPoints() > 0)
            {
                if (proj.LowerDistance() < dmin)
                {
                    imin = i;
                    dmin = proj.LowerDistance();
                }
            }
        }
        if (imin == 0)
        {
            lsubs.Clear();
        }
        else
        {
            itl.Initialize(lsubs);
            i = 1;
            while (i < imin)
            {
                lsubs.Remove(itl);
                i++;
            }
            itl.Next();
            while (itl.More())
            {
                lsubs.Remove(itl);
            }
        }
    }
}
