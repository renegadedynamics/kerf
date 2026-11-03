// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>Builds shells and solids from a list of faces.</summary>
/// <remarks>
/// A working object (decision 21). It builds new shells and solids around the faces it is given and never writes into
/// them. <see cref="Shape"/> freezes what it hands out; ported callers read <see cref="InternalShape"/>.
/// </remarks>
public sealed class LocOpe_BuildShape
{
    private TopoDS_Shape myRes = new();

    public LocOpe_BuildShape()
    {
    }

    /// <summary>Builds shape(s) from the list <paramref name="L"/>. Uses only the faces of <paramref name="L"/>.</summary>
    public LocOpe_BuildShape(NCollection_List<TopoDS_Shape> L)
    {
        Perform(L);
    }

    /// <summary>Builds shape(s) from the list <paramref name="L"/>. Uses only the faces of <paramref name="L"/>.</summary>
    public void Perform(NCollection_List<TopoDS_Shape> L)
    {
        int i;
        int j;
        int k;
        myRes.Nullify();

        TopoDS_Compound C = new();
        BRep_Builder B = new();
        B.MakeCompound(ref C);

        NCollection_IndexedMap<TopoDS_Shape> mapF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator itl = new();

        for (itl.Initialize(L); itl.More(); itl.Next())
        {
            if (itl.Value().ShapeType() == TopAbs_FACE && !mapF.Contains(itl.Value()))
            {
                mapF.Add(itl.Value());
                B.Add(C, itl.Value());
            }
        }

        if (mapF.Extent() == 0)
        {
            return; // no face
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapEF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(C, TopAbs_EDGE, TopAbs_FACE, theMapEF);

        NCollection_IntMap mapI = new(), mapIf = new();
        int Nbedges = theMapEF.Extent();

        NCollection_List<TopoDS_Shape> lshell = new();
        NCollection_List<TopoDS_Shape> lresult = new();

        do
        {
            // Recherche du premier edge non traite
            for (i = 1; i <= Nbedges; i++)
            {
                if (!mapI.Contains(i))
                {
                    break;
                }
            }
            if (i <= Nbedges)
            {
                mapF.Clear();
                mapIf.Clear();
                Add(i, mapI, mapF, theMapEF);
                bool Manifold = true;
                TopoDS_Shape FaceRef = new();
                TopAbs_Orientation orient;

                for (j = 1; j <= mapF.Extent(); j++)
                {
                    orient = mapF[j].Orientation();
                    if (orient == TopAbs_INTERNAL || orient == TopAbs_EXTERNAL)
                    {
                        Manifold = false;
                    }
                    else if (FaceRef.IsNull())
                    {
                        FaceRef = mapF[j];
                    }
                    mapIf.Add(j);
                }

                TopoDS_Shell newSh = new();
                B.MakeShell(ref newSh);
                if (!Manifold && FaceRef.IsNull())
                {
                    // on a un paquet de faces. pas d'orientation possible ?
                    for (j = 1; j <= mapF.Extent(); j++)
                    {
                        B.Add(newSh, mapF[j]);
                    }
                }
                else
                {
                    // orienter ce qu`on peut
                    if (!Manifold)
                    {
                        for (j = 1; j <= mapF.Extent(); j++)
                        {
                            if (mapF[j].Orientation() == TopAbs_INTERNAL || mapF[j].Orientation() == TopAbs_EXTERNAL)
                            {
                                B.Add(newSh, mapF[j]);
                                mapIf.Remove(j);
                            }
                        }
                    }

                    B.Add(newSh, FaceRef);
                    Propagate(FaceRef, newSh, mapF, mapIf);
                }
                newSh.Closed(BRep_Tool.IsClosed(newSh));
                if (!Manifold)
                {
                    lshell.Append(newSh.Oriented(TopAbs_INTERNAL));
                }
                else
                {
                    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapEFbis = new(TopTools_ShapeMapHasher.Instance);
                    TopExp.MapShapesAndAncestors(newSh, TopAbs_EDGE, TopAbs_FACE, theMapEFbis);
                    for (k = 1; k <= theMapEFbis.Extent(); k++)
                    {
                        TopoDS_Edge Ed = TopoDS.Edge(theMapEFbis.FindKey(k));
                        TopAbs_Orientation OriEd = Ed.Orientation();
                        if (OriEd != TopAbs_INTERNAL && OriEd != TopAbs_EXTERNAL)
                        {
                            int Nbfac = theMapEFbis[k].Extent();
                            if (Nbfac > 2)
                            { // peu probable
                                break;
                            }
                            else if (Nbfac == 1)
                            {
                                if (!BRep_Tool.Degenerated(Ed))
                                {
                                    break;
                                }
                            }
                        }
                    }
                    if (k > theMapEFbis.Extent())
                    {
                        TopoDS_Solid newSo = new();
                        B.MakeSolid(ref newSo);
                        B.Add(newSo, newSh); // newSh est FORWARD
                        BRepClass3d_SolidClassifier Class = new(newSo);
                        Class.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
                        if (Class.State() == TopAbs_IN)
                        {
                            lresult.Append(newSh.Oriented(TopAbs_REVERSED));
                        }
                        else
                        {
                            lresult.Append(newSh);
                        }
                    }
                    else
                    {
                        lshell.Append(newSh.Oriented(TopAbs_INTERNAL));
                    }
                }
            }
        } while (mapI.Extent() < Nbedges);

        // on a une list de shells dans lresult. on suppose qu`ils ne s`intersectent pas.
        // il faut classifier les shells orientes pour en faire des solides...
        // on n`accepte qu`1 niveau d'imbrication

        // OCCT iterates this map keyed by the shells' addresses; the port in insertion order (lresult's)
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> imbSh = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> LIntern = new();

        for (itl.Initialize(lresult); itl.More(); itl.Next())
        {
            TopoDS_Shape sh = itl.Value();
            TopoDS_Solid tempo = new();
            B.MakeSolid(ref tempo);
            B.Add(tempo, sh);

            NCollection_List<TopoDS_Shape> thelist = new();
            imbSh.Bind(sh, thelist);
            NCollection_List<TopoDS_Shape>.Iterator itl2 = new();
            for (itl2.Initialize(lresult); itl2.More(); itl2.Next())
            {
                TopoDS_Shape sh2 = itl2.Value();
                if (!sh2.IsSame(sh))
                {
                    if (IsInside(sh2, tempo))
                    {
                        LIntern.Append(sh2);
                        imbSh[sh].Append(sh2);
                    }
                }
            }
        }

        // LPA 07/10/98: on vire les shells imbriques comme
        // etant aussi des solides a part entiere.
        for (itl.Initialize(LIntern); itl.More(); itl.Next())
        {
            TopoDS_Shape sh = itl.Value();
            if (imbSh.IsBound(sh))
            {
                imbSh.UnBind(sh);
            }
        }

        NCollection_List<TopoDS_Shape> lsolid = new();
        do
        {
            TopoDS_Shape aKey = new();
            NCollection_List<TopoDS_Shape>? aValue = null;
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itdm in imbSh)
            {
                if (itdm.Value.Extent() != 0)
                {
                    aKey = itdm.Key;
                    aValue = itdm.Value;
                    break;
                }
            }
            if (aValue is not null)
            {
                TopoDS_Solid newSo = new();
                B.MakeSolid(ref newSo);
                B.Add(newSo, aKey);
                for (itl.Initialize(aValue); itl.More(); itl.Next())
                {
                    B.Add(newSo, itl.Value().Reversed());
                }
                lsolid.Append(newSo);
                imbSh.UnBind(aKey);
            }
            else
            {
                foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itdm in imbSh)
                {
                    TopoDS_Solid newSo = new();
                    B.MakeSolid(ref newSo);
                    B.Add(newSo, itdm.Key);
                    lsolid.Append(newSo);
                }
                imbSh.Clear();
            }
        } while (imbSh.Extent() != 0);

        int nbsol = lsolid.Extent();
        int nbshl = lshell.Extent();

        if (nbsol == 1 && nbshl == 0)
        {
            myRes = lsolid.First();
        }
        else if (nbsol == 0 && nbshl == 1)
        {
            myRes = lshell.First();
        }
        else
        {
            TopoDS_Compound aRes = new();
            B.MakeCompound(ref aRes);
            myRes = aRes;
            for (itl.Initialize(lsolid); itl.More(); itl.Next())
            {
                B.Add(myRes, itl.Value());
            }
            for (itl.Initialize(lshell); itl.More(); itl.Next())
            {
                B.Add(myRes, itl.Value());
            }
        }
    }

    /// <summary>The shape built (frozen).</summary>
    public TopoDS_Shape Shape()
    {
        myRes.TShape()?.FreezeGraph();
        return myRes;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal TopoDS_Shape InternalShape() => myRes;

    private static void Add(int ind,
                            NCollection_IntMap mapI,
                            NCollection_IndexedMap<TopoDS_Shape> mapF,
                            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mapEF)
    {
        if (!mapI.Add(ind))
        {
            throw new ConstructionErrorException();
        }

        NCollection_List<TopoDS_Shape>.Iterator itl = new(mapEF[ind]);
        for (; itl.More(); itl.Next())
        {
            if (!mapF.Contains(itl.Value()))
            {
                mapF.Add(itl.Value());
                TopExp_Explorer exp = new();
                for (exp.Init(itl.Value(), TopAbs_EDGE); exp.More(); exp.Next())
                {
                    TopoDS_Shape edg = exp.Current();
                    int indedg = mapEF.FindIndex(edg);
                    if (indedg == 0)
                    {
                        throw new ConstructionErrorException();
                    }
                    if (!mapI.Contains(indedg))
                    {
                        Add(indedg, mapI, mapF, mapEF);
                    }
                }
            }
        }
    }

    private static void Propagate(in TopoDS_Shape F,
                                  in TopoDS_Shape Sh,
                                  NCollection_IndexedMap<TopoDS_Shape> mapF,
                                  NCollection_IntMap mapIf)
    {
        BRep_Builder B = new();
        int indf = mapF.FindIndex(F);
        if (!mapIf.Contains(indf))
        {
            return;
        }
        mapIf.Remove(indf);
        if (mapIf.Extent() == 0)
        {
            return;
        }

        TopExp_Explorer exp = new();
        for (exp.Init(F, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Shape edg = exp.Current();

            TopAbs_Orientation ored1 = edg.Orientation(), ored2 = TopAbs_FORWARD;

            if (ored1 == TopAbs_INTERNAL || ored1 == TopAbs_EXTERNAL)
            {
                continue;
            }
            NCollection_IntMap.Iterator itm = new(mapIf);
            for (; itm.More(); itm.Next())
            {
                TopoDS_Shape newF = mapF[itm.Key()];
                TopExp_Explorer exp2 = new(newF, TopAbs_EDGE);
                for (; exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(edg))
                    {
                        break;
                    }
                }
                if (exp2.More())
                {
                    ored2 = exp2.Current().Orientation();
                    break;
                }
            }
            if (itm.More())
            {
                TopoDS_Shape FtoAdd = mapF[itm.Key()];
                bool added = false;
                if (ored2 == ored1)
                {
                    FtoAdd.Reverse();
                    B.Add(Sh, FtoAdd);
                    added = true;
                }
                else if (ored2 == TopAbs.Reverse(ored1))
                {
                    B.Add(Sh, FtoAdd);
                    added = true;
                }
                if (added)
                {
                    Propagate(FtoAdd, Sh, mapF, mapIf);
                }
            }
        }
    }

    private static bool IsInside(in TopoDS_Shape S1, in TopoDS_Shape S2)
    {
        BRepClass3d_SolidClassifier Class = new(S2);
        TopExp_Explorer exp = new();
        for (exp.Init(S1, TopAbs_VERTEX); exp.More(); exp.Next())
        {
            TopoDS_Vertex vtx = TopoDS.Vertex(exp.Current());
            Point3D Pttest = BRep_Tool.Pnt(vtx);
            double Tol = BRep_Tool.Tolerance(vtx);
            Class.Perform(Pttest, Tol);
            if (Class.State() == TopAbs_IN)
            {
                return true;
            }
            else if (Class.State() == TopAbs_OUT)
            {
                return false;
            }
        }
        // OCCT_DEBUG trace left out

        return true;
    }
}
