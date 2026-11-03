// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>Builds shells and solids from a list of faces.</summary>
/// <remarks>
/// A working object (decision 21). It builds new shells and solids around the faces it is given and never writes into
/// them. <see cref="Shape"/> freezes what it hands out; ported callers read <see cref="InternalShape"/>.
/// </remarks>
public sealed class LocOpe_BuildShape
{
    private Shape myRes = new();

    public LocOpe_BuildShape()
    {
    }

    /// <summary>Builds shape(s) from the list <paramref name="L"/>. Uses only the faces of <paramref name="L"/>.</summary>
    public LocOpe_BuildShape(NCollection_List<Shape> L)
    {
        Perform(L);
    }

    /// <summary>Builds shape(s) from the list <paramref name="L"/>. Uses only the faces of <paramref name="L"/>.</summary>
    public void Perform(NCollection_List<Shape> L)
    {
        int i;
        int j;
        int k;
        myRes.Nullify();

        Compound C = new();
        BrepBuilder B = new();
        B.MakeCompound(ref C);

        NCollection_IndexedMap<Shape> mapF = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itl = new();

        for (itl.Initialize(L); itl.More(); itl.Next())
        {
            if (itl.Value().ShapeType() == ShapeType.Face && !mapF.Contains(itl.Value()))
            {
                mapF.Add(itl.Value());
                B.Add(C, itl.Value());
            }
        }

        if (mapF.Extent() == 0)
        {
            return; // no face
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(C, ShapeType.Edge, ShapeType.Face, theMapEF);

        NCollection_IntMap mapI = new(), mapIf = new();
        int Nbedges = theMapEF.Extent();

        NCollection_List<Shape> lshell = new();
        NCollection_List<Shape> lresult = new();

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
                Shape FaceRef = new();
                Orientation orient;

                for (j = 1; j <= mapF.Extent(); j++)
                {
                    orient = mapF[j].Orientation();
                    if (orient == Internal || orient == External)
                    {
                        Manifold = false;
                    }
                    else if (FaceRef.IsNull)
                    {
                        FaceRef = mapF[j];
                    }
                    mapIf.Add(j);
                }

                Shell newSh = new();
                B.MakeShell(ref newSh);
                if (!Manifold && FaceRef.IsNull)
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
                            if (mapF[j].Orientation() == Internal || mapF[j].Orientation() == External)
                            {
                                B.Add(newSh, mapF[j]);
                                mapIf.Remove(j);
                            }
                        }
                    }

                    B.Add(newSh, FaceRef);
                    Propagate(FaceRef, newSh, mapF, mapIf);
                }
                newSh.Closed(Tool.IsClosed(newSh));
                if (!Manifold)
                {
                    lshell.Append(newSh.Oriented(Internal));
                }
                else
                {
                    NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEFbis = new(ShapeHasher.Instance);
                    Exploration.MapShapesAndAncestors(newSh, ShapeType.Edge, ShapeType.Face, theMapEFbis);
                    for (k = 1; k <= theMapEFbis.Extent(); k++)
                    {
                        Edge Ed = ShapeCasts.Edge(theMapEFbis.FindKey(k));
                        Orientation OriEd = Ed.Orientation();
                        if (OriEd != Internal && OriEd != External)
                        {
                            int Nbfac = theMapEFbis[k].Extent();
                            if (Nbfac > 2)
                            { // peu probable
                                break;
                            }
                            else if (Nbfac == 1)
                            {
                                if (!Tool.Degenerated(Ed))
                                {
                                    break;
                                }
                            }
                        }
                    }
                    if (k > theMapEFbis.Extent())
                    {
                        Solid newSo = new();
                        B.MakeSolid(ref newSo);
                        B.Add(newSo, newSh); // newSh est FORWARD
                        BRepClass3d_SolidClassifier Class = new(newSo);
                        Class.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
                        if (Class.State() == In)
                        {
                            lresult.Append(newSh.Oriented(Reversed));
                        }
                        else
                        {
                            lresult.Append(newSh);
                        }
                    }
                    else
                    {
                        lshell.Append(newSh.Oriented(Internal));
                    }
                }
            }
        } while (mapI.Extent() < Nbedges);

        // on a une list de shells dans lresult. on suppose qu`ils ne s`intersectent pas.
        // il faut classifier les shells orientes pour en faire des solides...
        // on n`accepte qu`1 niveau d'imbrication

        // OCCT iterates this map keyed by the shells' addresses; the port in insertion order (lresult's)
        NCollection_DataMap<Shape, NCollection_List<Shape>> imbSh = new(ShapeHasher.Instance);
        NCollection_List<Shape> LIntern = new();

        for (itl.Initialize(lresult); itl.More(); itl.Next())
        {
            Shape sh = itl.Value();
            Solid tempo = new();
            B.MakeSolid(ref tempo);
            B.Add(tempo, sh);

            NCollection_List<Shape> thelist = new();
            imbSh.Bind(sh, thelist);
            NCollection_List<Shape>.Iterator itl2 = new();
            for (itl2.Initialize(lresult); itl2.More(); itl2.Next())
            {
                Shape sh2 = itl2.Value();
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
            Shape sh = itl.Value();
            if (imbSh.IsBound(sh))
            {
                imbSh.UnBind(sh);
            }
        }

        NCollection_List<Shape> lsolid = new();
        do
        {
            Shape aKey = new();
            NCollection_List<Shape>? aValue = null;
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in imbSh)
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
                Solid newSo = new();
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
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in imbSh)
                {
                    Solid newSo = new();
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
            Compound aRes = new();
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
    public Shape Shape()
    {
        myRes.TShape()?.FreezeGraph();
        return myRes;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal Shape InternalShape() => myRes;

    private static void Add(int ind,
                            NCollection_IntMap mapI,
                            NCollection_IndexedMap<Shape> mapF,
                            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapEF)
    {
        if (!mapI.Add(ind))
        {
            throw new ConstructionErrorException();
        }

        NCollection_List<Shape>.Iterator itl = new(mapEF[ind]);
        for (; itl.More(); itl.Next())
        {
            if (!mapF.Contains(itl.Value()))
            {
                mapF.Add(itl.Value());
                Explorer exp = new();
                for (exp.Init(itl.Value(), ShapeType.Edge); exp.More(); exp.Next())
                {
                    Shape edg = exp.Current();
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

    private static void Propagate(in Shape F,
                                  in Shape Sh,
                                  NCollection_IndexedMap<Shape> mapF,
                                  NCollection_IntMap mapIf)
    {
        BrepBuilder B = new();
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

        Explorer exp = new();
        for (exp.Init(F, ShapeType.Edge); exp.More(); exp.Next())
        {
            Shape edg = exp.Current();

            Orientation ored1 = edg.Orientation(), ored2 = Forward;

            if (ored1 == Internal || ored1 == External)
            {
                continue;
            }
            NCollection_IntMap.Iterator itm = new(mapIf);
            for (; itm.More(); itm.Next())
            {
                Shape newF = mapF[itm.Key()];
                Explorer exp2 = new(newF, ShapeType.Edge);
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
                Shape FtoAdd = mapF[itm.Key()];
                bool added = false;
                if (ored2 == ored1)
                {
                    FtoAdd.Reverse();
                    B.Add(Sh, FtoAdd);
                    added = true;
                }
                else if (ored2 == global::Kerf.Engine.Topology.TopAbs.Reverse(ored1))
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

    private static bool IsInside(in Shape S1, in Shape S2)
    {
        BRepClass3d_SolidClassifier Class = new(S2);
        Explorer exp = new();
        for (exp.Init(S1, ShapeType.Vertex); exp.More(); exp.Next())
        {
            Vertex vtx = ShapeCasts.Vertex(exp.Current());
            Point3D Pttest = Tool.Pnt(vtx);
            double Tol = Tool.Tolerance(vtx);
            Class.Perform(Pttest, Tol);
            if (Class.State() == In)
            {
                return true;
            }
            else if (Class.State() == Out)
            {
                return false;
            }
        }
        // OCCT_DEBUG trace left out

        return true;
    }
}
