// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Features.Local;

/// <summary>Builds shells and solids from a list of faces.</summary>
/// <remarks>
/// A working object (decision 21). It builds new shells and solids around the faces it is given and never writes into
/// them. <see cref="Shape"/> freezes what it hands out; ported callers read <see cref="InternalShape"/>.
/// </remarks>
internal sealed class BuildShape
{
    private Shape _res = new();

    public BuildShape()
    {
    }

    /// <summary>Builds shape(s) from the list <paramref name="l"/>. Uses only the faces of <paramref name="l"/>.</summary>
    public BuildShape(NCollection_List<Shape> l)
    {
        Perform(l);
    }

    /// <summary>Builds shape(s) from the list <paramref name="l"/>. Uses only the faces of <paramref name="l"/>.</summary>
    public void Perform(NCollection_List<Shape> l)
    {
        int i;
        int j;
        int k;
        _res.Nullify();

        Compound c = new();
        BrepBuilder b = new();
        b.MakeCompound(ref c);

        IndexedSet<Shape> mapF = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itl = new();

        for (itl.Initialize(l); itl.More(); itl.Next())
        {
            if (itl.Value().Type == ShapeType.Face && !mapF.Contains(itl.Value()))
            {
                mapF.Add(itl.Value());
                b.Add(c, itl.Value());
            }
        }

        if (mapF.Count == 0)
        {
            return; // no face
        }

        IndexedMap<Shape, NCollection_List<Shape>> theMapEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(c, ShapeType.Edge, ShapeType.Face, theMapEF);

        NCollection_IntMap mapI = new(), mapIf = new();
        int nbedges = theMapEF.Count;

        NCollection_List<Shape> lshell = new();
        NCollection_List<Shape> lresult = new();

        do
        {
            // Recherche du premier edge non traite
            for (i = 1; i <= nbedges; i++)
            {
                if (!mapI.Contains(i))
                {
                    break;
                }
            }
            if (i <= nbedges)
            {
                mapF.Clear();
                mapIf.Clear();
                Add(i, mapI, mapF, theMapEF);
                bool manifold = true;
                Shape faceRef = new();
                Orientation orient;

                for (j = 1; j <= mapF.Count; j++)
                {
                    orient = mapF[j - 1].Orientation;
                    if (orient == Internal || orient == External)
                    {
                        manifold = false;
                    }
                    else if (faceRef.IsNull)
                    {
                        faceRef = mapF[j - 1];
                    }
                    mapIf.Add(j);
                }

                Shell newSh = new();
                b.MakeShell(ref newSh);
                if (!manifold && faceRef.IsNull)
                {
                    // on a un paquet de faces. pas d'orientation possible ?
                    for (j = 1; j <= mapF.Count; j++)
                    {
                        b.Add(newSh, mapF[j - 1]);
                    }
                }
                else
                {
                    // orienter ce qu`on peut
                    if (!manifold)
                    {
                        for (j = 1; j <= mapF.Count; j++)
                        {
                            if (mapF[j - 1].Orientation == Internal || mapF[j - 1].Orientation == External)
                            {
                                b.Add(newSh, mapF[j - 1]);
                                mapIf.Remove(j);
                            }
                        }
                    }

                    b.Add(newSh, faceRef);
                    Propagate(faceRef, newSh, mapF, mapIf);
                }
                newSh.Closed(Tool.IsClosed(newSh));
                if (!manifold)
                {
                    lshell.Append(newSh.Oriented(Internal));
                }
                else
                {
                    IndexedMap<Shape, NCollection_List<Shape>> theMapEFbis = new(ShapeHasher.Instance);
                    Exploration.MapShapesAndAncestors(newSh, ShapeType.Edge, ShapeType.Face, theMapEFbis);
                    for (k = 1; k <= theMapEFbis.Count; k++)
                    {
                        Edge ed = ShapeCasts.Edge(theMapEFbis.KeyAt(k - 1));
                        Orientation oriEd = ed.Orientation;
                        if (oriEd != Internal && oriEd != External)
                        {
                            int nbfac = theMapEFbis.ValueAt(k - 1).Extent();
                            if (nbfac > 2)
                            { // peu probable
                                break;
                            }
                            else if (nbfac == 1)
                            {
                                if (!Tool.Degenerated(ed))
                                {
                                    break;
                                }
                            }
                        }
                    }
                    if (k > theMapEFbis.Count)
                    {
                        Solid newSo = new();
                        b.MakeSolid(ref newSo);
                        b.Add(newSo, newSh); // newSh est FORWARD
                        SolidClassifier Class = new(newSo);
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
        } while (mapI.Extent() < nbedges);

        // on a une list de shells dans lresult. on suppose qu`ils ne s`intersectent pas.
        // il faut classifier les shells orientes pour en faire des solides...
        // on n`accepte qu`1 niveau d'imbrication

        // OCCT iterates this map keyed by the shells' addresses; the port in insertion order (lresult's)
        Dictionary<Shape, NCollection_List<Shape>> imbSh = new(ShapeHasher.Instance);
        NCollection_List<Shape> lIntern = new();

        for (itl.Initialize(lresult); itl.More(); itl.Next())
        {
            Shape sh = itl.Value();
            Solid tempo = new();
            b.MakeSolid(ref tempo);
            b.Add(tempo, sh);

            NCollection_List<Shape> thelist = new();
            imbSh[sh] = thelist;
            NCollection_List<Shape>.Iterator itl2 = new();
            for (itl2.Initialize(lresult); itl2.More(); itl2.Next())
            {
                Shape sh2 = itl2.Value();
                if (!sh2.IsSame(sh))
                {
                    if (IsInside(sh2, tempo))
                    {
                        lIntern.Append(sh2);
                        imbSh.GetValue(sh).Append(sh2);
                    }
                }
            }
        }

        // LPA 07/10/98: on vire les shells imbriques comme
        // etant aussi des solides a part entiere.
        for (itl.Initialize(lIntern); itl.More(); itl.Next())
        {
            Shape sh = itl.Value();
            if (imbSh.ContainsKey(sh))
            {
                imbSh.Remove(sh);
            }
        }

        NCollection_List<Shape> lsolid = new();
        do
        {
            Shape key = new();
            NCollection_List<Shape>? value = null;
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in imbSh)
            {
                if (itdm.Value.Extent() != 0)
                {
                    key = itdm.Key;
                    value = itdm.Value;
                    break;
                }
            }
            if (value is not null)
            {
                Solid newSo = new();
                b.MakeSolid(ref newSo);
                b.Add(newSo, key);
                for (itl.Initialize(value); itl.More(); itl.Next())
                {
                    b.Add(newSo, itl.Value().Reversed());
                }
                lsolid.Append(newSo);
                imbSh.Remove(key);
            }
            else
            {
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in imbSh)
                {
                    Solid newSo = new();
                    b.MakeSolid(ref newSo);
                    b.Add(newSo, itdm.Key);
                    lsolid.Append(newSo);
                }
                imbSh.Clear();
            }
        } while (imbSh.Count != 0);

        int nbsol = lsolid.Extent();
        int nbshl = lshell.Extent();

        if (nbsol == 1 && nbshl == 0)
        {
            _res = lsolid.First();
        }
        else if (nbsol == 0 && nbshl == 1)
        {
            _res = lshell.First();
        }
        else
        {
            Compound res = new();
            b.MakeCompound(ref res);
            _res = res;
            for (itl.Initialize(lsolid); itl.More(); itl.Next())
            {
                b.Add(_res, itl.Value());
            }
            for (itl.Initialize(lshell); itl.More(); itl.Next())
            {
                b.Add(_res, itl.Value());
            }
        }
    }

    /// <summary>The shape built (frozen).</summary>
    public Shape Shape()
    {
        _res.TShape()?.FreezeGraph();
        return _res;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal Shape InternalShape() => _res;

    private static void Add(int ind,
                            NCollection_IntMap mapI,
                            IndexedSet<Shape> mapF,
                            IndexedMap<Shape, NCollection_List<Shape>> mapEF)
    {
        if (!mapI.Add(ind))
        {
            throw new ConstructionErrorException();
        }

        NCollection_List<Shape>.Iterator itl = new(mapEF.ValueAt(ind - 1));
        for (; itl.More(); itl.Next())
        {
            if (!mapF.Contains(itl.Value()))
            {
                mapF.Add(itl.Value());
                Explorer exp = new();
                for (exp.Init(itl.Value(), ShapeType.Edge); exp.More(); exp.Next())
                {
                    Shape edg = exp.Current();
                    int indedg = mapEF.IndexOf(edg) + 1;
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

    private static void Propagate(in Shape f,
                                  in Shape sh,
                                  IndexedSet<Shape> mapF,
                                  NCollection_IntMap mapIf)
    {
        BrepBuilder b = new();
        int indf = mapF.IndexOf(f) + 1;
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
        for (exp.Init(f, ShapeType.Edge); exp.More(); exp.Next())
        {
            Shape edg = exp.Current();

            Orientation ored1 = edg.Orientation, ored2 = Forward;

            if (ored1 == Internal || ored1 == External)
            {
                continue;
            }
            NCollection_IntMap.Iterator itm = new(mapIf);
            for (; itm.More(); itm.Next())
            {
                Shape newF = mapF[itm.Key() - 1];
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
                    ored2 = exp2.Current().Orientation;
                    break;
                }
            }
            if (itm.More())
            {
                Shape ftoAdd = mapF[itm.Key() - 1];
                bool added = false;
                if (ored2 == ored1)
                {
                    ftoAdd.Reverse();
                    b.Add(sh, ftoAdd);
                    added = true;
                }
                else if (ored2 == global::Kerf.Engine.Topology.TopAbs.Reverse(ored1))
                {
                    b.Add(sh, ftoAdd);
                    added = true;
                }
                if (added)
                {
                    Propagate(ftoAdd, sh, mapF, mapIf);
                }
            }
        }
    }

    private static bool IsInside(in Shape s1, in Shape s2)
    {
        SolidClassifier Class = new(s2);
        Explorer exp = new();
        for (exp.Init(s1, ShapeType.Vertex); exp.More(); exp.Next())
        {
            Vertex vtx = ShapeCasts.Vertex(exp.Current());
            Point3D pttest = Tool.Pnt(vtx);
            double tol = Tool.Tolerance(vtx);
            Class.Perform(pttest, tol);
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
