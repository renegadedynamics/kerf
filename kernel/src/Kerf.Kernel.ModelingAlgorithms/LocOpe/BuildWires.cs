// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildWires.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildWires.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Features.Local;

/// <summary>Builds wires from a list of edges, stopping at the vertices that lie on the shape's edges or vertices.</summary>
/// <remarks>
/// A working object (decision 21). The wires are new; their edges are the given ones. <see cref="Result"/> freezes what
/// it hands out; ported callers read <see cref="InternalResult"/>.
/// </remarks>
internal sealed class BuildWires
{
    private bool _done;
    private readonly NCollection_List<Shape> _res = new();

    public BuildWires()
    {
        _done = false;
    }

    //  Modified by skv - Mon May 31 12:58:27 2004 OCC5865
    public BuildWires(NCollection_List<Shape> l, WiresOnShape pw)
    {
        Perform(l, pw);
    }

    //  Modified by skv - Mon May 31 12:59:09 2004 OCC5865
    public void Perform(NCollection_List<Shape> l, WiresOnShape pw)
    {
        _done = false;
        _res.Clear();

        BrepBuilder b = new();
        Compound c = new();
        b.MakeCompound(ref c);

        HashSet<Shape> theMap = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itl = new(l);
        for (; itl.More(); itl.Next())
        {
            Shape edg = itl.Value();
            if (theMap.Add(edg) && edg.Type == ShapeType.Edge)
            {
                b.Add(c, edg.Oriented(Forward)); // orientation importante pour
                // appel a TopExp::Vertices
            }
        }

        IndexedMap<Shape, NCollection_List<Shape>> theMapVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(c, ShapeType.Vertex, ShapeType.Edge, theMapVE);

        HashSet<Shape> bords = new(ShapeHasher.Instance);
        int i;

        for (i = 1; i <= theMapVE.Count; i++)
        {
            //  Modified by skv - Mon May 31 13:07:50 2004 OCC5865 Begin
            Vertex vtx = ShapeCasts.Vertex(theMapVE.KeyAt(i - 1));
            if (theMapVE.ValueAt(i - 1).Extent() == 1
                || (pw.OnVertex(vtx, out Vertex _) || pw.OnEdge(vtx, out Edge _, out double _)))
            {
                bords.Add(vtx);
                //  Modified by skv - Mon May 31 13:07:50 2004 OCC5865 End
            }
        }

        while ((i = FindFirstEdge(theMapVE, bords)) <= theMapVE.Count)
        {
            IndexedSet<Shape> mapE = new(ShapeHasher.Instance);
            // iterated below in another order than OCCT's (by address), which isn't observable: each vertex is handled alone
            HashSet<Shape> mapV = new(ShapeHasher.Instance);
            Edge edgf = ShapeCasts.Edge(theMapVE.ValueAt(i - 1).First());

            Exploration.Vertices(edgf, out Vertex VF, out Vertex vl);

            if (bords.Contains(vl) && !bords.Contains(VF))
            {
                mapE.Add(edgf.Oriented(Reversed));
                Vertex temp = VF;
                VF = vl;
                vl = temp;
            }
            else
            {
                mapE.Add(edgf.Oriented(Forward));
            }
            mapV.Add(VF);

            while (!(mapV.Contains(vl) || bords.Contains(vl)))
            {
                int ind = theMapVE.IndexOf(vl) + 1;
                NCollection_List<Shape>.Iterator iterl = new(theMapVE.ValueAt(ind - 1));
                for (; iterl.More(); iterl.Next())
                {
                    if (!mapE.Contains(iterl.Value()))
                    {
                        break;
                    }
                }

                if (!iterl.More())
                {
                    throw new ConstructionErrorException();
                }
                Edge theEdge = ShapeCasts.Edge(iterl.Value());
                Exploration.Vertices(theEdge, out Vertex vf1, out Vertex vl1);
                mapV.Add(vl);
                if (vf1.IsSame(vl))
                {
                    mapE.Add(theEdge.Oriented(Forward));
                    vl = vl1;
                }
                else
                { // on doit avoir Vl == VL
                    mapE.Add(theEdge.Oriented(Reversed));
                    vl = vf1;
                }
            }

            Wire newWire = new();
            b.MakeWire(ref newWire);

            if (mapV.Contains(vl))
            { // on sort avec une boucle a recreer
                Vertex vf;
                int j;
                for (j = 1; j <= mapE.Count; j++)
                {
                    Edge edg = ShapeCasts.Edge(mapE[j - 1]);
                    if (edg.Orientation == Forward)
                    {
                        vf = Exploration.FirstVertex(edg);
                    }
                    else
                    {
                        vf = Exploration.LastVertex(edg);
                    }
                    if (vf.IsSame(vl))
                    {
                        break;
                    }
                    mapV.Remove(vf);
                }
                for (; j <= mapE.Count; j++)
                {
                    b.Add(newWire, mapE[j - 1]);
                }
                newWire.Closed(true);
            }
            else
            { // on sort sur un bord : wire ouvert...
                mapV.Add(vl);
                for (int j = 0; j < mapE.Count; j++)
                {
                    b.Add(newWire, mapE[j]);
                }
                newWire.Closed(false);
            }

            _res.Append(newWire);
            foreach (Shape v in mapV)
            {
                Vertex vtx = ShapeCasts.Vertex(v);
                bords.Add(vtx);
                int ind = theMapVE.IndexOf(vtx) + 1;
                itl.Initialize(theMapVE.ValueAt(ind - 1));
                while (itl.More())
                {
                    if (mapE.Contains(itl.Value()))
                    {
                        theMapVE.ValueAt(ind - 1).Remove(itl);
                    }
                    else
                    {
                        itl.Next();
                    }
                }
            }
        }

        _done = true;
    }

    public bool IsDone => _done;

    /// <summary>The wires built (frozen).</summary>
    public NCollection_List<Shape> Result()
    {
        NCollection_List<Shape> res = InternalResult();
        foreach (Shape w in res)
        {
            w.TShape()?.FreezeGraph();
        }
        return res;
    }

    /// <summary><see cref="Result"/> without freezing, for ported callers.</summary>
    internal NCollection_List<Shape> InternalResult()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _res;
    }

    private static int FindFirstEdge(IndexedMap<Shape, NCollection_List<Shape>> mapVE,
                                     HashSet<Shape> bord)
    {
        int i = 1;

        for (; i <= mapVE.Count; i++)
        {
            if (mapVE.ValueAt(i - 1).Extent() > 0)
            {
                break;
            }
        }

        if (i > mapVE.Count)
        {
            return i;
        }

        int goodi = i;
        for (; i <= mapVE.Count; i++)
        {
            Shape vtx = mapVE.KeyAt(i - 1);
            if (mapVE.ValueAt(i - 1).Extent() > 0 && bord.Contains(vtx))
            {
                goodi = i;
                break;
            }
        }
        return goodi;
    }
}
