// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildWires.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildWires.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Local;

/// <summary>Builds wires from a list of edges, stopping at the vertices that lie on the shape's edges or vertices.</summary>
/// <remarks>
/// A working object (decision 21). The wires are new; their edges are the given ones. <see cref="Result"/> freezes what
/// it hands out; ported callers read <see cref="InternalResult"/>.
/// </remarks>
public sealed class BuildWires
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
            if (theMap.Add(edg) && edg.ShapeType() == ShapeType.Edge)
            {
                b.Add(c, edg.Oriented(Forward)); // orientation importante pour
                // appel a TopExp::Vertices
            }
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(c, ShapeType.Vertex, ShapeType.Edge, theMapVE);

        HashSet<Shape> bords = new(ShapeHasher.Instance);
        int i;

        for (i = 1; i <= theMapVE.Extent(); i++)
        {
            //  Modified by skv - Mon May 31 13:07:50 2004 OCC5865 Begin
            Vertex vtx = ShapeCasts.Vertex(theMapVE.FindKey(i));
            if (theMapVE[i].Extent() == 1
                || (pw.OnVertex(vtx, out Vertex _) || pw.OnEdge(vtx, out Edge _, out double _)))
            {
                bords.Add(vtx);
                //  Modified by skv - Mon May 31 13:07:50 2004 OCC5865 End
            }
        }

        while ((i = FindFirstEdge(theMapVE, bords)) <= theMapVE.Extent())
        {
            NCollection_IndexedMap<Shape> mapE = new(ShapeHasher.Instance);
            // iterated below in another order than OCCT's (by address), which isn't observable: each vertex is handled alone
            HashSet<Shape> mapV = new(ShapeHasher.Instance);
            Edge edgf = ShapeCasts.Edge(theMapVE[i].First());

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
                int ind = theMapVE.FindIndex(vl);
                NCollection_List<Shape>.Iterator iterl = new(theMapVE[ind]);
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
                for (j = 1; j <= mapE.Extent(); j++)
                {
                    Edge edg = ShapeCasts.Edge(mapE[j]);
                    if (edg.Orientation() == Forward)
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
                for (; j <= mapE.Extent(); j++)
                {
                    b.Add(newWire, mapE[j]);
                }
                newWire.Closed(true);
            }
            else
            { // on sort sur un bord : wire ouvert...
                mapV.Add(vl);
                for (int j = 1; j <= mapE.Extent(); j++)
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
                int ind = theMapVE.FindIndex(vtx);
                itl.Initialize(theMapVE[ind]);
                while (itl.More())
                {
                    if (mapE.Contains(itl.Value()))
                    {
                        theMapVE[ind].Remove(itl);
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

    private static int FindFirstEdge(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapVE,
                                     HashSet<Shape> bord)
    {
        int i = 1;

        for (; i <= mapVE.Extent(); i++)
        {
            if (mapVE[i].Extent() > 0)
            {
                break;
            }
        }

        if (i > mapVE.Extent())
        {
            return i;
        }

        int goodi = i;
        for (; i <= mapVE.Extent(); i++)
        {
            Shape vtx = mapVE.FindKey(i);
            if (mapVE[i].Extent() > 0 && bord.Contains(vtx))
            {
                goodi = i;
                break;
            }
        }
        return goodi;
    }
}
