// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildWires.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildWires.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Builds wires from a list of edges, stopping at the vertices that lie on the shape's edges or vertices.</summary>
/// <remarks>
/// A working object (decision 21). The wires are new; their edges are the given ones. <see cref="Result"/> freezes what
/// it hands out; ported callers read <see cref="InternalResult"/>.
/// </remarks>
public sealed class LocOpe_BuildWires
{
    private bool myDone;
    private readonly NCollection_List<Shape> myRes = new();

    public LocOpe_BuildWires()
    {
        myDone = false;
    }

    //  Modified by skv - Mon May 31 12:58:27 2004 OCC5865
    public LocOpe_BuildWires(NCollection_List<Shape> L, LocOpe_WiresOnShape PW)
    {
        Perform(L, PW);
    }

    //  Modified by skv - Mon May 31 12:59:09 2004 OCC5865
    public void Perform(NCollection_List<Shape> L, LocOpe_WiresOnShape PW)
    {
        myDone = false;
        myRes.Clear();

        BrepBuilder B = new();
        Compound C = new();
        B.MakeCompound(ref C);

        NCollection_Map<Shape> theMap = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itl = new(L);
        for (; itl.More(); itl.Next())
        {
            Shape edg = itl.Value();
            if (theMap.Add(edg) && edg.ShapeType() == ShapeType.Edge)
            {
                B.Add(C, edg.Oriented(Forward)); // orientation importante pour
                // appel a TopExp::Vertices
            }
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(C, ShapeType.Vertex, ShapeType.Edge, theMapVE);

        NCollection_Map<Shape> Bords = new(ShapeHasher.Instance);
        int i;

        for (i = 1; i <= theMapVE.Extent(); i++)
        {
            //  Modified by skv - Mon May 31 13:07:50 2004 OCC5865 Begin
            Vertex vtx = ShapeCasts.Vertex(theMapVE.FindKey(i));
            if (theMapVE[i].Extent() == 1
                || (PW.OnVertex(vtx, out Vertex _) || PW.OnEdge(vtx, out Edge _, out double _)))
            {
                Bords.Add(vtx);
                //  Modified by skv - Mon May 31 13:07:50 2004 OCC5865 End
            }
        }

        while ((i = FindFirstEdge(theMapVE, Bords)) <= theMapVE.Extent())
        {
            NCollection_IndexedMap<Shape> mapE = new(ShapeHasher.Instance);
            // iterated below in another order than OCCT's (by address), which isn't observable: each vertex is handled alone
            NCollection_Map<Shape> mapV = new(ShapeHasher.Instance);
            Edge edgf = ShapeCasts.Edge(theMapVE[i].First());

            Exploration.Vertices(edgf, out Vertex VF, out Vertex VL);

            if (Bords.Contains(VL) && !Bords.Contains(VF))
            {
                mapE.Add(edgf.Oriented(Reversed));
                Vertex temp = VF;
                VF = VL;
                VL = temp;
            }
            else
            {
                mapE.Add(edgf.Oriented(Forward));
            }
            mapV.Add(VF);

            while (!(mapV.Contains(VL) || Bords.Contains(VL)))
            {
                int ind = theMapVE.FindIndex(VL);
                NCollection_List<Shape>.Iterator anIterl = new(theMapVE[ind]);
                for (; anIterl.More(); anIterl.Next())
                {
                    if (!mapE.Contains(anIterl.Value()))
                    {
                        break;
                    }
                }

                if (!anIterl.More())
                {
                    throw new ConstructionErrorException();
                }
                Edge theEdge = ShapeCasts.Edge(anIterl.Value());
                Exploration.Vertices(theEdge, out Vertex Vf1, out Vertex Vl1);
                mapV.Add(VL);
                if (Vf1.IsSame(VL))
                {
                    mapE.Add(theEdge.Oriented(Forward));
                    VL = Vl1;
                }
                else
                { // on doit avoir Vl == VL
                    mapE.Add(theEdge.Oriented(Reversed));
                    VL = Vf1;
                }
            }

            Wire newWire = new();
            B.MakeWire(ref newWire);

            if (mapV.Contains(VL))
            { // on sort avec une boucle a recreer
                Vertex Vf;
                int j;
                for (j = 1; j <= mapE.Extent(); j++)
                {
                    Edge edg = ShapeCasts.Edge(mapE[j]);
                    if (edg.Orientation() == Forward)
                    {
                        Vf = Exploration.FirstVertex(edg);
                    }
                    else
                    {
                        Vf = Exploration.LastVertex(edg);
                    }
                    if (Vf.IsSame(VL))
                    {
                        break;
                    }
                    mapV.Remove(Vf);
                }
                for (; j <= mapE.Extent(); j++)
                {
                    B.Add(newWire, mapE[j]);
                }
                newWire.Closed(true);
            }
            else
            { // on sort sur un bord : wire ouvert...
                mapV.Add(VL);
                for (int j = 1; j <= mapE.Extent(); j++)
                {
                    B.Add(newWire, mapE[j]);
                }
                newWire.Closed(false);
            }

            myRes.Append(newWire);
            foreach (Shape aV in mapV)
            {
                Vertex vtx = ShapeCasts.Vertex(aV);
                Bords.Add(vtx);
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

        myDone = true;
    }

    public bool IsDone() => myDone;

    /// <summary>The wires built (frozen).</summary>
    public NCollection_List<Shape> Result()
    {
        NCollection_List<Shape> aRes = InternalResult();
        foreach (Shape aW in aRes)
        {
            aW.TShape()?.FreezeGraph();
        }
        return aRes;
    }

    /// <summary><see cref="Result"/> without freezing, for ported callers.</summary>
    internal NCollection_List<Shape> InternalResult()
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        return myRes;
    }

    private static int FindFirstEdge(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapVE,
                                     NCollection_Map<Shape> theBord)
    {
        int i = 1;

        for (; i <= theMapVE.Extent(); i++)
        {
            if (theMapVE[i].Extent() > 0)
            {
                break;
            }
        }

        if (i > theMapVE.Extent())
        {
            return i;
        }

        int goodi = i;
        for (; i <= theMapVE.Extent(); i++)
        {
            Shape vtx = theMapVE.FindKey(i);
            if (theMapVE[i].Extent() > 0 && theBord.Contains(vtx))
            {
                goodi = i;
                break;
            }
        }
        return goodi;
    }
}
