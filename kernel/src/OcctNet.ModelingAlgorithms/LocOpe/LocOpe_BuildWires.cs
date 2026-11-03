// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildWires.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_BuildWires.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Builds wires from a list of edges, stopping at the vertices that lie on the shape's edges or vertices.</summary>
/// <remarks>
/// A working object (decision 21). The wires are new; their edges are the given ones. <see cref="Result"/> freezes what
/// it hands out; ported callers read <see cref="InternalResult"/>.
/// </remarks>
public sealed class LocOpe_BuildWires
{
    private bool myDone;
    private readonly NCollection_List<TopoDS_Shape> myRes = new();

    public LocOpe_BuildWires()
    {
        myDone = false;
    }

    //  Modified by skv - Mon May 31 12:58:27 2004 OCC5865
    public LocOpe_BuildWires(NCollection_List<TopoDS_Shape> L, LocOpe_WiresOnShape PW)
    {
        Perform(L, PW);
    }

    //  Modified by skv - Mon May 31 12:59:09 2004 OCC5865
    public void Perform(NCollection_List<TopoDS_Shape> L, LocOpe_WiresOnShape PW)
    {
        myDone = false;
        myRes.Clear();

        BRep_Builder B = new();
        TopoDS_Compound C = new();
        B.MakeCompound(ref C);

        NCollection_Map<TopoDS_Shape> theMap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator itl = new(L);
        for (; itl.More(); itl.Next())
        {
            TopoDS_Shape edg = itl.Value();
            if (theMap.Add(edg) && edg.ShapeType() == TopAbs_EDGE)
            {
                B.Add(C, edg.Oriented(TopAbs_FORWARD)); // orientation importante pour
                // appel a TopExp::Vertices
            }
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapVE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(C, TopAbs_VERTEX, TopAbs_EDGE, theMapVE);

        NCollection_Map<TopoDS_Shape> Bords = new(TopTools_ShapeMapHasher.Instance);
        int i;

        for (i = 1; i <= theMapVE.Extent(); i++)
        {
            //  Modified by skv - Mon May 31 13:07:50 2004 OCC5865 Begin
            TopoDS_Vertex vtx = TopoDS.Vertex(theMapVE.FindKey(i));
            if (theMapVE[i].Extent() == 1
                || (PW.OnVertex(vtx, out TopoDS_Vertex _) || PW.OnEdge(vtx, out TopoDS_Edge _, out double _)))
            {
                Bords.Add(vtx);
                //  Modified by skv - Mon May 31 13:07:50 2004 OCC5865 End
            }
        }

        while ((i = FindFirstEdge(theMapVE, Bords)) <= theMapVE.Extent())
        {
            NCollection_IndexedMap<TopoDS_Shape> mapE = new(TopTools_ShapeMapHasher.Instance);
            // iterated below in another order than OCCT's (by address), which isn't observable: each vertex is handled alone
            NCollection_Map<TopoDS_Shape> mapV = new(TopTools_ShapeMapHasher.Instance);
            TopoDS_Edge edgf = TopoDS.Edge(theMapVE[i].First());

            TopExp.Vertices(edgf, out TopoDS_Vertex VF, out TopoDS_Vertex VL);

            if (Bords.Contains(VL) && !Bords.Contains(VF))
            {
                mapE.Add(edgf.Oriented(TopAbs_REVERSED));
                TopoDS_Vertex temp = VF;
                VF = VL;
                VL = temp;
            }
            else
            {
                mapE.Add(edgf.Oriented(TopAbs_FORWARD));
            }
            mapV.Add(VF);

            while (!(mapV.Contains(VL) || Bords.Contains(VL)))
            {
                int ind = theMapVE.FindIndex(VL);
                NCollection_List<TopoDS_Shape>.Iterator anIterl = new(theMapVE[ind]);
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
                TopoDS_Edge theEdge = TopoDS.Edge(anIterl.Value());
                TopExp.Vertices(theEdge, out TopoDS_Vertex Vf1, out TopoDS_Vertex Vl1);
                mapV.Add(VL);
                if (Vf1.IsSame(VL))
                {
                    mapE.Add(theEdge.Oriented(TopAbs_FORWARD));
                    VL = Vl1;
                }
                else
                { // on doit avoir Vl == VL
                    mapE.Add(theEdge.Oriented(TopAbs_REVERSED));
                    VL = Vf1;
                }
            }

            TopoDS_Wire newWire = new();
            B.MakeWire(ref newWire);

            if (mapV.Contains(VL))
            { // on sort avec une boucle a recreer
                TopoDS_Vertex Vf;
                int j;
                for (j = 1; j <= mapE.Extent(); j++)
                {
                    TopoDS_Edge edg = TopoDS.Edge(mapE[j]);
                    if (edg.Orientation() == TopAbs_FORWARD)
                    {
                        Vf = TopExp.FirstVertex(edg);
                    }
                    else
                    {
                        Vf = TopExp.LastVertex(edg);
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
            foreach (TopoDS_Shape aV in mapV)
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(aV);
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
    public NCollection_List<TopoDS_Shape> Result()
    {
        NCollection_List<TopoDS_Shape> aRes = InternalResult();
        foreach (TopoDS_Shape aW in aRes)
        {
            aW.TShape()?.FreezeGraph();
        }
        return aRes;
    }

    /// <summary><see cref="Result"/> without freezing, for ported callers.</summary>
    internal NCollection_List<TopoDS_Shape> InternalResult()
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        return myRes;
    }

    private static int FindFirstEdge(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapVE,
                                     NCollection_Map<TopoDS_Shape> theBord)
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
            TopoDS_Shape vtx = theMapVE.FindKey(i);
            if (theMapVE[i].Extent() > 0 && theBord.Contains(vtx))
            {
                goodi = i;
                break;
            }
        }
        return goodi;
    }
}
