// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_ConnectedShapes.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_ConnectedShapes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// From a TopoDS_Shape, or from the entity which has produced it, searches for the shapes, and the entities
/// which have produced them in last transfer, which are adjacent to it by VERTICES
/// </summary>
public class XSControl_ConnectedShapes : IFSelect_SelectExplore
{

    /// <summary>
    /// Creates a Selection ConnectedShapes. It remains to be set a TransferReader
    /// </summary>
    public XSControl_ConnectedShapes() : base(1)
    {
    }

    /// <summary>
    /// Creates a Selection ConnectedShapes, which will work with the current TransferProcess brought by the
    /// TransferReader
    /// </summary>
    public XSControl_ConnectedShapes(XSControl_TransferReader? TR) : base(1)
    {
        theTR = TR;
    }

    /// <summary>
    /// Sets a TransferReader to sort entities : it brings the TransferProcess which may change, while the
    /// TransferReader does not
    /// </summary>
    public void SetReader(XSControl_TransferReader? TR)
    {
        theTR = TR;
    }

    /// <summary>
    /// Explores an entity : entities from which are connected to that produced by this entity, including itself
    /// </summary>
    public override bool Explore(int level, Transient? ent, Interface_Graph G, Interface_EntityIterator explored)
    {
        Transfer_TransientProcess TP = null;
        if (theTR is not null)
        {
            TP = theTR.TransientProcess();
        }
        if (TP is null)
        {
            return false;
        }
        TopoDS_Shape Shape = TransferBRep.ShapeResult(TP, ent);
        if (Shape.IsNull())
        {
            return false;
        }
        NCollection_HSequence<Transient> li =
            AdjacentEntities(Shape, TP, TopAbs_FACE);
        explored.AddList(li);
        return true;
    }

    /// <summary>
    /// Returns a text defining the criterium. "Connected Entities through produced Shapes"
    /// </summary>
    public override string ExploreLabel()
    {
        string lab = "Connected Entities through produced Shapes";
        return lab;
    }

    /// <summary>
    /// This functions considers a shape from a transfer and performs the search function explained above
    /// </summary>
    public static NCollection_HSequence<Transient>? AdjacentEntities(TopoDS_Shape ashape, Transfer_TransientProcess? TP, TopAbs_ShapeEnum type)
    {
        NCollection_HSequence<Transient> li =
            new NCollection_HSequence<Transient>();
        int i, nb = TP.NbMapped();
        //  NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> adj (nb);
        NCollection_Map<TopoDS_Shape> vtx = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        for (TopExp_Explorer vert = new TopExp_Explorer(ashape, TopAbs_VERTEX); vert.More(); vert.Next())
        {
            vtx.Add(vert.Current());
        }

        for (i = 1; i <= nb; i++)
        {
            Transfer_Binder bnd = TP.MapItem(i);
            TopoDS_Shape sh = TransferBRep.ShapeResult(bnd);
            if (sh.IsNull())
            {
                continue;
            }
            if (sh.ShapeType() != type)
            {
                continue;
            }
            for (TopExp_Explorer vsh = new TopExp_Explorer(sh, TopAbs_VERTEX); vsh.More(); vsh.Next())
            {
                TopoDS_Shape avtx = vsh.Current();
                if (vtx.Contains(avtx))
                {
                    li.Append(TP.Mapped(i));
                    break; // break from this inner for, next entity
                }
            }
        }

        return li;
    }

    private XSControl_TransferReader? theTR;
}
