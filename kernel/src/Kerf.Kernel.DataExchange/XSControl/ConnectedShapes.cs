// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_ConnectedShapes.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_ConnectedShapes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// From a TopoDS_Shape, or from the entity which has produced it, searches for the shapes, and the entities
/// which have produced them in last transfer, which are adjacent to it by VERTICES
/// </summary>
internal class ConnectedShapes : SelectExplore
{

    /// <summary>
    /// Creates a Selection ConnectedShapes. It remains to be set a TransferReader
    /// </summary>
    public ConnectedShapes() : base(1)
    {
    }

    /// <summary>
    /// Creates a Selection ConnectedShapes, which will work with the current TransferProcess brought by the
    /// TransferReader
    /// </summary>
    public ConnectedShapes(TransferReader? tr) : base(1)
    {
        _tr = tr;
    }

    /// <summary>
    /// Sets a TransferReader to sort entities : it brings the TransferProcess which may change, while the
    /// TransferReader does not
    /// </summary>
    public void SetReader(TransferReader? tr)
    {
        _tr = tr;
    }

    /// <summary>
    /// Explores an entity : entities from which are connected to that produced by this entity, including itself
    /// </summary>
    public override bool Explore(int level, Transient? ent, EntityGraph g, EntityIterator explored)
    {
        TransientProcess tp = null;
        if (_tr is not null)
        {
            tp = _tr.TransientProcess();
        }
        if (tp is null)
        {
            return false;
        }
        Shape shape = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(tp, ent);
        if (shape.IsNull)
        {
            return false;
        }
        NCollection_HSequence<Transient> li =
            AdjacentEntities(shape, tp, ShapeType.Face);
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
    public static NCollection_HSequence<Transient>? AdjacentEntities(Shape ashape, TransientProcess? tp, ShapeType type)
    {
        NCollection_HSequence<Transient> li =
            new NCollection_HSequence<Transient>();
        int i, nb = tp.MappedCount;
        //  NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> adj (nb);
        HashSet<Shape> vtx = new HashSet<Shape>(ShapeHasher.Instance);

        for (Explorer vert = new Explorer(ashape, ShapeType.Vertex); vert.More(); vert.Next())
        {
            vtx.Add(vert.Current());
        }

        for (i = 1; i <= nb; i++)
        {
            Binder bnd = tp.MapItem(i);
            Shape sh = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(bnd);
            if (sh.IsNull)
            {
                continue;
            }
            if (sh.Type != type)
            {
                continue;
            }
            for (Explorer vsh = new Explorer(sh, ShapeType.Vertex); vsh.More(); vsh.Next())
            {
                Shape avtx = vsh.Current();
                if (vtx.Contains(avtx))
                {
                    li.Append(tp.Mapped(i));
                    break; // break from this inner for, next entity
                }
            }
        }

        return li;
    }

    private TransferReader? _tr;
}
