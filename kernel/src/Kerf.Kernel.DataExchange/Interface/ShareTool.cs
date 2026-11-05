// Port of: src/DataExchange/TKXSBase/Interface/Interface_ShareTool.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ShareTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Builds the Graph of Dependencies, from the General Service "Shared" -&gt; builds for each Entity of a
/// Model, the Shared and Sharing Lists, and gives access to them. Allows to complete with Implied References
/// (which are not regarded as Shared Entities, but are nevertheless Referenced), this can be useful for
/// Reference Checking
/// </summary>
internal class ShareTool
{

    /// <summary>
    /// Creates a ShareTool from a Model and builds all required data, by calling the General Service Library and
    /// Modules (GeneralLib given as an argument)
    /// </summary>
    public ShareTool(InterfaceModel? amodel, GeneralLib lib)
    {
        _hGraph = new HGraph(amodel, lib);
    }

    /// <summary>
    /// Same a above, but GeneralLib is detained by a GTool
    /// </summary>
    public ShareTool(InterfaceModel? amodel, GTool? gtool)
    {
        _hGraph = new HGraph(amodel, gtool);
    }

    /// <summary>
    /// Same a above, but GeneralLib is defined through a Protocol Protocol is used to build the working library
    /// </summary>
    public ShareTool(InterfaceModel? amodel, Protocol? protocol)
    {
        _hGraph = new HGraph(amodel, protocol);
    }

    /// <summary>
    /// Same as above, but works with the GTool of the Model
    /// </summary>
    public ShareTool(InterfaceModel? amodel)
    {
        _hGraph = new HGraph(amodel);
    }

    /// <summary>
    /// Creates a ShareTool from an already defined Graph Remark that the data of the Graph are copied
    /// </summary>
    public ShareTool(EntityGraph agraph)
    {
        _hGraph = new HGraph(agraph.Model());
    }

    /// <summary>
    /// Completes the Graph by Adding Implied References. Hence, they are considered as Sharing References in all
    /// the other queries
    /// </summary>
    public ShareTool(HGraph? ahgraph)
    {
        _hGraph = ahgraph;
    }

    /// <summary>
    /// Returns the Model used for Creation (directly or for Graph)
    /// </summary>
    public InterfaceModel? Model()
    {
        return _hGraph.Graph().Model();
    }

    /// <summary>
    /// Returns the data used by the ShareTool to work Can then be used directly (read only)
    /// </summary>
    public EntityGraph Graph()
    {
        return _hGraph.Graph();
    }

    /// <summary>
    /// Returns the Entities which are not Shared (their Sharing List is empty) in the Model
    /// </summary>
    public EntityIterator RootEntities()
    {
        return _hGraph.Graph().RootEntities();
    }

    /// <summary>
    /// Returns True if &lt;ent&gt; is Shared by other Entities in the Model
    /// </summary>
    public bool IsShared(Transient? ent)
    {
        EntityGraph thegraph = _hGraph.Graph();
        NCollection_HSequence<Transient> list =
            thegraph.GetShareds(ent);
        return (list is not null && list.Length() > 0);
    }

    /// <summary>
    /// Returns the List of Entities Shared by a given Entity &lt;ent&gt;
    /// </summary>
    public EntityIterator Shareds(Transient? ent)
    {
        return _hGraph.Graph().Shareds(ent);
    }

    /// <summary>
    /// Returns the List of Entities Sharing a given Entity &lt;ent&gt;
    /// </summary>
    public EntityIterator Sharings(Transient? ent)
    {
        return _hGraph.Graph().Sharings(ent);
    }

    /// <summary>
    /// Returns the count of Sharing Entities of an Entity, which are Kind of a given Type
    /// </summary>
    public int NbTypedSharings(Transient? ent, Type? atype)
    {
        EntityGraph thegraph = _hGraph.CGraph();
        NCollection_HSequence<Transient> list =
            thegraph.GetSharings(ent);
        if (list is null)
        {
            return 0;
        }

        int result = 0;
        int n = list.Length();
        for (int i = 1; i <= n; i++)
        {
            Transient entsh = list.Value(i);
            if (entsh is null)
            {
                continue;
            }
            if (entsh.IsKind(atype))
            {
                result++;
            }
        }
        return result;
    }

    /// <summary>
    /// Returns the Sharing Entity of an Entity, which is Kind of a given Type. Allows to access a Sharing Entity
    /// of a given type when there is one and only one (current case)
    /// </summary>
    public Transient? TypedSharing(Transient? ent, Type? atype)
    {
        EntityGraph thegraph = _hGraph.CGraph();
        NCollection_HSequence<Transient> list =
            thegraph.GetSharings(ent);
        if (list is null)
        {
            return null;
        }
        Transient entresult = null;
        int result = 0;
        int n = list.Length();
        for (int i = 1; i <= n; i++)
        {
            Transient entsh = list.Value(i);
            if (entsh is null)
            {
                continue;
            }
            if (entsh.IsKind(atype))
            {
                entresult = entsh;
                result++;
                if (result > 1)
                {
                    throw new InterfaceErrorException("Interface ShareTool : TypedSharing, more than one found");
                }
            }
        }
        if (result == 0)
        {
            throw new InterfaceErrorException("Interface ShareTool : TypedSharing, not found");
        }
        return entresult;
    }

    /// <summary>
    /// Returns the complete list of entities shared by &lt;ent&gt; at any level, including &lt;ent&gt; itself If
    /// &lt;ent&gt; is the Model, considers the concatenation of AllShared for each root If &lt;rootlast&gt; is
    /// True (D), the list starts with lower level entities and ends by the root. Else, the root is first and the
    /// lower level entities are at end
    /// </summary>
    public EntityIterator All(Transient? ent, bool rootlast = true)
    {
        InterfaceModel model = Model();
        EntityIterator list = new EntityIterator();
        int i, n0 = 0, nb = model.EntityCount;
        NCollection_HArray1<int> fl = new NCollection_HArray1<int>(0, nb);
        fl.Init(0);
        if (ent == model)
        {
            //    We review the roots (the base order is preserved)
            EntityIterator roots = RootEntities();
            for (roots.Start(); roots.More(); roots.Next())
            {
                EntityIterator subl = All(roots.Value(), rootlast);
                for (subl.Start(); subl.More(); subl.Next())
                {
                    int nm = model.Number(subl.Value());
                    if (fl.Value(nm) > 0)
                    {
                        continue;
                    }
                    n0++;
                    fl.SetValue(nm, n0);
                }
            }
            //    Warning, are there any omissions?
            for (i = 1; i <= nb; i++)
            {
                if (fl.Value(i) == 0)
                {
                    n0++;
                    fl.SetValue(i, n0);
                }
            }
        }
        else
        {
            NCollection_HSequence<Transient> sq =
                new NCollection_HSequence<Transient>();
            sq.Append(ent);
            //    file type process
            for (i = 1; i <= sq.Length(); i++)
            { // Length croit
                Transient en = sq.Value(i);
                int num = model.Number(en);
                if (fl.Value(num) != 0)
                {
                    continue; // already seen
                }
                n0++;
                fl.SetValue(num, n0);
                EntityIterator sh = Shareds(en);
                sq.Append(sh.Content());
            }
        }
        //    Remains to constitute the list, return if necessary
        NCollection_HArray1<int> ord = new NCollection_HArray1<int>(0, nb);
        ord.Init(0);
        for (i = 1; i <= nb; i++)
        {
            n0 = fl.Value(i);
            ord.SetValue(n0, i);
        }
        if (rootlast && ent != model)
        {
            for (i = 1; i <= nb; i++)
            {
                if (ord.Value(i) != 0)
                {
                    list.AddItem(model.Value(ord.Value(i)));
                }
            }
        }
        else
        {
            for (i = nb; i > 0; i--)
            {
                if (ord.Value(i) != 0)
                {
                    list.AddItem(model.Value(ord.Value(i)));
                }
            }
        }

        return list;
    }

    /// <summary>
    /// Utility method which Prints the content of an iterator (by their Numbers)
    /// </summary>
    public void Print(EntityIterator iter, TextWriter s)
    {
        s.Write(" Nb.Entities : ");
        s.Write(iter.NbEntities());
        s.Write(" : ");
        for (iter.Start(); iter.More(); iter.Next())
        {
            Transient ent = iter.Value();
            s.Write(" n0/id:");
            Model().Print(ent, s);
        }
        s.Write('\n');
    }


    private HGraph? _hGraph;
}
