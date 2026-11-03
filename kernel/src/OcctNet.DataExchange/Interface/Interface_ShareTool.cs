// Port of: src/DataExchange/TKXSBase/Interface/Interface_ShareTool.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ShareTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Builds the Graph of Dependencies, from the General Service "Shared" -&gt; builds for each Entity of a
/// Model, the Shared and Sharing Lists, and gives access to them. Allows to complete with Implied References
/// (which are not regarded as Shared Entities, but are nevertheless Referenced), this can be useful for
/// Reference Checking
/// </summary>
public class Interface_ShareTool
{

    /// <summary>
    /// Creates a ShareTool from a Model and builds all required data, by calling the General Service Library and
    /// Modules (GeneralLib given as an argument)
    /// </summary>
    public Interface_ShareTool(Interface_InterfaceModel? amodel, Interface_GeneralLib lib)
    {
        theHGraph = new Interface_HGraph(amodel, lib);
    }

    /// <summary>
    /// Same a above, but GeneralLib is detained by a GTool
    /// </summary>
    public Interface_ShareTool(Interface_InterfaceModel? amodel, Interface_GTool? gtool)
    {
        theHGraph = new Interface_HGraph(amodel, gtool);
    }

    /// <summary>
    /// Same a above, but GeneralLib is defined through a Protocol Protocol is used to build the working library
    /// </summary>
    public Interface_ShareTool(Interface_InterfaceModel? amodel, Interface_Protocol? protocol)
    {
        theHGraph = new Interface_HGraph(amodel, protocol);
    }

    /// <summary>
    /// Same as above, but works with the GTool of the Model
    /// </summary>
    public Interface_ShareTool(Interface_InterfaceModel? amodel)
    {
        theHGraph = new Interface_HGraph(amodel);
    }

    /// <summary>
    /// Creates a ShareTool from an already defined Graph Remark that the data of the Graph are copied
    /// </summary>
    public Interface_ShareTool(Interface_Graph agraph)
    {
        theHGraph = new Interface_HGraph(agraph.Model());
    }

    /// <summary>
    /// Completes the Graph by Adding Implied References. Hence, they are considered as Sharing References in all
    /// the other queries
    /// </summary>
    public Interface_ShareTool(Interface_HGraph? ahgraph)
    {
        theHGraph = ahgraph;
    }

    /// <summary>
    /// Returns the Model used for Creation (directly or for Graph)
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return theHGraph.Graph().Model();
    }

    /// <summary>
    /// Returns the data used by the ShareTool to work Can then be used directly (read only)
    /// </summary>
    public Interface_Graph Graph()
    {
        return theHGraph.Graph();
    }

    /// <summary>
    /// Returns the Entities which are not Shared (their Sharing List is empty) in the Model
    /// </summary>
    public Interface_EntityIterator RootEntities()
    {
        return theHGraph.Graph().RootEntities();
    }

    /// <summary>
    /// Returns True if &lt;ent&gt; is Shared by other Entities in the Model
    /// </summary>
    public bool IsShared(Transient? ent)
    {
        Interface_Graph thegraph = theHGraph.Graph();
        NCollection_HSequence<Transient> list =
            thegraph.GetShareds(ent);
        return (list is not null && list.Length() > 0);
    }

    /// <summary>
    /// Returns the List of Entities Shared by a given Entity &lt;ent&gt;
    /// </summary>
    public Interface_EntityIterator Shareds(Transient? ent)
    {
        return theHGraph.Graph().Shareds(ent);
    }

    /// <summary>
    /// Returns the List of Entities Sharing a given Entity &lt;ent&gt;
    /// </summary>
    public Interface_EntityIterator Sharings(Transient? ent)
    {
        return theHGraph.Graph().Sharings(ent);
    }

    /// <summary>
    /// Returns the count of Sharing Entities of an Entity, which are Kind of a given Type
    /// </summary>
    public int NbTypedSharings(Transient? ent, Type? atype)
    {
        Interface_Graph thegraph = theHGraph.CGraph();
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
        Interface_Graph thegraph = theHGraph.CGraph();
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
                    throw new Interface_InterfaceError("Interface ShareTool : TypedSharing, more than one found");
                }
            }
        }
        if (result == 0)
        {
            throw new Interface_InterfaceError("Interface ShareTool : TypedSharing, not found");
        }
        return entresult;
    }

    /// <summary>
    /// Returns the complete list of entities shared by &lt;ent&gt; at any level, including &lt;ent&gt; itself If
    /// &lt;ent&gt; is the Model, considers the concatenation of AllShared for each root If &lt;rootlast&gt; is
    /// True (D), the list starts with lower level entities and ends by the root. Else, the root is first and the
    /// lower level entities are at end
    /// </summary>
    public Interface_EntityIterator All(Transient? ent, bool rootlast = true)
    {
        Interface_InterfaceModel model = Model();
        Interface_EntityIterator list = new Interface_EntityIterator();
        int i, n0 = 0, nb = model.NbEntities();
        NCollection_HArray1<int> fl = new NCollection_HArray1<int>(0, nb);
        fl.Init(0);
        if (ent == model)
        {
            //    We review the roots (the base order is preserved)
            Interface_EntityIterator roots = RootEntities();
            for (roots.Start(); roots.More(); roots.Next())
            {
                Interface_EntityIterator subl = All(roots.Value(), rootlast);
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
                Interface_EntityIterator sh = Shareds(en);
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
    public void Print(Interface_EntityIterator iter, TextWriter S)
    {
        S.Write(" Nb.Entities : ");
        S.Write(iter.NbEntities());
        S.Write(" : ");
        for (iter.Start(); iter.More(); iter.Next())
        {
            Transient ent = iter.Value();
            S.Write(" n0/id:");
            Model().Print(ent, S);
        }
        S.Write('\n');
    }


    private Interface_HGraph? theHGraph;
}
