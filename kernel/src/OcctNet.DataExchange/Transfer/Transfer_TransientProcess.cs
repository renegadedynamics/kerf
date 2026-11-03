// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientProcess.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientProcess.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.Transfer_StatusExec;

namespace Occt;

/// <summary>
/// Adds specific features to the generic definition : TransientProcess is intended to work from an InterfaceModel to
/// a set of application objects. Hence, some information about starting entities can be gotten from the model : for
/// Trace, CheckList, Integrity Status.
/// </summary>
public class Transfer_TransientProcess : Transfer_ProcessForTransient
{
    /// <summary>Sets TransientProcess at initial state, with an initial size.</summary>
    public Transfer_TransientProcess(int nb = 10000)
        : base(nb)
    {
        thetrroots = new NCollection_HSequence<Transient>();
    }

    /// <summary>
    /// Sets an InterfaceModel, used by StartTrace, CheckList, queries on Integrity, to give information significant for
    /// each norm.
    /// </summary>
    public void SetModel(Interface_InterfaceModel? model)
    {
        themodel = model;
    }

    /// <summary>Returns the Model used for StartTrace.</summary>
    public Interface_InterfaceModel? Model()
    {
        return themodel;
    }

    /// <summary>Sets a Graph : supersedes SetModel if already done.</summary>
    public void SetGraph(Interface_HGraph? HG)
    {
        thegraph = HG;
        if (thegraph is not null)
        {
            SetModel(thegraph.Graph().Model());
        }
        else
        {
            themodel = null;
        }
    }

    public bool HasGraph()
    {
        return thegraph is not null;
    }

    public Interface_HGraph? HGraph()
    {
        return thegraph;
    }

    public Interface_Graph Graph()
    {
        return thegraph.Graph();
    }

    /// <summary>Sets a Context : according to receiving appli, to be interpreted by the Actor.</summary>
    public void SetContext(string name, Transient? ctx)
    {
        thectx.Bind(name, ctx);
    }

    /// <summary>
    /// Returns the Context attached to a name, if set and if it is Kind of the type, else a Null Handle. Returns True
    /// if OK, False if no Context.
    /// </summary>
    public bool GetContext(string name, Type? type, ref Transient? ctx)
    {
        if (thectx.IsEmpty())
        {
            return false;
        }
        if (!thectx.Find(name, out ctx))
        {
            ctx = null;
        }
        if (ctx is null)
        {
            return false;
        }
        if (type is null)
        {
            return true;
        }
        if (!ctx.IsKind(type))
        {
            ctx = null;
        }
        return ctx is not null;
    }

    /// <summary>
    /// Returns (modifiable) the whole definition of Context. Rather for internal use (ex.: preparing and setting in
    /// once).
    /// </summary>
    public NCollection_DataMap<string, Transient?> Context()
    {
        return thectx;
    }

    /// <summary>Specific printing to trace an entity : prints label and type (if model is set).</summary>
    public override void PrintTrace(Transient? start, TextWriter S)
    {
        if (start is not null)
        {
            if (themodel is not null)
            {
                S.Write("Entity ");
                themodel.Print(start, S, 1);
                //      S<<"id ";  themodel->Print (start,S);
                //      S<<" Type:"<<themodel->TypeName (start);
            }
            else
            {
                S.Write("Entity Type:");
                S.Write(Interface_InterfaceModel.ClassName(start.DynamicTypeName()));
            }
            //  << start (handle)  ??
        }
    }

    /// <summary>Specific number of a starting object for check-list : Number in model.</summary>
    public override int CheckNum(Transient? start)
    {
        return themodel is null ? 0 : themodel.Number(start);
    }

    /// <summary>
    /// Returns the list of sharings entities, AT ANY LEVEL, which are kind of a given type. Calls TypedSharings from
    /// Graph. Returns an empty list if the Graph has not been acknowledged.
    /// </summary>
    public Interface_EntityIterator TypedSharings(Transient? start, Type? type)
    {
        var iter = new Interface_EntityIterator();
        if (thegraph is null)
        {
            return iter;
        }
        return thegraph.Graph().TypedSharings(start, type);
    }

    /// <summary>
    /// Tells if an entity is well loaded from file (even if its data fail on checking, they are present). Mostly often,
    /// answers True. Else, there was a syntactic error in the file. A non-loaded entity MAY NOT BE transferred, unless
    /// its Report (in the model) is interpreted.
    /// </summary>
    public bool IsDataLoaded(Transient? start)
    {
        if (themodel is null)
        {
            return true;
        }
        int num = themodel.Number(start);
        if (num == 0)
        {
            return true;
        }
        if (themodel.IsUnknownEntity(num))
        {
            return false;
        }
        return !themodel.IsRedefinedContent(num);
    }

    /// <summary>
    /// Tells if an entity fails on data checking (load time, syntactic, or semantic check). Normally, should answer
    /// False. It is not prudent to try transferring an entity which fails on data checking.
    /// </summary>
    public bool IsDataFail(Transient? start)
    {
        if (themodel is null)
        {
            return false;
        }
        int num = themodel.Number(start);
        if (num == 0)
        {
            return false;
        }
        if (themodel.IsErrorEntity(num))
        {
            return true;
        }
        Interface_Check ach = themodel.Check(num, false); // semantic
        return ach.HasFailed();
    }

    /// <summary>Prints statistics on a given output, according mode.</summary>
    public void PrintStats(int mode, TextWriter S)
    {
        S.Write("\n*******************************************************************\n");
        //  if (mode == 1) {    //  Basic statistics
        S.Write("********                 Basic Statistics                  ********\n");

        Interface_InterfaceModel? model = Model();
        if (model is null)
        {
            S.Write("****        Model unknown\n");
        }
        else
        {
            S.Write("****        Nb Entities         : " + model.NbEntities() + "\n");
        }

        int nbr = 0, nbe = 0, nbw = 0;
        int i, max = NbMapped(), nbroots = NbRoots();
        S.Write("****        Nb Final Results    : " + nbroots + "\n");

        for (i = 1; i <= max; i++)
        {
            Transfer_Binder? binder = MapItem(i);
            if (binder is null)
            {
                continue;
            }
            Interface_Check ach = binder.Check();
            Transfer_StatusExec stat = binder.StatusExec();
            if (stat != Transfer_StatusExec.Transfer_StatusInitial && stat != Transfer_StatusExec.Transfer_StatusDone)
            {
                nbe++;
            }
            else
            {
                if (ach.NbWarnings() > 0)
                {
                    nbw++;
                }
                if (binder.HasResult())
                {
                    nbr++;
                }
            }
        }
        if (nbr > nbroots)
        {
            S.Write("****      ( Itermediate Results : " + (nbr - nbroots) + " )\n");
        }
        if (nbe > 0)
        {
            S.Write("****                  Errors on : " + Interface_MSG.Blanks(nbe, 4) + nbe + " Entities\n");
        }
        if (nbw > 0)
        {
            S.Write("****                Warnings on : " + Interface_MSG.Blanks(nbw, 4) + nbw + " Entities\n");
        }
        S.Write("*******************************************************************");
        //  }
        S.Write('\n');
    }

    /// <summary>The list of the roots to transfer (filled by the actors that ask for them).</summary>
    public NCollection_HSequence<Transient> RootsForTransfer()
    {
        return thetrroots;
    }

    private Interface_InterfaceModel? themodel;
    private Interface_HGraph? thegraph;
    private readonly NCollection_DataMap<string, Transient?> thectx = new(StringComparer.Ordinal);
    private readonly NCollection_HSequence<Transient> thetrroots;
}
