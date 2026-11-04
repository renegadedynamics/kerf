// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientProcess.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientProcess.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Framework.Transfer.StatusExec;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// Adds specific features to the generic definition : TransientProcess is intended to work from an InterfaceModel to
/// a set of application objects. Hence, some information about starting entities can be gotten from the model : for
/// Trace, CheckList, Integrity Status.
/// </summary>
internal class TransientProcess : ProcessForTransient
{
    /// <summary>Sets TransientProcess at initial state, with an initial size.</summary>
    public TransientProcess(int nb = 10000)
        : base(nb)
    {
        _thetrroots = new NCollection_HSequence<Transient>();
    }

    /// <summary>
    /// Sets an InterfaceModel, used by StartTrace, CheckList, queries on Integrity, to give information significant for
    /// each norm.
    /// </summary>
    public void SetModel(InterfaceModel? model)
    {
        _themodel = model;
    }

    /// <summary>Returns the Model used for StartTrace.</summary>
    public InterfaceModel? Model()
    {
        return _themodel;
    }

    /// <summary>Sets a Graph : supersedes SetModel if already done.</summary>
    public void SetGraph(HGraph? hg)
    {
        _thegraph = hg;
        if (_thegraph is not null)
        {
            SetModel(_thegraph.Graph().Model());
        }
        else
        {
            _themodel = null;
        }
    }

    public bool HasGraph
    {
        get
        {
            return _thegraph is not null;
        }
    }

    public HGraph? HGraph()
    {
        return _thegraph;
    }

    public EntityGraph Graph()
    {
        return _thegraph.Graph();
    }

    /// <summary>Sets a Context : according to receiving appli, to be interpreted by the Actor.</summary>
    public void SetContext(string name, Transient? ctx)
    {
        _thectx[name] = ctx;
    }

    /// <summary>
    /// Returns the Context attached to a name, if set and if it is Kind of the type, else a Null Handle. Returns True
    /// if OK, False if no Context.
    /// </summary>
    public bool GetContext(string name, Type? type, ref Transient? ctx)
    {
        if (_thectx.Count == 0)
        {
            return false;
        }
        if (!_thectx.TryGetValue(name, out ctx))
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
    public Dictionary<string, Transient?> Context()
    {
        return _thectx;
    }

    /// <summary>Specific printing to trace an entity : prints label and type (if model is set).</summary>
    public override void PrintTrace(Transient? start, TextWriter s)
    {
        if (start is not null)
        {
            if (_themodel is not null)
            {
                s.Write("Entity ");
                _themodel.Print(start, s, 1);
                //      S<<"id ";  themodel->Print (start,S);
                //      S<<" Type:"<<themodel->TypeName (start);
            }
            else
            {
                s.Write("Entity Type:");
                s.Write(InterfaceModel.ClassName(start.DynamicTypeName()));
            }
            //  << start (handle)  ??
        }
    }

    /// <summary>Specific number of a starting object for check-list : Number in model.</summary>
    public override int CheckNum(Transient? start)
    {
        return _themodel is null ? 0 : _themodel.Number(start);
    }

    /// <summary>
    /// Returns the list of sharings entities, AT ANY LEVEL, which are kind of a given type. Calls TypedSharings from
    /// Graph. Returns an empty list if the Graph has not been acknowledged.
    /// </summary>
    public EntityIterator TypedSharings(Transient? start, Type? type)
    {
        var iter = new EntityIterator();
        if (_thegraph is null)
        {
            return iter;
        }
        return _thegraph.Graph().TypedSharings(start, type);
    }

    /// <summary>
    /// Tells if an entity is well loaded from file (even if its data fail on checking, they are present). Mostly often,
    /// answers True. Else, there was a syntactic error in the file. A non-loaded entity MAY NOT BE transferred, unless
    /// its Report (in the model) is interpreted.
    /// </summary>
    public bool IsDataLoaded(Transient? start)
    {
        if (_themodel is null)
        {
            return true;
        }
        int num = _themodel.Number(start);
        if (num == 0)
        {
            return true;
        }
        if (_themodel.IsUnknownEntity(num))
        {
            return false;
        }
        return !_themodel.IsRedefinedContent(num);
    }

    /// <summary>
    /// Tells if an entity fails on data checking (load time, syntactic, or semantic check). Normally, should answer
    /// False. It is not prudent to try transferring an entity which fails on data checking.
    /// </summary>
    public bool IsDataFail(Transient? start)
    {
        if (_themodel is null)
        {
            return false;
        }
        int num = _themodel.Number(start);
        if (num == 0)
        {
            return false;
        }
        if (_themodel.IsErrorEntity(num))
        {
            return true;
        }
        FrameworkCheck ach = _themodel.Check(num, false); // semantic
        return ach.HasFailed;
    }

    /// <summary>Prints statistics on a given output, according mode.</summary>
    public void PrintStats(int mode, TextWriter s)
    {
        s.Write("\n*******************************************************************\n");
        //  if (mode == 1) {    //  Basic statistics
        s.Write("********                 Basic Statistics                  ********\n");

        InterfaceModel? model = Model();
        if (model is null)
        {
            s.Write("****        Model unknown\n");
        }
        else
        {
            s.Write("****        Nb Entities         : " + model.EntityCount + "\n");
        }

        int nbr = 0, nbe = 0, nbw = 0;
        int i, max = MappedCount, nbroots = RootCount;
        s.Write("****        Nb Final Results    : " + nbroots + "\n");

        for (i = 1; i <= max; i++)
        {
            Binder? binder = MapItem(i);
            if (binder is null)
            {
                continue;
            }
            FrameworkCheck ach = binder.Check();
            StatusExec stat = binder.StatusExec();
            if (stat != StatusExec.StatusInitial && stat != StatusExec.StatusDone)
            {
                nbe++;
            }
            else
            {
                if (ach.WarningCount > 0)
                {
                    nbw++;
                }
                if (binder.HasResult)
                {
                    nbr++;
                }
            }
        }
        if (nbr > nbroots)
        {
            s.Write("****      ( Itermediate Results : " + (nbr - nbroots) + " )\n");
        }
        if (nbe > 0)
        {
            s.Write("****                  Errors on : " + FrameworkMsg.Blanks(nbe, 4) + nbe + " Entities\n");
        }
        if (nbw > 0)
        {
            s.Write("****                Warnings on : " + FrameworkMsg.Blanks(nbw, 4) + nbw + " Entities\n");
        }
        s.Write("*******************************************************************");
        //  }
        s.Write('\n');
    }

    /// <summary>The list of the roots to transfer (filled by the actors that ask for them).</summary>
    public NCollection_HSequence<Transient> RootsForTransfer()
    {
        return _thetrroots;
    }

    private InterfaceModel? _themodel;
    private HGraph? _thegraph;
    private readonly Dictionary<string, Transient?> _thectx = new(StringComparer.Ordinal);
    private readonly NCollection_HSequence<Transient> _thetrroots;
}
