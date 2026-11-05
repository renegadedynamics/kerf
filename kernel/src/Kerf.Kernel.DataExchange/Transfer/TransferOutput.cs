// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferOutput.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferOutput.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;
using static Kerf.Engine.Exchange.Framework.Transfer.StatusExec;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// A TransferOutput is a Tool which manages the transfer of entities created by an Interface, stored in an
/// InterfaceModel, into a set of Objects suitable for an Application. Objects to be transferred are given, by method
/// Transfer (which calls Transfer from TransientProcess). A default action is available to get all roots of the Model.
/// Result is given as a TransferIterator (see TransferProcess). Also, it is possible to pilot directly the
/// TransientProcess.
/// </summary>
internal class TransferOutput
{
    /// <summary>
    /// Creates a TransferOutput ready to use, with a TransientProcess (created here, sized by the model, with the
    /// actor).
    /// </summary>
    public TransferOutput(ActorOfTransientProcess? actor, InterfaceModel amodel)
    {
        _theproc = new TransientProcess(amodel.EntityCount);
        _theproc.SetActor(actor);
        _themodel = amodel;
        //  thescope = false;
        //  theundef = Transfer_UndefIgnore;
    }

    /// <summary>Creates a TransferOutput from an already existing TransientProcess, and a Model.</summary>
    public TransferOutput(TransientProcess proc, InterfaceModel? amodel)
    {
        _theproc = proc;
        _themodel = amodel;
        //  thescope = false; //szv#4:S4163:12Mar99 initialization needed
        //  theundef = Transfer_UndefIgnore;
    }

    /// <summary>Returns the Starting Model.</summary>
    public InterfaceModel? Model()
    {
        return _themodel;
    }

    /// <summary>Returns the TransientProcess used to work.</summary>
    public TransientProcess TransientProcess()
    {
        return _theproc;
    }

    /// <summary>
    /// Transfer checks that all taken Entities come from the same Model, then calls Transfer from TransientProcess.
    /// </summary>
    /// <exception cref="TransferFailureException">The entity is not in the model.</exception>
    public void Transfer(Transient? obj, ProgressRange progress = default)
    {
        if (_themodel.Number(obj) == 0)
        {
            throw new TransferFailureException("TransferOutput : Transfer, entities do not come from same initial model");
        }
        //  int scope = 0;
        //  if (thescope) scope = theproc->NewScope (obj);

        //: 1 modified by ABV 5 Nov 97
        //: 1  if (!theproc->Transfer(obj)) return;    // auparavant, traitement Undefined
        //  bool ok =
        _theproc.Transfer(obj, progress);
        //  if (scope > 0) theproc->EndScope (scope);
        //  if ( ! ok ) return;
    }

    //  Results :
    //   To transfer quite simply all roots of an interface model
    //   Each one is noted "Root" in the final Process

    /// <summary>
    /// Runs transfer on the roots of the Interface Model. The Roots are computed with the model's protocol, where OCCT
    /// uses the process-wide active protocol (decision 1183).
    /// </summary>
    public void TransferRoots(ProgressRange progress = default)
    {
        TransferRoots(_themodel.Protocol(), progress);
    }

    /// <summary>
    /// Runs transfer on the roots of the Interface Model. The Roots are computed with a ShareFlags created from a
    /// Protocol given as Argument.
    /// </summary>
    public void TransferRoots(Protocol? protocol, ProgressRange progress = default)
    {
        _theproc.SetRootManagement(false);
        var tool = new ShareFlags(_themodel, protocol);
        EntityIterator list = tool.RootEntities();
        using var ps = new ProgressScope(progress, null, list.NbEntities());
        for (list.Start(); list.More() && ps.More(); list.Next())
        {
            Transient? ent = list.Value();
            //    int scope = 0;
            //    if (thescope) scope = theproc->NewScope (ent);
            if (_theproc.Transfer(ent, ps.Next()))
            {
                _theproc.SetRoot(ent);
            }
            //    if (scope > 0) theproc->EndScope (scope);
        }
    }

    /// <summary>Runs transfer on the roots defined by a Graph of dependences (which detains also a Model and its Entities).</summary>
    public void TransferRoots(EntityGraph g, ProgressRange progress = default)
    {
        _theproc.SetRootManagement(false);
        var tool = new ShareFlags(g);
        _theproc.SetModel(g.Model());
        EntityIterator list = tool.RootEntities();
        using var ps = new ProgressScope(progress, null, list.NbEntities());
        for (list.Start(); list.More() && ps.More(); list.Next())
        {
            Transient? ent = list.Value();
            //    int scope = 0;
            //    if (thescope) scope = theproc->NewScope (ent);
            if (_theproc.Transfer(ent, ps.Next()))
            {
                _theproc.SetRoot(ent);
            }
            //    if (scope > 0) theproc->EndScope (scope);
        }
    }

    /// <summary>
    /// Returns the list of Starting Entities with these criteria : <paramref name="normal"/> False, gives the entities
    /// bound with ABNORMAL STATUS (e.g. : Fail recorded, Exception raised during Transfer, etc...);
    /// <paramref name="normal"/> True, gives Entities with or without a Result, but with no Fail, no Exception (Warnings
    /// are not counted). <paramref name="roots"/> True (D), considers only roots of Transfer (the Entities recorded at
    /// highest level); False, considers all Entities bound with a Binder.
    /// </summary>
    public EntityIterator ListForStatus(bool normal, bool roots = true)
    {
        var list = new EntityIterator();
        int max = roots ? _theproc.RootCount : _theproc.MappedCount;
        for (int i = 1; i <= max; i++)
        {
            Binder? binder = roots ? _theproc.RootItem(i) : _theproc.MapItem(i);
            if (binder is null)
            {
                continue;
            }
            StatusExec statex = binder.StatusExec();
            bool ok = statex == StatusExec.StatusInitial || statex == StatusExec.StatusDone;
            if (ok == normal)
            {
                list.AddItem(roots ? _theproc.Root(i) : _theproc.Mapped(i));
            }
        }
        return list;
    }

    /// <summary>
    /// Fills a Model with the list determined by ListForStatus. This model starts from scratch (made by NewEmptyModel
    /// from the current Model), then is filled by AddWithRefs.
    /// </summary>
    public InterfaceModel? ModelForStatus(Protocol? protocol, bool normal, bool roots = true)
    {
        InterfaceModel? newmod = null;
        if (_themodel is null)
        {
            return newmod;
        }
        newmod = _themodel.NewEmptyModel();
        EntityIterator list = ListForStatus(normal, roots);
        for (list.Start(); list.More(); list.Next())
        {
            newmod.AddWithRefs(list.Value(), protocol);
        }
        return newmod;
    }

    private readonly TransientProcess _theproc;
    private readonly InterfaceModel? _themodel;
}
