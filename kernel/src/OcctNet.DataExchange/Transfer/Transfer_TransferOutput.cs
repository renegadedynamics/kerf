// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferOutput.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferOutput.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.Transfer_StatusExec;

namespace Occt;

/// <summary>
/// A TransferOutput is a Tool which manages the transfer of entities created by an Interface, stored in an
/// InterfaceModel, into a set of Objects suitable for an Application. Objects to be transferred are given, by method
/// Transfer (which calls Transfer from TransientProcess). A default action is available to get all roots of the Model.
/// Result is given as a TransferIterator (see TransferProcess). Also, it is possible to pilot directly the
/// TransientProcess.
/// </summary>
public class Transfer_TransferOutput
{
    /// <summary>
    /// Creates a TransferOutput ready to use, with a TransientProcess (created here, sized by the model, with the
    /// actor).
    /// </summary>
    public Transfer_TransferOutput(Transfer_ActorOfTransientProcess? actor, Interface_InterfaceModel amodel)
    {
        theproc = new Transfer_TransientProcess(amodel.NbEntities());
        theproc.SetActor(actor);
        themodel = amodel;
        //  thescope = false;
        //  theundef = Transfer_UndefIgnore;
    }

    /// <summary>Creates a TransferOutput from an already existing TransientProcess, and a Model.</summary>
    public Transfer_TransferOutput(Transfer_TransientProcess proc, Interface_InterfaceModel? amodel)
    {
        theproc = proc;
        themodel = amodel;
        //  thescope = false; //szv#4:S4163:12Mar99 initialization needed
        //  theundef = Transfer_UndefIgnore;
    }

    /// <summary>Returns the Starting Model.</summary>
    public Interface_InterfaceModel? Model()
    {
        return themodel;
    }

    /// <summary>Returns the TransientProcess used to work.</summary>
    public Transfer_TransientProcess TransientProcess()
    {
        return theproc;
    }

    /// <summary>
    /// Transfer checks that all taken Entities come from the same Model, then calls Transfer from TransientProcess.
    /// </summary>
    /// <exception cref="Transfer_TransferFailure">The entity is not in the model.</exception>
    public void Transfer(Transient? obj, ProgressRange theProgress = default)
    {
        if (themodel.Number(obj) == 0)
        {
            throw new Transfer_TransferFailure("TransferOutput : Transfer, entities do not come from same initial model");
        }
        //  int scope = 0;
        //  if (thescope) scope = theproc->NewScope (obj);

        //: 1 modified by ABV 5 Nov 97
        //: 1  if (!theproc->Transfer(obj)) return;    // auparavant, traitement Undefined
        //  bool ok =
        theproc.Transfer(obj, theProgress);
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
    public void TransferRoots(ProgressRange theProgress = default)
    {
        TransferRoots(themodel.Protocol(), theProgress);
    }

    /// <summary>
    /// Runs transfer on the roots of the Interface Model. The Roots are computed with a ShareFlags created from a
    /// Protocol given as Argument.
    /// </summary>
    public void TransferRoots(Interface_Protocol? protocol, ProgressRange theProgress = default)
    {
        theproc.SetRootManagement(false);
        var tool = new Interface_ShareFlags(themodel, protocol);
        Interface_EntityIterator list = tool.RootEntities();
        using var aPS = new ProgressScope(theProgress, null, list.NbEntities());
        for (list.Start(); list.More() && aPS.More(); list.Next())
        {
            Transient? ent = list.Value();
            //    int scope = 0;
            //    if (thescope) scope = theproc->NewScope (ent);
            if (theproc.Transfer(ent, aPS.Next()))
            {
                theproc.SetRoot(ent);
            }
            //    if (scope > 0) theproc->EndScope (scope);
        }
    }

    /// <summary>Runs transfer on the roots defined by a Graph of dependences (which detains also a Model and its Entities).</summary>
    public void TransferRoots(Interface_Graph G, ProgressRange theProgress = default)
    {
        theproc.SetRootManagement(false);
        var tool = new Interface_ShareFlags(G);
        theproc.SetModel(G.Model());
        Interface_EntityIterator list = tool.RootEntities();
        using var aPS = new ProgressScope(theProgress, null, list.NbEntities());
        for (list.Start(); list.More() && aPS.More(); list.Next())
        {
            Transient? ent = list.Value();
            //    int scope = 0;
            //    if (thescope) scope = theproc->NewScope (ent);
            if (theproc.Transfer(ent, aPS.Next()))
            {
                theproc.SetRoot(ent);
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
    public Interface_EntityIterator ListForStatus(bool normal, bool roots = true)
    {
        var list = new Interface_EntityIterator();
        int max = roots ? theproc.NbRoots() : theproc.NbMapped();
        for (int i = 1; i <= max; i++)
        {
            Transfer_Binder? binder = roots ? theproc.RootItem(i) : theproc.MapItem(i);
            if (binder is null)
            {
                continue;
            }
            Transfer_StatusExec statex = binder.StatusExec();
            bool ok = statex == Transfer_StatusExec.Transfer_StatusInitial || statex == Transfer_StatusExec.Transfer_StatusDone;
            if (ok == normal)
            {
                list.AddItem(roots ? theproc.Root(i) : theproc.Mapped(i));
            }
        }
        return list;
    }

    /// <summary>
    /// Fills a Model with the list determined by ListForStatus. This model starts from scratch (made by NewEmptyModel
    /// from the current Model), then is filled by AddWithRefs.
    /// </summary>
    public Interface_InterfaceModel? ModelForStatus(Interface_Protocol? protocol, bool normal, bool roots = true)
    {
        Interface_InterfaceModel? newmod = null;
        if (themodel is null)
        {
            return newmod;
        }
        newmod = themodel.NewEmptyModel();
        Interface_EntityIterator list = ListForStatus(normal, roots);
        for (list.Start(); list.More(); list.Next())
        {
            newmod.AddWithRefs(list.Value(), protocol);
        }
        return newmod;
    }

    private readonly Transfer_TransientProcess theproc;
    private readonly Interface_InterfaceModel? themodel;
}
