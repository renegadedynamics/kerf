// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorDispatch.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorDispatch.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// This class allows to work with a TransferDispatch, i.e. to transfer entities from a data set to another one defined
/// by the same interface norm, with the following features : ActorDispatch itself acts as a default actor, i.e. it
/// copies entities with the general service Copy, as CopyTool does; it allows to add other actors for specific ways
/// of transfer, which may include data modifications, conversions ...; and other features from TransferDispatch (such
/// as mapping other than one-one).
/// </summary>
public class Transfer_ActorDispatch : Transfer_ActorOfTransientProcess
{
    /// <summary>
    /// Creates an ActorDispatch from a Model. Works with a General Service Library, given as an Argument. This causes
    /// TransferDispatch and its TransientProcess to be created, with default actor this one.
    /// </summary>
    public Transfer_ActorDispatch(Interface_InterfaceModel amodel, Interface_GeneralLib lib)
    {
        thetool = new Transfer_TransferDispatch(amodel, lib);
        SetLast(true); // actor par defaut
        thetool.TransientProcess().SetActor(this);
    }

    /// <summary>Same as above, but Library is defined through a Protocol.</summary>
    public Transfer_ActorDispatch(Interface_InterfaceModel amodel, Interface_Protocol? protocol)
    {
        thetool = new Transfer_TransferDispatch(amodel, protocol);
        SetLast(true); // actor par defaut
        thetool.TransientProcess().SetActor(this);
    }

    /// <summary>Same as above, but works with the model's protocol (OCCT: the active protocol, decision 1183).</summary>
    public Transfer_ActorDispatch(Interface_InterfaceModel amodel)
    {
        thetool = new Transfer_TransferDispatch(amodel);
        SetLast(true); // actor par defaut
        thetool.TransientProcess().SetActor(this);
    }

    /// <summary>Utility which adds an actor to the default one (it calls SetActor from the TransientProcess).</summary>
    public void AddActor(Transfer_ActorOfTransientProcess? actor)
    {
        thetool.TransientProcess().SetActor(actor);
    }

    /// <summary>
    /// Returns the TransferDispatch, which does the work, records the intermediate data, etc... See TransferDispatch
    /// &amp; CopyTool, to see the available methods.
    /// </summary>
    public Transfer_TransferDispatch TransferDispatch()
    {
        return thetool;
    }

    /// <summary>
    /// Specific action : it calls the method Transfer from CopyTool i.e. the general service Copy, then returns the
    /// Binder produced by the TransientProcess.
    /// </summary>
    public override Transfer_Binder? Transfer(Transient? start, Transfer_TransientProcess? TP,
                                              ProgressRange theProgress = default)
    {
        // OCCT.NET fix (decision 1207): the copy of <start> is the copy tool's own, not the process's again
        Transient? aFormer = thetool.myDispatched;
        thetool.myDispatched = start;
        try
        {
            thetool.TransferEntity(start);
        }
        finally
        {
            thetool.myDispatched = aFormer;
        }
        return thetool.TransientProcess().Find(start);
    }

    private readonly Transfer_TransferDispatch thetool;
}
