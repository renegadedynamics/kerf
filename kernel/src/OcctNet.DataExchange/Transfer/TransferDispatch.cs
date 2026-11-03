// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferDispatch.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferDispatch.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// A TransferDispatch is aimed to dispatch Entities between two Interface Models, by default by copying them, as
/// CopyTool, but with more capabilities of adapting : Copy is redefined to firstly pass the hand to a TransferProcess.
/// If this gives no result, standard Copy is called.
/// </summary>
/// <remarks>
/// This allow, for instance, to modify the copied Entity (such as changing a Name for a VDA Entity), or to do a deeper
/// work (such as Substituting a kind of Entity to another one).
/// </remarks>
public class TransferDispatch : CopyTool
{
    /// <summary>
    /// Creates a TransferDispatch from a Model. Works with a General Service Library, given as an Argument. A
    /// TransferDispatch is created as a CopyTool in which the Control is set to TransientProcess.
    /// </summary>
    public TransferDispatch(InterfaceModel amodel, GeneralLib lib)
        : base(amodel, lib)
    {
        SetControl(new DispatchControl(amodel, new TransientProcess(amodel.EntityCount)));
    }

    /// <summary>Same as above, but Library is defined through a Protocol.</summary>
    public TransferDispatch(InterfaceModel amodel, Protocol? protocol)
        : base(amodel, protocol)
    {
        SetControl(new DispatchControl(amodel, new TransientProcess(amodel.EntityCount)));
    }

    /// <summary>Same as above, but works with the model's protocol (OCCT: the active protocol, decision 1183).</summary>
    public TransferDispatch(InterfaceModel amodel)
        : base(amodel)
    {
        SetControl(new DispatchControl(amodel, new TransientProcess(amodel.EntityCount)));
    }

    /// <summary>Returns the content of Control Object, as a TransientProcess.</summary>
    public TransientProcess TransientProcess()
    {
        return (Control() as DispatchControl).TransientProcess();
    }

    /// <summary>
    /// Copies an Entity by calling the method Transferring from the TransferProcess. If this called produces a Null
    /// Binder, then the standard, inherited Copy is called.
    /// </summary>
    public override bool Copy(Transient? entfrom, ref Transient? entto, bool mapped, bool errstat)
    {
        // OCCT.NET fix (decision 1207): the entity Transfer_ActorDispatch is copying gets the standard copy. OCCT asks
        // the process again, which calls the same actor again: an endless recursion (a stack overflow)
        if (entfrom is not null && entfrom == _dispatched)
        {
            return base.Copy(entfrom, ref entto, mapped, errstat);
        }
        Binder? result = TransientProcess().Transferring(entfrom);
        if (result is null)
        {
            return base.Copy(entfrom, ref entto, mapped, errstat);
        }
        if (!result.IsKind(typeof(SimpleBinderOfTransient)))
        {
            return false; // Produces something, but what ?
        }
        entto = (result as SimpleBinderOfTransient).Result();
        return true;
    }

    // The entity Transfer_ActorDispatch.Transfer is copying (decision 1207)
    internal Transient? _dispatched;
}
