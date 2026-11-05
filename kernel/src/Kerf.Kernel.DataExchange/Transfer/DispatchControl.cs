// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_DispatchControl.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_DispatchControl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// This is an auxiliary class for TransferDispatch, which allows to record simple copies, as CopyControl from
/// Interface, but based on a TransientProcess. Hence, it allows in addition more actions (such as recording results
/// of adaptations).
/// </summary>
internal class DispatchControl : CopyControl
{
    /// <summary>Creates the DispatchControl, ready for use.</summary>
    public DispatchControl(InterfaceModel? model, TransientProcess tp)
    {
        _themodel = model;
        _tp = tp;
    }

    /// <summary>Returns the content of the DispatchControl : it can be used for a direct call, if the basic methods do not suffice.</summary>
    public TransientProcess TransientProcess()
    {
        return _tp;
    }

    /// <summary>Returns the Model from which the transfer is to be done.</summary>
    public InterfaceModel? StartingModel()
    {
        return _themodel;
    }

    /// <summary>Clears the List of Copied Results.</summary>
    public override void Clear()
    {
        _tp.Clear();
    }

    /// <summary>Binds a (Transient) Result to a (Transient) Starting Entity.</summary>
    public override void Bind(Transient? ent, Transient? res)
    {
        _tp.BindTransient(ent, res);
    }

    /// <summary>
    /// Searches for the Result bound to a Starting Entity. If Found, returns True and fills <paramref name="res"/>.
    /// Else, returns False and nullifies <paramref name="res"/>.
    /// </summary>
    public override bool Search(Transient? ent, out Transient? res)
    {
        res = _tp.FindTransient(ent);
        return res is not null;
    }

    private readonly TransientProcess _tp;
    private readonly InterfaceModel? _themodel;
}
