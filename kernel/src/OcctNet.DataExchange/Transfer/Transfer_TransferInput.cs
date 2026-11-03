// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferInput.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferInput.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// A TransferInput is a Tool which fills an InterfaceModel with the result of the Transfer of CasCade Objects, once
/// determined. The Result comes from a TransferProcess, either from Transient (the Complete Result is considered, it
/// must contain only Transient Objects).
/// </summary>
public class Transfer_TransferInput
{
    /// <summary>Creates a TransferInput ready to use.</summary>
    public Transfer_TransferInput()
    {
    }

    //  Results : For the Model ...

    /// <summary>
    /// Takes the transient items stored in a TransferIterator.
    /// </summary>
    /// <exception cref="Transfer_TransferFailure">A result is not transient.</exception>
    public Interface_EntityIterator Entities(Transfer_TransferIterator list)
    {
        var iter = new Interface_EntityIterator();
        for (list.Start(); list.More(); list.Next())
        {
            Transfer_Binder binder = list.Value();
            if (binder is null)
            {
                continue;
            }
            if (binder.IsKind(typeof(Transfer_VoidBinder)))
            {
                continue;
            }

            // True result : must be transient (simple or list)
            Transfer_SimpleBinderOfTransient? transb = binder as Transfer_SimpleBinderOfTransient;
            Transfer_MultipleBinder? multi = binder as Transfer_MultipleBinder;
            if (transb is not null)
            {
                if (transb.HasResult())
                {
                    iter.AddItem(transb.Result());
                }
            }
            else if (multi is not null)
            {
                NCollection_HSequence<Transient>? mulres = multi.MultipleResult();
                int nbres = 0;
                if (mulres is not null)
                {
                    nbres = mulres.Length();
                }
                for (int i = 1; i <= nbres; i++)
                {
                    iter.AddItem(mulres.Value(i));
                }
            }
            else
            {
                throw new Transfer_TransferFailure("TransferInput : Entities, one of the Results is not Transient Handle");
            }
        }
        return iter;
    }

    /// <summary>
    /// Fills an InterfaceModel with the Complete Result of a Transfer stored in a TransientProcess (Starting Objects are
    /// Transient). The complete result is exactly added to the model.
    /// </summary>
    public void FillModel(Transfer_TransientProcess proc, Interface_InterfaceModel amodel)
    {
        Transfer_TransferIterator list = proc.CompleteResult();
        Interface_EntityIterator iter = Entities(list);
        amodel.GetFromTransfer(iter);
    }

    /// <summary>
    /// Fills an InterfaceModel with results of the Transfer recorded in a TransientProcess (Starting Objects are
    /// Transient) : Root Result if <paramref name="roots"/> is True (Default), Complete Result else. The entities added
    /// to the model are determined from the result by by adding the referenced entities.
    /// </summary>
    public void FillModel(Transfer_TransientProcess proc, Interface_InterfaceModel amodel, Interface_Protocol? proto,
                          bool roots = true)
    {
        Transfer_TransferIterator list;
        if (roots)
        {
            list = proc.RootResult();
        }
        else
        {
            list = proc.CompleteResult();
        }
        Interface_EntityIterator iter = Entities(list);
        for (iter.Start(); iter.More(); iter.Next())
        {
            amodel.AddWithRefs(iter.Value(), proto);
        }
    }

    /// <summary>
    /// Fills an InterfaceModel with the Complete Result of a Transfer stored in a FinderProcess (Starting Objects are
    /// Finders).
    /// </summary>
    public void FillModel(Transfer_FinderProcess proc, Interface_InterfaceModel amodel)
    {
        Transfer_TransferIterator list = proc.CompleteResult();
        Interface_EntityIterator iter = Entities(list);
        amodel.GetFromTransfer(iter);
    }

    /// <summary>
    /// Fills an InterfaceModel with results of the Transfer recorded in a FinderProcess : Root Result if
    /// <paramref name="roots"/> is True (Default), Complete Result else, the referenced entities added.
    /// </summary>
    public void FillModel(Transfer_FinderProcess proc, Interface_InterfaceModel amodel, Interface_Protocol? proto,
                          bool roots = true)
    {
        Transfer_TransferIterator list;
        if (roots)
        {
            list = proc.RootResult();
        }
        else
        {
            list = proc.CompleteResult();
        }
        Interface_EntityIterator iter = Entities(list);
        for (iter.Start(); iter.More(); iter.Next())
        {
            amodel.AddWithRefs(iter.Value(), proto);
        }
    }
}
