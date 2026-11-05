// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;

namespace Kerf.Kernel.DataExchange.Tests.Interface;

/// <summary>
/// A toy norm for the tests of the exchange framework: entities that share others (and imply others), a protocol, a
/// model, and a general module registered once in <see cref="GeneralLib"/>'s global list. Before StepData
/// exists (unit 8B), this is the only way to drive the graph, share, check and copy tools.
/// </summary>
internal sealed class ToyEntity(string theName) : Transient
{
    public string Name = theName;
    public readonly List<ToyEntity> Refs = [];
    public readonly List<ToyEntity> Implied = [];
    public bool Fails;
    public bool Warns;

    public ToyEntity Ref(params ToyEntity[] theRefs)
    {
        Refs.AddRange(theRefs);
        return this;
    }

    public override string ToString() => Name;
}

internal sealed class ToyUnknown : Transient
{
}

internal sealed class ToyProtocol : Protocol
{
    public override int ResourceCount => 0;

    public override Protocol? Resource(int num) => null;

    public override int TypeNumber(Type? atype) =>
        atype == typeof(ToyEntity) ? 1 : atype == typeof(ToyUnknown) ? 2 : atype == typeof(TCollection_HAsciiString) ? 3 : 0;

    public override InterfaceModel? NewModel() => new ToyModel();

    public override bool IsSuitableModel(InterfaceModel? model) => model is ToyModel;

    public override Transient? UnknownEntity() => new ToyUnknown();

    public override bool IsUnknownEntity(Transient? ent) => ent is ToyUnknown;
}

internal sealed class ToyGeneralModule : GeneralModule
{
    public override void FillSharedCase(int CN, Transient? ent, EntityIterator iter)
    {
        if (ent is ToyEntity anEnt)
        {
            foreach (ToyEntity aRef in anEnt.Refs)
            {
                iter.GetOneItem(aRef);
            }
        }
    }

    public override void ListImpliedCase(int CN, Transient? ent, EntityIterator iter)
    {
        if (ent is ToyEntity anEnt)
        {
            foreach (ToyEntity aRef in anEnt.Implied)
            {
                iter.GetOneItem(aRef);
            }
        }
    }

    public override void CheckCase(int CN, Transient? ent, ShareTool shares, ref FrameworkCheck? ach)
    {
        if (ent is ToyEntity anEnt)
        {
            if (anEnt.Fails)
            {
                ach!.AddFail("fails: " + anEnt.Name);
            }
            if (anEnt.Warns)
            {
                ach!.AddWarning("warns: " + anEnt.Name);
            }
        }
    }

    public override bool NewVoid(int CN, out Transient? entto)
    {
        entto = CN switch
        {
            1 => new ToyEntity(""),
            2 => new ToyUnknown(),
            _ => null,
        };
        return entto is not null;
    }

    public override void CopyCase(int CN, Transient? entfrom, Transient? entto, CopyTool TC)
    {
        if (entfrom is ToyEntity aFrom && entto is ToyEntity aTo)
        {
            aTo.Name = aFrom.Name + "'";
            foreach (ToyEntity aRef in aFrom.Refs)
            {
                aTo.Refs.Add((ToyEntity)TC.Transferred(aRef)!);
            }
        }
    }

    public override int CategoryNumber(int CN, Transient? ent, ShareTool shares) =>
        CN == 1 ? Category.Number("Structure") : 0;

    public override TCollection_HAsciiString? Name(int CN, Transient? ent, ShareTool shares) =>
        ent is ToyEntity anEnt ? new TCollection_HAsciiString(anEnt.Name) : null;
}

internal sealed class ToyModel : InterfaceModel
{
    public override void ClearLabels()
    {
    }

    public override void ClearHeader()
    {
    }

    public override void GetFromAnother(InterfaceModel? other)
    {
    }

    public override InterfaceModel? NewEmptyModel() => new ToyModel();

    public override void DumpHeader(TextWriter S, int level = 0) => S.Write("toy header");

    public override void PrintLabel(Transient? ent, TextWriter S) => S.Write(StringLabel(ent)?.ToCString());

    public override TCollection_HAsciiString? StringLabel(Transient? ent) =>
        ent is ToyEntity anEnt ? new TCollection_HAsciiString("#" + anEnt.Name) : new TCollection_HAsciiString("?");
}

internal static class ToyNorm
{
    // The protocol and its module, registered once (a protocol's one-time initialisation)
    public static readonly ToyProtocol ProtocolInstance = Register();

    private static ToyProtocol Register()
    {
        var aProtocol = new ToyProtocol();
        GeneralLib.SetGlobal(new ToyGeneralModule(), aProtocol);
        return aProtocol;
    }

    /// <summary>A model of the entities, in order, with the toy protocol.</summary>
    public static ToyModel Model(params ToyEntity[] theEntities)
    {
        var aModel = new ToyModel();
        aModel.SetProtocol(ProtocolInstance);
        foreach (ToyEntity anEnt in theEntities)
        {
            aModel.AddEntity(anEnt);
        }
        return aModel;
    }

    /// <summary>The names of an iteration's entities, in order.</summary>
    public static string Names(EntityIterator theIter)
    {
        var aNames = new List<string>();
        for (theIter.Start(); theIter.More(); theIter.Next())
        {
            aNames.Add(theIter.Value()!.ToString()!);
        }
        return string.Join(" ", aNames);
    }

    /// <summary>The names, sorted (for results whose order isn't meaningful).</summary>
    public static string Sorted(EntityIterator theIter)
    {
        var aNames = Names(theIter).Split(' ', StringSplitOptions.RemoveEmptyEntries).ToList();
        aNames.Sort(StringComparer.Ordinal);
        return string.Join(" ", aNames);
    }
}
