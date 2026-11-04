// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Protocol.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Protocol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// Description of Basic Protocol for Step. The class Protocol from StepData itself describes a default Protocol,
/// which recognizes only UnknownEntities. Sub-classes will redefine CaseNumber and, if necessary, NbResources and
/// Resources.
/// </summary>
/// <remarks>
/// The descriptions (late binding) are kept in maps; <see cref="ECDescr"/> iterates the map of descriptions, keyed by
/// reference in OCCT, in insertion order here (decision 1194).
/// </remarks>
internal class StepProtocol : Protocol
{
    //  The base Protocol recognizes UnknownEntity
    private const string Thename = "(DEFAULT)";

    private readonly Dictionary<Transient, int> _thedscnum = new(ReferenceEqualityComparer.Instance);
    private readonly Dictionary<string, Transient> _thedscnam = new();
    private readonly Dictionary<string, Transient> _thepdescr = new();
    private readonly Dictionary<string, Transient> _thedscbas = new();

    public StepProtocol()
    {
    }

    /// <summary>Gives the count of Protocols used as Resource (can be zero). Here, No resource.</summary>
    public override int ResourceCount
    {
        get
        {
            return 0;
        }
    }

    /// <summary>Returns a Resource, given a rank. Here, none.</summary>
    public override Protocol? Resource(int num)
    {
        Protocol nulproto = null;
        return nulproto;
    }

    /// <summary>Returns a unique positive number for any recognized entity. Redefined to work by default on both Dynamic and Static types (for Static, redefines CaseNumber of Protocol).</summary>
    public override int CaseNumber(Transient? obj)
    {
        if (obj is null)
        {
            return 0;
        }
        int num = TypeNumber(obj.DynamicType());
        if (num > 0)
        {
            return num;
        }
        if (obj is not Described dc)
        {
            return 0;
        }
        return DescrNumber(dc.Description());
    }

    /// <summary>Returns a Case Number, specific of each recognized Type. Here, only Unknown Entity is recognized.</summary>
    public override int TypeNumber(Type? atype)
    {
        if (atype == typeof(UndefinedEntity))
        {
            return 1;
        }
        return 0;
    }

    /// <summary>Returns the Schema Name attached to each class of Protocol. To be redefined by each sub-class. Here, SchemaName returns "(DEFAULT)". Was C++: return const.</summary>
    public virtual string SchemaName(InterfaceModel? model)
    {
        return Thename;
    }

    /// <summary>Creates an empty Model for Step Norm.</summary>
    public override InterfaceModel? NewModel()
    {
        return new StepModel();
    }

    /// <summary>Returns True if model is a Model of Step Norm.</summary>
    public override bool IsSuitableModel(InterfaceModel? model)
    {
        return model is StepModel;
    }

    /// <summary>Creates a new Unknown Entity for Step (UndefinedEntity).</summary>
    public override Transient? UnknownEntity()
    {
        return new UndefinedEntity();
    }

    /// <summary>Returns True if Unknown Entity is (kind of) an UndefinedEntity from StepData.</summary>
    public override bool IsUnknownEntity(Transient? ent)
    {
        if (ent is not null)
        {
            return ent is UndefinedEntity;
        }
        return false;
    }

    //  ####    Description for LateBinding (runtime entity description support)

    /// <summary>Returns a unique positive CaseNumber for types described by an EDescr (late binding). Warning: TypeNumber and DescrNumber must give together a unique positive case number for each distinct case, type or descr.</summary>
    public virtual int DescrNumber(EDescr? adescr)
    {
        if (adescr is not null && _thedscnum.ContainsKey(adescr))
        {
            return _thedscnum.GetValue(adescr);
        }
        return 0;
    }

    /// <summary>Records an EDescr with its case number. Also records its name for an ESDescr (simple type): an ESDescr is then used, for case number, or for type name.</summary>
    public void AddDescr(EDescr adescr, int cn)
    {
        ESDescr sd = adescr as ESDescr;
        _thedscnum[adescr] = cn;

        //  Simple descriptor: stored according to its name
        //  Otherwise what to do? Store according to the passed number in alpha-numeric form...
        //  (temporary solution)
        if (sd is not null)
        {
            _thedscnam[sd.TypeName()] = sd;
        }
        string fonom = global::Kerf.Engine.Foundation.Standard.Sprintf("%d", cn);
        _thedscnam[fonom] = adescr;
    }

    /// <summary>Tells if a Protocol brings at least one ESDescr, i.e. if it defines at least one entity description by ESDescr mechanism.</summary>
    public bool HasDescr
    {
        get
        {
            return _thedscnam.Count != 0;
        }
    }

    /// <summary>Returns the description attached to a case number, or null.</summary>
    public EDescr? Descr(int num)
    {
        EDescr dsc = null;
        if (_thedscnam.Count == 0)
        {
            return dsc;
        }
        string fonom = global::Kerf.Engine.Foundation.Standard.Sprintf("%d", num);
        if (_thedscnam.TryGetValue(fonom, out Transient tDsc))
        {
            dsc = tDsc as EDescr;
        }
        else
        {
            dsc = null;
        }
        return dsc;
    }

    /// <summary>Returns a description according to its name. anylevel True (D) means that if it is not defined in this protocol, it is searched in the resources.</summary>
    public EDescr? Descr(string name, bool anylevel = true)
    {
        EDescr sd = null;
        if (_thedscnam.Count != 0)
        {
            if (_thedscnam.TryGetValue(name, out Transient tSd))
            {
                return tSd as EDescr;
            }
        }
        if (!anylevel)
        {
            return sd;
        }

        int i, nb = ResourceCount;
        for (i = 1; i <= nb; i++)
        {
            if (Resource(i) is not StepProtocol sp)
            {
                continue;
            }
            sd = sp.Descr(name, anylevel);
            if (sd is not null)
            {
                return sd;
            }
        }
        return sd;
    }

    /// <summary>Idem as Descr but cast to simple description.</summary>
    public ESDescr? ESDescr(string name, bool anylevel = true)
    {
        return Descr(name, anylevel) as ESDescr;
    }

    /// <summary>Returns a complex description according to list of names. anylevel True (D) means that if it is not defined in this protocol, it is searched in the resources.</summary>
    public ECDescr? ECDescr(NCollection_Sequence<string> names, bool anylevel = true)
    {
        int i, nb = names.Length();
        ECDescr cd;
        foreach (KeyValuePair<Transient, int> iter in _thedscnum) // OCCT.NET: insertion order (decision 1194)
        {
            cd = iter.Key as ECDescr;
            if (cd is null)
            {
                continue;
            }
            if (cd.MemberCount != nb)
            {
                continue;
            }
            bool ok = true;
            for (i = 1; i <= nb; i++)
            {
                if (!string.Equals(names.Value(i), cd.Member(i).TypeName(), StringComparison.Ordinal))
                {
                    ok = false;
                    break;
                }
            }
            if (ok)
            {
                return cd;
            }
        }
        cd = null;
        if (!anylevel)
        {
            return cd;
        }

        nb = ResourceCount;
        for (i = 1; i <= nb; i++)
        {
            if (Resource(i) is not StepProtocol sp)
            {
                continue;
            }
            cd = sp.ECDescr(names, anylevel);
            if (cd is not null)
            {
                return cd;
            }
        }
        return cd;
    }

    /// <summary>Records an PDescr.</summary>
    public void AddPDescr(PDescr pdescr)
    {
        _thepdescr[pdescr.Name()] = pdescr;
    }

    /// <summary>Returns a parameter description according to its name. anylevel True (D) means that if it is not defined in this protocol, it is searched in the resources.</summary>
    public PDescr? PDescr(string name, bool anylevel = true)
    {
        PDescr sd = null;
        if (_thepdescr.Count != 0)
        {
            if (_thepdescr.TryGetValue(name, out Transient tSd))
            {
                return tSd as PDescr;
            }
        }
        if (!anylevel)
        {
            return sd;
        }

        int i, nb = ResourceCount;
        for (i = 1; i <= nb; i++)
        {
            if (Resource(i) is not StepProtocol sp)
            {
                continue;
            }
            sd = sp.PDescr(name, anylevel);
            if (sd is not null)
            {
                return sd;
            }
        }
        return sd;
    }

    /// <summary>Records an ESDescr, intended to build complex descriptions.</summary>
    public void AddBasicDescr(ESDescr esdescr)
    {
        _thedscbas[esdescr.TypeName()] = esdescr;
    }

    /// <summary>Returns a basic description according to its name. anylevel True (D) means that if it is not defined in this protocol, it is searched in the resources.</summary>
    public EDescr? BasicDescr(string name, bool anylevel = true)
    {
        EDescr sd = null;
        if (_thedscbas.Count != 0)
        {
            if (_thedscbas.TryGetValue(name, out Transient tSd))
            {
                return tSd as EDescr;
            }
        }
        if (!anylevel)
        {
            return sd;
        }

        int i, nb = ResourceCount;
        for (i = 1; i <= nb; i++)
        {
            if (Resource(i) is not StepProtocol sp)
            {
                continue;
            }
            sd = sp.BasicDescr(name, anylevel);
            if (sd is not null)
            {
                return sd;
            }
        }
        return sd;
    }
}
