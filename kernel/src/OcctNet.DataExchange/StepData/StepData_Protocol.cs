// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Protocol.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Protocol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Description of Basic Protocol for Step. The class Protocol from StepData itself describes a default Protocol,
/// which recognizes only UnknownEntities. Sub-classes will redefine CaseNumber and, if necessary, NbResources and
/// Resources.
/// </summary>
/// <remarks>
/// The descriptions (late binding) are kept in maps; <see cref="ECDescr"/> iterates the map of descriptions, keyed by
/// reference in OCCT, in insertion order here (decision 1194).
/// </remarks>
public class StepData_Protocol : Interface_Protocol
{
    //  The base Protocol recognizes UnknownEntity
    private const string thename = "(DEFAULT)";

    private readonly NCollection_DataMap<Standard_Transient, int> thedscnum = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_DataMap<string, Standard_Transient> thedscnam = new();
    private readonly NCollection_DataMap<string, Standard_Transient> thepdescr = new();
    private readonly NCollection_DataMap<string, Standard_Transient> thedscbas = new();

    public StepData_Protocol()
    {
    }

    /// <summary>Gives the count of Protocols used as Resource (can be zero). Here, No resource.</summary>
    public override int NbResources()
    {
        return 0;
    }

    /// <summary>Returns a Resource, given a rank. Here, none.</summary>
    public override Interface_Protocol? Resource(int num)
    {
        Interface_Protocol nulproto = null;
        return nulproto;
    }

    /// <summary>Returns a unique positive number for any recognized entity. Redefined to work by default on both Dynamic and Static types (for Static, redefines CaseNumber of Protocol).</summary>
    public override int CaseNumber(Standard_Transient? obj)
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
        if (obj is not StepData_Described dc)
        {
            return 0;
        }
        return DescrNumber(dc.Description());
    }

    /// <summary>Returns a Case Number, specific of each recognized Type. Here, only Unknown Entity is recognized.</summary>
    public override int TypeNumber(Type? atype)
    {
        if (atype == typeof(StepData_UndefinedEntity))
        {
            return 1;
        }
        return 0;
    }

    /// <summary>Returns the Schema Name attached to each class of Protocol. To be redefined by each sub-class. Here, SchemaName returns "(DEFAULT)". Was C++: return const.</summary>
    public virtual string SchemaName(Interface_InterfaceModel? theModel)
    {
        return thename;
    }

    /// <summary>Creates an empty Model for Step Norm.</summary>
    public override Interface_InterfaceModel? NewModel()
    {
        return new StepData_StepModel();
    }

    /// <summary>Returns True if model is a Model of Step Norm.</summary>
    public override bool IsSuitableModel(Interface_InterfaceModel? model)
    {
        return model is StepData_StepModel;
    }

    /// <summary>Creates a new Unknown Entity for Step (UndefinedEntity).</summary>
    public override Standard_Transient? UnknownEntity()
    {
        return new StepData_UndefinedEntity();
    }

    /// <summary>Returns True if Unknown Entity is (kind of) an UndefinedEntity from StepData.</summary>
    public override bool IsUnknownEntity(Standard_Transient? ent)
    {
        if (ent is not null)
        {
            return ent is StepData_UndefinedEntity;
        }
        return false;
    }

    //  ####    Description for LateBinding (runtime entity description support)

    /// <summary>Returns a unique positive CaseNumber for types described by an EDescr (late binding). Warning: TypeNumber and DescrNumber must give together a unique positive case number for each distinct case, type or descr.</summary>
    public virtual int DescrNumber(StepData_EDescr? adescr)
    {
        if (adescr is not null && thedscnum.IsBound(adescr))
        {
            return thedscnum.Find(adescr);
        }
        return 0;
    }

    /// <summary>Records an EDescr with its case number. Also records its name for an ESDescr (simple type): an ESDescr is then used, for case number, or for type name.</summary>
    public void AddDescr(StepData_EDescr adescr, int CN)
    {
        StepData_ESDescr sd = adescr as StepData_ESDescr;
        thedscnum.Bind(adescr, CN);

        //  Simple descriptor: stored according to its name
        //  Otherwise what to do? Store according to the passed number in alpha-numeric form...
        //  (temporary solution)
        if (sd is not null)
        {
            thedscnam.Bind(sd.TypeName(), sd);
        }
        string fonom = Standard.Sprintf("%d", CN);
        thedscnam.Bind(fonom, adescr);
    }

    /// <summary>Tells if a Protocol brings at least one ESDescr, i.e. if it defines at least one entity description by ESDescr mechanism.</summary>
    public bool HasDescr()
    {
        return !thedscnam.IsEmpty();
    }

    /// <summary>Returns the description attached to a case number, or null.</summary>
    public StepData_EDescr? Descr(int num)
    {
        StepData_EDescr dsc = null;
        if (thedscnam.IsEmpty())
        {
            return dsc;
        }
        string fonom = Standard.Sprintf("%d", num);
        if (thedscnam.Find(fonom, out Standard_Transient aTDsc))
        {
            dsc = aTDsc as StepData_EDescr;
        }
        else
        {
            dsc = null;
        }
        return dsc;
    }

    /// <summary>Returns a description according to its name. anylevel True (D) means that if it is not defined in this protocol, it is searched in the resources.</summary>
    public StepData_EDescr? Descr(string name, bool anylevel = true)
    {
        StepData_EDescr sd = null;
        if (!thedscnam.IsEmpty())
        {
            if (thedscnam.Find(name, out Standard_Transient aTSd))
            {
                return aTSd as StepData_EDescr;
            }
        }
        if (!anylevel)
        {
            return sd;
        }

        int i, nb = NbResources();
        for (i = 1; i <= nb; i++)
        {
            if (Resource(i) is not StepData_Protocol sp)
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
    public StepData_ESDescr? ESDescr(string name, bool anylevel = true)
    {
        return Descr(name, anylevel) as StepData_ESDescr;
    }

    /// <summary>Returns a complex description according to list of names. anylevel True (D) means that if it is not defined in this protocol, it is searched in the resources.</summary>
    public StepData_ECDescr? ECDescr(NCollection_Sequence<string> names, bool anylevel = true)
    {
        int i, nb = names.Length();
        StepData_ECDescr cd;
        foreach (KeyValuePair<Standard_Transient, int> iter in thedscnum) // OCCT.NET: insertion order (decision 1194)
        {
            cd = iter.Key as StepData_ECDescr;
            if (cd is null)
            {
                continue;
            }
            if (cd.NbMembers() != nb)
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

        nb = NbResources();
        for (i = 1; i <= nb; i++)
        {
            if (Resource(i) is not StepData_Protocol sp)
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
    public void AddPDescr(StepData_PDescr pdescr)
    {
        thepdescr.Bind(pdescr.Name(), pdescr);
    }

    /// <summary>Returns a parameter description according to its name. anylevel True (D) means that if it is not defined in this protocol, it is searched in the resources.</summary>
    public StepData_PDescr? PDescr(string name, bool anylevel = true)
    {
        StepData_PDescr sd = null;
        if (!thepdescr.IsEmpty())
        {
            if (thepdescr.Find(name, out Standard_Transient aTSd))
            {
                return aTSd as StepData_PDescr;
            }
        }
        if (!anylevel)
        {
            return sd;
        }

        int i, nb = NbResources();
        for (i = 1; i <= nb; i++)
        {
            if (Resource(i) is not StepData_Protocol sp)
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
    public void AddBasicDescr(StepData_ESDescr esdescr)
    {
        thedscbas.Bind(esdescr.TypeName(), esdescr);
    }

    /// <summary>Returns a basic description according to its name. anylevel True (D) means that if it is not defined in this protocol, it is searched in the resources.</summary>
    public StepData_EDescr? BasicDescr(string name, bool anylevel = true)
    {
        StepData_EDescr sd = null;
        if (!thedscbas.IsEmpty())
        {
            if (thedscbas.Find(name, out Standard_Transient aTSd))
            {
                return aTSd as StepData_EDescr;
            }
        }
        if (!anylevel)
        {
            return sd;
        }

        int i, nb = NbResources();
        for (i = 1; i <= nb; i++)
        {
            if (Resource(i) is not StepData_Protocol sp)
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
