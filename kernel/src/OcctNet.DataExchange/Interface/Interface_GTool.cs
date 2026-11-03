// Port of: src/DataExchange/TKXSBase/Interface/Interface_GTool.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_GTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// GTool - General Tool for a Model Provides the functions performed by Protocol/GeneralModule for entities
/// of a Model, and recorded in a GeneralLib Optimized : once an entity has been queried, the GeneralLib is
/// not longer queried Shareable between several users : as a Handle
/// </summary>
public class Interface_GTool : Transient
{

    /// <summary>
    /// Creates an empty, not set, GTool
    /// </summary>
    public Interface_GTool()
    {
    }

    /// <summary>
    /// Creates a GTool from a Protocol Optional starting count of entities
    /// </summary>
    public Interface_GTool(Interface_Protocol? proto, int nb = 0)
    {
        theproto = proto;
        thelib = new Interface_GeneralLib(proto);
        // OCCT's map reservation (nb > 0) dropped: .NET maps grow
    }

    /// <summary>
    /// Sets a new SignType
    /// </summary>
    public void SetSignType(Interface_SignType? sign)
    {
        thesign = sign;
    }

    /// <summary>
    /// Returns the SignType. Can be null
    /// </summary>
    public Interface_SignType? SignType()
    {
        return thesign;
    }

    /// <summary>
    /// Returns the Signature for a Transient Object in a Model It calls SignType to do that If SignType is not
    /// defined, return ClassName of &lt;ent&gt;
    /// </summary>
    public string SignValue(Transient? ent, Interface_InterfaceModel? model)
    {
        if (ent is null)
        {
            return "";
        }
        if (thesign is null)
        {
            return Interface_SignType.ClassName(ent.DynamicTypeName());
        }
        return thesign.Value(ent, model);
    }

    /// <summary>
    /// Returns the Name of the SignType, or "Class Name"
    /// </summary>
    public string SignName()
    {
        if (thesign is null)
        {
            return "Class Name";
        }
        return thesign.Name();
    }

    /// <summary>
    /// Sets a new Protocol if &lt;enforce&gt; is False and the new Protocol equates the old one then nothing is
    /// done
    /// </summary>
    public void SetProtocol(Interface_Protocol? proto, bool enforce = false)
    {
        if (proto == theproto && !enforce)
        {
            return;
        }
        theproto = proto;
        thelib.Clear();
        thelib.AddProtocol(proto);
    }

    /// <summary>
    /// Returns the Protocol. Warning: it can be Null
    /// </summary>
    public Interface_Protocol? Protocol()
    {
        return theproto;
    }

    /// <summary>
    /// Returns the GeneralLib itself
    /// </summary>
    public Interface_GeneralLib Lib()
    {
        return thelib;
    }

    /// <summary>
    /// Reservates maps for a count of entities &lt;enforce&gt; False : minimum count &lt;enforce&gt; True :
    /// clears former reservations Does not clear the maps
    /// </summary>
    public void Reservate(int nb, bool enforce = false)
    {
        // OCCT reserves its maps' buckets; .NET maps grow, so there is nothing to do
        _ = nb;
        _ = enforce;
    }

    /// <summary>
    /// Clears the maps which record, for each already recorded entity its Module and Case Number
    /// </summary>
    public void ClearEntities()
    {
        thentnum.Clear();
        thentmod.Clear();
    }

    /// <summary>
    /// Selects for an entity, its Module and Case Number It is optimised : once done for each entity, the result
    /// is mapped and the GeneralLib is not longer queried &lt;enforce&gt; True overpasses this optimisation
    /// </summary>
    public bool Select(Transient? ent, out Interface_GeneralModule? gmod, out int CN, bool enforce = false)
    {
        Type aType = ent.DynamicType();
        int num = thentmod.FindIndex(aType); // (ent);
        if (num == 0 || enforce)
        {
            if (thelib.Select(ent, out gmod, out CN))
            {
                num = thentmod.Add(aType, gmod);
                thentnum.Bind(aType, CN);
                return true;
            }
            return false;
        }
        gmod = (thentmod.FindFromKey(aType) as Interface_GeneralModule);
        CN = thentnum.Find(aType);
        return true;
    }

    private Interface_Protocol? theproto;
    private Interface_SignType? thesign;
    private Interface_GeneralLib thelib = new Interface_GeneralLib();

    // The cache of selections, by type (OCCT keys them by the Standard_Type handle)
    private readonly NCollection_DataMap<Type, int> thentnum = new();
    private readonly NCollection_IndexedDataMap<Type, Transient> thentmod = new();
}
