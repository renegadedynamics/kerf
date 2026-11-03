// Port of: src/DataExchange/TKXSBase/Interface/Interface_GTool.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_GTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// GTool - General Tool for a Model Provides the functions performed by Protocol/GeneralModule for entities
/// of a Model, and recorded in a GeneralLib Optimized : once an entity has been queried, the GeneralLib is
/// not longer queried Shareable between several users : as a Handle
/// </summary>
public class GTool : Transient
{

    /// <summary>
    /// Creates an empty, not set, GTool
    /// </summary>
    public GTool()
    {
    }

    /// <summary>
    /// Creates a GTool from a Protocol Optional starting count of entities
    /// </summary>
    public GTool(Protocol? proto, int nb = 0)
    {
        _theproto = proto;
        _thelib = new GeneralLib(proto);
        // OCCT's map reservation (nb > 0) dropped: .NET maps grow
    }

    /// <summary>
    /// Sets a new SignType
    /// </summary>
    public void SetSignType(SignType? sign)
    {
        _thesign = sign;
    }

    /// <summary>
    /// Returns the SignType. Can be null
    /// </summary>
    public SignType? SignType()
    {
        return _thesign;
    }

    /// <summary>
    /// Returns the Signature for a Transient Object in a Model It calls SignType to do that If SignType is not
    /// defined, return ClassName of &lt;ent&gt;
    /// </summary>
    public string SignValue(Transient? ent, InterfaceModel? model)
    {
        if (ent is null)
        {
            return "";
        }
        if (_thesign is null)
        {
            return global::Kerf.Engine.Exchange.Framework.SignType.ClassName(ent.DynamicTypeName());
        }
        return _thesign.Value(ent, model);
    }

    /// <summary>
    /// Returns the Name of the SignType, or "Class Name"
    /// </summary>
    public string SignName()
    {
        if (_thesign is null)
        {
            return "Class Name";
        }
        return _thesign.Name();
    }

    /// <summary>
    /// Sets a new Protocol if &lt;enforce&gt; is False and the new Protocol equates the old one then nothing is
    /// done
    /// </summary>
    public void SetProtocol(Protocol? proto, bool enforce = false)
    {
        if (proto == _theproto && !enforce)
        {
            return;
        }
        _theproto = proto;
        _thelib.Clear();
        _thelib.AddProtocol(proto);
    }

    /// <summary>
    /// Returns the Protocol. Warning: it can be Null
    /// </summary>
    public Protocol? Protocol()
    {
        return _theproto;
    }

    /// <summary>
    /// Returns the GeneralLib itself
    /// </summary>
    public GeneralLib Lib()
    {
        return _thelib;
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
        _thentnum.Clear();
        _thentmod.Clear();
    }

    /// <summary>
    /// Selects for an entity, its Module and Case Number It is optimised : once done for each entity, the result
    /// is mapped and the GeneralLib is not longer queried &lt;enforce&gt; True overpasses this optimisation
    /// </summary>
    public bool Select(Transient? ent, out GeneralModule? gmod, out int cn, bool enforce = false)
    {
        Type type = ent.DynamicType();
        int num = _thentmod.FindIndex(type); // (ent);
        if (num == 0 || enforce)
        {
            if (_thelib.Select(ent, out gmod, out cn))
            {
                num = _thentmod.Add(type, gmod);
                _thentnum[type] = cn;
                return true;
            }
            return false;
        }
        gmod = (_thentmod.FindFromKey(type) as GeneralModule);
        cn = _thentnum.GetValue(type);
        return true;
    }

    private Protocol? _theproto;
    private SignType? _thesign;
    private GeneralLib _thelib = new GeneralLib();

    // The cache of selections, by type (OCCT keys them by the Standard_Type handle)
    private readonly Dictionary<Type, int> _thentnum = new();
    private readonly NCollection_IndexedDataMap<Type, Transient> _thentmod = new();
}
