// Port of: src/DataExchange/TKXSBase/Interface/Interface_CopyMap.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_CopyMap.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Manages a Map for the need of single Transfers, such as Copies In such transfer, Starting Entities are
/// read from a unique Starting Model, and each transferred Entity is bound to one and only one Result, which
/// cannot be changed later.
/// </summary>
internal class CopyMap : CopyControl
{

    /// <summary>
    /// Creates a CopyMap adapted to work from a Model
    /// </summary>
    public CopyMap(InterfaceModel? amodel)
    {
        _theres = new NCollection_Array1<Transient>(0, amodel.EntityCount);
        _themod = amodel;
    }

    /// <summary>
    /// Clears Transfer List. Gets Ready to begin another Transfer
    /// </summary>
    public override void Clear()
    {
        int nb = _theres.Upper();
        Transient bid = null; // Null
        for (int i = 1; i <= nb; i++)
        {
            _theres.SetValue(i, bid);
        }
    }

    /// <summary>
    /// Returns the InterfaceModel used at Creation time
    /// </summary>
    public InterfaceModel? Model()
    {
        return _themod;
    }

    /// <summary>
    /// Binds a Starting Entity identified by its Number &lt;num&gt; in the Starting Model, to a Result of
    /// Transfer &lt;res&gt;
    /// </summary>
    public override void Bind(Transient? ent, Transient? res)
    {
        int num = _themod.Number(ent);
        if (num == 0 || num > _theres.Upper())
        {
            throw new InterfaceErrorException(
                "CopyMap : Bind, Starting Entity not issued from Starting Model");
        }
        if (_theres.Value(num) is not null)
        {
            throw new InterfaceErrorException("CopyMap : Bind, Starting Entity already bound");
        }
        _theres.SetValue(num, res);
    }

    /// <summary>
    /// Search for the result of a Starting Object (i.e. an Entity, identified by its Number &lt;num&gt; in the
    /// Starting Model) Returns True if a Result is Bound (and fills &lt;res&gt;) Returns False if no result is
    /// Bound (and nullifies &lt;res&gt;)
    /// </summary>
    public override bool Search(Transient? ent, out Transient? res)
    {
        int num = _themod.Number(ent);
        if (num == 0)
        {
            res = null; // OCCT leaves <res> as it was
            return false;
        }
        res = _theres.Value(num);
        return (res is not null);
    }

    private InterfaceModel? _themod;
    private NCollection_Array1<Transient> _theres;
}
