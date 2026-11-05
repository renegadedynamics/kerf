// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientMapper.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientMapper_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>A Finder for a Transient object: two mappers are the same if they map the same object.</summary>
internal class TransientMapper : Finder
{
    /// <summary>
    /// Creates a Mapper with a Value. This Value can then not be changed. It is used by the Hasher to compute the
    /// HashCode, which will then be stored for an immediate reading.
    /// </summary>
    public TransientMapper(Transient? akey)
    {
        _theval = akey;
        // OCCT: std::hash of the handle, its address
        SetHashCode(akey is null ? 0 : RuntimeHelpers.GetHashCode(akey));
    }

    /// <summary>Returns the contained value.</summary>
    public Transient? Value()
    {
        return _theval;
    }

    /// <summary>
    /// Specific test of equality : defined as False if <paramref name="other"/> has not the same true Type, else
    /// contents are compared (by reference).
    /// </summary>
    public override bool Equates(Finder? other)
    {
        if (other is null)
        {
            return false;
        }
        if (GetHashCode() != other.GetHashCode())
        {
            return false;
        }
        if (other.DynamicType() != DynamicType())
        {
            return false;
        }
        TransientMapper another = (TransientMapper)other;
        return _theval == another.Value();
    }

    /// <summary>Returns the Type of the Value.</summary>
    public override Type? ValueType()
    {
        return TransferDataInfo.Type(_theval);
    }

    /// <summary>Returns the name of the Type of the Value.</summary>
    public override string ValueTypeName()
    {
        return TransferDataInfo.TypeName(_theval);
    }

    private readonly Transient? _theval;
}
