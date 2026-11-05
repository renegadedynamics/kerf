// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeMapper.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeMapper_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// A Finder for a Shape: two mappers are the same if their shapes are the same (TopTools_ShapeMapHasher: the same
/// TShape at the same location, whatever the orientation).
/// </summary>
internal class ShapeMapper : Finder
{
    /// <summary>
    /// Creates a Mapper with a Value. This Value can then not be changed. It is used by the Hasher to compute the
    /// HashCode, which will then be stored for an immediate reading.
    /// </summary>
    public ShapeMapper(in Shape akey)
    {
        _theval = akey;
        SetHashCode(ShapeHasher.Instance.GetHashCode(akey));
    }

    /// <summary>Returns the contained value.</summary>
    public Shape Value()
    {
        return _theval;
    }

    /// <summary>
    /// Specific test of equality : defined as False if <paramref name="other"/> has not the same true Type, else
    /// contents are compared (TopTools_ShapeMapHasher).
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
        ShapeMapper another = (ShapeMapper)other;
        return ShapeHasher.Instance.Equals(_theval, another.Value());
    }

    /// <summary>Returns the Type of the Value.</summary>
    public override Type? ValueType()
    {
        return TransferShapeInfo.Type(_theval);
    }

    /// <summary>Returns the name of the Type of the Value.</summary>
    public override string ValueTypeName()
    {
        return TransferShapeInfo.TypeName(_theval);
    }

    private readonly Shape _theval;
}
