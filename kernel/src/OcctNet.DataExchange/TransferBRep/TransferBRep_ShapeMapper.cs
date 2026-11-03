// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeMapper.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeMapper_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// A Finder for a Shape: two mappers are the same if their shapes are the same (TopTools_ShapeMapHasher: the same
/// TShape at the same location, whatever the orientation).
/// </summary>
public class TransferBRep_ShapeMapper : Transfer_Finder
{
    /// <summary>
    /// Creates a Mapper with a Value. This Value can then not be changed. It is used by the Hasher to compute the
    /// HashCode, which will then be stored for an immediate reading.
    /// </summary>
    public TransferBRep_ShapeMapper(in Shape akey)
    {
        theval = akey;
        SetHashCode(ShapeHasher.Instance.GetHashCode(akey));
    }

    /// <summary>Returns the contained value.</summary>
    public Shape Value()
    {
        return theval;
    }

    /// <summary>
    /// Specific test of equality : defined as False if <paramref name="other"/> has not the same true Type, else
    /// contents are compared (TopTools_ShapeMapHasher).
    /// </summary>
    public override bool Equates(Transfer_Finder? other)
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
        TransferBRep_ShapeMapper another = (TransferBRep_ShapeMapper)other;
        return ShapeHasher.Instance.Equals(theval, another.Value());
    }

    /// <summary>Returns the Type of the Value.</summary>
    public override Type? ValueType()
    {
        return TransferBRep_ShapeInfo.Type(theval);
    }

    /// <summary>Returns the name of the Type of the Value.</summary>
    public override string ValueTypeName()
    {
        return TransferBRep_ShapeInfo.TypeName(theval);
    }

    private readonly Shape theval;
}
