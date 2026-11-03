// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_TransientElem.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_TransientElem.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// an TransientElem defines an Element for a specific input class its definition includes the value of the
/// Key to be mapped, and the HashCoder associated to the class of the Key Transient from Standard defines
/// the class to be keyed MapTransientHasher from TColStd is the associated Hasher DataInfo from MoniTool is
/// an additional class which helps to provide information on the value (template : see DataInfo)
/// </summary>
public class MoniTool_TransientElem : MoniTool_Element
{

    /// <summary>
    /// Creates a TransientElem with a Value. This Value can then not be changed. It is used by the Hasher to
    /// compute the HashCode, which will then be stored for an immediate reading.
    /// </summary>
    public MoniTool_TransientElem(Transient? akey)
    {
        theval = akey;
        // OCCT.NET: std::hash of the handle (its address) is the object's identity hash
        SetHashCode(System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(akey));
    }

    /// <summary>
    /// Returns the contained value
    /// </summary>
    public Transient? Value()
    {
        return theval;
    }

    /// <summary>
    /// Specific testof equality : defined as False if &lt;other&gt; has not the same true Type, else contents
    /// are compared (by C++ operator ==)
    /// </summary>
    public override bool Equates(MoniTool_Element? other)
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
        MoniTool_TransientElem another = (other as MoniTool_TransientElem);
        //  return (theval == another.Value());
        return theval == another.Value();
    }

    /// <summary>
    /// Returns the Type of the Value. By default, returns the DynamicType of &lt;me&gt;, but can be redefined
    /// </summary>
    public override Type? ValueType()
    {
        return MoniTool_DataInfo.Type(theval);
    }

    /// <summary>
    /// Returns the name of the Type of the Value. Default is name of ValueType, unless it is for a non-handled
    /// object
    /// </summary>
    public override string ValueTypeName()
    {
        return MoniTool_DataInfo.TypeName(theval);
    }

    private Transient? theval;
}
