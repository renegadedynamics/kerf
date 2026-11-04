// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_Element.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_Element.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Tools;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Monitoring;

/// <summary>
/// a Element allows to map any kind of object as a Key for a Map. This works by defining, for a Hash Code,
/// that of the real Key, not of the Element which acts only as an intermediate. When a Map asks for the
/// HashCode of a Element, this one returns the code it has determined at creation time
/// </summary>
internal abstract class Element : Transient
{

    /// <summary>
    /// Empty constructor
    /// </summary>
    public Element()
    {

    }

    /// <summary>
    /// Returns the HashCode which has been stored by SetHashCode (remark that HashCode could be deferred then be
    /// defined by sub-classes, the result is the same)
    /// </summary>
    public override int GetHashCode()
    {
        return _thecode;
    }

    /// <summary>
    /// Specific testof equality : to be defined by each sub-class, must be False if Elements have not the same
    /// true Type, else their contents must be compared
    /// </summary>
    public abstract bool Equates(Element? other);

    /// <summary>
    /// Returns the Type of the Value. By default, returns the DynamicType of &lt;me&gt;, but can be redefined
    /// </summary>
    public virtual Type? ValueType()
    {
        return DynamicType();
    }

    /// <summary>
    /// Returns the name of the Type of the Value. Default is name of ValueType, unless it is for a non-handled
    /// object
    /// </summary>
    public virtual string ValueTypeName()
    {
        return "(finder)";
    }

    /// <summary>
    /// Returns (readonly) the Attribute List
    /// </summary>
    public AttributeList ListAttr()
    {
        return _theattrib;
    }

    /// <summary>
    /// Returns (modifiable) the Attribute List
    /// </summary>
    public AttributeList ChangeAttr()
    {
        return _theattrib;
    }

    /// <summary>
    /// Stores the HashCode which corresponds to the Value given to create the Mapper
    /// </summary>
    protected internal void SetHashCode(int code)
    {
        _thecode = code;
    }

    private int _thecode;
    private readonly AttributeList _theattrib = new();
}
