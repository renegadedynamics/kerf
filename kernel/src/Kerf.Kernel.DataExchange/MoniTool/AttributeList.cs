// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_AttrList.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_AttrList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Monitoring;
using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Framework.Monitoring.MonitorValueType;

namespace Kerf.Engine.Exchange.Tools;

/// <summary>
/// a AttrList allows to record a list of attributes as Transients which can be edited, changed ... Each one
/// is identified by a name
/// </summary>
public class AttributeList
{

    /// <summary>
    /// Creates an AttrList, empty
    /// </summary>
    public AttributeList()
    {
    }

    /// <summary>
    /// Creates an AttrList from another one, definitions are shared (calls SameAttributes)
    /// </summary>
    public AttributeList(AttributeList other)
    {
        _theattrib = new Dictionary<string, Transient>(other.AttrList());

    }

    /// <summary>
    /// Adds an attribute with a given name (replaces the former one with the same name if already exists)
    /// </summary>
    public void SetAttribute(string name, Transient? val)
    {
        _theattrib[name] = val;
    }

    /// <summary>
    /// Removes an attribute Returns True when done, False if this attribute did not exist
    /// </summary>
    public bool RemoveAttribute(string name)
    {
        if (_theattrib.Count == 0)
        {
            return false;
        }
        return _theattrib.Remove(name);
    }

    /// <summary>
    /// Returns an attribute from its name, filtered by a type If no attribute has this name, or if it is not
    /// kind of this type, &lt;val&gt; is Null and returned value is False Else, it is True
    /// </summary>
    public bool GetAttribute(string name, Type? type, out Transient? val)
    {
        if (_theattrib.Count == 0)
        {
            val = null;
            return false;
        }
        if (!_theattrib.TryGetValue(name, out val))
        {
            val = null;
            return false;
        }
        if (!val.IsKind(type))
        {
            val = null;
            return false;
        }
        return true;
    }

    /// <summary>
    /// Returns an attribute from its name. Null Handle if not recorded (whatever Transient, Integer, Real ...)
    /// Integer is recorded as IntVal Real is recorded as RealVal Text is recorded as HAsciiString
    /// </summary>
    public Transient? Attribute(string name)
    {
        Transient atr = null;
        if (_theattrib.Count == 0)
        {
            return atr;
        }
        if (!_theattrib.TryGetValue(name, out atr))
        {
            atr = null;
        }
        return atr;
    }

    /// <summary>
    /// Returns the type of an attribute: ValueInt, ValueReal, ValueText (String), ValueIdent (any) or ValueVoid
    /// (not recorded)
    /// </summary>
    public MonitorValueType AttributeType(string name)
    {
        Transient atr = Attribute(name);
        if (atr is null)
        {
            return MonitorValueType.ValueVoid;
        }
        if (atr.DynamicType() == typeof(MonitorIntVal))
        {
            return MonitorValueType.ValueInteger;
        }
        if (atr.DynamicType() == typeof(RealVal))
        {
            return MonitorValueType.ValueReal;
        }
        if (atr.DynamicType() == typeof(TCollection_HAsciiString))
        {
            return MonitorValueType.ValueText;
        }
        return MonitorValueType.ValueIdent;
    }

    /// <summary>
    /// Adds an integer value for an attribute
    /// </summary>
    public void SetIntegerAttribute(string name, int val)
    {
        MonitorIntVal ival = new MonitorIntVal();
        ival.CValue() = val;
        SetAttribute(name, ival);
    }

    /// <summary>
    /// Returns an attribute from its name, as integer If no attribute has this name, or not an integer,
    /// &lt;val&gt; is 0 and returned value is False Else, it is True
    /// </summary>
    public bool GetIntegerAttribute(string name, out int val)
    {
        MonitorIntVal ival = (Attribute(name) as MonitorIntVal);
        if (ival is null)
        {
            val = 0;
            return false;
        }
        val = ival.Value();
        return true;
    }

    /// <summary>
    /// Returns an integer attribute from its name. 0 if not recorded
    /// </summary>
    public int IntegerAttribute(string name)
    {
        MonitorIntVal ival = (Attribute(name) as MonitorIntVal);
        if (ival is null)
        {
            return 0;
        }
        return ival.Value();
    }

    /// <summary>
    /// Adds a real value for an attribute
    /// </summary>
    public void SetRealAttribute(string name, double val)
    {
        RealVal rval = new RealVal();
        rval.CValue() = val;
        SetAttribute(name, rval);
    }

    /// <summary>
    /// Returns an attribute from its name, as real If no attribute has this name, or not a real &lt;val&gt; is
    /// 0.0 and returned value is False Else, it is True
    /// </summary>
    public bool GetRealAttribute(string name, out double val)
    {
        RealVal rval = (Attribute(name) as RealVal);
        if (rval is null)
        {
            val = 0.0;
            return false;
        }
        val = rval.Value();
        return true;
    }

    /// <summary>
    /// Returns a real attribute from its name. 0.0 if not recorded
    /// </summary>
    public double RealAttribute(string name)
    {
        RealVal rval = (Attribute(name) as RealVal);
        if (rval is null)
        {
            return 0;
        }
        return rval.Value();
    }

    /// <summary>
    /// Adds a String value for an attribute
    /// </summary>
    public void SetStringAttribute(string name, string val)
    {
        TCollection_HAsciiString hval = new TCollection_HAsciiString(val);
        SetAttribute(name, hval);
    }

    /// <summary>
    /// Returns an attribute from its name, as String If no attribute has this name, or not a String &lt;val&gt;
    /// is 0.0 and returned value is False Else, it is True
    /// </summary>
    public bool GetStringAttribute(string name, out string val)
    {
        TCollection_HAsciiString hval =
            (Attribute(name) as TCollection_HAsciiString);
        if (hval is null)
        {
            val = "";
            return false;
        }
        val = hval.ToCString();
        return true;
    }

    /// <summary>
    /// Returns a String attribute from its name. "" if not recorded
    /// </summary>
    public string StringAttribute(string name)
    {
        TCollection_HAsciiString hval =
            (Attribute(name) as TCollection_HAsciiString);
        if (hval is null)
        {
            return "";
        }
        return hval.ToCString();
    }

    /// <summary>
    /// Returns the exhaustive list of attributes
    /// </summary>
    public Dictionary<string, Transient> AttrList()
    {
        return _theattrib;
    }

    /// <summary>
    /// Gets the list of attributes from &lt;other&gt;, as such, i.e. not copied : attributes are shared, any
    /// attribute edited, added, or removed in &lt;other&gt; is also in &lt;me&gt; and vice versa The former list
    /// of attributes of &lt;me&gt; is dropped
    /// </summary>
    public void SameAttributes(AttributeList other)
    {
        _theattrib = new Dictionary<string, Transient>(other.AttrList());
    }

    /// <summary>
    /// Gets the list of attributes from &lt;other&gt;, by copying it By default, considers all the attributes
    /// from &lt;other&gt; If &lt;fromname&gt; is given, considers only the attributes with name beginning by
    /// &lt;fromname&gt; For each attribute, if &lt;copied&gt; is True (D), its value is also copied if it is a
    /// basic type (Integer,Real,String), else it remains shared between &lt;other&gt; and &lt;me&gt; These new
    /// attributes are added to the existing ones in &lt;me&gt;, in case of same name, they replace the existing
    /// ones
    /// </summary>
    public void GetAttributes(AttributeList other, string fromname = "", bool copied = true)
    {
        Dictionary<string, Transient> list =
            other.AttrList();
        if (list.Count == 0)
        {
            return;
        }

        foreach (KeyValuePair<string, Transient> iter in list)
        {
            string name = iter.Key;
            if (!name.StartsWith(fromname))
            {
                continue;
            }
            Transient atr = iter.Value;
            Transient newatr = atr;

            //    Copy ? according type
            if (copied)
            {
                MonitorIntVal ival = (atr as MonitorIntVal);
                if (ival is not null)
                {
                    int intval = ival.Value();
                    ival = new MonitorIntVal();
                    ival.CValue() = intval;
                    newatr = ival;
                }
                RealVal rval = (atr as RealVal);
                if (rval is not null)
                {
                    double realval = rval.Value();
                    rval = new RealVal();
                    rval.CValue() = realval;
                    newatr = rval;
                }
                TCollection_HAsciiString hval = (atr as TCollection_HAsciiString);
                if (hval is not null)
                {
                    TCollection_HAsciiString strval =
                        new TCollection_HAsciiString(hval);
                    newatr = strval;
                }
            }
            _theattrib[name] = newatr;
        }
    }

    private Dictionary<string, Transient> _theattrib = new();
}
