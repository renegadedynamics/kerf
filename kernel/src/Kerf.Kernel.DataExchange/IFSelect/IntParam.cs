// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_IntParam.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_IntParam.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// This class simply allows to access an Integer value through a Handle, as a String can be (by using
/// HString). Hence, this value can be accessed : read and modified, without passing through the specific
/// object which detains it. Thus, parameters of a Selection or a Dispatch (according its type) can be
/// controlled directly from the ShareOut which contains them Additionally, an IntParam can be bound to a
/// Static. Remember that for a String, binding is immediate, because the string value of a Static is a
/// HAsciiString, it then suffices to get its Handle. For an Integer, an IntParam can designate (by its name)
/// a Static : each time its value is required or set, the Static is acknowledged
/// </summary>
internal class IntParam : Transient
{

    /// <summary>
    /// Creates an IntParam. Initial value is set to zer
    /// </summary>
    public IntParam()
    {
        _theval = 0;
    }

    /// <summary>
    /// Commands this IntParam to be bound to a Static. Hence, Value will return the value if this Static if it
    /// is set, else Value works on the locally stored value. SetValue also will set the value of the Static.
    /// This works only for a present static of type integer or enum. Else, it is just ignored.
    /// </summary>
    /// <remarks>
    /// Kerf: the static is named in the parameter set <paramref name="statics"/> (OCCT: the process-wide
    /// statics; decision 1181); without a set, the IntParam works on its local value only.
    /// </remarks>
    public void SetStaticName(string statname, StaticSet? statics)
    {
        _thestn = statname ?? "";
        _thestatics = statics;
    }

    /// <summary>
    /// Returns the name of static parameter to which this IntParam is bound, empty if none
    /// </summary>
    /// <remarks>OCCT declares it but never defines it.</remarks>
    public string StaticName()
    {
        return _thestn;
    }

    /// <summary>
    /// Reads Integer Value of the IntParam. If a StaticName is defined and the Static is set, looks in priority
    /// the value of the static
    /// </summary>
    public int Value()
    {
        if (_thestn.Length == 0 || _thestatics is null)
        {
            return _theval;
        }
        if (!_thestatics.IsSet(_thestn))
        {
            return _theval;
        }
        return _thestatics.IVal(_thestn);
    }

    /// <summary>
    /// Sets a new Integer Value for the IntParam. If a StaticName is defined and the Static is set, also sets
    /// the value of the static
    /// </summary>
    public void SetValue(int val)
    {
        _theval = val;
        if (_thestn.Length == 0 || _thestatics is null)
        {
            return;
        }
        if (!_thestatics.IsPresent(_thestn))
        {
            return;
        }
        _thestatics.SetIVal(_thestn, _theval);
    }

    private int _theval;
    private string _thestn = "";
    private StaticSet? _thestatics;
}
