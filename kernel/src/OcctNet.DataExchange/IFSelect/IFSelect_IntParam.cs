// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_IntParam.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_IntParam.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class simply allows to access an Integer value through a Handle, as a String can be (by using
/// HString). Hence, this value can be accessed : read and modified, without passing through the specific
/// object which detains it. Thus, parameters of a Selection or a Dispatch (according its type) can be
/// controlled directly from the ShareOut which contains them Additionally, an IntParam can be bound to a
/// Static. Remember that for a String, binding is immediate, because the string value of a Static is a
/// HAsciiString, it then suffices to get its Handle. For an Integer, an IntParam can designate (by its name)
/// a Static : each time its value is required or set, the Static is acknowledged
/// </summary>
public class IFSelect_IntParam : Standard_Transient
{

    /// <summary>
    /// Creates an IntParam. Initial value is set to zer
    /// </summary>
    public IFSelect_IntParam()
    {
        theval = 0;
    }

    /// <summary>
    /// Commands this IntParam to be bound to a Static. Hence, Value will return the value if this Static if it
    /// is set, else Value works on the locally stored value. SetValue also will set the value of the Static.
    /// This works only for a present static of type integer or enum. Else, it is just ignored.
    /// </summary>
    /// <remarks>
    /// OCCT.NET: the static is named in the parameter set <paramref name="statics"/> (OCCT: the process-wide
    /// statics; decision 1181); without a set, the IntParam works on its local value only.
    /// </remarks>
    public void SetStaticName(string statname, Interface_StaticSet? statics)
    {
        thestn = statname ?? "";
        thestatics = statics;
    }

    /// <summary>
    /// Returns the name of static parameter to which this IntParam is bound, empty if none
    /// </summary>
    /// <remarks>OCCT declares it but never defines it.</remarks>
    public string StaticName()
    {
        return thestn;
    }

    /// <summary>
    /// Reads Integer Value of the IntParam. If a StaticName is defined and the Static is set, looks in priority
    /// the value of the static
    /// </summary>
    public int Value()
    {
        if (thestn.Length == 0 || thestatics is null)
        {
            return theval;
        }
        if (!thestatics.IsSet(thestn))
        {
            return theval;
        }
        return thestatics.IVal(thestn);
    }

    /// <summary>
    /// Sets a new Integer Value for the IntParam. If a StaticName is defined and the Static is set, also sets
    /// the value of the static
    /// </summary>
    public void SetValue(int val)
    {
        theval = val;
        if (thestn.Length == 0 || thestatics is null)
        {
            return;
        }
        if (!thestatics.IsPresent(thestn))
        {
            return;
        }
        thestatics.SetIVal(thestn, theval);
    }

    private int theval;
    private string thestn = "";
    private Interface_StaticSet? thestatics;
}
