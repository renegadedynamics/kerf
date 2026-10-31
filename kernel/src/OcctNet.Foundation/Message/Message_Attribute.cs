// Port of: src/FoundationClasses/TKernel/Message/Message_Attribute.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Attribute.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The data of an extended alert (<see cref="Message_AlertExtended"/>): its name, which keys its message.</summary>
/// <remarks><c>DumpJson</c> is left out (translation rules, section 6).</remarks>
public class Message_Attribute : Standard_Transient
{
    private string myName; //!< alert name, if defined is used in GetMessageKey

    /// <summary>An attribute with the name (empty by default).</summary>
    public Message_Attribute(string theName = "")
    {
        myName = theName;
    }

    /// <summary>The key of the alert's message: the name, or an empty string.</summary>
    public virtual string GetMessageKey() => myName.Length != 0 ? myName : "";

    /// <summary>The custom name of the alert, if set.</summary>
    public string GetName() => myName;

    /// <summary>Sets the custom name of the alert.</summary>
    public void SetName(string theName) => myName = theName;
}
