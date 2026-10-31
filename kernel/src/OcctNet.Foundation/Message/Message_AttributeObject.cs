// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeObject.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeObject.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An alert attribute holding an object.</summary>
public class Message_AttributeObject : Message_Attribute
{
    private Standard_Transient? myObject; //!< alert object

    /// <summary>An attribute holding the object, with the name.</summary>
    public Message_AttributeObject(Standard_Transient? theObject, string theName = "")
        : base(theName)
    {
        myObject = theObject;
    }

    /// <summary>The object.</summary>
    public Standard_Transient? Object() => myObject;

    /// <summary>Sets the object.</summary>
    public void SetObject(Standard_Transient? theObject) => myObject = theObject;
}
