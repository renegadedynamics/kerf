// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeObject.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeObject.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Foundation;

/// <summary>An alert attribute holding an object.</summary>
internal class AttributeObject : MessageAttribute
{
    private Transient? _object; //!< alert object

    /// <summary>An attribute holding the object, with the name.</summary>
    public AttributeObject(Transient? theObject, string name = "")
        : base(name)
    {
        _object = theObject;
    }

    /// <summary>The object.</summary>
    public Transient? Object() => _object;

    /// <summary>Sets the object.</summary>
    public void SetObject(Transient? theObject) => _object = theObject;
}
