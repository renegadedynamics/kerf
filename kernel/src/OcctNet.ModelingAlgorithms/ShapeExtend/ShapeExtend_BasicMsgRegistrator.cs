// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_BasicMsgRegistrator.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_BasicMsgRegistrator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Attaches messages to objects (shapes, for one): Shape Healing's algorithms send it a message describing each case
/// they meet (removing a small edge from a wire, ...). The methods do nothing; derived classes (Data Exchange's
/// processors, <see cref="ShapeExtend_MsgRegistrator"/>) attach the messages.
/// </summary>
/// <remarks>A working object (decision 21). Nothing here writes to the console (decision 493).</remarks>
public class ShapeExtend_BasicMsgRegistrator : Standard_Transient
{
    /// <summary>An empty registrator.</summary>
    public ShapeExtend_BasicMsgRegistrator()
    {
    }

    /// <summary>Sends a message to be attached to the object; an object of any type a derived class interprets.</summary>
    public virtual void Send(Standard_Transient? @object, Message_Msg message, Message_Gravity gravity)
    {
    }

    /// <summary>Sends a message to be attached to the shape.</summary>
    public virtual void Send(in TopoDS_Shape shape, Message_Msg message, Message_Gravity gravity)
    {
    }

    /// <summary>Sends a message attached to no object (a null transient).</summary>
    public virtual void Send(Message_Msg message, Message_Gravity gravity)
    {
        Standard_Transient? dummy = null;
        Send(dummy, message, gravity);
    }
}
