// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Modification.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Modification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;

namespace Occt;

/// <summary>
/// A modification for <see cref="BRepTools_Modifier"/> that can send messages about the shapes it modifies to a
/// message registrator: the base of ShapeCustom's modifications.
/// </summary>
public abstract class ShapeCustom_Modification : BRepTools_Modification
{
    private ShapeExtend_BasicMsgRegistrator? myMsgReg;

    /// <summary>Sets the message registrator.</summary>
    public virtual void SetMsgRegistrator(ShapeExtend_BasicMsgRegistrator? msgreg)
    {
        myMsgReg = msgreg;
    }

    /// <summary>The message registrator.</summary>
    public ShapeExtend_BasicMsgRegistrator? MsgRegistrator()
    {
        return myMsgReg;
    }

    /// <summary>Sends a message attached to the shape to the message registrator, if any.</summary>
    public void SendMsg(in TopoDS_Shape shape, Message_Msg message, Message_Gravity gravity = Message_Info)
    {
        if (myMsgReg is not null)
        {
            myMsgReg.Send(shape, message, gravity);
        }
    }
}
