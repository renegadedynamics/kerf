// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Modification.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Modification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Healing.Custom;

/// <summary>
/// A modification for <see cref="Modifier"/> that can send messages about the shapes it modifies to a
/// message registrator: the base of ShapeCustom's modifications.
/// </summary>
internal abstract class CustomModification : Modification
{
    private BasicMsgRegistrator? _msgReg;

    /// <summary>Sets the message registrator.</summary>
    public virtual void SetMsgRegistrator(BasicMsgRegistrator? msgreg)
    {
        _msgReg = msgreg;
    }

    /// <summary>The message registrator.</summary>
    public BasicMsgRegistrator? MsgRegistrator()
    {
        return _msgReg;
    }

    /// <summary>Sends a message attached to the shape to the message registrator, if any.</summary>
    public void SendMsg(in Shape shape, Msg message, Gravity gravity = Info)
    {
        if (_msgReg is not null)
        {
            _msgReg.Send(shape, message, gravity);
        }
    }
}
