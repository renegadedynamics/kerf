// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_MsgRegistrator.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_MsgRegistrator.cxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_MsgRegistrator.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Kerf.Engine.Healing.Extend;

/// <summary>
/// Attaches messages to objects (any transient, or shapes): the healing algorithms send it the messages of what they
/// did, and it keeps them in two maps, for instance for Data Exchange to attach them to the file's entities.
/// </summary>
/// <remarks>
/// A working object (decision 21). The maps are its own: <see cref="MapTransient"/> and <see cref="MapShape"/> return
/// them as OCCT's const references do, for reading. Transients are keys by reference (OCCT's handle maps); shapes by
/// <see cref="ShapeHasher"/>, as OCCT's.
/// </remarks>
internal class MsgRegistrator : BasicMsgRegistrator
{
    private readonly Dictionary<Transient, NCollection_List<Msg>> _mapTransient =
        new(ReferenceEqualityComparer.Instance);

    private readonly Dictionary<Shape, NCollection_List<Msg>> _mapShape =
        new(ShapeHasher.Instance);

    /// <summary>An empty registrator.</summary>
    public MsgRegistrator()
    {
    }

    /// <summary>
    /// Attaches the message to the object: appended to its list if it is in the map, else the object is added with
    /// it. A null object is ignored.
    /// </summary>
    public override void Send(Transient? @object, Msg message, Gravity gravity)
    {
        if (@object is null)
        {
            return;
        }
        if (_mapTransient.ContainsKey(@object))
        {
            NCollection_List<Msg> list = _mapTransient.GetValue(@object);
            list.Append(message);
        }
        else
        {
            var list = new NCollection_List<Msg>();
            list.Append(message);
            _mapTransient[@object] = list;
        }
    }

    /// <summary>
    /// Attaches the message to the shape: appended to its list if it is in the map, else the shape is added with it.
    /// A null shape is ignored.
    /// </summary>
    public override void Send(in Shape shape, Msg message, Gravity gravity)
    {
        if (shape.IsNull)
        {
            return;
        }
        if (_mapShape.ContainsKey(shape))
        {
            NCollection_List<Msg> list = _mapShape.GetValue(shape);
            list.Append(message);
        }
        else
        {
            var list = new NCollection_List<Msg>();
            list.Append(message);
            _mapShape[shape] = list;
        }
    }

    /// <summary>The map of objects to their messages.</summary>
    public Dictionary<Transient, NCollection_List<Msg>> MapTransient() => _mapTransient;

    /// <summary>The map of shapes to their messages.</summary>
    public Dictionary<Shape, NCollection_List<Msg>> MapShape() => _mapShape;
}
