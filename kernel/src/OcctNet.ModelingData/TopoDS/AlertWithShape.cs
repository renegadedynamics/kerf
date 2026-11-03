// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_AlertWithShape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_AlertWithShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology;

/// <summary>An alert holding a shape: the shape the situation concerns. Such alerts never merge.</summary>
/// <remarks>
/// <para>
/// OCCT's <c>DEFINE_ALERT_WITH_SHAPE(Alert)</c> macro is a plain subclass in the port:
/// <c>public sealed class Alert(TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape);</c>.
/// </para>
/// <para>
/// An algorithm puts its own shapes in its alerts, which may be shapes it is still building. <see cref="GetShape"/>
/// is a publication point (decision 625, design 4.2): it freezes the shape's graph before handing it out, as a
/// maker's <c>Shape()</c> does. Ported code that reads an alert's shape and goes on working on it uses
/// <see cref="InternalGetShape"/>, which doesn't freeze. The setter is internal: alerts are made by the algorithms.
/// </para>
/// </remarks>
public class AlertWithShape : Alert
{
    private Shape _shape;

    /// <summary>An alert about the shape.</summary>
    public AlertWithShape(in Shape shape)
    {
        _shape = shape;
    }

    /// <summary>The shape, its graph frozen.</summary>
    public Shape GetShape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>The shape as the algorithm put it, not frozen (for ported code).</summary>
    internal Shape InternalGetShape() => _shape;

    /// <summary>Sets the shape.</summary>
    internal void SetShape(in Shape shape) => _shape = shape;

    /// <summary>False: alerts with shapes never merge.</summary>
    public override bool SupportsMerge() => false;

    /// <summary>False: alerts with shapes never merge.</summary>
    public override bool Merge(Alert target)
    {
        return false;
        //  occ::handle<TopoDS_AlertWithShape> aTarget = Handle(TopoDS_AlertWithShape)::DownCast (theTarget);
        //  return aTarget->GetShape() == myShape;
    }
}
