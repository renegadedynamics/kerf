// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_AlertAttribute.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_AlertAttribute.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Topology;

/// <summary>An alert attribute holding a shape, with a text describing it as its stream.</summary>
/// <remarks>
/// OCCT's stream is the shape's DumpJson, which the port leaves out (translation rules, section 6): the stream is the
/// shape's <see cref="Shape.ToString"/> ("Solid Forward", ...). OCCT's <c>operator&lt;&lt;</c> of a messenger and
/// a shape is <see cref="Send"/>. <see cref="GetShape"/> freezes, as <see cref="AlertWithShape.GetShape"/>.
/// </remarks>
internal class AlertAttribute : AttributeStream
{
    private readonly Shape _shape;

    /// <summary>An attribute holding the shape, with the name.</summary>
    public AlertAttribute(in Shape shape, string name = "")
        : base("", name)
    {
        _shape = shape;
        SetStream(shape.ToString());
    }

    /// <summary>The shape, its graph frozen.</summary>
    public Shape GetShape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>
    /// Adds an alert with the shape (Info gravity) to the report of every <see cref="PrinterToReport"/> of the
    /// messenger.
    /// </summary>
    public static void Send(Messenger messenger, in Shape shape)
    {
        foreach (Printer printer in messenger.Printers())
        {
            if (printer is not PrinterToReport printerToReport)
            {
                continue;
            }

            Report report = printerToReport.Report();
            AlertExtended.AddAlert(report, new AlertAttribute(shape), Info);
        }
    }
}
