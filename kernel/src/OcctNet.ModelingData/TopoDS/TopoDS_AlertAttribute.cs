// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_AlertAttribute.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_AlertAttribute.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;

namespace Occt;

/// <summary>An alert attribute holding a shape, with a text describing it as its stream.</summary>
/// <remarks>
/// OCCT's stream is the shape's DumpJson, which the port leaves out (translation rules, section 6): the stream is the
/// shape's <see cref="TopoDS_Shape.ToString"/> ("Solid Forward", ...). OCCT's <c>operator&lt;&lt;</c> of a messenger and
/// a shape is <see cref="Send"/>. <see cref="GetShape"/> freezes, as <see cref="TopoDS_AlertWithShape.GetShape"/>.
/// </remarks>
public class TopoDS_AlertAttribute : Message_AttributeStream
{
    private readonly TopoDS_Shape myShape;

    /// <summary>An attribute holding the shape, with the name.</summary>
    public TopoDS_AlertAttribute(in TopoDS_Shape theShape, string theName = "")
        : base("", theName)
    {
        myShape = theShape;
        SetStream(theShape.ToString());
    }

    /// <summary>The shape, its graph frozen.</summary>
    public TopoDS_Shape GetShape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>
    /// Adds an alert with the shape (Info gravity) to the report of every <see cref="Message_PrinterToReport"/> of the
    /// messenger.
    /// </summary>
    public static void Send(Message_Messenger theMessenger, in TopoDS_Shape theShape)
    {
        foreach (Message_Printer aPrinter in theMessenger.Printers())
        {
            if (aPrinter is not Message_PrinterToReport aPrinterToReport)
            {
                continue;
            }

            Message_Report aReport = aPrinterToReport.Report();
            Message_AlertExtended.AddAlert(aReport, new TopoDS_AlertAttribute(theShape), Message_Info);
        }
    }
}
