// Port of: src/FoundationClasses/TKernel/Message/Message_PrinterToReport.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_PrinterToReport.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace Occt;

/// <summary>
/// A printer that turns the messages sent to a messenger into alerts of a report (its own, or the default report).
/// A string stream is held back as the name of the next alert, and becomes an alert itself when another arrives.
/// </summary>
/// <remarks>
/// OCCT's <c>SendStringStream</c> makes an alert with a <see cref="Message_AttributeStream"/> when the stream holds
/// DumpJson output (<c>Standard_Dump::HasChildKey</c>). DumpJson is left out of the port (translation rules,
/// section 6), so no stream holds its keys: that branch is kept but never taken. As OCCT's printer has no default
/// report, <see cref="Report"/> makes the process-wide one when the printer has none.
/// </remarks>
public class Message_PrinterToReport : Message_Printer
{
    private string myName = "";
    private Message_Report? myReport; //!< the report for sending alerts

    /// <summary>A printer to the default report, until <see cref="SetReport"/>.</summary>
    public Message_PrinterToReport()
    {
    }

    /// <summary>The printer's report, or the default report (made if needed).</summary>
    public Message_Report Report()
    {
        if (myReport is not null)
        {
            return myReport;
        }
        return Message.DefaultReport(true)!;
    }

    /// <summary>Sets the printer's report; null means the default report.</summary>
    public void SetReport(Message_Report? theReport) => myReport = theReport;

    /// <summary>Sends the stream's text: held back as the next alert's name (see the remarks).</summary>
    public override void SendStringStream(StringBuilder theStream, Message_Gravity theGravity)
    {
        Message_Report aReport = Report();
        if (!aReport.ActiveMetrics().IsEmpty())
        {
            sendMetricAlert(theStream.ToString(), theGravity);
            return;
        }
        if (HasChildKey(theStream.ToString()))
        {
            Message_AlertExtended.AddAlert(aReport, new Message_AttributeStream(theStream.ToString(), myName), theGravity);
            myName = "";
        }
        else
        {
            if (myName.Length != 0)
            {
                string aName = myName;
                myName = "";
                send(aName, theGravity);
            }
            myName = theStream.ToString();
        }
    }

    /// <summary>Sends an object as an alert with a <see cref="Message_AttributeObject"/>.</summary>
    public override void SendObject(Standard_Transient? theObject, Message_Gravity theGravity)
    {
        Message_Report aReport = Report();
        if (!aReport.ActiveMetrics().IsEmpty())
        {
            sendMetricAlert(myName, theGravity);
            return;
        }

        Message_AlertExtended.AddAlert(aReport, new Message_AttributeObject(theObject, myName), theGravity);
    }

    /// <summary>Sends a string as an alert with a <see cref="Message_Attribute"/> named by it.</summary>
    protected override void send(string theString, Message_Gravity theGravity)
    {
        if (myName.Length != 0)
        {
            string aName = myName;
            myName = "";
            send(aName, theGravity);
        }

        Message_Report aReport = Report();
        if (!aReport.ActiveMetrics().IsEmpty())
        {
            sendMetricAlert(theString, theGravity);
            return;
        }
        Message_AlertExtended.AddAlert(aReport, new Message_Attribute(theString), theGravity);
    }

    /// <summary>Sends an alert with the metrics active in the report.</summary>
    protected void sendMetricAlert(string theValue, Message_Gravity theGravity)
    {
        Message_AlertExtended.AddAlert(Report(), new Message_AttributeMeter(theValue), theGravity);
    }

    // Standard_Dump::HasChildKey: true for the text of a DumpJson, which the port doesn't write.
    private static bool HasChildKey(string theSourceValue) => false;
}
