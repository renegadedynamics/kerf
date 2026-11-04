// Port of: src/FoundationClasses/TKernel/Message/Message_PrinterToReport.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_PrinterToReport.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Text;

namespace Kerf.Engine.Foundation;

/// <summary>
/// A printer that turns the messages sent to a messenger into alerts of a report (its own, or the default report).
/// A string stream is held back as the name of the next alert, and becomes an alert itself when another arrives.
/// </summary>
/// <remarks>
/// OCCT's <c>SendStringStream</c> makes an alert with a <see cref="AttributeStream"/> when the stream holds
/// DumpJson output (<c>Standard_Dump::HasChildKey</c>). DumpJson is left out of the port (translation rules,
/// section 6), so no stream holds its keys: that branch is kept but never taken. As OCCT's printer has no default
/// report, <see cref="Report"/> makes the process-wide one when the printer has none.
/// </remarks>
internal class PrinterToReport : Printer
{
    private string _name = "";
    private Report? _report; //!< the report for sending alerts

    /// <summary>A printer to the default report, until <see cref="SetReport"/>.</summary>
    public PrinterToReport()
    {
    }

    /// <summary>The printer's report, or the default report (made if needed).</summary>
    public Report Report()
    {
        if (_report is not null)
        {
            return _report;
        }
        return global::Kerf.Engine.Foundation.Message.DefaultReport(true)!;
    }

    /// <summary>Sets the printer's report; null means the default report.</summary>
    public void SetReport(Report? report) => _report = report;

    /// <summary>Sends the stream's text: held back as the next alert's name (see the remarks).</summary>
    public override void SendStringStream(StringBuilder stream, Gravity gravity)
    {
        Report report = Report();
        if (report.ActiveMetrics().Count != 0)
        {
            SendMetricAlert(stream.ToString(), gravity);
            return;
        }
        if (HasChildKey(stream.ToString()))
        {
            AlertExtended.AddAlert(report, new AttributeStream(stream.ToString(), _name), gravity);
            _name = "";
        }
        else
        {
            if (_name.Length != 0)
            {
                string name = _name;
                _name = "";
                SendCore(name, gravity);
            }
            _name = stream.ToString();
        }
    }

    /// <summary>Sends an object as an alert with a <see cref="AttributeObject"/>.</summary>
    public override void SendObject(Transient? theObject, Gravity gravity)
    {
        Report report = Report();
        if (report.ActiveMetrics().Count != 0)
        {
            SendMetricAlert(_name, gravity);
            return;
        }

        AlertExtended.AddAlert(report, new AttributeObject(theObject, _name), gravity);
    }

    /// <summary>Sends a string as an alert with a <see cref="MessageAttribute"/> named by it.</summary>
    protected override void SendCore(string theString, Gravity gravity)
    {
        if (_name.Length != 0)
        {
            string name = _name;
            _name = "";
            SendCore(name, gravity);
        }

        Report report = Report();
        if (report.ActiveMetrics().Count != 0)
        {
            SendMetricAlert(theString, gravity);
            return;
        }
        AlertExtended.AddAlert(report, new MessageAttribute(theString), gravity);
    }

    /// <summary>Sends an alert with the metrics active in the report.</summary>
    protected void SendMetricAlert(string value, Gravity gravity)
    {
        AlertExtended.AddAlert(Report(), new AttributeMeter(value), gravity);
    }

    // Standard_Dump::HasChildKey: true for the text of a DumpJson, which the port doesn't write.
    private static bool HasChildKey(string sourceValue) => false;
}
