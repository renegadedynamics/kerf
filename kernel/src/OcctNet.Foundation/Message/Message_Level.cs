// Port of: src/FoundationClasses/TKernel/Message/Message_Level.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Level.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A level of alerts in the process-wide default report (<see cref="Message.DefaultReport"/>): while it is open, the
/// alerts added to that report go below the level's root alert. Opening it adds the level to the report if the report
/// is active in the default messenger; disposing it removes it (OCCT's destructor):
/// <c>using var aLevel = new Message_Level("name");</c> is OCCT's <c>OCCT_ADD_MESSAGE_LEVEL_SENTRY</c>.
/// </summary>
/// <remarks>
/// If the root alert has a <see cref="Message_AttributeMeter"/> attribute (when the report has active metrics), the
/// level records the metrics of each alert added on it: started when the alert is added, stopped when the next one is
/// added or the level closes.
/// </remarks>
public sealed class Message_Level : IDisposable
{
    private Message_AlertExtended? myRootAlert; //!< root alert
    private Message_AlertExtended? myLastAlert; //!< last added alert on the root alert
    private bool myIsDisposed;

    /// <summary>Opens a level named <paramref name="theName"/> in the default report, if that report is active.</summary>
    public Message_Level(string theName = "")
    {
        Message_Report? aDefaultReport = Message.DefaultReport();
        if (aDefaultReport is not null && aDefaultReport.IsActiveInMessenger())
        {
            aDefaultReport.AddLevel(this, theName);
        }
    }

    /// <summary>Closes the level (OCCT's destructor).</summary>
    public void Dispose()
    {
        if (!myIsDisposed)
        {
            myIsDisposed = true;
            remove();
        }
    }

    /// <summary>The root alert of the level, or null.</summary>
    public Message_AlertExtended? RootAlert() => myRootAlert;

    /// <summary>Sets the root alert, and starts its metrics if <paramref name="isRequiredToStart"/>.</summary>
    public void SetRootAlert(Message_AlertExtended? theAlert, bool isRequiredToStart)
    {
        myRootAlert = theAlert;
        if (isRequiredToStart)
        {
            Message_AttributeMeter.StartAlert(myRootAlert);
        }
    }

    /// <summary>
    /// Adds an alert below the root alert: stops the metrics of the last alert and starts the new one's. Only extended
    /// alerts are added; true if added.
    /// </summary>
    public bool AddAlert(Message_Gravity theGravity, Message_Alert theAlert)
    {
        if (theAlert is not Message_AlertExtended anAlertExtended)
        {
            return false;
        }

        // looking for the parent of the parameter alert to release the previous alert
        Message_AlertExtended aRootAlert = myRootAlert!;
        Message_CompositeAlerts aCompositeAlert = aRootAlert.CompositeAlerts(true)!;

        // update metrics of the previous alert
        Message_AttributeMeter.StopAlert(myLastAlert);

        myLastAlert = anAlertExtended;
        // set start metrics of the new alert
        Message_AttributeMeter.StartAlert(myLastAlert);

        // add child alert
        aCompositeAlert.AddAlert(theGravity, theAlert);

        return true;
    }

    // Removes the level from the report, stopping the metrics of the last and the root alerts.
    private void remove()
    {
        Message_Report? aDefaultReport = Message.DefaultReport();
        if (aDefaultReport is null || !aDefaultReport.IsActiveInMessenger())
        {
            return;
        }

        Message_AttributeMeter.StopAlert(myLastAlert);

        Message.DefaultReport()?.RemoveLevel(this);
    }
}
