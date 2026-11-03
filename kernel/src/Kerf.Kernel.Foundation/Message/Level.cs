// Port of: src/FoundationClasses/TKernel/Message/Message_Level.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Level.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>
/// A level of alerts in the process-wide default report (<see cref="Message.DefaultReport"/>): while it is open, the
/// alerts added to that report go below the level's root alert. Opening it adds the level to the report if the report
/// is active in the default messenger; disposing it removes it (OCCT's destructor):
/// <c>using var aLevel = new Message_Level("name");</c> is OCCT's <c>OCCT_ADD_MESSAGE_LEVEL_SENTRY</c>.
/// </summary>
/// <remarks>
/// If the root alert has a <see cref="AttributeMeter"/> attribute (when the report has active metrics), the
/// level records the metrics of each alert added on it: started when the alert is added, stopped when the next one is
/// added or the level closes.
/// </remarks>
public sealed class Level : IDisposable
{
    private AlertExtended? _rootAlert; //!< root alert
    private AlertExtended? _lastAlert; //!< last added alert on the root alert
    private bool _isDisposed;

    /// <summary>Opens a level named <paramref name="name"/> in the default report, if that report is active.</summary>
    public Level(string name = "")
    {
        Report? defaultReport = global::Kerf.Engine.Foundation.Message.DefaultReport();
        if (defaultReport is not null && defaultReport.IsActiveInMessenger())
        {
            defaultReport.AddLevel(this, name);
        }
    }

    /// <summary>Closes the level (OCCT's destructor).</summary>
    public void Dispose()
    {
        if (!_isDisposed)
        {
            _isDisposed = true;
            Remove();
        }
    }

    /// <summary>The root alert of the level, or null.</summary>
    public AlertExtended? RootAlert() => _rootAlert;

    /// <summary>Sets the root alert, and starts its metrics if <paramref name="isRequiredToStart"/>.</summary>
    public void SetRootAlert(AlertExtended? alert, bool isRequiredToStart)
    {
        _rootAlert = alert;
        if (isRequiredToStart)
        {
            AttributeMeter.StartAlert(_rootAlert);
        }
    }

    /// <summary>
    /// Adds an alert below the root alert: stops the metrics of the last alert and starts the new one's. Only extended
    /// alerts are added; true if added.
    /// </summary>
    public bool AddAlert(Gravity gravity, Alert alert)
    {
        if (alert is not AlertExtended alertExtended)
        {
            return false;
        }

        // looking for the parent of the parameter alert to release the previous alert
        AlertExtended rootAlert = _rootAlert!;
        CompositeAlerts compositeAlert = rootAlert.CompositeAlerts(true)!;

        // update metrics of the previous alert
        AttributeMeter.StopAlert(_lastAlert);

        _lastAlert = alertExtended;
        // set start metrics of the new alert
        AttributeMeter.StartAlert(_lastAlert);

        // add child alert
        compositeAlert.AddAlert(gravity, alert);

        return true;
    }

    // Removes the level from the report, stopping the metrics of the last and the root alerts.
    private void Remove()
    {
        Report? defaultReport = global::Kerf.Engine.Foundation.Message.DefaultReport();
        if (defaultReport is null || !defaultReport.IsActiveInMessenger())
        {
            return;
        }

        AttributeMeter.StopAlert(_lastAlert);

        global::Kerf.Engine.Foundation.Message.DefaultReport()?.RemoveLevel(this);
    }
}
