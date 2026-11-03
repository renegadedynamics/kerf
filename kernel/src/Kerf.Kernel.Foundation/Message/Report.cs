// Port of: src/FoundationClasses/TKernel/Message/Message_Report.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Report.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Occt;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Foundation;

/// <summary>
/// A container of alerts sorted by gravity: an algorithm adds alerts as it runs (<see cref="AddAlert"/>), the caller
/// asks for them (<see cref="HasAlert(Type)"/>, <see cref="GetAlerts"/>), dumps them (<see cref="Dump(TextWriter)"/>)
/// or sends them to a messenger. An alert that can merge with one of the same type already in the report is merged,
/// not added. Reports of nested algorithms are collected with <see cref="Merge(Report)"/>.
/// </summary>
/// <remarks>
/// <para>
/// A working object owned by its algorithm (decision 584). <see cref="AddAlert"/> and the level and clearing methods
/// are thread-safe, as OCCT's (a lock in place of <c>std::mutex</c>), so parallel loop bodies can add alerts.
/// </para>
/// <para>
/// OCCT keeps the levels as raw pointers to <see cref="Level"/> sentries; the port keeps references, compared
/// by identity. <c>Standard_OStream</c> is a <see cref="TextWriter"/>; <c>DumpJson</c> is left out.
/// </para>
/// </remarks>
public class Report : Transient
{
    protected readonly Lock _mutex = new();
    protected CompositeAlerts? _compositAlerts; //!< container of alerts
    protected readonly NCollection_Sequence<Level> _alertLevels = new(); //!< container of active levels, new alerts are added below the latest level
    protected readonly IndexedSet<MetricType> _activeMetrics = new(); //!< metrics to compute on alerts
    protected int _limit;                  //!< Maximum number of collected alerts on the top level
    protected bool _isActiveInMessenger;   //!< state whether the report is activated in messenger

    /// <summary>An empty report, without limit.</summary>
    public Report()
    {
        _limit = -1;
        _isActiveInMessenger = false;
    }

    /// <summary>
    /// Adds the alert with the gravity, below the latest level if one is open; merges it if it can merge with an alert
    /// of the same type. Thread-safe.
    /// </summary>
    public void AddAlert(Gravity gravity, Alert alert)
    {
        lock (_mutex)
        {
            // alerts of the top level
            if (_alertLevels.IsEmpty())
            {
                CompositeAlerts compositeAlert = CompositeAlerts(true)!;
                if (compositeAlert.AddAlert(gravity, alert))
                {
                    return;
                }

                // remove alerts under the report only
                // OCCT: this is reached when the alert was MERGED (not appended), and with the default limit -1 the
                // test is always true, so merging removes the first alert of the gravity: a second simple alert of the
                // same type empties the list. The limit is never applied to an appended alert (occt-issues.md, 6A).
                NCollection_List<Alert> alerts = compositeAlert.Alerts(gravity);
                if (alerts.Extent() > _limit)
                {
                    compositeAlert.RemoveAlert(gravity, alerts.First());
                }
                return;
            }

            // if there are some levels of alerts, the new alert will be placed below the root
            _alertLevels.Last().AddAlert(gravity, alert);
        }
    }

    /// <summary>The alerts of the gravity: the report's own list, which callers only read.</summary>
    public NCollection_List<Alert> GetAlerts(Gravity gravity)
    {
        if (_compositAlerts is null)
        {
            return new NCollection_List<Alert>();
        }
        return _compositAlerts.Alerts(gravity);
    }

    /// <summary>True if an alert of exactly the type (not a derived type) was recorded, with any gravity.</summary>
    public bool HasAlert(Type type)
    {
        for (int iGravity = (int)Trace; iGravity <= (int)Fail; ++iGravity)
        {
            if (HasAlert(type, (Gravity)iGravity))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>True if an alert of exactly the type (not a derived type) was recorded with the gravity.</summary>
    public bool HasAlert(Type type, Gravity gravity)
    {
        if (CompositeAlerts() is null)
        {
            return false;
        }
        return CompositeAlerts()!.HasAlert(type, gravity);
    }

    /// <summary>True if a printer to this report is registered in a messenger (the flag kept by the report).</summary>
    public bool IsActiveInMessenger(Messenger? messenger = null) => _isActiveInMessenger;

    /// <summary>
    /// Registers a new <see cref="PrinterToReport"/> to this report in the messenger (the default one if
    /// null), or removes this report's printers from it.
    /// </summary>
    /// <remarks>
    /// Adding and removing printers is internal to the port (decision 521): on the default messenger only Draw-lite
    /// and tests may do it.
    /// </remarks>
    internal void ActivateInMessenger(bool toActivate, Messenger? messenger = null)
    {
        if (toActivate == IsActiveInMessenger())
        {
            return;
        }

        _isActiveInMessenger = toActivate;
        Messenger aMessenger = messenger ?? global::Kerf.Engine.Foundation.Message.DefaultMessenger();
        if (toActivate)
        {
            PrinterToReport printerToReport = new();
            printerToReport.SetReport(this);
            aMessenger.AddPrinter(printerToReport);
        }
        else // deactivate
        {
            NCollection_Sequence<Printer> printersToRemove = new();
            foreach (Printer printer in aMessenger.Printers())
            {
                if (printer is PrinterToReport toReport && toReport.Report() == this)
                {
                    printersToRemove.Append(printer);
                }
            }
            foreach (Printer printer in printersToRemove)
            {
                aMessenger.RemovePrinter(printer);
            }
        }
    }

    /// <summary>
    /// Updates the active flag: true if the messenger (the default one if null) has a printer to this report.
    /// </summary>
    public void UpdateActiveInMessenger(Messenger? messenger = null)
    {
        Messenger aMessenger = messenger ?? global::Kerf.Engine.Foundation.Message.DefaultMessenger();
        foreach (Printer printer in aMessenger.Printers())
        {
            if (printer is PrinterToReport toReport && toReport.Report() == this)
            {
                _isActiveInMessenger = true;
                return;
            }
        }
        _isActiveInMessenger = false;
    }

    /// <summary>Opens a level: its root alert goes on the report or below the previous level.</summary>
    public void AddLevel(Level level, string name)
    {
        lock (_mutex)
        {
            _alertLevels.Append(level);

            AlertExtended levelRootAlert = new();

            MessageAttribute attribute;
            if (ActiveMetrics().Count != 0)
            {
                attribute = new AttributeMeter(name);
            }
            else
            {
                attribute = new MessageAttribute(name);
            }
            levelRootAlert.SetAttribute(attribute);
            level.SetRootAlert(levelRootAlert, _alertLevels.Size() == 1);

            if (_alertLevels.Size() == 1) // this is the first level, so root alert should be pushed in the report composite of alerts
            {
                CompositeAlerts(true)!.AddAlert(Info, level.RootAlert());
            }
            if (_alertLevels.Size() > 1)
            {
                // root alert of next levels should be pushed under the previous level
                Level prevLevel = _alertLevels.Value(_alertLevels.Size() - 1); // previous level
                prevLevel.AddAlert(Info, levelRootAlert);
            }
        }
    }

    /// <summary>Closes the level and the levels opened after it.</summary>
    public void RemoveLevel(Level level)
    {
        lock (_mutex)
        {
            for (int levelIndex = _alertLevels.Length(); levelIndex >= 1; levelIndex--)
            {
                Level aLevel = _alertLevels.Value(levelIndex);
                AttributeMeter.StopAlert(aLevel.RootAlert());

                _alertLevels.Remove(levelIndex);
                if (ReferenceEquals(aLevel, level))
                {
                    return;
                }
            }
        }
    }

    /// <summary>Removes all the alerts (and the levels).</summary>
    public void Clear()
    {
        if (CompositeAlerts() is null)
        {
            return;
        }

        lock (_mutex)
        {
            CompositeAlerts()!.Clear();
            _alertLevels.Clear();
        }
    }

    /// <summary>Removes the alerts of the gravity (and the levels).</summary>
    public void Clear(Gravity gravity)
    {
        if (CompositeAlerts() is null)
        {
            return;
        }

        lock (_mutex)
        {
            CompositeAlerts()!.Clear(gravity);
            _alertLevels.Clear();
        }
    }

    /// <summary>Removes the alerts of exactly the type (and the levels).</summary>
    public void Clear(Type type)
    {
        if (CompositeAlerts() is null)
        {
            return;
        }

        lock (_mutex)
        {
            CompositeAlerts()!.Clear(type);
            _alertLevels.Clear();
        }
    }

    /// <summary>The metrics computed on the alerts (the report's own map, which callers only read).</summary>
    public IndexedSet<MetricType> ActiveMetrics() => _activeMetrics;

    /// <summary>Activates or deactivates a metric.</summary>
    public void SetActiveMetric(MetricType metricType, bool activate)
    {
        if (activate == _activeMetrics.Contains(metricType))
        {
            return;
        }

        if (activate)
        {
            _activeMetrics.Add(metricType);
        }
        else
        {
            _activeMetrics.Remove(metricType);
        }
    }

    /// <summary>Deactivates all the metrics.</summary>
    public void ClearMetrics() => _activeMetrics.Clear();

    /// <summary>
    /// The largest number of alerts kept on the top level (-1, no limit, by default). See the OCCT issue in
    /// <see cref="AddAlert"/>: OCCT applies it only when an alert merges.
    /// </summary>
    public int Limit() => _limit;

    /// <summary>Sets the largest number of alerts kept on the top level.</summary>
    public void SetLimit(int limit) => _limit = limit;

    /// <summary>Writes the message key of every alert, one per line, gravity by gravity.</summary>
    public void Dump(TextWriter os)
    {
        for (int iGravity = (int)Trace; iGravity <= (int)Fail; ++iGravity)
        {
            Dump(os, (Gravity)iGravity);
        }
    }

    /// <summary>Writes the message key of every alert of the gravity (with their children), one per line.</summary>
    public void Dump(TextWriter os, Gravity gravity)
    {
        if (CompositeAlerts() is null)
        {
            return;
        }

        DumpMessages(os, gravity, CompositeAlerts());
    }

    /// <summary>Sends every alert's message key to the messenger, gravity by gravity.</summary>
    public virtual void SendMessages(Messenger messenger)
    {
        for (int gravIter = (int)Trace; gravIter <= (int)Fail; ++gravIter)
        {
            SendMessages(messenger, (Gravity)gravIter);
        }
    }

    /// <summary>Sends the message key of every alert of the gravity (with their children) to the messenger.</summary>
    public virtual void SendMessages(Messenger messenger, Gravity gravity)
    {
        if (CompositeAlerts() is null)
        {
            return;
        }
        this.SendMessages(messenger, gravity, CompositeAlerts());
    }

    /// <summary>Adds the other report's alerts to this one, gravity by gravity.</summary>
    public void Merge(Report other)
    {
        for (int gravIter = (int)Trace; gravIter <= (int)Fail; ++gravIter)
        {
            Merge(other, (Gravity)gravIter);
        }
    }

    /// <summary>Adds the other report's alerts of the gravity to this one.</summary>
    public void Merge(Report other, Gravity gravity)
    {
        // OCCT iterates the other report's own list; merging a report into itself would grow the list it iterates,
        // so the port iterates a copy of it (the same alerts, in the same order).
        NCollection_List<Alert> alerts = new(other.GetAlerts(gravity));
        for (NCollection_List<Alert>.Iterator it = new(alerts); it.More(); it.Next())
        {
            AddAlert(gravity, it.Value());
        }
    }

    /// <summary>The container of alerts, made first if <paramref name="isCreate"/>.</summary>
    protected CompositeAlerts? CompositeAlerts(bool isCreate = false)
    {
        if (_compositAlerts is null && isCreate)
        {
            _compositAlerts = new CompositeAlerts();
        }

        return _compositAlerts;
    }

    /// <summary>Sends the alerts of the container and their children to the messenger.</summary>
    protected void SendMessages(Messenger messenger, Gravity gravity, CompositeAlerts? compositeAlert)
    {
        if (compositeAlert is null)
        {
            return;
        }

        NCollection_List<Alert> alerts = compositeAlert.Alerts(gravity);
        for (NCollection_List<Alert>.Iterator it = new(alerts); it.More(); it.Next())
        {
            messenger.Send(it.Value().GetMessageKey(), gravity);

            if (it.Value() is not AlertExtended extendedAlert)
            {
                continue;
            }

            CompositeAlerts? compositeAlerts = extendedAlert.CompositeAlerts();
            if (compositeAlerts is null)
            {
                continue;
            }

            this.SendMessages(messenger, gravity, compositeAlerts);
        }
    }

    /// <summary>Writes the alerts of the container and their children.</summary>
    protected void DumpMessages(TextWriter os, Gravity gravity, CompositeAlerts? compositeAlert)
    {
        if (compositeAlert is null)
        {
            return;
        }

        NCollection_List<Alert> alerts = compositeAlert.Alerts(gravity);
        for (NCollection_List<Alert>.Iterator it = new(alerts); it.More(); it.Next())
        {
            os.Write(it.Value().GetMessageKey());
            os.Write('\n');

            if (it.Value() is not AlertExtended extendedAlert)
            {
                continue;
            }

            DumpMessages(os, gravity, extendedAlert.CompositeAlerts());
        }
    }
}
