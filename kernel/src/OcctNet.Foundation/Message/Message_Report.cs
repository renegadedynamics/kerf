// Port of: src/FoundationClasses/TKernel/Message/Message_Report.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Report.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;

namespace Occt;

/// <summary>
/// A container of alerts sorted by gravity: an algorithm adds alerts as it runs (<see cref="AddAlert"/>), the caller
/// asks for them (<see cref="HasAlert(Type)"/>, <see cref="GetAlerts"/>), dumps them (<see cref="Dump(TextWriter)"/>)
/// or sends them to a messenger. An alert that can merge with one of the same type already in the report is merged,
/// not added. Reports of nested algorithms are collected with <see cref="Merge(Message_Report)"/>.
/// </summary>
/// <remarks>
/// <para>
/// A working object owned by its algorithm (decision 5009). <see cref="AddAlert"/> and the level and clearing methods
/// are thread-safe, as OCCT's (a lock in place of <c>std::mutex</c>), so parallel loop bodies can add alerts.
/// </para>
/// <para>
/// OCCT keeps the levels as raw pointers to <see cref="Message_Level"/> sentries; the port keeps references, compared
/// by identity. <c>Standard_OStream</c> is a <see cref="TextWriter"/>; <c>DumpJson</c> is left out.
/// </para>
/// </remarks>
public class Message_Report : Standard_Transient
{
    protected readonly Lock myMutex = new();
    protected Message_CompositeAlerts? myCompositAlerts; //!< container of alerts
    protected readonly NCollection_Sequence<Message_Level> myAlertLevels = new(); //!< container of active levels, new alerts are added below the latest level
    protected readonly NCollection_IndexedMap<Message_MetricType> myActiveMetrics = new(); //!< metrics to compute on alerts
    protected int myLimit;                  //!< Maximum number of collected alerts on the top level
    protected bool myIsActiveInMessenger;   //!< state whether the report is activated in messenger

    /// <summary>An empty report, without limit.</summary>
    public Message_Report()
    {
        myLimit = -1;
        myIsActiveInMessenger = false;
    }

    /// <summary>
    /// Adds the alert with the gravity, below the latest level if one is open; merges it if it can merge with an alert
    /// of the same type. Thread-safe.
    /// </summary>
    public void AddAlert(Message_Gravity theGravity, Message_Alert theAlert)
    {
        lock (myMutex)
        {
            // alerts of the top level
            if (myAlertLevels.IsEmpty())
            {
                Message_CompositeAlerts aCompositeAlert = compositeAlerts(true)!;
                if (aCompositeAlert.AddAlert(theGravity, theAlert))
                {
                    return;
                }

                // remove alerts under the report only
                // OCCT: this is reached when the alert was MERGED (not appended), and with the default limit -1 the
                // test is always true, so merging removes the first alert of the gravity: a second simple alert of the
                // same type empties the list. The limit is never applied to an appended alert (occt-issues.md, 6A).
                NCollection_List<Message_Alert> anAlerts = aCompositeAlert.Alerts(theGravity);
                if (anAlerts.Extent() > myLimit)
                {
                    aCompositeAlert.RemoveAlert(theGravity, anAlerts.First());
                }
                return;
            }

            // if there are some levels of alerts, the new alert will be placed below the root
            myAlertLevels.Last().AddAlert(theGravity, theAlert);
        }
    }

    /// <summary>The alerts of the gravity: the report's own list, which callers only read.</summary>
    public NCollection_List<Message_Alert> GetAlerts(Message_Gravity theGravity)
    {
        if (myCompositAlerts is null)
        {
            return new NCollection_List<Message_Alert>();
        }
        return myCompositAlerts.Alerts(theGravity);
    }

    /// <summary>True if an alert of exactly the type (not a derived type) was recorded, with any gravity.</summary>
    public bool HasAlert(Type theType)
    {
        for (int iGravity = (int)Message_Trace; iGravity <= (int)Message_Fail; ++iGravity)
        {
            if (HasAlert(theType, (Message_Gravity)iGravity))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>True if an alert of exactly the type (not a derived type) was recorded with the gravity.</summary>
    public bool HasAlert(Type theType, Message_Gravity theGravity)
    {
        if (compositeAlerts() is null)
        {
            return false;
        }
        return compositeAlerts()!.HasAlert(theType, theGravity);
    }

    /// <summary>True if a printer to this report is registered in a messenger (the flag kept by the report).</summary>
    public bool IsActiveInMessenger(Message_Messenger? theMessenger = null) => myIsActiveInMessenger;

    /// <summary>
    /// Registers a new <see cref="Message_PrinterToReport"/> to this report in the messenger (the default one if
    /// null), or removes this report's printers from it.
    /// </summary>
    /// <remarks>
    /// Adding and removing printers is internal to the port (decision 521): on the default messenger only Draw-lite
    /// and tests may do it.
    /// </remarks>
    internal void ActivateInMessenger(bool toActivate, Message_Messenger? theMessenger = null)
    {
        if (toActivate == IsActiveInMessenger())
        {
            return;
        }

        myIsActiveInMessenger = toActivate;
        Message_Messenger aMessenger = theMessenger ?? Message.DefaultMessenger();
        if (toActivate)
        {
            Message_PrinterToReport aPrinterToReport = new();
            aPrinterToReport.SetReport(this);
            aMessenger.AddPrinter(aPrinterToReport);
        }
        else // deactivate
        {
            NCollection_Sequence<Message_Printer> aPrintersToRemove = new();
            foreach (Message_Printer aPrinter in aMessenger.Printers())
            {
                if (aPrinter is Message_PrinterToReport aToReport && aToReport.Report() == this)
                {
                    aPrintersToRemove.Append(aPrinter);
                }
            }
            foreach (Message_Printer aPrinter in aPrintersToRemove)
            {
                aMessenger.RemovePrinter(aPrinter);
            }
        }
    }

    /// <summary>
    /// Updates the active flag: true if the messenger (the default one if null) has a printer to this report.
    /// </summary>
    public void UpdateActiveInMessenger(Message_Messenger? theMessenger = null)
    {
        Message_Messenger aMessenger = theMessenger ?? Message.DefaultMessenger();
        foreach (Message_Printer aPrinter in aMessenger.Printers())
        {
            if (aPrinter is Message_PrinterToReport aToReport && aToReport.Report() == this)
            {
                myIsActiveInMessenger = true;
                return;
            }
        }
        myIsActiveInMessenger = false;
    }

    /// <summary>Opens a level: its root alert goes on the report or below the previous level.</summary>
    public void AddLevel(Message_Level theLevel, string theName)
    {
        lock (myMutex)
        {
            myAlertLevels.Append(theLevel);

            Message_AlertExtended aLevelRootAlert = new();

            Message_Attribute anAttribute;
            if (!ActiveMetrics().IsEmpty())
            {
                anAttribute = new Message_AttributeMeter(theName);
            }
            else
            {
                anAttribute = new Message_Attribute(theName);
            }
            aLevelRootAlert.SetAttribute(anAttribute);
            theLevel.SetRootAlert(aLevelRootAlert, myAlertLevels.Size() == 1);

            if (myAlertLevels.Size() == 1) // this is the first level, so root alert should be pushed in the report composite of alerts
            {
                compositeAlerts(true)!.AddAlert(Message_Info, theLevel.RootAlert());
            }
            if (myAlertLevels.Size() > 1)
            {
                // root alert of next levels should be pushed under the previous level
                Message_Level aPrevLevel = myAlertLevels.Value(myAlertLevels.Size() - 1); // previous level
                aPrevLevel.AddAlert(Message_Info, aLevelRootAlert);
            }
        }
    }

    /// <summary>Closes the level and the levels opened after it.</summary>
    public void RemoveLevel(Message_Level theLevel)
    {
        lock (myMutex)
        {
            for (int aLevelIndex = myAlertLevels.Length(); aLevelIndex >= 1; aLevelIndex--)
            {
                Message_Level aLevel = myAlertLevels.Value(aLevelIndex);
                Message_AttributeMeter.StopAlert(aLevel.RootAlert());

                myAlertLevels.Remove(aLevelIndex);
                if (ReferenceEquals(aLevel, theLevel))
                {
                    return;
                }
            }
        }
    }

    /// <summary>Removes all the alerts (and the levels).</summary>
    public void Clear()
    {
        if (compositeAlerts() is null)
        {
            return;
        }

        lock (myMutex)
        {
            compositeAlerts()!.Clear();
            myAlertLevels.Clear();
        }
    }

    /// <summary>Removes the alerts of the gravity (and the levels).</summary>
    public void Clear(Message_Gravity theGravity)
    {
        if (compositeAlerts() is null)
        {
            return;
        }

        lock (myMutex)
        {
            compositeAlerts()!.Clear(theGravity);
            myAlertLevels.Clear();
        }
    }

    /// <summary>Removes the alerts of exactly the type (and the levels).</summary>
    public void Clear(Type theType)
    {
        if (compositeAlerts() is null)
        {
            return;
        }

        lock (myMutex)
        {
            compositeAlerts()!.Clear(theType);
            myAlertLevels.Clear();
        }
    }

    /// <summary>The metrics computed on the alerts (the report's own map, which callers only read).</summary>
    public NCollection_IndexedMap<Message_MetricType> ActiveMetrics() => myActiveMetrics;

    /// <summary>Activates or deactivates a metric.</summary>
    public void SetActiveMetric(Message_MetricType theMetricType, bool theActivate)
    {
        if (theActivate == myActiveMetrics.Contains(theMetricType))
        {
            return;
        }

        if (theActivate)
        {
            myActiveMetrics.Add(theMetricType);
        }
        else
        {
            myActiveMetrics.RemoveKey(theMetricType);
        }
    }

    /// <summary>Deactivates all the metrics.</summary>
    public void ClearMetrics() => myActiveMetrics.Clear();

    /// <summary>
    /// The largest number of alerts kept on the top level (-1, no limit, by default). See the OCCT issue in
    /// <see cref="AddAlert"/>: OCCT applies it only when an alert merges.
    /// </summary>
    public int Limit() => myLimit;

    /// <summary>Sets the largest number of alerts kept on the top level.</summary>
    public void SetLimit(int theLimit) => myLimit = theLimit;

    /// <summary>Writes the message key of every alert, one per line, gravity by gravity.</summary>
    public void Dump(TextWriter theOS)
    {
        for (int iGravity = (int)Message_Trace; iGravity <= (int)Message_Fail; ++iGravity)
        {
            Dump(theOS, (Message_Gravity)iGravity);
        }
    }

    /// <summary>Writes the message key of every alert of the gravity (with their children), one per line.</summary>
    public void Dump(TextWriter theOS, Message_Gravity theGravity)
    {
        if (compositeAlerts() is null)
        {
            return;
        }

        dumpMessages(theOS, theGravity, compositeAlerts());
    }

    /// <summary>Sends every alert's message key to the messenger, gravity by gravity.</summary>
    public virtual void SendMessages(Message_Messenger theMessenger)
    {
        for (int aGravIter = (int)Message_Trace; aGravIter <= (int)Message_Fail; ++aGravIter)
        {
            SendMessages(theMessenger, (Message_Gravity)aGravIter);
        }
    }

    /// <summary>Sends the message key of every alert of the gravity (with their children) to the messenger.</summary>
    public virtual void SendMessages(Message_Messenger theMessenger, Message_Gravity theGravity)
    {
        if (compositeAlerts() is null)
        {
            return;
        }
        sendMessages(theMessenger, theGravity, compositeAlerts());
    }

    /// <summary>Adds the other report's alerts to this one, gravity by gravity.</summary>
    public void Merge(Message_Report theOther)
    {
        for (int aGravIter = (int)Message_Trace; aGravIter <= (int)Message_Fail; ++aGravIter)
        {
            Merge(theOther, (Message_Gravity)aGravIter);
        }
    }

    /// <summary>Adds the other report's alerts of the gravity to this one.</summary>
    public void Merge(Message_Report theOther, Message_Gravity theGravity)
    {
        // OCCT iterates the other report's own list; merging a report into itself would grow the list it iterates,
        // so the port iterates a copy of it (the same alerts, in the same order).
        NCollection_List<Message_Alert> anAlerts = new(theOther.GetAlerts(theGravity));
        for (NCollection_List<Message_Alert>.Iterator anIt = new(anAlerts); anIt.More(); anIt.Next())
        {
            AddAlert(theGravity, anIt.Value());
        }
    }

    /// <summary>The container of alerts, made first if <paramref name="isCreate"/>.</summary>
    protected Message_CompositeAlerts? compositeAlerts(bool isCreate = false)
    {
        if (myCompositAlerts is null && isCreate)
        {
            myCompositAlerts = new Message_CompositeAlerts();
        }

        return myCompositAlerts;
    }

    /// <summary>Sends the alerts of the container and their children to the messenger.</summary>
    protected void sendMessages(Message_Messenger theMessenger, Message_Gravity theGravity, Message_CompositeAlerts? theCompositeAlert)
    {
        if (theCompositeAlert is null)
        {
            return;
        }

        NCollection_List<Message_Alert> anAlerts = theCompositeAlert.Alerts(theGravity);
        for (NCollection_List<Message_Alert>.Iterator anIt = new(anAlerts); anIt.More(); anIt.Next())
        {
            theMessenger.Send(anIt.Value().GetMessageKey(), theGravity);

            if (anIt.Value() is not Message_AlertExtended anExtendedAlert)
            {
                continue;
            }

            Message_CompositeAlerts? aCompositeAlerts = anExtendedAlert.CompositeAlerts();
            if (aCompositeAlerts is null)
            {
                continue;
            }

            sendMessages(theMessenger, theGravity, aCompositeAlerts);
        }
    }

    /// <summary>Writes the alerts of the container and their children.</summary>
    protected void dumpMessages(TextWriter theOS, Message_Gravity theGravity, Message_CompositeAlerts? theCompositeAlert)
    {
        if (theCompositeAlert is null)
        {
            return;
        }

        NCollection_List<Message_Alert> anAlerts = theCompositeAlert.Alerts(theGravity);
        for (NCollection_List<Message_Alert>.Iterator anIt = new(anAlerts); anIt.More(); anIt.Next())
        {
            theOS.Write(anIt.Value().GetMessageKey());
            theOS.Write('\n');

            if (anIt.Value() is not Message_AlertExtended anExtendedAlert)
            {
                continue;
            }

            dumpMessages(theOS, theGravity, anExtendedAlert.CompositeAlerts());
        }
    }
}
