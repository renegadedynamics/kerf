// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeMeter.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeMeter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using static Occt.Message_MetricType;

namespace Occt;

/// <summary>An alert attribute holding the start and stop values of metrics (time, memory).</summary>
/// <remarks>
/// <para>
/// The metrics come from .NET in place of OSD_Timer, OSD_Chronometer and OSD_MemInfo (decision 6113): the wall
/// clock is <see cref="Stopwatch.GetTimestamp"/> in seconds; the process's CPU user and system times are
/// <see cref="Process.UserProcessorTime"/> and <see cref="Process.PrivilegedProcessorTime"/> in seconds; the
/// memory metrics are the process's counters in MiB (private bytes, virtual memory, working set and its peak, paged
/// memory and its peak for the swap usage) and the managed heap (<see cref="GC.GetTotalMemory"/>) for the heap
/// usage. .NET has no portable per-thread CPU time, so the thread metrics are left undefined
/// (<see cref="UndefinedMetricValue"/>), as are all but the wall clock where the runtime has no process (browser
/// WebAssembly, WASI).
/// </para>
/// <para>
/// The metrics are those of the process-wide default report (<see cref="Message.DefaultReport"/>), which only
/// Draw-lite and tests set up; an algorithm's own report never computes them.
/// </para>
/// </remarks>
public class Message_AttributeMeter : Message_Attribute
{
    private readonly NCollection_IndexedDataMap<Message_MetricType, (double first, double second)> myMetrics = new(); //!< computed metrics

    /// <summary>The value of a metric that isn't defined: -1.</summary>
    public static double UndefinedMetricValue() => -1.0;

    /// <summary>A meter with the name.</summary>
    public Message_AttributeMeter(string theName = "")
        : base(theName)
    {
    }

    /// <summary>True if the attribute has values for the metric.</summary>
    public bool HasMetric(Message_MetricType theMetric) => myMetrics.Contains(theMetric);

    /// <summary>True if both values of the metric are set.</summary>
    public bool IsMetricValid(Message_MetricType theMetric)
    {
        return Math.Abs(StartValue(theMetric) - UndefinedMetricValue()) > Precision.Confusion()
               && Math.Abs(StopValue(theMetric) - UndefinedMetricValue()) > Precision.Confusion();
    }

    /// <summary>The start value of the metric (undefined if none).</summary>
    public double StartValue(Message_MetricType theMetric)
    {
        if (!HasMetric(theMetric))
        {
            return UndefinedMetricValue();
        }
        return myMetrics.FindFromKey(theMetric).first;
    }

    /// <summary>Sets the start value of the metric (adding the metric, its stop value undefined).</summary>
    public void SetStartValue(Message_MetricType theMetric, double theValue)
    {
        if (myMetrics.Contains(theMetric))
        {
            myMetrics.ChangeFromKey(theMetric).first = theValue;
        }
        else
        {
            myMetrics.Add(theMetric, (theValue, UndefinedMetricValue()));
        }
    }

    /// <summary>The stop value of the metric (undefined if none).</summary>
    public double StopValue(Message_MetricType theMetric)
    {
        if (!HasMetric(theMetric))
        {
            return UndefinedMetricValue();
        }
        return myMetrics.FindFromKey(theMetric).second;
    }

    /// <summary>Sets the stop value of the metric, if the metric has a start value.</summary>
    public void SetStopValue(Message_MetricType theMetric, double theValue)
    {
        if (myMetrics.Contains(theMetric))
        {
            myMetrics.ChangeFromKey(theMetric).second = theValue;
        }
    }

    /// <summary>Sets the start values of the default report's metrics into the alert.</summary>
    public static void StartAlert(Message_AlertExtended? theAlert) => SetAlertMetrics(theAlert, true);

    /// <summary>Sets the stop values of the default report's metrics into the alert.</summary>
    public static void StopAlert(Message_AlertExtended? theAlert) => SetAlertMetrics(theAlert, false);

    /// <summary>
    /// Sets the current values of the default report's active metrics into the alert, if its attribute is a meter:
    /// as start values if <paramref name="theStartValue"/>, as stop values otherwise.
    /// </summary>
    public static void SetAlertMetrics(Message_AlertExtended? theAlert, bool theStartValue)
    {
        if (theAlert is null)
        {
            return;
        }

        if (theAlert.Attribute() is not Message_AttributeMeter aMeterAttribute)
        {
            return;
        }

        Message_Report aReport = Message.DefaultReport(true)!;
        NCollection_IndexedMap<Message_MetricType> anActiveMetrics = aReport.ActiveMetrics();

        // time metrics
        if (anActiveMetrics.Contains(Message_MetricType_WallClock))
        {
            double aTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            if (theStartValue)
            {
                aMeterAttribute.SetStartValue(Message_MetricType_WallClock, aTime);
            }
            else
            {
                aMeterAttribute.SetStopValue(Message_MetricType_WallClock, aTime);
            }
        }

        if (!HasProcess)
        {
            return;
        }

        if (anActiveMetrics.Contains(Message_MetricType_ProcessCPUUserTime)
            || anActiveMetrics.Contains(Message_MetricType_ProcessCPUSystemTime))
        {
            using Process aProcess = Process.GetCurrentProcess();
            double aProcessUserTime = aProcess.UserProcessorTime.TotalSeconds;
            double aProcessSystemTime = aProcess.PrivilegedProcessorTime.TotalSeconds;
            if (anActiveMetrics.Contains(Message_MetricType_ProcessCPUUserTime))
            {
                SetValue(aMeterAttribute, Message_MetricType_ProcessCPUUserTime, aProcessUserTime, theStartValue);
            }
            if (anActiveMetrics.Contains(Message_MetricType_ProcessCPUSystemTime))
            {
                SetValue(aMeterAttribute, Message_MetricType_ProcessCPUSystemTime, aProcessSystemTime, theStartValue);
            }
        }
        // Message_MetricType_ThreadCPUUserTime and ThreadCPUSystemTime: no portable per-thread CPU time in .NET

        // memory metrics
        bool hasMemory = false;
        for (int anIter = 1; anIter <= anActiveMetrics.Extent(); ++anIter)
        {
            if (anActiveMetrics.FindKey(anIter) >= Message_MetricType_MemPrivate)
            {
                hasMemory = true;
            }
        }
        if (!hasMemory)
        {
            return;
        }

        using Process aMemProcess = Process.GetCurrentProcess();
        for (int anIter = 1; anIter <= anActiveMetrics.Extent(); ++anIter)
        {
            Message_MetricType aMetric = anActiveMetrics.FindKey(anIter);
            long aBytes;
            switch (aMetric)
            {
                case Message_MetricType_MemPrivate: aBytes = aMemProcess.PrivateMemorySize64; break;
                case Message_MetricType_MemVirtual: aBytes = aMemProcess.VirtualMemorySize64; break;
                case Message_MetricType_MemWorkingSet: aBytes = aMemProcess.WorkingSet64; break;
                case Message_MetricType_MemWorkingSetPeak: aBytes = aMemProcess.PeakWorkingSet64; break;
                case Message_MetricType_MemSwapUsage: aBytes = aMemProcess.PagedMemorySize64; break;
                case Message_MetricType_MemSwapUsagePeak: aBytes = aMemProcess.PeakPagedMemorySize64; break;
                case Message_MetricType_MemHeapUsage: aBytes = GC.GetTotalMemory(false); break;
                default: continue;
            }
            // OSD_MemInfo::ValuePreciseMiB
            SetValue(aMeterAttribute, aMetric, aBytes / (1024.0 * 1024.0), theStartValue);
        }
    }

    private static readonly bool HasProcess = !OperatingSystem.IsBrowser() && !OperatingSystem.IsWasi();

    private static void SetValue(Message_AttributeMeter theMeter, Message_MetricType theMetric, double theValue, bool theStartValue)
    {
        if (theStartValue)
        {
            theMeter.SetStartValue(theMetric, theValue);
        }
        else
        {
            theMeter.SetStopValue(theMetric, theValue);
        }
    }
}
