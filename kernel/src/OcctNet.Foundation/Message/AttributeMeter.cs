// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeMeter.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_AttributeMeter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Diagnostics;
using static Kerf.Engine.Foundation.MetricType;

namespace Kerf.Engine.Foundation;

/// <summary>An alert attribute holding the start and stop values of metrics (time, memory).</summary>
/// <remarks>
/// <para>
/// The metrics come from .NET in place of OSD_Timer, OSD_Chronometer and OSD_MemInfo (decision 624): the wall
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
public class AttributeMeter : MessageAttribute
{
    private readonly NCollection_IndexedDataMap<MetricType, (double first, double second)> _metrics = new(); //!< computed metrics

    /// <summary>The value of a metric that isn't defined: -1.</summary>
    public static double UndefinedMetricValue() => -1.0;

    /// <summary>A meter with the name.</summary>
    public AttributeMeter(string name = "")
        : base(name)
    {
    }

    /// <summary>True if the attribute has values for the metric.</summary>
    public bool HasMetric(MetricType metric) => _metrics.Contains(metric);

    /// <summary>True if both values of the metric are set.</summary>
    public bool IsMetricValid(MetricType metric)
    {
        return Math.Abs(StartValue(metric) - UndefinedMetricValue()) > global::Kerf.Engine.Foundation.Precision.Confusion()
               && Math.Abs(StopValue(metric) - UndefinedMetricValue()) > global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    /// <summary>The start value of the metric (undefined if none).</summary>
    public double StartValue(MetricType metric)
    {
        if (!HasMetric(metric))
        {
            return UndefinedMetricValue();
        }
        return _metrics.FindFromKey(metric).first;
    }

    /// <summary>Sets the start value of the metric (adding the metric, its stop value undefined).</summary>
    public void SetStartValue(MetricType metric, double value)
    {
        if (_metrics.Contains(metric))
        {
            _metrics.ChangeFromKey(metric).first = value;
        }
        else
        {
            _metrics.Add(metric, (value, UndefinedMetricValue()));
        }
    }

    /// <summary>The stop value of the metric (undefined if none).</summary>
    public double StopValue(MetricType metric)
    {
        if (!HasMetric(metric))
        {
            return UndefinedMetricValue();
        }
        return _metrics.FindFromKey(metric).second;
    }

    /// <summary>Sets the stop value of the metric, if the metric has a start value.</summary>
    public void SetStopValue(MetricType metric, double value)
    {
        if (_metrics.Contains(metric))
        {
            _metrics.ChangeFromKey(metric).second = value;
        }
    }

    /// <summary>Sets the start values of the default report's metrics into the alert.</summary>
    public static void StartAlert(AlertExtended? alert) => SetAlertMetrics(alert, true);

    /// <summary>Sets the stop values of the default report's metrics into the alert.</summary>
    public static void StopAlert(AlertExtended? alert) => SetAlertMetrics(alert, false);

    /// <summary>
    /// Sets the current values of the default report's active metrics into the alert, if its attribute is a meter:
    /// as start values if <paramref name="startValue"/>, as stop values otherwise.
    /// </summary>
    public static void SetAlertMetrics(AlertExtended? alert, bool startValue)
    {
        if (alert is null)
        {
            return;
        }

        if (alert.Attribute() is not AttributeMeter meterAttribute)
        {
            return;
        }

        Report report = global::Kerf.Engine.Foundation.Message.DefaultReport(true)!;
        NCollection_IndexedMap<MetricType> activeMetrics = report.ActiveMetrics();

        // time metrics
        if (activeMetrics.Contains(MetricType_WallClock))
        {
            double time = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            if (startValue)
            {
                meterAttribute.SetStartValue(MetricType_WallClock, time);
            }
            else
            {
                meterAttribute.SetStopValue(MetricType_WallClock, time);
            }
        }

        if (!HasProcess)
        {
            return;
        }

        if (activeMetrics.Contains(MetricType_ProcessCPUUserTime)
            || activeMetrics.Contains(MetricType_ProcessCPUSystemTime))
        {
            using Process process = Process.GetCurrentProcess();
            double processUserTime = process.UserProcessorTime.TotalSeconds;
            double processSystemTime = process.PrivilegedProcessorTime.TotalSeconds;
            if (activeMetrics.Contains(MetricType_ProcessCPUUserTime))
            {
                SetValue(meterAttribute, MetricType_ProcessCPUUserTime, processUserTime, startValue);
            }
            if (activeMetrics.Contains(MetricType_ProcessCPUSystemTime))
            {
                SetValue(meterAttribute, MetricType_ProcessCPUSystemTime, processSystemTime, startValue);
            }
        }
        // Message_MetricType_ThreadCPUUserTime and ThreadCPUSystemTime: no portable per-thread CPU time in .NET

        // memory metrics
        bool hasMemory = false;
        for (int iter = 1; iter <= activeMetrics.Extent(); ++iter)
        {
            if (activeMetrics.FindKey(iter) >= MetricType_MemPrivate)
            {
                hasMemory = true;
            }
        }
        if (!hasMemory)
        {
            return;
        }

        using Process memProcess = Process.GetCurrentProcess();
        for (int iter = 1; iter <= activeMetrics.Extent(); ++iter)
        {
            MetricType metric = activeMetrics.FindKey(iter);
            long bytes;
            switch (metric)
            {
                case MetricType_MemPrivate: bytes = memProcess.PrivateMemorySize64; break;
                case MetricType_MemVirtual: bytes = memProcess.VirtualMemorySize64; break;
                case MetricType_MemWorkingSet: bytes = memProcess.WorkingSet64; break;
                case MetricType_MemWorkingSetPeak: bytes = memProcess.PeakWorkingSet64; break;
                case MetricType_MemSwapUsage: bytes = memProcess.PagedMemorySize64; break;
                case MetricType_MemSwapUsagePeak: bytes = memProcess.PeakPagedMemorySize64; break;
                case MetricType_MemHeapUsage: bytes = GC.GetTotalMemory(false); break;
                default: continue;
            }
            // OSD_MemInfo::ValuePreciseMiB
            SetValue(meterAttribute, metric, bytes / (1024.0 * 1024.0), startValue);
        }
    }

    private static readonly bool HasProcess = !OperatingSystem.IsBrowser() && !OperatingSystem.IsWasi();

    private static void SetValue(AttributeMeter meter, MetricType metric, double value, bool startValue)
    {
        if (startValue)
        {
            meter.SetStartValue(metric, value);
        }
        else
        {
            meter.SetStopValue(metric, value);
        }
    }
}
