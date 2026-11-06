// Port of: src/FoundationClasses/TKernel/Message/Message_MetricType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Foundation;

/// <summary>The metrics a report can record on its alerts (<see cref="AttributeMeter"/>).</summary>
internal enum MetricType
{
    MetricType_None,                 //!< no computation
    MetricType_ThreadCPUUserTime,    //!< OSD_Chronometer::GetThreadCPU user time
    MetricType_ThreadCPUSystemTime,  //!< OSD_Chronometer::GetThreadCPU system time
    MetricType_ProcessCPUUserTime,   //!< OSD_Chronometer::GetProcessCPU user time
    MetricType_ProcessCPUSystemTime, //!< OSD_Chronometer::GetProcessCPU system time
    MetricType_WallClock,            //!< OSD_Timer elapsed time
    MetricType_MemPrivate,           //!< OSD_MemInfo::MemPrivate
    MetricType_MemVirtual,           //!< OSD_MemInfo::MemVirtual
    MetricType_MemWorkingSet,        //!< OSD_MemInfo::MemWorkingSet
    MetricType_MemWorkingSetPeak,    //!< OSD_MemInfo::MemWorkingSetPeak
    MetricType_MemSwapUsage,         //!< OSD_MemInfo::MemSwapUsage
    MetricType_MemSwapUsagePeak,     //!< OSD_MemInfo::MemSwapUsagePeak
    MetricType_MemHeapUsage,         //!< OSD_MemInfo::MemHeapUsage
}
