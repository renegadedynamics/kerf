// Port of: src/FoundationClasses/TKernel/Message/Message_MetricType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The metrics a report can record on its alerts (<see cref="Message_AttributeMeter"/>).</summary>
public enum Message_MetricType
{
    Message_MetricType_None,                 //!< no computation
    Message_MetricType_ThreadCPUUserTime,    //!< OSD_Chronometer::GetThreadCPU user time
    Message_MetricType_ThreadCPUSystemTime,  //!< OSD_Chronometer::GetThreadCPU system time
    Message_MetricType_ProcessCPUUserTime,   //!< OSD_Chronometer::GetProcessCPU user time
    Message_MetricType_ProcessCPUSystemTime, //!< OSD_Chronometer::GetProcessCPU system time
    Message_MetricType_WallClock,            //!< OSD_Timer elapsed time
    Message_MetricType_MemPrivate,           //!< OSD_MemInfo::MemPrivate
    Message_MetricType_MemVirtual,           //!< OSD_MemInfo::MemVirtual
    Message_MetricType_MemWorkingSet,        //!< OSD_MemInfo::MemWorkingSet
    Message_MetricType_MemWorkingSetPeak,    //!< OSD_MemInfo::MemWorkingSetPeak
    Message_MetricType_MemSwapUsage,         //!< OSD_MemInfo::MemSwapUsage
    Message_MetricType_MemSwapUsagePeak,     //!< OSD_MemInfo::MemSwapUsagePeak
    Message_MetricType_MemHeapUsage,         //!< OSD_MemInfo::MemHeapUsage
}
