// Port of: src/FoundationClasses/TKernel/Message/Message_StatusType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS. Original implementation copyright (c) RINA S.p.A.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>The types of execution status of <see cref="ExecStatus"/>.</summary>
public enum StatusType
{
    Done = 0x00000100,
    Warn = 0x00000200,
    Alarm = 0x00000400,
    Fail = 0x00000800,
}
