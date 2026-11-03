// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Runtime.CompilerServices;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Loads Shape Healing's messages before any test runs. As OCCT's, ShapeExtend.Init loads them only when its probe
/// keyword is missing, and Message_MsgFile.Msg stores an unknown keyword's error text under the keyword, so a test that
/// sends a healing message first would leave every later test in the process without the texts, depending on the order
/// the tests run in (occt-issues.md, unit 7A's section).
/// </summary>
internal static class ShapeHealingMessages
{
    [ModuleInitializer]
    internal static void Load() => MsgFile.LoadFromString(SHMessage_SHAPE_us.Text);
}
