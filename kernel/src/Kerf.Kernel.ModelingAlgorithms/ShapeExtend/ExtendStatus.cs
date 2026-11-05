// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Healing.Extend;

/// <summary>
/// The flags of the return statuses of Shape Healing's methods: eight DONE and eight FAIL flags, with
/// <see cref="OK"/> (no flag), <see cref="Done"/> (any DONE flag) and
/// <see cref="Fail"/> (any FAIL flag). A status can hold several flags at once.
/// </summary>
internal enum ExtendStatus
{
    /// <summary>Nothing done, everything OK.</summary>
    OK,
    DONE1,
    DONE2,
    DONE3,
    DONE4,
    DONE5,
    DONE6,
    DONE7,
    DONE8,
    /// <summary>Something was done (any of the DONE flags).</summary>
    Done,
    FAIL1,
    FAIL2,
    FAIL3,
    FAIL4,
    FAIL5,
    FAIL6,
    FAIL7,
    FAIL8,
    /// <summary>The method failed (any of the FAIL flags).</summary>
    Fail,
}
