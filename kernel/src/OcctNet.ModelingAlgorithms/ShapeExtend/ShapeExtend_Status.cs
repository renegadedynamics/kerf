// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The flags of the return statuses of Shape Healing's methods: eight DONE and eight FAIL flags, with
/// <see cref="ShapeExtend_OK"/> (no flag), <see cref="ShapeExtend_DONE"/> (any DONE flag) and
/// <see cref="ShapeExtend_FAIL"/> (any FAIL flag). A status can hold several flags at once.
/// </summary>
public enum ShapeExtend_Status
{
    /// <summary>Nothing done, everything OK.</summary>
    ShapeExtend_OK,
    ShapeExtend_DONE1,
    ShapeExtend_DONE2,
    ShapeExtend_DONE3,
    ShapeExtend_DONE4,
    ShapeExtend_DONE5,
    ShapeExtend_DONE6,
    ShapeExtend_DONE7,
    ShapeExtend_DONE8,
    /// <summary>Something was done (any of the DONE flags).</summary>
    ShapeExtend_DONE,
    ShapeExtend_FAIL1,
    ShapeExtend_FAIL2,
    ShapeExtend_FAIL3,
    ShapeExtend_FAIL4,
    ShapeExtend_FAIL5,
    ShapeExtend_FAIL6,
    ShapeExtend_FAIL7,
    ShapeExtend_FAIL8,
    /// <summary>The method failed (any of the FAIL flags).</summary>
    ShapeExtend_FAIL,
}
