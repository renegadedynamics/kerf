// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ChamfMode.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Fillets.Data;

/// <summary>The mode of a chamfer.</summary>
internal enum ChamfMode
{
    /// <summary>Constant distance from the spine to one of the two surfaces.</summary>
    ClassicChamfer,
    /// <summary>Symmetric chamfer of constant throat: the height of the isosceles triangle in the section.</summary>
    ConstThroatChamfer,
    /// <summary>Chamfer of constant throat whose section is a right-angled triangle: the first surface is virtually offset into the solid, the apex lies on its intersection with the second surface, the right angle is at the top, and the leg from the apex to the top is the throat.</summary>
    ConstThroatWithPenetrationChamfer,
}
