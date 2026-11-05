// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ChamfMethod.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Fillets.Data;

/// <summary>The method of a chamfer: symmetric, two distances, or a distance and an angle.</summary>
internal enum ChamfMethod
{
    Sym,
    TwoDist,
    DistAngle,
}
