// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_TypeOfConcavity.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Fillets.Data;

/// <summary>The type of connection of two faces along an edge (<see cref="ChFi3d.DefineConnectType"/>).</summary>
internal enum TypeOfConcavity
{
    Concave,
    Convex,
    Tangential,
    FreeBound,
    Other,
    Mixed,
}
