// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_TypeOfConcavity.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The type of connection of two faces along an edge (<see cref="ChFi3d.DefineConnectType"/>).</summary>
public enum ChFiDS_TypeOfConcavity
{
    ChFiDS_Concave,
    ChFiDS_Convex,
    ChFiDS_Tangential,
    ChFiDS_FreeBound,
    ChFiDS_Other,
    ChFiDS_Mixed,
}
