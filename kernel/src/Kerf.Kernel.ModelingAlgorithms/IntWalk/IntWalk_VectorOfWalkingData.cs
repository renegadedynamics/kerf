// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_VectorOfWalkingData.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_VectorOfInteger.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>The state of a start point of IntWalk_IWalking: its parameters and its state (etat).</summary>
/// <remarks>
/// OCCT's typedefs are written out: <c>IntWalk_VectorOfWalkingData</c> is
/// <c>NCollection_LinearVector&lt;IntWalk_WalkingData&gt;</c> and <c>IntWalk_VectorOfInteger</c>
/// <c>NCollection_LinearVector&lt;int&gt;</c>, both indexed from 0 (IWalking puts a dummy item at 0).
/// </remarks>
public struct WalkingData
{
    public double ustart;
    public double vstart;
    public int etat;
}
