// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_PointPair.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_PointPair.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// A pair of STEP points, unordered: (P1, P2) equals (P2, P1). The key of the edges a faceted brep's poly loops share.
/// Points are compared by reference, as OCCT compares handles.
/// </summary>
public readonly struct StepToTopoDS_PointPair : IEquatable<StepToTopoDS_PointPair>
{
    private readonly StepGeom_CartesianPoint? myP1;
    private readonly StepGeom_CartesianPoint? myP2;

    public StepToTopoDS_PointPair(StepGeom_CartesianPoint? P1, StepGeom_CartesianPoint? P2)
    {
        myP1 = P1;
        myP2 = P2;
    }

    public StepGeom_CartesianPoint? GetPoint1() => myP1;

    public StepGeom_CartesianPoint? GetPoint2() => myP2;

    public bool Equals(StepToTopoDS_PointPair thePointPair)
        => (ReferenceEquals(myP1, thePointPair.myP1) && ReferenceEquals(myP2, thePointPair.myP2))
           || (ReferenceEquals(myP1, thePointPair.myP2) && ReferenceEquals(myP2, thePointPair.myP1));

    public override bool Equals(object? obj) => obj is StepToTopoDS_PointPair aPair && Equals(aPair);

    // Symmetric, as OCCT's (the two handles' hashes sorted), and by identity, never by the points' contents
    public override int GetHashCode()
    {
        int a = myP1 is null ? 0 : RuntimeHelpers.GetHashCode(myP1);
        int b = myP2 is null ? 0 : RuntimeHelpers.GetHashCode(myP2);
        return a < b ? HashCode.Combine(a, b) : HashCode.Combine(b, a);
    }

    public static bool operator ==(StepToTopoDS_PointPair left, StepToTopoDS_PointPair right) => left.Equals(right);

    public static bool operator !=(StepToTopoDS_PointPair left, StepToTopoDS_PointPair right) => !left.Equals(right);
}
