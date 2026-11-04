// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_PointPair.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_PointPair.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Geometry;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>
/// A pair of STEP points, unordered: (P1, P2) equals (P2, P1). The key of the edges a faceted brep's poly loops share.
/// Points are compared by reference, as OCCT compares handles.
/// </summary>
internal readonly struct PointPair : IEquatable<PointPair>
{
    private readonly StepCartesianPoint? _p1;
    private readonly StepCartesianPoint? _p2;

    public PointPair(StepCartesianPoint? p1, StepCartesianPoint? p2)
    {
        _p1 = p1;
        _p2 = p2;
    }

    public StepCartesianPoint? GetPoint1() => _p1;

    public StepCartesianPoint? GetPoint2() => _p2;

    public bool Equals(PointPair pointPair)
        => (ReferenceEquals(_p1, pointPair._p1) && ReferenceEquals(_p2, pointPair._p2))
           || (ReferenceEquals(_p1, pointPair._p2) && ReferenceEquals(_p2, pointPair._p1));

    public override bool Equals(object? obj) => obj is PointPair pair && Equals(pair);

    // Symmetric, as OCCT's (the two handles' hashes sorted), and by identity, never by the points' contents
    public override int GetHashCode()
    {
        int a = _p1 is null ? 0 : RuntimeHelpers.GetHashCode(_p1);
        int b = _p2 is null ? 0 : RuntimeHelpers.GetHashCode(_p2);
        return a < b ? HashCode.Combine(a, b) : HashCode.Combine(b, a);
    }

    public static bool operator ==(PointPair left, PointPair right) => left.Equals(right);

    public static bool operator !=(PointPair left, PointPair right) => !left.Equals(right);
}
