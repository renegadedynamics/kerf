// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Curve.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>
/// The base class of the bisecting curves of the medial axis (Bisector_BisecAna, Bisector_BisecPC,
/// Bisector_BisecCC): a 2D curve that knows the parameter of a point on it, whether it is extended at its ends, and
/// its intervals of C1 continuity.
/// </summary>
public abstract class BisectorCurve : Curve2D
{
    /// <summary>The parameter of the point P on the curve.</summary>
    public abstract double Parameter(in Point2D p);

    /// <summary>True if the curve is extended at its start.</summary>
    public abstract bool IsExtendAtStart { get; }

    /// <summary>True if the curve is extended at its end.</summary>
    public abstract bool IsExtendAtEnd { get; }

    /// <summary>If necessary, breaks the curve in intervals of continuity C1, and returns their number.</summary>
    public abstract int IntervalCount { get; }

    /// <summary>The first parameter of the interval Index.</summary>
    public abstract double IntervalFirst(int index);

    /// <summary>The last parameter of the interval Index.</summary>
    public abstract double IntervalLast(int index);
}
