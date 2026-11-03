// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Curve.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The base class of the bisecting curves of the medial axis (Bisector_BisecAna, Bisector_BisecPC,
/// Bisector_BisecCC): a 2D curve that knows the parameter of a point on it, whether it is extended at its ends, and
/// its intervals of C1 continuity.
/// </summary>
public abstract class Bisector_Curve : Geom2d_Curve
{
    /// <summary>The parameter of the point P on the curve.</summary>
    public abstract double Parameter(in Point2D P);

    /// <summary>True if the curve is extended at its start.</summary>
    public abstract bool IsExtendAtStart();

    /// <summary>True if the curve is extended at its end.</summary>
    public abstract bool IsExtendAtEnd();

    /// <summary>If necessary, breaks the curve in intervals of continuity C1, and returns their number.</summary>
    public abstract int NbIntervals();

    /// <summary>The first parameter of the interval Index.</summary>
    public abstract double IntervalFirst(int Index);

    /// <summary>The last parameter of the interval Index.</summary>
    public abstract double IntervalLast(int Index);
}
