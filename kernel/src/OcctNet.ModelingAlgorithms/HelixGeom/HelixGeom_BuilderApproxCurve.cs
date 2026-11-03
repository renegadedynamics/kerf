// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderApproxCurve.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderApproxCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Schematiq.Collections;

namespace Occt;

/// <summary>
/// The root of the helix builders that approximate a curve by B-splines: the approximation's parameters and
/// tolerance, the curves built, and the statuses.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Curves"/> is a publication point: it freezes the curves it returns
/// (decision 993); ported code that goes on working on them reads <see cref="InternalCurves"/>.
/// </remarks>
public abstract class HelixGeom_BuilderApproxCurve
{
    protected int myErrorStatus;
    protected int myWarningStatus;
    protected double myTolerance;
    protected Continuity myCont;
    protected int myMaxDegree;
    protected int myMaxSeg;
    protected double myTolReached;
    protected NCollection_Sequence<Curve3D> myCurves = new();

    /// <summary>Sets the default approximation parameters: tolerance 1e-4, C2, degree 8, 150 segments.</summary>
    protected HelixGeom_BuilderApproxCurve()
    {
        myErrorStatus = 0;
        myWarningStatus = 0;
        myTolerance = 0.0001;
        myCont = Continuity.C2;
        myMaxDegree = 8;
        myMaxSeg = 150;
        myTolReached = 99.0;
    }

    /// <summary>Sets the approximation parameters.</summary>
    public void SetApproxParameters(Continuity aCont, int aMaxDegree, int aMaxSeg)
    {
        myCont = aCont;
        myMaxDegree = aMaxDegree;
        myMaxSeg = aMaxSeg;
    }

    /// <summary>The approximation parameters.</summary>
    public void ApproxParameters(out Continuity aCont, out int aMaxDegree, out int aMaxSeg)
    {
        aCont = myCont;
        aMaxDegree = myMaxDegree;
        aMaxSeg = myMaxSeg;
    }

    /// <summary>Sets the approximation tolerance.</summary>
    public void SetTolerance(double aTolerance)
    {
        myTolerance = aTolerance;
    }

    /// <summary>The approximation tolerance.</summary>
    public double Tolerance() => myTolerance;

    /// <summary>The tolerance the approximation reached.</summary>
    public double ToleranceReached() => myTolReached;

    /// <summary>The B-spline curves representing the helix's coils, in order: a new list each call, of the same curves, frozen.</summary>
    public ISchematiqList<Curve3D> Curves()
    {
        var aCurves = new Curve3D[myCurves.Length()];
        for (int i = 1; i <= myCurves.Length(); i++)
        {
            Curve3D aCurve = myCurves.Value(i);
            aCurve.Freeze();
            aCurves[i - 1] = aCurve;
        }

        return PackedList.Own(aCurves);
    }

    /// <summary>OCCT's <c>Curves()</c>: the stored sequence, unfrozen, for ported code.</summary>
    internal NCollection_Sequence<Curve3D> InternalCurves() => myCurves;

    /// <summary>The error status: 0 on success, 2 if the approximation failed.</summary>
    public int ErrorStatus() => myErrorStatus;

    /// <summary>The warning status.</summary>
    public int WarningStatus() => myWarningStatus;

    /// <summary>Performs the calculations.</summary>
    public abstract void Perform();
}
