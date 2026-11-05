// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderApproxCurve.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderApproxCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Helix.Geometry;

/// <summary>
/// The root of the helix builders that approximate a curve by B-splines: the approximation's parameters and
/// tolerance, the curves built, and the statuses.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Curves"/> is a publication point: it freezes the curves it returns
/// (decision 993); ported code that goes on working on them reads <see cref="InternalCurves"/>.
/// </remarks>
internal abstract class BuilderApproxCurve
{
    protected int _errorStatus;
    protected int _warningStatus;
    protected double _tolerance;
    protected Continuity _cont;
    protected int _maxDegree;
    protected int _maxSeg;
    protected double _tolReached;
    protected NCollection_Sequence<Curve3D> _curves = new();

    /// <summary>Sets the default approximation parameters: tolerance 1e-4, C2, degree 8, 150 segments.</summary>
    protected BuilderApproxCurve()
    {
        _errorStatus = 0;
        _warningStatus = 0;
        _tolerance = 0.0001;
        _cont = Continuity.C2;
        _maxDegree = 8;
        _maxSeg = 150;
        _tolReached = 99.0;
    }

    /// <summary>Sets the approximation parameters.</summary>
    public void SetApproxParameters(Continuity aCont, int aMaxDegree, int aMaxSeg)
    {
        _cont = aCont;
        _maxDegree = aMaxDegree;
        _maxSeg = aMaxSeg;
    }

    /// <summary>The approximation parameters.</summary>
    public void ApproxParameters(out Continuity aCont, out int aMaxDegree, out int aMaxSeg)
    {
        aCont = _cont;
        aMaxDegree = _maxDegree;
        aMaxSeg = _maxSeg;
    }

    /// <summary>Sets the approximation tolerance.</summary>
    public void SetTolerance(double aTolerance)
    {
        _tolerance = aTolerance;
    }

    /// <summary>The approximation tolerance.</summary>
    public double Tolerance() => _tolerance;

    /// <summary>The tolerance the approximation reached.</summary>
    public double ToleranceReached() => _tolReached;

    /// <summary>The B-spline curves representing the helix's coils, in order: a new list each call, of the same curves, frozen.</summary>
    public IReadOnlyList<Curve3D> Curves()
    {
        var curves = new Curve3D[_curves.Length()];
        for (int i = 1; i <= _curves.Length(); i++)
        {
            Curve3D curve = _curves.Value(i);
            curve.Freeze();
            curves[i - 1] = curve;
        }

        return ImmutableArrays.Own(curves);
    }

    /// <summary>OCCT's <c>Curves()</c>: the stored sequence, unfrozen, for ported code.</summary>
    internal NCollection_Sequence<Curve3D> InternalCurves() => _curves;

    /// <summary>The error status: 0 on success, 2 if the approximation failed.</summary>
    public int ErrorStatus() => _errorStatus;

    /// <summary>The warning status.</summary>
    public int WarningStatus() => _warningStatus;

    /// <summary>Performs the calculations.</summary>
    public abstract void Perform();
}
