// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelix.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Helix.Geometry;

/// <summary>
/// Builds a helix about an arbitrary axis, approximated by B-spline curves, one per full turn and one for the rest.
/// </summary>
internal sealed class GeometryBuilderHelix : BuilderHelixGen
{
    private Frame3D _position = new();

    /// <summary>A builder with the default parameters, about the global Z axis.</summary>
    public GeometryBuilderHelix()
    {
    }

    /// <summary>Sets the coordinate system of the helix (its Z axis is the helix's axis).</summary>
    public void SetPosition(in Frame3D aAx2)
    {
        _position = aAx2;
    }

    /// <summary>The coordinate system of the helix.</summary>
    public Frame3D Position() => _position;

    /// <summary>
    /// Builds the curves. <see cref="BuilderApproxCurve.ErrorStatus"/> is 2 if an approximation fails.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The helix parameters are invalid.</exception>
    public override void Perform()
    {
        _errorStatus = 0;
        _warningStatus = 0;

        // Initialize variables for helix segmentation
        int iErr, n, i, nbC;
        double twoPI, dT, t1x, t2x, tr;
        BuilderHelixCoil bhc = new();

        // Initialize result containers
        _curves.Clear();
        _tolReached = -1.0;
        twoPI = 2.0 * M_PI;
        bhc.SetTolerance(_tolerance);
        bhc.SetApproxParameters(_cont, _maxDegree, _maxSeg);

        // Determine number of full turns for segmentation
        dT = _t2 - _t1;
        n = (int)(dT / twoPI);
        if (n == 0)
        {
            bhc.SetCurveParameters(_t1, _t2, _pitch, _rStart, _taperAngle, _isClockWise);
            bhc.Perform();
            iErr = bhc.ErrorStatus();
            if (iErr != 0)
            {
                _errorStatus = 2;
                return;
            }
            NCollection_Sequence<Curve3D> sc = bhc.InternalCurves();
            Curve3D c = sc[1];
            _curves.Append(c);
            _tolReached = bhc.ToleranceReached();
        }
        else
        {
            // Case: helix spans multiple full turns - process in segments
            bool bIsCylindrical;
            double tolAngle;

            tolAngle = 1.0e-4;
            bIsCylindrical = Math.Abs(_taperAngle) < tolAngle;
            t1x = _t1;
            t2x = _t1 + twoPI;
            for (i = 1; i <= n; ++i)
            {
                if (i > 1 && bIsCylindrical)
                {
                    // Optimization: for cylindrical helixes, reuse first coil with translation
                    Curve3D ci;
                    Point3D p1, pi = new();

                    Curve3D c1 = _curves[1];
                    c1.D0(c1.FirstParameter, out p1);
                    pi = pi.WithCoord(p1.X, p1.Y, p1.Z + (i - 1) * _pitch);
                    ci = (Curve3D)c1.Translated(p1, pi);

                    _curves.Append(ci);
                    t1x = t2x;
                    t2x = t1x + twoPI;
                    // Skip to next iteration for optimization
                    continue;
                }

                bhc.SetCurveParameters(t1x, t2x, _pitch, _rStart, _taperAngle, _isClockWise);
                // Perform approximation for this segment
                bhc.Perform();
                iErr = bhc.ErrorStatus();
                if (iErr != 0)
                {
                    _errorStatus = 2;
                    return;
                }
                // Extract approximated curves from builder
                NCollection_Sequence<Curve3D> sc = bhc.InternalCurves();
                Curve3D c = sc[1];
                _curves.Append(c);
                tr = bhc.ToleranceReached();
                if (tr > _tolReached)
                {
                    _tolReached = tr;
                }
                // Move to next segment parameters
                t1x = t2x;
                t2x = t1x + twoPI;
            } // for (i=1; i<=aN; ++i) {
            // Handle remaining partial turn if any
            t2x = _t2;
            double eps = 1.0e-7 * twoPI;
            if (Math.Abs(t2x - t1x) > eps)
            {
                bhc.SetCurveParameters(t1x, t2x, _pitch, _rStart, _taperAngle, _isClockWise);
                bhc.Perform();
                iErr = bhc.ErrorStatus();
                if (iErr != 0)
                {
                    _errorStatus = 2;
                    return;
                }
                // Extract curves from the final partial segment
                NCollection_Sequence<Curve3D> sc = bhc.InternalCurves();
                Curve3D c = sc[1];
                _curves.Append(c);
                tr = bhc.ToleranceReached();
                if (tr > _tolReached)
                {
                    _tolReached = tr;
                }
            }
        }
        // Apply coordinate system transformation to all curves
        // Transformation
        Transform3D trsf = new();
        CoordinateSystem ax3 = new(), ax3x = new(_position);
        // Set transformation from standard coordinate system to helix position
        trsf = trsf.WithDisplacement(ax3, ax3x);
        // Apply transformation to all generated curves (new and unfrozen: the coil builder's own, or translated copies)
        nbC = _curves.Length();
        for (i = 1; i <= nbC; ++i)
        {
            Curve3D c = _curves[i];
            c.Transform(trsf);
        }
    }
}
