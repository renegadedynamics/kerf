// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelixCoil.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_BuilderHelixCoil.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Helix.Geometry;

/// <summary>Builds one helix coil about the Z axis, approximated by a B-spline curve.</summary>
public sealed class BuilderHelixCoil : BuilderHelixGen
{
    /// <summary>A builder with the default parameters.</summary>
    public BuilderHelixCoil()
    {
        _t1 = 0.0;
        _t2 = 2.0 * M_PI;
        _pitch = 1.0;
        _rStart = 1.0;
        _taperAngle = 0.0;
        _isClockWise = true;
    }

    /// <summary>
    /// Approximates the coil. <see cref="BuilderApproxCurve.ErrorStatus"/> is 2 if the approximation fails.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The helix parameters are invalid (see <see cref="HelixCurve.Load(double, double, double, double, double, bool)"/>).</exception>
    public override void Perform()
    {
        _errorStatus = 0;
        _warningStatus = 0;
        // Initialize variables for curve approximation
        int iErr;
        HelixCurve adaptor = new();
        HelixCurve hAdaptor;
        BSplineCurve3D? bc = null;
        // Clear previous results and setup helix adaptor
        _curves.Clear();
        // Load helix parameters into the adaptor
        adaptor.Load(_t1, _t2, _pitch, _rStart, _taperAngle, _isClockWise);
        hAdaptor = new HelixCurve(adaptor);
        // Perform B-spline approximation of the helix curve
        iErr = GeometryTools.ApprCurve3D(hAdaptor,
                                           _tolerance,
                                           _cont,
                                           _maxSeg,
                                           _maxDegree,
                                           ref bc,
                                           ref _tolReached);
        if (iErr != 0)
        {
            _errorStatus = 2;
        }
        else
        {
            _curves.Append(bc!);
        }
    }
}
