// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_RestrictionParameters.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_RestrictionParameters.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_RestrictionParameters.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Healing.Custom;

/// <summary>
/// The parameters of <see cref="BSplineRestriction"/>: the global maximal degree and number of spans,
/// and which kinds of geometry to convert. Each accessor returns a reference, for reading and writing.
/// </summary>
/// <remarks>
/// A working object (decision 21). The defaults: degree 15, 10000 spans; surfaces of revolution and extrusion, offset
/// surfaces, 3D and 2D curves and offset curves converted, the surfaces segmented to their faces; planes, Bezier
/// surfaces and the elementary surfaces not converted. As in OCCT, <see cref="ConvertCylindricalSurf"/> and
/// <see cref="ConvertToroidalSurf"/> return each other's field; each always the same one, so nothing shows it.
/// </remarks>
internal class RestrictionParameters : Transient
{
    private int _gMaxDegree;
    private int _gMaxSeg;
    private bool _convPlane;
    private bool _convConicalSurf;
    private bool _convSphericalSurf;
    private bool _convCylindricalSurf;
    private bool _convToroidalSurf;
    private bool _convBezierSurf;
    private bool _convRevolSurf;
    private bool _convExtrSurf;
    private bool _convOffsetSurf;
    private bool _segmentSurfaceMode;
    private bool _convCurve3d;
    private bool _convOffsetCurv3d;
    private bool _convCurve2d;
    private bool _convOffsetCurv2d;

    /// <summary>The default parameters.</summary>
    public RestrictionParameters()
    {
        _gMaxSeg = 10000;
        _gMaxDegree = 15;

        _convPlane = false;
        // myConvElementarySurf = false;
        // conversion of elementary surfaces are off by default
        _convConicalSurf = false;
        _convSphericalSurf = false;
        _convCylindricalSurf = false;
        _convToroidalSurf = false;

        _convBezierSurf = false;
        _convRevolSurf = true;
        _convExtrSurf = true;
        _convOffsetSurf = true;
        _segmentSurfaceMode = true;
        _convCurve3d = true;
        _convOffsetCurv3d = true;
        _convCurve2d = true;
        _convOffsetCurv2d = true;
    }

    /// <summary>The maximal degree of approximation.</summary>
    public ref int GMaxDegree() => ref _gMaxDegree;

    /// <summary>The maximal number of spans of approximation.</summary>
    public ref int GMaxSeg() => ref _gMaxSeg;

    /// <summary>Whether planes are converted to B-spline surfaces.</summary>
    public ref bool ConvertPlane() => ref _convPlane;

    /// <summary>Whether Bezier surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertBezierSurf() => ref _convBezierSurf;

    /// <summary>Whether surfaces of revolution are converted to B-spline surfaces.</summary>
    public ref bool ConvertRevolutionSurf() => ref _convRevolSurf;

    /// <summary>Whether surfaces of linear extrusion are converted to B-spline surfaces.</summary>
    public ref bool ConvertExtrusionSurf() => ref _convExtrSurf;

    /// <summary>Whether offset surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertOffsetSurf() => ref _convOffsetSurf;

    /// <summary>Whether surfaces are approximated within the bounds of the face lying on them.</summary>
    public ref bool SegmentSurfaceMode() => ref _segmentSurfaceMode;

    /// <summary>Whether 3D curves are converted to B-spline curves.</summary>
    public ref bool ConvertCurve3d() => ref _convCurve3d;

    /// <summary>Whether offset 3D curves are converted to B-spline curves.</summary>
    public ref bool ConvertOffsetCurv3d() => ref _convOffsetCurv3d;

    /// <summary>Whether 2D curves are converted to B-spline curves.</summary>
    public ref bool ConvertCurve2d() => ref _convCurve2d;

    /// <summary>Whether offset 2D curves are converted to B-spline curves.</summary>
    public ref bool ConvertOffsetCurv2d() => ref _convOffsetCurv2d;

    /// <summary>Whether conical surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertConicalSurf() => ref _convConicalSurf;

    /// <summary>Whether spherical surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertSphericalSurf() => ref _convSphericalSurf;

    /// <summary>Whether toroidal surfaces are converted to B-spline surfaces.</summary>
    // OCCT: the cylinders' field, and ConvertCylindricalSurf the tori's (harmless: each accessor always uses the same one)
    public ref bool ConvertToroidalSurf() => ref _convCylindricalSurf;

    /// <summary>Whether cylindrical surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertCylindricalSurf() => ref _convToroidalSurf;
}
