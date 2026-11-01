// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_RestrictionParameters.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_RestrictionParameters.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_RestrictionParameters.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The parameters of <see cref="ShapeCustom_BSplineRestriction"/>: the global maximal degree and number of spans,
/// and which kinds of geometry to convert. Each accessor returns a reference, for reading and writing.
/// </summary>
/// <remarks>
/// A working object (decision 21). The defaults: degree 15, 10000 spans; surfaces of revolution and extrusion, offset
/// surfaces, 3D and 2D curves and offset curves converted, the surfaces segmented to their faces; planes, Bezier
/// surfaces and the elementary surfaces not converted. As in OCCT, <see cref="ConvertCylindricalSurf"/> and
/// <see cref="ConvertToroidalSurf"/> return each other's field; each always the same one, so nothing shows it.
/// </remarks>
public class ShapeCustom_RestrictionParameters : Standard_Transient
{
    private int myGMaxDegree;
    private int myGMaxSeg;
    private bool myConvPlane;
    private bool myConvConicalSurf;
    private bool myConvSphericalSurf;
    private bool myConvCylindricalSurf;
    private bool myConvToroidalSurf;
    private bool myConvBezierSurf;
    private bool myConvRevolSurf;
    private bool myConvExtrSurf;
    private bool myConvOffsetSurf;
    private bool mySegmentSurfaceMode;
    private bool myConvCurve3d;
    private bool myConvOffsetCurv3d;
    private bool myConvCurve2d;
    private bool myConvOffsetCurv2d;

    /// <summary>The default parameters.</summary>
    public ShapeCustom_RestrictionParameters()
    {
        myGMaxSeg = 10000;
        myGMaxDegree = 15;

        myConvPlane = false;
        // myConvElementarySurf = false;
        // conversion of elementary surfaces are off by default
        myConvConicalSurf = false;
        myConvSphericalSurf = false;
        myConvCylindricalSurf = false;
        myConvToroidalSurf = false;

        myConvBezierSurf = false;
        myConvRevolSurf = true;
        myConvExtrSurf = true;
        myConvOffsetSurf = true;
        mySegmentSurfaceMode = true;
        myConvCurve3d = true;
        myConvOffsetCurv3d = true;
        myConvCurve2d = true;
        myConvOffsetCurv2d = true;
    }

    /// <summary>The maximal degree of approximation.</summary>
    public ref int GMaxDegree() => ref myGMaxDegree;

    /// <summary>The maximal number of spans of approximation.</summary>
    public ref int GMaxSeg() => ref myGMaxSeg;

    /// <summary>Whether planes are converted to B-spline surfaces.</summary>
    public ref bool ConvertPlane() => ref myConvPlane;

    /// <summary>Whether Bezier surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertBezierSurf() => ref myConvBezierSurf;

    /// <summary>Whether surfaces of revolution are converted to B-spline surfaces.</summary>
    public ref bool ConvertRevolutionSurf() => ref myConvRevolSurf;

    /// <summary>Whether surfaces of linear extrusion are converted to B-spline surfaces.</summary>
    public ref bool ConvertExtrusionSurf() => ref myConvExtrSurf;

    /// <summary>Whether offset surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertOffsetSurf() => ref myConvOffsetSurf;

    /// <summary>Whether surfaces are approximated within the bounds of the face lying on them.</summary>
    public ref bool SegmentSurfaceMode() => ref mySegmentSurfaceMode;

    /// <summary>Whether 3D curves are converted to B-spline curves.</summary>
    public ref bool ConvertCurve3d() => ref myConvCurve3d;

    /// <summary>Whether offset 3D curves are converted to B-spline curves.</summary>
    public ref bool ConvertOffsetCurv3d() => ref myConvOffsetCurv3d;

    /// <summary>Whether 2D curves are converted to B-spline curves.</summary>
    public ref bool ConvertCurve2d() => ref myConvCurve2d;

    /// <summary>Whether offset 2D curves are converted to B-spline curves.</summary>
    public ref bool ConvertOffsetCurv2d() => ref myConvOffsetCurv2d;

    /// <summary>Whether conical surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertConicalSurf() => ref myConvConicalSurf;

    /// <summary>Whether spherical surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertSphericalSurf() => ref myConvSphericalSurf;

    /// <summary>Whether toroidal surfaces are converted to B-spline surfaces.</summary>
    // OCCT: the cylinders' field, and ConvertCylindricalSurf the tori's (harmless: each accessor always uses the same one)
    public ref bool ConvertToroidalSurf() => ref myConvCylindricalSurf;

    /// <summary>Whether cylindrical surfaces are converted to B-spline surfaces.</summary>
    public ref bool ConvertCylindricalSurf() => ref myConvToroidalSurf;
}
