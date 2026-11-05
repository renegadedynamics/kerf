// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolygonOfInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolygonOfInterCSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// A polygon discretizing a curve for HLRBRep_InterCSurf: its points, its box enlarged by its deflection, and
/// the curve parameters of its points.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>SetDeflectionOverEstimation</c> and <c>Closed(bool)</c> are public, as in
/// OCCT. At least five points are taken. OCCT's empty <c>Dump</c> is left out.
/// </remarks>
internal sealed class ThePolygonOfInterCSurf
    : IThePolygon<ThePolygonOfInterCSurf, Line3D>
{
    private Box _theBnd = new();
    private double _theDeflection;
    private readonly int _nbPntIn;
    private readonly NCollection_Array1<Point3D> _thePnts;
    private bool _closedPolygon;
    private readonly double _binf;
    private readonly double _bsup;
    private NCollection_HArray1<double>? _params;

    /// <summary>A polygon of <paramref name="nbPnt"/> points (at least 5) over the whole curve.</summary>
    public ThePolygonOfInterCSurf(Line3D curve, int nbPnt)
    {
        _thePnts = new NCollection_Array1<Point3D>(1, (nbPnt < 5) ? 5 : nbPnt);
        _closedPolygon = false;
        _nbPntIn = (nbPnt < 5) ? 5 : nbPnt;
        _binf = HiddenLineLineTool.FirstParameter(curve);
        _bsup = HiddenLineLineTool.LastParameter(curve);
        Init(curve);
    }

    /// <summary>A polygon of <paramref name="nbPnt"/> points (at least 5) between the parameters <paramref name="u1"/> and <paramref name="u2"/>.</summary>
    public ThePolygonOfInterCSurf(Line3D curve, double u1, double u2, int nbPnt)
    {
        _thePnts = new NCollection_Array1<Point3D>(1, (nbPnt < 5) ? 5 : nbPnt);
        _closedPolygon = false;
        _binf = u1;
        _bsup = u2;
        _nbPntIn = (nbPnt < 5) ? 5 : nbPnt;
        Init(curve);
    }

    /// <summary>A polygon through the points of the curve at the parameters <paramref name="upars"/>.</summary>
    public ThePolygonOfInterCSurf(Line3D curve, NCollection_Array1<double> upars)
    {
        _thePnts = new NCollection_Array1<Point3D>(1, upars.Length());
        _closedPolygon = false;
        _binf = upars[upars.Lower()];
        _bsup = upars[upars.Upper()];
        _nbPntIn = upars.Length();
        Init(curve, upars);
    }

    static ThePolygonOfInterCSurf IThePolygon<ThePolygonOfInterCSurf, Line3D>.Create(
        in Line3D curve, int nbPnt) => new(curve, nbPnt);

    static ThePolygonOfInterCSurf IThePolygon<ThePolygonOfInterCSurf, Line3D>.Create(
        in Line3D curve, double u1, double u2, int nbPnt) => new(curve, u1, u2, nbPnt);

    static ThePolygonOfInterCSurf IThePolygon<ThePolygonOfInterCSurf, Line3D>.Create(
        in Line3D curve, NCollection_Array1<double> upars) => new(curve, upars);

    /// <summary>The bounding box of the polygon.</summary>
    public Box Bounding() => _theBnd;

    public double DeflectionOverEstimation() => _theDeflection;

    public void SetDeflectionOverEstimation(double x)
    {
        _theDeflection = x;
        _theBnd.Enlarge(_theDeflection);
    }

    public void Closed(bool flag) => _closedPolygon = flag;

    public bool Closed() => _closedPolygon; // -- check whether the Closed case can be handled

    /// <summary>The number of segments of the polyline.</summary>
    public int SegmentCount => _nbPntIn - 1;

    /// <summary>The first point of the segment of range <paramref name="index"/>.</summary>
    public Point3D BeginOfSeg(int index) => _thePnts[index];

    /// <summary>The last point of the segment of range <paramref name="index"/>.</summary>
    public Point3D EndOfSeg(int index) => _thePnts[index + 1];

    /// <summary>The parameter on the curve of the first point of the polygon.</summary>
    public double InfParameter() => _binf;

    /// <summary>The parameter on the curve of the last point of the polygon.</summary>
    public double SupParameter() => _bsup;

    /// <summary>An approximation of the parameter on the curve, from the discretization of the curve.</summary>
    public double ApproxParamOnCurve(int index, double paramOnLine) =>
        PolygonUtils.ApproxParamOnCurve(index, paramOnLine, _binf, _bsup, _nbPntIn, _params);

    private void Init(Line3D curve)
    {
        PolygonUtils.InitUniform<Line3D, HiddenLineLineTool>(
            curve, _binf, _bsup, _nbPntIn, _thePnts, ref _theBnd, out _theDeflection);
        _closedPolygon = false;
    }

    private void Init(Line3D curve, NCollection_Array1<double> upars)
    {
        PolygonUtils.InitWithParams<Line3D, HiddenLineLineTool>(
            curve, upars, _nbPntIn, _thePnts, ref _theBnd, out _theDeflection, out NCollection_HArray1<double> aParams);
        _params = aParams;
        _closedPolygon = false;
    }
}
