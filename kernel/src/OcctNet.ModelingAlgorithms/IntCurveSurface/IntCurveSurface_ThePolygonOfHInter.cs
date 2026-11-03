// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_ThePolygonOfHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_ThePolygonOfHInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A polygon discretizing a curve for IntCurveSurface_HInter: its points, its box enlarged by its deflection, and
/// the curve parameters of its points.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>SetDeflectionOverEstimation</c> and <c>Closed(bool)</c> are public, as in
/// OCCT. At least five points are taken. OCCT's empty <c>Dump</c> is left out.
/// </remarks>
public sealed class IntCurveSurface_ThePolygonOfHInter
    : IIntCurveSurface_ThePolygon<IntCurveSurface_ThePolygonOfHInter, CurveAdaptorBase>
{
    private Box TheBnd = new();
    private double TheDeflection;
    private readonly int NbPntIn;
    private readonly NCollection_Array1<Point3D> ThePnts;
    private bool ClosedPolygon;
    private readonly double Binf;
    private readonly double Bsup;
    private NCollection_HArray1<double>? myParams;

    /// <summary>A polygon of <paramref name="NbPnt"/> points (at least 5) over the whole curve.</summary>
    public IntCurveSurface_ThePolygonOfHInter(CurveAdaptorBase Curve, int NbPnt)
    {
        ThePnts = new NCollection_Array1<Point3D>(1, (NbPnt < 5) ? 5 : NbPnt);
        ClosedPolygon = false;
        NbPntIn = (NbPnt < 5) ? 5 : NbPnt;
        Binf = IntCurveSurface_TheHCurveTool.FirstParameter(Curve);
        Bsup = IntCurveSurface_TheHCurveTool.LastParameter(Curve);
        Init(Curve);
    }

    /// <summary>A polygon of <paramref name="NbPnt"/> points (at least 5) between the parameters <paramref name="U1"/> and <paramref name="U2"/>.</summary>
    public IntCurveSurface_ThePolygonOfHInter(CurveAdaptorBase Curve, double U1, double U2, int NbPnt)
    {
        ThePnts = new NCollection_Array1<Point3D>(1, (NbPnt < 5) ? 5 : NbPnt);
        ClosedPolygon = false;
        Binf = U1;
        Bsup = U2;
        NbPntIn = (NbPnt < 5) ? 5 : NbPnt;
        Init(Curve);
    }

    /// <summary>A polygon through the points of the curve at the parameters <paramref name="Upars"/>.</summary>
    public IntCurveSurface_ThePolygonOfHInter(CurveAdaptorBase Curve, NCollection_Array1<double> Upars)
    {
        ThePnts = new NCollection_Array1<Point3D>(1, Upars.Length());
        ClosedPolygon = false;
        Binf = Upars[Upars.Lower()];
        Bsup = Upars[Upars.Upper()];
        NbPntIn = Upars.Length();
        Init(Curve, Upars);
    }

    static IntCurveSurface_ThePolygonOfHInter IIntCurveSurface_ThePolygon<IntCurveSurface_ThePolygonOfHInter, CurveAdaptorBase>.Create(
        in CurveAdaptorBase Curve, int NbPnt) => new(Curve, NbPnt);

    static IntCurveSurface_ThePolygonOfHInter IIntCurveSurface_ThePolygon<IntCurveSurface_ThePolygonOfHInter, CurveAdaptorBase>.Create(
        in CurveAdaptorBase Curve, double U1, double U2, int NbPnt) => new(Curve, U1, U2, NbPnt);

    static IntCurveSurface_ThePolygonOfHInter IIntCurveSurface_ThePolygon<IntCurveSurface_ThePolygonOfHInter, CurveAdaptorBase>.Create(
        in CurveAdaptorBase Curve, NCollection_Array1<double> Upars) => new(Curve, Upars);

    /// <summary>The bounding box of the polygon.</summary>
    public Box Bounding() => TheBnd;

    public double DeflectionOverEstimation() => TheDeflection;

    public void SetDeflectionOverEstimation(double x)
    {
        TheDeflection = x;
        TheBnd.Enlarge(TheDeflection);
    }

    public void Closed(bool flag) => ClosedPolygon = flag;

    public bool Closed() => ClosedPolygon; // -- check whether the Closed case can be handled

    /// <summary>The number of segments of the polyline.</summary>
    public int NbSegments() => NbPntIn - 1;

    /// <summary>The first point of the segment of range <paramref name="theIndex"/>.</summary>
    public Point3D BeginOfSeg(int theIndex) => ThePnts[theIndex];

    /// <summary>The last point of the segment of range <paramref name="theIndex"/>.</summary>
    public Point3D EndOfSeg(int theIndex) => ThePnts[theIndex + 1];

    /// <summary>The parameter on the curve of the first point of the polygon.</summary>
    public double InfParameter() => Binf;

    /// <summary>The parameter on the curve of the last point of the polygon.</summary>
    public double SupParameter() => Bsup;

    /// <summary>An approximation of the parameter on the curve, from the discretization of the curve.</summary>
    public double ApproxParamOnCurve(int Index, double ParamOnLine) =>
        IntCurveSurface_PolygonUtils.ApproxParamOnCurve(Index, ParamOnLine, Binf, Bsup, NbPntIn, myParams);

    private void Init(CurveAdaptorBase Curve)
    {
        IntCurveSurface_PolygonUtils.InitUniform<CurveAdaptorBase, IntCurveSurface_TheHCurveTool>(
            Curve, Binf, Bsup, NbPntIn, ThePnts, ref TheBnd, out TheDeflection);
        ClosedPolygon = false;
    }

    private void Init(CurveAdaptorBase Curve, NCollection_Array1<double> Upars)
    {
        IntCurveSurface_PolygonUtils.InitWithParams<CurveAdaptorBase, IntCurveSurface_TheHCurveTool>(
            Curve, Upars, NbPntIn, ThePnts, ref TheBnd, out TheDeflection, out NCollection_HArray1<double> aParams);
        myParams = aParams;
        ClosedPolygon = false;
    }
}
