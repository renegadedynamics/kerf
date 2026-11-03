// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

namespace Occt;

/// <summary>
/// Computes bounding box for a 3D Bezier curve (Geom_BezierCurve): the poles' convex hull reduced by sampling with a
/// deflection estimate.
/// </summary>
public sealed class GeomBndLib_BezierCurve : GeomBndLib_Curve.ICurveEvaluator
{
    private readonly Geom_BezierCurve myGeom;

    public GeomBndLib_BezierCurve(Geom_BezierCurve theCurve)
    {
        myGeom = theCurve;
    }

    public Geom_BezierCurve Geometry() => myGeom;

    Geom_Curve? GeomBndLib_Curve.ICurveEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double theU1, double theU2, double theTol)
    {
        const double aWeakness = 1.5;
        GeomAdaptor_Curve aGACurve = new(myGeom);
        Box aSampledBox = new();
        double aDeflection = GeomBndLib_SplineHelpers.FillBox(ref aSampledBox, aGACurve, theU1, theU2, myGeom.Degree());
        aSampledBox.Enlarge(aWeakness * aDeflection);
        Box aBox = new();
        GeomBndLib_SplineHelpers.ReduceSplineBox(myGeom.InternalPoles(), aSampledBox, ref aBox);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    public Box BoxOptimal(double theU1, double theU2, double theTol)
    {
        GeomAdaptor_Curve aGACurve = new(myGeom);
        return GeomBndLib_SplineHelpers.CurveBoxOptimal(aGACurve, theU1, theU2, theTol);
    }
}
