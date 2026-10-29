// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierCurve2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierCurve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a 2D Bezier curve (Geom2d_BezierCurve): the poles of the curve, segmented to the range
/// when it is a strict sub-range.
/// </summary>
public sealed class GeomBndLib_BezierCurve2d : GeomBndLib_Curve2d.ICurve2dEvaluator
{
    private readonly Geom2d_BezierCurve myGeom;

    public GeomBndLib_BezierCurve2d(Geom2d_BezierCurve theCurve)
    {
        myGeom = theCurve;
    }

    public Geom2d_BezierCurve Geometry() => myGeom;

    Geom2d_Curve? GeomBndLib_Curve2d.ICurve2dEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Bnd_Box2d Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box2d Box(double theU1, double theU2, double theTol)
    {
        Geom2d_BezierCurve aCurve = myGeom;
        double aU1 = aCurve.FirstParameter();
        double aU2 = aCurve.LastParameter();
        double aTrim1 = Max(theU1, aU1);
        double aTrim2 = Min(theU2, aU2);
        double anEps = Precision.PConfusion();
        if (Math.Abs(aU1 - aTrim1) > anEps || Math.Abs(aU2 - aTrim2) > anEps)
        {
            Geom2d_Geometry aCopy = aCurve.Copy();
            aCurve = (Geom2d_BezierCurve)aCopy;
            aCurve.Segment(aTrim1, aTrim2);
        }

        Bnd_Box2d aBox = new();
        for (int anIdx = 1; anIdx <= aCurve.NbPoles(); ++anIdx)
        {
            aBox.Add(aCurve.Pole(anIdx));
        }
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    public Bnd_Box2d BoxOptimal(double theU1, double theU2, double theTol)
    {
        Geom2dAdaptor_Curve aGACurve = new(myGeom);
        return GeomBndLib_SplineHelpers.CurveBoxOptimal(aGACurve, theU1, theU2, theTol);
    }
}
