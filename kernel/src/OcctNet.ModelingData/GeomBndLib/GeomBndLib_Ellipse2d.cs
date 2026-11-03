// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Ellipse2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Ellipse2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a 2D ellipse (Geom2d_Ellipse), with analytical per-coordinate extrema. The static
/// methods accepting gp_Elips2d can be used directly without constructing a Geom2d_Ellipse.
/// </summary>
public sealed class GeomBndLib_Ellipse2d : GeomBndLib_Curve2d.ICurve2dEvaluator
{
    private readonly Geom2d_Ellipse myGeom;

    public GeomBndLib_Ellipse2d(Geom2d_Ellipse theEllipse)
    {
        myGeom = theEllipse;
    }

    public Geom2d_Ellipse Geometry() => myGeom;

    Geom2d_Curve? GeomBndLib_Curve2d.ICurve2dEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for full ellipse.</summary>
    public Box2d Box(double theTol) => Box(myGeom.Elips2d(), theTol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double theU1, double theU2, double theTol) => Box(myGeom.Elips2d(), theU1, theU2, theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box2d BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for a full ellipse defined by gp_Elips2d.</summary>
    public static Box2d Box(in Ellipse2D theElips, double theTol)
    {
        Box2d aBox = new();
        double aMajR = theElips.MajorRadius();
        double aMinR = theElips.MinorRadius();
        Xy aO = theElips.Location().XY();
        Xy aXd = theElips.XAxis().Direction().XY();
        Xy aYd = theElips.YAxis().Direction().XY();

        Span<double> aMin = stackalloc double[2];
        Span<double> aMax = stackalloc double[2];
        for (int k = 1; k <= 2; ++k)
        {
            double aXk = aXd.Coord(k);
            double aYk = aYd.Coord(k);
            double aAmp = Math.Sqrt(aMajR * aMajR * aXk * aXk + aMinR * aMinR * aYk * aYk);
            aMin[k - 1] = aO.Coord(k) - aAmp;
            aMax[k - 1] = aO.Coord(k) + aAmp;
        }
        aBox.Update(aMin[0], aMin[1], aMax[0], aMax[1]);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute bounding box for an ellipse arc [theU1, theU2] defined by gp_Elips2d.</summary>
    public static Box2d Box(in Ellipse2D theElips, double theU1, double theU2, double theTol)
    {
        Box2d aBox = new();
        double aMajR = theElips.MajorRadius();
        double aMinR = theElips.MinorRadius();
        Xy aO = theElips.Location().XY();
        Xy aXd = theElips.XAxis().Direction().XY();
        Xy aYd = theElips.YAxis().Direction().XY();

        if (theU2 - theU1 >= 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return Box(theElips, theTol);
        }

        // Add arc endpoints (OCCT's lambda aEval).
        Point2D aEval(double theT) => new(aO + aMajR * Math.Cos(theT) * aXd + aMinR * Math.Sin(theT) * aYd);

        aBox.Add(aEval(theU1));
        aBox.Add(aEval(theU2));

        // For each coordinate, check if the extremal parameter lies within the arc.
        for (int k = 1; k <= 2; ++k)
        {
            double aXk = aXd.Coord(k);
            double aYk = aYd.Coord(k);

            double aTExtrMin;
            if (Math.Abs(aXk) > Constants.Resolution())
            {
                aTExtrMin = Math.Atan((aMinR * aYk) / (aMajR * aXk));
                aTExtrMin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aTExtrMin, 0.0, 2.0 * M_PI);
            }
            else
            {
                aTExtrMin = M_PI / 2.0;
            }
            double aTExtrMax = aTExtrMin <= M_PI ? aTExtrMin + M_PI : aTExtrMin - M_PI;

            double aTk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aTExtrMin, theU1, theU1 + 2.0 * M_PI);
            if (aTk >= theU1 && aTk <= theU2)
            {
                aBox.Add(aEval(aTExtrMin));
            }
            aTk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aTExtrMax, theU1, theU1 + 2.0 * M_PI);
            if (aTk >= theU1 && aTk <= theU2)
            {
                aBox.Add(aEval(aTExtrMax));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
