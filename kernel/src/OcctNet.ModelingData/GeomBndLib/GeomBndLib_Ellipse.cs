// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Ellipse.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a 3D ellipse (Geom_Ellipse), with analytical per-coordinate extrema. The static methods
/// accepting gp_Elips can be used directly without constructing a Geom_Ellipse.
/// </summary>
public sealed class GeomBndLib_Ellipse : GeomBndLib_Curve.ICurveEvaluator
{
    private readonly Geom_Ellipse myGeom;

    public GeomBndLib_Ellipse(Geom_Ellipse theEllipse)
    {
        myGeom = theEllipse;
    }

    public Geom_Ellipse Geometry() => myGeom;

    Geom_Curve? GeomBndLib_Curve.ICurveEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for full ellipse.</summary>
    public Box Box(double theTol) => Box(myGeom.Elips(), theTol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double theU1, double theU2, double theTol) => Box(myGeom.Elips(), theU1, theU2, theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for a full ellipse defined by gp_Elips.</summary>
    public static Box Box(in Ellipse3D theElips, double theTol)
    {
        Box aBox = new();
        double aMajR = theElips.MajorRadius();
        double aMinR = theElips.MinorRadius();
        Xyz aO = theElips.Location().XYZ();
        Xyz aXd = theElips.XAxis().Direction().XYZ();
        Xyz aYd = theElips.YAxis().Direction().XYZ();

        // For a full ellipse, compute per-coordinate analytical extrema.
        Span<double> aMin = stackalloc double[3];
        Span<double> aMax = stackalloc double[3];
        for (int k = 1; k <= 3; ++k)
        {
            double aXk = aXd.Coord(k);
            double aYk = aYd.Coord(k);
            double aAmp = Math.Sqrt(aMajR * aMajR * aXk * aXk + aMinR * aMinR * aYk * aYk);
            aMin[k - 1] = aO.Coord(k) - aAmp;
            aMax[k - 1] = aO.Coord(k) + aAmp;
        }
        aBox.Update(aMin[0], aMin[1], aMin[2], aMax[0], aMax[1], aMax[2]);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute bounding box for an ellipse arc [theU1, theU2] defined by gp_Elips.</summary>
    public static Box Box(in Ellipse3D theElips, double theU1, double theU2, double theTol)
    {
        Box aBox = new();
        double aMajR = theElips.MajorRadius();
        double aMinR = theElips.MinorRadius();
        Xyz aO = theElips.Location().XYZ();
        Xyz aXd = theElips.XAxis().Direction().XYZ();
        Xyz aYd = theElips.YAxis().Direction().XYZ();

        double aPeriod = 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (theU2 - theU1 >= aPeriod)
        {
            return Box(theElips, theTol);
        }

        double aU1 = theU1, aU2 = theU2;
        double aTol = Epsilon(1.0);
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, aTol, ref aU1, ref aU2);

        // Add arc endpoints.
        Frame3D aPos = theElips.Position();
        aBox.Add(global::Kerf.Engine.Geometry.ElCLib.EllipseValue(aU1, aPos, aMajR, aMinR));
        aBox.Add(global::Kerf.Engine.Geometry.ElCLib.EllipseValue(aU2, aPos, aMajR, aMinR));

        // For each coordinate, check if the extremal parameter lies within the arc.
        for (int k = 1; k <= 3; ++k)
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

            double aValMin = aMajR * Math.Cos(aTExtrMin) * aXk + aMinR * Math.Sin(aTExtrMin) * aYk + aO.Coord(k);
            double aValMax = aMajR * Math.Cos(aTExtrMax) * aXk + aMinR * Math.Sin(aTExtrMax) * aYk + aO.Coord(k);
            if (aValMin > aValMax)
            {
                (aTExtrMin, aTExtrMax) = (aTExtrMax, aTExtrMin);
            }

            double aTk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aTExtrMin, aU1, aU1 + 2.0 * M_PI);
            if (aTk >= aU1 && aTk <= aU2)
            {
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.EllipseValue(aTExtrMin, aPos, aMajR, aMinR));
            }
            aTk = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aTExtrMax, aU1, aU1 + 2.0 * M_PI);
            if (aTk >= aU1 && aTk <= aU2)
            {
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.EllipseValue(aTExtrMax, aPos, aMajR, aMinR));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
