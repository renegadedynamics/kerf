// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Circle.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Circle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a 3D circle (Geom_Circle), with analytical per-coordinate extrema. The static methods
/// accepting gp_Circ can be used directly without constructing a Geom_Circle.
/// </summary>
public sealed class GeomBndLib_Circle : GeomBndLib_Curve.ICurveEvaluator
{
    private readonly Geom_Circle myGeom;

    public GeomBndLib_Circle(Geom_Circle theCircle)
    {
        myGeom = theCircle;
    }

    public Geom_Circle Geometry() => myGeom;

    Geom_Curve? GeomBndLib_Curve.ICurveEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for full circle.</summary>
    public Bnd_Box Box(double theTol) => Box(myGeom.Circ(), theTol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box Box(double theU1, double theU2, double theTol) => Box(myGeom.Circ(), theU1, theU2, theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Bnd_Box BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for a full circle defined by gp_Circ.</summary>
    public static Bnd_Box Box(in gp_Circ theCirc, double theTol)
    {
        Bnd_Box aBox = new();
        double aR = theCirc.Radius();
        gp_XYZ aO = theCirc.Location().XYZ();
        gp_XYZ aXd = theCirc.XAxis().Direction().XYZ();
        gp_XYZ aYd = theCirc.YAxis().Direction().XYZ();

        // For a full circle, compute per-coordinate analytical extrema.
        Span<double> aMin = stackalloc double[3];
        Span<double> aMax = stackalloc double[3];
        for (int k = 1; k <= 3; ++k)
        {
            double aXk = aXd.Coord(k);
            double aYk = aYd.Coord(k);
            double aAmp = Math.Sqrt(aR * aR * aXk * aXk + aR * aR * aYk * aYk);
            aMin[k - 1] = aO.Coord(k) - aAmp;
            aMax[k - 1] = aO.Coord(k) + aAmp;
        }
        aBox.Update(aMin[0], aMin[1], aMin[2], aMax[0], aMax[1], aMax[2]);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute bounding box for a circle arc [theU1, theU2] defined by gp_Circ.</summary>
    public static Bnd_Box Box(in gp_Circ theCirc, double theU1, double theU2, double theTol)
    {
        Bnd_Box aBox = new();
        double aR = theCirc.Radius();
        gp_XYZ aO = theCirc.Location().XYZ();
        gp_XYZ aXd = theCirc.XAxis().Direction().XYZ();
        gp_XYZ aYd = theCirc.YAxis().Direction().XYZ();

        double aPeriod = 2.0 * M_PI - Precision.PConfusion();
        if (theU2 - theU1 >= aPeriod)
        {
            return Box(theCirc, theTol);
        }

        double aU1 = theU1, aU2 = theU2;
        double aTol = Epsilon(1.0);
        ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, aTol, ref aU1, ref aU2);

        // Add arc endpoints.
        gp_Ax2 aPos = theCirc.Position();
        aBox.Add(ElCLib.CircleValue(aU1, aPos, aR));
        aBox.Add(ElCLib.CircleValue(aU2, aPos, aR));

        // For each coordinate, check if the extremal parameter lies within the arc.
        for (int k = 1; k <= 3; ++k)
        {
            double aXk = aXd.Coord(k);
            double aYk = aYd.Coord(k);

            // Extremal parameter for min: tan(t) = (R * Yk) / (R * Xk)
            double aTExtrMin;
            if (Math.Abs(aXk) > gp.Resolution())
            {
                aTExtrMin = Math.Atan(aYk / aXk);
                aTExtrMin = ElCLib.InPeriod(aTExtrMin, 0.0, 2.0 * M_PI);
            }
            else
            {
                aTExtrMin = M_PI / 2.0;
            }
            double aTExtrMax = aTExtrMin <= M_PI ? aTExtrMin + M_PI : aTExtrMin - M_PI;

            double aValMin = aR * Math.Cos(aTExtrMin) * aXk + aR * Math.Sin(aTExtrMin) * aYk + aO.Coord(k);
            double aValMax = aR * Math.Cos(aTExtrMax) * aXk + aR * Math.Sin(aTExtrMax) * aYk + aO.Coord(k);
            if (aValMin > aValMax)
            {
                (aValMin, aValMax) = (aValMax, aValMin);
                (aTExtrMin, aTExtrMax) = (aTExtrMax, aTExtrMin);
            }

            double aTk = ElCLib.InPeriod(aTExtrMin, aU1, aU1 + 2.0 * M_PI);
            if (aTk >= aU1 && aTk <= aU2)
            {
                aBox.Add(ElCLib.CircleValue(aTExtrMin, aPos, aR));
            }
            aTk = ElCLib.InPeriod(aTExtrMax, aU1, aU1 + 2.0 * M_PI);
            if (aTk >= aU1 && aTk <= aU2)
            {
                aBox.Add(ElCLib.CircleValue(aTExtrMax, aPos, aR));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
