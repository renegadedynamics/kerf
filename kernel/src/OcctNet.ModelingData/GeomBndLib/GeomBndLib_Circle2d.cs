// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Circle2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Circle2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a 2D circle (Geom2d_Circle), with analytical per-coordinate extrema. The static methods
/// accepting gp_Circ2d can be used directly without constructing a Geom2d_Circle.
/// </summary>
public sealed class GeomBndLib_Circle2d : GeomBndLib_Curve2d.ICurve2dEvaluator
{
    private readonly Geom2d_Circle myGeom;

    public GeomBndLib_Circle2d(Geom2d_Circle theCircle)
    {
        myGeom = theCircle;
    }

    public Geom2d_Circle Geometry() => myGeom;

    Geom2d_Curve? GeomBndLib_Curve2d.ICurve2dEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for full circle.</summary>
    public Bnd_Box2d Box(double theTol) => Box(myGeom.Circ2d(), theTol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box2d Box(double theU1, double theU2, double theTol) => Box(myGeom.Circ2d(), theU1, theU2, theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Bnd_Box2d BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for a full circle defined by gp_Circ2d.</summary>
    public static Bnd_Box2d Box(in gp_Circ2d theCirc, double theTol)
    {
        Bnd_Box2d aBox = new();
        double aR = theCirc.Radius();
        gp_XY aO = theCirc.Location().XY();
        gp_XY aXd = theCirc.XAxis().Direction().XY();
        gp_XY aYd = theCirc.YAxis().Direction().XY();

        // For a full circle, compute per-coordinate analytical extrema.
        Span<double> aMin = stackalloc double[2];
        Span<double> aMax = stackalloc double[2];
        for (int k = 1; k <= 2; ++k)
        {
            double aXk = aXd.Coord(k);
            double aYk = aYd.Coord(k);
            double aAmp = Math.Sqrt(aR * aR * aXk * aXk + aR * aR * aYk * aYk);
            aMin[k - 1] = aO.Coord(k) - aAmp;
            aMax[k - 1] = aO.Coord(k) + aAmp;
        }
        aBox.Update(aMin[0], aMin[1], aMax[0], aMax[1]);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute bounding box for a circle arc [theU1, theU2] defined by gp_Circ2d.</summary>
    public static Bnd_Box2d Box(in gp_Circ2d theCirc, double theU1, double theU2, double theTol)
    {
        Bnd_Box2d aBox = new();
        double aR = theCirc.Radius();
        gp_XY aO = theCirc.Location().XY();
        gp_XY aXd = theCirc.XAxis().Direction().XY();
        gp_XY aYd = theCirc.YAxis().Direction().XY();

        if (theU2 - theU1 >= 2.0 * M_PI - Precision.PConfusion())
        {
            return Box(theCirc, theTol);
        }

        // Add arc endpoints (OCCT's lambda aEval).
        gp_Pnt2d aEval(double theT) => new(aO + aR * Math.Cos(theT) * aXd + aR * Math.Sin(theT) * aYd);

        aBox.Add(aEval(theU1));
        aBox.Add(aEval(theU2));

        // For each coordinate, check if the extremal parameter lies within the arc.
        for (int k = 1; k <= 2; ++k)
        {
            double aXk = aXd.Coord(k);
            double aYk = aYd.Coord(k);

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

            double aTk = ElCLib.InPeriod(aTExtrMin, theU1, theU1 + 2.0 * M_PI);
            if (aTk >= theU1 && aTk <= theU2)
            {
                aBox.Add(aEval(aTExtrMin));
            }
            aTk = ElCLib.InPeriod(aTExtrMax, theU1, theU1 + 2.0 * M_PI);
            if (aTk >= theU1 && aTk <= theU2)
            {
                aBox.Add(aEval(aTExtrMax));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
