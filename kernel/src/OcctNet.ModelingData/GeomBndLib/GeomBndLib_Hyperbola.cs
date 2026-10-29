// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a 3D hyperbola (Geom_Hyperbola): the arc ends and the per-coordinate extrema, opened
/// for infinite parameters. The static method accepting gp_Hypr can be used directly.
/// </summary>
public sealed class GeomBndLib_Hyperbola : GeomBndLib_Curve.ICurveEvaluator
{
    private readonly Geom_Hyperbola myGeom;

    public GeomBndLib_Hyperbola(Geom_Hyperbola theHyperbola)
    {
        myGeom = theHyperbola;
    }

    public Geom_Hyperbola Geometry() => myGeom;

    Geom_Curve? GeomBndLib_Curve.ICurveEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box Box(double theU1, double theU2, double theTol) => Box(myGeom.Hypr(), theU1, theU2, theTol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Bnd_Box Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Bnd_Box BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for finite hyperbola arc, adding per-coordinate extrema.</summary>
    private static void computeHyperbolaBox(in gp_Hypr theHypr, double theT1, double theT2, ref Bnd_Box theBox)
    {
        gp_Pnt aP1 = ElCLib.Value(theT1, theHypr);
        gp_Pnt aP2 = ElCLib.Value(theT2, theHypr);
        theBox.Add(aP1);
        theBox.Add(aP2);

        if (theT1 * theT2 < 0.0)
        {
            theBox.Add(ElCLib.Value(0.0, theHypr));
        }

        gp_Ax2 aPos = theHypr.Position();
        gp_XYZ aXDir = aPos.XDirection().XYZ();
        gp_XYZ aYDir = aPos.YDirection().XYZ();
        double aRmaj = theHypr.MajorRadius();
        double aRmin = theHypr.MinorRadius();
        double aEps = Epsilon(1.0);

        for (int i = 1; i <= 3; ++i)
        {
            double aA = aRmin * aYDir.Coord(i);
            double aB = aRmaj * aXDir.Coord(i);

            double aABP = Math.Abs(aA + aB);
            double aBAM = Math.Abs(aB - aA);

            if (aABP < aEps || aBAM < aEps)
            {
                continue;
            }

            double aCf = aBAM / aABP;
            double aT3 = 0.5 * Math.Log(aCf);

            if (aT3 < theT1 || aT3 > theT2)
            {
                continue;
            }
            theBox.Add(ElCLib.Value(aT3, theHypr));
        }
    }

    /// <summary>Compute bounding box for a hyperbola arc [theU1, theU2] defined by gp_Hypr.</summary>
    public static Bnd_Box Box(in gp_Hypr theHypr, double theU1, double theU2, double theTol)
    {
        Bnd_Box aBox = new();
        if (Precision.IsNegativeInfinite(theU1))
        {
            if (Precision.IsNegativeInfinite(theU2))
            {
                throw new Standard_Failure("GeomBndLib_Hyperbola::Box - bad parameter");
            }
            else if (Precision.IsPositiveInfinite(theU2))
            {
                aBox.OpenXmax();
                aBox.OpenYmax();
                aBox.OpenZmax();
            }
            else
            {
                aBox.Add(ElCLib.Value(theU2, theHypr));
            }
            aBox.OpenXmin();
            aBox.OpenYmin();
            aBox.OpenZmin();
        }
        else if (Precision.IsPositiveInfinite(theU1))
        {
            if (Precision.IsNegativeInfinite(theU2))
            {
                aBox.OpenXmin();
                aBox.OpenYmin();
                aBox.OpenZmin();
            }
            else if (Precision.IsPositiveInfinite(theU2))
            {
                throw new Standard_Failure("GeomBndLib_Hyperbola::Box - bad parameter");
            }
            else
            {
                aBox.Add(ElCLib.Value(theU2, theHypr));
            }
            aBox.OpenXmax();
            aBox.OpenYmax();
            aBox.OpenZmax();
        }
        else
        {
            aBox.Add(ElCLib.Value(theU1, theHypr));
            if (Precision.IsNegativeInfinite(theU2))
            {
                aBox.OpenXmin();
                aBox.OpenYmin();
                aBox.OpenZmin();
            }
            else if (Precision.IsPositiveInfinite(theU2))
            {
                aBox.OpenXmax();
                aBox.OpenYmax();
                aBox.OpenZmax();
            }
            else
            {
                computeHyperbolaBox(theHypr, theU1, theU2, ref aBox);
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
