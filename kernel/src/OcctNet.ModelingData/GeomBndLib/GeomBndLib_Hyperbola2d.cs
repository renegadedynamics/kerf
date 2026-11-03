// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Hyperbola2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a 2D hyperbola (Geom2d_Hyperbola): the arc ends and the per-coordinate extrema, opened
/// for infinite parameters. The static method accepting gp_Hypr2d can be used directly.
/// </summary>
public sealed class GeomBndLib_Hyperbola2d : GeomBndLib_Curve2d.ICurve2dEvaluator
{
    private readonly Geom2d_Hyperbola myGeom;

    public GeomBndLib_Hyperbola2d(Geom2d_Hyperbola theHyperbola)
    {
        myGeom = theHyperbola;
    }

    public Geom2d_Hyperbola Geometry() => myGeom;

    Geom2d_Curve? GeomBndLib_Curve2d.ICurve2dEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double theU1, double theU2, double theTol) => Box(myGeom.Hypr2d(), theU1, theU2, theTol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box2d BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for finite 2D hyperbola arc, adding per-coordinate extrema.</summary>
    private static void computeHyperbola2dBox(in Hyperbola2D theHypr, double theT1, double theT2, ref Box2d theBox)
    {
        Point2D aP1 = global::Kerf.Engine.Geometry.ElCLib.Value(theT1, theHypr);
        Point2D aP2 = global::Kerf.Engine.Geometry.ElCLib.Value(theT2, theHypr);
        theBox.Add(aP1);
        theBox.Add(aP2);

        if (theT1 * theT2 < 0.0)
        {
            theBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, theHypr));
        }

        Frame2D aPos = theHypr.Axis();
        Xy aXDir = aPos.XDirection().XY();
        Xy aYDir = aPos.YDirection().XY();
        double aRmaj = theHypr.MajorRadius();
        double aRmin = theHypr.MinorRadius();
        double aEps = Epsilon(1.0);

        for (int i = 1; i <= 2; ++i)
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
            theBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(aT3, theHypr));
        }
    }

    /// <summary>Compute bounding box for a 2D hyperbola arc [theU1, theU2] defined by gp_Hypr2d.</summary>
    public static Box2d Box(in Hyperbola2D theHypr, double theU1, double theU2, double theTol)
    {
        Box2d aBox = new();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                throw new FailureException("GeomBndLib_Hyperbola2d::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU2))
            {
                aBox.OpenXmax();
                aBox.OpenYmax();
            }
            else
            {
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU2, theHypr));
            }
            aBox.OpenXmin();
            aBox.OpenYmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                aBox.OpenXmin();
                aBox.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU2))
            {
                throw new FailureException("GeomBndLib_Hyperbola2d::Box - bad parameter");
            }
            else
            {
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU2, theHypr));
            }
            aBox.OpenXmax();
            aBox.OpenYmax();
        }
        else
        {
            aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU1, theHypr));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                aBox.OpenXmin();
                aBox.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU2))
            {
                aBox.OpenXmax();
                aBox.OpenYmax();
            }
            else
            {
                computeHyperbola2dBox(theHypr, theU1, theU2, ref aBox);
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
