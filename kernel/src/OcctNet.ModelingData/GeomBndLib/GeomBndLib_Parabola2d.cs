// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a 2D parabola (Geom2d_Parabola): the arc ends and the vertex, opened for infinite
/// parameters. The static method accepting gp_Parab2d can be used directly.
/// </summary>
public sealed class GeomBndLib_Parabola2d : GeomBndLib_Curve2d.ICurve2dEvaluator
{
    private readonly Geom2d_Parabola myGeom;

    public GeomBndLib_Parabola2d(Geom2d_Parabola theParabola)
    {
        myGeom = theParabola;
    }

    public Geom2d_Parabola Geometry() => myGeom;

    Geom2d_Curve? GeomBndLib_Curve2d.ICurve2dEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box2d Box(double theU1, double theU2, double theTol) => Box(myGeom.Parab2d(), theU1, theU2, theTol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Bnd_Box2d Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Bnd_Box2d BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for a 2D parabola arc [theU1, theU2] defined by gp_Parab2d.</summary>
    public static Bnd_Box2d Box(in gp_Parab2d theParab, double theU1, double theU2, double theTol)
    {
        Bnd_Box2d aBox = new();
        if (Precision.IsNegativeInfinite(theU1))
        {
            if (Precision.IsNegativeInfinite(theU2))
            {
                throw new Standard_Failure("GeomBndLib_Parabola2d::Box - bad parameter");
            }
            else if (Precision.IsPositiveInfinite(theU2))
            {
                aBox.OpenXmax();
                aBox.OpenYmax();
            }
            else
            {
                aBox.Add(ElCLib.Value(theU2, theParab));
            }
            aBox.OpenXmin();
            aBox.OpenYmin();
        }
        else if (Precision.IsPositiveInfinite(theU1))
        {
            if (Precision.IsNegativeInfinite(theU2))
            {
                aBox.OpenXmin();
                aBox.OpenYmin();
            }
            else if (Precision.IsPositiveInfinite(theU2))
            {
                throw new Standard_Failure("GeomBndLib_Parabola2d::Box - bad parameter");
            }
            else
            {
                aBox.Add(ElCLib.Value(theU2, theParab));
            }
            aBox.OpenXmax();
            aBox.OpenYmax();
        }
        else
        {
            aBox.Add(ElCLib.Value(theU1, theParab));
            if (Precision.IsNegativeInfinite(theU2))
            {
                aBox.OpenXmin();
                aBox.OpenYmin();
            }
            else if (Precision.IsPositiveInfinite(theU2))
            {
                aBox.OpenXmax();
                aBox.OpenYmax();
            }
            else
            {
                // OCCT adds only the ends and the vertex: the coordinate extrema of a parabola turned in its plane
                // (at U = -2F.Yk/Xk) are missed, so the box may not contain the arc (occt-issues.md).
                aBox.Add(ElCLib.Value(theU2, theParab));
                if (theU1 * theU2 < 0)
                {
                    aBox.Add(ElCLib.Value(0.0, theParab));
                }
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
