// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a 3D parabola (Geom_Parabola): the arc ends and the vertex, opened for infinite
/// parameters. The static method accepting gp_Parab can be used directly.
/// </summary>
public sealed class GeomBndLib_Parabola : GeomBndLib_Curve.ICurveEvaluator
{
    private readonly Geom_Parabola myGeom;

    public GeomBndLib_Parabola(Geom_Parabola theParabola)
    {
        myGeom = theParabola;
    }

    public Geom_Parabola Geometry() => myGeom;

    Geom_Curve? GeomBndLib_Curve.ICurveEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double theU1, double theU2, double theTol) => Box(myGeom.Parab(), theU1, theU2, theTol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for a parabola arc [theU1, theU2] defined by gp_Parab.</summary>
    public static Box Box(in Parabola3D theParab, double theU1, double theU2, double theTol)
    {
        Box aBox = new();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                throw new FailureException("GeomBndLib_Parabola::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU2))
            {
                aBox.OpenXmax();
                aBox.OpenYmax();
                aBox.OpenZmax();
            }
            else
            {
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU2, theParab));
            }
            aBox.OpenXmin();
            aBox.OpenYmin();
            aBox.OpenZmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                aBox.OpenXmin();
                aBox.OpenYmin();
                aBox.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU2))
            {
                throw new FailureException("GeomBndLib_Parabola::Box - bad parameter");
            }
            else
            {
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU2, theParab));
            }
            aBox.OpenXmax();
            aBox.OpenYmax();
            aBox.OpenZmax();
        }
        else
        {
            aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU1, theParab));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                aBox.OpenXmin();
                aBox.OpenYmin();
                aBox.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU2))
            {
                aBox.OpenXmax();
                aBox.OpenYmax();
                aBox.OpenZmax();
            }
            else
            {
                // OCCT adds only the ends and the vertex: the coordinate extrema of a parabola turned in its plane
                // (at U = -2F.Yk/Xk) are missed, so the box may not contain the arc (occt-issues.md).
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU2, theParab));
                if (theU1 * theU2 < 0)
                {
                    aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, theParab));
                }
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
