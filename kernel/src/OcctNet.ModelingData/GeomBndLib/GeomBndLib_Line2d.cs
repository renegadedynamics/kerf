// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Line2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a 2D line (Geom2d_Line). Handles infinite parameters by opening the box in appropriate
/// directions. The static method accepting gp_Lin2d can be used directly without constructing a Geom2d_Line.
/// </summary>
public sealed class GeomBndLib_Line2d : GeomBndLib_Curve2d.ICurve2dEvaluator
{
    private readonly Geom2d_Line myGeom;

    public GeomBndLib_Line2d(Geom2d_Line theLine)
    {
        myGeom = theLine;
    }

    public Geom2d_Line Geometry() => myGeom;

    Geom2d_Curve? GeomBndLib_Curve2d.ICurve2dEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for line segment [theU1, theU2].</summary>
    public Box2d Box(double theU1, double theU2, double theTol) => Box(myGeom.Lin2d(), theU1, theU2, theTol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box2d BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for a 2D line segment [theU1, theU2] defined by gp_Lin2d.</summary>
    public static Box2d Box(in Line2D theLin, double theU1, double theU2, double theTol)
    {
        Box2d aBox = new();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                throw new FailureException("GeomBndLib_Line2d::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU2))
            {
                GeomBndLib_InfiniteHelpers.OpenMinMax(theLin.Direction(), ref aBox);
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, theLin));
            }
            else
            {
                GeomBndLib_InfiniteHelpers.OpenMin(theLin.Direction(), ref aBox);
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU2, theLin));
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                GeomBndLib_InfiniteHelpers.OpenMinMax(theLin.Direction(), ref aBox);
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, theLin));
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU2))
            {
                throw new FailureException("GeomBndLib_Line2d::Box - bad parameter");
            }
            else
            {
                GeomBndLib_InfiniteHelpers.OpenMax(theLin.Direction(), ref aBox);
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU2, theLin));
            }
        }
        else
        {
            aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU1, theLin));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                GeomBndLib_InfiniteHelpers.OpenMin(theLin.Direction(), ref aBox);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theU2))
            {
                GeomBndLib_InfiniteHelpers.OpenMax(theLin.Direction(), ref aBox);
            }
            else
            {
                aBox.Add(global::Kerf.Engine.Geometry.ElCLib.Value(theU2, theLin));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
