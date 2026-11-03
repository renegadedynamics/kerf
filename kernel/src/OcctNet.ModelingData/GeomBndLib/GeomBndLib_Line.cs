// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Line.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a 3D line (Geom_Line). Handles infinite parameters by opening the box in appropriate
/// directions. The static method accepting gp_Lin can be used directly without constructing a Geom_Line.
/// </summary>
public sealed class GeomBndLib_Line : GeomBndLib_Curve.ICurveEvaluator
{
    private readonly Geom_Line myGeom;

    public GeomBndLib_Line(Geom_Line theLine)
    {
        myGeom = theLine;
    }

    public Geom_Line Geometry() => myGeom;

    Geom_Curve? GeomBndLib_Curve.ICurveEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for line segment [theU1, theU2].</summary>
    public Box Box(double theU1, double theU2, double theTol) => Box(myGeom.Lin(), theU1, theU2, theTol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for a line segment [theU1, theU2] defined by gp_Lin.</summary>
    public static Box Box(in Line3D theLin, double theU1, double theU2, double theTol)
    {
        Box aBox = new();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theU2))
            {
                throw new FailureException("GeomBndLib_Line::Box - bad parameter");
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
                throw new FailureException("GeomBndLib_Line::Box - bad parameter");
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
