// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Line.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    public Bnd_Box Box(double theU1, double theU2, double theTol) => Box(myGeom.Lin(), theU1, theU2, theTol);

    /// <summary>Compute bounding box for full curve.</summary>
    public Bnd_Box Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>For analytical curves, BoxOptimal is same as Box.</summary>
    public Bnd_Box BoxOptimal(double theU1, double theU2, double theTol) => Box(theU1, theU2, theTol);

    /// <summary>Compute bounding box for a line segment [theU1, theU2] defined by gp_Lin.</summary>
    public static Bnd_Box Box(in gp_Lin theLin, double theU1, double theU2, double theTol)
    {
        Bnd_Box aBox = new();
        if (Precision.IsNegativeInfinite(theU1))
        {
            if (Precision.IsNegativeInfinite(theU2))
            {
                throw new Standard_Failure("GeomBndLib_Line::Box - bad parameter");
            }
            else if (Precision.IsPositiveInfinite(theU2))
            {
                GeomBndLib_InfiniteHelpers.OpenMinMax(theLin.Direction(), ref aBox);
                aBox.Add(ElCLib.Value(0.0, theLin));
            }
            else
            {
                GeomBndLib_InfiniteHelpers.OpenMin(theLin.Direction(), ref aBox);
                aBox.Add(ElCLib.Value(theU2, theLin));
            }
        }
        else if (Precision.IsPositiveInfinite(theU1))
        {
            if (Precision.IsNegativeInfinite(theU2))
            {
                GeomBndLib_InfiniteHelpers.OpenMinMax(theLin.Direction(), ref aBox);
                aBox.Add(ElCLib.Value(0.0, theLin));
            }
            else if (Precision.IsPositiveInfinite(theU2))
            {
                throw new Standard_Failure("GeomBndLib_Line::Box - bad parameter");
            }
            else
            {
                GeomBndLib_InfiniteHelpers.OpenMax(theLin.Direction(), ref aBox);
                aBox.Add(ElCLib.Value(theU2, theLin));
            }
        }
        else
        {
            aBox.Add(ElCLib.Value(theU1, theLin));
            if (Precision.IsNegativeInfinite(theU2))
            {
                GeomBndLib_InfiniteHelpers.OpenMin(theLin.Direction(), ref aBox);
            }
            else if (Precision.IsPositiveInfinite(theU2))
            {
                GeomBndLib_InfiniteHelpers.OpenMax(theLin.Direction(), ref aBox);
            }
            else
            {
                aBox.Add(ElCLib.Value(theU2, theLin));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }
}
