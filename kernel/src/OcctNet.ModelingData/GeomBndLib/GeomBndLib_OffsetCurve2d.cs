// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a 2D offset curve (Geom2d_OffsetCurve): exactly for offsets of lines and circles,
/// otherwise by sampling the offset curve itself (GeomBndLib_OtherCurve2d).
/// </summary>
public sealed class GeomBndLib_OffsetCurve2d : GeomBndLib_Curve2d.ICurve2dEvaluator
{
    private readonly Geom2d_OffsetCurve myGeom;

    public GeomBndLib_OffsetCurve2d(Geom2d_OffsetCurve theCurve)
    {
        myGeom = theCurve;
    }

    public Geom2d_OffsetCurve Geometry() => myGeom;

    Geom2d_Curve? GeomBndLib_Curve2d.ICurve2dEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Box2d BoxOptimal(double theTol) => BoxOptimal(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>Try the exact offset-line fast path. Returns a non-void box on success.</summary>
    private static Box2d tryOffsetLine2d(Geom2d_Curve theBasis, Geom2d_OffsetCurve theOff, double theU1, double theU2, double theTol)
    {
        if (theBasis is not Geom2d_Line aLine)
        {
            return new Box2d();
        }

        Line2D aBasisLin = aLine.Lin2d();
        Vector2D aNormal = new(aBasisLin.Direction().Y(), -aBasisLin.Direction().X());
        if (aNormal.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box2d();
        }

        aNormal.Normalize();
        aNormal *= theOff.Offset();
        Point2D aLoc = aBasisLin.Location().Translated(aNormal);
        Box2d aLocalBox = GeomBndLib_Line2d.Box(new Line2D(aLoc, aBasisLin.Direction()), theU1, theU2, 0.0);
        aLocalBox.Enlarge(theTol);
        return aLocalBox;
    }

    /// <summary>Try the exact offset-circle fast path. Returns a non-void box on success.</summary>
    private static Box2d tryOffsetCircle2d(Geom2d_Curve theBasis, Geom2d_OffsetCurve theOff, double theU1, double theU2, double theTol)
    {
        if (theBasis is not Geom2d_Circle aCircle)
        {
            return new Box2d();
        }

        Circle2D aBasisCirc = aCircle.Circ2d();
        double aUMid = 0.5 * (theU1 + theU2);
        global::Kerf.Engine.Geometry.ElCLib.D1(aUMid, aBasisCirc, out Point2D aP, out Vector2D aV1);
        if (aV1.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box2d();
        }

        Vector2D aNormal = new(aV1.Y(), -aV1.X());
        if (aNormal.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box2d();
        }
        aNormal.Normalize();

        Vector2D aRadial = new(aBasisCirc.Location(), aP);
        if (aRadial.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box2d();
        }
        aRadial.Normalize();

        double aSign = aNormal.Dot(aRadial);
        if (Math.Abs(Math.Abs(aSign) - 1.0) > 1e-9)
        {
            return new Box2d();
        }

        double aNewRadius = aBasisCirc.Radius() + theOff.Offset() * aSign;
        Box2d aLocalBox = new();
        if (aNewRadius > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Circle2D aOffsetCirc = aBasisCirc;
            aOffsetCirc.SetRadius(aNewRadius);
            aLocalBox = GeomBndLib_Circle2d.Box(aOffsetCirc, theU1, theU2, 0.0);
        }
        else if (Math.Abs(aNewRadius) <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            aLocalBox.Add(aBasisCirc.Location());
        }
        else
        {
            return new Box2d();
        }

        aLocalBox.Enlarge(theTol);
        return aLocalBox;
    }

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double theU1, double theU2, double theTol)
    {
        Geom2d_Curve aBasis = myGeom.BasisCurve();

        Box2d aBox = tryOffsetLine2d(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid)
        {
            return aBox;
        }

        aBox = tryOffsetCircle2d(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid)
        {
            return aBox;
        }

        // Generic fallback: sample the actual offset curve for a tight bounding box.
        // This avoids the large over-inflation of basis_box + Enlarge(|offset|).
        Geom2dAdaptor_Curve anAdaptor = new(myGeom);
        GeomBndLib_OtherCurve2d anOther = new(anAdaptor);
        return anOther.Box(theU1, theU2, theTol);
    }

    /// <summary>Compute precise bounding box for arc [theU1, theU2] by sampling the offset curve.</summary>
    public Box2d BoxOptimal(double theU1, double theU2, double theTol)
    {
        // Reuse exact fast paths for line and circle.
        Geom2d_Curve aBasis = myGeom.BasisCurve();

        Box2d aBox = tryOffsetLine2d(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid)
        {
            return aBox;
        }

        aBox = tryOffsetCircle2d(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid)
        {
            return aBox;
        }

        // Sampling fallback for precise result on general offset curves.
        Geom2dAdaptor_Curve anAdaptor = new(myGeom);
        GeomBndLib_OtherCurve2d anOther = new(anAdaptor);
        return anOther.BoxOptimal(theU1, theU2, theTol);
    }
}
