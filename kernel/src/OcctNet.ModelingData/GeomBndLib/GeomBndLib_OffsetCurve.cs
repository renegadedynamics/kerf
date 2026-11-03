// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a 3D offset curve (Geom_OffsetCurve): exactly for offsets of lines and of circles
/// offset in their plane, otherwise the basis curve's box enlarged per coordinate by the offset.
/// </summary>
public sealed class GeomBndLib_OffsetCurve : GeomBndLib_Curve.ICurveEvaluator
{
    private readonly Geom_OffsetCurve myGeom;

    public GeomBndLib_OffsetCurve(Geom_OffsetCurve theCurve)
    {
        myGeom = theCurve;
    }

    public Geom_OffsetCurve Geometry() => myGeom;

    Geom_Curve? GeomBndLib_Curve.ICurveEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Box BoxOptimal(double theTol) => BoxOptimal(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>Try the exact line fast path. Returns a non-void box on success.</summary>
    private static Box tryOffsetLine(Geom_Curve theBasis, Geom_OffsetCurve theOff, double theU1, double theU2, double theTol)
    {
        if (theBasis is not Geom_Line aLine)
        {
            return new Box();
        }

        Direction3D aDir = theOff.Direction();
        Line3D aBasisLin = aLine.Lin();
        Vector3D aShift = new Vector3D(aBasisLin.Direction()).Crossed(new Vector3D(aDir));
        if (aShift.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box();
        }

        aShift.Normalize();
        aShift *= theOff.Offset();
        Point3D aLoc = aBasisLin.Location().Translated(aShift);
        Box aBox = GeomBndLib_Line.Box(new Line3D(aLoc, aBasisLin.Direction()), theU1, theU2, 0.0);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Try the exact axis-aligned circle fast path. Returns a non-void box on success.</summary>
    private static Box tryOffsetCircle(Geom_Curve theBasis, Geom_OffsetCurve theOff, double theU1, double theU2, double theTol)
    {
        if (theBasis is not Geom_Circle aCircle)
        {
            return new Box();
        }

        Direction3D aDir = theOff.Direction();
        Circle3D aBasisCirc = aCircle.Circ();
        double aDotAxis = Math.Abs(aBasisCirc.Axis().Direction().Dot(aDir));
        if (Math.Abs(1.0 - aDotAxis) > global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return new Box();
        }

        double aUMid = 0.5 * (theU1 + theU2);
        global::Kerf.Engine.Geometry.ElCLib.D1(aUMid, aBasisCirc, out Point3D aP, out Vector3D aV1);
        if (aV1.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box();
        }

        Vector3D aNormal = aV1.Crossed(new Vector3D(aDir));
        if (aNormal.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box();
        }
        aNormal.Normalize();

        Vector3D aRadial = new(aBasisCirc.Location(), aP);
        if (aRadial.SquareMagnitude() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box();
        }
        aRadial.Normalize();

        double aSign = aNormal.Dot(aRadial);
        if (Math.Abs(Math.Abs(aSign) - 1.0) > 1e-9)
        {
            return new Box();
        }

        double aNewRadius = aBasisCirc.Radius() + theOff.Offset() * aSign;
        Box aBox = new();
        if (aNewRadius > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Circle3D aOffsetCirc = aBasisCirc;
            aOffsetCirc.SetRadius(aNewRadius);
            aBox = GeomBndLib_Circle.Box(aOffsetCirc, theU1, theU2, 0.0);
        }
        else if (Math.Abs(aNewRadius) <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            aBox.Add(aBasisCirc.Location());
        }
        else
        {
            return new Box();
        }

        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double theU1, double theU2, double theTol)
    {
        // Specialized fast paths for line and axis-aligned circle.
        Geom_Curve aBasis = myGeom.BasisCurve();

        Box aBox = tryOffsetLine(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid)
        {
            return aBox;
        }

        aBox = tryOffsetCircle(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid)
        {
            return aBox;
        }

        // Generic fallback: enlarge basis curve box per coordinate.
        // The offset displacement magnitude per coordinate k is bounded by |d| * sqrt(1 - Vk^2).
        double anOffset = Math.Abs(myGeom.Offset());
        Direction3D aDir = myGeom.Direction();

        GeomBndLib_Curve aCurveEval = new(aBasis);
        aBox = aCurveEval.Box(theU1, theU2, 0.0);
        if (aBox.IsVoid)
        {
            return aBox;
        }

        double aDx = aDir.X();
        double aDy = aDir.Y();
        double aDz = aDir.Z();
        double aEnlargeX = anOffset * Math.Sqrt(Max(0.0, 1.0 - aDx * aDx));
        double aEnlargeY = anOffset * Math.Sqrt(Max(0.0, 1.0 - aDy * aDy));
        double aEnlargeZ = anOffset * Math.Sqrt(Max(0.0, 1.0 - aDz * aDz));

        bool isOpenXmin = aBox.IsOpenXmin;
        bool isOpenXmax = aBox.IsOpenXmax;
        bool isOpenYmin = aBox.IsOpenYmin;
        bool isOpenYmax = aBox.IsOpenYmax;
        bool isOpenZmin = aBox.IsOpenZmin;
        bool isOpenZmax = aBox.IsOpenZmax;

        if (aBox.HasFinitePart)
        {
            (double aXmin, double aXmax, double aYmin, double aYmax, double aZmin, double aZmax) = aBox.FinitePart().Get();
            aBox.Update(aXmin - aEnlargeX,
                        aYmin - aEnlargeY,
                        aZmin - aEnlargeZ,
                        aXmax + aEnlargeX,
                        aYmax + aEnlargeY,
                        aZmax + aEnlargeZ);
        }

        if (isOpenXmin)
        {
            aBox.OpenXmin();
        }
        if (isOpenXmax)
        {
            aBox.OpenXmax();
        }
        if (isOpenYmin)
        {
            aBox.OpenYmin();
        }
        if (isOpenYmax)
        {
            aBox.OpenYmax();
        }
        if (isOpenZmin)
        {
            aBox.OpenZmin();
        }
        if (isOpenZmax)
        {
            aBox.OpenZmax();
        }

        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute precise bounding box for arc [theU1, theU2] by sampling the offset curve.</summary>
    public Box BoxOptimal(double theU1, double theU2, double theTol)
    {
        // Reuse exact fast paths.
        Geom_Curve aBasis = myGeom.BasisCurve();

        Box aBox = tryOffsetLine(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid)
        {
            return aBox;
        }

        aBox = tryOffsetCircle(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid)
        {
            return aBox;
        }

        // Sampling fallback for precise result on general offset curves.
        GeomAdaptor_Curve anAdaptor = new(myGeom);
        GeomBndLib_OtherCurve anOther = new(anAdaptor);
        return anOther.BoxOptimal(theU1, theU2, theTol);
    }
}
