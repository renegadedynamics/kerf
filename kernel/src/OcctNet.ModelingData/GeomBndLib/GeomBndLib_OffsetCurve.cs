// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    public Bnd_Box Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Bnd_Box BoxOptimal(double theTol) => BoxOptimal(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>Try the exact line fast path. Returns a non-void box on success.</summary>
    private static Bnd_Box tryOffsetLine(Geom_Curve theBasis, Geom_OffsetCurve theOff, double theU1, double theU2, double theTol)
    {
        if (theBasis is not Geom_Line aLine)
        {
            return new Bnd_Box();
        }

        gp_Dir aDir = theOff.Direction();
        gp_Lin aBasisLin = aLine.Lin();
        gp_Vec aShift = new gp_Vec(aBasisLin.Direction()).Crossed(new gp_Vec(aDir));
        if (aShift.SquareMagnitude() <= Precision.SquareConfusion())
        {
            return new Bnd_Box();
        }

        aShift.Normalize();
        aShift *= theOff.Offset();
        gp_Pnt aLoc = aBasisLin.Location().Translated(aShift);
        Bnd_Box aBox = GeomBndLib_Line.Box(new gp_Lin(aLoc, aBasisLin.Direction()), theU1, theU2, 0.0);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Try the exact axis-aligned circle fast path. Returns a non-void box on success.</summary>
    private static Bnd_Box tryOffsetCircle(Geom_Curve theBasis, Geom_OffsetCurve theOff, double theU1, double theU2, double theTol)
    {
        if (theBasis is not Geom_Circle aCircle)
        {
            return new Bnd_Box();
        }

        gp_Dir aDir = theOff.Direction();
        gp_Circ aBasisCirc = aCircle.Circ();
        double aDotAxis = Math.Abs(aBasisCirc.Axis().Direction().Dot(aDir));
        if (Math.Abs(1.0 - aDotAxis) > Precision.Angular())
        {
            return new Bnd_Box();
        }

        double aUMid = 0.5 * (theU1 + theU2);
        ElCLib.D1(aUMid, aBasisCirc, out gp_Pnt aP, out gp_Vec aV1);
        if (aV1.SquareMagnitude() <= Precision.SquareConfusion())
        {
            return new Bnd_Box();
        }

        gp_Vec aNormal = aV1.Crossed(new gp_Vec(aDir));
        if (aNormal.SquareMagnitude() <= Precision.SquareConfusion())
        {
            return new Bnd_Box();
        }
        aNormal.Normalize();

        gp_Vec aRadial = new(aBasisCirc.Location(), aP);
        if (aRadial.SquareMagnitude() <= Precision.SquareConfusion())
        {
            return new Bnd_Box();
        }
        aRadial.Normalize();

        double aSign = aNormal.Dot(aRadial);
        if (Math.Abs(Math.Abs(aSign) - 1.0) > 1e-9)
        {
            return new Bnd_Box();
        }

        double aNewRadius = aBasisCirc.Radius() + theOff.Offset() * aSign;
        Bnd_Box aBox = new();
        if (aNewRadius > Precision.Confusion())
        {
            gp_Circ aOffsetCirc = aBasisCirc;
            aOffsetCirc.SetRadius(aNewRadius);
            aBox = GeomBndLib_Circle.Box(aOffsetCirc, theU1, theU2, 0.0);
        }
        else if (Math.Abs(aNewRadius) <= Precision.Confusion())
        {
            aBox.Add(aBasisCirc.Location());
        }
        else
        {
            return new Bnd_Box();
        }

        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box Box(double theU1, double theU2, double theTol)
    {
        // Specialized fast paths for line and axis-aligned circle.
        Geom_Curve aBasis = myGeom.BasisCurve();

        Bnd_Box aBox = tryOffsetLine(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid())
        {
            return aBox;
        }

        aBox = tryOffsetCircle(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid())
        {
            return aBox;
        }

        // Generic fallback: enlarge basis curve box per coordinate.
        // The offset displacement magnitude per coordinate k is bounded by |d| * sqrt(1 - Vk^2).
        double anOffset = Math.Abs(myGeom.Offset());
        gp_Dir aDir = myGeom.Direction();

        GeomBndLib_Curve aCurveEval = new(aBasis);
        aBox = aCurveEval.Box(theU1, theU2, 0.0);
        if (aBox.IsVoid())
        {
            return aBox;
        }

        double aDx = aDir.X();
        double aDy = aDir.Y();
        double aDz = aDir.Z();
        double aEnlargeX = anOffset * Math.Sqrt(Max(0.0, 1.0 - aDx * aDx));
        double aEnlargeY = anOffset * Math.Sqrt(Max(0.0, 1.0 - aDy * aDy));
        double aEnlargeZ = anOffset * Math.Sqrt(Max(0.0, 1.0 - aDz * aDz));

        bool isOpenXmin = aBox.IsOpenXmin();
        bool isOpenXmax = aBox.IsOpenXmax();
        bool isOpenYmin = aBox.IsOpenYmin();
        bool isOpenYmax = aBox.IsOpenYmax();
        bool isOpenZmin = aBox.IsOpenZmin();
        bool isOpenZmax = aBox.IsOpenZmax();

        if (aBox.HasFinitePart())
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
    public Bnd_Box BoxOptimal(double theU1, double theU2, double theTol)
    {
        // Reuse exact fast paths.
        Geom_Curve aBasis = myGeom.BasisCurve();

        Bnd_Box aBox = tryOffsetLine(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid())
        {
            return aBox;
        }

        aBox = tryOffsetCircle(aBasis, myGeom, theU1, theU2, theTol);
        if (!aBox.IsVoid())
        {
            return aBox;
        }

        // Sampling fallback for precise result on general offset curves.
        GeomAdaptor_Curve anAdaptor = new(myGeom);
        GeomBndLib_OtherCurve anOther = new(anAdaptor);
        return anOther.BoxOptimal(theU1, theU2, theTol);
    }
}
