// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SurfaceOfExtrusion.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SurfaceOfExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a surface of linear extrusion (Geom_SurfaceOfLinearExtrusion) analytically:
/// P(U, V) = BasisCurve(U) + V * Direction, so the box is the basis curve box swept along the direction.
/// </summary>
internal sealed class SurfaceOfExtrusionBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly ExtrusionSurface _geom;

    public SurfaceOfExtrusionBounds(ExtrusionSurface surf)
    {
        _geom = surf;
    }

    public ExtrusionSurface Geometry() => _geom;

    /// <summary>Build the extrusion box from a pre-computed basis curve box.</summary>
    private static Box BuildExtrusionBox(in Box curveBox,
                                             in Direction3D dir,
                                             in Xyz dirXYZ,
                                             double vMin,
                                             double vMax,
                                             double tol)
    {
        Box box = new();
        if (curveBox.IsVoid)
        {
            box.Enlarge(tol);
            return box;
        }

        bool isVMinInf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin);
        bool isVMaxInf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax);

        if (curveBox.IsOpen)
        {
            // Propagate curve openness through the extrusion rather than returning whole space.
            if (curveBox.IsOpenXmin)
            {
                box.OpenXmin();
            }
            if (curveBox.IsOpenXmax)
            {
                box.OpenXmax();
            }
            if (curveBox.IsOpenYmin)
            {
                box.OpenYmin();
            }
            if (curveBox.IsOpenYmax)
            {
                box.OpenYmax();
            }
            if (curveBox.IsOpenZmin)
            {
                box.OpenZmin();
            }
            if (curveBox.IsOpenZmax)
            {
                box.OpenZmax();
            }

            if (isVMinInf && isVMaxInf)
            {
                InfiniteHelpers.OpenMinMax(dir, ref box);
            }
            else if (isVMinInf)
            {
                InfiniteHelpers.OpenMin(dir, ref box);
            }
            else if (isVMaxInf)
            {
                InfiniteHelpers.OpenMax(dir, ref box);
            }

            if (curveBox.HasFinitePart)
            {
                (double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) = curveBox.FinitePart().Get();
                if (!isVMinInf)
                {
                    Xyz shift = vMin * dirXYZ;
                    box.Add(new Point3D(xmin + shift.X(), ymin + shift.Y(), zmin + shift.Z()));
                    box.Add(new Point3D(xmax + shift.X(), ymax + shift.Y(), zmax + shift.Z()));
                }
                if (!isVMaxInf)
                {
                    Xyz shift = vMax * dirXYZ;
                    box.Add(new Point3D(xmin + shift.X(), ymin + shift.Y(), zmin + shift.Z()));
                    box.Add(new Point3D(xmax + shift.X(), ymax + shift.Y(), zmax + shift.Z()));
                }
            }

            box.Enlarge(tol);
            return box;
        }

        if (isVMinInf && isVMaxInf)
        {
            box.Add(curveBox);
            InfiniteHelpers.OpenMinMax(dir, ref box);
            box.Enlarge(tol);
            return box;
        }

        (double bXmin, double bXmax, double bYmin, double bYmax, double bZmin, double bZmax) = curveBox.Get();
        if (isVMinInf)
        {
            InfiniteHelpers.OpenMin(dir, ref box);
            Xyz shift = vMax * dirXYZ;
            box.Add(new Point3D(bXmin + shift.X(), bYmin + shift.Y(), bZmin + shift.Z()));
            box.Add(new Point3D(bXmax + shift.X(), bYmax + shift.Y(), bZmax + shift.Z()));
        }
        else if (isVMaxInf)
        {
            InfiniteHelpers.OpenMax(dir, ref box);
            Xyz shift = vMin * dirXYZ;
            box.Add(new Point3D(bXmin + shift.X(), bYmin + shift.Y(), bZmin + shift.Z()));
            box.Add(new Point3D(bXmax + shift.X(), bYmax + shift.Y(), bZmax + shift.Z()));
        }
        else
        {
            Xyz shiftMin = vMin * dirXYZ;
            box.Add(new Point3D(bXmin + shiftMin.X(), bYmin + shiftMin.Y(), bZmin + shiftMin.Z()));
            box.Add(new Point3D(bXmax + shiftMin.X(), bYmax + shiftMin.Y(), bZmax + shiftMin.Z()));
            Xyz shiftMax = vMax * dirXYZ;
            box.Add(new Point3D(bXmin + shiftMax.X(), bYmin + shiftMax.Y(), bZmin + shiftMax.Z()));
            box.Add(new Point3D(bXmax + shiftMax.X(), bYmax + shiftMax.Y(), bZmax + shiftMax.Z()));
        }

        box.Enlarge(tol);
        return box;
    }

    /// <summary>Clamps [theU1, theU2] to a non-periodic basis curve's range, as both of OCCT's methods do inline.</summary>
    private static void ClampCurveRange(Curve3D aBasisCurve, ref double aCurveU1, ref double aCurveU2)
    {
        if (!aBasisCurve.IsPeriodic)
        {
            double first = aBasisCurve.FirstParameter;
            double last = aBasisCurve.LastParameter;
            if (aCurveU1 < first)
            {
                aCurveU1 = first;
            }
            else if (aCurveU1 > last)
            {
                aCurveU1 = last;
            }
            if (aCurveU2 < first)
            {
                aCurveU2 = first;
            }
            else if (aCurveU2 > last)
            {
                aCurveU2 = last;
            }
            if (aCurveU1 > aCurveU2)
            {
                double tmp = aCurveU1;
                aCurveU1 = aCurveU2;
                aCurveU2 = tmp;
            }
        }
    }

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double tol)
    {
        _geom.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        return Box(u1, u2, v1, v2, tol);
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        try
        {
            // P(U, V) = BasisCurve(U) + V * Direction
            Curve3D basisCurve = _geom.BasisCurve;
            Direction3D dir = _geom.Direction;
            Xyz dirXYZ = dir.XYZ();

            double curveU1 = uMin;
            double curveU2 = uMax;
            ClampCurveRange(basisCurve, ref curveU1, ref curveU2);

            CurveBounds curveEval = new(basisCurve);
            Box curveBox = curveEval.Box(curveU1, curveU2, 0.0);
            if (curveBox.IsVoid)
            {
                SurfaceAdaptor adaptor0 = new(_geom);
                OtherSurfaceBounds other0 = new(adaptor0);
                return other0.Box(uMin, uMax, vMin, vMax, tol);
            }

            return BuildExtrusionBox(curveBox, dir, dirXYZ, vMin, vMax, tol);
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            // Fall back to robust generic sampling if basis-curve segmentation fails.
            SurfaceAdaptor adaptor = new(_geom);
            OtherSurfaceBounds other = new(adaptor);
            return other.Box(uMin, uMax, vMin, vMax, tol);
        }
    }

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Box BoxOptimal(double tol)
    {
        _geom.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        return BoxOptimal(u1, u2, v1, v2, tol);
    }

    /// <summary>Compute precise bounding box using tight basis curve bounds.</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        try
        {
            // Use the tight basis curve box for a more precise result.
            Curve3D basisCurve = _geom.BasisCurve;
            Direction3D dir = _geom.Direction;
            Xyz dirXYZ = dir.XYZ();

            double curveU1 = uMin;
            double curveU2 = uMax;
            ClampCurveRange(basisCurve, ref curveU1, ref curveU2);

            CurveBounds curveEval = new(basisCurve);
            Box curveBox = curveEval.BoxOptimal(curveU1, curveU2, 0.0);
            if (curveBox.IsVoid)
            {
                SurfaceAdaptor adaptor0 = new(_geom);
                OtherSurfaceBounds other0 = new(adaptor0);
                return other0.BoxOptimal(uMin, uMax, vMin, vMax, tol);
            }

            return BuildExtrusionBox(curveBox, dir, dirXYZ, vMin, vMax, tol);
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            // Fall back to robust generic sampling if basis-curve segmentation fails.
            SurfaceAdaptor adaptor = new(_geom);
            OtherSurfaceBounds other = new(adaptor);
            return other.BoxOptimal(uMin, uMax, vMin, vMax, tol);
        }
    }
}
