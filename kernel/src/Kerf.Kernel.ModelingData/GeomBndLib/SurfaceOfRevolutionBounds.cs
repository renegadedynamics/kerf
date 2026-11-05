// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SurfaceOfRevolution.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a surface of revolution (Geom_SurfaceOfRevolution): the basis curve's box is computed,
/// and the eight corners of that box are revolved about the axis, each circle arc bounded by GeomBndLib_Circle.
/// </summary>
internal sealed class SurfaceOfRevolutionBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly RevolutionSurface _geom;

    public SurfaceOfRevolutionBounds(RevolutionSurface surf)
    {
        _geom = surf;
    }

    public RevolutionSurface Geometry() => _geom;

    /// <summary>Add the bounding box contribution of a single revolution circle for a point revolved around the axis.</summary>
    private static void AddRevolutionCircle(in Point3D origin,
                                            in Direction3D axisDir,
                                            in Point3D basisPt,
                                            double uMin,
                                            double uMax,
                                            ref Box box)
    {
        // Project the basis curve point onto the revolution axis.
        Xyz delta = basisPt.XYZ() - origin.XYZ();
        double h = delta.Dot(axisDir.XYZ());
        Xyz center = origin.XYZ() + h * axisDir.XYZ();

        // Radial vector from axis to basis point.
        Xyz radial = basisPt.XYZ() - center;
        double radius = radial.Modulus();
        if (radius < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // Point is on the axis - revolution is a single point.
            box.Add(basisPt);
            return;
        }

        // Construct the revolution circle.
        Direction3D xDir = new(radial);
        Frame3D ax2 = new(new Point3D(center), axisDir, xDir);
        Circle3D circ = new(ax2, radius);
        box.Add(CircleBounds.Box(circ, uMin, uMax, 0.0));
    }

    /// <summary>Revolves the eight corners of a box (OCCT's two identical triple loops).</summary>
    private static void AddRevolvedCorners(double aXmin, double aXmax, double aYmin, double aYmax, double aZmin, double aZmax,
                                           in Point3D origin, in Direction3D axisDir, double uMin, double uMax,
                                           ref Box aBox)
    {
        ReadOnlySpan<double> xVals = [aXmin, aXmax];
        ReadOnlySpan<double> yVals = [aYmin, aYmax];
        ReadOnlySpan<double> zVals = [aZmin, aZmax];
        for (int ix = 0; ix < 2; ++ix)
        {
            for (int iy = 0; iy < 2; ++iy)
            {
                for (int iz = 0; iz < 2; ++iz)
                {
                    AddRevolutionCircle(origin,
                                        axisDir,
                                        new Point3D(xVals[ix], yVals[iy], zVals[iz]),
                                        uMin,
                                        uMax,
                                        ref aBox);
                }
            }
        }
    }

    /// <summary>Build the revolution box from a pre-computed basis curve box.</summary>
    private static Box BuildRevolutionBox(in Box curveBox,
                                              in Point3D origin,
                                              in Direction3D axisDir,
                                              double uMin,
                                              double uMax,
                                              double tol)
    {
        Box box = new();
        if (curveBox.IsVoid)
        {
            box.Enlarge(tol);
            return box;
        }

        if (curveBox.IsOpen)
        {
            // Revolve the finite part corners for the bounded portion.
            if (curveBox.HasFinitePart)
            {
                (double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) = curveBox.FinitePart().Get();
                AddRevolvedCorners(xmin, xmax, ymin, ymax, zmin, zmax, origin, axisDir, uMin, uMax, ref box);
            }

            // Propagate curve openness: open directions remain open after revolution.
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
            box.Enlarge(tol);
            return box;
        }

        // Get() returns {Xmin, Xmax, Ymin, Ymax, Zmin, Zmax}.
        (double bXmin, double bXmax, double bYmin, double bYmax, double bZmin, double bZmax) = curveBox.Get();

        // Revolve all 8 corners of the basis curve bounding box.
        AddRevolvedCorners(bXmin, bXmax, bYmin, bYmax, bZmin, bZmax, origin, axisDir, uMin, uMax, ref box);

        box.Enlarge(tol);
        return box;
    }

    /// <summary>Clamps the V range to the basis curve's bounds, as both of OCCT's methods do inline.</summary>
    private static void ClampVRange(Curve3D aBasisCurve, double vMin, double vMax, out double aVMin, out double aVMax)
    {
        double vFirst = aBasisCurve.FirstParameter;
        double vLast = aBasisCurve.LastParameter;
        aVMin = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin) ? vFirst : vMin;
        aVMax = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax) ? vLast : vMax;
        if (!aBasisCurve.IsPeriodic)
        {
            if (aVMin < vFirst)
            {
                aVMin = vFirst;
            }
            else if (aVMin > vLast)
            {
                aVMin = vLast;
            }
            if (aVMax < vFirst)
            {
                aVMax = vFirst;
            }
            else if (aVMax > vLast)
            {
                aVMax = vLast;
            }
            if (aVMin > aVMax)
            {
                double tmp = aVMin;
                aVMin = aVMax;
                aVMax = tmp;
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
            Curve3D basisCurve = _geom.BasisCurve;
            Axis3D axis = _geom.Axis;
            Point3D origin = axis.Location;
            Direction3D axisDir = axis.Direction;

            // Clamp V range to basis curve bounds.
            ClampVRange(basisCurve, vMin, vMax, out double aVMin, out double aVMax);

            // Compute the basis curve bounding box, then revolve all 8 corners around the axis.
            CurveBounds curveEval = new(basisCurve);
            Box curveBox = curveEval.Box(aVMin, aVMax, 0.0);
            if (curveBox.IsVoid)
            {
                SurfaceAdaptor adaptor0 = new(_geom);
                OtherSurfaceBounds other0 = new(adaptor0);
                return other0.Box(uMin, uMax, vMin, vMax, tol);
            }

            return BuildRevolutionBox(curveBox, origin, axisDir, uMin, uMax, tol);
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
            Curve3D basisCurve = _geom.BasisCurve;
            Axis3D axis = _geom.Axis;
            Point3D origin = axis.Location;
            Direction3D axisDir = axis.Direction;

            // Clamp V range to basis curve bounds.
            ClampVRange(basisCurve, vMin, vMax, out double aVMin, out double aVMax);

            // Use the tight basis curve box for a more precise result.
            CurveBounds curveEval = new(basisCurve);
            Box curveBox = curveEval.BoxOptimal(aVMin, aVMax, 0.0);
            if (curveBox.IsVoid)
            {
                SurfaceAdaptor adaptor0 = new(_geom);
                OtherSurfaceBounds other0 = new(adaptor0);
                return other0.BoxOptimal(uMin, uMax, vMin, vMax, tol);
            }

            return BuildRevolutionBox(curveBox, origin, axisDir, uMin, uMax, tol);
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
