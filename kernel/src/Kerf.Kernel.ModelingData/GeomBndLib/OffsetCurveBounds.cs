// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D offset curve (Geom_OffsetCurve): exactly for offsets of lines and of circles
/// offset in their plane, otherwise the basis curve's box enlarged per coordinate by the offset.
/// </summary>
internal sealed class OffsetCurveBounds : CurveBounds.ICurveEvaluator
{
    private readonly OffsetCurve3D _geom;

    public OffsetCurveBounds(OffsetCurve3D curve)
    {
        _geom = curve;
    }

    public OffsetCurve3D Geometry() => _geom;

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Box BoxOptimal(double tol) => BoxOptimal(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>Try the exact line fast path. Returns a non-void box on success.</summary>
    private static Box TryOffsetLine(Curve3D basis, OffsetCurve3D off, double u1, double u2, double tol)
    {
        if (basis is not LineCurve3D line)
        {
            return new Box();
        }

        Direction3D dir = off.Direction;
        Line3D basisLin = line.Line;
        Vector3D shift = new Vector3D(basisLin.Direction).Crossed(new Vector3D(dir));
        if (shift.SquareMagnitude <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box();
        }

        shift = shift.Normalized();
        shift *= off.Offset;
        Point3D loc = basisLin.Location.Translated(shift);
        Box box = LineBounds.Box(new Line3D(loc, basisLin.Direction), u1, u2, 0.0);
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Try the exact axis-aligned circle fast path. Returns a non-void box on success.</summary>
    private static Box TryOffsetCircle(Curve3D basis, OffsetCurve3D off, double u1, double u2, double tol)
    {
        if (basis is not CircleCurve3D circle)
        {
            return new Box();
        }

        Direction3D dir = off.Direction;
        Circle3D basisCirc = circle.Circle;
        double dotAxis = Math.Abs(basisCirc.Axis.Direction.Dot(dir));
        if (Math.Abs(1.0 - dotAxis) > global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return new Box();
        }

        double uMid = 0.5 * (u1 + u2);
        global::Kerf.Engine.Geometry.ElCLib.D1(uMid, basisCirc, out Point3D p, out Vector3D v1);
        if (v1.SquareMagnitude <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box();
        }

        Vector3D normal = v1.Crossed(new Vector3D(dir));
        if (normal.SquareMagnitude <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box();
        }
        normal = normal.Normalized();

        Vector3D radial = new(basisCirc.Location, p);
        if (radial.SquareMagnitude <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box();
        }
        radial = radial.Normalized();

        double sign = normal.Dot(radial);
        if (Math.Abs(Math.Abs(sign) - 1.0) > 1e-9)
        {
            return new Box();
        }

        double newRadius = basisCirc.Radius + off.Offset * sign;
        Box box = new();
        if (newRadius > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Circle3D offsetCirc = basisCirc;
            offsetCirc = offsetCirc.WithRadius(newRadius);
            box = CircleBounds.Box(offsetCirc, u1, u2, 0.0);
        }
        else if (Math.Abs(newRadius) <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            box.Add(basisCirc.Location);
        }
        else
        {
            return new Box();
        }

        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol)
    {
        // Specialized fast paths for line and axis-aligned circle.
        Curve3D basis = _geom.BasisCurve;

        Box box = TryOffsetLine(basis, _geom, u1, u2, tol);
        if (!box.IsVoid)
        {
            return box;
        }

        box = TryOffsetCircle(basis, _geom, u1, u2, tol);
        if (!box.IsVoid)
        {
            return box;
        }

        // Generic fallback: enlarge basis curve box per coordinate.
        // The offset displacement magnitude per coordinate k is bounded by |d| * sqrt(1 - Vk^2).
        double offset = Math.Abs(_geom.Offset);
        Direction3D dir = _geom.Direction;

        CurveBounds curveEval = new(basis);
        box = curveEval.Box(u1, u2, 0.0);
        if (box.IsVoid)
        {
            return box;
        }

        double dx = dir.X;
        double dy = dir.Y;
        double dz = dir.Z;
        double enlargeX = offset * Math.Sqrt(Max(0.0, 1.0 - dx * dx));
        double enlargeY = offset * Math.Sqrt(Max(0.0, 1.0 - dy * dy));
        double enlargeZ = offset * Math.Sqrt(Max(0.0, 1.0 - dz * dz));

        bool isOpenXmin = box.IsOpenXmin;
        bool isOpenXmax = box.IsOpenXmax;
        bool isOpenYmin = box.IsOpenYmin;
        bool isOpenYmax = box.IsOpenYmax;
        bool isOpenZmin = box.IsOpenZmin;
        bool isOpenZmax = box.IsOpenZmax;

        if (box.HasFinitePart)
        {
            (double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) = box.FinitePart().Get();
            box.Update(xmin - enlargeX,
                        ymin - enlargeY,
                        zmin - enlargeZ,
                        xmax + enlargeX,
                        ymax + enlargeY,
                        zmax + enlargeZ);
        }

        if (isOpenXmin)
        {
            box.OpenXmin();
        }
        if (isOpenXmax)
        {
            box.OpenXmax();
        }
        if (isOpenYmin)
        {
            box.OpenYmin();
        }
        if (isOpenYmax)
        {
            box.OpenYmax();
        }
        if (isOpenZmin)
        {
            box.OpenZmin();
        }
        if (isOpenZmax)
        {
            box.OpenZmax();
        }

        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute precise bounding box for arc [theU1, theU2] by sampling the offset curve.</summary>
    public Box BoxOptimal(double u1, double u2, double tol)
    {
        // Reuse exact fast paths.
        Curve3D basis = _geom.BasisCurve;

        Box box = TryOffsetLine(basis, _geom, u1, u2, tol);
        if (!box.IsVoid)
        {
            return box;
        }

        box = TryOffsetCircle(basis, _geom, u1, u2, tol);
        if (!box.IsVoid)
        {
            return box;
        }

        // Sampling fallback for precise result on general offset curves.
        CurveAdaptor adaptor = new(_geom);
        OtherCurveBounds other = new(adaptor);
        return other.BoxOptimal(u1, u2, tol);
    }
}
