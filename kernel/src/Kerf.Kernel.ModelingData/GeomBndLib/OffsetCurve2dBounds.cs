// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetCurve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 2D offset curve (Geom2d_OffsetCurve): exactly for offsets of lines and circles,
/// otherwise by sampling the offset curve itself (GeomBndLib_OtherCurve2d).
/// </summary>
internal sealed class OffsetCurve2dBounds : Curve2dBounds.ICurve2dEvaluator
{
    private readonly OffsetCurve2D _geom;

    public OffsetCurve2dBounds(OffsetCurve2D curve)
    {
        _geom = curve;
    }

    public OffsetCurve2D Geometry() => _geom;

    Curve2D? Curve2dBounds.ICurve2dEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Box2d BoxOptimal(double tol) => BoxOptimal(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>Try the exact offset-line fast path. Returns a non-void box on success.</summary>
    private static Box2d TryOffsetLine2d(Curve2D basis, OffsetCurve2D off, double u1, double u2, double tol)
    {
        if (basis is not LineCurve2D line)
        {
            return new Box2d();
        }

        Line2D basisLin = line.Line;
        Vector2D normal = new(basisLin.Direction.Y, -basisLin.Direction.X);
        if (normal.SquareMagnitude <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box2d();
        }

        normal = normal.Normalized();
        normal *= off.Offset;
        Point2D loc = basisLin.Location.Translated(normal);
        Box2d localBox = Line2dBounds.Box(new Line2D(loc, basisLin.Direction), u1, u2, 0.0);
        localBox.Enlarge(tol);
        return localBox;
    }

    /// <summary>Try the exact offset-circle fast path. Returns a non-void box on success.</summary>
    private static Box2d TryOffsetCircle2d(Curve2D basis, OffsetCurve2D off, double u1, double u2, double tol)
    {
        if (basis is not CircleCurve2D circle)
        {
            return new Box2d();
        }

        Circle2D basisCirc = circle.Circle;
        double uMid = 0.5 * (u1 + u2);
        global::Kerf.Engine.Geometry.ElCLib.D1(uMid, basisCirc, out Point2D p, out Vector2D v1);
        if (v1.SquareMagnitude <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box2d();
        }

        Vector2D normal = new(v1.Y, -v1.X);
        if (normal.SquareMagnitude <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box2d();
        }
        normal = normal.Normalized();

        Vector2D radial = new(basisCirc.Location, p);
        if (radial.SquareMagnitude <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return new Box2d();
        }
        radial = radial.Normalized();

        double sign = normal.Dot(radial);
        if (Math.Abs(Math.Abs(sign) - 1.0) > 1e-9)
        {
            return new Box2d();
        }

        double newRadius = basisCirc.Radius + off.Offset * sign;
        Box2d localBox = new();
        if (newRadius > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Circle2D offsetCirc = basisCirc;
            offsetCirc = offsetCirc.WithRadius(newRadius);
            localBox = Circle2dBounds.Box(offsetCirc, u1, u2, 0.0);
        }
        else if (Math.Abs(newRadius) <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            localBox.Add(basisCirc.Location);
        }
        else
        {
            return new Box2d();
        }

        localBox.Enlarge(tol);
        return localBox;
    }

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double u1, double u2, double tol)
    {
        Curve2D basis = _geom.BasisCurve;

        Box2d box = TryOffsetLine2d(basis, _geom, u1, u2, tol);
        if (!box.IsVoid)
        {
            return box;
        }

        box = TryOffsetCircle2d(basis, _geom, u1, u2, tol);
        if (!box.IsVoid)
        {
            return box;
        }

        // Generic fallback: sample the actual offset curve for a tight bounding box.
        // This avoids the large over-inflation of basis_box + Enlarge(|offset|).
        CurveAdaptor2D adaptor = new(_geom);
        OtherCurve2dBounds other = new(adaptor);
        return other.Box(u1, u2, tol);
    }

    /// <summary>Compute precise bounding box for arc [theU1, theU2] by sampling the offset curve.</summary>
    public Box2d BoxOptimal(double u1, double u2, double tol)
    {
        // Reuse exact fast paths for line and circle.
        Curve2D basis = _geom.BasisCurve;

        Box2d box = TryOffsetLine2d(basis, _geom, u1, u2, tol);
        if (!box.IsVoid)
        {
            return box;
        }

        box = TryOffsetCircle2d(basis, _geom, u1, u2, tol);
        if (!box.IsVoid)
        {
            return box;
        }

        // Sampling fallback for precise result on general offset curves.
        CurveAdaptor2D adaptor = new(_geom);
        OtherCurve2dBounds other = new(adaptor);
        return other.BoxOptimal(u1, u2, tol);
    }
}
