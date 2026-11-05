// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cone.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>The cone primitive.</summary>
internal sealed class PrimitiveCone : Revolution
{
    private double _halfAngle;
    private double _radius;

    /// <summary>
    /// The STEP definition: the semi-angle, the coordinate system, the height and the radius of the truncated face at
    /// z = 0; the apex is on z &lt; 0.
    /// </summary>
    /// <exception cref="DomainErrorException">A null height, a null angle or an angle over π/2.</exception>
    public PrimitiveCone(double angle, in Frame3D position, double height, double radius = 0)
        : base(position, 0, 0)
    {
        _halfAngle = angle;
        _radius = radius;
        if (height < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with null height");
        }
        if (_halfAngle * height < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with null angle");
        }
        if ((M_PI / 2 - _halfAngle) * height < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with angle > PI/2");
        }

        // cut at top
        VMax(height / Math.Cos(_halfAngle));
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>An infinite cone at the origin.</summary>
    public PrimitiveCone(double angle)
        : base(Constants.XOY(), 0, RealLast())
    {
        _halfAngle = angle;
        _radius = 0.0;
        if ((angle < 0) || (angle > M_PI / 2))
        {
            throw new DomainErrorException("cone with angle <0 or > PI/2");
        }
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>An infinite cone at <paramref name="apex"/>.</summary>
    public PrimitiveCone(double angle, in Point3D apex)
        : base(new Frame3D(apex, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), 0, RealLast())
    {
        _halfAngle = angle;
        _radius = 0.0;
        if ((angle < 0) || (angle > M_PI / 2))
        {
            throw new DomainErrorException("cone with angle <0 or > PI/2");
        }
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>An infinite cone on <paramref name="axes"/>.</summary>
    public PrimitiveCone(double angle, in Frame3D axes)
        : base(axes, 0, RealLast())
    {
        // OCCT leaves myRadius uninitialised here (occt-issues.md); the port's field is 0
        _halfAngle = angle;
        if ((angle < 0) || (angle > M_PI / 2))
        {
            throw new DomainErrorException("cone with angle <0 or > PI/2");
        }
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>
    /// A cone at the origin on the Z axis, of height <paramref name="h"/>, radius <paramref name="r1"/> at Z = 0 and
    /// <paramref name="r2"/> at Z = H. If R1 or R2 is 0 there is an apex; otherwise it is a truncated cone.
    /// </summary>
    public PrimitiveCone(double r1, double r2, double h)
        : base(Constants.XOY(), 0, 0)
    {
        SetParameters(r1, r2, h);
        SetMeridian();
    }

    /// <summary>As above, at a given point.</summary>
    public PrimitiveCone(in Point3D center, double r1, double r2, double h)
        : base(new Frame3D(center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), 0, 0)
    {
        SetParameters(r1, r2, h);
        SetMeridian();
    }

    /// <summary>As above, with given axes.</summary>
    public PrimitiveCone(in Frame3D axes, double r1, double r2, double h)
        : base(axes, 0, 0)
    {
        SetParameters(r1, r2, h);
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override Face MakeEmptyLateralFace()
    {
        ConicalSurface c = new(new CoordinateSystem(Axes()), _halfAngle, _radius);
        Face f = new();
        _builder.Builder().MakeFace(ref f, c, global::Kerf.Engine.Foundation.Precision.Confusion());
        return f;
    }

    private void SetMeridian()
    {
        Axis3D a = Axes().Axis;
        a = a.Rotated(new Axis3D(Axes().Location, Axes().YDirection), _halfAngle);
        Vector3D v = new(Axes().XDirection);
        v *= _radius;
        a = a.Translated(v);
        LineCurve3D l = new(a);
        LineCurve2D l2d = new(new Point2D(_radius, 0), new Direction2D(Math.Sin(_halfAngle), Math.Cos(_halfAngle)));
        Meridian(l, l2d);
    }

    private void SetParameters(double r1, double r2, double h)
    {
        if (((r1 != 0) && (r1 < global::Kerf.Engine.Foundation.Precision.Confusion())) || ((r2 != 0) && (r2 < global::Kerf.Engine.Foundation.Precision.Confusion())))
        {
            throw new DomainErrorException("cone with negative or too small radius");
        }
        if (Math.Abs(r1 - r2) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with two identic radii");
        }
        if (h < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with negative or null height");
        }

        _radius = r1;
        _halfAngle = Math.Atan((r2 - r1) / h);

        // cut top and bottom
        VMin(0.0);
        VMax(Math.Sqrt(h * h + (r2 - r1) * (r2 - r1)));
    }
}
