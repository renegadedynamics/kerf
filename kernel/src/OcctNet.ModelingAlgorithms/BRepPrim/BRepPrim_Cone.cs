// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cone.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>The cone primitive.</summary>
internal sealed class BRepPrim_Cone : BRepPrim_Revolution
{
    private double myHalfAngle;
    private double myRadius;

    /// <summary>
    /// The STEP definition: the semi-angle, the coordinate system, the height and the radius of the truncated face at
    /// z = 0; the apex is on z &lt; 0.
    /// </summary>
    /// <exception cref="DomainErrorException">A null height, a null angle or an angle over π/2.</exception>
    public BRepPrim_Cone(double Angle, in Frame3D Position, double Height, double Radius = 0)
        : base(Position, 0, 0)
    {
        myHalfAngle = Angle;
        myRadius = Radius;
        if (Height < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with null height");
        }
        if (myHalfAngle * Height < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with null angle");
        }
        if ((M_PI / 2 - myHalfAngle) * Height < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with angle > PI/2");
        }

        // cut at top
        VMax(Height / Math.Cos(myHalfAngle));
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>An infinite cone at the origin.</summary>
    public BRepPrim_Cone(double Angle)
        : base(Constants.XOY(), 0, RealLast())
    {
        myHalfAngle = Angle;
        myRadius = 0.0;
        if ((Angle < 0) || (Angle > M_PI / 2))
        {
            throw new DomainErrorException("cone with angle <0 or > PI/2");
        }
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>An infinite cone at <paramref name="Apex"/>.</summary>
    public BRepPrim_Cone(double Angle, in Point3D Apex)
        : base(new Frame3D(Apex, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), 0, RealLast())
    {
        myHalfAngle = Angle;
        myRadius = 0.0;
        if ((Angle < 0) || (Angle > M_PI / 2))
        {
            throw new DomainErrorException("cone with angle <0 or > PI/2");
        }
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>An infinite cone on <paramref name="Axes"/>.</summary>
    public BRepPrim_Cone(double Angle, in Frame3D Axes)
        : base(Axes, 0, RealLast())
    {
        // OCCT leaves myRadius uninitialised here (occt-issues.md); the port's field is 0
        myHalfAngle = Angle;
        if ((Angle < 0) || (Angle > M_PI / 2))
        {
            throw new DomainErrorException("cone with angle <0 or > PI/2");
        }
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>
    /// A cone at the origin on the Z axis, of height <paramref name="H"/>, radius <paramref name="R1"/> at Z = 0 and
    /// <paramref name="R2"/> at Z = H. If R1 or R2 is 0 there is an apex; otherwise it is a truncated cone.
    /// </summary>
    public BRepPrim_Cone(double R1, double R2, double H)
        : base(Constants.XOY(), 0, 0)
    {
        SetParameters(R1, R2, H);
        SetMeridian();
    }

    /// <summary>As above, at a given point.</summary>
    public BRepPrim_Cone(in Point3D Center, double R1, double R2, double H)
        : base(new Frame3D(Center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), 0, 0)
    {
        SetParameters(R1, R2, H);
        SetMeridian();
    }

    /// <summary>As above, with given axes.</summary>
    public BRepPrim_Cone(in Frame3D Axes, double R1, double R2, double H)
        : base(Axes, 0, 0)
    {
        SetParameters(R1, R2, H);
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override TopoDS_Face MakeEmptyLateralFace()
    {
        Geom_ConicalSurface C = new(new CoordinateSystem(Axes()), myHalfAngle, myRadius);
        TopoDS_Face F = new();
        myBuilder.Builder().MakeFace(ref F, C, global::Kerf.Engine.Foundation.Precision.Confusion());
        return F;
    }

    private void SetMeridian()
    {
        Axis3D A = Axes().Axis();
        A.Rotate(new Axis3D(Axes().Location(), Axes().YDirection()), myHalfAngle);
        Vector3D V = new(Axes().XDirection());
        V *= myRadius;
        A.Translate(V);
        Geom_Line L = new(A);
        Geom2d_Line L2d = new(new Point2D(myRadius, 0), new Direction2D(Math.Sin(myHalfAngle), Math.Cos(myHalfAngle)));
        Meridian(L, L2d);
    }

    private void SetParameters(double R1, double R2, double H)
    {
        if (((R1 != 0) && (R1 < global::Kerf.Engine.Foundation.Precision.Confusion())) || ((R2 != 0) && (R2 < global::Kerf.Engine.Foundation.Precision.Confusion())))
        {
            throw new DomainErrorException("cone with negative or too small radius");
        }
        if (Math.Abs(R1 - R2) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with two identic radii");
        }
        if (H < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new DomainErrorException("cone with negative or null height");
        }

        myRadius = R1;
        myHalfAngle = Math.Atan((R2 - R1) / H);

        // cut top and bottom
        VMin(0.0);
        VMax(Math.Sqrt(H * H + (R2 - R1) * (R2 - R1)));
    }
}
