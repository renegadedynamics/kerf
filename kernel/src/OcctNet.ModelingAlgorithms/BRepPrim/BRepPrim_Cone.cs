// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cone.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    /// <exception cref="Standard_DomainError">A null height, a null angle or an angle over π/2.</exception>
    public BRepPrim_Cone(double Angle, in gp_Ax2 Position, double Height, double Radius = 0)
        : base(Position, 0, 0)
    {
        myHalfAngle = Angle;
        myRadius = Radius;
        if (Height < Precision.Confusion())
        {
            throw new Standard_DomainError("cone with null height");
        }
        if (myHalfAngle * Height < Precision.Confusion())
        {
            throw new Standard_DomainError("cone with null angle");
        }
        if ((M_PI / 2 - myHalfAngle) * Height < Precision.Confusion())
        {
            throw new Standard_DomainError("cone with angle > PI/2");
        }

        // cut at top
        VMax(Height / Math.Cos(myHalfAngle));
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>An infinite cone at the origin.</summary>
    public BRepPrim_Cone(double Angle)
        : base(gp.XOY(), 0, RealLast())
    {
        myHalfAngle = Angle;
        myRadius = 0.0;
        if ((Angle < 0) || (Angle > M_PI / 2))
        {
            throw new Standard_DomainError("cone with angle <0 or > PI/2");
        }
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>An infinite cone at <paramref name="Apex"/>.</summary>
    public BRepPrim_Cone(double Angle, in gp_Pnt Apex)
        : base(new gp_Ax2(Apex, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), 0, RealLast())
    {
        myHalfAngle = Angle;
        myRadius = 0.0;
        if ((Angle < 0) || (Angle > M_PI / 2))
        {
            throw new Standard_DomainError("cone with angle <0 or > PI/2");
        }
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>An infinite cone on <paramref name="Axes"/>.</summary>
    public BRepPrim_Cone(double Angle, in gp_Ax2 Axes)
        : base(Axes, 0, RealLast())
    {
        // OCCT leaves myRadius uninitialised here (occt-issues.md); the port's field is 0
        myHalfAngle = Angle;
        if ((Angle < 0) || (Angle > M_PI / 2))
        {
            throw new Standard_DomainError("cone with angle <0 or > PI/2");
        }
        VMin(0.0);
        SetMeridian();
    }

    /// <summary>
    /// A cone at the origin on the Z axis, of height <paramref name="H"/>, radius <paramref name="R1"/> at Z = 0 and
    /// <paramref name="R2"/> at Z = H. If R1 or R2 is 0 there is an apex; otherwise it is a truncated cone.
    /// </summary>
    public BRepPrim_Cone(double R1, double R2, double H)
        : base(gp.XOY(), 0, 0)
    {
        SetParameters(R1, R2, H);
        SetMeridian();
    }

    /// <summary>As above, at a given point.</summary>
    public BRepPrim_Cone(in gp_Pnt Center, double R1, double R2, double H)
        : base(new gp_Ax2(Center, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), 0, 0)
    {
        SetParameters(R1, R2, H);
        SetMeridian();
    }

    /// <summary>As above, with given axes.</summary>
    public BRepPrim_Cone(in gp_Ax2 Axes, double R1, double R2, double H)
        : base(Axes, 0, 0)
    {
        SetParameters(R1, R2, H);
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override TopoDS_Face MakeEmptyLateralFace()
    {
        Geom_ConicalSurface C = new(new gp_Ax3(Axes()), myHalfAngle, myRadius);
        TopoDS_Face F = new();
        myBuilder.Builder().MakeFace(ref F, C, Precision.Confusion());
        return F;
    }

    private void SetMeridian()
    {
        gp_Ax1 A = Axes().Axis();
        A.Rotate(new gp_Ax1(Axes().Location(), Axes().YDirection()), myHalfAngle);
        gp_Vec V = new(Axes().XDirection());
        V *= myRadius;
        A.Translate(V);
        Geom_Line L = new(A);
        Geom2d_Line L2d = new(new gp_Pnt2d(myRadius, 0), new gp_Dir2d(Math.Sin(myHalfAngle), Math.Cos(myHalfAngle)));
        Meridian(L, L2d);
    }

    private void SetParameters(double R1, double R2, double H)
    {
        if (((R1 != 0) && (R1 < Precision.Confusion())) || ((R2 != 0) && (R2 < Precision.Confusion())))
        {
            throw new Standard_DomainError("cone with negative or too small radius");
        }
        if (Math.Abs(R1 - R2) < Precision.Confusion())
        {
            throw new Standard_DomainError("cone with two identic radii");
        }
        if (H < Precision.Confusion())
        {
            throw new Standard_DomainError("cone with negative or null height");
        }

        myRadius = R1;
        myHalfAngle = Math.Atan((R2 - R1) / H);

        // cut top and bottom
        VMin(0.0);
        VMax(Math.Sqrt(H * H + (R2 - R1) * (R2 - R1)));
    }
}
