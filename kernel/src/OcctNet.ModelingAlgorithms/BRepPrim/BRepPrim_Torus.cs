// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Torus.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>The torus primitive.</summary>
internal sealed class BRepPrim_Torus : BRepPrim_Revolution
{
    private readonly double myMajor; // distance from the center of the pipe to the center of the torus
    private readonly double myMinor; // radius of the pipe

    /// <summary>The STEP definition: center and axes, major and minor radii.</summary>
    public BRepPrim_Torus(in Frame3D Position, double Major, double Minor)
        : base(Position, 0, 2 * M_PI)
    {
        myMajor = Major;
        myMinor = Minor;
        SetMeridian();
    }

    /// <summary>A torus centered at the origin.</summary>
    public BRepPrim_Torus(double Major, double Minor)
        : base(Constants.XOY(), 0, 2 * M_PI)
    {
        myMajor = Major;
        myMinor = Minor;
        SetMeridian();
    }

    /// <summary>A torus at <paramref name="Center"/>.</summary>
    public BRepPrim_Torus(in Point3D Center, double Major, double Minor)
        : base(new Frame3D(Center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), 0, 2 * M_PI)
    {
        myMajor = Major;
        myMinor = Minor;
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override Face MakeEmptyLateralFace()
    {
        ToroidalSurface T = new(new CoordinateSystem(Axes()), myMajor, myMinor);
        Face F = new();
        myBuilder.Builder().MakeFace(ref F, T, global::Kerf.Engine.Foundation.Precision.Confusion());
        return F;
    }

    private void SetMeridian()
    {
        Direction3D D = Axes().YDirection();
        D.Reverse();
        Frame3D A = new(Axes().Location(), D, Axes().XDirection());
        Vector3D V = new(Axes().XDirection());
        V.Multiply(myMajor);
        A.Translate(V);
        CircleCurve3D C = new(A, myMinor);
        CircleCurve2D C2d = new(new Axis2D(new Point2D(myMajor, 0), new Direction2D(Direction2D.D.X)), myMinor);
        Meridian(C, C2d);
    }
}
