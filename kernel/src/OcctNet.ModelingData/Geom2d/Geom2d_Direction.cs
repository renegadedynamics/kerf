// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Direction.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Direction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A unit vector in the plane (a shareable <see cref="gp_Dir2d"/>); never null.</summary>
public class Geom2d_Direction : Geom2d_Vector
{
    /// <summary>The unit vector of (X, Y).</summary>
    /// <exception cref="Standard_ConstructionError">sqrt(X*X + Y*Y) is at most <c>gp::Resolution()</c>.</exception>
    public Geom2d_Direction(double X, double Y)
    {
        double D = Math.Sqrt(X * X + Y * Y);
        Standard_ConstructionError_Raise_if(D <= gp.Resolution(),
                                            "Geom2d_Direction() - input vector has zero length");
        gpVec2d = new gp_Vec2d(X / D, Y / D);
    }

    /// <summary>A direction equal to V.</summary>
    public Geom2d_Direction(in gp_Dir2d V) => gpVec2d = new gp_Vec2d(V);

    /// <summary>A new, unfrozen direction equal to this one.</summary>
    public override Geom2d_Geometry Copy()
    {
        Geom2d_Direction D;
        D = new Geom2d_Direction(new gp_Dir2d(gpVec2d));
        return D;
    }

    internal void SetCoord(double X, double Y)
    {
        CheckMutable();
        double D = Math.Sqrt(X * X + Y * Y);
        Standard_ConstructionError_Raise_if(
          D <= gp.Resolution(),
          "Geom2d_Direction::SetCoord() - input vector has zero length");
        gpVec2d = new gp_Vec2d(X / D, Y / D);
    }

    internal void SetDir2d(in gp_Dir2d V)
    {
        CheckMutable();
        gpVec2d = new gp_Vec2d(V);
    }

    internal void SetX(double X)
    {
        CheckMutable();
        double D = Math.Sqrt(X * X + gpVec2d.Y() * gpVec2d.Y());
        Standard_ConstructionError_Raise_if(D <= gp.Resolution(),
                                            "Geom2d_Direction::SetX() - input vector has zero length");
        gpVec2d = new gp_Vec2d(X / D, gpVec2d.Y() / D);
    }

    internal void SetY(double Y)
    {
        CheckMutable();
        double D = Math.Sqrt(gpVec2d.X() * gpVec2d.X() + Y * Y);
        Standard_ConstructionError_Raise_if(D <= gp.Resolution(),
                                            "Geom2d_Direction::SetY() - input vector has zero length");
        gpVec2d = new gp_Vec2d(gpVec2d.X() / D, Y / D);
    }

    /// <summary>The direction as a <see cref="gp_Dir2d"/>.</summary>
    public gp_Dir2d Dir2d() => new gp_Dir2d(gpVec2d);

    /// <summary>1.</summary>
    public override double Magnitude() => 1.0;

    /// <summary>1.</summary>
    public override double SquareMagnitude() => 1.0;

    /// <summary>The cross product this ^ Other.</summary>
    public override double Crossed(Geom2d_Vector Other) => gpVec2d.Crossed(Other.Vec2d());

    /// <summary>The cross product theLeft ^ Other.</summary>
    public static double operator ^(Geom2d_Direction theLeft, Geom2d_Vector Other) => theLeft.Crossed(Other);

    /// <summary>Applies T to this direction, then normalizes it.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        gp_Dir2d dir = new gp_Dir2d(gpVec2d);
        dir.Transform(T);
        gpVec2d = new gp_Vec2d(dir);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom2d_Direction WithCoord(double X, double Y)
    {
        Geom2d_Direction aCopy = (Geom2d_Direction)Copy();
        aCopy.SetCoord(X, Y);
        return aCopy;
    }

    public Geom2d_Direction WithDir2d(in gp_Dir2d V)
    {
        Geom2d_Direction aCopy = (Geom2d_Direction)Copy();
        aCopy.SetDir2d(V);
        return aCopy;
    }

    public Geom2d_Direction WithX(double X)
    {
        Geom2d_Direction aCopy = (Geom2d_Direction)Copy();
        aCopy.SetX(X);
        return aCopy;
    }

    public Geom2d_Direction WithY(double Y)
    {
        Geom2d_Direction aCopy = (Geom2d_Direction)Copy();
        aCopy.SetY(Y);
        return aCopy;
    }
}
