// Port of: src/FoundationClasses/TKMath/gp/gp_Ax22d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax22d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A coordinate system in 2D space: a location and orthonormal X and Y directions. It is right-handed when
/// X ^ Y is positive, left-handed otherwise.
/// </summary>
/// <remarks><c>new gp_Ax22d()</c> is the global system, as in OCCT; <c>default(gp_Ax22d)</c> is not valid.</remarks>
public partial struct gp_Ax22d : IEquatable<gp_Ax22d>
{
    private gp_Pnt2d point;
    private gp_Dir2d vydir;
    private gp_Dir2d vxdir;

    public gp_Ax22d()
    {
        point = new gp_Pnt2d();
        vydir = new gp_Dir2d(gp_Dir2d.D.Y);
        vxdir = new gp_Dir2d();
    }

    /// <summary>
    /// A system at <paramref name="theP"/> with X direction <paramref name="theVx"/>; the Y direction is normal to it,
    /// on the side of <paramref name="theVy"/>.
    /// </summary>
    public gp_Ax22d(in gp_Pnt2d theP, in gp_Dir2d theVx, in gp_Dir2d theVy)
    {
        point = theP;
        vydir = theVy;
        vxdir = theVx;
        double aValue = theVx.Crossed(theVy);
        if (aValue >= 0.0)
        {
            vydir.SetCoord(-vxdir.Y(), vxdir.X());
        }
        else
        {
            vydir.SetCoord(vxdir.Y(), -vxdir.X());
        }
    }

    /// <summary>A system at <paramref name="theP"/> with X direction <paramref name="theV"/>, right-handed when <paramref name="theIsSense"/> is true.</summary>
    public gp_Ax22d(in gp_Pnt2d theP, in gp_Dir2d theV, bool theIsSense = true)
    {
        point = theP;
        vydir = new gp_Dir2d();
        vxdir = theV;
        if (theIsSense)
        {
            vydir.SetCoord(-theV.Y(), theV.X());
        }
        else
        {
            vydir.SetCoord(theV.Y(), -theV.X());
        }
    }

    /// <summary>A system on the axis <paramref name="theA"/> (its X axis), right-handed when <paramref name="theIsSense"/> is true.</summary>
    public gp_Ax22d(in gp_Ax2d theA, bool theIsSense = true)
    {
        point = theA.Location();
        vydir = new gp_Dir2d();
        vxdir = theA.Direction();
        if (theIsSense)
        {
            vydir.SetCoord(-vxdir.Y(), vxdir.X());
        }
        else
        {
            vydir.SetCoord(vxdir.Y(), -vxdir.X());
        }
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax22d theA1)
    {
        point = theA1.Location();
        vxdir = theA1.XDirection();
        vydir = theA1.YDirection();
    }

    internal void SetXAxis(in gp_Ax2d theA1)
    {
        bool isSign = (vxdir.Crossed(vydir)) >= 0.0;
        point = theA1.Location();
        vxdir = theA1.Direction();
        if (isSign)
        {
            vydir.SetCoord(-vxdir.Y(), vxdir.X());
        }
        else
        {
            vydir.SetCoord(vxdir.Y(), -vxdir.X());
        }
    }

    internal void SetYAxis(in gp_Ax2d theA1)
    {
        bool isSign = (vxdir.Crossed(vydir)) >= 0.0;
        point = theA1.Location();
        vydir = theA1.Direction();
        if (isSign)
        {
            vxdir.SetCoord(vydir.Y(), -vydir.X());
        }
        else
        {
            vxdir.SetCoord(-vydir.Y(), vydir.X());
        }
    }

    internal void SetLocation(in gp_Pnt2d theP) => point = theP;

    internal void SetXDirection(in gp_Dir2d theVx)
    {
        bool isSign = (vxdir.Crossed(vydir)) >= 0.0;
        vxdir = theVx;
        if (isSign)
        {
            vydir.SetCoord(-theVx.Y(), theVx.X());
        }
        else
        {
            vydir.SetCoord(theVx.Y(), -theVx.X());
        }
    }

    internal void SetYDirection(in gp_Dir2d theVy)
    {
        bool isSign = (vxdir.Crossed(vydir)) >= 0.0;
        vydir = theVy;
        if (isSign)
        {
            vxdir.SetCoord(theVy.Y(), -theVy.X());
        }
        else
        {
            vxdir.SetCoord(-theVy.Y(), theVy.X());
        }
    }

    internal void Mirror(in gp_Pnt2d P)
    {
        gp_Pnt2d Temp = point;
        Temp.Mirror(P);
        point = Temp;
        vxdir.Reverse();
        vydir.Reverse();
    }

    internal void Mirror(in gp_Ax2d A1)
    {
        vydir.Mirror(A1);
        vxdir.Mirror(A1);
        gp_Pnt2d Temp = point;
        Temp.Mirror(A1);
        point = Temp;
    }

    internal void Rotate(in gp_Pnt2d theP, double theAng)
    {
        gp_Pnt2d aTemp = point;
        aTemp.Rotate(theP, theAng);
        point = aTemp;
        vxdir.Rotate(theAng);
        vydir.Rotate(theAng);
    }

    internal void Scale(in gp_Pnt2d theP, double theS)
    {
        gp_Pnt2d aTemp = point;
        aTemp.Scale(theP, theS);
        point = aTemp;
        if (theS < 0.0)
        {
            vxdir.Reverse();
            vydir.Reverse();
        }
    }

    internal void Transform(in gp_Trsf2d theT)
    {
        gp_Pnt2d aTemp = point;
        aTemp.Transform(theT);
        point = aTemp;
        vxdir.Transform(theT);
        vydir.Transform(theT);
    }

    internal void Translate(in gp_Vec2d theV) => point.Translate(theV);

    internal void Translate(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => point.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Ax22d WithAxis(in gp_Ax22d theA1)
    {
        gp_Ax22d aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Ax22d WithXAxis(in gp_Ax2d theA1)
    {
        gp_Ax22d aCopy = this;
        aCopy.SetXAxis(theA1);
        return aCopy;
    }

    public readonly gp_Ax22d WithYAxis(in gp_Ax2d theA1)
    {
        gp_Ax22d aCopy = this;
        aCopy.SetYAxis(theA1);
        return aCopy;
    }

    public readonly gp_Ax22d WithLocation(in gp_Pnt2d theP)
    {
        gp_Ax22d aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Ax22d WithXDirection(in gp_Dir2d theVx)
    {
        gp_Ax22d aCopy = this;
        aCopy.SetXDirection(theVx);
        return aCopy;
    }

    public readonly gp_Ax22d WithYDirection(in gp_Dir2d theVy)
    {
        gp_Ax22d aCopy = this;
        aCopy.SetYDirection(theVy);
        return aCopy;
    }

    // Queries

    public readonly gp_Ax2d XAxis() => new(point, vxdir);

    public readonly gp_Ax2d YAxis() => new(point, vydir);

    public readonly gp_Pnt2d Location() => point;

    public readonly gp_Dir2d XDirection() => vxdir;

    public readonly gp_Dir2d YDirection() => vydir;

    public readonly gp_Ax22d Mirrored(in gp_Pnt2d P)
    {
        gp_Ax22d Temp = this;
        Temp.Mirror(P);
        return Temp;
    }

    public readonly gp_Ax22d Mirrored(in gp_Ax2d A1)
    {
        gp_Ax22d Temp = this;
        Temp.Mirror(A1);
        return Temp;
    }

    public readonly gp_Ax22d Rotated(in gp_Pnt2d theP, double theAng)
    {
        gp_Ax22d aTemp = this;
        aTemp.Rotate(theP, theAng);
        return aTemp;
    }

    public readonly gp_Ax22d Scaled(in gp_Pnt2d theP, double theS)
    {
        gp_Ax22d aTemp = this;
        aTemp.Scale(theP, theS);
        return aTemp;
    }

    public readonly gp_Ax22d Transformed(in gp_Trsf2d theT)
    {
        gp_Ax22d aTemp = this;
        aTemp.Transform(theT);
        return aTemp;
    }

    public readonly gp_Ax22d Translated(in gp_Vec2d theV)
    {
        gp_Ax22d aTemp = this;
        aTemp.Translate(theV);
        return aTemp;
    }

    public readonly gp_Ax22d Translated(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        gp_Ax22d aTemp = this;
        aTemp.Translate(theP1, theP2);
        return aTemp;
    }

    public readonly bool Equals(gp_Ax22d other) => point.Equals(other.point) && vydir.Equals(other.vydir) && vxdir.Equals(other.vxdir);

    public override readonly bool Equals(object? obj) => obj is gp_Ax22d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(point, vydir, vxdir);

    public static bool operator ==(in gp_Ax22d theLeft, in gp_Ax22d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Ax22d theLeft, in gp_Ax22d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Ax22d({point}, {vxdir}, {vydir})";
}
