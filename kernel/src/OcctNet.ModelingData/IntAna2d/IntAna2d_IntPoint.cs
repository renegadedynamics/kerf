// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_IntPoint.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_IntPoint.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_IntPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An intersection point between two 2D curves: the point, its parameter on the first curve and, unless the
/// second curve is given by an implicit equation, its parameter on the second curve.
/// </summary>
/// <remarks>
/// <c>new IntAna2d_IntPoint()</c> has every coordinate and parameter at <see cref="Standard.RealLast"/>, as in OCCT;
/// <c>default(IntAna2d_IntPoint)</c> is all zeros. OCCT's virtual <c>SetValue</c> methods are internal mutators
/// (no class derives from IntAna2d_IntPoint); the constructors are their non-mutating twins.
/// </remarks>
public partial struct IntAna2d_IntPoint : IEquatable<IntAna2d_IntPoint>
{
    private double myu1;
    private double myu2;
    private gp_Pnt2d myp;
    private bool myimplicit;

    /// <summary>An intersection point between two parametric 2D curves.</summary>
    public IntAna2d_IntPoint(double X, double Y, double U1, double U2)
    {
        myu1 = U1;
        myu2 = U2;
        myp = new gp_Pnt2d(X, Y);
        myimplicit = false;
    }

    /// <summary>An intersection point between a parametric 2D curve and a curve given by an implicit equation.</summary>
    public IntAna2d_IntPoint(double X, double Y, double U1)
    {
        myu1 = U1;
        myu2 = RealLast();
        myp = new gp_Pnt2d(X, Y);
        myimplicit = true;
    }

    public IntAna2d_IntPoint()
    {
        myu1 = RealLast();
        myu2 = RealLast();
        myp = new gp_Pnt2d(RealLast(), RealLast());
        myimplicit = false;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values for a "non-implicit" point.</summary>
    internal void SetValue(double X, double Y, double U1, double U2)
    {
        myimplicit = false;
        myp.SetCoord(X, Y);
        myu1 = U1;
        myu2 = U2;
    }

    /// <summary>Sets the values for an "implicit" point.</summary>
    internal void SetValue(double X, double Y, double U1)
    {
        myimplicit = true;
        myp.SetCoord(X, Y);
        myu1 = U1;
        myu2 = RealLast();
    }

    // Queries

    /// <summary>The geometric point.</summary>
    public readonly gp_Pnt2d Value() => myp;

    /// <summary>True if the second curve is implicit.</summary>
    public readonly bool SecondIsImplicit() => myimplicit;

    /// <summary>The parameter on the first element.</summary>
    public readonly double ParamOnFirst() => myu1;

    /// <summary>The parameter on the second element.</summary>
    /// <exception cref="Standard_DomainError">The second element is an implicit curve.</exception>
    public readonly double ParamOnSecond()
    {
        if (myimplicit)
        {
            throw new Standard_DomainError();
        }

        return myu2;
    }

    public readonly bool Equals(IntAna2d_IntPoint other)
        => myu1.Equals(other.myu1) && myu2.Equals(other.myu2) && myp.Equals(other.myp) && myimplicit == other.myimplicit;

    public override readonly bool Equals(object? obj) => obj is IntAna2d_IntPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myu1, myu2, myp, myimplicit);

    public static bool operator ==(in IntAna2d_IntPoint theLeft, in IntAna2d_IntPoint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntAna2d_IntPoint theLeft, in IntAna2d_IntPoint theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => myimplicit
               ? gp.Format("IntAna2d_IntPoint", myp.X(), myp.Y(), myu1)
               : gp.Format("IntAna2d_IntPoint", myp.X(), myp.Y(), myu1, myu2);
}
