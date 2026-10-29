// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An intersection point between two 2D curves: the point, its parameters and transitions on both curves.</summary>
/// <remarks>
/// <c>new IntRes2d_IntersectionPoint()</c> is OCCT's default: the origin, both parameters at
/// <see cref="Standard.RealLast"/> and two default (undecided) transitions; <c>default(IntRes2d_IntersectionPoint)</c>
/// is all zeros. <c>SetValues</c> is an internal mutator; the constructor is its twin.
/// </remarks>
public partial struct IntRes2d_IntersectionPoint : IEquatable<IntRes2d_IntersectionPoint>
{
    private gp_Pnt2d pt;
    private double p1;
    private double p2;
    private IntRes2d_Transition trans1;
    private IntRes2d_Transition trans2;

    /// <summary>Empty constructor.</summary>
    public IntRes2d_IntersectionPoint()
    {
        pt = new gp_Pnt2d();
        p1 = RealLast();
        p2 = RealLast();
        trans1 = new IntRes2d_Transition();
        trans2 = new IntRes2d_Transition();
    }

    /// <summary>
    /// Creates an intersection point. If <paramref name="ReversedFlag"/> is false, <paramref name="Uc1"/> and
    /// <paramref name="Trans1"/> (resp. <paramref name="Uc2"/> and <paramref name="Trans2"/>) refer to the first
    /// curve (resp. the second); otherwise to the second (resp. the first).
    /// </summary>
    public IntRes2d_IntersectionPoint(in gp_Pnt2d P, double Uc1, double Uc2, in IntRes2d_Transition Trans1,
                                      in IntRes2d_Transition Trans2, bool ReversedFlag)
    {
        pt = P;
        p1 = Uc1;
        p2 = Uc2;
        trans1 = Trans1;
        trans2 = Trans2;
        if (ReversedFlag)
        {
            trans1 = Trans2;
            trans2 = Trans1;
            p1 = Uc2;
            p2 = Uc1;
        }
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values of the point; the parameters mean what they mean for the constructor.</summary>
    internal void SetValues(in gp_Pnt2d P, double Uc1, double Uc2, in IntRes2d_Transition Trans1,
                            in IntRes2d_Transition Trans2, bool ReversedFlag)
    {
        pt = P;
        if (!ReversedFlag)
        {
            trans1 = Trans1;
            trans2 = Trans2;
            p1 = Uc1;
            p2 = Uc2;
        }
        else
        {
            trans1 = Trans2;
            trans2 = Trans1;
            p1 = Uc2;
            p2 = Uc1;
        }
    }

    // Queries

    /// <summary>The intersection point in the 2D space.</summary>
    public readonly gp_Pnt2d Value() => pt;

    /// <summary>The parameter on the first curve.</summary>
    public readonly double ParamOnFirst() => p1;

    /// <summary>The parameter on the second curve.</summary>
    public readonly double ParamOnSecond() => p2;

    /// <summary>The transition of the first curve compared to the second one.</summary>
    public readonly IntRes2d_Transition TransitionOfFirst() => trans1;

    /// <summary>The transition of the second curve compared to the first one.</summary>
    public readonly IntRes2d_Transition TransitionOfSecond() => trans2;

    public readonly bool Equals(IntRes2d_IntersectionPoint other)
        => pt.Equals(other.pt) && p1.Equals(other.p1) && p2.Equals(other.p2) && trans1.Equals(other.trans1)
           && trans2.Equals(other.trans2);

    public override readonly bool Equals(object? obj) => obj is IntRes2d_IntersectionPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pt, p1, p2, trans1, trans2);

    public static bool operator ==(in IntRes2d_IntersectionPoint theLeft, in IntRes2d_IntersectionPoint theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in IntRes2d_IntersectionPoint theLeft, in IntRes2d_IntersectionPoint theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"{gp.Format("IntRes2d_IntersectionPoint", pt.X(), pt.Y(), p1, p2)} {trans1} {trans2}";
}
