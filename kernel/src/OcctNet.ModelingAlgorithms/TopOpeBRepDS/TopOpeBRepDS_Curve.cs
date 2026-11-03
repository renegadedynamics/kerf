// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Curve.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A curve of the legacy data structure: a 3D curve with a tolerance and an optional range, the two shapes it lies on
/// with its two surface/curve interferences there (which hold its pcurves), and bookkeeping (kept, mother, index).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. The data structure keeps its own copy (<c>TopOpeBRepDS_DataStructure.AddCurve</c>,
/// which also sets the caller's <see cref="DSIndex"/>, as OCCT does) and gives it for writing by <c>ref</c>
/// (<see cref="TopOpeBRepDS_DataStructure.ChangeCurve"/>).
/// </remarks>
public partial struct TopOpeBRepDS_Curve : IEquatable<TopOpeBRepDS_Curve>
{
    private Geom_Curve? myCurve;
    private double myFirst;
    private double myLast;
    private bool myRangeDefined;
    private double myTolerance;
    private bool myIsWalk;
    private TopoDS_Shape myS1;
    private TopoDS_Shape myS2;
    private TopOpeBRepDS_Interference? mySCI1;
    private TopOpeBRepDS_Interference? mySCI2;
    private bool myKeep;
    private int myMother;
    private int myDSIndex;

    /// <summary>An empty curve with the confusion tolerance, kept.</summary>
    public TopOpeBRepDS_Curve()
    {
        myCurve = null;
        myFirst = 0.0;
        myLast = 0.0;
        myRangeDefined = false;
        myTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        myIsWalk = false;
        myS1 = new TopoDS_Shape();
        myS2 = new TopoDS_Shape();
        mySCI1 = null;
        mySCI2 = null;
        myKeep = true;
        myMother = 0;
        myDSIndex = 0;
    }

    /// <summary>
    /// The curve <paramref name="C"/> with the tolerance <paramref name="T"/>; <paramref name="IsWalk"/> if it comes
    /// from a walking line.
    /// </summary>
    public TopOpeBRepDS_Curve(Geom_Curve? C, double T, bool IsWalk = false)
    {
        myFirst = 0.0;
        myLast = 0.0;
        myRangeDefined = false;
        myS1 = new TopoDS_Shape();
        myS2 = new TopoDS_Shape();
        mySCI1 = null;
        mySCI2 = null;
        myKeep = true;
        myMother = 0;
        myDSIndex = 0;
        DefineCurve(C, T, IsWalk);
    }

    internal void DefineCurve(Geom_Curve? C, double T, bool IsWalk)
    {
        myCurve = C;
        myTolerance = T;
        myIsWalk = IsWalk;
    }

    internal void Tolerance(double T)
    {
        myTolerance = T;
    }

    /// <summary>Sets the surface/curve interferences on the first and second shapes.</summary>
    internal void SetSCI(TopOpeBRepDS_Interference? SCI1, TopOpeBRepDS_Interference? SCI2)
    {
        mySCI1 = SCI1;
        mySCI2 = SCI2;
    }

    /// <summary>The surface/curve interferences on the first and second shapes.</summary>
    public readonly void GetSCI(out TopOpeBRepDS_Interference? theI1, out TopOpeBRepDS_Interference? theI2)
    {
        theI1 = GetSCI1();
        theI2 = GetSCI2();
    }

    /// <summary>The surface/curve interference on the first shape.</summary>
    public readonly TopOpeBRepDS_Interference? GetSCI1() => mySCI1;

    /// <summary>The surface/curve interference on the second shape.</summary>
    public readonly TopOpeBRepDS_Interference? GetSCI2() => mySCI2;

    internal void SetShapes(in TopoDS_Shape S1, in TopoDS_Shape S2)
    {
        myS1 = S1;
        myS2 = S2;
    }

    /// <summary>The two shapes the curve lies on.</summary>
    public readonly void GetShapes(out TopoDS_Shape S1, out TopoDS_Shape S2)
    {
        S1 = myS1;
        S2 = myS2;
    }

    /// <summary>The first shape.</summary>
    public readonly TopoDS_Shape Shape1() => myS1;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref TopoDS_Shape ChangeShape1() => ref myS1;

    /// <summary>The second shape.</summary>
    public readonly TopoDS_Shape Shape2() => myS2;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref TopoDS_Shape ChangeShape2() => ref myS2;

    /// <summary>The 3D curve (null for a degenerated curve).</summary>
    public readonly Geom_Curve? Curve() => myCurve;

    internal void SetRange(double First, double Last)
    {
        myFirst = First;
        myLast = Last;
        myRangeDefined = true;
    }

    /// <summary>Whether the range is defined; if so, its bounds.</summary>
    /// <remarks>OCCT leaves the bounds unchanged when the range is not defined; the port writes 0.</remarks>
    public readonly bool Range(out double First, out double Last)
    {
        First = 0.0;
        Last = 0.0;
        if (myRangeDefined)
        {
            First = myFirst;
            Last = myLast;
        }
        return myRangeDefined;
    }

    /// <summary>The tolerance.</summary>
    public readonly double Tolerance() => myTolerance;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Geom_Curve? ChangeCurve() => ref myCurve;

    internal void Curve(Geom_Curve? C3D, double Tol)
    {
        myCurve = C3D;
        myTolerance = Tol;
    }

    /// <summary>The pcurve on the first shape: that of its surface/curve interference, or null.</summary>
    /// <exception cref="InvalidCastException">The first interference is not a surface/curve interference.</exception>
    public readonly Geom2d_Curve? Curve1()
    {
        if (mySCI1 is not null)
        {
            // OCCT down-casts without checking (and dereferences a null handle on a mismatch); the port's cast throws
            return ((TopOpeBRepDS_SurfaceCurveInterference)mySCI1).PCurve();
        }
        else
        {
            return null;
        }
    }

    internal readonly void Curve1(Geom2d_Curve? PC1)
    {
        if (mySCI1 is not null)
        {
            ((TopOpeBRepDS_SurfaceCurveInterference)mySCI1).PCurve(PC1);
        }
    }

    /// <summary>The pcurve on the second shape: that of its surface/curve interference, or null.</summary>
    /// <exception cref="InvalidCastException">The second interference is not a surface/curve interference.</exception>
    public readonly Geom2d_Curve? Curve2()
    {
        if (mySCI2 is not null)
        {
            return ((TopOpeBRepDS_SurfaceCurveInterference)mySCI2).PCurve();
        }
        else
        {
            return null;
        }
    }

    internal readonly void Curve2(Geom2d_Curve? PC2)
    {
        if (mySCI2 is not null)
        {
            ((TopOpeBRepDS_SurfaceCurveInterference)mySCI2).PCurve(PC2);
        }
    }

    /// <summary>Whether the curve comes from a walking line.</summary>
    public readonly bool IsWalk() => myIsWalk;

    internal void ChangeIsWalk(bool B)
    {
        myIsWalk = B;
    }

    /// <summary>Whether the curve is kept (not removed from the data structure).</summary>
    public readonly bool Keep() => myKeep;

    internal void ChangeKeep(bool b)
    {
        myKeep = b;
    }

    /// <summary>The index of the curve this one was made from (<see cref="TopOpeBRepDS_HDataStructure.MakeCurve"/>), or 0.</summary>
    public readonly int Mother() => myMother;

    internal void ChangeMother(int b)
    {
        myMother = b;
    }

    /// <summary>The curve's index in the data structure (set when it is added), or 0.</summary>
    public readonly int DSIndex() => myDSIndex;

    internal void ChangeDSIndex(int b)
    {
        myDSIndex = b;
    }

    /// <summary>A copy with the range [<paramref name="First"/>, <paramref name="Last"/>].</summary>
    public readonly TopOpeBRepDS_Curve WithRange(double First, double Last)
    {
        TopOpeBRepDS_Curve aCopy = this;
        aCopy.SetRange(First, Last);
        return aCopy;
    }

    /// <summary>A copy with the tolerance <paramref name="T"/>.</summary>
    public readonly TopOpeBRepDS_Curve WithTolerance(double T)
    {
        TopOpeBRepDS_Curve aCopy = this;
        aCopy.Tolerance(T);
        return aCopy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(TopOpeBRepDS_Curve other)
        => ReferenceEquals(myCurve, other.myCurve) && myFirst.Equals(other.myFirst) && myLast.Equals(other.myLast)
           && myRangeDefined == other.myRangeDefined && myTolerance.Equals(other.myTolerance)
           && myIsWalk == other.myIsWalk && myS1.Equals(other.myS1) && myS2.Equals(other.myS2)
           && ReferenceEquals(mySCI1, other.mySCI1) && ReferenceEquals(mySCI2, other.mySCI2)
           && myKeep == other.myKeep && myMother == other.myMother && myDSIndex == other.myDSIndex;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is TopOpeBRepDS_Curve other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode()
        => HashCode.Combine(myCurve is null ? 0 : System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(myCurve),
                            myFirst, myLast, myTolerance, myS1, myS2, myMother, myDSIndex);

    public static bool operator ==(in TopOpeBRepDS_Curve theLeft, in TopOpeBRepDS_Curve theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in TopOpeBRepDS_Curve theLeft, in TopOpeBRepDS_Curve theRight) => !theLeft.Equals(theRight);

    /// <inheritdoc/>
    public override readonly string ToString()
        => "TopOpeBRepDS_Curve(" + (myCurve is null ? "null" : global::Kerf.Engine.Foundation.OcctTypeNames.Of(myCurve.GetType())) + ", " + myTolerance.ToString("R", System.Globalization.CultureInfo.InvariantCulture)
           + (myRangeDefined ? ", [" + myFirst.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", " + myLast.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + "]" : "")
           + ", index " + myDSIndex.ToString(System.Globalization.CultureInfo.InvariantCulture) + ")";
}
