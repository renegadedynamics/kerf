// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Transition.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Transition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// How an interference separates the geometry it lies on: the state before and after it (IN, OUT, ON, UNKNOWN), the
/// kind of shape on each side, and the data structure's indices of the shapes before and after.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <c>default</c> is not OCCT's default: <c>new()</c> is (UNKNOWN, UNKNOWN, FACE,
/// FACE, indices 0).
/// </remarks>
public partial struct TopOpeBRepDS_Transition : IEquatable<TopOpeBRepDS_Transition>
{
    private Containment myStateBefore;
    private Containment myStateAfter;
    private ShapeType myShapeBefore;
    private ShapeType myShapeAfter;
    private int myIndexBefore;
    private int myIndexAfter;

    /// <summary>An unknown transition: UNKNOWN before and after, on faces, indices 0.</summary>
    public TopOpeBRepDS_Transition()
    {
        myStateBefore = Unknown;
        myStateAfter = Unknown;
        myShapeBefore = ShapeType.Face;
        myShapeAfter = ShapeType.Face;
        myIndexBefore = 0;
        myIndexAfter = 0;
    }

    /// <summary>The transition from <paramref name="SB"/> to <paramref name="SA"/>, on shapes of the given kinds.</summary>
    public TopOpeBRepDS_Transition(Containment SB,
                                   Containment SA,
                                   ShapeType ONB = ShapeType.Face,
                                   ShapeType ONA = ShapeType.Face)
    {
        myStateBefore = SB;
        myStateAfter = SA;
        myShapeBefore = ONB;
        myShapeAfter = ONA;
        myIndexBefore = 0;
        myIndexAfter = 0;
    }

    /// <summary>The transition of the orientation <paramref name="O"/> (see <see cref="Set(Orientation)"/>).</summary>
    public TopOpeBRepDS_Transition(Orientation O)
    {
        // OCCT leaves the states unset for an orientation outside the four; the port's are IN (C#'s zero)
        myShapeBefore = ShapeType.Face;
        myShapeAfter = ShapeType.Face;
        myIndexBefore = 0;
        myIndexAfter = 0;
        Set(O);
    }

    /// <summary>Sets the states and the kinds of shape before and after.</summary>
    internal void Set(Containment SB,
                      Containment SA,
                      ShapeType ONB = ShapeType.Face,
                      ShapeType ONA = ShapeType.Face)
    {
        myStateBefore = SB;
        myStateAfter = SA;
        myShapeBefore = ONB;
        myShapeAfter = ONA;
    }

    internal void StateBefore(Containment S)
    {
        myStateBefore = S;
    }

    internal void StateAfter(Containment S)
    {
        myStateAfter = S;
    }

    internal void ShapeBefore(ShapeType SE)
    {
        myShapeBefore = SE;
    }

    internal void ShapeAfter(ShapeType SE)
    {
        myShapeAfter = SE;
    }

    internal void Before(Containment S, ShapeType ONB = ShapeType.Face)
    {
        myStateBefore = S;
        myShapeBefore = ONB;
    }

    internal void After(Containment S, ShapeType ONA = ShapeType.Face)
    {
        myStateAfter = S;
        myShapeAfter = ONA;
    }

    /// <summary>Sets the indices before and after to <paramref name="I"/>.</summary>
    internal void Index(int I)
    {
        myIndexBefore = myIndexAfter = I;
    }

    internal void IndexBefore(int I)
    {
        myIndexBefore = I;
    }

    internal void IndexAfter(int I)
    {
        myIndexAfter = I;
    }

    /// <summary>The state before.</summary>
    public readonly Containment Before() => myStateBefore;

    /// <summary>The kind of shape before.</summary>
    public readonly ShapeType ONBefore() => myShapeBefore;

    /// <summary>The state after.</summary>
    public readonly Containment After() => myStateAfter;

    /// <summary>The kind of shape after.</summary>
    public readonly ShapeType ONAfter() => myShapeAfter;

    /// <summary>The kind of shape before.</summary>
    public readonly ShapeType ShapeBefore() => myShapeBefore;

    /// <summary>The kind of shape after.</summary>
    public readonly ShapeType ShapeAfter() => myShapeAfter;

    /// <summary>The index of the shape before and after.</summary>
    /// <exception cref="FailureException">The indices before and after differ.</exception>
    public readonly int Index()
    {
        if (myIndexAfter != myIndexBefore)
        {
            throw new FailureException("Transition::Index() on different shapes");
        }
        return myIndexBefore;
    }

    /// <summary>The index of the shape before.</summary>
    public readonly int IndexBefore() => myIndexBefore;

    /// <summary>The index of the shape after.</summary>
    public readonly int IndexAfter() => myIndexAfter;

    /// <summary>
    /// Sets the states of the orientation <paramref name="O"/>: FORWARD is (OUT, IN), REVERSED (IN, OUT), INTERNAL
    /// (IN, IN), EXTERNAL (OUT, OUT).
    /// </summary>
    internal void Set(Orientation O)
    {
        switch (O)
        {
            case Forward:
                myStateBefore = Out;
                myStateAfter = In;
                break;
            case Reversed:
                myStateBefore = In;
                myStateAfter = Out;
                break;
            case Internal:
                myStateBefore = In;
                myStateAfter = In;
                break;
            case External:
                myStateBefore = Out;
                myStateAfter = Out;
                break;
        }
    }

    /// <summary>
    /// The orientation corresponding to the state <paramref name="S"/>: REVERSED when leaving it, FORWARD when
    /// entering it, INTERNAL when staying in it, EXTERNAL when staying outside it (with ON states, see OCCT).
    /// </summary>
    public readonly Orientation Orientation(Containment S, ShapeType T = ShapeType.Face)
    {
        if (myStateBefore == On || myStateAfter == On)
        {
            return OrientationON(S, T);
        }
        else
        {
            if (myStateBefore == S)
            {
                if (myStateAfter == S)
                {
                    return Internal;
                }
                else
                {
                    return Reversed;
                }
            }
            else
            {
                if (myStateAfter == S)
                {
                    return Forward;
                }
                else
                {
                    return External;
                }
            }
        }
    }

    // the orientation corresponding to state S, when at least one of the states is ON
    private readonly Orientation OrientationON(Containment S, ShapeType T)
    {
        _ = T;
        Orientation result = Forward;

        if (myStateBefore == On && myStateAfter == On)
        {
            if (S == In)
            {
                result = Internal;
            }
            else if (S == Out)
            {
                result = External;
            }
            else if (S == On)
            {
                result = Internal;
            }
        }
        else if (myStateBefore == On)
        {
            if (myStateAfter == S)
            {
                return Forward;
            }
            else
            {
                return Reversed;
            }
        }
        else if (myStateAfter == On)
        {
            if (myStateBefore == S)
            {
                return Reversed;
            }
            else
            {
                return Forward;
            }
        }

        return result;
    }

    /// <summary>The complementary transition (IN and OUT exchanged; with an ON state, before and after exchanged).</summary>
    public readonly TopOpeBRepDS_Transition Complement()
    {
        TopOpeBRepDS_Transition T = new();
        T.myIndexBefore = myIndexBefore;
        T.myIndexAfter = myIndexAfter;

        // xpu : 14-01-98
        if (myStateBefore == Unknown && myStateAfter == Unknown)
        {
            T.Set(myStateAfter, myStateBefore, myShapeAfter, myShapeBefore);
            return T;
        }
        // xpu : 14-01-98

        if (myStateBefore == On || myStateAfter == On)
        {
            T.Set(myStateAfter, myStateBefore, myShapeAfter, myShapeBefore);
        }
        else
        {
            Orientation o = Orientation(In);
            if (o == Forward)
            { // (OUT,IN) --> (IN,OUT)
                T.Set(In, Out, myShapeBefore, myShapeAfter);
            }
            else if (o == Reversed)
            { // (IN,OUT) --> (OUT,IN)
                T.Set(Out, In, myShapeBefore, myShapeAfter);
            }
            else if (o == External)
            { // (OUT,OUT) --> (IN,IN)
                T.Set(In, In, myShapeBefore, myShapeAfter);
            }
            else if (o == Internal)
            { // (IN,IN) --> (OUT,OUT)
                T.Set(Out, Out, myShapeBefore, myShapeAfter);
            }
        }

        return T;
    }

    /// <summary>Whether both states are UNKNOWN.</summary>
    public readonly bool IsUnknown() => (myStateBefore == Unknown) && (myStateAfter == Unknown);

    /// <summary>A copy with the indices before and after set to <paramref name="I"/>.</summary>
    public readonly TopOpeBRepDS_Transition WithIndex(int I)
    {
        TopOpeBRepDS_Transition aCopy = this;
        aCopy.Index(I);
        return aCopy;
    }

    /// <summary>A copy with the states and kinds of shape set.</summary>
    public readonly TopOpeBRepDS_Transition WithSet(Containment SB,
                                                    Containment SA,
                                                    ShapeType ONB = ShapeType.Face,
                                                    ShapeType ONA = ShapeType.Face)
    {
        TopOpeBRepDS_Transition aCopy = this;
        aCopy.Set(SB, SA, ONB, ONA);
        return aCopy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(TopOpeBRepDS_Transition other)
        => myStateBefore == other.myStateBefore && myStateAfter == other.myStateAfter
           && myShapeBefore == other.myShapeBefore && myShapeAfter == other.myShapeAfter
           && myIndexBefore == other.myIndexBefore && myIndexAfter == other.myIndexAfter;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is TopOpeBRepDS_Transition other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode()
        => HashCode.Combine(myStateBefore, myStateAfter, myShapeBefore, myShapeAfter, myIndexBefore, myIndexAfter);

    public static bool operator ==(in TopOpeBRepDS_Transition theLeft, in TopOpeBRepDS_Transition theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in TopOpeBRepDS_Transition theLeft, in TopOpeBRepDS_Transition theRight)
        => !theLeft.Equals(theRight);

    /// <summary>The states, kinds and indices, as in the data structure's text dump (<c>OUT/IN FACE/FACE 0/0</c>).</summary>
    public override readonly string ToString() => TopOpeBRepDS_TextDump.TransitionText(this);
}
