// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Transition.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Transition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

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
    private TopAbs_State myStateBefore;
    private TopAbs_State myStateAfter;
    private TopAbs_ShapeEnum myShapeBefore;
    private TopAbs_ShapeEnum myShapeAfter;
    private int myIndexBefore;
    private int myIndexAfter;

    /// <summary>An unknown transition: UNKNOWN before and after, on faces, indices 0.</summary>
    public TopOpeBRepDS_Transition()
    {
        myStateBefore = TopAbs_UNKNOWN;
        myStateAfter = TopAbs_UNKNOWN;
        myShapeBefore = TopAbs_FACE;
        myShapeAfter = TopAbs_FACE;
        myIndexBefore = 0;
        myIndexAfter = 0;
    }

    /// <summary>The transition from <paramref name="SB"/> to <paramref name="SA"/>, on shapes of the given kinds.</summary>
    public TopOpeBRepDS_Transition(TopAbs_State SB,
                                   TopAbs_State SA,
                                   TopAbs_ShapeEnum ONB = TopAbs_FACE,
                                   TopAbs_ShapeEnum ONA = TopAbs_FACE)
    {
        myStateBefore = SB;
        myStateAfter = SA;
        myShapeBefore = ONB;
        myShapeAfter = ONA;
        myIndexBefore = 0;
        myIndexAfter = 0;
    }

    /// <summary>The transition of the orientation <paramref name="O"/> (see <see cref="Set(TopAbs_Orientation)"/>).</summary>
    public TopOpeBRepDS_Transition(TopAbs_Orientation O)
    {
        // OCCT leaves the states unset for an orientation outside the four; the port's are IN (C#'s zero)
        myShapeBefore = TopAbs_FACE;
        myShapeAfter = TopAbs_FACE;
        myIndexBefore = 0;
        myIndexAfter = 0;
        Set(O);
    }

    /// <summary>Sets the states and the kinds of shape before and after.</summary>
    internal void Set(TopAbs_State SB,
                      TopAbs_State SA,
                      TopAbs_ShapeEnum ONB = TopAbs_FACE,
                      TopAbs_ShapeEnum ONA = TopAbs_FACE)
    {
        myStateBefore = SB;
        myStateAfter = SA;
        myShapeBefore = ONB;
        myShapeAfter = ONA;
    }

    internal void StateBefore(TopAbs_State S)
    {
        myStateBefore = S;
    }

    internal void StateAfter(TopAbs_State S)
    {
        myStateAfter = S;
    }

    internal void ShapeBefore(TopAbs_ShapeEnum SE)
    {
        myShapeBefore = SE;
    }

    internal void ShapeAfter(TopAbs_ShapeEnum SE)
    {
        myShapeAfter = SE;
    }

    internal void Before(TopAbs_State S, TopAbs_ShapeEnum ONB = TopAbs_FACE)
    {
        myStateBefore = S;
        myShapeBefore = ONB;
    }

    internal void After(TopAbs_State S, TopAbs_ShapeEnum ONA = TopAbs_FACE)
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
    public readonly TopAbs_State Before() => myStateBefore;

    /// <summary>The kind of shape before.</summary>
    public readonly TopAbs_ShapeEnum ONBefore() => myShapeBefore;

    /// <summary>The state after.</summary>
    public readonly TopAbs_State After() => myStateAfter;

    /// <summary>The kind of shape after.</summary>
    public readonly TopAbs_ShapeEnum ONAfter() => myShapeAfter;

    /// <summary>The kind of shape before.</summary>
    public readonly TopAbs_ShapeEnum ShapeBefore() => myShapeBefore;

    /// <summary>The kind of shape after.</summary>
    public readonly TopAbs_ShapeEnum ShapeAfter() => myShapeAfter;

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
    internal void Set(TopAbs_Orientation O)
    {
        switch (O)
        {
            case TopAbs_FORWARD:
                myStateBefore = TopAbs_OUT;
                myStateAfter = TopAbs_IN;
                break;
            case TopAbs_REVERSED:
                myStateBefore = TopAbs_IN;
                myStateAfter = TopAbs_OUT;
                break;
            case TopAbs_INTERNAL:
                myStateBefore = TopAbs_IN;
                myStateAfter = TopAbs_IN;
                break;
            case TopAbs_EXTERNAL:
                myStateBefore = TopAbs_OUT;
                myStateAfter = TopAbs_OUT;
                break;
        }
    }

    /// <summary>
    /// The orientation corresponding to the state <paramref name="S"/>: REVERSED when leaving it, FORWARD when
    /// entering it, INTERNAL when staying in it, EXTERNAL when staying outside it (with ON states, see OCCT).
    /// </summary>
    public readonly TopAbs_Orientation Orientation(TopAbs_State S, TopAbs_ShapeEnum T = TopAbs_FACE)
    {
        if (myStateBefore == TopAbs_ON || myStateAfter == TopAbs_ON)
        {
            return OrientationON(S, T);
        }
        else
        {
            if (myStateBefore == S)
            {
                if (myStateAfter == S)
                {
                    return TopAbs_INTERNAL;
                }
                else
                {
                    return TopAbs_REVERSED;
                }
            }
            else
            {
                if (myStateAfter == S)
                {
                    return TopAbs_FORWARD;
                }
                else
                {
                    return TopAbs_EXTERNAL;
                }
            }
        }
    }

    // the orientation corresponding to state S, when at least one of the states is ON
    private readonly TopAbs_Orientation OrientationON(TopAbs_State S, TopAbs_ShapeEnum T)
    {
        _ = T;
        TopAbs_Orientation result = TopAbs_FORWARD;

        if (myStateBefore == TopAbs_ON && myStateAfter == TopAbs_ON)
        {
            if (S == TopAbs_IN)
            {
                result = TopAbs_INTERNAL;
            }
            else if (S == TopAbs_OUT)
            {
                result = TopAbs_EXTERNAL;
            }
            else if (S == TopAbs_ON)
            {
                result = TopAbs_INTERNAL;
            }
        }
        else if (myStateBefore == TopAbs_ON)
        {
            if (myStateAfter == S)
            {
                return TopAbs_FORWARD;
            }
            else
            {
                return TopAbs_REVERSED;
            }
        }
        else if (myStateAfter == TopAbs_ON)
        {
            if (myStateBefore == S)
            {
                return TopAbs_REVERSED;
            }
            else
            {
                return TopAbs_FORWARD;
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
        if (myStateBefore == TopAbs_UNKNOWN && myStateAfter == TopAbs_UNKNOWN)
        {
            T.Set(myStateAfter, myStateBefore, myShapeAfter, myShapeBefore);
            return T;
        }
        // xpu : 14-01-98

        if (myStateBefore == TopAbs_ON || myStateAfter == TopAbs_ON)
        {
            T.Set(myStateAfter, myStateBefore, myShapeAfter, myShapeBefore);
        }
        else
        {
            TopAbs_Orientation o = Orientation(TopAbs_IN);
            if (o == TopAbs_FORWARD)
            { // (OUT,IN) --> (IN,OUT)
                T.Set(TopAbs_IN, TopAbs_OUT, myShapeBefore, myShapeAfter);
            }
            else if (o == TopAbs_REVERSED)
            { // (IN,OUT) --> (OUT,IN)
                T.Set(TopAbs_OUT, TopAbs_IN, myShapeBefore, myShapeAfter);
            }
            else if (o == TopAbs_EXTERNAL)
            { // (OUT,OUT) --> (IN,IN)
                T.Set(TopAbs_IN, TopAbs_IN, myShapeBefore, myShapeAfter);
            }
            else if (o == TopAbs_INTERNAL)
            { // (IN,IN) --> (OUT,OUT)
                T.Set(TopAbs_OUT, TopAbs_OUT, myShapeBefore, myShapeAfter);
            }
        }

        return T;
    }

    /// <summary>Whether both states are UNKNOWN.</summary>
    public readonly bool IsUnknown() => (myStateBefore == TopAbs_UNKNOWN) && (myStateAfter == TopAbs_UNKNOWN);

    /// <summary>A copy with the indices before and after set to <paramref name="I"/>.</summary>
    public readonly TopOpeBRepDS_Transition WithIndex(int I)
    {
        TopOpeBRepDS_Transition aCopy = this;
        aCopy.Index(I);
        return aCopy;
    }

    /// <summary>A copy with the states and kinds of shape set.</summary>
    public readonly TopOpeBRepDS_Transition WithSet(TopAbs_State SB,
                                                    TopAbs_State SA,
                                                    TopAbs_ShapeEnum ONB = TopAbs_FACE,
                                                    TopAbs_ShapeEnum ONA = TopAbs_FACE)
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
