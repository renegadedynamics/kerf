// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Transition.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Transition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>
/// How an interference separates the geometry it lies on: the state before and after it (IN, OUT, ON, UNKNOWN), the
/// kind of shape on each side, and the data structure's indices of the shapes before and after.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <c>default</c> is not OCCT's default: <c>new()</c> is (UNKNOWN, UNKNOWN, FACE,
/// FACE, indices 0).
/// </remarks>
public partial struct DsTransition : IEquatable<DsTransition>
{
    private Containment _stateBefore;
    private Containment _stateAfter;
    private ShapeType _shapeBefore;
    private ShapeType _shapeAfter;
    private int _indexBefore;
    private int _indexAfter;

    /// <summary>An unknown transition: UNKNOWN before and after, on faces, indices 0.</summary>
    public DsTransition()
    {
        _stateBefore = Unknown;
        _stateAfter = Unknown;
        _shapeBefore = ShapeType.Face;
        _shapeAfter = ShapeType.Face;
        _indexBefore = 0;
        _indexAfter = 0;
    }

    /// <summary>The transition from <paramref name="sb"/> to <paramref name="sa"/>, on shapes of the given kinds.</summary>
    public DsTransition(Containment sb,
                                   Containment sa,
                                   ShapeType onb = ShapeType.Face,
                                   ShapeType ona = ShapeType.Face)
    {
        _stateBefore = sb;
        _stateAfter = sa;
        _shapeBefore = onb;
        _shapeAfter = ona;
        _indexBefore = 0;
        _indexAfter = 0;
    }

    /// <summary>The transition of the orientation <paramref name="o"/> (see <see cref="Set(Orientation)"/>).</summary>
    public DsTransition(Orientation o)
    {
        // OCCT leaves the states unset for an orientation outside the four; the port's are IN (C#'s zero)
        _shapeBefore = ShapeType.Face;
        _shapeAfter = ShapeType.Face;
        _indexBefore = 0;
        _indexAfter = 0;
        Set(o);
    }

    /// <summary>Sets the states and the kinds of shape before and after.</summary>
    internal void Set(Containment sb,
                      Containment sa,
                      ShapeType onb = ShapeType.Face,
                      ShapeType ona = ShapeType.Face)
    {
        _stateBefore = sb;
        _stateAfter = sa;
        _shapeBefore = onb;
        _shapeAfter = ona;
    }

    internal void StateBefore(Containment s)
    {
        _stateBefore = s;
    }

    internal void StateAfter(Containment s)
    {
        _stateAfter = s;
    }

    internal void ShapeBefore(ShapeType se)
    {
        _shapeBefore = se;
    }

    internal void ShapeAfter(ShapeType se)
    {
        _shapeAfter = se;
    }

    internal void Before(Containment s, ShapeType onb = ShapeType.Face)
    {
        _stateBefore = s;
        _shapeBefore = onb;
    }

    internal void After(Containment s, ShapeType ona = ShapeType.Face)
    {
        _stateAfter = s;
        _shapeAfter = ona;
    }

    /// <summary>Sets the indices before and after to <paramref name="i"/>.</summary>
    internal void Index(int i)
    {
        _indexBefore = _indexAfter = i;
    }

    internal void IndexBefore(int i)
    {
        _indexBefore = i;
    }

    internal void IndexAfter(int i)
    {
        _indexAfter = i;
    }

    /// <summary>The state before.</summary>
    public readonly Containment Before() => _stateBefore;

    /// <summary>The kind of shape before.</summary>
    public readonly ShapeType ONBefore() => _shapeBefore;

    /// <summary>The state after.</summary>
    public readonly Containment After() => _stateAfter;

    /// <summary>The kind of shape after.</summary>
    public readonly ShapeType ONAfter() => _shapeAfter;

    /// <summary>The kind of shape before.</summary>
    public readonly ShapeType ShapeBefore() => _shapeBefore;

    /// <summary>The kind of shape after.</summary>
    public readonly ShapeType ShapeAfter() => _shapeAfter;

    /// <summary>The index of the shape before and after.</summary>
    /// <exception cref="FailureException">The indices before and after differ.</exception>
    public readonly int Index()
    {
        if (_indexAfter != _indexBefore)
        {
            throw new FailureException("Transition::Index() on different shapes");
        }
        return _indexBefore;
    }

    /// <summary>The index of the shape before.</summary>
    public readonly int IndexBefore() => _indexBefore;

    /// <summary>The index of the shape after.</summary>
    public readonly int IndexAfter() => _indexAfter;

    /// <summary>
    /// Sets the states of the orientation <paramref name="o"/>: FORWARD is (OUT, IN), REVERSED (IN, OUT), INTERNAL
    /// (IN, IN), EXTERNAL (OUT, OUT).
    /// </summary>
    internal void Set(Orientation o)
    {
        switch (o)
        {
            case Forward:
                _stateBefore = Out;
                _stateAfter = In;
                break;
            case Reversed:
                _stateBefore = In;
                _stateAfter = Out;
                break;
            case Internal:
                _stateBefore = In;
                _stateAfter = In;
                break;
            case External:
                _stateBefore = Out;
                _stateAfter = Out;
                break;
        }
    }

    /// <summary>
    /// The orientation corresponding to the state <paramref name="s"/>: REVERSED when leaving it, FORWARD when
    /// entering it, INTERNAL when staying in it, EXTERNAL when staying outside it (with ON states, see OCCT).
    /// </summary>
    public readonly Orientation Orientation(Containment s, ShapeType t = ShapeType.Face)
    {
        if (_stateBefore == On || _stateAfter == On)
        {
            return OrientationON(s, t);
        }
        else
        {
            if (_stateBefore == s)
            {
                if (_stateAfter == s)
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
                if (_stateAfter == s)
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
    private readonly Orientation OrientationON(Containment s, ShapeType t)
    {
        _ = t;
        Orientation result = Forward;

        if (_stateBefore == On && _stateAfter == On)
        {
            if (s == In)
            {
                result = Internal;
            }
            else if (s == Out)
            {
                result = External;
            }
            else if (s == On)
            {
                result = Internal;
            }
        }
        else if (_stateBefore == On)
        {
            if (_stateAfter == s)
            {
                return Forward;
            }
            else
            {
                return Reversed;
            }
        }
        else if (_stateAfter == On)
        {
            if (_stateBefore == s)
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
    public readonly DsTransition Complement()
    {
        DsTransition t = new();
        t._indexBefore = _indexBefore;
        t._indexAfter = _indexAfter;

        // xpu : 14-01-98
        if (_stateBefore == Unknown && _stateAfter == Unknown)
        {
            t.Set(_stateAfter, _stateBefore, _shapeAfter, _shapeBefore);
            return t;
        }
        // xpu : 14-01-98

        if (_stateBefore == On || _stateAfter == On)
        {
            t.Set(_stateAfter, _stateBefore, _shapeAfter, _shapeBefore);
        }
        else
        {
            Orientation o = Orientation(In);
            if (o == Forward)
            { // (OUT,IN) --> (IN,OUT)
                t.Set(In, Out, _shapeBefore, _shapeAfter);
            }
            else if (o == Reversed)
            { // (IN,OUT) --> (OUT,IN)
                t.Set(Out, In, _shapeBefore, _shapeAfter);
            }
            else if (o == External)
            { // (OUT,OUT) --> (IN,IN)
                t.Set(In, In, _shapeBefore, _shapeAfter);
            }
            else if (o == Internal)
            { // (IN,IN) --> (OUT,OUT)
                t.Set(Out, Out, _shapeBefore, _shapeAfter);
            }
        }

        return t;
    }

    /// <summary>Whether both states are UNKNOWN.</summary>
    public readonly bool IsUnknown => (_stateBefore == Unknown) && (_stateAfter == Unknown);

    /// <summary>A copy with the indices before and after set to <paramref name="i"/>.</summary>
    public readonly DsTransition WithIndex(int i)
    {
        DsTransition copy = this;
        copy.Index(i);
        return copy;
    }

    /// <summary>A copy with the states and kinds of shape set.</summary>
    public readonly DsTransition WithSet(Containment sb,
                                                    Containment sa,
                                                    ShapeType onb = ShapeType.Face,
                                                    ShapeType ona = ShapeType.Face)
    {
        DsTransition copy = this;
        copy.Set(sb, sa, onb, ona);
        return copy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(DsTransition other)
        => _stateBefore == other._stateBefore && _stateAfter == other._stateAfter
           && _shapeBefore == other._shapeBefore && _shapeAfter == other._shapeAfter
           && _indexBefore == other._indexBefore && _indexAfter == other._indexAfter;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is DsTransition other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode()
        => HashCode.Combine(_stateBefore, _stateAfter, _shapeBefore, _shapeAfter, _indexBefore, _indexAfter);

    public static bool operator ==(in DsTransition left, in DsTransition right)
        => left.Equals(right);

    public static bool operator !=(in DsTransition left, in DsTransition right)
        => !left.Equals(right);

    /// <summary>The states, kinds and indices, as in the data structure's text dump (<c>OUT/IN FACE/FACE 0/0</c>).</summary>
    public override readonly string ToString() => TextDump.TransitionText(this);
}
