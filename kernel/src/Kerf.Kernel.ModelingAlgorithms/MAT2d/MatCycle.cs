// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>Finds a bisecting locus that has come back to a state it was in before (Kerf's; OCCT has none).</summary>
/// <remarks>
/// <para>MAT2d_Mat2d's main loop removes pieces of the contour and puts back those whose bisectors turn out wrong. On
/// bad geometry it can remove and put back the same pieces for ever, which OCCT stops by counting the passes up to the
/// square of the number of elements: 47,524 passes for 218 elements, each as dear as the last
/// (offset/wire_closed_inside_0_025/G4: 9 s and 22 GB to fail). The loop is a function of its state alone, so a pass
/// that ends in the state an earlier pass ended in is followed by the same passes again, and can only end at that
/// count.</para>
/// <para>The state written down after a pass is everything a later pass reads: the contour's edges in order, each with
/// its distance and intersection point, and both its bisectors, each with its parameters, sense, points, vectors, edges
/// and the ends of its trimmed curve; the bisectors made by this pass with their children too; and the loop's own
/// counters. Numbers are compared by their bits and points and vectors by their coordinates, as their indices only
/// grow. A bisector made before the reference pass is named by its index (the same object, whose descendants nothing
/// touches while it lives: only cancelling a bisector changes its children, and a cancelled bisector never comes back);
/// one made by the pass by its place among that pass's; one made in between and still there matches nothing, so such a
/// state is never taken for a repeat.</para>
/// <para>The reference state is Brent's: it moves to the current pass after 1, 2, 4, ... passes, so a cycle of any
/// length that starts anywhere is found within a few times its length. Nothing is written until a pass ends without
/// the contour being shorter than it has ever been, so a map that only shrinks pays nothing.</para>
/// </remarks>
internal sealed class MatCycle
{
    private long[] _reference = [];
    private int _referenceLength;
    private long[] _current = [];
    private int _currentLength;
    private int _referencePass;
    private int _referenceBegin;
    private int _power = 1;
    private int _steps;
    private int _fewestEdges = int.MaxValue;
    private bool _armed;

    /// <summary>
    /// Called at the end of pass <paramref name="pass"/>: the number of passes since one that ended in this state, or 0
    /// if none is known. <paramref name="beginBisector"/> is the index of the first bisector made by this pass,
    /// <paramref name="counters"/> the loop's own state, and <paramref name="carriedPoint"/> the index of the last
    /// intersection point found, which the next intersection that finds none leaves on its edge.
    /// </summary>
    public int Step(int pass, MedialListOfEdge edges, Tool2d tool, int beginBisector, ReadOnlySpan<int> counters,
        int carriedPoint)
    {
        if (!_armed)
        {
            if (edges.Number() < _fewestEdges)
            {
                _fewestEdges = edges.Number();
                return 0;
            }
            _armed = true;
            SetReference(pass, edges, tool, beginBisector, counters, carriedPoint);
            return 0;
        }

        Write(edges, tool, _referenceBegin, beginBisector, counters, carriedPoint);
        if (_currentLength == _referenceLength
            && _current.AsSpan(0, _currentLength).SequenceEqual(_reference.AsSpan(0, _referenceLength)))
        {
            return pass - _referencePass;
        }

        if (++_steps == _power)
        {
            _power *= 2;
            _steps = 0;
            SetReference(pass, edges, tool, beginBisector, counters, carriedPoint);
        }
        return 0;
    }

    private void SetReference(int pass, MedialListOfEdge edges, Tool2d tool, int beginBisector,
        ReadOnlySpan<int> counters, int carriedPoint)
    {
        Write(edges, tool, beginBisector, beginBisector, counters, carriedPoint);
        (_reference, _current) = (_current, _reference);
        _referenceLength = _currentLength;
        _referencePass = pass;
        _referenceBegin = beginBisector;
    }

    // The state, into _current. Bisectors of index below `old` were there at the reference pass; those from `begin` were
    // made by the pass that has just ended
    private void Write(MedialListOfEdge edges, Tool2d tool, int old, int begin, ReadOnlySpan<int> counters,
        int carriedPoint)
    {
        int number = edges.Number();
        if (_current.Length < 64 + 40 * number)
        {
            _current = new long[64 + 40 * number];
        }
        _currentLength = 0;
        foreach (int counter in counters)
        {
            Push(counter);
        }
        PushPoint(tool, carriedPoint);

        Push(number);
        MedialBisector? last = null;
        MedialTListNode<MedialEdge>? node = edges.FirstNode;
        for (int i = 0; i < number && node is not null; i++, node = node.Next())
        {
            MedialEdge edge = node.GetItem();
            Push(edge.EdgeNumber());
            Push(node.Previous()?.GetItem().EdgeNumber() ?? -1);
            Push(edge.Distance());
            PushPoint(tool, edge.IntersectionPoint());
            // An edge's first bisector is as a rule the second of the edge before, written already
            if (last is not null && ReferenceEquals(edge.FirstBisector(), last))
            {
                Push(-2);
            }
            else
            {
                PushBisector(tool, edge.FirstBisector(), old, begin);
            }
            last = edge.SecondBisector();
            PushBisector(tool, last, old, begin);
        }
    }

    private void PushBisector(Tool2d tool, MedialBisector? bisector, int old, int begin)
    {
        if (bisector is null)
        {
            Push(-1);
            return;
        }

        int index = bisector.IndexNumber();
        bool made = index >= begin;
        if (index < old)
        {
            Push(0);
            Push(index);
        }
        else if (made)
        {
            Push(1);
            Push(index - begin);
        }
        else
        {
            // Made since the reference pass and still there: the reference has none, so the states differ
            Push(2);
            Push(index);
        }

        Push(bisector.FirstParameter());
        Push(bisector.SecondParameter());
        Push(bisector.Sense());
        Push(bisector.DistIssuePoint());
        PushPoint(tool, bisector.IssuePoint());
        PushPoint(tool, bisector.EndPoint());
        PushVector(tool, bisector.FirstVector());
        PushVector(tool, bisector.SecondVector());
        Push(((MedialEdge?)bisector.FirstEdge())?.EdgeNumber() ?? -1);
        Push(((MedialEdge?)bisector.SecondEdge())?.EdgeNumber() ?? -1);

        // Its curve: the same object for the same number, trimmed in place
        Push(bisector.BisectorNumber());
        TrimmedCurve2D? curve = tool.TrimmedBisector(bisector.BisectorNumber());
        if (curve is null)
        {
            Push(0);
        }
        else
        {
            Push(1);
            Push(curve.FirstParameter);
            Push(curve.LastParameter);
        }

        MedialListOfBisector children = bisector.List();
        Push(children.Number());
        if (made)
        {
            MedialTListNode<MedialBisector>? node = children.FirstNode;
            for (int i = 0; i < children.Number() && node is not null; i++, node = node.Next())
            {
                PushBisector(tool, node.GetItem(), old, begin);
            }
        }
    }

    private void PushPoint(Tool2d tool, int index)
    {
        if (index == 0)
        {
            Push(0);
        }
        else if (tool.TryGeomPnt(index, out Point2D point))
        {
            Push(1);
            Push(point.X);
            Push(point.Y);
        }
        else
        {
            Push(2);
            Push(index);
        }
    }

    private void PushVector(Tool2d tool, int index)
    {
        if (tool.TryGeomVec(index, out Vector2D vector))
        {
            Push(1);
            Push(vector.X);
            Push(vector.Y);
        }
        else
        {
            Push(2);
            Push(index);
        }
    }

    private void Push(double value) => Push(BitConverter.DoubleToInt64Bits(value));

    private void Push(long value)
    {
        if (_currentLength == _current.Length)
        {
            Array.Resize(ref _current, Math.Max(256, 2 * _current.Length));
        }
        _current[_currentLength++] = value;
    }
}
