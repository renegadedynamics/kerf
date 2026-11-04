// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Hatcher.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Hatcher.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Hatcher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Classification.Hatching.LineForm;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>
/// Computes cross hatchings in a 2D plane, mainly for display: lines are stored (<c>AddLine</c>, <c>AddXLine</c>,
/// <c>AddYLine</c>), trimmed by a boundary whose inside is on its left (<c>Trim</c>), and read back as parameter
/// intervals on each line (<c>NbIntervals</c>, <c>Start</c>, <c>End</c>). An interval may start at RealFirst() and
/// end at RealLast().
/// </summary>
/// <remarks>
/// In the oriented mode the orientation of the trimming curves is considered (hatchings are kept on their left, and
/// may be infinite); in the unoriented mode the hatchings are always finite. A working object (decision 21).
/// </remarks>
internal sealed class Hatcher
{
    private double _toler;
    private readonly NCollection_Sequence<HatchingLine> _lines = new();
    private readonly bool _orient;

    /// <summary>An empty hatcher; <paramref name="tol"/> is the tolerance for intersections.</summary>
    public Hatcher(double tol, bool oriented = true)
    {
        _toler = tol;
        _orient = oriented;
    }

    /// <summary>Sets the tolerance for intersections.</summary>
    public void Tolerance(double tol)
    {
        _toler = tol;
    }

    /// <summary>The tolerance for intersections.</summary>
    public double Tolerance() => _toler;

    /// <summary>Adds a line <paramref name="l"/> to be trimmed; its form <paramref name="t"/> is only kept for information.</summary>
    public void AddLine(in Line2D l, LineForm t = global::Kerf.Engine.Classification.Hatching.LineForm.Anyline)
    {
        HatchingLine hl = new(l, t);
        _lines.Append(hl);
    }

    /// <summary>
    /// Adds an infinite line of direction <paramref name="d"/> at the (signed) distance <paramref name="dist"/> from
    /// the origin: the points P with OP · V = Dist, V perpendicular to D in the direct sense; P's parameter is OP · D.
    /// </summary>
    public void AddLine(in Direction2D d, double dist)
    {
        double x = d.X;
        double y = d.Y;
        Point2D o = new(-y * dist, x * dist);
        Line2D l = new(o, d);
        AddLine(l, global::Kerf.Engine.Classification.Hatching.LineForm.Anyline);
    }

    /// <summary>Adds an infinite line parallel to the Y axis at the abscissa <paramref name="x"/>.</summary>
    public void AddXLine(double x)
    {
        Point2D o = new(x, 0);
        Direction2D d = new(Direction2D.D.Y);
        Line2D l = new(o, d);
        AddLine(l, global::Kerf.Engine.Classification.Hatching.LineForm.Xline);
    }

    /// <summary>Adds an infinite line parallel to the X axis at the ordinate <paramref name="y"/>.</summary>
    public void AddYLine(double y)
    {
        Point2D o = new(0, y);
        Direction2D d = new(Direction2D.D.X);
        Line2D l = new(o, d);
        AddLine(l, global::Kerf.Engine.Classification.Hatching.LineForm.Yline);
    }

    /// <summary>Trims the lines at their intersections with <paramref name="l"/>.</summary>
    public void Trim(in Line2D l, int index = 0)
    {
        Trim(l, RealFirst(), RealLast(), index);
    }

    /// <summary>Trims the lines at their intersections with <paramref name="l"/> in its parameter range [Start, End].</summary>
    public void Trim(in Line2D l, double start, double end, int index = 0)
    {
        IntPoint pinter;
        AnalyticIntersection inters = new();
        int iLine;
        for (iLine = 1; iLine <= _lines.Length(); iLine++)
        {
            inters.Perform(_lines[iLine]._lin, l);
            if (inters.IsDone)
            {
                if (!inters.IdenticalElements() && !inters.ParallelElements())
                {
                    // we have got something
                    pinter = inters.Point(1);
                    double linePar = pinter.ParamOnSecond();
                    if (linePar - start < -_toler)
                    {
                        continue;
                    }
                    if (linePar - end > _toler)
                    {
                        continue;
                    }
                    double norm = l.Direction ^ _lines[iLine]._lin.Direction;
                    if (linePar - start < _toler)
                    {
                        // on the limit of the trimming segment
                        // accept if the other extremity is on the left
                        if (norm < 0)
                        {
                            continue;
                        }
                    }
                    if (linePar - end > -_toler)
                    {
                        // on the limit of the trimming segment
                        // accept if the other extremity is on the left
                        if (norm > 0)
                        {
                            continue;
                        }
                    }
                    // insert the parameter
                    _lines[iLine].AddIntersection(pinter.ParamOnFirst(),
                                                   norm > 0,
                                                   index,
                                                   pinter.ParamOnSecond(),
                                                   _toler);
                }
            }
        }
    }

    /// <summary>Trims the lines at their intersections with the oriented segment from <paramref name="p1"/> to <paramref name="p2"/>.</summary>
    public void Trim(in Point2D p1, in Point2D p2, int index = 0)
    {
        Vector2D v = new(p1, p2);
        if (Math.Abs(v.X) > .9 * RealLast())
        {
            v = v.Multiplied(1 / v.X);
        }
        else if (Math.Abs(v.Y) > .9 * RealLast())
        {
            v = v.Multiplied(1 / v.Y);
        }
        if (v.Magnitude > _toler)
        {
            Direction2D d = new(v);
            Line2D l = new(p1, d);
            Trim(l, 0, p1.DistanceTo(p2), index);
        }
    }

    /// <summary>The total number of intervals on all the lines.</summary>
    public int NbIntervals()
    {
        int i, nb = 0;
        for (i = 1; i <= _lines.Length(); i++)
        {
            nb += NbIntervals(i);
        }
        return nb;
    }

    /// <summary>The number of lines.</summary>
    public int LineCount => _lines.Length();

    /// <summary>The line of index <paramref name="i"/>.</summary>
    public Line2D Line(int i) => _lines[i]._lin;

    /// <summary>The form of the line of index <paramref name="i"/>.</summary>
    public LineForm LineForm(int i) => _lines[i]._form;

    /// <summary>True if the line of index <paramref name="i"/> has a constant X.</summary>
    public bool IsXLine(int i) => LineForm(i) == global::Kerf.Engine.Classification.Hatching.LineForm.Xline;

    /// <summary>True if the line of index <paramref name="i"/> has a constant Y.</summary>
    public bool IsYLine(int i) => LineForm(i) == global::Kerf.Engine.Classification.Hatching.LineForm.Yline;

    /// <summary>The X or Y coordinate of the line of index <paramref name="i"/>, an X or a Y line.</summary>
    /// <exception cref="OutOfRangeException">The line is neither an X nor a Y line.</exception>
    public double Coordinate(int i)
    {
        switch (_lines[i]._form)
        {
            case global::Kerf.Engine.Classification.Hatching.LineForm.Xline:
                return _lines[i]._lin.Location.X;

            case global::Kerf.Engine.Classification.Hatching.LineForm.Yline:
                return _lines[i]._lin.Location.Y;

            case global::Kerf.Engine.Classification.Hatching.LineForm.Anyline:
                throw new OutOfRangeException("Hatcher : not an X or Y line");
        }

        return 0.0;
    }

    /// <summary>The number of intervals on the line of index <paramref name="i"/>.</summary>
    public int NbIntervals(int i)
    {
        int l = _lines[i]._inters.Length();
        if (l == 0)
        {
            l = _orient ? 1 : 0;
        }
        else
        {
            l = l / 2;
            if (_orient)
            {
                if (!_lines[i]._inters[1]._start)
                {
                    l++;
                }
            }
        }
        return l;
    }

    /// <summary>The first parameter of the interval <paramref name="j"/> on the line <paramref name="i"/>.</summary>
    public double Start(int i, int j)
    {
        if (_lines[i]._inters.IsEmpty())
        {
            if (j != 1 || !_orient)
            {
                throw new OutOfRangeException();
            }
            return RealFirst();
        }
        else
        {
            int jj = 2 * j - 1;
            if (!_lines[i]._inters[1]._start && _orient)
            {
                jj--;
            }
            if (jj == 0)
            {
                return RealFirst();
            }
            return _lines[i]._inters[jj]._par1;
        }
    }

    /// <summary>
    /// The index of the trimming element and the parameter on it at the start of the interval <paramref name="j"/> on
    /// the line <paramref name="i"/> (0 and 0 at an infinite start).
    /// </summary>
    public void StartIndex(int i, int j, out int index, out double par2)
    {
        if (_lines[i]._inters.IsEmpty())
        {
            if (j != 1) // OCCT: doesn't check myOrient, as Start and End do (occt-issues.md)
            {
                throw new OutOfRangeException();
            }
            index = 0;
            par2 = 0;
        }
        else
        {
            int jj = 2 * j - 1;
            if (!_lines[i]._inters[1]._start && _orient)
            {
                jj--;
            }
            if (jj == 0)
            {
                index = 0;
                par2 = 0;
            }
            else
            {
                index = _lines[i]._inters[jj]._index;
                par2 = _lines[i]._inters[jj]._par2;
            }
        }
    }

    /// <summary>The last parameter of the interval <paramref name="j"/> on the line <paramref name="i"/>.</summary>
    public double End(int i, int j)
    {
        if (_lines[i]._inters.IsEmpty())
        {
            if (j != 1 || !_orient)
            {
                throw new OutOfRangeException();
            }
            return RealLast();
        }
        else
        {
            int jj = 2 * j;
            if (!_lines[i]._inters[1]._start && _orient)
            {
                jj--;
            }
            if (jj > _lines[i]._inters.Length())
            {
                return RealLast();
            }
            return _lines[i]._inters[jj]._par1;
        }
    }

    /// <summary>
    /// The index of the trimming element and the parameter on it at the end of the interval <paramref name="j"/> on
    /// the line <paramref name="i"/> (0 and 0 at an infinite end).
    /// </summary>
    public void EndIndex(int i, int j, out int index, out double par2)
    {
        if (_lines[i]._inters.IsEmpty())
        {
            if (j != 1) // OCCT: doesn't check myOrient, as Start and End do (occt-issues.md)
            {
                throw new OutOfRangeException();
            }
            index = 0;
            par2 = 0;
        }
        else
        {
            int jj = 2 * j;
            if (!_lines[i]._inters[1]._start && _orient)
            {
                jj--;
            }
            if (jj > _lines[i]._inters.Length())
            {
                index = 0;
                par2 = 0;
            }
            else
            {
                index = _lines[i]._inters[jj]._index;
                par2 = _lines[i]._inters[jj]._par2;
            }
        }
    }
}
