// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundsProperties.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundsProperties.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundsProperties.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// The properties of the free bounds of a shape (found by <see cref="FreeBounds"/>): for each contour,
/// its area and perimeter, the ratio of its average length to its average width, its average width, and its notches
/// (narrow V-like sub-contours) with their maximum widths, kept in <see cref="FreeBoundDataAnalysis"/>.
/// </summary>
/// <remarks>
/// A public working object (decision 21; OCCT uses it by value). It only reads the shape; the free bounds and the
/// notches it hands out are frozen.
/// </remarks>
internal sealed class FreeBoundsPropertiesAnalysis
{
    private const int NbControl = 23;

    private Shape _shape = new();
    private double _tolerance;
    private bool _splitClosed;
    private bool _splitOpen;
    private readonly NCollection_HSequence<FreeBoundDataAnalysis> _closedFreeBounds;
    private readonly NCollection_HSequence<FreeBoundDataAnalysis> _openFreeBounds;

    private static void ContourProperties(in Wire wire, out double countourArea, out double countourLength)
    {
        int nbe = 0;
        double length = 0.0;
        Xyz area = new(0.0, 0.0, 0.0);
        Xyz prev = new(), cont = new();
        for (var exp = new WireExplorer(wire); exp.More(); exp.Next())
        {
            Edge edge = exp.Current();
            nbe++;
            var sae = new EdgeAnalysis();
            if (!sae.Curve3d(edge, out Curve3D? c3d, out double first, out double last))
            {
                continue;
            }
            int ibeg = 0;
            if (nbe == 1)
            {
                Point3D pntIni = c3d!.Value(first);
                prev = pntIni.XYZ();
                cont = prev;
                ibeg = 1;
            }
            for (int i = ibeg; i < NbControl; i++)
            {
                double prm = ((NbControl - 1 - i) * first + i * last) / (NbControl - 1);
                Point3D pntCurr = c3d!.Value(prm);
                Xyz curr = pntCurr.XYZ();
                Xyz delta = curr - prev;
                length += delta.Modulus();
                area += curr ^ prev;
                prev = curr;
            }
        }
        area += cont ^ prev;
        countourArea = area.Modulus() / 2;
        countourLength = length;
    }

    /// <summary>An empty analyser.</summary>
    public FreeBoundsPropertiesAnalysis()
    {
        _closedFreeBounds = new NCollection_HSequence<FreeBoundDataAnalysis>();
        _openFreeBounds = new NCollection_HSequence<FreeBoundDataAnalysis>();
        _tolerance = 0.0;
    }

    /// <summary>An analyser of the free bounds of a compound of faces, sewn with the tolerance (see <see cref="Init(in Shape, double, bool, bool)"/>).</summary>
    public FreeBoundsPropertiesAnalysis(in Shape shape, double tolerance, bool splitclosed = false, bool splitopen = false)
    {
        _closedFreeBounds = new NCollection_HSequence<FreeBoundDataAnalysis>();
        _openFreeBounds = new NCollection_HSequence<FreeBoundDataAnalysis>();
        Init(shape, tolerance, splitclosed, splitopen);
    }

    /// <summary>An analyser of the free bounds of a compound of shells (see <see cref="Init(in Shape, bool, bool)"/>).</summary>
    public FreeBoundsPropertiesAnalysis(in Shape shape, bool splitclosed = false, bool splitopen = false)
    {
        _closedFreeBounds = new NCollection_HSequence<FreeBoundDataAnalysis>();
        _openFreeBounds = new NCollection_HSequence<FreeBoundDataAnalysis>();
        _tolerance = 0.0;
        Init(shape, splitclosed, splitopen);
    }

    /// <summary>Initializes with a compound of faces, sewn with the tolerance (the options as <see cref="FreeBounds"/>'s).</summary>
    public void Init(in Shape shape, double tolerance, bool splitclosed = false, bool splitopen = false)
    {
        Init(shape, splitclosed, splitopen);
        _tolerance = tolerance;
    }

    /// <summary>Initializes with a compound of shells (the options as <see cref="FreeBounds"/>'s).</summary>
    public void Init(in Shape shape, bool splitclosed = false, bool splitopen = false)
    {
        _shape = shape;
        _splitClosed = splitclosed;
        _splitOpen = splitopen;
    }

    /// <summary>
    /// Builds the free bounds and computes each one's properties (area, perimeter, ratio, average width, notches and
    /// their widths). True if no step failed and free bounds were found.
    /// </summary>
    public bool Perform()
    {
        bool result = false;
        result |= DispatchBounds();
        result |= CheckNotches();
        result |= CheckContours();
        return result;
    }

    /// <summary>True if a shape is loaded.</summary>
    public bool IsLoaded => !(_shape.IsNull);

    /// <summary>The shape.</summary>
    public Shape Shape() => _shape;

    /// <summary>The tolerance (0 for a compound of shells).</summary>
    public double Tolerance() => _tolerance;

    /// <summary>The number of free bounds.</summary>
    public int FreeBoundCount => (_closedFreeBounds.Length() + _openFreeBounds.Length());

    /// <summary>The number of closed free bounds.</summary>
    public int ClosedFreeBoundCount => _closedFreeBounds.Length();

    /// <summary>The number of open free bounds.</summary>
    public int OpenFreeBoundCount => _openFreeBounds.Length();

    /// <summary>The closed free bounds (the object's own sequence, as OCCT's handle).</summary>
    public NCollection_HSequence<FreeBoundDataAnalysis> ClosedFreeBounds() => _closedFreeBounds;

    /// <summary>The open free bounds (the object's own sequence, as OCCT's handle).</summary>
    public NCollection_HSequence<FreeBoundDataAnalysis> OpenFreeBounds() => _openFreeBounds;

    /// <summary>The closed free bound of the given rank.</summary>
    public FreeBoundDataAnalysis ClosedFreeBound(int index) => _closedFreeBounds.Value(index);

    /// <summary>The open free bound of the given rank.</summary>
    public FreeBoundDataAnalysis OpenFreeBound(int index) => _openFreeBounds.Value(index);

    /// <summary>Builds the free bounds and records the closed and open ones. False if no shape is loaded.</summary>
    public bool DispatchBounds()
    {
        if (!IsLoaded)
        {
            return false;
        }

        Compound tmpClosedBounds, tmpOpenBounds;
        if (_tolerance > 0.0)
        {
            var safb = new FreeBounds(_shape, _tolerance, _splitClosed, _splitOpen);
            tmpClosedBounds = safb.GetClosedWires();
            tmpOpenBounds = safb.GetOpenWires();
        }
        else
        {
            var safb = new FreeBounds(_shape, _splitClosed, _splitOpen);
            tmpClosedBounds = safb.GetClosedWires();
            tmpOpenBounds = safb.GetOpenWires();
        }

        var shexpl = new ExtendExplorer();
        NCollection_HSequence<Shape> tmpSeq = shexpl.SeqFromCompound(tmpClosedBounds, false);
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= tmpSeq.Length(); i++)
        {
            Wire wire = ShapeCasts.Wire(tmpSeq.Value(i));
            var fbData = new FreeBoundDataAnalysis();
            fbData.SetFreeBound(wire);
            _closedFreeBounds.Append(fbData);
        }

        NCollection_HSequence<Shape> tmpSeq2 = shexpl.SeqFromCompound(tmpOpenBounds, false);
        for (i = 1; i <= tmpSeq2.Length(); i++)
        {
            Wire wire = ShapeCasts.Wire(tmpSeq2.Value(i));
            var fbData = new FreeBoundDataAnalysis();
            fbData.SetFreeBound(wire);
            _openFreeBounds.Append(fbData);
        }

        return true;
    }

    /// <summary>Finds the notches of every free bound.</summary>
    public bool CheckNotches(double prec = 0.0)
    {
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= _closedFreeBounds.Length(); i++)
        {
            FreeBoundDataAnalysis fbData = _closedFreeBounds.Value(i);
            CheckNotches(fbData, prec);
        }
        for (i = 1; i <= _openFreeBounds.Length(); i++)
        {
            FreeBoundDataAnalysis fbData = _openFreeBounds.Value(i);
            CheckNotches(fbData, prec);
        }
        return true;
    }

    /// <summary>Finds the notches of a free bound and adds them to its data.</summary>
    public bool CheckNotches(FreeBoundDataAnalysis fbData, double prec = 0.0)
    {
        var swd = new ExtendWireData(fbData.FreeBound());
        if (swd.EdgeCount > 1)
        {
            for (int j = 1; j <= swd.EdgeCount; j++)
            {
                if (CheckNotches(fbData.FreeBound(), j, out Wire notch, out double dMax, prec))
                {
                    fbData.AddNotch(notch, dMax);
                }
            }
        }
        return true;
    }

    /// <summary>Computes the properties of every free bound.</summary>
    public bool CheckContours(double prec = 0.0)
    {
        bool status = false;
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= _closedFreeBounds.Length(); i++)
        {
            FreeBoundDataAnalysis fbData = _closedFreeBounds.Value(i);
            status |= FillProperties(fbData, prec);
        }
        for (i = 1; i <= _openFreeBounds.Length(); i++)
        {
            FreeBoundDataAnalysis fbData = _openFreeBounds.Value(i);
            status |= FillProperties(fbData, prec);
        }
        return status;
    }

    /// <summary>
    /// True if the edge <paramref name="num"/> of the wire and the next one (skipping a small edge between them)
    /// make a notch: they turn back by more than 0.95 pi. <paramref name="notch"/> is the wire of those edges (frozen;
    /// always built), <paramref name="distMax"/> the largest distance from 23 points of the first to the second (0
    /// when false).
    /// </summary>
    public bool CheckNotches(in Wire wire, int num, out Wire notch, out double distMax, double prec = 0.0)
    {
        distMax = 0.0; // Kerf: OCCT leaves it unset when it returns false
        bool res = CheckNotchesCore(wire, num, out notch, ref distMax);
        notch.TShape()?.FreezeGraph();
        return res;
    }

    private bool CheckNotchesCore(in Wire wire, int num, out Wire notch, ref double distMax)
    {
        double tol = Max(_tolerance, Kerf.Engine.Foundation.Precision.Confusion());
        var wdt = new ExtendWireData(wire);
        BrepBuilder b = new();
        notch = new Wire();
        b.MakeWire(ref notch);

        if ((num <= 0) || (num > wdt.EdgeCount))
        {
            return false;
        }

        int n1 = (num > 0 ? num : wdt.EdgeCount);
        int n2 = (n1 < wdt.EdgeCount ? n1 + 1 : 1);

        Edge e1 = wdt.Edge(n1);
        b.Add(notch, e1);

        var saw = new WireAnalysis();
        saw.Load(wdt);
        saw.SetPrecision(_tolerance);
        if (saw.CheckSmall(n2, tol))
        {
            b.Add(notch, wdt.Edge(n2));
            n2 = (n2 < wdt.EdgeCount ? n2 + 1 : 1);
        }

        Edge e2 = wdt.Edge(n2);
        b.Add(notch, e2);

        var sae = new EdgeAnalysis();
        // szv#4:S4163:12Mar99 optimized
        if (!sae.Curve3d(e1, out Curve3D? c3d1, out double first1, out double last1)
            || !sae.Curve3d(e2, out Curve3D? c3d2, out double first2, out double last2))
        {
            return false;
        }

        c3d1!.D1(last1, out _, out Vector3D vec1);
        c3d2!.D1(first2, out _, out Vector3D vec2);
        if (e1.Orientation == Reversed)
        {
            vec1 = vec1.Reversed();
        }
        if (e2.Orientation == Reversed)
        {
            vec2 = vec2.Reversed();
        }

        double angl = Math.Abs(vec1.AngleTo(vec2));
        if (angl > 0.95 * M_PI)
        {
            distMax = 0.0;
            for (int i = 0; i < NbControl; i++)
            {
                double prm = ((NbControl - 1 - i) * first1 + i * last1) / (NbControl - 1);
                Point3D pntCurr = c3d1.Value(prm);

                double p1, p2;
                if (first2 < last2)
                {
                    p1 = first2;
                    p2 = last2;
                }
                else
                {
                    p1 = last2;
                    p2 = first2;
                }

                // szv#4:S4163:12Mar99 warning
                var ppc = new ProjectPointOnCurve(pntCurr, c3d2, p1, p2);
                double newDist = (ppc.NbPoints() != 0 ? ppc.LowerDistance() : 0);
                if (newDist > distMax)
                {
                    distMax = newDist;
                }
            }

            return true;
        }

        return false;
    }

    /// <summary>Computes the area, perimeter, ratio and average width of a free bound into its data.</summary>
    public bool FillProperties(FreeBoundDataAnalysis fbData, double prec = 0.0)
    {
        ContourProperties(fbData.FreeBound(), out double area, out double length);

        double r = 0;
        double aver = 0;

        if (length != 0.0)
        {
            // szv#4:S4163:12Mar99 anti-exception
            double k = area / (length * length); // szv#4:S4163:12Mar99
            // szv#4:S4163:12Mar99 optimized
            if (k != 0.0)
            {
                // szv#4:S4163:12Mar99 anti-exception
                double aux = 1.0 - 16.0 * k;
                if (aux >= 0.0)
                {
                    r = (1.0 + Math.Sqrt(aux)) / (8.0 * k);
                    aver = length / (2.0 * r);
                    r -= 1.0;
                }
            }
        }

        fbData.SetArea(area);
        fbData.SetPerimeter(length);
        fbData.SetRatio(r);
        fbData.SetWidth(aver);

        return true;
    }
}
