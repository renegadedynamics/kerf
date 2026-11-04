// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatching.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatching.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Classification.Hatching.HatchingErrorStatus;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>
/// A hatching of a 2D hatcher: its curve, its sorted intersection points with the elements, its domains, and the
/// flags and status of their computation.
/// </summary>
/// <remarks>
/// A working object (decisions 21 and 666): a class, since the hatcher changes it in place, with OCCT's public API.
/// The constructor keeps a shallow copy of the caller's adaptor, as OCCT copies it.
/// </remarks>
internal sealed class Hatching2D
{
    private readonly CurveAdaptor2D _curve;
    private bool _trimDone;
    private bool _trimFailed;
    private readonly NCollection_Sequence<PointOnHatching> _points = new();
    private bool _isDone;
    private HatchingErrorStatus _status;
    private readonly NCollection_Sequence<HatchingDomain> _domains = new();

    /// <summary>A hatching on an empty curve.</summary>
    public Hatching2D()
    {
        _curve = new CurveAdaptor2D();
        _trimDone = false;
        _trimFailed = false;
        _isDone = false;
        _status = HatchingErrorStatus.NoProblem;
    }

    /// <summary>A hatching on (a copy of) <paramref name="curve"/>.</summary>
    public Hatching2D(CurveAdaptor2D curve)
    {
        _curve = (CurveAdaptor2D)curve.ShallowCopy();
        _trimDone = false;
        _trimFailed = false;
        _isDone = false;
        _status = HatchingErrorStatus.NoProblem;
    }

    /// <summary>The curve of the hatching.</summary>
    public CurveAdaptor2D Curve() => _curve;

    /// <summary>The curve of the hatching (OCCT's non-const reference: the same object).</summary>
    public CurveAdaptor2D ChangeCurve() => _curve;

    /// <summary>Sets the flag that the trimmings were computed.</summary>
    public void TrimDone(bool flag)
    {
        _trimDone = flag;
    }

    /// <summary>True if the trimmings were computed.</summary>
    public bool TrimDone() => _trimDone;

    /// <summary>Sets the flag that a trimming failed (and then the status TrimFailure).</summary>
    public void TrimFailed(bool flag)
    {
        _trimFailed = flag;
        if (_trimFailed)
        {
            _status = HatchingErrorStatus.TrimFailure;
        }
    }

    /// <summary>True if a trimming failed.</summary>
    public bool TrimFailed() => _trimFailed;

    /// <summary>Sets the flag that the domains were computed.</summary>
    public void IsDone(bool flag)
    {
        _isDone = flag;
    }

    /// <summary>True if the domains were computed.</summary>
    public bool IsDone() => _isDone;

    /// <summary>Sets the error status.</summary>
    public void Status(HatchingErrorStatus status)
    {
        _status = status;
    }

    /// <summary>The error status.</summary>
    public HatchingErrorStatus Status() => _status;

    /// <summary>
    /// Adds an intersection point, in parameter order; a point within <paramref name="confusion"/> of an existing
    /// one gives that one its points on element instead.
    /// </summary>
    public void AddPoint(PointOnHatching point, double confusion)
    {
        int nbPoints = _points.Length();
        // for (int IPntH = 1 ; IPntH <= NbPoints ; IPntH++) {
        int iPntH;
        for (iPntH = 1; iPntH <= nbPoints; iPntH++)
        {
            PointOnHatching pntH = _points.Value(iPntH);
            if (!pntH.IsLower(point, confusion))
            {
                break;
            }
        }
        if (iPntH > nbPoints)
        {
            _points.Append(new PointOnHatching(point));
        }
        else
        {
            PointOnHatching pntH = _points.ChangeValue(iPntH);
            if (pntH.IsGreater(point, confusion))
            {
                _points.InsertBefore(iPntH, new PointOnHatching(point));
            }
            else
            {
                for (int iPntE = 1; iPntE <= point.PointCount; iPntE++)
                {
                    PointOnElement pntE = point.Point(iPntE);
                    pntH.AddPoint(pntE, confusion);
                }
            }
        }
        if (_isDone)
        {
            ClrDomains();
        }
    }

    /// <summary>The number of intersection points.</summary>
    public int PointCount => _points.Length();

    /// <summary>The <paramref name="index"/>-th intersection point.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is out of range.</exception>
    public PointOnHatching Point(int index) => _points.Value(index);

    /// <summary>The <paramref name="index"/>-th intersection point, for writing (the same object).</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is out of range.</exception>
    public PointOnHatching ChangePoint(int index) => _points.ChangeValue(index);

    /// <summary>Removes the <paramref name="index"/>-th intersection point (and the domains, if computed).</summary>
    public void RemPoint(int index)
    {
        if (_isDone)
        {
            ClrDomains();
        }
        _points.Remove(index);
    }

    /// <summary>Removes all the intersection points (and the domains, if computed), and resets the trimming flags.</summary>
    public void ClrPoints()
    {
        if (_isDone)
        {
            ClrDomains();
        }
        for (int iPntH = 1; iPntH <= _points.Length(); iPntH++)
        {
            PointOnHatching point = _points.ChangeValue(iPntH);
            point.ClrPoints();
        }
        _points.Clear();
        _trimDone = false;
        _trimFailed = false;
    }

    /// <summary>Adds a domain.</summary>
    public void AddDomain(in HatchingDomain domain)
    {
        _domains.Append(domain);
    }

    /// <summary>The number of domains.</summary>
    public int DomainCount => _domains.Length();

    /// <summary>The <paramref name="index"/>-th domain.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is out of range.</exception>
    public HatchingDomain Domain(int index) => _domains.Value(index);

    /// <summary>The <paramref name="index"/>-th domain, for writing (OCCT.NET: ChFi3d_Builder_SpKP's <c>Tri</c> casts the
    /// constness of <c>Domain</c> away and edits the hatcher's domain in place).</summary>
    internal ref HatchingDomain ChangeDomain(int index) => ref _domains.ChangeValue(index);

    /// <summary>Removes the <paramref name="index"/>-th domain.</summary>
    public void RemDomain(int index)
    {
        _domains.Remove(index);
    }

    /// <summary>Removes all the domains; they are then not computed.</summary>
    public void ClrDomains()
    {
        _domains.Clear();
        _isDone = false;
    }

    /// <summary>
    /// A point of the curve for the classification: at its last parameter, else its first, else 0.
    /// </summary>
    public Point2D ClassificationPoint()
    {
        double t, a, b;
        a = _curve.FirstParameter();
        b = _curve.LastParameter();
        if (b >= global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            if (a <= -global::Kerf.Engine.Foundation.Precision.Infinite())
            {
                t = 0;
            }
            else
            {
                t = a;
            }
        }
        else
        {
            t = b;
        }
        return (_curve.Value(t));
    }
}
