// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>
/// The intersection of two Geom surfaces: IntPatch_Intersection's lines made into Geom curves (conics trimmed to
/// the domains, walking lines approximated by B-splines, restriction lines approximated on their surface), with
/// their 2D curves on either surface on request.
/// </summary>
/// <remarks>
/// <para>
/// A working object, as OCCT's: each <c>Perform</c> computes a new result. The curves <see cref="Line"/>,
/// <see cref="LineOnS1"/>, <see cref="LineOnS2"/> and <see cref="Boundary"/> return are frozen (design 4.2): they
/// are the algorithm's results handed to the caller. The static helpers (<see cref="BuildPCurves(double, double, ref
/// double, Surface, Curve3D, ref Curve2D?)"/>, <see cref="MakeBSpline"/>, <see cref="TreatRLine"/>, ...)
/// return new, unfrozen curves, as IntTools_FaceFace expects.
/// </para>
/// <para>
/// The curve building (OCCT's GeomInt_IntSS_1.cxx) is in GeomInt_IntSS.MakeCurve.cs.
/// </para>
/// </remarks>
public partial class IntSS
{
    private readonly PatchIntersection _intersector = new();
    private readonly LineConstructor _lConstruct = new();
    private SurfaceAdaptor? _hs1;
    private SurfaceAdaptor? _hs2;
    private int _nbrestr;
    private readonly NCollection_Sequence<Curve3D> _sline = new();
    private readonly NCollection_Sequence<Curve2D?> _slineS1 = new();
    private readonly NCollection_Sequence<Curve2D?> _slineS2 = new();
    private double _tolReached2d;
    private double _tolReached3d;
    private double _tolCheck;
    private double _tolAngCheck;

    public IntSS()
    {
        _nbrestr = 0;
        _tolReached2d = 0.0;
        _tolReached3d = 0.0;
        _tolCheck = 1.0e-7;
        _tolAngCheck = 1.0e-6;
    }

    /// <summary>Intersects S1 and S2 with the tolerance Tol (see <see cref="Perform(Surface, Surface,
    /// double, bool, bool, bool)"/>).</summary>
    public IntSS(Surface s1, Surface s2, double tol, bool approx = true, bool approxS1 = false,
                         bool approxS2 = false)
        : this()
    {
        Perform(s1, s2, tol, approx, approxS1, approxS2);
    }

    /// <summary>
    /// Intersects S1 and S2 with the tolerance Tol. Approx approximates the walking lines (otherwise they are
    /// polylines); ApproxS1 and ApproxS2 compute the 2D curves on S1 and S2.
    /// </summary>
    public void Perform(Surface s1, Surface s2, double tol, bool approx = true, bool approxS1 = false,
                        bool approxS2 = false)
    {
        _hs1 = new SurfaceAdaptor(s1);
        if (s1 == s2)
        {
            _hs2 = _hs1;
        }
        else
        {
            _hs2 = new SurfaceAdaptor(s2);
        }
        InternalPerform(tol, approx, approxS1, approxS2, false, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>Intersects two surface adaptors (see <see cref="Perform(Surface, Surface, double, bool,
    /// bool, bool)"/>). The adaptors are kept, as OCCT keeps the handles.</summary>
    public void Perform(SurfaceAdaptor hs1, SurfaceAdaptor hs2, double tol, bool approx = true,
                        bool approxS1 = false, bool approxS2 = false)
    {
        _hs1 = hs1;
        _hs2 = hs2;
        InternalPerform(tol, approx, approxS1, approxS2, false, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>Intersects S1 and S2 from the starting point (U1, V1) on S1 and (U2, V2) on S2.</summary>
    public void Perform(Surface s1, Surface s2, double tol, double u1, double v1, double u2, double v2,
                        bool approx = true, bool approxS1 = false, bool approxS2 = false)
    {
        _hs1 = new SurfaceAdaptor(s1);
        if (s1 == s2)
        {
            _hs2 = _hs1;
        }
        else
        {
            _hs2 = new SurfaceAdaptor(s2);
        }
        InternalPerform(tol, approx, approxS1, approxS2, true, u1, v1, u2, v2);
    }

    /// <summary>Intersects two surface adaptors from a starting point.</summary>
    public void Perform(SurfaceAdaptor hs1, SurfaceAdaptor hs2, double tol, double u1, double v1, double u2,
                        double v2, bool approx = true, bool approxS1 = false, bool approxS2 = false)
    {
        _hs1 = hs1;
        _hs2 = hs2;
        InternalPerform(tol, approx, approxS1, approxS2, true, u1, v1, u2, v2);
    }

    /// <summary>True if the intersection was computed.</summary>
    public bool IsDone => _intersector.IsDone;

    /// <summary>The largest tolerance reached by the 3D curves.</summary>
    public double TolReached3d() => _tolReached3d;

    /// <summary>The largest tolerance reached by the 2D curves.</summary>
    public double TolReached2d() => _tolReached2d;

    /// <summary>The number of intersection lines.</summary>
    public int LineCount => _sline.Length() - _nbrestr;

    /// <summary>The intersection line of index Index, frozen. Throws StdFail_NotDone if not done.</summary>
    public Curve3D Line(int index)
    {
        StdFail_NotDone_Raise_if(!_intersector.IsDone, "GeomInt_IntSS::Line");
        Curve3D c = _sline.Value(index + _nbrestr);
        c.Freeze();
        return c;
    }

    /// <summary>True if the line of index Index has a 2D curve on the first surface. Throws StdFail_NotDone if not
    /// done.</summary>
    public bool HasLineOnS1(int index)
    {
        StdFail_NotDone_Raise_if(!_intersector.IsDone, "GeomInt_IntSS::HasLineOnS1");
        return (_slineS1.Value(index) is not null);
    }

    /// <summary>The 2D curve on the first surface of the line of index Index, frozen, or null. Throws
    /// StdFail_NotDone if not done.</summary>
    public Curve2D? LineOnS1(int index)
    {
        StdFail_NotDone_Raise_if(!_intersector.IsDone, "GeomInt_IntSS::LineOnS1");
        Curve2D? c = _slineS1.Value(index);
        c?.Freeze();
        return c;
    }

    /// <summary>True if the line of index Index has a 2D curve on the second surface. Throws StdFail_NotDone if not
    /// done.</summary>
    public bool HasLineOnS2(int index)
    {
        StdFail_NotDone_Raise_if(!_intersector.IsDone, "GeomInt_IntSS::HasLineOnS2");
        return (_slineS2.Value(index) is not null);
    }

    /// <summary>The 2D curve on the second surface of the line of index Index, frozen, or null. Throws
    /// StdFail_NotDone if not done.</summary>
    public Curve2D? LineOnS2(int index)
    {
        StdFail_NotDone_Raise_if(!_intersector.IsDone, "GeomInt_IntSS::LineOnS2");
        Curve2D? c = _slineS2.Value(index);
        c?.Freeze();
        return c;
    }

    /// <summary>The number of boundary lines (always 0: OCCT never records any). Throws StdFail_NotDone if not
    /// done.</summary>
    public int BoundaryCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_intersector.IsDone, "GeomInt_IntSS::NbBoundaries() - no result");
            return _nbrestr;
        }
    }

    /// <summary>The boundary line of index Index, frozen. Throws StdFail_NotDone if not done, Standard_OutOfRange if
    /// Index is out of range.</summary>
    public Curve3D Boundary(int index)
    {
        StdFail_NotDone_Raise_if(!_intersector.IsDone, "GeomInt_IntSS::Line");
        Standard_OutOfRange_Raise_if(index <= 0 || index > _nbrestr, "GeomInt_IntSS::Boundary");
        Curve3D c = _sline.Value(index);
        c.Freeze();
        return c;
    }

    /// <summary>The number of isolated intersection points.</summary>
    public int PointCount => _intersector.PntCount;

    /// <summary>The isolated intersection point of index Index.</summary>
    public Point3D Point(int index) => _intersector.Point(index).Value();

    /// <summary>The parameters on the first (OnFirst) or second surface of the point of index Index.</summary>
    public Point2D Pnt2d(int index, bool onFirst)
    {
        PatchPoint thept = _intersector.Point(index);
        double u, v;
        if (onFirst)
        {
            thept.ParametersOnS1(out u, out v);
        }
        else
        {
            thept.ParametersOnS2(out u, out v);
        }
        return new Point2D(u, v);
    }

    /// <summary>Sets the tolerances of GeomLib_CheckBSplineCurve's tangent fixes of the approximated curves.</summary>
    public void SetTolFixTangents(double aTolCheck, double aTolAngCheck)
    {
        _tolCheck = aTolCheck;
        _tolAngCheck = aTolAngCheck;
    }

    /// <summary>The tolerances of the tangent fixes.</summary>
    public void TolFixTangents(out double aTolCheck, out double aTolAngCheck)
    {
        aTolCheck = _tolCheck;
        aTolAngCheck = _tolAngCheck;
    }

    protected void InternalPerform(double tol, bool approx, bool approxS1, bool approxS2, bool useStart, double u1,
                                   double v1, double u2, double v2)
    {
        _tolReached2d = _tolReached3d = 0.0;
        _nbrestr = 0;
        _sline.Clear();
        // OCCT: slineS1 and slineS2 aren't cleared, so after a second Perform LineOnS1/S2(i) and HasLineOnS1/S2(i)
        // read the first Perform's 2D curves (the index isn't shifted as Line's is).
        double tolArc = tol;
        double tolTang = tol;
        double deflection = 0.1;
        if (_hs1!.GetType() == SurfaceKind.BSplineSurface && _hs2!.GetType() == SurfaceKind.BSplineSurface)
        {
            deflection /= 10.0;
        }

        TopolTool dom1 = new(_hs1);
        TopolTool dom2 = new(_hs2!);

        NCollection_DynamicArray<SurfaceAdaptorBase> vecHS1 = new();
        NCollection_DynamicArray<SurfaceAdaptorBase> vecHS2 = new();

        if (_hs1 == _hs2)
        {
            vecHS1.Append(_hs1);
            vecHS2.Append(_hs2);
        }
        else
        {
            PatchIntersection.PrepareSurfaces(_hs1, dom1, _hs2!, dom2, tol, vecHS1, vecHS2);
        }

        for (int numOfHS1 = 0; numOfHS1 < vecHS1.Length(); numOfHS1++)
        {
            SurfaceAdaptorBase hs1 = vecHS1.Value(numOfHS1);
            for (int numOfHS2 = 0; numOfHS2 < vecHS2.Length(); numOfHS2++)
            {
                SurfaceAdaptorBase hs2 = vecHS2.Value(numOfHS2);

                TopolTool aDom1 = new(hs1);
                TopolTool aDom2 = new(hs2);

                _lConstruct.Load(aDom1, aDom2, hs1 as SurfaceAdaptor, hs2 as SurfaceAdaptor);

                double uvMaxStep = PatchIntersection.DefineUVMaxStep(hs1, aDom1, hs2, aDom2);

                _intersector.SetTolerances(tolArc, tolTang, uvMaxStep, deflection);

                if (hs1 == hs2)
                {
                    _intersector.Perform(hs1, aDom1, tolArc, tolTang);
                }
                else if (!useStart)
                {
                    _intersector.Perform(hs1, aDom1, hs2, aDom2, tolArc, tolTang);
                }
                else
                {
                    Containment state1 = aDom1.Classify(new Point2D(u1, v1), tol);
                    Containment state2 = aDom2.Classify(new Point2D(u2, v2), tol);

                    if ((state1 == In || state1 == On) && (state2 == In || state2 == On))
                    {
                        _intersector.Perform(hs1, aDom1, hs2, aDom2, u1, v1, u2, v2, tolArc, tolTang);
                    }
                }

                // ============================================================
                if (_intersector.IsDone)
                {
                    int nblin = _intersector.LineCount;
                    for (int i = 1; i <= nblin; i++)
                    {
                        MakeCurve(i, aDom1, aDom2, tol, approx, approxS1, approxS2);
                    }
                }
            }
        }
    }
}
