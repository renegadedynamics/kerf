// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_CurveTransition.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_CurveTransition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Legacy;

/// <summary>
/// Computes the transition of a curve intersecting a curvilinear boundary.
/// </summary>
/// <remarks>
/// The geometric elements are described locally at the intersection point by a second order development: the
/// curve by its tangent, normal and curvature; the boundary by a set of curve elements, each a curve or a
/// half-curve (a curve and an orientation saying whether the boundary is before or after the intersection).
/// A working object (decision 21): <see cref="Reset(in Direction3D, in Direction3D, double)"/> and
/// <see cref="Compare(double, in Direction3D, in Direction3D, double, Orientation, Orientation)"/> are public,
/// as in OCCT. A class: OCCT holds it by value as a member of the classifiers, which pass it by reference.
/// </remarks>
internal class CurveTransition
{
    private const int Greater = 1;
    private const int Same = 0;
    private const int Lower = -1;

    private Direction3D _tgt = new();
    private Direction3D _norm = new();
    private double _curv;
    private bool _init;
    private Direction3D _tgtFirst = new();
    private Direction3D _normFirst = new();
    private double _curvFirst;
    private Orientation _tranFirst;
    private Direction3D _tgtLast = new();
    private Direction3D _normLast = new();
    private double _curvLast;
    private Orientation _tranLast;

    /// <summary>Creates an empty curve transition.</summary>
    public CurveTransition()
    {
        _curv = 0.0;
        _init = false;
        _curvFirst = 0.0;
        _curvLast = 0.0;
        _tranFirst = Forward;
        _tranLast = Forward;
    }

    /// <summary>Initializes the transition with the local description of a curve.</summary>
    public void Reset(in Direction3D tgt, in Direction3D norm, double curv)
    {
        _tgt = tgt;
        _norm = norm;
        _curv = curv;
        _init = true;
    }

    /// <summary>Initializes the transition with the local description of a straight line.</summary>
    public void Reset(in Direction3D tgt)
    {
        _tgt = tgt;
        _curv = 0.0;
        _init = true;
    }

    /// <summary>
    /// Adds a curve element to the boundary. If <paramref name="or"/> is REVERSED the curve is before the
    /// intersection, if FORWARD after it, and if INTERNAL the intersection is in the middle of the curve.
    /// </summary>
    public void Compare(double tole, in Direction3D t, in Direction3D n, double c, Orientation st, Orientation or)
    {
        // S is the transition, how the curve cross the boundary
        // O is the orientation, how the intersection is set on the boundary
        Orientation s = st;
        Orientation o = or;

        // adjustment for INTERNAL transition
        if (s == Internal)
        {
            if (t * _tgt < 0)
            {
                s = global::Kerf.Engine.Topology.TopAbs.Reverse(o);
            }
            else
            {
                s = o;
            }
        }

        // It is the first comparison for this complex transition
        if (_init)
        {
            _init = false;
            _tgtFirst = t;
            _normFirst = n;
            _curvFirst = c;
            _tranFirst = s;
            _tgtLast = t;
            _normLast = n;
            _curvLast = c;
            _tranLast = s;
            switch (o)
            {
                // Interference at the end of the edge: the tangent must be reversed
                case Reversed:
                    _tgtFirst = _tgtFirst.Reversed();
                    _tgtLast = _tgtLast.Reversed();
                    break;
                case Internal:
                    // Interference in the middle of the edge: reverse depending on the position of the
                    // reference tangent
                    if (_tgt * t > 0)
                    {
                        _tgtFirst = _tgtFirst.Reversed();
                    }
                    else
                    {
                        _tgtLast = _tgtLast.Reversed();
                    }

                    break;
                case Forward:
                case External:
                    break;
            }
        }
        // Compare with the existent first and last transition :
        else
        {
            bool firstSet = false;
            double cosAngWithT = _tgt * t;
            switch (o)
            {
                case Reversed:
                    cosAngWithT = -cosAngWithT;
                    break;
                case Internal:
                    if (cosAngWithT > 0)
                    {
                        cosAngWithT = -cosAngWithT;
                    }

                    break;
                case Forward:
                case External:
                    break;
            }

            double cosAngWith1 = _tgt * _tgtFirst;

            switch (Compare(cosAngWithT, cosAngWith1, tole))
            {
                case Lower:
                    // If the angle is greater than the first the new become the first
                    firstSet = true;
                    _tgtFirst = t;
                    switch (o)
                    {
                        case Reversed:
                            _tgtFirst = _tgtFirst.Reversed();
                            break;
                        case Internal:
                            if (_tgt * t > 0)
                            {
                                _tgtFirst = _tgtFirst.Reversed();
                            }

                            break;
                        case Forward:
                        case External:
                            break;
                    }

                    _normFirst = n;
                    _curvFirst = c;
                    _tranFirst = s;
                    break;

                case Same:
                    // If same angles we look at the Curvature
                    if (IsBefore(tole, cosAngWithT, n, c, _normFirst, _curvFirst))
                    {
                        firstSet = true;
                        _tgtFirst = t;
                        switch (o)
                        {
                            case Reversed:
                                _tgtFirst = _tgtFirst.Reversed();
                                break;
                            case Internal:
                                if (_tgt * t > 0)
                                {
                                    _tgtFirst = _tgtFirst.Reversed();
                                }

                                break;
                            case Forward:
                            case External:
                                break;
                        }

                        _normFirst = n;
                        _curvFirst = c;
                        _tranFirst = s;
                    }

                    break;

                case Greater:
                    break;
            }

            if (!firstSet || o == Internal)
            {
                // In the tangent cases the first can also be the last
                if (o == Internal)
                {
                    cosAngWithT = -cosAngWithT;
                }

                double cosAngWith2 = _tgt * _tgtLast;

                switch (Compare(cosAngWithT, cosAngWith2, tole))
                {
                    case Greater:
                        // If the angle is lower than the last the new become the last
                        _tgtLast = t;
                        switch (o)
                        {
                            case Reversed:
                                _tgtLast = _tgtLast.Reversed();
                                break;
                            case Internal:
                                if (_tgt * t < 0)
                                {
                                    _tgtLast = _tgtLast.Reversed();
                                }

                                break;
                            case Forward:
                            case External:
                                break;
                        }

                        _normLast = n;
                        _curvLast = c;
                        _tranLast = s;
                        break;

                    case Same:
                        // If the angle is the same we look at the curvature
                        if (IsBefore(tole, cosAngWithT, _normLast, _curvLast, n, c))
                        {
                            _tgtLast = t;
                            switch (o)
                            {
                                case Reversed:
                                    _tgtLast = _tgtLast.Reversed();
                                    break;
                                case Internal:
                                    if (_tgt * t < 0)
                                    {
                                        _tgtLast = _tgtLast.Reversed();
                                    }

                                    break;
                                case Forward:
                                case External:
                                    break;
                            }

                            _normLast = n;
                            _curvLast = c;
                            _tranLast = s;
                        }

                        break;
                }
            }
        }
    }

    /// <summary>
    /// The state of the curve before the intersection: the position relative to the boundary of a point very
    /// close to the intersection on the negative side of the tangent.
    /// </summary>
    public Containment StateBefore()
    {
        if (_init)
        {
            return Unknown;
        }

        switch (_tranFirst)
        {
            case Forward:
            case External:
                return Out;
            case Reversed:
            case Internal:
                return In;
        }

        return Out;
    }

    /// <summary>
    /// The state of the curve after the intersection: the position relative to the boundary of a point very
    /// close to the intersection on the positive side of the tangent.
    /// </summary>
    public Containment StateAfter()
    {
        if (_init)
        {
            return Unknown;
        }

        switch (_tranLast)
        {
            case Forward:
            case Internal:
                return In;
            case Reversed:
            case External:
                return Out;
        }

        return Out;
    }

    /// <summary>Compares the curvature of two transitions: true if (N1, C1) is before (N2, C2).</summary>
    private bool IsBefore(double tole, double cosAngl, in Direction3D n1, double c1, in Direction3D n2, double c2)
    {
        double tn1 = _tgt * n1;
        double tn2 = _tgt * n2;
        bool oneBefore = false;

        if (Math.Abs(tn1) <= tole || Math.Abs(tn2) <= tole)
        {
            // Tangent : The first is the interference which have the nearest curvature
            //           from the reference.
            if (_curv == 0)
            {
                // The reference is straight
                // The first is the interference which have the lowest curvature.
                if (c1 < c2)
                {
                    oneBefore = true;
                }

                //  Modified by Sergey KHROMOV - Wed Dec 27 17:08:49 2000
                if (cosAngl > 0)
                {
                    oneBefore = !oneBefore;
                }
            }
            else
            {
                // The reference is curv
                // The first is the interference which have the nearest curvature
                // in the direction
                double deltaC1, deltaC2;
                if (c1 == 0.0 || _curv == 0.0)
                {
                    deltaC1 = c1 - _curv;
                }
                else
                {
                    deltaC1 = (c1 - _curv) * (n1 * _norm);
                }

                if (c2 == 0.0 || _curv == 0.0)
                {
                    deltaC2 = c2 - _curv;
                }
                else
                {
                    deltaC2 = (c2 - _curv) * (n2 * _norm);
                }

                if (deltaC1 < deltaC2)
                {
                    oneBefore = true;
                }

                if (cosAngl > 0)
                {
                    oneBefore = !oneBefore;
                }
            }
        }
        else if (tn1 < 0)
        {
            // Before the first interference we are in the curvature
            if (tn2 > 0)
            {
                // Before the second  interference we are out the curvature
                // The first interference is before  /* ->)( */
                oneBefore = true;
            }
            else
            {
                // Before the second interference we are in the curvature
                if (c1 > c2)
                {
                    // We choice the greater curvature
                    // The first interference is before   /* ->)) */
                    oneBefore = true;
                }
            }
        }
        else if (tn1 > 0)
        {
            // Before the first interference we are out the curvature
            if (tn2 > 0)
            {
                // Before the second interference we are out the curvature /* ->(( */
                if (c1 < c2)
                {
                    // We choice the lower curvature
                    // The first interference is before
                    oneBefore = true;
                }
            }
        }

        return oneBefore;
    }

    /// <summary>Compares two angles at tolerance Tole.</summary>
    private int Compare(double ang1, double ang2, double tole)
    {
        int res = Same;
        if (ang1 - ang2 > tole)
        {
            res = Greater;
        }
        else if (ang2 - ang1 > tole)
        {
            res = Lower;
        }

        return res;
    }
}
