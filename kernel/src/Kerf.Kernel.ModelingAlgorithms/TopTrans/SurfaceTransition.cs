// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_SurfaceTransition.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_SurfaceTransition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Legacy;

/// <summary>
/// Computes the transition of a 3D surface intersecting a topological surfacic boundary on a 3D curve (the
/// intersection curve).
/// </summary>
/// <remarks>
/// The boundary is a set of faces, each described by its support surface and an orientation defining its matter
/// side. The geometric elements are described locally at the intersection point by a second order development.
/// The algorithm keeps the two face elements closest to the part of the curve "before" and "after" the
/// intersection, updated for each new face; once one surface element has been given, the position of the curve
/// before and after the intersection can be computed. A working object (decision 21): Reset and Compare are
/// public, as in OCCT. OCCT declares the private <c>UpdateReference</c> and <c>ComputeCos</c> but never defines
/// them: left out.
/// </remarks>
internal class SurfaceTransition
{
    private const int MUnknown = -100;
    private const int MNoupdate = 0;
    private const int MUpdateREF = 1;
    private const int MOinternal = 10;

    private const int Before = 2;
    private const int After = 1;

    private Direction3D _tgt = new();
    private Direction3D _norm = new();
    private Direction3D _beafter = new();
    private double _curvRef;
    private readonly NCollection_Array2<double> _ang;
    private readonly NCollection_Array2<double> _curv;
    private readonly NCollection_Array2<Orientation> _ori;
    private bool _isDefined;
    private bool _touchFlag;

    /// <summary>Creates an empty surface transition.</summary>
    public SurfaceTransition()
    {
        _curvRef = 0.0;
        _ang = new NCollection_Array2<double>(1, 2, 1, 2);
        _curv = new NCollection_Array2<double>(1, 2, 1, 2);
        _ori = new NCollection_Array2<Orientation>(1, 2, 1, 2);
        _isDefined = false;
        _touchFlag = false;
    }

    /// <summary>
    /// Initializes the transition with the local description of the intersection curve and of the reference
    /// surface. <paramref name="norm"/> must be oriented outside the geometric matter.
    /// </summary>
    public void Reset(in Direction3D tgt, in Direction3D norm, in Direction3D maxD, in Direction3D minD, double maxCurv, double minCurv)
    {
        _isDefined = false;

        _norm = norm;
        _tgt = tgt;
        _beafter = norm ^ tgt;

        double tola = global::Kerf.Engine.Foundation.Precision.Angular();
        bool curismax = Math.Abs(maxD.Dot(tgt)) < tola;
        bool curismin = Math.Abs(minD.Dot(tgt)) < tola;

        if ((Math.Abs(maxCurv) < tola) && (Math.Abs(minCurv) < tola))
        {
            Reset(tgt, norm);
            return;
        }

        if (!curismax && !curismin)
        {
            // In the plane normal to <myTgt>, we see the boundary face as
            // a boundary curve.
            // NYIxpu : compute the curvature of the curve if not MaxCurv
            //          nor MinCurv.
            return;
        }

        if (curismax)
        {
            _curvRef = Math.Abs(maxCurv);
        }

        if (curismin)
        {
            _curvRef = Math.Abs(minCurv);
        }

        if (_curvRef < tola)
        {
            _curvRef = 0.0;
        }

        // recall : <Norm> is oriented OUTSIDE the "geometric matter" described
        //          by the surface
        //          -  if (myCurvRef != 0.) Sref is UNDER axis (sin = 0)
        //             referential (beafter,myNorm,myTgt)  -
        for (int i = 1; i <= 2; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                _ang[i, j] = 100.0;
            }
        }

        _touchFlag = false; // eap Mar 25 2002
        _isDefined = true;
    }

    /// <summary>Initializes the transition with the local description of a straight line.</summary>
    public void Reset(in Direction3D tgt, in Direction3D norm)
    {
        _isDefined = false;

        // beafter oriented (before, after) the intersection on the reference surface.
        _norm = norm;
        _tgt = tgt;
        _beafter = norm ^ tgt;
        for (int i = 1; i <= 2; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                _ang[i, j] = 100.0;
            }
        }

        _curvRef = 0.0;
        _touchFlag = false; // eap Mar 25 2002
        _isDefined = true;
    }

    /// <summary>
    /// Adds a face element to the boundary.
    /// </summary>
    /// <remarks>
    /// <paramref name="s"/> is the face's topological orientation: FORWARD means that along the intersection curve
    /// on the reference surface the states while crossing the face are OUT, IN; REVERSED IN, OUT; INTERNAL IN, IN.
    /// <paramref name="o"/> is the curve's position on the face: FORWARD means the face is before the
    /// intersection, REVERSED after it, INTERNAL that the intersection is inside the face.
    /// <paramref name="norm"/> must be oriented outside the geometric matter.
    /// </remarks>
    public void Compare(double tole, in Direction3D norm, in Direction3D maxD, in Direction3D minD, double maxCurv, double minCurv,
                        Orientation s, Orientation o)
    {
        if (!_isDefined)
        {
            return;
        }

        double curv = 0.0;
        double tola = (tole > 0.0) ? tole : global::Kerf.Engine.Foundation.Precision.Angular();
        bool curismax = Math.Abs(maxD.Dot(_tgt)) < tola;
        bool curismin = Math.Abs(minD.Dot(_tgt)) < tola;
        if (!curismax && !curismin)
        {
            // In the plane normal to <myTgt>, we see the boundary face as
            // a boundary curve.
            // NYIxpu : compute the curvature of the curve if not MaxCurv
            //          nor MinCurv.
            _isDefined = false;
            return;
        }

        if (curismax)
        {
            curv = Math.Abs(maxCurv);
        }

        if (curismin)
        {
            curv = Math.Abs(minCurv);
        }

        if (_curvRef < tola)
        {
            curv = 0.0;
        }

        Direction3D dironF = FUN_nCinsideS(_tgt, norm);
        double prod = (dironF ^ norm).Dot(_tgt);
        if (prod < 0.0)
        {
            curv = -curv;
        }

        double ang;
        ang = FUN_Ang(_norm, _beafter, _tgt, norm, o);

        // i = 0,1,2 : cos = 0,>0,<0
        // j = 0,1,2 : sin = 0,>0,<0
        FUN_getSTA(ang, tola, out int i, out int j);

        // update nearest :
        int kmax = MInternal(o) ? 2 : 1;
        for (int k = 1; k <= kmax; k++)
        {
            if (k == 2)
            {
                // get the opposite Ang
                i = FUN_OO(i);
                j = FUN_OO(j);
            }

            bool i0 = i == 0, j0 = j == 0;
            int nmax = (i0 || j0) ? 2 : 1;
            for (int n = 1; n <= nmax; n++)
            {
                if (i0)
                {
                    i = n;
                }

                if (j0)
                {
                    j = n;
                }

                int refn = FUN_refnearest(i,
                                          j,
                                          _curvRef,
                                          _ang[i, j],
                                          _ori[i, j],
                                          _curv[i, j],
                                          ang,
                                          /*O*/ s,
                                          curv,
                                          tola,
                                          ref _touchFlag); // eap Mar 25 2002
                if (refn == MUnknown)
                {
                    _isDefined = false;
                    return;
                }

                if (refn > 0)
                {
                    _ang[i, j] = ang;
                    _ori[i, j] = (refn == MOinternal) ? Internal : s;
                    _curv[i, j] = curv;
                }
            } // n=1..nmax
        } // k=1..kmax
    }

    /// <summary>Adds a plane or a cylindrical face to the boundary.</summary>
    public void Compare(double tole, in Direction3D norm, Orientation s, Orientation o)
    {
        if (!_isDefined)
        {
            return;
        }

        // oriented Ang(beafter,dironF),
        // dironF normal to the curve, oriented INSIDE F, the added oriented support
        double ang = FUN_Ang(_norm, _beafter, _tgt, norm, o);
        double tola = (tole > 0.0) ? tole : global::Kerf.Engine.Foundation.Precision.Angular();

        // i = 0,1,2 : cos = 0,>0,<0
        // j = 0,1,2 : sin = 0,>0,<0
        FUN_getSTA(ang, tola, out int i, out int j);

        int kmax = MInternal(o) ? 2 : 1;
        for (int k = 1; k <= kmax; k++)
        {
            if (k == 2)
            {
                // get the opposite Ang
                i = FUN_OO(i);
                j = FUN_OO(j);
            }

            bool i0 = i == 0, j0 = j == 0;
            int nmax = (i0 || j0) ? 2 : 1;
            for (int n = 1; n <= nmax; n++)
            {
                if (i0)
                {
                    i = n;
                }

                if (j0)
                {
                    j = n;
                }

                int refn = FUN_refnearest(_ang[i, j], _ori[i, j], ang, /*O*/ s, tola); // eap

                if (refn == MUnknown)
                {
                    _isDefined = false;
                    return;
                }

                if (refn > 0)
                {
                    _ang[i, j] = ang;
                    _ori[i, j] = (refn == MOinternal) ? Internal : s;
                }
            } // n=1..nmax
        } // k=1..kmax
    }

    /// <summary>
    /// The state of the reference surface before the interference: the position relative to the surface of a
    /// point very close to the intersection on the negative side of the tangent.
    /// </summary>
    public Containment StateBefore()
    {
        if (!_isDefined)
        {
            return Unknown;
        }

        // we take the state before of before orientations
        Containment before = FUN_getstate(_ang, _ori, Before, Before);
        if (M_UNKNOWN(before))
        {
            // looking back in before for defined states
            // we take the state before of after orientations
            before = FUN_getstate(_ang, _ori, After, Before);
            // eap Mar 25 2002
            if (_touchFlag)
            {
                if (before == Out)
                {
                    before = In;
                }
                else if (before == In)
                {
                    before = Out;
                }
            }
        }

        return before;
    }

    /// <summary>
    /// The state of the reference surface after the interference: the position relative to the surface of a
    /// point very close to the intersection on the positive side of the tangent.
    /// </summary>
    public Containment StateAfter()
    {
        if (!_isDefined)
        {
            return Unknown;
        }

        Containment after = FUN_getstate(_ang, _ori, After, After);
        if (M_UNKNOWN(after))
        {
            // looking back in before for defined states
            after = FUN_getstate(_ang, _ori, Before, After);
            // eap Mar 25 2002
            if (_touchFlag)
            {
                if (after == Out)
                {
                    after = In;
                }
                else if (after == In)
                {
                    after = Out;
                }
            }
        }

        return after;
    }

    /// <summary>The state before a transition of orientation <paramref name="tran"/>.</summary>
    public static Containment GetBefore(Orientation tran)
    {
        switch (tran)
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

    /// <summary>The state after a transition of orientation <paramref name="tran"/>.</summary>
    public static Containment GetAfter(Orientation tran)
    {
        switch (tran)
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

    // File-static functions of TopTrans_SurfaceTransition.cxx

    private static bool MReversed(Orientation st) => st == Reversed;

    private static bool MInternal(Orientation st) => st == Internal;

    private static bool M_UNKNOWN(Containment st) => st == Unknown;

    // Modified: eap Mar 25 2002 (occ102,occ227), touch case
    private static Direction3D FUN_nCinsideS(in Direction3D tgC, in Direction3D ngS)
    {
        // Give us a curve C on surface S, <parOnC>, a parameter
        // Purpose : compute normal vector to C, tangent to S at
        //           given point , oriented INSIDE S
        // <tgC> : geometric tangent at point of <parOnC>
        // <ngS> : geometric normal at point of <parOnC>
        Direction3D xx = ngS ^ tgC;
        return xx;
    }

    private static int FUN_OO(int i)
    {
        if (i == 1)
        {
            return 2;
        }

        if (i == 2)
        {
            return 1;
        }

        return 0;
    }

    private static double FUN_Ang(in Direction3D _, in Direction3D beafter, in Direction3D tgC, in Direction3D norm, Orientation o)
    {
        Direction3D dironF = FUN_nCinsideS(tgC, norm);
        if (MReversed(o))
        {
            dironF = dironF.Reversed();
        }

        double ang = beafter.AngleWithRef(dironF, tgC);
        return ang;
    }

    private static void FUN_getSTA(double ang, double tola, out int i, out int j)
    {
        double cos = Math.Cos(ang);
        double sin = Math.Sin(ang);
        bool nullcos = Math.Abs(cos) < tola;
        bool nullsin = Math.Abs(sin) < tola;
        if (nullcos)
        {
            i = 0;
        }
        else
        {
            i = (cos > 0.0) ? 1 : 2;
        }

        if (nullsin)
        {
            j = 0;
        }
        else
        {
            j = (sin > 0.0) ? 1 : 2;
        }
    }

    private static int FUN_refnearest(double angref, Orientation oriref, double ang, Orientation ori,
                                      double tola)
    {
        bool undef = angref == 100.0;
        if (undef)
        {
            return MUpdateREF;
        }

        double cosref = Math.Cos(angref), cos = Math.Cos(ang);
        double dcos = Math.Abs(cosref) - Math.Abs(cos);
        if (Math.Abs(dcos) < tola)
        {
            // Analysis for tangent cases : if two boundary faces are same sided
            // and have tangent normals, if they have opposite orientations
            // we choose INTERNAL as resulting complex transition (case EXTERNAL
            // referring to no logical case)
            if (global::Kerf.Engine.Topology.TopAbs.Complement(ori) == oriref)
            {
                return MOinternal;
            }
            else
            {
                return MUnknown; // nyi FUN_RAISE
            }
        }

        int updateref = (dcos > 0.0) ? MNoupdate : MUpdateREF;
        return updateref;
    }

    private static int FUN_refnearest(int i, int j, double curvSref, double angref, Orientation oriref,
                                      double curvref, double ang, Orientation ori, double curv, double tola,
                                      ref bool touchFlag) // eap Mar 25 2002
    {
        bool iisj = i == j;
        double abscos = Math.Abs(Math.Cos(ang));
        bool i0 = Math.Abs(1.0 - abscos) < tola;
        bool j0 = abscos < tola;
        bool nullcurv = curv == 0.0;
        bool curvpos = curv > tola;
        bool curvneg = curv < -tola;
        bool nullcsref = curvSref == 0.0;

        bool undef = angref == 100.0;
        if (undef)
        {
            if (i0)
            {
                if (iisj && curvneg)
                {
                    return MNoupdate;
                }

                if (!iisj && curvpos)
                {
                    return MNoupdate;
                }
            }

            if (j0)
            {
                if (!nullcsref && (j == 1) && iisj && (curvpos || nullcurv))
                {
                    return MUpdateREF;
                }

                if (!nullcsref && (j == 1) && !iisj && (curvneg || nullcurv))
                {
                    return MUpdateREF;
                }

                if (iisj && curvpos)
                {
                    return MNoupdate;
                }

                if (!iisj && curvneg)
                {
                    return MNoupdate;
                }
            }

            return MUpdateREF;
        } // undef

        double cosref = Math.Cos(angref), cos = Math.Cos(ang);
        double dcos = Math.Abs(cosref) - Math.Abs(cos);
        bool samecos = Math.Abs(dcos) < tola;
        if (samecos)
        {
            // Analysis for tangent cases : if two boundary faces are same sided
            // and have sma dironF.
            if (Math.Abs(curvref - curv) < 1.0e-4)
            {
                if (global::Kerf.Engine.Topology.TopAbs.Complement(ori) == oriref)
                {
                    return MOinternal;
                }
                else
                {
                    return MUnknown; // nyi FUN_RAISE
                }
            }

            bool noupdate = false;
            if (iisj && (curvref > curv))
            {
                noupdate = true;
            }

            if (!iisj && (curvref < curv))
            {
                noupdate = true;
            }

            int updateref1 = noupdate ? MNoupdate : MUpdateREF;
            if (!j0)
            {
                return updateref1;
            }

            if (!noupdate && !nullcsref)
            {
                // check for (j==1) the face is ABOVE Sref
                // check for (j==2) the face is BELOW Sref
                if ((j == 2) && (Math.Abs(curv) < curvSref))
                {
                    updateref1 = MNoupdate;
                }

                if ((j == 1) && (Math.Abs(curv) > curvSref))
                {
                    updateref1 = MNoupdate;
                }
            }

            return updateref1;
        } // samecos

        int updateref = (dcos > 0.0) ? MNoupdate : MUpdateREF;
        if (oriref != ori)
        {
            touchFlag = true; // eap Mar 25 2002
        }

        return updateref;
    }

    private static Containment FUN_getstate(NCollection_Array2<double> ang, NCollection_Array2<Orientation> ori,
                                             int iSTA, int iINDEX)
    {
        double a1 = ang[iSTA, 1], a2 = ang[iSTA, 2];
        bool undef1 = a1 == 100.0, undef2 = a2 == 100.0;
        bool undef = undef1 && undef2;
        if (undef)
        {
            return Unknown;
        }

        if (undef1 || undef2)
        {
            int jok = undef1 ? 2 : 1;
            Orientation o = ori[iSTA, jok];
            Containment st = (iINDEX == Before) ? GetBefore(o) : GetAfter(o);
            return st;
        }

        Orientation o1 = ori[iSTA, 1], o2 = ori[iSTA, 2];
        Containment st1 = (iINDEX == Before) ? GetBefore(o1) : GetAfter(o1);
        Containment st2 = (iINDEX == Before) ? GetBefore(o2) : GetAfter(o2);
        if (st1 != st2)
        {
            return Unknown; // Incoherent data
        }

        return st1;
    }
}
