// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_SurfaceTransition.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_SurfaceTransition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

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
public class TopTrans_SurfaceTransition
{
    private const int M_Unknown = -100;
    private const int M_noupdate = 0;
    private const int M_updateREF = 1;
    private const int M_Ointernal = 10;

    private const int BEFORE = 2;
    private const int AFTER = 1;

    private Direction3D myTgt = new();
    private Direction3D myNorm = new();
    private Direction3D beafter = new();
    private double myCurvRef;
    private readonly NCollection_Array2<double> myAng;
    private readonly NCollection_Array2<double> myCurv;
    private readonly NCollection_Array2<TopAbs_Orientation> myOri;
    private bool myIsDefined;
    private bool myTouchFlag;

    /// <summary>Creates an empty surface transition.</summary>
    public TopTrans_SurfaceTransition()
    {
        myCurvRef = 0.0;
        myAng = new NCollection_Array2<double>(1, 2, 1, 2);
        myCurv = new NCollection_Array2<double>(1, 2, 1, 2);
        myOri = new NCollection_Array2<TopAbs_Orientation>(1, 2, 1, 2);
        myIsDefined = false;
        myTouchFlag = false;
    }

    /// <summary>
    /// Initializes the transition with the local description of the intersection curve and of the reference
    /// surface. <paramref name="Norm"/> must be oriented outside the geometric matter.
    /// </summary>
    public void Reset(in Direction3D Tgt, in Direction3D Norm, in Direction3D MaxD, in Direction3D MinD, double MaxCurv, double MinCurv)
    {
        myIsDefined = false;

        myNorm = Norm;
        myTgt = Tgt;
        beafter = Norm ^ Tgt;

        double tola = global::Kerf.Engine.Foundation.Precision.Angular();
        bool curismax = Math.Abs(MaxD.Dot(Tgt)) < tola;
        bool curismin = Math.Abs(MinD.Dot(Tgt)) < tola;

        if ((Math.Abs(MaxCurv) < tola) && (Math.Abs(MinCurv) < tola))
        {
            Reset(Tgt, Norm);
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
            myCurvRef = Math.Abs(MaxCurv);
        }

        if (curismin)
        {
            myCurvRef = Math.Abs(MinCurv);
        }

        if (myCurvRef < tola)
        {
            myCurvRef = 0.0;
        }

        // recall : <Norm> is oriented OUTSIDE the "geometric matter" described
        //          by the surface
        //          -  if (myCurvRef != 0.) Sref is UNDER axis (sin = 0)
        //             referential (beafter,myNorm,myTgt)  -
        for (int i = 1; i <= 2; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                myAng[i, j] = 100.0;
            }
        }

        myTouchFlag = false; // eap Mar 25 2002
        myIsDefined = true;
    }

    /// <summary>Initializes the transition with the local description of a straight line.</summary>
    public void Reset(in Direction3D Tgt, in Direction3D Norm)
    {
        myIsDefined = false;

        // beafter oriented (before, after) the intersection on the reference surface.
        myNorm = Norm;
        myTgt = Tgt;
        beafter = Norm ^ Tgt;
        for (int i = 1; i <= 2; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                myAng[i, j] = 100.0;
            }
        }

        myCurvRef = 0.0;
        myTouchFlag = false; // eap Mar 25 2002
        myIsDefined = true;
    }

    /// <summary>
    /// Adds a face element to the boundary.
    /// </summary>
    /// <remarks>
    /// <paramref name="S"/> is the face's topological orientation: FORWARD means that along the intersection curve
    /// on the reference surface the states while crossing the face are OUT, IN; REVERSED IN, OUT; INTERNAL IN, IN.
    /// <paramref name="O"/> is the curve's position on the face: FORWARD means the face is before the
    /// intersection, REVERSED after it, INTERNAL that the intersection is inside the face.
    /// <paramref name="Norm"/> must be oriented outside the geometric matter.
    /// </remarks>
    public void Compare(double Tole, in Direction3D Norm, in Direction3D MaxD, in Direction3D MinD, double MaxCurv, double MinCurv,
                        TopAbs_Orientation S, TopAbs_Orientation O)
    {
        if (!myIsDefined)
        {
            return;
        }

        double Curv = 0.0;
        double tola = (Tole > 0.0) ? Tole : global::Kerf.Engine.Foundation.Precision.Angular();
        bool curismax = Math.Abs(MaxD.Dot(myTgt)) < tola;
        bool curismin = Math.Abs(MinD.Dot(myTgt)) < tola;
        if (!curismax && !curismin)
        {
            // In the plane normal to <myTgt>, we see the boundary face as
            // a boundary curve.
            // NYIxpu : compute the curvature of the curve if not MaxCurv
            //          nor MinCurv.
            myIsDefined = false;
            return;
        }

        if (curismax)
        {
            Curv = Math.Abs(MaxCurv);
        }

        if (curismin)
        {
            Curv = Math.Abs(MinCurv);
        }

        if (myCurvRef < tola)
        {
            Curv = 0.0;
        }

        Direction3D dironF = FUN_nCinsideS(myTgt, Norm);
        double prod = (dironF ^ Norm).Dot(myTgt);
        if (prod < 0.0)
        {
            Curv = -Curv;
        }

        double Ang;
        Ang = FUN_Ang(myNorm, beafter, myTgt, Norm, O);

        // i = 0,1,2 : cos = 0,>0,<0
        // j = 0,1,2 : sin = 0,>0,<0
        FUN_getSTA(Ang, tola, out int i, out int j);

        // update nearest :
        int kmax = M_INTERNAL(O) ? 2 : 1;
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
                                          myCurvRef,
                                          myAng[i, j],
                                          myOri[i, j],
                                          myCurv[i, j],
                                          Ang,
                                          /*O*/ S,
                                          Curv,
                                          tola,
                                          ref myTouchFlag); // eap Mar 25 2002
                if (refn == M_Unknown)
                {
                    myIsDefined = false;
                    return;
                }

                if (refn > 0)
                {
                    myAng[i, j] = Ang;
                    myOri[i, j] = (refn == M_Ointernal) ? TopAbs_INTERNAL : S;
                    myCurv[i, j] = Curv;
                }
            } // n=1..nmax
        } // k=1..kmax
    }

    /// <summary>Adds a plane or a cylindrical face to the boundary.</summary>
    public void Compare(double Tole, in Direction3D Norm, TopAbs_Orientation S, TopAbs_Orientation O)
    {
        if (!myIsDefined)
        {
            return;
        }

        // oriented Ang(beafter,dironF),
        // dironF normal to the curve, oriented INSIDE F, the added oriented support
        double Ang = FUN_Ang(myNorm, beafter, myTgt, Norm, O);
        double tola = (Tole > 0.0) ? Tole : global::Kerf.Engine.Foundation.Precision.Angular();

        // i = 0,1,2 : cos = 0,>0,<0
        // j = 0,1,2 : sin = 0,>0,<0
        FUN_getSTA(Ang, tola, out int i, out int j);

        int kmax = M_INTERNAL(O) ? 2 : 1;
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

                int refn = FUN_refnearest(myAng[i, j], myOri[i, j], Ang, /*O*/ S, tola); // eap

                if (refn == M_Unknown)
                {
                    myIsDefined = false;
                    return;
                }

                if (refn > 0)
                {
                    myAng[i, j] = Ang;
                    myOri[i, j] = (refn == M_Ointernal) ? TopAbs_INTERNAL : S;
                }
            } // n=1..nmax
        } // k=1..kmax
    }

    /// <summary>
    /// The state of the reference surface before the interference: the position relative to the surface of a
    /// point very close to the intersection on the negative side of the tangent.
    /// </summary>
    public TopAbs_State StateBefore()
    {
        if (!myIsDefined)
        {
            return TopAbs_UNKNOWN;
        }

        // we take the state before of before orientations
        TopAbs_State before = FUN_getstate(myAng, myOri, BEFORE, BEFORE);
        if (M_UNKNOWN(before))
        {
            // looking back in before for defined states
            // we take the state before of after orientations
            before = FUN_getstate(myAng, myOri, AFTER, BEFORE);
            // eap Mar 25 2002
            if (myTouchFlag)
            {
                if (before == TopAbs_OUT)
                {
                    before = TopAbs_IN;
                }
                else if (before == TopAbs_IN)
                {
                    before = TopAbs_OUT;
                }
            }
        }

        return before;
    }

    /// <summary>
    /// The state of the reference surface after the interference: the position relative to the surface of a
    /// point very close to the intersection on the positive side of the tangent.
    /// </summary>
    public TopAbs_State StateAfter()
    {
        if (!myIsDefined)
        {
            return TopAbs_UNKNOWN;
        }

        TopAbs_State after = FUN_getstate(myAng, myOri, AFTER, AFTER);
        if (M_UNKNOWN(after))
        {
            // looking back in before for defined states
            after = FUN_getstate(myAng, myOri, BEFORE, AFTER);
            // eap Mar 25 2002
            if (myTouchFlag)
            {
                if (after == TopAbs_OUT)
                {
                    after = TopAbs_IN;
                }
                else if (after == TopAbs_IN)
                {
                    after = TopAbs_OUT;
                }
            }
        }

        return after;
    }

    /// <summary>The state before a transition of orientation <paramref name="Tran"/>.</summary>
    public static TopAbs_State GetBefore(TopAbs_Orientation Tran)
    {
        switch (Tran)
        {
            case TopAbs_FORWARD:
            case TopAbs_EXTERNAL:
                return TopAbs_OUT;
            case TopAbs_REVERSED:
            case TopAbs_INTERNAL:
                return TopAbs_IN;
        }

        return TopAbs_OUT;
    }

    /// <summary>The state after a transition of orientation <paramref name="Tran"/>.</summary>
    public static TopAbs_State GetAfter(TopAbs_Orientation Tran)
    {
        switch (Tran)
        {
            case TopAbs_FORWARD:
            case TopAbs_INTERNAL:
                return TopAbs_IN;
            case TopAbs_REVERSED:
            case TopAbs_EXTERNAL:
                return TopAbs_OUT;
        }

        return TopAbs_OUT;
    }

    // File-static functions of TopTrans_SurfaceTransition.cxx

    private static bool M_REVERSED(TopAbs_Orientation st) => st == TopAbs_REVERSED;

    private static bool M_INTERNAL(TopAbs_Orientation st) => st == TopAbs_INTERNAL;

    private static bool M_UNKNOWN(TopAbs_State st) => st == TopAbs_UNKNOWN;

    // Modified: eap Mar 25 2002 (occ102,occ227), touch case
    private static Direction3D FUN_nCinsideS(in Direction3D tgC, in Direction3D ngS)
    {
        // Give us a curve C on surface S, <parOnC>, a parameter
        // Purpose : compute normal vector to C, tangent to S at
        //           given point , oriented INSIDE S
        // <tgC> : geometric tangent at point of <parOnC>
        // <ngS> : geometric normal at point of <parOnC>
        Direction3D XX = ngS ^ tgC;
        return XX;
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

    private static double FUN_Ang(in Direction3D _, in Direction3D beafter, in Direction3D TgC, in Direction3D Norm, TopAbs_Orientation O)
    {
        Direction3D dironF = FUN_nCinsideS(TgC, Norm);
        if (M_REVERSED(O))
        {
            dironF.Reverse();
        }

        double ang = beafter.AngleWithRef(dironF, TgC);
        return ang;
    }

    private static void FUN_getSTA(double Ang, double tola, out int i, out int j)
    {
        double cos = Math.Cos(Ang);
        double sin = Math.Sin(Ang);
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

    private static int FUN_refnearest(double Angref, TopAbs_Orientation Oriref, double Ang, TopAbs_Orientation Ori,
                                      double tola)
    {
        bool undef = Angref == 100.0;
        if (undef)
        {
            return M_updateREF;
        }

        double cosref = Math.Cos(Angref), cos = Math.Cos(Ang);
        double dcos = Math.Abs(cosref) - Math.Abs(cos);
        if (Math.Abs(dcos) < tola)
        {
            // Analysis for tangent cases : if two boundary faces are same sided
            // and have tangent normals, if they have opposite orientations
            // we choose INTERNAL as resulting complex transition (case EXTERNAL
            // referring to no logical case)
            if (TopAbs.Complement(Ori) == Oriref)
            {
                return M_Ointernal;
            }
            else
            {
                return M_Unknown; // nyi FUN_RAISE
            }
        }

        int updateref = (dcos > 0.0) ? M_noupdate : M_updateREF;
        return updateref;
    }

    private static int FUN_refnearest(int i, int j, double CurvSref, double Angref, TopAbs_Orientation Oriref,
                                      double Curvref, double Ang, TopAbs_Orientation Ori, double Curv, double tola,
                                      ref bool TouchFlag) // eap Mar 25 2002
    {
        bool iisj = i == j;
        double abscos = Math.Abs(Math.Cos(Ang));
        bool i0 = Math.Abs(1.0 - abscos) < tola;
        bool j0 = abscos < tola;
        bool nullcurv = Curv == 0.0;
        bool curvpos = Curv > tola;
        bool curvneg = Curv < -tola;
        bool nullcsref = CurvSref == 0.0;

        bool undef = Angref == 100.0;
        if (undef)
        {
            if (i0)
            {
                if (iisj && curvneg)
                {
                    return M_noupdate;
                }

                if (!iisj && curvpos)
                {
                    return M_noupdate;
                }
            }

            if (j0)
            {
                if (!nullcsref && (j == 1) && iisj && (curvpos || nullcurv))
                {
                    return M_updateREF;
                }

                if (!nullcsref && (j == 1) && !iisj && (curvneg || nullcurv))
                {
                    return M_updateREF;
                }

                if (iisj && curvpos)
                {
                    return M_noupdate;
                }

                if (!iisj && curvneg)
                {
                    return M_noupdate;
                }
            }

            return M_updateREF;
        } // undef

        double cosref = Math.Cos(Angref), cos = Math.Cos(Ang);
        double dcos = Math.Abs(cosref) - Math.Abs(cos);
        bool samecos = Math.Abs(dcos) < tola;
        if (samecos)
        {
            // Analysis for tangent cases : if two boundary faces are same sided
            // and have sma dironF.
            if (Math.Abs(Curvref - Curv) < 1.0e-4)
            {
                if (TopAbs.Complement(Ori) == Oriref)
                {
                    return M_Ointernal;
                }
                else
                {
                    return M_Unknown; // nyi FUN_RAISE
                }
            }

            bool noupdate = false;
            if (iisj && (Curvref > Curv))
            {
                noupdate = true;
            }

            if (!iisj && (Curvref < Curv))
            {
                noupdate = true;
            }

            int updateref1 = noupdate ? M_noupdate : M_updateREF;
            if (!j0)
            {
                return updateref1;
            }

            if (!noupdate && !nullcsref)
            {
                // check for (j==1) the face is ABOVE Sref
                // check for (j==2) the face is BELOW Sref
                if ((j == 2) && (Math.Abs(Curv) < CurvSref))
                {
                    updateref1 = M_noupdate;
                }

                if ((j == 1) && (Math.Abs(Curv) > CurvSref))
                {
                    updateref1 = M_noupdate;
                }
            }

            return updateref1;
        } // samecos

        int updateref = (dcos > 0.0) ? M_noupdate : M_updateREF;
        if (Oriref != Ori)
        {
            TouchFlag = true; // eap Mar 25 2002
        }

        return updateref;
    }

    private static TopAbs_State FUN_getstate(NCollection_Array2<double> Ang, NCollection_Array2<TopAbs_Orientation> Ori,
                                             int iSTA, int iINDEX)
    {
        double a1 = Ang[iSTA, 1], a2 = Ang[iSTA, 2];
        bool undef1 = a1 == 100.0, undef2 = a2 == 100.0;
        bool undef = undef1 && undef2;
        if (undef)
        {
            return TopAbs_UNKNOWN;
        }

        if (undef1 || undef2)
        {
            int jok = undef1 ? 2 : 1;
            TopAbs_Orientation o = Ori[iSTA, jok];
            TopAbs_State st = (iINDEX == BEFORE) ? GetBefore(o) : GetAfter(o);
            return st;
        }

        TopAbs_Orientation o1 = Ori[iSTA, 1], o2 = Ori[iSTA, 2];
        TopAbs_State st1 = (iINDEX == BEFORE) ? GetBefore(o1) : GetAfter(o1);
        TopAbs_State st2 = (iINDEX == BEFORE) ? GetBefore(o2) : GetAfter(o2);
        if (st1 != st2)
        {
            return TopAbs_UNKNOWN; // Incoherent data
        }

        return st1;
    }
}
