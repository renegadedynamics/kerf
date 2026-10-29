// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_CurveTransition.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_CurveTransition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Computes the transition of a curve intersecting a curvilinear boundary.
/// </summary>
/// <remarks>
/// The geometric elements are described locally at the intersection point by a second order development: the
/// curve by its tangent, normal and curvature; the boundary by a set of curve elements, each a curve or a
/// half-curve (a curve and an orientation saying whether the boundary is before or after the intersection).
/// A working object (decision 21): <see cref="Reset(in gp_Dir, in gp_Dir, double)"/> and
/// <see cref="Compare(double, in gp_Dir, in gp_Dir, double, TopAbs_Orientation, TopAbs_Orientation)"/> are public,
/// as in OCCT. A class: OCCT holds it by value as a member of the classifiers, which pass it by reference.
/// </remarks>
public class TopTrans_CurveTransition
{
    private const int GREATER = 1;
    private const int SAME = 0;
    private const int LOWER = -1;

    private gp_Dir myTgt = new();
    private gp_Dir myNorm = new();
    private double myCurv;
    private bool Init;
    private gp_Dir TgtFirst = new();
    private gp_Dir NormFirst = new();
    private double CurvFirst;
    private TopAbs_Orientation TranFirst;
    private gp_Dir TgtLast = new();
    private gp_Dir NormLast = new();
    private double CurvLast;
    private TopAbs_Orientation TranLast;

    /// <summary>Creates an empty curve transition.</summary>
    public TopTrans_CurveTransition()
    {
        myCurv = 0.0;
        Init = false;
        CurvFirst = 0.0;
        CurvLast = 0.0;
        TranFirst = TopAbs_FORWARD;
        TranLast = TopAbs_FORWARD;
    }

    /// <summary>Initializes the transition with the local description of a curve.</summary>
    public void Reset(in gp_Dir Tgt, in gp_Dir Norm, double Curv)
    {
        myTgt = Tgt;
        myNorm = Norm;
        myCurv = Curv;
        Init = true;
    }

    /// <summary>Initializes the transition with the local description of a straight line.</summary>
    public void Reset(in gp_Dir Tgt)
    {
        myTgt = Tgt;
        myCurv = 0.0;
        Init = true;
    }

    /// <summary>
    /// Adds a curve element to the boundary. If <paramref name="Or"/> is REVERSED the curve is before the
    /// intersection, if FORWARD after it, and if INTERNAL the intersection is in the middle of the curve.
    /// </summary>
    public void Compare(double Tole, in gp_Dir T, in gp_Dir N, double C, TopAbs_Orientation St, TopAbs_Orientation Or)
    {
        // S is the transition, how the curve cross the boundary
        // O is the orientation, how the intersection is set on the boundary
        TopAbs_Orientation S = St;
        TopAbs_Orientation O = Or;

        // adjustment for INTERNAL transition
        if (S == TopAbs_INTERNAL)
        {
            if (T * myTgt < 0)
            {
                S = TopAbs.Reverse(O);
            }
            else
            {
                S = O;
            }
        }

        // It is the first comparison for this complex transition
        if (Init)
        {
            Init = false;
            TgtFirst = T;
            NormFirst = N;
            CurvFirst = C;
            TranFirst = S;
            TgtLast = T;
            NormLast = N;
            CurvLast = C;
            TranLast = S;
            switch (O)
            {
                // Interference at the end of the edge: the tangent must be reversed
                case TopAbs_REVERSED:
                    TgtFirst.Reverse();
                    TgtLast.Reverse();
                    break;
                case TopAbs_INTERNAL:
                    // Interference in the middle of the edge: reverse depending on the position of the
                    // reference tangent
                    if (myTgt * T > 0)
                    {
                        TgtFirst.Reverse();
                    }
                    else
                    {
                        TgtLast.Reverse();
                    }

                    break;
                case TopAbs_FORWARD:
                case TopAbs_EXTERNAL:
                    break;
            }
        }
        // Compare with the existent first and last transition :
        else
        {
            bool FirstSet = false;
            double cosAngWithT = myTgt * T;
            switch (O)
            {
                case TopAbs_REVERSED:
                    cosAngWithT = -cosAngWithT;
                    break;
                case TopAbs_INTERNAL:
                    if (cosAngWithT > 0)
                    {
                        cosAngWithT = -cosAngWithT;
                    }

                    break;
                case TopAbs_FORWARD:
                case TopAbs_EXTERNAL:
                    break;
            }

            double cosAngWith1 = myTgt * TgtFirst;

            switch (Compare(cosAngWithT, cosAngWith1, Tole))
            {
                case LOWER:
                    // If the angle is greater than the first the new become the first
                    FirstSet = true;
                    TgtFirst = T;
                    switch (O)
                    {
                        case TopAbs_REVERSED:
                            TgtFirst.Reverse();
                            break;
                        case TopAbs_INTERNAL:
                            if (myTgt * T > 0)
                            {
                                TgtFirst.Reverse();
                            }

                            break;
                        case TopAbs_FORWARD:
                        case TopAbs_EXTERNAL:
                            break;
                    }

                    NormFirst = N;
                    CurvFirst = C;
                    TranFirst = S;
                    break;

                case SAME:
                    // If same angles we look at the Curvature
                    if (IsBefore(Tole, cosAngWithT, N, C, NormFirst, CurvFirst))
                    {
                        FirstSet = true;
                        TgtFirst = T;
                        switch (O)
                        {
                            case TopAbs_REVERSED:
                                TgtFirst.Reverse();
                                break;
                            case TopAbs_INTERNAL:
                                if (myTgt * T > 0)
                                {
                                    TgtFirst.Reverse();
                                }

                                break;
                            case TopAbs_FORWARD:
                            case TopAbs_EXTERNAL:
                                break;
                        }

                        NormFirst = N;
                        CurvFirst = C;
                        TranFirst = S;
                    }

                    break;

                case GREATER:
                    break;
            }

            if (!FirstSet || O == TopAbs_INTERNAL)
            {
                // In the tangent cases the first can also be the last
                if (O == TopAbs_INTERNAL)
                {
                    cosAngWithT = -cosAngWithT;
                }

                double cosAngWith2 = myTgt * TgtLast;

                switch (Compare(cosAngWithT, cosAngWith2, Tole))
                {
                    case GREATER:
                        // If the angle is lower than the last the new become the last
                        TgtLast = T;
                        switch (O)
                        {
                            case TopAbs_REVERSED:
                                TgtLast.Reverse();
                                break;
                            case TopAbs_INTERNAL:
                                if (myTgt * T < 0)
                                {
                                    TgtLast.Reverse();
                                }

                                break;
                            case TopAbs_FORWARD:
                            case TopAbs_EXTERNAL:
                                break;
                        }

                        NormLast = N;
                        CurvLast = C;
                        TranLast = S;
                        break;

                    case SAME:
                        // If the angle is the same we look at the curvature
                        if (IsBefore(Tole, cosAngWithT, NormLast, CurvLast, N, C))
                        {
                            TgtLast = T;
                            switch (O)
                            {
                                case TopAbs_REVERSED:
                                    TgtLast.Reverse();
                                    break;
                                case TopAbs_INTERNAL:
                                    if (myTgt * T < 0)
                                    {
                                        TgtLast.Reverse();
                                    }

                                    break;
                                case TopAbs_FORWARD:
                                case TopAbs_EXTERNAL:
                                    break;
                            }

                            NormLast = N;
                            CurvLast = C;
                            TranLast = S;
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
    public TopAbs_State StateBefore()
    {
        if (Init)
        {
            return TopAbs_UNKNOWN;
        }

        switch (TranFirst)
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

    /// <summary>
    /// The state of the curve after the intersection: the position relative to the boundary of a point very
    /// close to the intersection on the positive side of the tangent.
    /// </summary>
    public TopAbs_State StateAfter()
    {
        if (Init)
        {
            return TopAbs_UNKNOWN;
        }

        switch (TranLast)
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

    /// <summary>Compares the curvature of two transitions: true if (N1, C1) is before (N2, C2).</summary>
    private bool IsBefore(double Tole, double CosAngl, in gp_Dir N1, double C1, in gp_Dir N2, double C2)
    {
        double TN1 = myTgt * N1;
        double TN2 = myTgt * N2;
        bool OneBefore = false;

        if (Math.Abs(TN1) <= Tole || Math.Abs(TN2) <= Tole)
        {
            // Tangent : The first is the interference which have the nearest curvature
            //           from the reference.
            if (myCurv == 0)
            {
                // The reference is straight
                // The first is the interference which have the lowest curvature.
                if (C1 < C2)
                {
                    OneBefore = true;
                }

                //  Modified by Sergey KHROMOV - Wed Dec 27 17:08:49 2000
                if (CosAngl > 0)
                {
                    OneBefore = !OneBefore;
                }
            }
            else
            {
                // The reference is curv
                // The first is the interference which have the nearest curvature
                // in the direction
                double deltaC1, deltaC2;
                if (C1 == 0.0 || myCurv == 0.0)
                {
                    deltaC1 = C1 - myCurv;
                }
                else
                {
                    deltaC1 = (C1 - myCurv) * (N1 * myNorm);
                }

                if (C2 == 0.0 || myCurv == 0.0)
                {
                    deltaC2 = C2 - myCurv;
                }
                else
                {
                    deltaC2 = (C2 - myCurv) * (N2 * myNorm);
                }

                if (deltaC1 < deltaC2)
                {
                    OneBefore = true;
                }

                if (CosAngl > 0)
                {
                    OneBefore = !OneBefore;
                }
            }
        }
        else if (TN1 < 0)
        {
            // Before the first interference we are in the curvature
            if (TN2 > 0)
            {
                // Before the second  interference we are out the curvature
                // The first interference is before  /* ->)( */
                OneBefore = true;
            }
            else
            {
                // Before the second interference we are in the curvature
                if (C1 > C2)
                {
                    // We choice the greater curvature
                    // The first interference is before   /* ->)) */
                    OneBefore = true;
                }
            }
        }
        else if (TN1 > 0)
        {
            // Before the first interference we are out the curvature
            if (TN2 > 0)
            {
                // Before the second interference we are out the curvature /* ->(( */
                if (C1 < C2)
                {
                    // We choice the lower curvature
                    // The first interference is before
                    OneBefore = true;
                }
            }
        }

        return OneBefore;
    }

    /// <summary>Compares two angles at tolerance Tole.</summary>
    private int Compare(double Ang1, double Ang2, double Tole)
    {
        int res = SAME;
        if (Ang1 - Ang2 > Tole)
        {
            res = GREATER;
        }
        else if (Ang2 - Ang1 > Tole)
        {
            res = LOWER;
        }

        return res;
    }
}
