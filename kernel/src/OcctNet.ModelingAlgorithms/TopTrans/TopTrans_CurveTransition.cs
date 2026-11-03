// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_CurveTransition.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopTrans/TopTrans_CurveTransition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

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
public class TopTrans_CurveTransition
{
    private const int GREATER = 1;
    private const int SAME = 0;
    private const int LOWER = -1;

    private Direction3D myTgt = new();
    private Direction3D myNorm = new();
    private double myCurv;
    private bool Init;
    private Direction3D TgtFirst = new();
    private Direction3D NormFirst = new();
    private double CurvFirst;
    private Orientation TranFirst;
    private Direction3D TgtLast = new();
    private Direction3D NormLast = new();
    private double CurvLast;
    private Orientation TranLast;

    /// <summary>Creates an empty curve transition.</summary>
    public TopTrans_CurveTransition()
    {
        myCurv = 0.0;
        Init = false;
        CurvFirst = 0.0;
        CurvLast = 0.0;
        TranFirst = Forward;
        TranLast = Forward;
    }

    /// <summary>Initializes the transition with the local description of a curve.</summary>
    public void Reset(in Direction3D Tgt, in Direction3D Norm, double Curv)
    {
        myTgt = Tgt;
        myNorm = Norm;
        myCurv = Curv;
        Init = true;
    }

    /// <summary>Initializes the transition with the local description of a straight line.</summary>
    public void Reset(in Direction3D Tgt)
    {
        myTgt = Tgt;
        myCurv = 0.0;
        Init = true;
    }

    /// <summary>
    /// Adds a curve element to the boundary. If <paramref name="Or"/> is REVERSED the curve is before the
    /// intersection, if FORWARD after it, and if INTERNAL the intersection is in the middle of the curve.
    /// </summary>
    public void Compare(double Tole, in Direction3D T, in Direction3D N, double C, Orientation St, Orientation Or)
    {
        // S is the transition, how the curve cross the boundary
        // O is the orientation, how the intersection is set on the boundary
        Orientation S = St;
        Orientation O = Or;

        // adjustment for INTERNAL transition
        if (S == Internal)
        {
            if (T * myTgt < 0)
            {
                S = global::Kerf.Engine.Topology.TopAbs.Reverse(O);
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
                case Reversed:
                    TgtFirst.Reverse();
                    TgtLast.Reverse();
                    break;
                case Internal:
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
                case Forward:
                case External:
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

            double cosAngWith1 = myTgt * TgtFirst;

            switch (Compare(cosAngWithT, cosAngWith1, Tole))
            {
                case LOWER:
                    // If the angle is greater than the first the new become the first
                    FirstSet = true;
                    TgtFirst = T;
                    switch (O)
                    {
                        case Reversed:
                            TgtFirst.Reverse();
                            break;
                        case Internal:
                            if (myTgt * T > 0)
                            {
                                TgtFirst.Reverse();
                            }

                            break;
                        case Forward:
                        case External:
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
                            case Reversed:
                                TgtFirst.Reverse();
                                break;
                            case Internal:
                                if (myTgt * T > 0)
                                {
                                    TgtFirst.Reverse();
                                }

                                break;
                            case Forward:
                            case External:
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

            if (!FirstSet || O == Internal)
            {
                // In the tangent cases the first can also be the last
                if (O == Internal)
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
                            case Reversed:
                                TgtLast.Reverse();
                                break;
                            case Internal:
                                if (myTgt * T < 0)
                                {
                                    TgtLast.Reverse();
                                }

                                break;
                            case Forward:
                            case External:
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
                                case Reversed:
                                    TgtLast.Reverse();
                                    break;
                                case Internal:
                                    if (myTgt * T < 0)
                                    {
                                        TgtLast.Reverse();
                                    }

                                    break;
                                case Forward:
                                case External:
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
    public Containment StateBefore()
    {
        if (Init)
        {
            return Unknown;
        }

        switch (TranFirst)
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
        if (Init)
        {
            return Unknown;
        }

        switch (TranLast)
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
    private bool IsBefore(double Tole, double CosAngl, in Direction3D N1, double C1, in Direction3D N2, double C2)
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
