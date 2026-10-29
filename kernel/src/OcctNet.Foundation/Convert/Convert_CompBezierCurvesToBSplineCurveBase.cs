// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompBezierCurvesToBSplineCurveBase.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The operations <see cref="Convert_CompBezierCurvesToBSplineCurveBase{PointType, VecType, Traits}"/> needs on
/// its point and vector types, which OCCT's template resolves by overload at instantiation.
/// </summary>
/// <remarks>
/// Public only because the base class is public and generic over it (C# forbids a public class deriving from a
/// base built on an internal type); implemented by <see cref="Convert_CompBezierCurvesTraits3d"/> and
/// <see cref="Convert_CompBezierCurvesTraits2d"/>.
/// </remarks>
public interface IConvert_CompBezierCurvesTraits<PointType, VecType>
    where PointType : struct
    where VecType : struct
{
    /// <summary>True for gp_Pnt (OCCT's <c>std::is_same_v&lt;PointType, gp_Pnt&gt;</c>).</summary>
    static abstract bool Is3d { get; }

    /// <summary><c>VecType(theP1, theP2)</c>.</summary>
    static abstract VecType Vec(in PointType theP1, in PointType theP2);

    static abstract double SquareMagnitude(in VecType theV);

    static abstract bool IsParallel(in VecType theV1, in VecType theV2, double theAngularTolerance);

    /// <summary><c>BSplCLib::IncreaseDegree(theNewDegree, thePoles, NoWeights(), theNewPoles, NoWeights())</c>.</summary>
    static abstract void IncreaseDegree(int theNewDegree, NCollection_Array1<PointType> thePoles,
                                        NCollection_Array1<PointType> theNewPoles);
}

/// <summary>The traits of <see cref="Convert_CompBezierCurvesToBSplineCurve"/>: gp_Pnt and gp_Vec.</summary>
public readonly struct Convert_CompBezierCurvesTraits3d : IConvert_CompBezierCurvesTraits<gp_Pnt, gp_Vec>
{
    public static bool Is3d => true;

    public static gp_Vec Vec(in gp_Pnt theP1, in gp_Pnt theP2) => new(theP1, theP2);

    public static double SquareMagnitude(in gp_Vec theV) => theV.SquareMagnitude();

    public static bool IsParallel(in gp_Vec theV1, in gp_Vec theV2, double theAngularTolerance)
        => theV1.IsParallel(theV2, theAngularTolerance);

    public static void IncreaseDegree(int theNewDegree, NCollection_Array1<gp_Pnt> thePoles,
                                      NCollection_Array1<gp_Pnt> theNewPoles)
        => BSplCLib.IncreaseDegree(theNewDegree, thePoles, BSplCLib.NoWeights(), theNewPoles, BSplCLib.NoWeights());
}

/// <summary>The traits of <see cref="Convert_CompBezierCurves2dToBSplineCurve2d"/>: gp_Pnt2d and gp_Vec2d.</summary>
public readonly struct Convert_CompBezierCurvesTraits2d : IConvert_CompBezierCurvesTraits<gp_Pnt2d, gp_Vec2d>
{
    public static bool Is3d => false;

    public static gp_Vec2d Vec(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => new(theP1, theP2);

    public static double SquareMagnitude(in gp_Vec2d theV) => theV.SquareMagnitude();

    public static bool IsParallel(in gp_Vec2d theV1, in gp_Vec2d theV2, double theAngularTolerance)
        => theV1.IsParallel(theV2, theAngularTolerance);

    public static void IncreaseDegree(int theNewDegree, NCollection_Array1<gp_Pnt2d> thePoles,
                                      NCollection_Array1<gp_Pnt2d> theNewPoles)
        => BSplCLib.IncreaseDegree(theNewDegree, thePoles, BSplCLib.NoWeights(), theNewPoles, BSplCLib.NoWeights());
}

/// <summary>
/// Converts a sequence of adjacent non-rational Bezier curves into a B-spline curve: <see cref="AddCurve"/> each
/// curve, then <see cref="Perform"/>. Where the tangents at a junction are parallel, the B-spline is made at least
/// C1 there.
/// </summary>
/// <remarks>
/// OCCT's template on (PointType, VecType), ported generically with a traits type (as BSplCLib_CurveComputation).
/// A builder: <see cref="AddCurve"/> and <see cref="Perform"/> change it.
/// </remarks>
public class Convert_CompBezierCurvesToBSplineCurveBase<PointType, VecType, Traits>
    where PointType : struct
    where VecType : struct
    where Traits : struct, IConvert_CompBezierCurvesTraits<PointType, VecType>
{
    private readonly NCollection_Sequence<NCollection_Array1<PointType>> mySequence = new();
    private readonly NCollection_Sequence<PointType> myCurvePoles = new();
    private readonly NCollection_Sequence<double> myCurveKnots = new();
    private readonly NCollection_Sequence<int> myKnotsMults = new();
    private int myDegree;
    private readonly double myAngular;

    /// <summary>
    /// A converter checking the parallelism of the tangents at the junctions with
    /// <paramref name="theAngularTolerance"/> (radians).
    /// </summary>
    public Convert_CompBezierCurvesToBSplineCurveBase(double theAngularTolerance = 1.0e-4)
    {
        myDegree = 0;
        myAngular = theAngularTolerance;
    }

    /// <summary>Adds the Bezier curve with the poles <paramref name="thePoles"/> (copied) to the sequence.</summary>
    public void AddCurve(NCollection_Array1<PointType> thePoles) { mySequence.Append(new NCollection_Array1<PointType>(thePoles)); }

    /// <summary>Computes the B-spline curve equivalent to the sequence of Bezier curves.</summary>
    public void Perform()
    {
        myCurvePoles.Clear();
        myCurveKnots.Clear();
        myKnotsMults.Clear();
        if (mySequence.IsEmpty())
        {
            return;
        }
        const int aLowerI = 1;
        int anUpperI = mySequence.Length();
        int aNbrCurv = anUpperI - aLowerI + 1;
        NCollection_Array1<double> aCurveKnVals = new(1, aNbrCurv);

        myDegree = 0;
        for (int i = 1; i <= mySequence.Length(); i++)
        {
            myDegree = Max(myDegree, mySequence[i].Length() - 1);
        }

        double aDet = 0;
        PointType aP1 = new(), aP2, aP3;
        int aMaxDegree = myDegree;
        NCollection_Array1<PointType> aPoints = new(1, myDegree + 1);

        for (int i = aLowerI; i <= anUpperI; i++)
        {
            // 1- Raise the Bezier curve to the maximum degree.
            int aDeg = mySequence[i].Length() - 1;
            int anInc = myDegree - aDeg;
            if (anInc > 0)
            {
                Traits.IncreaseDegree(myDegree, mySequence[i], aPoints);
            }
            else
            {
                aPoints.Assign(mySequence[i]);
            }

            // 2- Process the node of junction between 2 Bezier curves.
            if (i == aLowerI)
            {
                // Processing of the initial node of the BSpline.
                for (int j = 1; j <= aMaxDegree; j++)
                {
                    myCurvePoles.Append(aPoints[j]);
                }
                aCurveKnVals[1] = 1.0; // To begin the series.
                myKnotsMults.Append(aMaxDegree + 1);
                aDet = 1.0;
            }

            if (i != aLowerI)
            {
                aP2 = aPoints[1];
                aP3 = aPoints[2];
                VecType aV1 = Traits.Vec(aP1, aP2), aV2 = Traits.Vec(aP2, aP3);

                // Processing of the tangency between Bezier and the previous.
                // This allows to guarantee at least a C1 continuity if the tangents are coherent.
                double aD1 = Traits.SquareMagnitude(aV1);
                double aD2 = Traits.SquareMagnitude(aV2);
                if (aMaxDegree > 1 && aD1 > gp.Resolution() && aD2 > gp.Resolution()
                    && Traits.IsParallel(aV1, aV2, myAngular))
                {
                    double aLambda = Math.Sqrt(aD2 / aD1);
                    if (Traits.Is3d)
                    {
                        // 3D-specific epsilon guard to avoid numerical issues
                        // when accumulated knot values become too small relative to Det.
                        if (aCurveKnVals[i - 1] * aLambda > 10.0 * Epsilon(aDet))
                        {
                            myKnotsMults.Append(aMaxDegree - 1);
                            aCurveKnVals[i] = aCurveKnVals[i - 1] * aLambda;
                        }
                        else
                        {
                            myCurvePoles.Append(aPoints[1]);
                            myKnotsMults.Append(aMaxDegree);
                            aCurveKnVals[i] = 1.0;
                        }
                    }
                    else
                    {
                        myKnotsMults.Append(aMaxDegree - 1);
                        aCurveKnVals[i] = aCurveKnVals[i - 1] * aLambda;
                    }
                }
                else
                {
                    myCurvePoles.Append(aPoints[1]);
                    myKnotsMults.Append(aMaxDegree);
                    aCurveKnVals[i] = 1.0;
                }
                aDet += aCurveKnVals[i];

                // Store the poles.
                for (int j = 2; j <= aMaxDegree; j++)
                {
                    myCurvePoles.Append(aPoints[j]);
                }
            }

            if (i == anUpperI)
            {
                // Processing of the end node of the BSpline.
                myCurvePoles.Append(aPoints[aMaxDegree + 1]);
                myKnotsMults.Append(aMaxDegree + 1);
            }
            aP1 = aPoints[aMaxDegree];
        }

        // Correct nodal values to make them variable within [0.,1.].
        myCurveKnots.Append(0.0);
        for (int i = 2; i <= aNbrCurv; i++)
        {
            myCurveKnots.Append(myCurveKnots[i - 1] + (aCurveKnVals[i - 1] / aDet));
        }
        myCurveKnots.Append(1.0);
    }

    /// <summary>The degree of the B-spline curve.</summary>
    public int Degree() { return myDegree; }

    /// <summary>The number of poles of the B-spline curve.</summary>
    public int NbPoles() { return myCurvePoles.Length(); }

    /// <summary>Copies the poles of the B-spline curve into <paramref name="thePoles"/>.</summary>
    public void Poles(NCollection_Array1<PointType> thePoles)
    {
        int k = 1;
        for (int i = thePoles.Lower(); i <= thePoles.Upper(); i++)
        {
            thePoles[i] = myCurvePoles[k++];
        }
    }

    /// <summary>The number of knots of the B-spline curve.</summary>
    public int NbKnots() { return myCurveKnots.Length(); }

    /// <summary>Copies the knots and their multiplicities into <paramref name="theKnots"/> and <paramref name="theMults"/>.</summary>
    public void KnotsAndMults(NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults)
    {
        int k = 1;
        for (int i = theKnots.Lower(); i <= theKnots.Upper(); i++)
        {
            theKnots[i] = myCurveKnots[k++];
        }
        k = 1;
        for (int i = theMults.Lower(); i <= theMults.Upper(); i++)
        {
            theMults[i] = myKnotsMults[k++];
        }
    }
}
