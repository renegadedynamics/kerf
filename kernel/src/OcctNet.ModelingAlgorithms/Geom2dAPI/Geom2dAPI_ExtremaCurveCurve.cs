// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ExtremaCurveCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ExtremaCurveCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ExtremaCurveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// All the extrema of the distance between two 2D curves: the pairs of points, one on each curve, at the ends of
/// the common perpendiculars.
/// </summary>
/// <remarks>
/// An algorithm object (decision 117). The nearest points of two curves may instead be an end of a curve and its
/// projection, two ends, or an intersection point. OCCT copies the Extrema_ExtCC2d it builds into its field; the
/// port keeps the object it builds (decision 224). OCCT's implicit conversions to int and double are explicit
/// operators (decision 327).
/// </remarks>
public class Geom2dAPI_ExtremaCurveCurve
{
    private readonly bool myIsDone;
    private readonly int myIndex;
    private readonly ExtCC2d myExtCC;
    private readonly CurveAdaptor2D myC1 = new();
    private readonly CurveAdaptor2D myC2 = new();

    /// <summary>
    /// The extrema between the part of <paramref name="C1"/> between <paramref name="U1min"/> and
    /// <paramref name="U1max"/> and the part of <paramref name="C2"/> between <paramref name="U2min"/> and
    /// <paramref name="U2max"/>.
    /// </summary>
    public Geom2dAPI_ExtremaCurveCurve(Curve2D C1, Curve2D C2, double U1min, double U1max, double U2min,
                                       double U2max)
    {
        myC1.Load(C1, U1min, U1max);
        myC2.Load(C2, U2min, U2max);
        ExtCC2d theExtCC = new(myC1, myC2);
        myExtCC = theExtCC;
        myIsDone = myExtCC.IsDone && (myExtCC.ExtCount > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
            double Dist2, Dist2Min = myExtCC.SquareDistance(1);
            myIndex = 1;

            for (int i = 2; i <= myExtCC.ExtCount; i++)
            {
                Dist2 = myExtCC.SquareDistance(i);
                if (Dist2 < Dist2Min)
                {
                    Dist2Min = Dist2;
                    myIndex = i;
                }
            }
        }
    }

    /// <summary>The number of extrema (0 if the computation failed).</summary>
    public int NbExtrema()
    {
        if (myIsDone)
        {
            return myExtCC.ExtCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The points on the first and the second curve of the extremum of index <paramref name="Index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="Index"/> is not in [1, NbExtrema].</exception>
    public void Points(int Index, out Point2D P1, out Point2D P2)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "Geom2dAPI_ExtremaCurveCurve::Points");
        myExtCC.Points(Index, out POnCurv2d PC1, out POnCurv2d PC2);

        P1 = PC1.Value();
        P2 = PC2.Value();
    }

    /// <summary>The parameters on the first and the second curve of the extremum of index <paramref name="Index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="Index"/> is not in [1, NbExtrema].</exception>
    public void Parameters(int Index, out double U1, out double U2)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "Geom2dAPI_ExtremaCurveCurve::Parameters");
        myExtCC.Points(Index, out POnCurv2d PC1, out POnCurv2d PC2);

        U1 = PC1.Parameter();
        U2 = PC2.Parameter();
    }

    /// <summary>The length of the extremum of index <paramref name="Index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="Index"/> is not in [1, NbExtrema].</exception>
    public double Distance(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "Geom2dAPI_ExtremaCurveCurve:Distance");
        return Math.Sqrt(myExtCC.SquareDistance(Index));
    }

    /// <summary>The points of the shortest extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public void NearestPoints(out Point2D P1, out Point2D P2)
    {
        StdFail_NotDone_Raise_if(!myIsDone, "Geom2dAPI_ExtremaCurveCurve:NearestPoints");
        Points(myIndex, out P1, out P2);
    }

    /// <summary>The parameters of the shortest extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public void LowerDistanceParameters(out double U1, out double U2)
    {
        StdFail_NotDone_Raise_if(!myIsDone, "Geom2dAPI_ExtremaCurveCurve:LowerDistanceParameters");
        Parameters(myIndex, out U1, out U2);
    }

    /// <summary>The length of the shortest extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "Geom2dAPI_ExtremaCurveCurve:LowerDistance");
        return Math.Sqrt(myExtCC.SquareDistance(myIndex));
    }

    /// <summary>The Extrema algorithm used.</summary>
    public ExtCC2d Extrema() => myExtCC;

    /// <summary>OCCT's <c>operator double()</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(Geom2dAPI_ExtremaCurveCurve theExtrema) => theExtrema.LowerDistance();

    /// <summary>OCCT's <c>operator int()</c>: the number of extrema of the Extrema algorithm.</summary>
    /// <exception cref="NotDoneException">The Extrema algorithm failed.</exception>
    public static explicit operator int(Geom2dAPI_ExtremaCurveCurve theExtrema) => theExtrema.myExtCC.ExtCount;
}
