// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_TorusRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_TorusRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Measurement;
using Kerf.Geometry;

namespace Occt;

/// <summary>Extends the UV parameter range splitter to generate the internal nodes of a toroidal surface.</summary>
public class BRepMesh_TorusRangeSplitter : BRepMesh_UVParamRangeSplitter
{
    /// <summary>A splitter.</summary>
    public BRepMesh_TorusRangeSplitter()
    {
    }

    /// <summary>
    /// The nodes of a grid over the face's range, built from the border points' parameters; null when the steps are
    /// degenerate.
    /// </summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in IMeshTools_Parameters theParameters)
    {
        (double first, double second) aRangeU = GetRangeU();
        (double first, double second) aRangeV = GetRangeV();

        double aDiffU = aRangeU.second - aRangeU.first;
        double aDiffV = aRangeV.second - aRangeV.first;

        Torus aTorus = GetDFace().GetSurface().Torus();
        double r = aTorus.MinorRadius();
        double R = aTorus.MajorRadius();

        double oldDv = TangentialDeflection.ArcAngularStep(r,
                                                                  GetDFace().GetDeflection(),
                                                                  theParameters.Angle,
                                                                  theParameters.MinSize);

        double Dv = 0.9 * oldDv; // TWOTHIRD * oldDv;
        Dv = oldDv;

        int nbV = Max((int)(aDiffV / Dv), 2);
        Dv = aDiffV / (nbV + 1);

        double Du;
        double ru = R + r;
        if (ru > 1.0e-16)
        {
            Du = TangentialDeflection.ArcAngularStep(ru,
                                                            GetDFace().GetDeflection(),
                                                            theParameters.Angle,
                                                            theParameters.MinSize);

            double aa = Math.Sqrt(Du * Du + oldDv * oldDv);
            if (aa < Constants.Resolution())
            {
                return null;
            }

            Du *= Min(oldDv, Du) / aa;
        }
        else
        {
            Du = Dv;
        }

        int nbU = Max((int)(aDiffU / Du), 2);
        nbU = Max(nbU, (int)(nbV * aDiffU * R / (aDiffV * r) / 5.0));
        Du = aDiffU / (nbU + 1);

        NCollection_Sequence<double> aParamU, aParamV;
        if (R < r)
        {
            // As the points of edges are returned.
            // in this case, the points are not representative.

            //-- Choose DeltaX and DeltaY so that to avoid skipping points on the grid
            aParamU = new NCollection_Sequence<double>();
            for (int i = 0; i <= nbU; i++)
            {
                aParamU.Append(aRangeU.first + i * Du);
            }
        } // R<r
        else // U if R > r
        {
            aParamU = fillParams(GetParametersU(), GetRangeU(), nbU, 0.5);
        }

        aParamV = fillParams(GetParametersV(), GetRangeV(), nbV, 2.0 / 3.0);

        (double first, double second) aNewRangeU = (aRangeU.first + Du * 0.1, aRangeU.second - Du * 0.1);

        (double first, double second) aNewRangeV = (aRangeV.first + Dv * 0.1, aRangeV.second - Dv * 0.1);

        NCollection_List<Point2D> aNodes = new();
        for (int i = 1; i <= aParamU.Length(); ++i)
        {
            double aPasU = aParamU.Value(i);
            if (aPasU >= aNewRangeU.first && aPasU < aNewRangeU.second)
            {
                for (int j = 1; j <= aParamV.Length(); ++j)
                {
                    double aPasV = aParamV.Value(j);
                    if (aPasV >= aNewRangeV.first && aPasV < aNewRangeV.second)
                    {
                        aNodes.Append(new Point2D(aPasU, aPasV));
                    }
                }
            }
        }

        return aNodes;
    }

    /// <summary>Registers a border point and its parameters.</summary>
    public override void AddPoint(in Point2D thePoint)
    {
        base.AddPoint(thePoint);
        GetParametersU().Add(thePoint.X());
        GetParametersV().Add(thePoint.Y());
    }

    // The parameters, sorted, without those closer than the scaled average step to one already taken.
    private NCollection_Sequence<double> fillParams(NCollection_IndexedMap<double> theParams,
                                                    (double first, double second) theRange,
                                                    int theStepsNb,
                                                    double theScale)
    {
        NCollection_Sequence<double> aParams = new();

        int aLength = theParams.Extent();
        NCollection_Array1<double> aParamArray = new(1, aLength);

        for (int j = 1; j <= aLength; ++j)
        {
            aParamArray[j] = theParams[j];
        }

        // Calculate DU, leave array of parameters
        double aDiff = Math.Abs(theRange.second - theRange.first);
        double aStep = FUN_CalcAverageDUV(aParamArray, aLength);
        aStep = Max(aStep, aDiff / (double)theStepsNb / 2.0);

        double aStdStep = aDiff / (double)aLength;
        if (aStep > aStdStep)
        {
            aStdStep = aStep;
        }
        aStdStep *= theScale;

        // Add parameters
        for (int j = 1; j <= aLength; ++j)
        {
            double pp = aParamArray[j];

            bool isToInsert = true;
            int aParamsLength = aParams.Length();
            for (int i = 1; i <= aParamsLength && isToInsert; ++i)
            {
                isToInsert = (Math.Abs(aParams.Value(i) - pp) > aStdStep);
            }

            if (isToInsert)
            {
                aParams.Append(pp);
            }
        }

        return aParams;
    }

    // Sorts the parameters in place (OCCT's exchange sort) and returns the average gap between neighbours above
    // 1e-7, or -1 if there is none.
    private static double FUN_CalcAverageDUV(NCollection_Array1<double> P, int PLen)
    {
        int i, j, n = 0;
        double p, result = 0.0;

        for (i = 1; i <= PLen; i++)
        {
            // Sort
            for (j = i + 1; j <= PLen; j++)
            {
                if (P[i] > P[j])
                {
                    p = P[i];
                    P[i] = P[j];
                    P[j] = p;
                }
            }
            // Accumulate
            if (i != 1)
            {
                p = Math.Abs(P[i] - P[i - 1]);
                if (p > 1.0e-7)
                {
                    result += p;
                    n++;
                }
            }
        }
        return (n != 0 ? (result / (double)n) : -1.0);
    }
}
