// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_TorusRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_TorusRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;

using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>Extends the UV parameter range splitter to generate the internal nodes of a toroidal surface.</summary>
public class TorusRangeSplitter : UVParamRangeSplitter
{
    /// <summary>A splitter.</summary>
    public TorusRangeSplitter()
    {
    }

    /// <summary>
    /// The nodes of a grid over the face's range, built from the border points' parameters; null when the steps are
    /// degenerate.
    /// </summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in ToolParameters parameters)
    {
        (double first, double second) rangeU = GetRangeU();
        (double first, double second) rangeV = GetRangeV();

        double diffU = rangeU.second - rangeU.first;
        double diffV = rangeV.second - rangeV.first;

        Torus torus = GetDFace().GetSurface().Torus();
        double r = torus.MinorRadius();
        double R = torus.MajorRadius();

        double oldDv = TangentialDeflection.ArcAngularStep(r,
                                                                  GetDFace().GetDeflection(),
                                                                  parameters.Angle,
                                                                  parameters.MinSize);

        double dv = 0.9 * oldDv; // TWOTHIRD * oldDv;
        dv = oldDv;

        int nbV = Max((int)(diffV / dv), 2);
        dv = diffV / (nbV + 1);

        double du;
        double ru = R + r;
        if (ru > 1.0e-16)
        {
            du = TangentialDeflection.ArcAngularStep(ru,
                                                            GetDFace().GetDeflection(),
                                                            parameters.Angle,
                                                            parameters.MinSize);

            double aa = Math.Sqrt(du * du + oldDv * oldDv);
            if (aa < Constants.Resolution())
            {
                return null;
            }

            du *= Min(oldDv, du) / aa;
        }
        else
        {
            du = dv;
        }

        int nbU = Max((int)(diffU / du), 2);
        nbU = Max(nbU, (int)(nbV * diffU * R / (diffV * r) / 5.0));
        du = diffU / (nbU + 1);

        NCollection_Sequence<double> paramU, paramV;
        if (R < r)
        {
            // As the points of edges are returned.
            // in this case, the points are not representative.

            //-- Choose DeltaX and DeltaY so that to avoid skipping points on the grid
            paramU = new NCollection_Sequence<double>();
            for (int i = 0; i <= nbU; i++)
            {
                paramU.Append(rangeU.first + i * du);
            }
        } // R<r
        else // U if R > r
        {
            paramU = FillParams(GetParametersU(), GetRangeU(), nbU, 0.5);
        }

        paramV = FillParams(GetParametersV(), GetRangeV(), nbV, 2.0 / 3.0);

        (double first, double second) newRangeU = (rangeU.first + du * 0.1, rangeU.second - du * 0.1);

        (double first, double second) newRangeV = (rangeV.first + dv * 0.1, rangeV.second - dv * 0.1);

        NCollection_List<Point2D> nodes = new();
        for (int i = 1; i <= paramU.Length(); ++i)
        {
            double pasU = paramU.Value(i);
            if (pasU >= newRangeU.first && pasU < newRangeU.second)
            {
                for (int j = 1; j <= paramV.Length(); ++j)
                {
                    double pasV = paramV.Value(j);
                    if (pasV >= newRangeV.first && pasV < newRangeV.second)
                    {
                        nodes.Append(new Point2D(pasU, pasV));
                    }
                }
            }
        }

        return nodes;
    }

    /// <summary>Registers a border point and its parameters.</summary>
    public override void AddPoint(in Point2D point)
    {
        base.AddPoint(point);
        GetParametersU().Add(point.X());
        GetParametersV().Add(point.Y());
    }

    // The parameters, sorted, without those closer than the scaled average step to one already taken.
    private NCollection_Sequence<double> FillParams(IndexedSet<double> theParams,
                                                    (double first, double second) range,
                                                    int stepsNb,
                                                    double scale)
    {
        NCollection_Sequence<double> aParams = new();

        int length = theParams.Count;
        NCollection_Array1<double> paramArray = new(1, length);

        for (int j = 1; j <= length; ++j)
        {
            paramArray[j] = theParams[j - 1];
        }

        // Calculate DU, leave array of parameters
        double diff = Math.Abs(range.second - range.first);
        double step = FUN_CalcAverageDUV(paramArray, length);
        step = Max(step, diff / (double)stepsNb / 2.0);

        double stdStep = diff / (double)length;
        if (step > stdStep)
        {
            stdStep = step;
        }
        stdStep *= scale;

        // Add parameters
        for (int j = 1; j <= length; ++j)
        {
            double pp = paramArray[j];

            bool isToInsert = true;
            int paramsLength = aParams.Length();
            for (int i = 1; i <= paramsLength && isToInsert; ++i)
            {
                isToInsert = (Math.Abs(aParams.Value(i) - pp) > stdStep);
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
    private static double FUN_CalcAverageDUV(NCollection_Array1<double> P, int pLen)
    {
        int i, j, n = 0;
        double p, result = 0.0;

        for (i = 1; i <= pLen; i++)
        {
            // Sort
            for (j = i + 1; j <= pLen; j++)
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
