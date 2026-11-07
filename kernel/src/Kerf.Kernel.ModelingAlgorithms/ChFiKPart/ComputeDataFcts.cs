// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Fcts.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Fcts.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Fillets.Analytic;

/// <summary>The analytic fillets' shared functions (<c>ChFiKPart_*</c> of <c>ChFiKPart_ComputeData_Fcts</c>).</summary>
/// <remarks>
/// <c>ChFiKPart_RstMap</c> (OCCT 8, deprecated) is only a typedef of <c>NCollection_DataMap&lt;int, handle&lt;
/// Adaptor2d_Curve2d&gt;&gt;</c>: it is written out where used.
/// </remarks>
internal static class ComputeDataFcts
{
    //=================================================================================================

    /// <summary><paramref name="U"/> brought into [<paramref name="uFirst"/>, <paramref name="uLast"/>], up to
    /// <paramref name="eps"/>.</summary>
    internal static double ChFiKPart_InPeriod(double U, double uFirst, double uLast, double eps)
    {
        double u = U, period = uLast - uFirst;
        for (int aStep = 0; eps < (uFirst - u); StepLimit.Next(ref aStep))
        {
            u += period;
        }
        for (int aStep = 0; eps > (uLast - u); StepLimit.Next(ref aStep))
        {
            u -= period;
        }
        if (u < uFirst)
        {
            u = uFirst;
        }
        return u;
    }

    //=======================================================================
    // function : PCurve
    // purpose  : Calculate a straight line in form of BSpline to guarantee
    //           the parameters.
    //=======================================================================

    /// <summary>The segment from <paramref name="uv1"/> to <paramref name="uv2"/> as a degree 1 B-spline over
    /// [<paramref name="pardeb"/>, <paramref name="parfin"/>].</summary>
    internal static BSplineCurve2D ChFiKPart_PCurve(in Point2D uv1, in Point2D uv2, double pardeb,
                                                         double parfin)
    {
        NCollection_Array1<Point2D> p = new(1, 2);
        NCollection_Array1<double> k = new(1, 2);
        NCollection_Array1<int> m = new(1, 2);
        m.Init(2);
        k[1] = pardeb;
        k[2] = parfin;
        p[1] = uv1;
        p[2] = uv2;
        BSplineCurve2D pcurv = new(p, k, m, 1);
        return pcurv;
    }

    //=======================================================================
    // function : ProjPC
    // purpose  : For spherical corners the contours which of are not
    //           isos the circle is projected.
    //=======================================================================

    /// <summary>The projection of <paramref name="cg"/> on <paramref name="sg"/> as a 2D curve.</summary>
    /// <exception cref="MethodNotImplementedException">The projection is not a line, a Bézier or a B-spline, or the
    /// surface is not elementary.</exception>
    internal static void ChFiKPart_ProjPC(CurveAdaptor cg, SurfaceAdaptor sg, out Curve2D? pcurv)
    {
        pcurv = null;
        if (sg.GetType() < SurfaceKind.BezierSurface)
        {
            CurveAdaptor hCg = (CurveAdaptor)cg.ShallowCopy(); // OCCT copy-constructs the adaptors
            SurfaceAdaptor hSg = (SurfaceAdaptor)sg.ShallowCopy();
            ProjectedCurve projc = new(hSg, hCg);
            switch (projc.GetType())
            {
                case Line:
                {
                    pcurv = new LineCurve2D(projc.Line());
                }
                    break;
                case BezierCurve:
                {
                    BezierCurve2D bezProjc = projc.Bezier();
                    NCollection_Array1<Point2D> tp = bezProjc.InternalPoles();
                    if (bezProjc.IsRational)
                    {
                        pcurv = new BezierCurve2D(tp, bezProjc.InternalWeights()!);
                    }
                    else
                    {
                        pcurv = new BezierCurve2D(tp);
                    }
                }
                    break;
                case BSplineCurve:
                {
                    BSplineCurve2D bspProjc = projc.BSpline();
                    NCollection_Array1<Point2D> tp = bspProjc.InternalPoles();
                    NCollection_Array1<double> tk = bspProjc.InternalKnots();
                    NCollection_Array1<int> tm = bspProjc.InternalMultiplicities();

                    if (bspProjc.IsRational)
                    {
                        pcurv = new BSplineCurve2D(tp, bspProjc.InternalWeightsArray(), tk, tm,
                                                        bspProjc.Degree);
                    }
                    else
                    {
                        pcurv = new BSplineCurve2D(tp, tk, tm, bspProjc.Degree);
                    }
                }
                    break;
                default:
                    throw new MethodNotImplementedException("failed approximation of the pcurve ");
            }
        }
        else
        {
            throw new MethodNotImplementedException("approximate pcurve on the left surface");
        }
    }

    //=======================================================================
    // function : IndexCurveInDS
    // purpose  : Place a Curve in the DS and return its index.
    //=======================================================================

    /// <summary>Adds <paramref name="c"/> to the data structure with a null tolerance; returns its index.</summary>
    internal static int ChFiKPart_IndexCurveInDS(Curve3D? c, DsDataStructure dStr)
    {
        return dStr.AddCurve(new DsCurve(c, 0.0));
    }

    //=======================================================================
    // function : IndexSurfaceInDS
    // purpose  : Place a Surface in the DS and return its index.
    //=======================================================================

    /// <summary>Adds <paramref name="s"/> to the data structure with a null tolerance; returns its index.</summary>
    internal static int ChFiKPart_IndexSurfaceInDS(Surface? s, DsDataStructure dStr)
    {
        return dStr.AddSurface(new DsSurface(s, 0.0));
    }
}
