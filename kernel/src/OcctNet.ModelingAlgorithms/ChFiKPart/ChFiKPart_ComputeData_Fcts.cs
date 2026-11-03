// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Fcts.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Fcts.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>The analytic fillets' shared functions (<c>ChFiKPart_*</c> of <c>ChFiKPart_ComputeData_Fcts</c>).</summary>
/// <remarks>
/// <c>ChFiKPart_RstMap</c> (OCCT 8, deprecated) is only a typedef of <c>NCollection_DataMap&lt;int, handle&lt;
/// Adaptor2d_Curve2d&gt;&gt;</c>: it is written out where used.
/// </remarks>
internal static class ChFiKPart_ComputeData_Fcts
{
    //=================================================================================================

    /// <summary><paramref name="U"/> brought into [<paramref name="UFirst"/>, <paramref name="ULast"/>], up to
    /// <paramref name="Eps"/>.</summary>
    internal static double ChFiKPart_InPeriod(double U, double UFirst, double ULast, double Eps)
    {
        double u = U, period = ULast - UFirst;
        while (Eps < (UFirst - u))
        {
            u += period;
        }
        while (Eps > (ULast - u))
        {
            u -= period;
        }
        if (u < UFirst)
        {
            u = UFirst;
        }
        return u;
    }

    //=======================================================================
    // function : PCurve
    // purpose  : Calculate a straight line in form of BSpline to guarantee
    //           the parameters.
    //=======================================================================

    /// <summary>The segment from <paramref name="UV1"/> to <paramref name="UV2"/> as a degree 1 B-spline over
    /// [<paramref name="Pardeb"/>, <paramref name="Parfin"/>].</summary>
    internal static BSplineCurve2D ChFiKPart_PCurve(in Point2D UV1, in Point2D UV2, double Pardeb,
                                                         double Parfin)
    {
        NCollection_Array1<Point2D> p = new(1, 2);
        NCollection_Array1<double> k = new(1, 2);
        NCollection_Array1<int> m = new(1, 2);
        m.Init(2);
        k[1] = Pardeb;
        k[2] = Parfin;
        p[1] = UV1;
        p[2] = UV2;
        BSplineCurve2D Pcurv = new(p, k, m, 1);
        return Pcurv;
    }

    //=======================================================================
    // function : ProjPC
    // purpose  : For spherical corners the contours which of are not
    //           isos the circle is projected.
    //=======================================================================

    /// <summary>The projection of <paramref name="Cg"/> on <paramref name="Sg"/> as a 2D curve.</summary>
    /// <exception cref="MethodNotImplementedException">The projection is not a line, a Bézier or a B-spline, or the
    /// surface is not elementary.</exception>
    internal static void ChFiKPart_ProjPC(CurveAdaptor Cg, SurfaceAdaptor Sg, out Curve2D? Pcurv)
    {
        Pcurv = null;
        if (Sg.GetType() < SurfaceKind.BezierSurface)
        {
            CurveAdaptor HCg = (CurveAdaptor)Cg.ShallowCopy(); // OCCT copy-constructs the adaptors
            SurfaceAdaptor HSg = (SurfaceAdaptor)Sg.ShallowCopy();
            ProjectedCurve Projc = new(HSg, HCg);
            switch (Projc.GetType())
            {
                case Line:
                {
                    Pcurv = new LineCurve2D(Projc.Line());
                }
                    break;
                case BezierCurve:
                {
                    BezierCurve2D BezProjc = Projc.Bezier();
                    NCollection_Array1<Point2D> TP = BezProjc.InternalPoles();
                    if (BezProjc.IsRational)
                    {
                        Pcurv = new BezierCurve2D(TP, BezProjc.InternalWeights()!);
                    }
                    else
                    {
                        Pcurv = new BezierCurve2D(TP);
                    }
                }
                    break;
                case BSplineCurve:
                {
                    BSplineCurve2D BspProjc = Projc.BSpline();
                    NCollection_Array1<Point2D> TP = BspProjc.InternalPoles();
                    NCollection_Array1<double> TK = BspProjc.InternalKnots();
                    NCollection_Array1<int> TM = BspProjc.InternalMultiplicities();

                    if (BspProjc.IsRational)
                    {
                        Pcurv = new BSplineCurve2D(TP, BspProjc.InternalWeightsArray(), TK, TM,
                                                        BspProjc.Degree());
                    }
                    else
                    {
                        Pcurv = new BSplineCurve2D(TP, TK, TM, BspProjc.Degree());
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

    /// <summary>Adds <paramref name="C"/> to the data structure with a null tolerance; returns its index.</summary>
    internal static int ChFiKPart_IndexCurveInDS(Curve3D? C, TopOpeBRepDS_DataStructure DStr)
    {
        return DStr.AddCurve(new TopOpeBRepDS_Curve(C, 0.0));
    }

    //=======================================================================
    // function : IndexSurfaceInDS
    // purpose  : Place a Surface in the DS and return its index.
    //=======================================================================

    /// <summary>Adds <paramref name="S"/> to the data structure with a null tolerance; returns its index.</summary>
    internal static int ChFiKPart_IndexSurfaceInDS(Surface? S, TopOpeBRepDS_DataStructure DStr)
    {
        return DStr.AddSurface(new TopOpeBRepDS_Surface(S, 0.0));
    }
}
