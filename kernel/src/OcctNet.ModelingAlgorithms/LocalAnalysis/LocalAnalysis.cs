// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.LocalAnalysis_StatusErrorType;

namespace Occt;

/// <summary>Writes the results of local continuity analyses.</summary>
public static class LocalAnalysis
{
    // OCCT's `o << value`: the writer's precision (6 by default), as printf's %g
    private static string R(TextWriter o, double theValue) => Standard_Stream.FormatReal(theValue, Standard_Stream.Precision(o));

    /// <summary>Writes the status and values of a surface continuity analysis.</summary>
    public static void Dump(LocalAnalysis_SurfaceContinuity surfconti, TextWriter o)
    {
        if (!surfconti.IsDone())
        {
            o.Write("Problem in the computation " + '\n');
            if (surfconti.StatusError() == LocalAnalysis_NullFirstDerivative)
            {
                o.Write("one of the first derivatives is null" + '\n');
            }
            else if (surfconti.StatusError() == LocalAnalysis_NullSecondDerivative)
            {
                o.Write("one of the  second derivatives is null" + '\n');
            }
            else if (surfconti.StatusError() == LocalAnalysis_NormalNotDefined)
            {
                o.Write("one (or both) normal is undefined" + '\n');
            }
            else if (surfconti.StatusError() == LocalAnalysis_CurvatureNotDefined)
            {
                o.Write("one of the mean curvatures is undefined" + '\n');
            }
        }
        else
        {
            switch (surfconti.ContinuityStatus())
            {
                case GeomAbs_C0:
                    if (surfconti.IsC0())
                    {
                        o.Write(" Continuity Status : C0 " + '\n');
                    }
                    else
                    {
                        o.Write(" Continuity Status : No C0 " + '\n');
                    }
                    o.Write(" C0Value = " + R(o, surfconti.C0Value()) + '\n');
                    break;
                case GeomAbs_C1:
                    if (surfconti.IsC1())
                    {
                        o.Write(" Continuity Status : C1 " + '\n');
                    }
                    else
                    {
                        if (surfconti.IsC0())
                        {
                            o.Write(" Continuity Status : C0 " + '\n');
                        }
                        else
                        {
                            o.Write(" Continuity Status : NoC0 " + '\n');
                        }
                    }
                    o.Write(" C0Value = " + R(o, surfconti.C0Value()) + '\n');
                    o.Write(" C1UAngle = " + R(o, surfconti.C1UAngle()) + '\n');
                    o.Write(" C1URatio = " + R(o, surfconti.C1URatio()) + '\n');
                    o.Write(" C1VAngle = " + R(o, surfconti.C1VAngle()) + '\n');
                    o.Write(" C1VRatio = " + R(o, surfconti.C1VRatio()) + '\n');
                    break;
                case GeomAbs_C2:
                    if (surfconti.IsC2())
                    {
                        o.Write(" Continuity Status : C2 " + '\n');
                    }
                    else
                    {
                        if (surfconti.IsC1())
                        {
                            o.Write(" Continuity Status : C1 " + '\n');
                        }
                        else
                        {
                            if (surfconti.IsC0())
                            {
                                o.Write(" Continuity Status : C0 " + '\n');
                            }
                            else
                            {
                                o.Write(" Continuity Status : NoC0 " + '\n');
                            }
                        }
                    }

                    o.Write(" C0Value = " + R(o, surfconti.C0Value()) + '\n');
                    o.Write(" C1UAngle = " + R(o, surfconti.C1UAngle()) + '\n');
                    o.Write(" C1VAngle = " + R(o, surfconti.C1VAngle()) + '\n');
                    o.Write(" C2UAngle = " + R(o, surfconti.C2UAngle()) + '\n');
                    o.Write(" C2VAngle = " + R(o, surfconti.C2VAngle()) + '\n');
                    o.Write(" C1URatio = " + R(o, surfconti.C1URatio()) + '\n');
                    o.Write(" C1VRatio = " + R(o, surfconti.C1VRatio()) + '\n');
                    o.Write(" C2URatio = " + R(o, surfconti.C2URatio()) + '\n');
                    o.Write(" C2VRatio = " + R(o, surfconti.C2VRatio()) + '\n');
                    break;
                case GeomAbs_G1:
                    if (surfconti.IsG1())
                    {
                        o.Write(" Continuity Status : G1 " + '\n');
                    }
                    else
                    {
                        if (surfconti.IsC0())
                        {
                            o.Write(" Continuity Status : G0 " + '\n');
                        }
                        else
                        {
                            o.Write(" Continuity Status : NoG0 " + '\n');
                        }
                    }
                    o.Write(" G0Value = " + R(o, surfconti.C0Value()) + '\n');
                    o.Write(" G1Angle = " + R(o, surfconti.G1Angle()) + '\n' + '\n');
                    break;
                case GeomAbs_G2:
                    if (surfconti.IsG2())
                    {
                        o.Write(" Continuity Status : G2 " + '\n');
                    }
                    else
                    {
                        if (surfconti.IsG1())
                        {
                            o.Write(" Continuity Status : G1 " + '\n');
                        }
                        else
                        {
                            if (surfconti.IsC0())
                            {
                                o.Write(" Continuity Status : G0 " + '\n');
                            }
                            else
                            {
                                o.Write(" Continuity Status : NoG0 " + '\n');
                            }
                        }
                    }
                    o.Write(" G0Value = " + R(o, surfconti.C0Value()) + '\n');
                    o.Write(" G1Value = " + R(o, surfconti.G1Angle()) + '\n');
                    o.Write(" G2CurvatureGap = " + R(o, surfconti.G2CurvatureGap()) + '\n');
                    break;

                default:
                    break;
            }
        }
    }

    /// <summary>Writes the status and values of a curve continuity analysis.</summary>
    public static void Dump(LocalAnalysis_CurveContinuity curvconti, TextWriter o)
    {
        if (!curvconti.IsDone())
        {
            o.Write("Problem in the computation " + '\n');
            if (curvconti.StatusError() == LocalAnalysis_NullFirstDerivative)
            {
                o.Write("one (or both) first derivative is null" + '\n');
            }
            else if (curvconti.StatusError() == LocalAnalysis_NullSecondDerivative)
            {
                o.Write("one (or both) second derivative is null" + '\n');
            }
            else if (curvconti.StatusError() == LocalAnalysis_TangentNotDefined)
            {
                o.Write("one (or both) tangent is undefined " + '\n');
            }
            else if (curvconti.StatusError() == LocalAnalysis_NormalNotDefined)
            {
                o.Write("one (or both) normal is undefined" + '\n');
            }
        }
        else
        {
            switch (curvconti.ContinuityStatus())
            {
                case GeomAbs_C0:
                    if (curvconti.IsC0())
                    {
                        o.Write(" Continuity Status : C0 " + '\n');
                    }
                    else
                    {
                        o.Write(" Continuity Status : No C0 " + '\n');
                    }
                    o.Write(" C0Value = " + R(o, curvconti.C0Value()) + '\n');
                    break;
                case GeomAbs_C1:
                    if (curvconti.IsC1())
                    {
                        o.Write(" Continuity Status : C1 " + '\n');
                    }
                    else
                    {
                        if (curvconti.IsC0())
                        {
                            o.Write(" Continuity Status : C0 " + '\n');
                        }
                        else
                        {
                            o.Write(" Continuity Status : NoC0 " + '\n');
                        }
                    }
                    o.Write(" C0Value = " + R(o, curvconti.C0Value()) + '\n');
                    o.Write(" C1Angle = " + R(o, curvconti.C1Angle()) + '\n');
                    o.Write(" C1Ratio = " + R(o, curvconti.C1Ratio()) + '\n');
                    break;
                case GeomAbs_C2:
                    if (curvconti.IsC2())
                    {
                        o.Write(" Continuity Status : C2 " + '\n');
                    }
                    else
                    {
                        if (curvconti.IsC1())
                        {
                            o.Write(" Continuity Status : C1 " + '\n');
                        }
                        else
                        {
                            if (curvconti.IsC0())
                            {
                                o.Write(" Continuity Status : C0 " + '\n');
                            }
                            else
                            {
                                o.Write(" Continuity Status : NoC0 " + '\n');
                            }
                        }
                    }
                    o.Write(" C0Value = " + R(o, curvconti.C0Value()) + '\n');
                    o.Write(" C1Angle = " + R(o, curvconti.C1Angle()) + '\n');
                    o.Write(" C2Angle = " + R(o, curvconti.C2Angle()) + '\n');
                    o.Write(" C1Ratio = " + R(o, curvconti.C1Ratio()) + '\n');
                    o.Write(" C2Ratio = " + R(o, curvconti.C2Ratio()) + '\n');
                    break;

                case GeomAbs_G1:
                    if (curvconti.IsG1())
                    {
                        o.Write(" Continuity Status : G1 " + '\n');
                    }
                    else
                    {
                        if (curvconti.IsC0())
                        {
                            o.Write(" Continuity Status : G0 " + '\n');
                        }
                        else
                        {
                            o.Write(" Continuity Status : NoG0 " + '\n');
                        }
                    }
                    o.Write(" G0Value = " + R(o, curvconti.C0Value()) + '\n');
                    o.Write(" G1Angle = " + R(o, curvconti.G1Angle()) + '\n');
                    break;
                case GeomAbs_G2:
                    if (curvconti.IsG2())
                    {
                        o.Write(" Continuity Status : G2 " + '\n');
                    }
                    else
                    {
                        if (curvconti.IsG1())
                        {
                            o.Write(" Continuity Status : G1 " + '\n');
                        }
                        else
                        {
                            if (curvconti.IsC0())
                            {
                                o.Write(" Continuity Status : G0 " + '\n');
                            }
                            else
                            {
                                o.Write(" Continuity Status : NoG0 " + '\n');
                            }
                        }
                    }
                    o.Write(" G0Value = " + R(o, curvconti.C0Value()) + '\n');
                    o.Write(" G1Angle = " + R(o, curvconti.G1Angle()) + '\n');
                    o.Write(" G2Angle = " + R(o, curvconti.G2Angle()) + '\n');
                    o.Write(" Relative curvature variation = " + R(o, curvconti.G2CurvatureVariation()) + '\n');
                    break;

                default:
                    break;
            }
        }
    }
}
