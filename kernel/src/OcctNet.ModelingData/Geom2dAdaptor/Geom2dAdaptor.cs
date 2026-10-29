// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor.hxx
// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>The geometric definition of 2D curves compatible with the Adaptor package.</summary>
public static class Geom2dAdaptor
{
    /// <summary>
    /// A Geom2d curve equivalent to the adaptor HC, trimmed to its parameter range where that differs from the
    /// curve's. Bezier, B-spline and offset curves are those of the adaptor (shared, not copied).
    /// </summary>
    /// <exception cref="Standard_DomainError">HC is of another curve type, or an offset curve adaptor that is
    /// not a <see cref="Geom2dAdaptor_Curve"/>.</exception>
    public static Geom2d_Curve MakeCurve(Adaptor2d_Curve2d HC)
    {
        Geom2d_Curve? C2D = null;
        switch (HC.GetType())
        {

            case GeomAbs_Line:
                {
                    Geom2d_Line GL = new Geom2d_Line(HC.Line());
                    C2D = GL;
                }
                break;

            case GeomAbs_Circle:
                {
                    Geom2d_Circle GL = new Geom2d_Circle(HC.Circle());
                    C2D = GL;
                }
                break;

            case GeomAbs_Ellipse:
                {
                    Geom2d_Ellipse GL = new Geom2d_Ellipse(HC.Ellipse());
                    C2D = GL;
                }
                break;

            case GeomAbs_Parabola:
                {
                    Geom2d_Parabola GL = new Geom2d_Parabola(HC.Parabola());
                    C2D = GL;
                }
                break;

            case GeomAbs_Hyperbola:
                {
                    Geom2d_Hyperbola GL = new Geom2d_Hyperbola(HC.Hyperbola());
                    C2D = GL;
                }
                break;

            case GeomAbs_BezierCurve:
                {
                    C2D = HC.Bezier();
                }
                break;

            case GeomAbs_BSplineCurve:
                {
                    C2D = HC.BSpline();
                }
                break;

            case GeomAbs_OffsetCurve:
                {
                    if (HC is Geom2dAdaptor_Curve pGAC)
                    {
                        C2D = pGAC.Curve();
                    }
                    else
                    {
                        throw new Standard_DomainError("Geom2dAdaptor::MakeCurve, Not Geom2dAdaptor_Curve");
                    }
                }
                break;

            default:
                throw new Standard_DomainError("Geom2dAdaptor::MakeCurve, OtherCurve");
        }

        // trim the curve if necassary.
        if (C2D is not null
            && ((HC.FirstParameter() != C2D.FirstParameter())
                || (HC.LastParameter() != C2D.LastParameter())))
        {

            if (C2D.IsPeriodic()
                || (HC.FirstParameter() >= C2D.FirstParameter()
                    && HC.LastParameter() <= C2D.LastParameter()))
            {
                C2D = new Geom2d_TrimmedCurve(C2D, HC.FirstParameter(), HC.LastParameter());
            }
            else
            {
                double tf = Max(HC.FirstParameter(), C2D.FirstParameter());
                double tl = Min(HC.LastParameter(), C2D.LastParameter());
                C2D = new Geom2d_TrimmedCurve(C2D, tf, tl);
            }
        }

        return C2D!;
    }
}
