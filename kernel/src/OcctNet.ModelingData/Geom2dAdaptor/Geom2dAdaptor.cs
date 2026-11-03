// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor.hxx
// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>The geometric definition of 2D curves compatible with the Adaptor package.</summary>
public static class Geom2dAdaptor
{
    /// <summary>
    /// A Geom2d curve equivalent to the adaptor HC, trimmed to its parameter range where that differs from the
    /// curve's. Bezier, B-spline and offset curves are those of the adaptor (shared, not copied).
    /// </summary>
    /// <exception cref="DomainErrorException">HC is of another curve type, or an offset curve adaptor that is
    /// not a <see cref="Geom2dAdaptor_Curve"/>.</exception>
    public static Geom2d_Curve MakeCurve(Adaptor2d_Curve2d HC)
    {
        Geom2d_Curve? C2D = null;
        switch (HC.GetType())
        {

            case Line:
                {
                    Geom2d_Line GL = new Geom2d_Line(HC.Line());
                    C2D = GL;
                }
                break;

            case Circle:
                {
                    Geom2d_Circle GL = new Geom2d_Circle(HC.Circle());
                    C2D = GL;
                }
                break;

            case Ellipse:
                {
                    Geom2d_Ellipse GL = new Geom2d_Ellipse(HC.Ellipse());
                    C2D = GL;
                }
                break;

            case Parabola:
                {
                    Geom2d_Parabola GL = new Geom2d_Parabola(HC.Parabola());
                    C2D = GL;
                }
                break;

            case Hyperbola:
                {
                    Geom2d_Hyperbola GL = new Geom2d_Hyperbola(HC.Hyperbola());
                    C2D = GL;
                }
                break;

            case BezierCurve:
                {
                    C2D = HC.Bezier();
                }
                break;

            case BSplineCurve:
                {
                    C2D = HC.BSpline();
                }
                break;

            case OffsetCurve:
                {
                    if (HC is Geom2dAdaptor_Curve pGAC)
                    {
                        C2D = pGAC.Curve();
                    }
                    else
                    {
                        throw new DomainErrorException("Geom2dAdaptor::MakeCurve, Not Geom2dAdaptor_Curve");
                    }
                }
                break;

            default:
                throw new DomainErrorException("Geom2dAdaptor::MakeCurve, OtherCurve");
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
