// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor.hxx
// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>The geometric definition of 2D curves compatible with the Adaptor package.</summary>
internal static class Geom2dAdaptor
{
    /// <summary>
    /// A Geom2d curve equivalent to the adaptor HC, trimmed to its parameter range where that differs from the
    /// curve's. Bezier, B-spline and offset curves are those of the adaptor (shared, not copied).
    /// </summary>
    /// <exception cref="DomainErrorException">HC is of another curve type, or an offset curve adaptor that is
    /// not a <see cref="CurveAdaptor2D"/>.</exception>
    public static Curve2D MakeCurve(Curve2DAdaptorBase hc)
    {
        Curve2D? c2D = null;
        switch (hc.GetType())
        {

            case Line:
                {
                    LineCurve2D gl = new LineCurve2D(hc.Line());
                    c2D = gl;
                }
                break;

            case Circle:
                {
                    CircleCurve2D gl = new CircleCurve2D(hc.Circle());
                    c2D = gl;
                }
                break;

            case Ellipse:
                {
                    EllipseCurve2D gl = new EllipseCurve2D(hc.Ellipse());
                    c2D = gl;
                }
                break;

            case Parabola:
                {
                    ParabolaCurve2D gl = new ParabolaCurve2D(hc.Parabola());
                    c2D = gl;
                }
                break;

            case Hyperbola:
                {
                    HyperbolaCurve2D gl = new HyperbolaCurve2D(hc.Hyperbola());
                    c2D = gl;
                }
                break;

            case BezierCurve:
                {
                    c2D = hc.Bezier();
                }
                break;

            case BSplineCurve:
                {
                    c2D = hc.BSpline();
                }
                break;

            case OffsetCurve:
                {
                    if (hc is CurveAdaptor2D pGAC)
                    {
                        c2D = pGAC.Curve();
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
        if (c2D is not null
            && ((hc.FirstParameter() != c2D.FirstParameter)
                || (hc.LastParameter() != c2D.LastParameter)))
        {

            if (c2D.IsPeriodic
                || (hc.FirstParameter() >= c2D.FirstParameter
                    && hc.LastParameter() <= c2D.LastParameter))
            {
                c2D = new TrimmedCurve2D(c2D, hc.FirstParameter(), hc.LastParameter());
            }
            else
            {
                double tf = Max(hc.FirstParameter(), c2D.FirstParameter);
                double tl = Min(hc.LastParameter(), c2D.LastParameter);
                c2D = new TrimmedCurve2D(c2D, tf, tl);
            }
        }

        return c2D!;
    }
}
