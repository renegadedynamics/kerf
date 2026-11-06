// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_ApproxSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_ApproxSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using System.Globalization;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a surface to a B-spline surface by approximation (AdvApp2Var) within a tolerance, continuities in U and
/// V, maximum degrees and a maximum number of patches. The approximation is made by the constructor.
/// </summary>
internal sealed class ApproxSurface
{
    private sealed class GeomConvert_ApproxSurface_Eval : EvaluatorFunc2Var
    {
        // Re-trimmed at every call, as OCCT's mutable handle is (decision 200).
        private SurfaceAdaptorBase _adaptor;

        public GeomConvert_ApproxSurface_Eval(SurfaceAdaptorBase adaptor) => _adaptor = adaptor;

        public override void Evaluate(int dimension,
                                      ReadOnlySpan<double> uStartEnd,
                                      ReadOnlySpan<double> vStartEnd,
                                      int favorIso,
                                      double constParam,
                                      int nbParams,
                                      ReadOnlySpan<double> parameters,
                                      int uOrder,
                                      int vOrder,
                                      Span<double> result,
                                      out int errorCode)
        {
            errorCode = 0;
            int jpar;
            double upar, vpar;

            // Dimension incorrecte
            if (dimension != 3)
            {
                errorCode = 1;
            }

            // Initialisation

            _adaptor = _adaptor.UTrim(uStartEnd[0], uStartEnd[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
            _adaptor = _adaptor.VTrim(vStartEnd[0], vStartEnd[1], global::Kerf.Engine.Foundation.Precision.PConfusion());

            int order = uOrder + vOrder;
            Point3D pnt;
            Vector3D vect, v1, v2, v3, v4, v5, v6, v7, v8, v9;

            if (favorIso == 1)
            {
                upar = constParam;
                switch (order)
                {
                    case 0:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            vpar = parameters[jpar - 1];
                            pnt = _adaptor.Value(upar, vpar);
                            result[(jpar - 1) * dimension] = pnt.X;
                            result[1 + (jpar - 1) * dimension] = pnt.Y;
                            result[2 + (jpar - 1) * dimension] = pnt.Z;
                        }

                        break;
                    case 1:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            vpar = parameters[jpar - 1];
                            _adaptor.D1(upar, vpar, out pnt, out v1, out v2);
                            if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v1.X;
                                result[1 + (jpar - 1) * dimension] = v1.Y;
                                result[2 + (jpar - 1) * dimension] = v1.Z;
                            }
                            else
                            {
                                result[(jpar - 1) * dimension] = v2.X;
                                result[1 + (jpar - 1) * dimension] = v2.Y;
                                result[2 + (jpar - 1) * dimension] = v2.Z;
                            }
                        }

                        break;
                    case 2:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            vpar = parameters[jpar - 1];
                            _adaptor.D2(upar, vpar, out pnt, out v1, out v2, out v3, out v4, out v5);
                            if (uOrder == 2)
                            {
                                result[(jpar - 1) * dimension] = v3.X;
                                result[1 + (jpar - 1) * dimension] = v3.Y;
                                result[2 + (jpar - 1) * dimension] = v3.Z;
                            }
                            else if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v5.X;
                                result[1 + (jpar - 1) * dimension] = v5.Y;
                                result[2 + (jpar - 1) * dimension] = v5.Z;
                            }
                            else if (uOrder == 0)
                            {
                                result[(jpar - 1) * dimension] = v4.X;
                                result[1 + (jpar - 1) * dimension] = v4.Y;
                                result[2 + (jpar - 1) * dimension] = v4.Z;
                            }
                        }

                        break;
                    case 3:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            vpar = parameters[jpar - 1];
                            _adaptor.D3(upar, vpar, out pnt, out v1, out v2, out v3, out v4, out v5, out v6, out v7, out v8, out v9);
                            if (uOrder == 2)
                            {
                                result[(jpar - 1) * dimension] = v8.X;
                                result[1 + (jpar - 1) * dimension] = v8.Y;
                                result[2 + (jpar - 1) * dimension] = v8.Z;
                            }
                            else if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v9.X;
                                result[1 + (jpar - 1) * dimension] = v9.Y;
                                result[2 + (jpar - 1) * dimension] = v9.Z;
                            }
                        }

                        break;
                    case 4:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            vpar = parameters[jpar - 1];
                            vect = _adaptor.DN(upar, vpar, uOrder, vOrder);
                            result[(jpar - 1) * dimension] = vect.X;
                            result[1 + (jpar - 1) * dimension] = vect.Y;
                            result[2 + (jpar - 1) * dimension] = vect.Z;
                        }

                        break;
                }
            }
            else
            {
                vpar = constParam;
                switch (order)
                {
                    case 0:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            upar = parameters[jpar - 1];
                            pnt = _adaptor.Value(upar, vpar);
                            result[(jpar - 1) * dimension] = pnt.X;
                            result[1 + (jpar - 1) * dimension] = pnt.Y;
                            result[2 + (jpar - 1) * dimension] = pnt.Z;
                        }

                        break;
                    case 1:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            upar = parameters[jpar - 1];
                            _adaptor.D1(upar, vpar, out pnt, out v1, out v2);
                            if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v1.X;
                                result[1 + (jpar - 1) * dimension] = v1.Y;
                                result[2 + (jpar - 1) * dimension] = v1.Z;
                            }
                            else
                            {
                                result[(jpar - 1) * dimension] = v2.X;
                                result[1 + (jpar - 1) * dimension] = v2.Y;
                                result[2 + (jpar - 1) * dimension] = v2.Z;
                            }
                        }

                        break;
                    case 2:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            upar = parameters[jpar - 1];
                            _adaptor.D2(upar, vpar, out pnt, out v1, out v2, out v3, out v4, out v5);
                            if (uOrder == 2)
                            {
                                result[(jpar - 1) * dimension] = v3.X;
                                result[1 + (jpar - 1) * dimension] = v3.Y;
                                result[2 + (jpar - 1) * dimension] = v3.Z;
                            }
                            else if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v5.X;
                                result[1 + (jpar - 1) * dimension] = v5.Y;
                                result[2 + (jpar - 1) * dimension] = v5.Z;
                            }
                            else if (uOrder == 0)
                            {
                                result[(jpar - 1) * dimension] = v4.X;
                                result[1 + (jpar - 1) * dimension] = v4.Y;
                                result[2 + (jpar - 1) * dimension] = v4.Z;
                            }
                        }

                        break;
                    case 3:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            upar = parameters[jpar - 1];
                            _adaptor.D3(upar, vpar, out pnt, out v1, out v2, out v3, out v4, out v5, out v6, out v7, out v8, out v9);
                            if (uOrder == 2)
                            {
                                result[(jpar - 1) * dimension] = v8.X;
                                result[1 + (jpar - 1) * dimension] = v8.Y;
                                result[2 + (jpar - 1) * dimension] = v8.Z;
                            }
                            else if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v9.X;
                                result[1 + (jpar - 1) * dimension] = v9.Y;
                                result[2 + (jpar - 1) * dimension] = v9.Z;
                            }
                        }

                        break;
                    case 4:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            upar = parameters[jpar - 1];
                            vect = _adaptor.DN(upar, vpar, uOrder, vOrder);
                            result[(jpar - 1) * dimension] = vect.X;
                            result[1 + (jpar - 1) * dimension] = vect.Y;
                            result[2 + (jpar - 1) * dimension] = vect.Z;
                        }

                        break;
                }
            }
        }
    }

    private bool _isDone;
    private bool _hasResult;
    private BSplineSurface? _bSplSurf;
    private double _maxError;

    /// <summary>
    /// Approximates Surf within Tol3d, with continuities UContinuity and VContinuity, degrees at most MaxDegU and
    /// MaxDegV, at most MaxSegments patches, and AdvApp2Var's precision code PrecisCode (0 to 3).
    /// </summary>
    public ApproxSurface(Surface surf,
                                     double tol3d,
                                     Continuity uContinuity,
                                     Continuity vContinuity,
                                     int maxDegU,
                                     int maxDegV,
                                     int maxSegments,
                                     int precisCode)
    {
        SurfaceAdaptorBase surfAdaptor = new SurfaceAdaptor(surf);
        Approximate(surfAdaptor, tol3d, uContinuity, vContinuity, maxDegU, maxDegV, maxSegments, precisCode);
    }

    /// <summary>As the other constructor, for a surface given by an adaptor (which the approximation trims).</summary>
    public ApproxSurface(SurfaceAdaptorBase surf,
                                     double tol3d,
                                     Continuity uContinuity,
                                     Continuity vContinuity,
                                     int maxDegU,
                                     int maxDegV,
                                     int maxSegments,
                                     int precisCode)
    {
        Approximate(surf, tol3d, uContinuity, vContinuity, maxDegU, maxDegV, maxSegments, precisCode);
    }

    private void Approximate(SurfaceAdaptorBase surf,
                             double tol3d,
                             Continuity uContinuity,
                             Continuity vContinuity,
                             int maxDegU,
                             int maxDegV,
                             int maxSegments,
                             int precisCode)
    {
        double u0 = surf.FirstUParameter();
        double u1 = surf.LastUParameter();
        double v0 = surf.FirstVParameter();
        double v1 = surf.LastVParameter();

        // "Init of subspace counts and tolerances"
        int nb1 = 0, nb2 = 0, nb3 = 1;
        NCollection_HArray1<double> nul1 = new(1, 1);
        nul1.SetValue(1, 0.0);
        NCollection_HArray2<double> nul2 = new(1, 1, 1, 4);
        nul2.SetValue(1, 1, 0.0);
        nul2.SetValue(1, 2, 0.0);
        nul2.SetValue(1, 3, 0.0);
        nul2.SetValue(1, 4, 0.0);
        NCollection_HArray1<double> eps3D = new(1, 1);
        eps3D.SetValue(1, tol3d);
        NCollection_HArray2<double> epsfr = new(1, 1, 1, 4);
        epsfr.SetValue(1, 1, tol3d);
        epsfr.SetValue(1, 2, tol3d);
        epsfr.SetValue(1, 3, tol3d);
        epsfr.SetValue(1, 4, tol3d);

        // Init of the iso type
        IsoType isoType = IsoV;
        int nbDec;

        nbDec = surf.NbUIntervals(C2);
        NCollection_Array1<double> UDec_C2 = new(1, nbDec + 1);
        surf.UIntervals(UDec_C2, C2);
        nbDec = surf.NbVIntervals(C2);
        NCollection_Array1<double> VDec_C2 = new(1, nbDec + 1);
        surf.VIntervals(VDec_C2, C2);

        nbDec = surf.NbUIntervals(C3);
        NCollection_Array1<double> UDec_C3 = new(1, nbDec + 1);
        surf.UIntervals(UDec_C3, C3);

        nbDec = surf.NbVIntervals(C3);
        NCollection_Array1<double> VDec_C3 = new(1, nbDec + 1);
        surf.VIntervals(VDec_C3, C3);
        // Approximation with preferential splitting
        // at C2 discontinuity locations
        PrefAndRec pUDec = new(UDec_C2, UDec_C3);
        PrefAndRec pVDec = new(VDec_C2, VDec_C3);

        GeomConvert_ApproxSurface_Eval ev = new(surf);
        ApproxAFunc2Var approx = new(nb1,
                                                nb2,
                                                nb3,
                                                nul1,
                                                nul1,
                                                eps3D,
                                                nul2,
                                                nul2,
                                                epsfr,
                                                u0,
                                                u1,
                                                v0,
                                                v1,
                                                isoType,
                                                uContinuity,
                                                vContinuity,
                                                precisCode,
                                                maxDegU,
                                                maxDegV,
                                                maxSegments,
                                                ev,
                                                pUDec,
                                                pVDec);

        // OCCT: AdvApp2Var's maximum error is underestimated (mma2ce1_, occt-issues.md)
        _maxError = approx.MaxError(3, 1);
        _bSplSurf = approx.Surface(1);
        _isDone = approx.IsDone;
        _hasResult = approx.HasResult;
    }

    /// <summary>The B-spline surface of the approximation (frozen: AdvApp2Var's own, decision 197).</summary>
    public BSplineSurface? Surface() => _bSplSurf;

    /// <summary>Whether the approximation is within the required tolerance.</summary>
    public bool IsDone => _isDone;

    /// <summary>
    /// Whether the approximation has a result, not necessarily within the required tolerance or with the wished
    /// continuities.
    /// </summary>
    public bool HasResult => _hasResult;

    /// <summary>
    /// The largest distance between the surface and its approximation, as AdvApp2Var estimates it (an underestimate,
    /// see occt-issues.md).
    /// </summary>
    public double MaxError() => _maxError;

    /// <summary>Writes whether there is a result, and its maximum error.</summary>
    public void Dump(TextWriter o)
    {
        o.Write('\n');
        if (!_hasResult)
        {
            o.Write("No result" + '\n');
        }
        else
        {
            o.Write("Result max error :" + _maxError.ToString(CultureInfo.InvariantCulture) + '\n');
        }

        o.Write('\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
