// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_ApproxSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_ApproxSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.GeomAbs_IsoType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Converts a surface to a B-spline surface by approximation (AdvApp2Var) within a tolerance, continuities in U and
/// V, maximum degrees and a maximum number of patches. The approximation is made by the constructor.
/// </summary>
public sealed class GeomConvert_ApproxSurface
{
    private sealed class GeomConvert_ApproxSurface_Eval : AdvApp2Var_EvaluatorFunc2Var
    {
        // Re-trimmed at every call, as OCCT's mutable handle is (decision 200).
        private Adaptor3d_Surface myAdaptor;

        public GeomConvert_ApproxSurface_Eval(Adaptor3d_Surface theAdaptor) => myAdaptor = theAdaptor;

        public override void Evaluate(int Dimension,
                                      ReadOnlySpan<double> UStartEnd,
                                      ReadOnlySpan<double> VStartEnd,
                                      int FavorIso,
                                      double ConstParam,
                                      int NbParams,
                                      ReadOnlySpan<double> Parameters,
                                      int UOrder,
                                      int VOrder,
                                      Span<double> Result,
                                      out int ErrorCode)
        {
            ErrorCode = 0;
            int jpar;
            double Upar, Vpar;

            // Dimension incorrecte
            if (Dimension != 3)
            {
                ErrorCode = 1;
            }

            // Initialisation

            myAdaptor = myAdaptor.UTrim(UStartEnd[0], UStartEnd[1], Precision.PConfusion());
            myAdaptor = myAdaptor.VTrim(VStartEnd[0], VStartEnd[1], Precision.PConfusion());

            int Order = UOrder + VOrder;
            gp_Pnt pnt;
            gp_Vec vect, v1, v2, v3, v4, v5, v6, v7, v8, v9;

            if (FavorIso == 1)
            {
                Upar = ConstParam;
                switch (Order)
                {
                    case 0:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Vpar = Parameters[jpar - 1];
                            pnt = myAdaptor.Value(Upar, Vpar);
                            Result[(jpar - 1) * Dimension] = pnt.X();
                            Result[1 + (jpar - 1) * Dimension] = pnt.Y();
                            Result[2 + (jpar - 1) * Dimension] = pnt.Z();
                        }

                        break;
                    case 1:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Vpar = Parameters[jpar - 1];
                            myAdaptor.D1(Upar, Vpar, out pnt, out v1, out v2);
                            if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v1.X();
                                Result[1 + (jpar - 1) * Dimension] = v1.Y();
                                Result[2 + (jpar - 1) * Dimension] = v1.Z();
                            }
                            else
                            {
                                Result[(jpar - 1) * Dimension] = v2.X();
                                Result[1 + (jpar - 1) * Dimension] = v2.Y();
                                Result[2 + (jpar - 1) * Dimension] = v2.Z();
                            }
                        }

                        break;
                    case 2:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Vpar = Parameters[jpar - 1];
                            myAdaptor.D2(Upar, Vpar, out pnt, out v1, out v2, out v3, out v4, out v5);
                            if (UOrder == 2)
                            {
                                Result[(jpar - 1) * Dimension] = v3.X();
                                Result[1 + (jpar - 1) * Dimension] = v3.Y();
                                Result[2 + (jpar - 1) * Dimension] = v3.Z();
                            }
                            else if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v5.X();
                                Result[1 + (jpar - 1) * Dimension] = v5.Y();
                                Result[2 + (jpar - 1) * Dimension] = v5.Z();
                            }
                            else if (UOrder == 0)
                            {
                                Result[(jpar - 1) * Dimension] = v4.X();
                                Result[1 + (jpar - 1) * Dimension] = v4.Y();
                                Result[2 + (jpar - 1) * Dimension] = v4.Z();
                            }
                        }

                        break;
                    case 3:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Vpar = Parameters[jpar - 1];
                            myAdaptor.D3(Upar, Vpar, out pnt, out v1, out v2, out v3, out v4, out v5, out v6, out v7, out v8, out v9);
                            if (UOrder == 2)
                            {
                                Result[(jpar - 1) * Dimension] = v8.X();
                                Result[1 + (jpar - 1) * Dimension] = v8.Y();
                                Result[2 + (jpar - 1) * Dimension] = v8.Z();
                            }
                            else if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v9.X();
                                Result[1 + (jpar - 1) * Dimension] = v9.Y();
                                Result[2 + (jpar - 1) * Dimension] = v9.Z();
                            }
                        }

                        break;
                    case 4:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Vpar = Parameters[jpar - 1];
                            vect = myAdaptor.DN(Upar, Vpar, UOrder, VOrder);
                            Result[(jpar - 1) * Dimension] = vect.X();
                            Result[1 + (jpar - 1) * Dimension] = vect.Y();
                            Result[2 + (jpar - 1) * Dimension] = vect.Z();
                        }

                        break;
                }
            }
            else
            {
                Vpar = ConstParam;
                switch (Order)
                {
                    case 0:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Upar = Parameters[jpar - 1];
                            pnt = myAdaptor.Value(Upar, Vpar);
                            Result[(jpar - 1) * Dimension] = pnt.X();
                            Result[1 + (jpar - 1) * Dimension] = pnt.Y();
                            Result[2 + (jpar - 1) * Dimension] = pnt.Z();
                        }

                        break;
                    case 1:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Upar = Parameters[jpar - 1];
                            myAdaptor.D1(Upar, Vpar, out pnt, out v1, out v2);
                            if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v1.X();
                                Result[1 + (jpar - 1) * Dimension] = v1.Y();
                                Result[2 + (jpar - 1) * Dimension] = v1.Z();
                            }
                            else
                            {
                                Result[(jpar - 1) * Dimension] = v2.X();
                                Result[1 + (jpar - 1) * Dimension] = v2.Y();
                                Result[2 + (jpar - 1) * Dimension] = v2.Z();
                            }
                        }

                        break;
                    case 2:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Upar = Parameters[jpar - 1];
                            myAdaptor.D2(Upar, Vpar, out pnt, out v1, out v2, out v3, out v4, out v5);
                            if (UOrder == 2)
                            {
                                Result[(jpar - 1) * Dimension] = v3.X();
                                Result[1 + (jpar - 1) * Dimension] = v3.Y();
                                Result[2 + (jpar - 1) * Dimension] = v3.Z();
                            }
                            else if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v5.X();
                                Result[1 + (jpar - 1) * Dimension] = v5.Y();
                                Result[2 + (jpar - 1) * Dimension] = v5.Z();
                            }
                            else if (UOrder == 0)
                            {
                                Result[(jpar - 1) * Dimension] = v4.X();
                                Result[1 + (jpar - 1) * Dimension] = v4.Y();
                                Result[2 + (jpar - 1) * Dimension] = v4.Z();
                            }
                        }

                        break;
                    case 3:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Upar = Parameters[jpar - 1];
                            myAdaptor.D3(Upar, Vpar, out pnt, out v1, out v2, out v3, out v4, out v5, out v6, out v7, out v8, out v9);
                            if (UOrder == 2)
                            {
                                Result[(jpar - 1) * Dimension] = v8.X();
                                Result[1 + (jpar - 1) * Dimension] = v8.Y();
                                Result[2 + (jpar - 1) * Dimension] = v8.Z();
                            }
                            else if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v9.X();
                                Result[1 + (jpar - 1) * Dimension] = v9.Y();
                                Result[2 + (jpar - 1) * Dimension] = v9.Z();
                            }
                        }

                        break;
                    case 4:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Upar = Parameters[jpar - 1];
                            vect = myAdaptor.DN(Upar, Vpar, UOrder, VOrder);
                            Result[(jpar - 1) * Dimension] = vect.X();
                            Result[1 + (jpar - 1) * Dimension] = vect.Y();
                            Result[2 + (jpar - 1) * Dimension] = vect.Z();
                        }

                        break;
                }
            }
        }
    }

    private bool myIsDone;
    private bool myHasResult;
    private Geom_BSplineSurface? myBSplSurf;
    private double myMaxError;

    /// <summary>
    /// Approximates Surf within Tol3d, with continuities UContinuity and VContinuity, degrees at most MaxDegU and
    /// MaxDegV, at most MaxSegments patches, and AdvApp2Var's precision code PrecisCode (0 to 3).
    /// </summary>
    public GeomConvert_ApproxSurface(Geom_Surface Surf,
                                     double Tol3d,
                                     GeomAbs_Shape UContinuity,
                                     GeomAbs_Shape VContinuity,
                                     int MaxDegU,
                                     int MaxDegV,
                                     int MaxSegments,
                                     int PrecisCode)
    {
        Adaptor3d_Surface aSurfAdaptor = new GeomAdaptor_Surface(Surf);
        Approximate(aSurfAdaptor, Tol3d, UContinuity, VContinuity, MaxDegU, MaxDegV, MaxSegments, PrecisCode);
    }

    /// <summary>As the other constructor, for a surface given by an adaptor (which the approximation trims).</summary>
    public GeomConvert_ApproxSurface(Adaptor3d_Surface Surf,
                                     double Tol3d,
                                     GeomAbs_Shape UContinuity,
                                     GeomAbs_Shape VContinuity,
                                     int MaxDegU,
                                     int MaxDegV,
                                     int MaxSegments,
                                     int PrecisCode)
    {
        Approximate(Surf, Tol3d, UContinuity, VContinuity, MaxDegU, MaxDegV, MaxSegments, PrecisCode);
    }

    private void Approximate(Adaptor3d_Surface theSurf,
                             double theTol3d,
                             GeomAbs_Shape theUContinuity,
                             GeomAbs_Shape theVContinuity,
                             int theMaxDegU,
                             int theMaxDegV,
                             int theMaxSegments,
                             int thePrecisCode)
    {
        double U0 = theSurf.FirstUParameter();
        double U1 = theSurf.LastUParameter();
        double V0 = theSurf.FirstVParameter();
        double V1 = theSurf.LastVParameter();

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
        eps3D.SetValue(1, theTol3d);
        NCollection_HArray2<double> epsfr = new(1, 1, 1, 4);
        epsfr.SetValue(1, 1, theTol3d);
        epsfr.SetValue(1, 2, theTol3d);
        epsfr.SetValue(1, 3, theTol3d);
        epsfr.SetValue(1, 4, theTol3d);

        // Init of the iso type
        GeomAbs_IsoType IsoType = GeomAbs_IsoV;
        int NbDec;

        NbDec = theSurf.NbUIntervals(GeomAbs_C2);
        NCollection_Array1<double> UDec_C2 = new(1, NbDec + 1);
        theSurf.UIntervals(UDec_C2, GeomAbs_C2);
        NbDec = theSurf.NbVIntervals(GeomAbs_C2);
        NCollection_Array1<double> VDec_C2 = new(1, NbDec + 1);
        theSurf.VIntervals(VDec_C2, GeomAbs_C2);

        NbDec = theSurf.NbUIntervals(GeomAbs_C3);
        NCollection_Array1<double> UDec_C3 = new(1, NbDec + 1);
        theSurf.UIntervals(UDec_C3, GeomAbs_C3);

        NbDec = theSurf.NbVIntervals(GeomAbs_C3);
        NCollection_Array1<double> VDec_C3 = new(1, NbDec + 1);
        theSurf.VIntervals(VDec_C3, GeomAbs_C3);
        // Approximation with preferential splitting
        // at C2 discontinuity locations
        AdvApprox_PrefAndRec pUDec = new(UDec_C2, UDec_C3);
        AdvApprox_PrefAndRec pVDec = new(VDec_C2, VDec_C3);

        GeomConvert_ApproxSurface_Eval ev = new(theSurf);
        AdvApp2Var_ApproxAFunc2Var approx = new(nb1,
                                                nb2,
                                                nb3,
                                                nul1,
                                                nul1,
                                                eps3D,
                                                nul2,
                                                nul2,
                                                epsfr,
                                                U0,
                                                U1,
                                                V0,
                                                V1,
                                                IsoType,
                                                theUContinuity,
                                                theVContinuity,
                                                thePrecisCode,
                                                theMaxDegU,
                                                theMaxDegV,
                                                theMaxSegments,
                                                ev,
                                                pUDec,
                                                pVDec);

        // OCCT: AdvApp2Var's maximum error is underestimated (mma2ce1_, occt-issues.md)
        myMaxError = approx.MaxError(3, 1);
        myBSplSurf = approx.Surface(1);
        myIsDone = approx.IsDone();
        myHasResult = approx.HasResult();
    }

    /// <summary>The B-spline surface of the approximation (frozen: AdvApp2Var's own, decision 197).</summary>
    public Geom_BSplineSurface? Surface() => myBSplSurf;

    /// <summary>Whether the approximation is within the required tolerance.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>
    /// Whether the approximation has a result, not necessarily within the required tolerance or with the wished
    /// continuities.
    /// </summary>
    public bool HasResult() => myHasResult;

    /// <summary>
    /// The largest distance between the surface and its approximation, as AdvApp2Var estimates it (an underestimate,
    /// see occt-issues.md).
    /// </summary>
    public double MaxError() => myMaxError;

    /// <summary>Writes whether there is a result, and its maximum error.</summary>
    public void Dump(TextWriter o)
    {
        o.Write('\n');
        if (!myHasResult)
        {
            o.Write("No result" + '\n');
        }
        else
        {
            o.Write("Result max error :" + myMaxError.ToString(CultureInfo.InvariantCulture) + '\n');
        }

        o.Write('\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
