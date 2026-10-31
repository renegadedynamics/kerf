// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function swept by <see cref="GeomFill_Sweep"/>: a section law placed by a location law, as the
/// <see cref="Approx_SweepFunction"/> that <see cref="Approx_SweepApproximation"/> approximates.
/// </summary>
/// <remarks>
/// A working object (decision 21): it keeps the caller's laws, as OCCT keeps the handles, and
/// <see cref="SetInterval"/> and <see cref="SetTolerance"/> change them.
/// </remarks>
public class GeomFill_SweepFunction : Approx_SweepFunction
{
    private readonly GeomFill_LocationLaw myLoc;
    private readonly GeomFill_SectionLaw mySec;
    private readonly double myf;
    private readonly double myfOnS;
    private readonly double myRatio;
    private gp_Mat M = new gp_Mat();
    private gp_Mat DM = new gp_Mat();
    private gp_Mat D2M = new gp_Mat();
    private gp_Vec V = new gp_Vec();
    private gp_Vec DV = new gp_Vec();
    private gp_Vec D2V = new gp_Vec();

    /// <summary>
    /// The sweep of Section along Location: the section law is evaluated at
    /// FirstParameterOnS + (v - FirstParameter) * RatioParameterOnS.
    /// </summary>
    public GeomFill_SweepFunction(GeomFill_SectionLaw Section,
                                  GeomFill_LocationLaw Location,
                                  double FirstParameter,
                                  double FirstParameterOnS,
                                  double RatioParameterOnS)
    {
        myLoc = Location;
        mySec = Section;
        myf = FirstParameter;
        myfOnS = FirstParameterOnS;
        myRatio = RatioParameterOnS;
    }

    public override bool D0(double Param,
                            double First,
                            double Last,
                            NCollection_Array1<gp_Pnt> Poles,
                            NCollection_Array1<gp_Pnt2d> Poles2d,
                            NCollection_Array1<double> Weigths)
    {
        int ii, L;
        bool Ok;
        double T = myfOnS + (Param - myf) * myRatio;
        L = Poles.Length();

        Ok = myLoc.D0(Param, ref M, ref V, Poles2d);
        if (!Ok)
        {
            return Ok;
        }
        Ok = mySec.D0(T, Poles, Weigths);
        if (!Ok)
        {
            return Ok;
        }

        for (ii = 1; ii <= L; ii++)
        {
            ref gp_XYZ aux = ref Poles[ii].ChangeCoord();
            aux *= M;
            aux += V.XYZ();
        }
        return true;
    }

    public override bool D1(double Param,
                            double First,
                            double Last,
                            NCollection_Array1<gp_Pnt> Poles,
                            NCollection_Array1<gp_Vec> DPoles,
                            NCollection_Array1<gp_Pnt2d> Poles2d,
                            NCollection_Array1<gp_Vec2d> DPoles2d,
                            NCollection_Array1<double> Weigths,
                            NCollection_Array1<double> DWeigths)
    {
        int ii, L;
        bool Ok;
        double T = myfOnS + (Param - myf) * myRatio;
        gp_XYZ PPrim;
        L = Poles.Length();

        Ok = myLoc.D1(Param, ref M, ref V, ref DM, ref DV, Poles2d, DPoles2d);
        if (!Ok)
        {
            return Ok;
        }
        Ok = mySec.D1(T, Poles, DPoles, Weigths, DWeigths);
        if (!Ok)
        {
            return Ok;
        }

        for (ii = 1; ii <= L; ii++)
        {
            PPrim = DPoles[ii].XYZ();
            ref gp_XYZ P = ref Poles[ii].ChangeCoord();
            PPrim *= myRatio;
            DWeigths[ii] *= myRatio;
            PPrim *= M;
            PPrim += DM * P;
            PPrim += DV.XYZ();
            DPoles[ii].SetXYZ(PPrim);

            P *= M;
            P += V.XYZ();
        }
        return true;
    }

    public override bool D2(double Param,
                            double First,
                            double Last,
                            NCollection_Array1<gp_Pnt> Poles,
                            NCollection_Array1<gp_Vec> DPoles,
                            NCollection_Array1<gp_Vec> D2Poles,
                            NCollection_Array1<gp_Pnt2d> Poles2d,
                            NCollection_Array1<gp_Vec2d> DPoles2d,
                            NCollection_Array1<gp_Vec2d> D2Poles2d,
                            NCollection_Array1<double> Weigths,
                            NCollection_Array1<double> DWeigths,
                            NCollection_Array1<double> D2Weigths)
    {
        int ii, L;
        bool Ok;
        double T = myfOnS + (Param - myf) * myRatio;
        double squareratio = myRatio * myRatio;
        L = Poles.Length();

        Ok = myLoc.D2(Param, ref M, ref V, ref DM, ref DV, ref D2M, ref D2V, Poles2d, DPoles2d, D2Poles2d);
        if (!Ok)
        {
            return Ok;
        }
        Ok = mySec.D2(T, Poles, DPoles, D2Poles, Weigths, DWeigths, D2Weigths);
        if (!Ok)
        {
            return Ok;
        }

        for (ii = 1; ii <= L; ii++)
        {
            gp_XYZ PSecn = D2Poles[ii].XYZ();
            gp_XYZ PPrim = DPoles[ii].XYZ();
            PPrim *= myRatio;
            DWeigths[ii] *= myRatio;
            PSecn *= squareratio;
            D2Weigths[ii] *= squareratio;
            ref gp_XYZ P = ref Poles[ii].ChangeCoord();

            PSecn *= M;
            PSecn += 2 * (DM * PPrim);
            PSecn += D2M * P;
            PSecn += D2V.XYZ();
            D2Poles[ii].SetXYZ(PSecn);

            PPrim *= M;
            PPrim += DM * P;
            PPrim += DV.XYZ();
            DPoles[ii].SetXYZ(PPrim);

            P *= M;
            P += V.XYZ();
        }
        return true;
    }

    public override int Nb2dCurves() => myLoc.Nb2dCurves();

    public override void SectionShape(out int NbPoles, out int NbKnots, out int Degree)
    {
        mySec.SectionShape(out NbPoles, out NbKnots, out Degree);
    }

    public override void Knots(NCollection_Array1<double> TKnots)
    {
        mySec.Knots(TKnots);
    }

    public override void Mults(NCollection_Array1<int> TMults)
    {
        mySec.Mults(TMults);
    }

    public override bool IsRational() => mySec.IsRational();

    public override int NbIntervals(GeomAbs_Shape S)
    {
        int Nb_Sec, Nb_Loc;
        Nb_Sec = mySec.NbIntervals(S);
        Nb_Loc = myLoc.NbIntervals(S);

        if (Nb_Sec == 1)
        {
            return Nb_Loc;
        }
        else if (Nb_Loc == 1)
        {
            return Nb_Sec;
        }

        NCollection_Array1<double> IntS = new(1, Nb_Sec + 1);
        NCollection_Array1<double> IntL = new(1, Nb_Loc + 1);
        NCollection_Sequence<double> Inter = new();
        double T;
        int ii;
        mySec.Intervals(IntS, S);
        for (ii = 1; ii <= Nb_Sec + 1; ii++)
        {
            T = (IntS[ii] - myfOnS) / myRatio + myf;
            IntS[ii] = T;
        }
        myLoc.Intervals(IntL, S);

        GeomLib.FuseIntervals(IntS, IntL, Inter, Precision.PConfusion() * 0.99);
        return Inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        int Nb_Sec, Nb_Loc, ii;
        Nb_Sec = mySec.NbIntervals(S);
        Nb_Loc = myLoc.NbIntervals(S);

        if (Nb_Sec == 1)
        {
            myLoc.Intervals(T, S);
            return;
        }
        else if (Nb_Loc == 1)
        {
            double tt;
            mySec.Intervals(T, S);
            for (ii = 1; ii <= Nb_Sec + 1; ii++)
            {
                tt = (T[ii] - myfOnS) / myRatio + myf;
                T[ii] = tt;
            }
            return;
        }

        NCollection_Array1<double> IntS = new(1, Nb_Sec + 1);
        NCollection_Array1<double> IntL = new(1, Nb_Loc + 1);
        NCollection_Sequence<double> Inter = new();
        double t;

        mySec.Intervals(IntS, S);
        for (ii = 1; ii <= Nb_Sec + 1; ii++)
        {
            t = (IntS[ii] - myfOnS) / myRatio + myf;
            IntS[ii] = t;
        }
        myLoc.Intervals(IntL, S);

        GeomLib.FuseIntervals(IntS, IntL, Inter, Precision.PConfusion() * 0.99);
        for (ii = 1; ii <= Inter.Length(); ii++)
        {
            T[ii] = Inter[ii];
        }
    }

    public override void SetInterval(double First, double Last)
    {
        double uf, ul;
        myLoc.SetInterval(First, Last);
        uf = myf + (First - myf) * myRatio;
        ul = myf + (Last - myf) * myRatio;
        mySec.SetInterval(uf, ul);
    }

    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol, NCollection_Array1<double> Tol3d)
    {
        mySec.GetTolerance(BoundTol, SurfTol, AngleTol, Tol3d);
    }

    public override void Resolution(int Index, double Tol, out double TolU, out double TolV)
    {
        myLoc.Resolution(Index, Tol, out TolU, out TolV);
    }

    public override void SetTolerance(double Tol3d, double Tol2d)
    {
        mySec.SetTolerance(Tol3d, Tol2d);
        myLoc.SetTolerance(Tol3d, Tol2d);
    }

    public override gp_Pnt BarycentreOfSurf()
    {
        gp_Pnt Bary;
        gp_Vec Translate = new gp_Vec();
        gp_Mat aM = new gp_Mat();

        Bary = mySec.BarycentreOfSurf();
        myLoc.GetAverageLaw(ref aM, ref Translate);
        Bary.ChangeCoord() *= aM;
        Bary.ChangeCoord() += Translate.XYZ();

        return Bary;
    }

    public override double MaximalSection()
    {
        double L = mySec.MaximalSection();
        L *= myLoc.GetMaximalNorm();
        return L;
    }

    public override void GetMinimalWeight(NCollection_Array1<double> Weigths)
    {
        mySec.GetMinimalWeight(Weigths);
    }
}
