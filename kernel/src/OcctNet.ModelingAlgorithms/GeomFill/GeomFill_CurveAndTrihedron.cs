// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CurveAndTrihedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CurveAndTrihedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// A location law defined by a curve and a trihedron law: M(t) has the normal, binormal and tangent as columns,
/// and V(t) is the point of the curve.
/// </summary>
/// <remarks>
/// A working object (decision 21). The law keeps the trihedron law it is given, as OCCT keeps the handle, and
/// sets its curve and interval: hand it a law of its own (<see cref="GeomFill_TrihedronLaw.Copy"/>) if the caller
/// still uses it.
/// </remarks>
public class GeomFill_CurveAndTrihedron : GeomFill_LocationLaw
{
    private bool WithTrans;
    private readonly GeomFill_TrihedronLaw myLaw;
    private Adaptor3d_Curve? myCurve;
    private Adaptor3d_Curve? myTrimmed;
    private gp_Pnt Point;
    private gp_Vec V1;
    private gp_Vec V2;
    private gp_Vec V3;
    private gp_Mat Trans;

    public GeomFill_CurveAndTrihedron(GeomFill_TrihedronLaw Trihedron)
    {
        myLaw = Trihedron;
        myCurve = null;
        Trans = new gp_Mat();
        Trans.SetIdentity();
        WithTrans = false;
    }

    public override GeomFill_LocationLaw Copy()
    {
        GeomFill_TrihedronLaw law;
        law = myLaw.Copy();
        GeomFill_CurveAndTrihedron copy = new GeomFill_CurveAndTrihedron(myLaw.Copy());
        copy.SetCurve(myCurve!);
        copy.SetTrsf(Trans);
        return copy;
    }

    public override bool SetCurve(Adaptor3d_Curve C)
    {
        myCurve = C;
        myTrimmed = C;
        return myLaw.SetCurve(C);
    }

    public override Adaptor3d_Curve? GetCurve() => myCurve;

    public override void SetTrsf(in gp_Mat Transfo)
    {
        Trans = Transfo;
        gp_Mat Aux = new gp_Mat();
        Aux.SetIdentity();
        Aux -= Trans;

        WithTrans = false; // Au cas ou Trans = I
        for (int ii = 1; ii <= 3 && !WithTrans; ii++)
        {
            for (int jj = 1; jj <= 3 && !WithTrans; jj++)
            {
                if (Math.Abs(Aux.Value(ii, jj)) > 1.0e-14)
                {
                    WithTrans = true;
                }
            }
        }
    }

    public override bool D0(double Param, ref gp_Mat M, ref gp_Vec V)
    {
        bool Ok;
        myTrimmed!.D0(Param, out Point);
        V.SetXYZ(Point.XYZ());

        Ok = myLaw.D0(Param, ref V1, ref V2, ref V3);
        M.SetCols(V2.XYZ(), V3.XYZ(), V1.XYZ());

        if (WithTrans)
        {
            M *= Trans;
        }

        return Ok;
    }

    public override bool D0(double Param, ref gp_Mat M, ref gp_Vec V, NCollection_Array1<gp_Pnt2d> Poles2d)
    {
        bool Ok;
        myTrimmed!.D0(Param, out Point);
        V.SetXYZ(Point.XYZ());

        Ok = myLaw.D0(Param, ref V1, ref V2, ref V3);
        M.SetCols(V2.XYZ(), V3.XYZ(), V1.XYZ());

        if (WithTrans)
        {
            M *= Trans;
        }

        return Ok;
    }

    public override bool D1(double Param, ref gp_Mat M, ref gp_Vec V, ref gp_Mat DM, ref gp_Vec DV,
                            NCollection_Array1<gp_Pnt2d> Poles2d, NCollection_Array1<gp_Vec2d> DPoles2d)
    {
        bool Ok;
        myTrimmed!.D1(Param, out Point, out DV);
        V.SetXYZ(Point.XYZ());

        gp_Vec DV1 = new gp_Vec(), DV2 = new gp_Vec(), DV3 = new gp_Vec();
        Ok = myLaw.D1(Param, ref V1, ref DV1, ref V2, ref DV2, ref V3, ref DV3);
        M.SetCols(V2.XYZ(), V3.XYZ(), V1.XYZ());
        DM.SetCols(DV2.XYZ(), DV3.XYZ(), DV1.XYZ());

        if (WithTrans)
        {
            M *= Trans;
            DM *= Trans;
        }

        return Ok;
    }

    public override bool D2(double Param, ref gp_Mat M, ref gp_Vec V, ref gp_Mat DM, ref gp_Vec DV, ref gp_Mat D2M,
                            ref gp_Vec D2V, NCollection_Array1<gp_Pnt2d> Poles2d, NCollection_Array1<gp_Vec2d> DPoles2d,
                            NCollection_Array1<gp_Vec2d> D2Poles2d)
    {
        bool Ok;
        myTrimmed!.D2(Param, out Point, out DV, out D2V);
        V.SetXYZ(Point.XYZ());

        gp_Vec DV1 = new gp_Vec(), DV2 = new gp_Vec(), DV3 = new gp_Vec();
        gp_Vec D2V1 = new gp_Vec(), D2V2 = new gp_Vec(), D2V3 = new gp_Vec();
        Ok = myLaw.D2(Param, ref V1, ref DV1, ref D2V1, ref V2, ref DV2, ref D2V2, ref V3, ref DV3, ref D2V3);

        M.SetCols(V2.XYZ(), V3.XYZ(), V1.XYZ());
        DM.SetCols(DV2.XYZ(), DV3.XYZ(), DV1.XYZ());
        D2M.SetCols(D2V2.XYZ(), D2V3.XYZ(), D2V1.XYZ());

        if (WithTrans)
        {
            M *= Trans;
            DM *= Trans;
            D2M *= Trans;
        }
        return Ok;
    }

    public override int NbIntervals(GeomAbs_Shape S)
    {
        int Nb_Sec, Nb_Law;
        Nb_Sec = myTrimmed!.NbIntervals(S);
        Nb_Law = myLaw.NbIntervals(S);

        if (Nb_Sec == 1)
        {
            return Nb_Law;
        }
        else if (Nb_Law == 1)
        {
            return Nb_Sec;
        }

        NCollection_Array1<double> IntC = new(1, Nb_Sec + 1);
        NCollection_Array1<double> IntL = new(1, Nb_Law + 1);
        NCollection_Sequence<double> Inter = new();
        myTrimmed.Intervals(IntC, S);
        myLaw.Intervals(IntL, S);

        GeomLib.FuseIntervals(IntC, IntL, Inter, Precision.PConfusion() * 0.99);
        return Inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        int Nb_Sec, Nb_Law;
        Nb_Sec = myTrimmed!.NbIntervals(S);
        Nb_Law = myLaw.NbIntervals(S);

        if (Nb_Sec == 1)
        {
            myLaw.Intervals(T, S);
            return;
        }
        else if (Nb_Law == 1)
        {
            myTrimmed.Intervals(T, S);
            return;
        }

        NCollection_Array1<double> IntC = new(1, Nb_Sec + 1);
        NCollection_Array1<double> IntL = new(1, Nb_Law + 1);
        NCollection_Sequence<double> Inter = new();
        myTrimmed.Intervals(IntC, S);
        myLaw.Intervals(IntL, S);

        GeomLib.FuseIntervals(IntC, IntL, Inter, Precision.PConfusion() * 0.99);
        for (int ii = 1; ii <= Inter.Length(); ii++)
        {
            T[ii] = Inter[ii];
        }
    }

    public override void SetInterval(double First, double Last)
    {
        myLaw.SetInterval(First, Last);
        myTrimmed = myCurve!.Trim(First, Last, 0);
    }

    public override void GetInterval(out double First, out double Last)
    {
        First = myTrimmed!.FirstParameter();
        Last = myTrimmed.LastParameter();
    }

    public override void GetDomain(out double First, out double Last)
    {
        First = myCurve!.FirstParameter();
        Last = myCurve.LastParameter();
    }

    /// <summary>1: the trihedrons are normed.</summary>
    public override double GetMaximalNorm() => 1.0;

    public override void GetAverageLaw(ref gp_Mat AM, ref gp_Vec AV)
    {
        int ii;
        double U, delta;
        gp_Vec V = new gp_Vec();

        myLaw.GetAverageLaw(ref V1, ref V2, ref V3);
        // OCCT: the columns are (tangent, normal, binormal) here, but (normal, binormal, tangent) in D0, D1 and D2.
        AM.SetCols(V1.XYZ(), V2.XYZ(), V3.XYZ());

        AV.SetCoord(0.0, 0.0, 0.0);
        delta = (myTrimmed!.LastParameter() - myTrimmed.FirstParameter()) / 10;
        U = myTrimmed.FirstParameter();
        for (ii = 0; ii <= 10; ii++, U += delta)
        {
            V.SetXYZ(myTrimmed.Value(U).XYZ());
            AV += V;
        }
        AV /= 11;
    }

    public override bool IsTranslation(ref double Error)
    {
        GeomAbs_CurveType Type;
        Error = 0;
        Type = myCurve!.GetType();
        if (Type == GeomAbs_Line)
        {
            return myLaw.IsConstant() || myLaw.IsOnlyBy3dCurve();
        }
        return false;
    }

    public override bool IsRotation(ref double Error)
    {
        GeomAbs_CurveType Type;
        Error = 0;
        Type = myCurve!.GetType();
        if (Type == GeomAbs_Circle)
        {
            return myLaw.IsOnlyBy3dCurve();
        }
        return false;
    }

    public override void Rotation(ref gp_Pnt Centre)
    {
        Centre = myCurve!.Circle().Location();
    }
}
