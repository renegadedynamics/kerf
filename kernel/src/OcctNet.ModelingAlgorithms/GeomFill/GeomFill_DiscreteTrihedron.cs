// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DiscreteTrihedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DiscreteTrihedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A discrete trihedron law: trihedrons at sampled parameters, each the previous one rotated onto the new tangent
/// (a rotation-minimizing frame). Lines and conics use the Frenet law. Its derivatives are zero.
/// </summary>
public class GeomFill_DiscreteTrihedron : GeomFill_TrihedronLaw
{
    private static readonly double TolConf = Precision.Confusion();

    private gp_Pnt myPoint;
    private readonly NCollection_HSequence<gp_Ax2> myTrihedrons;
    private readonly NCollection_HSequence<double> myKnots;
    private readonly GeomFill_Frenet myFrenet;
    private bool myUseFrenet;

    public GeomFill_DiscreteTrihedron()
    {
        myUseFrenet = false;
        myFrenet = new GeomFill_Frenet();
        myKnots = new NCollection_HSequence<double>();
        myTrihedrons = new NCollection_HSequence<gp_Ax2>();
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        GeomFill_DiscreteTrihedron copy = new GeomFill_DiscreteTrihedron();
        if (myCurve is not null)
        {
            copy.SetCurve(myCurve);
        }
        return copy;
    }

    /// <summary>Initializes the curve; returns true when the Frenet law is used (lines and conics).</summary>
    public override bool SetCurve(Adaptor3d_Curve C)
    {
        base.SetCurve(C);
        if (C is not null)
        {
            GeomAbs_CurveType type;
            type = C.GetType();
            switch (type)
            {
                case GeomAbs_Circle:
                case GeomAbs_Ellipse:
                case GeomAbs_Hyperbola:
                case GeomAbs_Parabola:
                case GeomAbs_Line:
                {
                    // No problem
                    myUseFrenet = true;
                    myFrenet.SetCurve(C);
                    break;
                }
                default:
                {
                    myUseFrenet = false;
                    // We have to fill <myKnots> and <myTrihedrons>
                    Init();
                    break;
                }
            }
        }
        return myUseFrenet;
    }

    /// <summary>Computes the sampled trihedrons.</summary>
    /// <remarks>
    /// OCCT appends to the knots and trihedrons without clearing them, so a second <see cref="SetCurve"/> on a
    /// free-form curve keeps the first curve's samples (occt-issues.md); kept.
    /// </remarks>
    public void Init()
    {
        int NbIntervals = myTrimmed!.NbIntervals(GeomAbs_CN);
        NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, NbIntervals + 1);
        myTrimmed.Intervals(Knots, GeomAbs_CN);

        int NbSamples = 10;

        int i, j;
        // OCCT: the sequences are not cleared first (a second SetCurve appends to the previous samples)
        for (i = 1; i <= NbIntervals; i++)
        {
            double delta = (Knots[i + 1] - Knots[i]) / NbSamples;
            for (j = 0; j < NbSamples; j++)
            {
                double Param = Knots[i] + j * delta;
                myKnots.Append(Param);
            }
        }
        myKnots.Append(Knots[NbIntervals + 1]);

        gp_Pnt Origin = new gp_Pnt(0.0, 0.0, 0.0), SubPnt;
        gp_Dir TangDir;
        double norm;
        for (i = 1; i <= myKnots.Length(); i++)
        {
            double Param = myKnots.Value(i);
            myTrimmed.D1(Param, out gp_Pnt Pnt, out gp_Vec Tangent);
            norm = Tangent.Magnitude();
            if (norm < TolConf)
            {
                double subdelta = (myKnots.Value(i + 1) - myKnots.Value(i)) / NbSamples;
                if (subdelta < Precision.PConfusion())
                {
                    subdelta = myKnots.Value(i + 1) - myKnots.Value(i);
                }
                SubPnt = myTrimmed.Value(Param + subdelta);
                Tangent.SetXYZ(SubPnt.XYZ() - Pnt.XYZ());
            }

            TangDir = new gp_Dir(Tangent); // normalize;
            Tangent = new gp_Vec(TangDir);
            if (i == 1) // first point
            {
                gp_Ax2 FirstAxis = new gp_Ax2(Origin, TangDir);
                myTrihedrons.Append(FirstAxis);
            }
            else
            {
                gp_Ax2 LastAxis = myTrihedrons.Value(myTrihedrons.Length());
                gp_Vec LastTangent = new gp_Vec(LastAxis.Direction());
                gp_Vec AxisOfRotation = LastTangent ^ Tangent;
                if (AxisOfRotation.Magnitude() <= gp.Resolution()) // tangents are equal or opposite
                {
                    double ScalarProduct = LastTangent * Tangent;
                    if (ScalarProduct > 0.0)
                    { // tangents are equal
                        myTrihedrons.Append(LastAxis);
                    }
                    else // tangents are opposite
                    {
                        double NewParam = (myKnots.Value(i - 1) + myKnots.Value(i)) / 2.0;
                        if (NewParam - myKnots.Value(i - 1) < gp.Resolution())
                        {
                            throw new Standard_ConstructionError(
                                "GeomFill_DiscreteTrihedron : impassable singularities on path curve");
                        }
                        myKnots.InsertBefore(i, NewParam);
                        i--;
                    }
                }
                else // good value of angle
                {
                    double theAngle = LastTangent.AngleWithRef(Tangent, AxisOfRotation);
                    gp_Ax1 theAxisOfRotation = new gp_Ax1(Origin, new gp_Dir(AxisOfRotation));
                    gp_Ax2 NewAxis = LastAxis.Rotated(theAxisOfRotation, theAngle);
                    NewAxis.SetDirection(TangDir); // to prevent accumulation of floating computations error
                    myTrihedrons.Append(NewAxis);
                }
            }
        }
    }

    public override bool D0(double Param, ref gp_Vec Tangent, ref gp_Vec Normal, ref gp_Vec BiNormal)
    {
        if (myUseFrenet)
        {
            myFrenet.D0(Param, ref Tangent, ref Normal, ref BiNormal);
        }
        else
        {
            // Locate <Param> in the sequence <myKnots>
            int Index = -1;
            double TolPar = Precision.PConfusion();
            int NbSamples = 10;
            gp_Pnt Origin = new gp_Pnt(0.0, 0.0, 0.0);

            int i;
            int I1, I2;
            I1 = 1;
            I2 = myKnots.Length();
            for (;;)
            {
                i = (I1 + I2) / 2;
                if (Param <= myKnots.Value(i))
                {
                    I2 = i;
                }
                else
                {
                    I1 = i;
                }
                if (I2 - I1 <= 1)
                {
                    break;
                }
            }
            Index = I1;
            if (Math.Abs(Param - myKnots.Value(I2)) < TolPar)
            {
                Index = I2;
            }

            double PrevParam = myKnots.Value(Index);
            gp_Ax2 PrevAxis = myTrihedrons.Value(Index);
            gp_Ax2 theAxis;
            if (Math.Abs(Param - PrevParam) < TolPar)
            {
                theAxis = PrevAxis;
            }
            else //<Param> is between knots
            {
                myTrimmed!.D1(Param, out myPoint, out Tangent);
                double norm = Tangent.Magnitude();
                if (norm < TolConf)
                {
                    double subdelta = (myKnots.Value(Index + 1) - Param) / NbSamples;
                    if (subdelta < Precision.PConfusion())
                    {
                        subdelta = myKnots.Value(Index + 1) - Param;
                    }
                    gp_Pnt SubPnt = myTrimmed.Value(Param + subdelta);
                    Tangent.SetXYZ(SubPnt.XYZ() - myPoint.XYZ());
                }

                gp_Dir TangDir = new gp_Dir(Tangent); // normalize;
                Tangent = new gp_Vec(TangDir);
                gp_Vec PrevTangent = new gp_Vec(PrevAxis.Direction());
                gp_Vec AxisOfRotation = PrevTangent ^ Tangent;
                if (AxisOfRotation.Magnitude() <= gp.Resolution()) // tangents are equal
                {
                    // we assume that tangents can not be opposite
                    theAxis = PrevAxis;
                }
                else // good value of angle
                {
                    double theAngle = PrevTangent.AngleWithRef(Tangent, AxisOfRotation);
                    gp_Ax1 theAxisOfRotation = new gp_Ax1(Origin, new gp_Dir(AxisOfRotation));
                    theAxis = PrevAxis.Rotated(theAxisOfRotation, theAngle);
                }
                theAxis.SetDirection(TangDir); // to prevent accumulation of floating computations error
            } // end of else (Param is between knots)

            Tangent = new gp_Vec(theAxis.Direction());
            Normal = new gp_Vec(theAxis.XDirection());
            BiNormal = new gp_Vec(theAxis.YDirection());
        }
        return true;
    }

    public override bool D1(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec Normal, ref gp_Vec DNormal,
                            ref gp_Vec BiNormal, ref gp_Vec DBiNormal)
    {
        if (myUseFrenet)
        {
            myFrenet.D1(Param, ref Tangent, ref DTangent, ref Normal, ref DNormal, ref BiNormal, ref DBiNormal);
        }
        else
        {
            D0(Param, ref Tangent, ref Normal, ref BiNormal);

            DTangent.SetCoord(0.0, 0.0, 0.0);
            DNormal.SetCoord(0.0, 0.0, 0.0);
            DBiNormal.SetCoord(0.0, 0.0, 0.0);
        }
        return true;
    }

    public override bool D2(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec D2Tangent, ref gp_Vec Normal,
                            ref gp_Vec DNormal, ref gp_Vec D2Normal, ref gp_Vec BiNormal, ref gp_Vec DBiNormal,
                            ref gp_Vec D2BiNormal)
    {
        if (myUseFrenet)
        {
            myFrenet.D2(Param, ref Tangent, ref DTangent, ref D2Tangent, ref Normal, ref DNormal, ref D2Normal,
                        ref BiNormal, ref DBiNormal, ref D2BiNormal);
        }
        else
        {
            D0(Param, ref Tangent, ref Normal, ref BiNormal);

            DTangent.SetCoord(0.0, 0.0, 0.0);
            DNormal.SetCoord(0.0, 0.0, 0.0);
            DBiNormal.SetCoord(0.0, 0.0, 0.0);
            D2Tangent.SetCoord(0.0, 0.0, 0.0);
            D2Normal.SetCoord(0.0, 0.0, 0.0);
            D2BiNormal.SetCoord(0.0, 0.0, 0.0);
        }
        return true;
    }

    public override int NbIntervals(GeomAbs_Shape S) => myTrimmed!.NbIntervals(GeomAbs_CN);

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        myTrimmed!.Intervals(T, GeomAbs_CN);
    }

    public override void GetAverageLaw(ref gp_Vec ATangent, ref gp_Vec ANormal, ref gp_Vec ABiNormal)
    {
        int Num = 20; // order of digitalization
        gp_Vec T = new gp_Vec(), N = new gp_Vec(), BN = new gp_Vec();
        ATangent = new gp_Vec(0, 0, 0);
        ANormal = new gp_Vec(0, 0, 0);
        ABiNormal = new gp_Vec(0, 0, 0);
        double Step = (myTrimmed!.LastParameter() - myTrimmed.FirstParameter()) / Num;
        double Param;
        for (int i = 0; i <= Num; i++)
        {
            Param = myTrimmed.FirstParameter() + i * Step;
            if (Param > myTrimmed.LastParameter())
            {
                Param = myTrimmed.LastParameter();
            }
            D0(Param, ref T, ref N, ref BN);
            ATangent += T;
            ANormal += N;
            ABiNormal += BN;
        }
        ATangent /= Num + 1;
        ANormal /= Num + 1;

        ATangent.Normalize();
        ABiNormal = ATangent.Crossed(ANormal).Normalized();
        ANormal = ABiNormal.Crossed(ATangent);
    }

    public override bool IsConstant() => myCurve!.GetType() == GeomAbs_Line;

    /// <summary>Returns true.</summary>
    public override bool IsOnlyBy3dCurve() => true;
}
