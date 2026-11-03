// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DiscreteTrihedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DiscreteTrihedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A discrete trihedron law: trihedrons at sampled parameters, each the previous one rotated onto the new tangent
/// (a rotation-minimizing frame). Lines and conics use the Frenet law. Its derivatives are zero.
/// </summary>
public class GeomFill_DiscreteTrihedron : GeomFill_TrihedronLaw
{
    private static readonly double TolConf = global::Kerf.Engine.Foundation.Precision.Confusion();

    private Point3D myPoint;
    private readonly NCollection_HSequence<Frame3D> myTrihedrons;
    private readonly NCollection_HSequence<double> myKnots;
    private readonly GeomFill_Frenet myFrenet;
    private bool myUseFrenet;

    public GeomFill_DiscreteTrihedron()
    {
        myUseFrenet = false;
        myFrenet = new GeomFill_Frenet();
        myKnots = new NCollection_HSequence<double>();
        myTrihedrons = new NCollection_HSequence<Frame3D>();
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
    public override bool SetCurve(CurveAdaptorBase C)
    {
        base.SetCurve(C);
        if (C is not null)
        {
            CurveKind type;
            type = C.GetType();
            switch (type)
            {
                case Circle:
                case Ellipse:
                case Hyperbola:
                case Parabola:
                case Line:
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
        int NbIntervals = myTrimmed!.NbIntervals(CN);
        NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, NbIntervals + 1);
        myTrimmed.Intervals(Knots, CN);

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

        Point3D Origin = new Point3D(0.0, 0.0, 0.0), SubPnt;
        Direction3D TangDir;
        double norm;
        for (i = 1; i <= myKnots.Length(); i++)
        {
            double Param = myKnots.Value(i);
            myTrimmed.D1(Param, out Point3D Pnt, out Vector3D Tangent);
            norm = Tangent.Magnitude();
            if (norm < TolConf)
            {
                double subdelta = (myKnots.Value(i + 1) - myKnots.Value(i)) / NbSamples;
                if (subdelta < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    subdelta = myKnots.Value(i + 1) - myKnots.Value(i);
                }
                SubPnt = myTrimmed.Value(Param + subdelta);
                Tangent.SetXYZ(SubPnt.XYZ() - Pnt.XYZ());
            }

            TangDir = new Direction3D(Tangent); // normalize;
            Tangent = new Vector3D(TangDir);
            if (i == 1) // first point
            {
                Frame3D FirstAxis = new Frame3D(Origin, TangDir);
                myTrihedrons.Append(FirstAxis);
            }
            else
            {
                Frame3D LastAxis = myTrihedrons.Value(myTrihedrons.Length());
                Vector3D LastTangent = new Vector3D(LastAxis.Direction());
                Vector3D AxisOfRotation = LastTangent ^ Tangent;
                if (AxisOfRotation.Magnitude() <= Constants.Resolution()) // tangents are equal or opposite
                {
                    double ScalarProduct = LastTangent * Tangent;
                    if (ScalarProduct > 0.0)
                    { // tangents are equal
                        myTrihedrons.Append(LastAxis);
                    }
                    else // tangents are opposite
                    {
                        double NewParam = (myKnots.Value(i - 1) + myKnots.Value(i)) / 2.0;
                        if (NewParam - myKnots.Value(i - 1) < Constants.Resolution())
                        {
                            throw new ConstructionErrorException(
                                "GeomFill_DiscreteTrihedron : impassable singularities on path curve");
                        }
                        myKnots.InsertBefore(i, NewParam);
                        i--;
                    }
                }
                else // good value of angle
                {
                    double theAngle = LastTangent.AngleWithRef(Tangent, AxisOfRotation);
                    Axis3D theAxisOfRotation = new Axis3D(Origin, new Direction3D(AxisOfRotation));
                    Frame3D NewAxis = LastAxis.Rotated(theAxisOfRotation, theAngle);
                    NewAxis.SetDirection(TangDir); // to prevent accumulation of floating computations error
                    myTrihedrons.Append(NewAxis);
                }
            }
        }
    }

    public override bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        if (myUseFrenet)
        {
            myFrenet.D0(Param, ref Tangent, ref Normal, ref BiNormal);
        }
        else
        {
            // Locate <Param> in the sequence <myKnots>
            int Index = -1;
            double TolPar = global::Kerf.Engine.Foundation.Precision.PConfusion();
            int NbSamples = 10;
            Point3D Origin = new Point3D(0.0, 0.0, 0.0);

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
            Frame3D PrevAxis = myTrihedrons.Value(Index);
            Frame3D theAxis;
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
                    if (subdelta < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        subdelta = myKnots.Value(Index + 1) - Param;
                    }
                    Point3D SubPnt = myTrimmed.Value(Param + subdelta);
                    Tangent.SetXYZ(SubPnt.XYZ() - myPoint.XYZ());
                }

                Direction3D TangDir = new Direction3D(Tangent); // normalize;
                Tangent = new Vector3D(TangDir);
                Vector3D PrevTangent = new Vector3D(PrevAxis.Direction());
                Vector3D AxisOfRotation = PrevTangent ^ Tangent;
                if (AxisOfRotation.Magnitude() <= Constants.Resolution()) // tangents are equal
                {
                    // we assume that tangents can not be opposite
                    theAxis = PrevAxis;
                }
                else // good value of angle
                {
                    double theAngle = PrevTangent.AngleWithRef(Tangent, AxisOfRotation);
                    Axis3D theAxisOfRotation = new Axis3D(Origin, new Direction3D(AxisOfRotation));
                    theAxis = PrevAxis.Rotated(theAxisOfRotation, theAngle);
                }
                theAxis.SetDirection(TangDir); // to prevent accumulation of floating computations error
            } // end of else (Param is between knots)

            Tangent = new Vector3D(theAxis.Direction());
            Normal = new Vector3D(theAxis.XDirection());
            BiNormal = new Vector3D(theAxis.YDirection());
        }
        return true;
    }

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
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

    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
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

    public override int NbIntervals(Continuity S) => myTrimmed!.NbIntervals(CN);

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        myTrimmed!.Intervals(T, CN);
    }

    public override void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal)
    {
        int Num = 20; // order of digitalization
        Vector3D T = new Vector3D(), N = new Vector3D(), BN = new Vector3D();
        ATangent = new Vector3D(0, 0, 0);
        ANormal = new Vector3D(0, 0, 0);
        ABiNormal = new Vector3D(0, 0, 0);
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

    public override bool IsConstant() => myCurve!.GetType() == Line;

    /// <summary>Returns true.</summary>
    public override bool IsOnlyBy3dCurve() => true;
}
