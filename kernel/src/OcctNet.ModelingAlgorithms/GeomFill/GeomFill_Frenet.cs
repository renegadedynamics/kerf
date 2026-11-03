// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Frenet.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Frenet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>The Frenet trihedron law: the tangent, normal and binormal of the curve.</summary>
/// <remarks>
/// <see cref="SetCurve"/> searches the curve's singular points (where C' ^ C'' vanishes) unless it is a line or a
/// conic; near them the trihedron is computed from higher derivatives.
/// </remarks>
public class GeomFill_Frenet : GeomFill_TrihedronLaw
{
    private const double NullTol = 1.0e-10;
    private const double MaxSingular = 1.0e-5;
    private const int maxDerivOrder = 3;

    private Point3D P;
    private NCollection_HArray1<double>? mySngl;
    private NCollection_HArray1<double>? mySnglLen;
    private bool isSngl;

    /// <summary>Computes (F/|F|)'.</summary>
    private static Vector3D FDeriv(in Vector3D F, in Vector3D DF)
    {
        double Norma = F.Magnitude();
        Vector3D Result = (DF - F * (F * DF) / (Norma * Norma)) / Norma;
        return Result;
    }

    /// <summary>Computes (F/|F|)''.</summary>
    private static Vector3D DDeriv(in Vector3D F, in Vector3D DF, in Vector3D D2F)
    {
        double Norma = F.Magnitude();
        Vector3D Result = (D2F - 2 * DF * (F * DF) / (Norma * Norma)) / Norma
                        - F * ((DF.SquareMagnitude() + F * D2F - 3 * (F * DF) * (F * DF) / (Norma * Norma))
                               / (Norma * Norma * Norma));
        return Result;
    }

    /// <summary>The cosine of the angle between theV1 and theV2.</summary>
    private static double CosAngle(in Vector3D theV1, in Vector3D theV2)
    {
        double aTol = Constants.Resolution();
        double m1 = theV1.Magnitude(), m2 = theV2.Magnitude();
        if ((m1 <= aTol) || (m2 <= aTol))
        { // Vectors are codirectional
            return 1.0;
        }
        double aCAng = theV1.Dot(theV2) / (m1 * m2);
        if (aCAng > 1.0)
        {
            aCAng = 1.0;
        }
        if (aCAng < -1.0)
        {
            aCAng = -1.0;
        }
        return aCAng;
    }

    public GeomFill_Frenet()
    {
        isSngl = false;
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        GeomFill_Frenet copy = new GeomFill_Frenet();
        if (myCurve is not null)
        {
            copy.SetCurve(myCurve);
        }
        return copy;
    }

    /// <summary>Initializes the curve of the Frenet law; returns true.</summary>
    public override bool SetCurve(Adaptor3d_Curve C)
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
                    isSngl = false;
                    break;
                }
                default:
                {
                    // We have to search singularities
                    Init();
                    break;
                }
            }
        }
        return true;
    }

    /// <summary>Searches the singular points of the curve.</summary>
    public void Init()
    {
        int i, j;
        GeomFill_SnglrFunc Func = new GeomFill_SnglrFunc(myCurve!);
        const double TolF = 1.0e-10;
        const double Tol = 10 * TolF;
        const double Tol2 = Tol * Tol;
        double PTol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        // We want to determine if the curve has linear segments
        int NbIntC2 = myCurve!.NbIntervals(C2);
        NCollection_HArray1<double> myC2Disc = new NCollection_HArray1<double>(1, NbIntC2 + 1);
        NCollection_HArray1<bool> IsLin = new NCollection_HArray1<bool>(1, NbIntC2);
        NCollection_HArray1<bool> IsConst = new NCollection_HArray1<bool>(1, NbIntC2);
        NCollection_Array1<double> AveFunc = new NCollection_Array1<double>(1, NbIntC2);
        myCurve.Intervals(myC2Disc.ChangeArray1(), C2);
        int NbControl = 10;
        double Step, Average = 0, modulus;
        Point3D C, C1 = new Point3D();
        for (i = 1; i <= NbIntC2; i++)
        {
            Step = (myC2Disc.Value(i + 1) - myC2Disc.Value(i)) / NbControl;
            IsLin[i] = true;
            IsConst[i] = true;
            for (j = 1; j <= NbControl; j++)
            {
                Func.D0(myC2Disc.Value(i) + (j - 1) * Step, out C);
                if (j == 1)
                {
                    C1 = C;
                }
                modulus = C.XYZ().Modulus();
                if (modulus > Tol)
                {
                    IsLin[i] = false;
                }
                Average += modulus;

                if (IsConst.Value(i))
                {
                    if (Math.Abs(C.X() - C1.X()) > Tol || Math.Abs(C.Y() - C1.Y()) > Tol
                        || Math.Abs(C.Z() - C1.Z()) > Tol)
                    {
                        IsConst[i] = false;
                    }
                }
            }
            AveFunc[i] = Average / NbControl;
        }

        // Here we are looking for singularities
        NCollection_Sequence<double>[] SeqArray = new NCollection_Sequence<double>[NbIntC2];
        for (i = 0; i < NbIntC2; i++)
        {
            SeqArray[i] = new NCollection_Sequence<double>();
        }
        NCollection_Sequence<double> SnglSeq = new NCollection_Sequence<double>();
        double Value2, t;
        Extrema_ExtPC Ext = new Extrema_ExtPC();
        Point3D Origin = new Point3D(0, 0, 0);

        for (i = 1; i <= NbIntC2; i++)
        {
            if (!IsLin.Value(i) && !IsConst.Value(i))
            {
                Func.SetRatio(1.0 / AveFunc[i]); // Normalization
                Ext.Initialize(Func, myC2Disc.Value(i), myC2Disc.Value(i + 1), TolF);
                Ext.Perform(Origin);
                if (Ext.IsDone() && Ext.NbExt() != 0)
                {
                    for (j = 1; j <= Ext.NbExt(); j++)
                    {
                        Value2 = Ext.SquareDistance(j);
                        if (Value2 < Tol2)
                        {
                            t = Ext.Point(j).Parameter();
                            SeqArray[i - 1].Append(t);
                        }
                    }
                }
                // sorting
                if (SeqArray[i - 1].Length() != 0)
                {
                    NCollection_Array1<double> anArray = new NCollection_Array1<double>(1, SeqArray[i - 1].Length());
                    for (j = 1; j <= anArray.Length(); j++)
                    {
                        anArray[j] = SeqArray[i - 1][j];
                    }
                    // std::sort of reals: only equal values tie
                    anArray.AsSpan().Sort();
                    for (j = 1; j <= anArray.Length(); j++)
                    {
                        SeqArray[i - 1][j] = anArray[j];
                    }
                }
            }
        }
        // Filling SnglSeq by first sets of roots
        for (i = 0; i < NbIntC2; i++)
        {
            for (j = 1; j <= SeqArray[i].Length(); j++)
            {
                SnglSeq.Append(SeqArray[i][j]);
            }
        }

        // Extrema works bad, need to pass second time
        for (i = 0; i < NbIntC2; i++)
        {
            if (!SeqArray[i].IsEmpty())
            {
                SeqArray[i].Prepend(myC2Disc.Value(i + 1));
                SeqArray[i].Append(myC2Disc.Value(i + 2));
                Func.SetRatio(1.0 / AveFunc[i + 1]); // Normalization
                for (j = 1; j < SeqArray[i].Length(); j++)
                {
                    if (SeqArray[i][j + 1] - SeqArray[i][j] > PTol)
                    {
                        Ext.Initialize(Func, SeqArray[i][j], SeqArray[i][j + 1], TolF);
                        Ext.Perform(Origin);
                        if (Ext.IsDone())
                        {
                            for (int k = 1; k <= Ext.NbExt(); k++)
                            {
                                Value2 = Ext.SquareDistance(k);
                                if (Value2 < Tol2)
                                {
                                    t = Ext.Point(k).Parameter();
                                    if (t - SeqArray[i][j] > PTol && SeqArray[i][j + 1] - t > PTol)
                                    {
                                        SnglSeq.Append(t);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if (SnglSeq.Length() > 0)
        {
            // sorting
            NCollection_Array1<double> anArray = new NCollection_Array1<double>(1, SnglSeq.Length());
            for (i = 1; i <= SnglSeq.Length(); i++)
            {
                anArray[i] = SnglSeq[i];
            }
            anArray.AsSpan().Sort();
            for (i = 1; i <= SnglSeq.Length(); i++)
            {
                SnglSeq[i] = anArray[i];
            }

            // discard repeating elements
            bool found = true;
            j = 1;
            while (found)
            {
                found = false;
                for (i = j; i < SnglSeq.Length(); i++)
                {
                    if (SnglSeq[i + 1] - SnglSeq[i] <= PTol)
                    {
                        SnglSeq.Remove(i + 1);
                        j = i;
                        found = true;
                        break;
                    }
                }
            }

            mySngl = new NCollection_HArray1<double>(1, SnglSeq.Length());
            for (i = 1; i <= mySngl.Length(); i++)
            {
                mySngl[i] = SnglSeq[i];
            }

            // computation of length of singular interval
            mySnglLen = new NCollection_HArray1<double>(1, mySngl.Length());
            Vector3D SnglDer, SnglDer2;
            double norm;
            for (i = 1; i <= mySngl.Length(); i++)
            {
                Func.D2(mySngl.Value(i), out C, out SnglDer, out SnglDer2);
                if ((norm = SnglDer.Magnitude()) > Constants.Resolution())
                {
                    mySnglLen[i] = Min(NullTol / norm, MaxSingular);
                }
                else if ((norm = SnglDer2.Magnitude()) > Constants.Resolution())
                {
                    mySnglLen[i] = Min(Math.Sqrt(2 * NullTol / norm), MaxSingular);
                }
                else
                {
                    mySnglLen[i] = MaxSingular;
                }
            }

            if (mySngl.Length() > 1)
            {
                // we have to merge singular points that have common parts of singular intervals
                NCollection_Sequence<Point2D> tmpSeq = new NCollection_Sequence<Point2D>();
                tmpSeq.Append(new Point2D(mySngl.Value(1), mySnglLen.Value(1)));
                double U11, U12, U21, U22;
                for (i = 2; i <= mySngl.Length(); i++)
                {
                    U12 = tmpSeq.Last().X() + tmpSeq.Last().Y();
                    U21 = mySngl.Value(i) - mySnglLen.Value(i);
                    if (U12 >= U21)
                    {
                        U11 = tmpSeq.Last().X() - tmpSeq.Last().Y();
                        U22 = mySngl.Value(i) + mySnglLen.Value(i);
                        tmpSeq[tmpSeq.Length()] = new Point2D((U11 + U22) / 2, (U22 - U11) / 2);
                    }
                    else
                    {
                        tmpSeq.Append(new Point2D(mySngl.Value(i), mySnglLen.Value(i)));
                    }
                }
                mySngl = new NCollection_HArray1<double>(1, tmpSeq.Length());
                mySnglLen = new NCollection_HArray1<double>(1, tmpSeq.Length());
                for (i = 1; i <= mySngl.Length(); i++)
                {
                    mySngl[i] = tmpSeq[i].X();
                    mySnglLen[i] = tmpSeq[i].Y();
                }
            }
            isSngl = true;
        }
        else
        {
            isSngl = false;
        }
    }

    /// <summary>
    /// Revolves the trihedron (Tangent, Normal, BiNormal) so that Tangent coincides with NewTangent.
    /// </summary>
    private bool RotateTrihedron(ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal, in Vector3D NewTangent)
    {
        double anInfCOS = Math.Cos(global::Kerf.Engine.Foundation.Precision.Angular()); // 0.99999995
        double aTol = Constants.Resolution();

        Vector3D anAxis = Tangent.Crossed(NewTangent);

        double NT = anAxis.Magnitude();
        if (NT <= aTol)
        {
            // No rotation required
            return true;
        }
        else
        {
            anAxis /= NT; // Normalization
        }

        double aPx = anAxis.X(), aPy = anAxis.Y(), aPz = anAxis.Z();
        double aCAng = CosAngle(Tangent, NewTangent); // cosine
        double anAddCAng = 1.0 - aCAng;
        double aSAng = Math.Sqrt(1.0 - aCAng * aCAng); // sine

        // According to rotate direction, sine of rotation angle might be positive or negative: rotate "Tangent"
        // in both directions, then choose the direction from the results.
        Vector3D aV11 = new Vector3D(anAddCAng * aPx * aPx + aCAng,
                                 anAddCAng * aPx * aPy - aPz * aSAng,
                                 anAddCAng * aPx * aPz + aPy * aSAng);
        Vector3D aV12 = new Vector3D(anAddCAng * aPx * aPx + aCAng,
                                 anAddCAng * aPx * aPy + aPz * aSAng,
                                 anAddCAng * aPx * aPz - aPy * aSAng);
        Vector3D aV21 = new Vector3D(anAddCAng * aPx * aPy + aPz * aSAng,
                                 anAddCAng * aPy * aPy + aCAng,
                                 anAddCAng * aPy * aPz - aPx * aSAng);
        Vector3D aV22 = new Vector3D(anAddCAng * aPx * aPy - aPz * aSAng,
                                 anAddCAng * aPy * aPy + aCAng,
                                 anAddCAng * aPy * aPz + aPx * aSAng);
        Vector3D aV31 = new Vector3D(anAddCAng * aPx * aPz - aPy * aSAng,
                                 anAddCAng * aPy * aPz + aPx * aSAng,
                                 anAddCAng * aPz * aPz + aCAng);
        Vector3D aV32 = new Vector3D(anAddCAng * aPx * aPz + aPy * aSAng,
                                 anAddCAng * aPy * aPz - aPx * aSAng,
                                 anAddCAng * aPz * aPz + aCAng);

        Vector3D aT1 = new Vector3D(Tangent.Dot(aV11), Tangent.Dot(aV21), Tangent.Dot(aV31));
        Vector3D aT2 = new Vector3D(Tangent.Dot(aV12), Tangent.Dot(aV22), Tangent.Dot(aV32));

        if (CosAngle(aT1, NewTangent) >= CosAngle(aT2, NewTangent))
        {
            Tangent = aT1;
            Normal = new Vector3D(Normal.Dot(aV11), Normal.Dot(aV21), Normal.Dot(aV31));
            BiNormal = new Vector3D(BiNormal.Dot(aV11), BiNormal.Dot(aV21), BiNormal.Dot(aV31));
        }
        else
        {
            Tangent = aT2;
            Normal = new Vector3D(Normal.Dot(aV12), Normal.Dot(aV22), Normal.Dot(aV32));
            BiNormal = new Vector3D(BiNormal.Dot(aV12), BiNormal.Dot(aV22), BiNormal.Dot(aV32));
        }

        return CosAngle(Tangent, NewTangent) >= anInfCOS;
    }

    public override bool D0(double theParam, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        double aTol = Constants.Resolution();

        double norm;
        double Delta = 0.0;
        if (IsSingular(theParam, out int Index))
        {
            if (SingularD0(theParam, Index, ref Tangent, ref Normal, ref BiNormal, ref Delta))
            {
                return true;
            }
        }

        double aParam = theParam + Delta;
        myTrimmed!.D2(aParam, out P, out Tangent, out BiNormal);

        const double DivisionFactor = 1.0e-3;
        double anUinfium = myTrimmed.FirstParameter();
        double anUsupremum = myTrimmed.LastParameter();
        double aDelta = (anUsupremum - anUinfium) * DivisionFactor;
        double Ndu = Tangent.Magnitude();

        if (Ndu <= aTol)
        {
            Vector3D aTn;
            // Derivative is approximated by Taylor-series
            int anIndex = 1; // Derivative order
            bool isDeriveFound = false;

            do
            {
                aTn = myTrimmed.DN(theParam, ++anIndex);
                Ndu = aTn.Magnitude();
                isDeriveFound = Ndu > aTol;
            } while (!isDeriveFound && anIndex < maxDerivOrder);

            if (isDeriveFound)
            {
                double u;

                if (theParam - anUinfium < aDelta)
                {
                    u = theParam + aDelta;
                }
                else
                {
                    u = theParam - aDelta;
                }

                myTrimmed.D0(Min(theParam, u), out Point3D P1);
                myTrimmed.D0(Max(theParam, u), out Point3D P2);

                Vector3D V1 = new Vector3D(P1, P2);
                double aDirFactor = aTn.Dot(V1);

                if (aDirFactor < 0.0)
                {
                    aTn = -aTn;
                }
            } // if(IsDeriveFound)
            else
            {
                // Derivative is approximated by three points
                Point3D Ptemp = new Point3D(0.0, 0.0, 0.0); //(0,0,0)-coordinate
                Point3D P1, P2, P3;
                bool IsParameterGrown;

                if (theParam - anUinfium < 2 * aDelta)
                {
                    myTrimmed.D0(theParam, out P1);
                    myTrimmed.D0(theParam + aDelta, out P2);
                    myTrimmed.D0(theParam + 2 * aDelta, out P3);
                    IsParameterGrown = true;
                }
                else
                {
                    myTrimmed.D0(theParam - 2 * aDelta, out P1);
                    myTrimmed.D0(theParam - aDelta, out P2);
                    myTrimmed.D0(theParam, out P3);
                    IsParameterGrown = false;
                }

                Vector3D V1 = new Vector3D(Ptemp, P1), V2 = new Vector3D(Ptemp, P2), V3 = new Vector3D(Ptemp, P3);

                if (IsParameterGrown)
                {
                    aTn = -3 * V1 + 4 * V2 - V3;
                }
                else
                {
                    aTn = V1 - 4 * V2 + 3 * V3;
                }
            } // else of "if(IsDeriveFound)" condition

            Ndu = aTn.Magnitude();
            Point3D Pt = P;

            double dPar = 10.0 * aDelta;
            // Recursive calling is used for determine of trihedron for
            // point, which is near to given.
            if (theParam - anUinfium < dPar)
            {
                if (!D0(aParam + dPar, ref Tangent, ref Normal, ref BiNormal))
                {
                    return false;
                }
            }
            else
            {
                if (!D0(aParam - dPar, ref Tangent, ref Normal, ref BiNormal))
                {
                    return false;
                }
            }

            P = Pt;

            if (!RotateTrihedron(ref Tangent, ref Normal, ref BiNormal, aTn))
            {
                return false;
            }
        } // if(Ndu <= aTol)
        else
        {
            Tangent = Tangent / Ndu;
            BiNormal = Tangent.Crossed(BiNormal);

            norm = BiNormal.Magnitude();
            if (norm <= Constants.Resolution())
            {
                Frame3D Axe = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Tangent));
                BiNormal.SetXYZ(Axe.YDirection().XYZ());
            }
            else
            {
                BiNormal.Normalize();
            }

            Normal = BiNormal;
            Normal.Cross(Tangent);
        }

        return true;
    }

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        double Delta = 0.0;
        if (IsSingular(Param, out int Index))
        {
            if (SingularD1(Param, Index, ref Tangent, ref DTangent, ref Normal, ref DNormal, ref BiNormal, ref DBiNormal,
                           ref Delta))
            {
                return true;
            }
        }

        double theParam = Param + Delta;
        myTrimmed!.D3(theParam, out P, out Vector3D DC1, out Vector3D DC2, out Vector3D DC3);
        Tangent = DC1.Normalized();

        if (Tangent.Crossed(DC2).Magnitude() <= Constants.Resolution())
        {
            Frame3D Axe = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Tangent));
            Normal.SetXYZ(Axe.XDirection().XYZ());
            BiNormal.SetXYZ(Axe.YDirection().XYZ());
            DTangent.SetCoord(0, 0, 0);
            DNormal.SetCoord(0, 0, 0);
            DBiNormal.SetCoord(0, 0, 0);
            return true;
        }
        else
        {
            BiNormal = Tangent.Crossed(DC2).Normalized();
        }

        Normal = BiNormal.Crossed(Tangent);

        DTangent = FDeriv(DC1, DC2);

        Vector3D instead_DC1, instead_DC2;
        instead_DC1 = Tangent.Crossed(DC2);
        instead_DC2 = DTangent.Crossed(DC2) + Tangent.Crossed(DC3);

        DBiNormal = FDeriv(instead_DC1, instead_DC2);

        DNormal = DBiNormal.Crossed(Tangent) + BiNormal.Crossed(DTangent);
        return true;
    }

    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
    {
        double Delta = 0.0;
        if (IsSingular(Param, out int Index))
        {
            if (SingularD2(Param, Index, ref Tangent, ref DTangent, ref D2Tangent, ref Normal, ref DNormal, ref D2Normal,
                           ref BiNormal, ref DBiNormal, ref D2BiNormal, ref Delta))
            {
                return true;
            }
        }

        double theParam = Param + Delta;
        myTrimmed!.D3(theParam, out P, out Vector3D DC1, out Vector3D DC2, out Vector3D DC3);
        Vector3D DC4 = myTrimmed.DN(theParam, 4);

        Tangent = DC1.Normalized();

        if (Tangent.Crossed(DC2).Magnitude() <= Constants.Resolution())
        {
            Frame3D Axe = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Tangent));
            Normal.SetXYZ(Axe.XDirection().XYZ());
            BiNormal.SetXYZ(Axe.YDirection().XYZ());
            DTangent.SetCoord(0, 0, 0);
            DNormal.SetCoord(0, 0, 0);
            DBiNormal.SetCoord(0, 0, 0);
            D2Tangent.SetCoord(0, 0, 0);
            D2Normal.SetCoord(0, 0, 0);
            D2BiNormal.SetCoord(0, 0, 0);
            return true;
        }
        else
        {
            BiNormal = Tangent.Crossed(DC2).Normalized();
        }

        Normal = BiNormal.Crossed(Tangent);

        DTangent = FDeriv(DC1, DC2);
        D2Tangent = DDeriv(DC1, DC2, DC3);

        Vector3D instead_DC1, instead_DC2, instead_DC3;
        instead_DC1 = Tangent.Crossed(DC2);
        instead_DC2 = DTangent.Crossed(DC2) + Tangent.Crossed(DC3);
        instead_DC3 = D2Tangent.Crossed(DC2) + 2 * DTangent.Crossed(DC3) + Tangent.Crossed(DC4);

        DBiNormal = FDeriv(instead_DC1, instead_DC2);
        D2BiNormal = DDeriv(instead_DC1, instead_DC2, instead_DC3);

        DNormal = DBiNormal.Crossed(Tangent) + BiNormal.Crossed(DTangent);

        D2Normal = D2BiNormal.Crossed(Tangent) + 2 * DBiNormal.Crossed(DTangent) + BiNormal.Crossed(D2Tangent);

        return true;
    }

    private static Continuity CurveShape(Continuity S)
    {
        switch (S)
        {
            case C0:
                return C2;
            case C1:
                return C3;
            case C2:
            case C3:
            case CN:
                return CN;
            default:
                throw new OutOfRangeException();
        }
    }

    public override int NbIntervals(Continuity S)
    {
        Continuity tmpS = CurveShape(S);
        int NbTrimmed;

        NbTrimmed = myCurve!.NbIntervals(tmpS);

        if (!isSngl)
        {
            return NbTrimmed;
        }

        NCollection_Array1<double> TrimInt = new NCollection_Array1<double>(1, NbTrimmed + 1);
        myCurve.Intervals(TrimInt, tmpS);

        NCollection_Sequence<double> Fusion = new NCollection_Sequence<double>();
        GeomLib.FuseIntervals(TrimInt, mySngl!.Array1(), Fusion, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

        return Fusion.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        Continuity tmpS = CurveShape(S);
        int NbTrimmed;

        if (!isSngl)
        {
            myCurve!.Intervals(T, tmpS);
            return;
        }

        NbTrimmed = myCurve!.NbIntervals(tmpS);
        NCollection_Array1<double> TrimInt = new NCollection_Array1<double>(1, NbTrimmed + 1);
        myCurve.Intervals(TrimInt, tmpS);

        NCollection_Sequence<double> Fusion = new NCollection_Sequence<double>();
        GeomLib.FuseIntervals(TrimInt, mySngl!.Array1(), Fusion, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

        for (int i = 1; i <= Fusion.Length(); i++)
        {
            T[i] = Fusion.Value(i);
        }
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

    private bool IsSingular(double U, out int Index)
    {
        Index = 0; // OCCT leaves it unwritten when false; callers then don't read it
        int i;
        if (!isSngl)
        {
            return false;
        }
        for (i = 1; i <= mySngl!.Length(); i++)
        {
            if (Math.Abs(U - mySngl.Value(i)) < mySnglLen!.Value(i))
            {
                Index = i;
                return true;
            }
        }
        return false;
    }

    private bool DoSingular(double U, int Index, ref Vector3D Tangent, ref Vector3D BiNormal, ref int n, ref int k,
                            ref int TFlag, ref int BNFlag, ref double Delta)
    {
        int i, MaxN = 20;
        Delta = 0.0;
        double h;
        h = 2 * mySnglLen!.Value(Index);

        Vector3D T = new Vector3D(), N = new Vector3D(), BN = new Vector3D();
        TFlag = 1;
        BNFlag = 1;
        GetInterval(out double A, out double B);
        if (U >= (A + B) / 2)
        {
            h = -h;
        }
        for (i = 1; i <= MaxN; i++)
        {
            Tangent = myTrimmed!.DN(U, i);
            if (Tangent.Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                break;
            }
        }
        if (i > MaxN)
        {
            return false;
        }
        Tangent.Normalize();
        n = i;

        i++;
        for (; i <= MaxN; i++)
        {
            BiNormal = Tangent.Crossed(myTrimmed!.DN(U, i));
            double magn = BiNormal.Magnitude();
            if (magn > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                // modified by jgv, 12.08.03 for OCC605 ////
                Vector3D NextBiNormal = Tangent.Crossed(myTrimmed.DN(U, i + 1));
                if (NextBiNormal.Magnitude() > magn)
                {
                    i++;
                    BiNormal = NextBiNormal;
                }
                break;
            }
        }
        if (i > MaxN)
        {
            Delta = h;
            return false;
        }
        BiNormal.Normalize();
        k = i;

        D0(U + h, ref T, ref N, ref BN);

        if (Tangent.Angle(T) > M_PI / 2)
        {
            TFlag = -1;
        }
        if (BiNormal.Angle(BN) > M_PI / 2)
        {
            BNFlag = -1;
        }

        return true;
    }

    private bool SingularD0(double Param, int Index, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal,
                            ref double Delta)
    {
        int n = 0, k = 0, TFlag = 0, BNFlag = 0;
        if (!DoSingular(Param, Index, ref Tangent, ref BiNormal, ref n, ref k, ref TFlag, ref BNFlag, ref Delta))
        {
            return false;
        }

        Tangent *= TFlag;
        BiNormal *= BNFlag;
        Normal = BiNormal;
        Normal.Cross(Tangent);

        return true;
    }

    private bool SingularD1(double Param, int Index, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D BiNormal, ref Vector3D DBiNormal, ref double Delta)
    {
        int n = 0, k = 0, TFlag = 0, BNFlag = 0;
        if (!DoSingular(Param, Index, ref Tangent, ref BiNormal, ref n, ref k, ref TFlag, ref BNFlag, ref Delta))
        {
            return false;
        }

        Vector3D F, DF, Dtmp;
        F = myTrimmed!.DN(Param, n);
        DF = myTrimmed.DN(Param, n + 1);
        DTangent = FDeriv(F, DF);

        Dtmp = myTrimmed.DN(Param, k);
        F = Tangent.Crossed(Dtmp);
        DF = DTangent.Crossed(Dtmp) + Tangent.Crossed(myTrimmed.DN(Param, k + 1));
        DBiNormal = FDeriv(F, DF);

        if (TFlag < 0)
        {
            Tangent = -Tangent;
            DTangent = -DTangent;
        }

        if (BNFlag < 0)
        {
            BiNormal = -BiNormal;
            DBiNormal = -DBiNormal;
        }

        Normal = BiNormal.Crossed(Tangent);
        DNormal = DBiNormal.Crossed(Tangent) + BiNormal.Crossed(DTangent);

        return true;
    }

    private bool SingularD2(double Param, int Index, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent,
                            ref Vector3D Normal, ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal,
                            ref Vector3D DBiNormal, ref Vector3D D2BiNormal, ref double Delta)
    {
        int n = 0, k = 0, TFlag = 0, BNFlag = 0;
        if (!DoSingular(Param, Index, ref Tangent, ref BiNormal, ref n, ref k, ref TFlag, ref BNFlag, ref Delta))
        {
            return false;
        }

        Vector3D F, DF, D2F, Dtmp1, Dtmp2;
        F = myTrimmed!.DN(Param, n);
        DF = myTrimmed.DN(Param, n + 1);
        D2F = myTrimmed.DN(Param, n + 2);
        DTangent = FDeriv(F, DF);
        D2Tangent = DDeriv(F, DF, D2F);

        Dtmp1 = myTrimmed.DN(Param, k);
        Dtmp2 = myTrimmed.DN(Param, k + 1);
        F = Tangent.Crossed(Dtmp1);
        DF = DTangent.Crossed(Dtmp1) + Tangent.Crossed(Dtmp2);
        D2F = D2Tangent.Crossed(Dtmp1) + 2 * DTangent.Crossed(Dtmp2) + Tangent.Crossed(myTrimmed.DN(Param, k + 2));
        DBiNormal = FDeriv(F, DF);
        D2BiNormal = DDeriv(F, DF, D2F);

        if (TFlag < 0)
        {
            Tangent = -Tangent;
            DTangent = -DTangent;
            D2Tangent = -D2Tangent;
        }

        if (BNFlag < 0)
        {
            BiNormal = -BiNormal;
            DBiNormal = -DBiNormal;
            D2BiNormal = -D2BiNormal;
        }

        Normal = BiNormal.Crossed(Tangent);
        DNormal = DBiNormal.Crossed(Tangent) + BiNormal.Crossed(DTangent);
        D2Normal = D2BiNormal.Crossed(Tangent) + 2 * DBiNormal.Crossed(DTangent) + BiNormal.Crossed(D2Tangent);

        return true;
    }
}
