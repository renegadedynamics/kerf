// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Frenet.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Frenet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>The Frenet trihedron law: the tangent, normal and binormal of the curve.</summary>
/// <remarks>
/// <see cref="SetCurve"/> searches the curve's singular points (where C' ^ C'' vanishes) unless it is a line or a
/// conic; near them the trihedron is computed from higher derivatives.
/// </remarks>
public class Frenet : TrihedronLaw
{
    private const double NullTol = 1.0e-10;
    private const double MaxSingular = 1.0e-5;
    private const int MaxDerivOrder = 3;

    private Point3D _p;
    private NCollection_HArray1<double>? _sngl;
    private NCollection_HArray1<double>? _snglLen;
    private bool _isSngl;

    /// <summary>Computes (F/|F|)'.</summary>
    private static Vector3D FDeriv(in Vector3D f, in Vector3D df)
    {
        double norma = f.Magnitude();
        Vector3D result = (df - f * (f * df) / (norma * norma)) / norma;
        return result;
    }

    /// <summary>Computes (F/|F|)''.</summary>
    private static Vector3D DDeriv(in Vector3D f, in Vector3D df, in Vector3D d2F)
    {
        double norma = f.Magnitude();
        Vector3D result = (d2F - 2 * df * (f * df) / (norma * norma)) / norma
                        - f * ((df.SquareMagnitude() + f * d2F - 3 * (f * df) * (f * df) / (norma * norma))
                               / (norma * norma * norma));
        return result;
    }

    /// <summary>The cosine of the angle between theV1 and theV2.</summary>
    private static double CosAngle(in Vector3D v1, in Vector3D v2)
    {
        double tol = Constants.Resolution();
        double m1 = v1.Magnitude(), m2 = v2.Magnitude();
        if ((m1 <= tol) || (m2 <= tol))
        { // Vectors are codirectional
            return 1.0;
        }
        double cAng = v1.Dot(v2) / (m1 * m2);
        if (cAng > 1.0)
        {
            cAng = 1.0;
        }
        if (cAng < -1.0)
        {
            cAng = -1.0;
        }
        return cAng;
    }

    public Frenet()
    {
        _isSngl = false;
    }

    public override TrihedronLaw Copy()
    {
        Frenet copy = new Frenet();
        if (_curve is not null)
        {
            copy.SetCurve(_curve);
        }
        return copy;
    }

    /// <summary>Initializes the curve of the Frenet law; returns true.</summary>
    public override bool SetCurve(CurveAdaptorBase c)
    {
        base.SetCurve(c);
        if (c is not null)
        {
            CurveKind type;
            type = c.GetType();
            switch (type)
            {
                case Circle:
                case Ellipse:
                case Hyperbola:
                case Parabola:
                case Line:
                {
                    // No problem
                    _isSngl = false;
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
        SnglrFunc func = new SnglrFunc(_curve!);
        const double tolF = 1.0e-10;
        const double tol = 10 * tolF;
        const double tol2 = tol * tol;
        double pTol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        // We want to determine if the curve has linear segments
        int nbIntC2 = _curve!.NbIntervals(C2);
        NCollection_HArray1<double> myC2Disc = new NCollection_HArray1<double>(1, nbIntC2 + 1);
        NCollection_HArray1<bool> isLin = new NCollection_HArray1<bool>(1, nbIntC2);
        NCollection_HArray1<bool> isConst = new NCollection_HArray1<bool>(1, nbIntC2);
        NCollection_Array1<double> aveFunc = new NCollection_Array1<double>(1, nbIntC2);
        _curve.Intervals(myC2Disc.ChangeArray1(), C2);
        int nbControl = 10;
        double step, average = 0, modulus;
        Point3D c, c1 = new Point3D();
        for (i = 1; i <= nbIntC2; i++)
        {
            step = (myC2Disc.Value(i + 1) - myC2Disc.Value(i)) / nbControl;
            isLin[i] = true;
            isConst[i] = true;
            for (j = 1; j <= nbControl; j++)
            {
                func.D0(myC2Disc.Value(i) + (j - 1) * step, out c);
                if (j == 1)
                {
                    c1 = c;
                }
                modulus = c.XYZ().Modulus();
                if (modulus > tol)
                {
                    isLin[i] = false;
                }
                average += modulus;

                if (isConst.Value(i))
                {
                    if (Math.Abs(c.X() - c1.X()) > tol || Math.Abs(c.Y() - c1.Y()) > tol
                        || Math.Abs(c.Z() - c1.Z()) > tol)
                    {
                        isConst[i] = false;
                    }
                }
            }
            aveFunc[i] = average / nbControl;
        }

        // Here we are looking for singularities
        NCollection_Sequence<double>[] seqArray = new NCollection_Sequence<double>[nbIntC2];
        for (i = 0; i < nbIntC2; i++)
        {
            seqArray[i] = new NCollection_Sequence<double>();
        }
        NCollection_Sequence<double> snglSeq = new NCollection_Sequence<double>();
        double value2, t;
        CurvePointExtrema ext = new CurvePointExtrema();
        Point3D origin = new Point3D(0, 0, 0);

        for (i = 1; i <= nbIntC2; i++)
        {
            if (!isLin.Value(i) && !isConst.Value(i))
            {
                func.SetRatio(1.0 / aveFunc[i]); // Normalization
                ext.Initialize(func, myC2Disc.Value(i), myC2Disc.Value(i + 1), tolF);
                ext.Perform(origin);
                if (ext.IsDone && ext.ExtCount != 0)
                {
                    for (j = 1; j <= ext.ExtCount; j++)
                    {
                        value2 = ext.SquareDistance(j);
                        if (value2 < tol2)
                        {
                            t = ext.Point(j).Parameter();
                            seqArray[i - 1].Append(t);
                        }
                    }
                }
                // sorting
                if (seqArray[i - 1].Length() != 0)
                {
                    NCollection_Array1<double> array = new NCollection_Array1<double>(1, seqArray[i - 1].Length());
                    for (j = 1; j <= array.Length(); j++)
                    {
                        array[j] = seqArray[i - 1][j];
                    }
                    // std::sort of reals: only equal values tie
                    array.AsSpan().Sort();
                    for (j = 1; j <= array.Length(); j++)
                    {
                        seqArray[i - 1][j] = array[j];
                    }
                }
            }
        }
        // Filling SnglSeq by first sets of roots
        for (i = 0; i < nbIntC2; i++)
        {
            for (j = 1; j <= seqArray[i].Length(); j++)
            {
                snglSeq.Append(seqArray[i][j]);
            }
        }

        // Extrema works bad, need to pass second time
        for (i = 0; i < nbIntC2; i++)
        {
            if (!seqArray[i].IsEmpty())
            {
                seqArray[i].Prepend(myC2Disc.Value(i + 1));
                seqArray[i].Append(myC2Disc.Value(i + 2));
                func.SetRatio(1.0 / aveFunc[i + 1]); // Normalization
                for (j = 1; j < seqArray[i].Length(); j++)
                {
                    if (seqArray[i][j + 1] - seqArray[i][j] > pTol)
                    {
                        ext.Initialize(func, seqArray[i][j], seqArray[i][j + 1], tolF);
                        ext.Perform(origin);
                        if (ext.IsDone)
                        {
                            for (int k = 1; k <= ext.ExtCount; k++)
                            {
                                value2 = ext.SquareDistance(k);
                                if (value2 < tol2)
                                {
                                    t = ext.Point(k).Parameter();
                                    if (t - seqArray[i][j] > pTol && seqArray[i][j + 1] - t > pTol)
                                    {
                                        snglSeq.Append(t);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if (snglSeq.Length() > 0)
        {
            // sorting
            NCollection_Array1<double> array = new NCollection_Array1<double>(1, snglSeq.Length());
            for (i = 1; i <= snglSeq.Length(); i++)
            {
                array[i] = snglSeq[i];
            }
            array.AsSpan().Sort();
            for (i = 1; i <= snglSeq.Length(); i++)
            {
                snglSeq[i] = array[i];
            }

            // discard repeating elements
            bool found = true;
            j = 1;
            while (found)
            {
                found = false;
                for (i = j; i < snglSeq.Length(); i++)
                {
                    if (snglSeq[i + 1] - snglSeq[i] <= pTol)
                    {
                        snglSeq.Remove(i + 1);
                        j = i;
                        found = true;
                        break;
                    }
                }
            }

            _sngl = new NCollection_HArray1<double>(1, snglSeq.Length());
            for (i = 1; i <= _sngl.Length(); i++)
            {
                _sngl[i] = snglSeq[i];
            }

            // computation of length of singular interval
            _snglLen = new NCollection_HArray1<double>(1, _sngl.Length());
            Vector3D snglDer, snglDer2;
            double norm;
            for (i = 1; i <= _sngl.Length(); i++)
            {
                func.D2(_sngl.Value(i), out c, out snglDer, out snglDer2);
                if ((norm = snglDer.Magnitude()) > Constants.Resolution())
                {
                    _snglLen[i] = Min(NullTol / norm, MaxSingular);
                }
                else if ((norm = snglDer2.Magnitude()) > Constants.Resolution())
                {
                    _snglLen[i] = Min(Math.Sqrt(2 * NullTol / norm), MaxSingular);
                }
                else
                {
                    _snglLen[i] = MaxSingular;
                }
            }

            if (_sngl.Length() > 1)
            {
                // we have to merge singular points that have common parts of singular intervals
                NCollection_Sequence<Point2D> tmpSeq = new NCollection_Sequence<Point2D>();
                tmpSeq.Append(new Point2D(_sngl.Value(1), _snglLen.Value(1)));
                double u11, u12, u21, u22;
                for (i = 2; i <= _sngl.Length(); i++)
                {
                    u12 = tmpSeq.Last().X() + tmpSeq.Last().Y();
                    u21 = _sngl.Value(i) - _snglLen.Value(i);
                    if (u12 >= u21)
                    {
                        u11 = tmpSeq.Last().X() - tmpSeq.Last().Y();
                        u22 = _sngl.Value(i) + _snglLen.Value(i);
                        tmpSeq[tmpSeq.Length()] = new Point2D((u11 + u22) / 2, (u22 - u11) / 2);
                    }
                    else
                    {
                        tmpSeq.Append(new Point2D(_sngl.Value(i), _snglLen.Value(i)));
                    }
                }
                _sngl = new NCollection_HArray1<double>(1, tmpSeq.Length());
                _snglLen = new NCollection_HArray1<double>(1, tmpSeq.Length());
                for (i = 1; i <= _sngl.Length(); i++)
                {
                    _sngl[i] = tmpSeq[i].X();
                    _snglLen[i] = tmpSeq[i].Y();
                }
            }
            _isSngl = true;
        }
        else
        {
            _isSngl = false;
        }
    }

    /// <summary>
    /// Revolves the trihedron (Tangent, Normal, BiNormal) so that Tangent coincides with NewTangent.
    /// </summary>
    private bool RotateTrihedron(ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal, in Vector3D newTangent)
    {
        double infCOS = Math.Cos(global::Kerf.Engine.Foundation.Precision.Angular()); // 0.99999995
        double tol = Constants.Resolution();

        Vector3D axis = tangent.Crossed(newTangent);

        double nt = axis.Magnitude();
        if (nt <= tol)
        {
            // No rotation required
            return true;
        }
        else
        {
            axis /= nt; // Normalization
        }

        double px = axis.X(), py = axis.Y(), pz = axis.Z();
        double cAng = CosAngle(tangent, newTangent); // cosine
        double addCAng = 1.0 - cAng;
        double sAng = Math.Sqrt(1.0 - cAng * cAng); // sine

        // According to rotate direction, sine of rotation angle might be positive or negative: rotate "Tangent"
        // in both directions, then choose the direction from the results.
        Vector3D v11 = new Vector3D(addCAng * px * px + cAng,
                                 addCAng * px * py - pz * sAng,
                                 addCAng * px * pz + py * sAng);
        Vector3D v12 = new Vector3D(addCAng * px * px + cAng,
                                 addCAng * px * py + pz * sAng,
                                 addCAng * px * pz - py * sAng);
        Vector3D v21 = new Vector3D(addCAng * px * py + pz * sAng,
                                 addCAng * py * py + cAng,
                                 addCAng * py * pz - px * sAng);
        Vector3D v22 = new Vector3D(addCAng * px * py - pz * sAng,
                                 addCAng * py * py + cAng,
                                 addCAng * py * pz + px * sAng);
        Vector3D v31 = new Vector3D(addCAng * px * pz - py * sAng,
                                 addCAng * py * pz + px * sAng,
                                 addCAng * pz * pz + cAng);
        Vector3D v32 = new Vector3D(addCAng * px * pz + py * sAng,
                                 addCAng * py * pz - px * sAng,
                                 addCAng * pz * pz + cAng);

        Vector3D t1 = new Vector3D(tangent.Dot(v11), tangent.Dot(v21), tangent.Dot(v31));
        Vector3D t2 = new Vector3D(tangent.Dot(v12), tangent.Dot(v22), tangent.Dot(v32));

        if (CosAngle(t1, newTangent) >= CosAngle(t2, newTangent))
        {
            tangent = t1;
            normal = new Vector3D(normal.Dot(v11), normal.Dot(v21), normal.Dot(v31));
            biNormal = new Vector3D(biNormal.Dot(v11), biNormal.Dot(v21), biNormal.Dot(v31));
        }
        else
        {
            tangent = t2;
            normal = new Vector3D(normal.Dot(v12), normal.Dot(v22), normal.Dot(v32));
            biNormal = new Vector3D(biNormal.Dot(v12), biNormal.Dot(v22), biNormal.Dot(v32));
        }

        return CosAngle(tangent, newTangent) >= infCOS;
    }

    public override bool D0(double param, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal)
    {
        double tol = Constants.Resolution();

        double norm;
        double Delta = 0.0;
        if (IsSingular(param, out int Index))
        {
            if (SingularD0(param, Index, ref tangent, ref normal, ref biNormal, ref Delta))
            {
                return true;
            }
        }

        double aParam = param + Delta;
        _trimmed!.D2(aParam, out _p, out tangent, out biNormal);

        const double divisionFactor = 1.0e-3;
        double uinfium = _trimmed.FirstParameter();
        double usupremum = _trimmed.LastParameter();
        double delta = (usupremum - uinfium) * divisionFactor;
        double ndu = tangent.Magnitude();

        if (ndu <= tol)
        {
            Vector3D tn;
            // Derivative is approximated by Taylor-series
            int index = 1; // Derivative order
            bool isDeriveFound = false;

            do
            {
                tn = _trimmed.DN(param, ++index);
                ndu = tn.Magnitude();
                isDeriveFound = ndu > tol;
            } while (!isDeriveFound && index < MaxDerivOrder);

            if (isDeriveFound)
            {
                double u;

                if (param - uinfium < delta)
                {
                    u = param + delta;
                }
                else
                {
                    u = param - delta;
                }

                _trimmed.D0(Min(param, u), out Point3D p1);
                _trimmed.D0(Max(param, u), out Point3D p2);

                Vector3D v1 = new Vector3D(p1, p2);
                double dirFactor = tn.Dot(v1);

                if (dirFactor < 0.0)
                {
                    tn = -tn;
                }
            } // if(IsDeriveFound)
            else
            {
                // Derivative is approximated by three points
                Point3D ptemp = new Point3D(0.0, 0.0, 0.0); //(0,0,0)-coordinate
                Point3D p1, p2, p3;
                bool isParameterGrown;

                if (param - uinfium < 2 * delta)
                {
                    _trimmed.D0(param, out p1);
                    _trimmed.D0(param + delta, out p2);
                    _trimmed.D0(param + 2 * delta, out p3);
                    isParameterGrown = true;
                }
                else
                {
                    _trimmed.D0(param - 2 * delta, out p1);
                    _trimmed.D0(param - delta, out p2);
                    _trimmed.D0(param, out p3);
                    isParameterGrown = false;
                }

                Vector3D v1 = new Vector3D(ptemp, p1), v2 = new Vector3D(ptemp, p2), v3 = new Vector3D(ptemp, p3);

                if (isParameterGrown)
                {
                    tn = -3 * v1 + 4 * v2 - v3;
                }
                else
                {
                    tn = v1 - 4 * v2 + 3 * v3;
                }
            } // else of "if(IsDeriveFound)" condition

            ndu = tn.Magnitude();
            Point3D pt = _p;

            double dPar = 10.0 * delta;
            // Recursive calling is used for determine of trihedron for
            // point, which is near to given.
            if (param - uinfium < dPar)
            {
                if (!D0(aParam + dPar, ref tangent, ref normal, ref biNormal))
                {
                    return false;
                }
            }
            else
            {
                if (!D0(aParam - dPar, ref tangent, ref normal, ref biNormal))
                {
                    return false;
                }
            }

            _p = pt;

            if (!RotateTrihedron(ref tangent, ref normal, ref biNormal, tn))
            {
                return false;
            }
        } // if(Ndu <= aTol)
        else
        {
            tangent = tangent / ndu;
            biNormal = tangent.Crossed(biNormal);

            norm = biNormal.Magnitude();
            if (norm <= Constants.Resolution())
            {
                Frame3D axe = new Frame3D(new Point3D(0, 0, 0), new Direction3D(tangent));
                biNormal = biNormal.WithXYZ(axe.YDirection().XYZ());
            }
            else
            {
                biNormal = biNormal.Normalized();
            }

            normal = biNormal;
            normal = normal.Crossed(tangent);
        }

        return true;
    }

    public override bool D1(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal, ref Vector3D dNormal,
                            ref Vector3D biNormal, ref Vector3D dBiNormal)
    {
        double delta = 0.0;
        if (IsSingular(param, out int index))
        {
            if (SingularD1(param, index, ref tangent, ref dTangent, ref normal, ref dNormal, ref biNormal, ref dBiNormal,
                           ref delta))
            {
                return true;
            }
        }

        double theParam = param + delta;
        _trimmed!.D3(theParam, out _p, out Vector3D dc1, out Vector3D dc2, out Vector3D dc3);
        tangent = dc1.Normalized();

        if (tangent.Crossed(dc2).Magnitude() <= Constants.Resolution())
        {
            Frame3D axe = new Frame3D(new Point3D(0, 0, 0), new Direction3D(tangent));
            normal = normal.WithXYZ(axe.XDirection().XYZ());
            biNormal = biNormal.WithXYZ(axe.YDirection().XYZ());
            dTangent = dTangent.WithCoord(0, 0, 0);
            dNormal = dNormal.WithCoord(0, 0, 0);
            dBiNormal = dBiNormal.WithCoord(0, 0, 0);
            return true;
        }
        else
        {
            biNormal = tangent.Crossed(dc2).Normalized();
        }

        normal = biNormal.Crossed(tangent);

        dTangent = FDeriv(dc1, dc2);

        Vector3D instead_DC1, instead_DC2;
        instead_DC1 = tangent.Crossed(dc2);
        instead_DC2 = dTangent.Crossed(dc2) + tangent.Crossed(dc3);

        dBiNormal = FDeriv(instead_DC1, instead_DC2);

        dNormal = dBiNormal.Crossed(tangent) + biNormal.Crossed(dTangent);
        return true;
    }

    public override bool D2(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal, ref Vector3D dBiNormal,
                            ref Vector3D d2BiNormal)
    {
        double delta = 0.0;
        if (IsSingular(param, out int index))
        {
            if (SingularD2(param, index, ref tangent, ref dTangent, ref d2Tangent, ref normal, ref dNormal, ref d2Normal,
                           ref biNormal, ref dBiNormal, ref d2BiNormal, ref delta))
            {
                return true;
            }
        }

        double theParam = param + delta;
        _trimmed!.D3(theParam, out _p, out Vector3D dc1, out Vector3D dc2, out Vector3D dc3);
        Vector3D dc4 = _trimmed.DN(theParam, 4);

        tangent = dc1.Normalized();

        if (tangent.Crossed(dc2).Magnitude() <= Constants.Resolution())
        {
            Frame3D axe = new Frame3D(new Point3D(0, 0, 0), new Direction3D(tangent));
            normal = normal.WithXYZ(axe.XDirection().XYZ());
            biNormal = biNormal.WithXYZ(axe.YDirection().XYZ());
            dTangent = dTangent.WithCoord(0, 0, 0);
            dNormal = dNormal.WithCoord(0, 0, 0);
            dBiNormal = dBiNormal.WithCoord(0, 0, 0);
            d2Tangent = d2Tangent.WithCoord(0, 0, 0);
            d2Normal = d2Normal.WithCoord(0, 0, 0);
            d2BiNormal = d2BiNormal.WithCoord(0, 0, 0);
            return true;
        }
        else
        {
            biNormal = tangent.Crossed(dc2).Normalized();
        }

        normal = biNormal.Crossed(tangent);

        dTangent = FDeriv(dc1, dc2);
        d2Tangent = DDeriv(dc1, dc2, dc3);

        Vector3D instead_DC1, instead_DC2, instead_DC3;
        instead_DC1 = tangent.Crossed(dc2);
        instead_DC2 = dTangent.Crossed(dc2) + tangent.Crossed(dc3);
        instead_DC3 = d2Tangent.Crossed(dc2) + 2 * dTangent.Crossed(dc3) + tangent.Crossed(dc4);

        dBiNormal = FDeriv(instead_DC1, instead_DC2);
        d2BiNormal = DDeriv(instead_DC1, instead_DC2, instead_DC3);

        dNormal = dBiNormal.Crossed(tangent) + biNormal.Crossed(dTangent);

        d2Normal = d2BiNormal.Crossed(tangent) + 2 * dBiNormal.Crossed(dTangent) + biNormal.Crossed(d2Tangent);

        return true;
    }

    private static Continuity CurveShape(Continuity s)
    {
        switch (s)
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

    public override int NbIntervals(Continuity s)
    {
        Continuity tmpS = CurveShape(s);
        int nbTrimmed;

        nbTrimmed = _curve!.NbIntervals(tmpS);

        if (!_isSngl)
        {
            return nbTrimmed;
        }

        NCollection_Array1<double> trimInt = new NCollection_Array1<double>(1, nbTrimmed + 1);
        _curve.Intervals(trimInt, tmpS);

        NCollection_Sequence<double> fusion = new NCollection_Sequence<double>();
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(trimInt, _sngl!.Array1(), fusion, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

        return fusion.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        Continuity tmpS = CurveShape(s);
        int nbTrimmed;

        if (!_isSngl)
        {
            _curve!.Intervals(t, tmpS);
            return;
        }

        nbTrimmed = _curve!.NbIntervals(tmpS);
        NCollection_Array1<double> trimInt = new NCollection_Array1<double>(1, nbTrimmed + 1);
        _curve.Intervals(trimInt, tmpS);

        NCollection_Sequence<double> fusion = new NCollection_Sequence<double>();
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(trimInt, _sngl!.Array1(), fusion, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

        for (int i = 1; i <= fusion.Length(); i++)
        {
            t[i] = fusion.Value(i);
        }
    }

    public override void GetAverageLaw(ref Vector3D aTangent, ref Vector3D aNormal, ref Vector3D aBiNormal)
    {
        int num = 20; // order of digitalization
        Vector3D t = new Vector3D(), n = new Vector3D(), bn = new Vector3D();
        aTangent = new Vector3D(0, 0, 0);
        aNormal = new Vector3D(0, 0, 0);
        aBiNormal = new Vector3D(0, 0, 0);
        double step = (_trimmed!.LastParameter() - _trimmed.FirstParameter()) / num;
        double param;
        for (int i = 0; i <= num; i++)
        {
            param = _trimmed.FirstParameter() + i * step;
            if (param > _trimmed.LastParameter())
            {
                param = _trimmed.LastParameter();
            }
            D0(param, ref t, ref n, ref bn);
            aTangent += t;
            aNormal += n;
            aBiNormal += bn;
        }
        aTangent /= num + 1;
        aNormal /= num + 1;

        aTangent = aTangent.Normalized();
        aBiNormal = aTangent.Crossed(aNormal).Normalized();
        aNormal = aBiNormal.Crossed(aTangent);
    }

    public override bool IsConstant => _curve!.GetType() == Line;

    /// <summary>Returns true.</summary>
    public override bool IsOnlyBy3dCurve => true;

    private bool IsSingular(double u, out int index)
    {
        index = 0; // OCCT leaves it unwritten when false; callers then don't read it
        int i;
        if (!_isSngl)
        {
            return false;
        }
        for (i = 1; i <= _sngl!.Length(); i++)
        {
            if (Math.Abs(u - _sngl.Value(i)) < _snglLen!.Value(i))
            {
                index = i;
                return true;
            }
        }
        return false;
    }

    private bool DoSingular(double u, int index, ref Vector3D tangent, ref Vector3D biNormal, ref int n, ref int k,
                            ref int tFlag, ref int bnFlag, ref double delta)
    {
        int i, maxN = 20;
        delta = 0.0;
        double h;
        h = 2 * _snglLen!.Value(index);

        Vector3D t = new Vector3D(), N = new Vector3D(), bn = new Vector3D();
        tFlag = 1;
        bnFlag = 1;
        GetInterval(out double a, out double b);
        if (u >= (a + b) / 2)
        {
            h = -h;
        }
        for (i = 1; i <= maxN; i++)
        {
            tangent = _trimmed!.DN(u, i);
            if (tangent.Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                break;
            }
        }
        if (i > maxN)
        {
            return false;
        }
        tangent = tangent.Normalized();
        n = i;

        i++;
        for (; i <= maxN; i++)
        {
            biNormal = tangent.Crossed(_trimmed!.DN(u, i));
            double magn = biNormal.Magnitude();
            if (magn > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                // modified by jgv, 12.08.03 for OCC605 ////
                Vector3D nextBiNormal = tangent.Crossed(_trimmed.DN(u, i + 1));
                if (nextBiNormal.Magnitude() > magn)
                {
                    i++;
                    biNormal = nextBiNormal;
                }
                break;
            }
        }
        if (i > maxN)
        {
            delta = h;
            return false;
        }
        biNormal = biNormal.Normalized();
        k = i;

        D0(u + h, ref t, ref N, ref bn);

        if (tangent.Angle(t) > M_PI / 2)
        {
            tFlag = -1;
        }
        if (biNormal.Angle(bn) > M_PI / 2)
        {
            bnFlag = -1;
        }

        return true;
    }

    private bool SingularD0(double param, int index, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal,
                            ref double delta)
    {
        int n = 0, k = 0, tFlag = 0, bnFlag = 0;
        if (!DoSingular(param, index, ref tangent, ref biNormal, ref n, ref k, ref tFlag, ref bnFlag, ref delta))
        {
            return false;
        }

        tangent *= tFlag;
        biNormal *= bnFlag;
        normal = biNormal;
        normal = normal.Crossed(tangent);

        return true;
    }

    private bool SingularD1(double param, int index, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D biNormal, ref Vector3D dBiNormal, ref double delta)
    {
        int n = 0, k = 0, tFlag = 0, bnFlag = 0;
        if (!DoSingular(param, index, ref tangent, ref biNormal, ref n, ref k, ref tFlag, ref bnFlag, ref delta))
        {
            return false;
        }

        Vector3D f, df, dtmp;
        f = _trimmed!.DN(param, n);
        df = _trimmed.DN(param, n + 1);
        dTangent = FDeriv(f, df);

        dtmp = _trimmed.DN(param, k);
        f = tangent.Crossed(dtmp);
        df = dTangent.Crossed(dtmp) + tangent.Crossed(_trimmed.DN(param, k + 1));
        dBiNormal = FDeriv(f, df);

        if (tFlag < 0)
        {
            tangent = -tangent;
            dTangent = -dTangent;
        }

        if (bnFlag < 0)
        {
            biNormal = -biNormal;
            dBiNormal = -dBiNormal;
        }

        normal = biNormal.Crossed(tangent);
        dNormal = dBiNormal.Crossed(tangent) + biNormal.Crossed(dTangent);

        return true;
    }

    private bool SingularD2(double param, int index, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent,
                            ref Vector3D normal, ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal,
                            ref Vector3D dBiNormal, ref Vector3D d2BiNormal, ref double delta)
    {
        int n = 0, k = 0, tFlag = 0, bnFlag = 0;
        if (!DoSingular(param, index, ref tangent, ref biNormal, ref n, ref k, ref tFlag, ref bnFlag, ref delta))
        {
            return false;
        }

        Vector3D f, df, d2F, dtmp1, dtmp2;
        f = _trimmed!.DN(param, n);
        df = _trimmed.DN(param, n + 1);
        d2F = _trimmed.DN(param, n + 2);
        dTangent = FDeriv(f, df);
        d2Tangent = DDeriv(f, df, d2F);

        dtmp1 = _trimmed.DN(param, k);
        dtmp2 = _trimmed.DN(param, k + 1);
        f = tangent.Crossed(dtmp1);
        df = dTangent.Crossed(dtmp1) + tangent.Crossed(dtmp2);
        d2F = d2Tangent.Crossed(dtmp1) + 2 * dTangent.Crossed(dtmp2) + tangent.Crossed(_trimmed.DN(param, k + 2));
        dBiNormal = FDeriv(f, df);
        d2BiNormal = DDeriv(f, df, d2F);

        if (tFlag < 0)
        {
            tangent = -tangent;
            dTangent = -dTangent;
            d2Tangent = -d2Tangent;
        }

        if (bnFlag < 0)
        {
            biNormal = -biNormal;
            dBiNormal = -dBiNormal;
            d2BiNormal = -d2BiNormal;
        }

        normal = biNormal.Crossed(tangent);
        dNormal = dBiNormal.Crossed(tangent) + biNormal.Crossed(dTangent);
        d2Normal = d2BiNormal.Crossed(tangent) + 2 * dBiNormal.Crossed(dTangent) + biNormal.Crossed(d2Tangent);

        return true;
    }
}
