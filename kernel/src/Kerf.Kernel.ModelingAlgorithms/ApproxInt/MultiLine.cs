// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_MultiLine.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// What ApproxInt's generics read from a walking line (OCCT's TheLine and TheWLine macros: IntPatch_WLine in GeomInt,
/// BRepApprox_ApproxLine in BRepApprox): its points, and a new line over a sequence of points.
/// </summary>
/// <remarks>Not an OCCT type (decision 607's instance interfaces).</remarks>
/// <typeparam name="TSelf">The line type.</typeparam>
internal interface IWLine<TSelf> where TSelf : class, IWLine<TSelf>
{
    /// <summary>The number of points of the line.</summary>
    int NbPnts();

    /// <summary>The point <paramref name="index"/> (from 1).</summary>
    PntOn2S Point(int index);

    /// <summary>OCCT's <c>new TheLine(lin, theTang)</c>: a line on the points of <paramref name="lin"/>.</summary>
    static abstract TSelf Create(LineOn2S lin, bool tang);
}

/// <summary>
/// The MultiLine of ApproxInt's approximation: the points <c>indicemin</c> to <c>indicemax</c> of a walking line as
/// one 3D point (translated by Xo, Yo, Zo) and up to two 2D points (translated by U1o, V1o and U2o, V2o) each, with
/// tangents and new points computed on the surfaces through an <see cref="SvSurfaces"/>.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's generic file ApproxInt_MultiLine.gxx, ported once as a generic class over the line (decision 607). TheSvSurfaces
/// is ApproxInt_SvSurfaces in every instantiation, so the body names it; OCCT's untyped pointer to it is a typed
/// reference (null: no points are ever added). OCCT's instantiations (GeomInt_TheMultiLineOfWLApprox,
/// BRepApprox_TheMultiLineOfApprox) are sealed subclasses; the generic algorithms build this class.
/// </para>
/// <para>
/// A C++ value class whose state never changes after construction: a class with read-only fields, as the MultiLine
/// argument of the AppParCurves and Approx generics expects (decision 142). It shares the line and the SvSurfaces it
/// is given, as OCCT's copies do.
/// </para>
/// </remarks>
/// <typeparam name="TheLine">The walking line.</typeparam>
internal class MultiLine<TheLine> where TheLine : class, IWLine<TheLine>
{
    private readonly SvSurfaces? _ptrOnmySvSurfaces;
    private readonly TheLine? _line;
    private readonly int _indicemin;
    private readonly int _indicemax;
    private readonly int _nbp3d;
    private readonly int _nbp2d;
    private readonly bool _approxU1V1;
    private readonly bool _approxU2V2;
    private readonly bool _p2donfirst;
    private readonly double _xo;
    private readonly double _yo;
    private readonly double _zo;
    private readonly double _u1o;
    private readonly double _v1o;
    private readonly double _u2o;
    private readonly double _v2o;

    /// <summary>An empty MultiLine, without a line.</summary>
    public MultiLine()
    {
        _ptrOnmySvSurfaces = null;
        _line = null;
        _indicemin = 0;
        _indicemax = 0;
        _nbp3d = 0;
        _nbp2d = 0;
        _approxU1V1 = false;
        _approxU2V2 = false;
        _p2donfirst = true;
        _xo = 0.0;
        _yo = 0.0;
        _zo = 0.0;
        _u1o = 0.0;
        _v1o = 0.0;
        _u2o = 0.0;
        _v2o = 0.0;
    }

    /// <summary>
    /// The points <paramref name="indMin"/> to <paramref name="indMax"/> of <paramref name="line"/>.
    /// <paramref name="svsurf"/> computes the tangents and the new points when the approximation needs some extra
    /// points: a new line is then made, which shares the same surfaces and functions.
    /// </summary>
    public MultiLine(TheLine line, SvSurfaces? svsurf, int nbP3d, int nbP2d, bool approxU1V1,
                               bool approxU2V2, double xo, double yo, double zo, double u1o, double v1o, double u2o,
                               double v2o, bool p2DOnFirst, int indMin = 0, int indMax = 0)
    {
        _ptrOnmySvSurfaces = svsurf;
        _line = line;
        _indicemin = Min(indMin, indMax);
        _indicemax = Max(indMin, indMax);
        _nbp3d = nbP3d;
        _nbp2d = nbP2d;
        _approxU1V1 = approxU1V1;
        _approxU2V2 = approxU2V2;
        _p2donfirst = p2DOnFirst;
        _xo = xo;
        _yo = yo;
        _zo = zo;
        _u1o = u1o;
        _v1o = v1o;
        _u2o = u2o;
        _v2o = v2o;
    }

    /// <summary>As the constructor above without SvSurfaces: no extra points will be added on the line.</summary>
    public MultiLine(TheLine line, int nbP3d, int nbP2d, bool approxU1V1, bool approxU2V2, double xo,
                               double yo, double zo, double u1o, double v1o, double u2o, double v2o, bool p2DOnFirst,
                               int indMin = 0, int indMax = 0)
        : this(line, null, nbP3d, nbP2d, approxU1V1, approxU2V2, xo, yo, zo, u1o, v1o, u2o, v2o, p2DOnFirst, indMin,
               indMax)
    {
    }

    public int FirstPoint() => _indicemin;

    public int LastPoint() => _indicemax;

    /// <summary>Approx_PointsAdded when points can be added (there are SvSurfaces), else Approx_NoPointsAdded.</summary>
    public Status WhatStatus()
    {
        if (_ptrOnmySvSurfaces is not null)
        {
            return Status.PointsAdded;
        }
        else
        {
            return Status.NoPointsAdded;
        }
    }

    /// <summary>The number of 3d points of a MultiPoint.</summary>
    public int P3dCount => _nbp3d;

    /// <summary>The number of 2d points of a MultiPoint.</summary>
    public int P2dCount => _nbp2d;

    /// <summary>The 3d point of the MultiPoint <paramref name="index"/>.</summary>
    public void Value(int index, NCollection_Array1<Point3D> tabPnt)
    {
        Point3D p = _line!.Point(index).Value();
        tabPnt[1] = tabPnt[1].WithCoord(p.X + _xo, p.Y + _yo, p.Z + _zo);
    }

    /// <summary>The 2d points of the MultiPoint <paramref name="index"/>.</summary>
    public void Value(int index, NCollection_Array1<Point2D> tabPnt2d)
    {
        PntOn2S pOn2S = _line!.Point(index);
        pOn2S.Parameters(out double u1, out double v1, out double u2, out double v2);
        if (_nbp2d == 1)
        {
            if (_p2donfirst)
            {
                tabPnt2d[1] = tabPnt2d[1].WithCoord(u1 + _u1o, v1 + _v1o);
            }
            else
            {
                tabPnt2d[1] = tabPnt2d[1].WithCoord(u2 + _u2o, v2 + _v2o);
            }
        }
        else
        {
            tabPnt2d[1] = tabPnt2d[1].WithCoord(u1 + _u1o, v1 + _v1o);
            if (tabPnt2d.Length() >= 2)
            {
                tabPnt2d[2] = tabPnt2d[2].WithCoord(u2 + _u2o, v2 + _v2o);
            }
        }
    }

    /// <summary>The 3d and 2d points of the MultiPoint <paramref name="index"/>.</summary>
    public void Value(int index, NCollection_Array1<Point3D> tabPnt, NCollection_Array1<Point2D> tabPnt2d)
    {
        Value(index, tabPnt);
        Value(index, tabPnt2d);
    }

    /// <summary>The 3d tangent at the MultiPoint <paramref name="index"/>; false (a null vector) where there is none.</summary>
    public bool Tangency(int index, NCollection_Array1<Vector3D> tabVec)
    {
        if (_ptrOnmySvSurfaces is null)
        {
            return false;
        }

        PntOn2S pOn2S = _line!.Point(index);
        pOn2S.Parameters(out double u1, out double v1, out double u2, out double v2);

        bool ret = _ptrOnmySvSurfaces.Tangency(u1, v1, u2, v2, ref tabVec[1]);
        if (!ret)
        {
            tabVec[1] = tabVec[1].WithCoord(0.0, 0.0, 0.0);
        }

        return ret;
    }

    /// <summary>The 2d tangents at the MultiPoint <paramref name="index"/>; false (null vectors) where there are none.</summary>
    public bool Tangency(int index, NCollection_Array1<Vector2D> tabVec2d)
    {
        if (_ptrOnmySvSurfaces is null)
        {
            return false;
        }

        PntOn2S pOn2S = _line!.Point(index);
        pOn2S.Parameters(out double u1, out double v1, out double u2, out double v2);

        bool ret = false;
        if (_nbp2d == 1)
        {
            if (_p2donfirst)
            {
                ret = _ptrOnmySvSurfaces.TangencyOnSurf1(u1, v1, u2, v2, ref tabVec2d[1]);
            }
            else
            {
                ret = _ptrOnmySvSurfaces.TangencyOnSurf2(u1, v1, u2, v2, ref tabVec2d[1]);
            }
        }
        else
        {
            ret = _ptrOnmySvSurfaces.TangencyOnSurf1(u1, v1, u2, v2, ref tabVec2d[1]);
            if (ret)
            {
                if (tabVec2d.Length() >= 2)
                {
                    ret = (ret && _ptrOnmySvSurfaces.TangencyOnSurf2(u1, v1, u2, v2, ref tabVec2d[2]));
                }
            }
        }

        if (!ret)
        {
            tabVec2d[1] = new Vector2D(0.0, 0.0);
            if (tabVec2d.Length() >= 2)
            {
                tabVec2d[2] = new Vector2D(0.0, 0.0);
            }
        }

        return ret;
    }

    /// <summary>The 3d and 2d tangents at the MultiPoint <paramref name="index"/>.</summary>
    public bool Tangency(int index, NCollection_Array1<Vector3D> tabVec, NCollection_Array1<Vector2D> tabVec2d)
    {
        return (Tangency(index, tabVec) && Tangency(index, tabVec2d));
    }

    /// <summary>
    /// Tries to make a sub-line between the points <paramref name="low"/> and <paramref name="high"/> of this line
    /// by adding <paramref name="aNbPntsToInsert"/> new points; an empty line if it can't.
    /// </summary>
    public global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> MakeMLBetween(int low, int high, int aNbPntsToInsert)
    {
        if (_ptrOnmySvSurfaces is null)
        {
            //-- cout<<"\n Erreur dans : ApproxInt_MultiLine  ApproxInt_MultiLine::MakeMLBetween "<<endl;
            LineOn2S vide1 = new();
            TheLine vide = TheLine.Create(vide1, false);
            return (new global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine>(vide, null, _nbp3d, _nbp2d, _approxU1V1, _approxU2V2, _xo, _yo, _zo,
                                                     _u1o, _v1o, _u2o, _v2o, _p2donfirst, 1, 1));
            //-- return(*this);
        }

        bool saveUseSolver = _ptrOnmySvSurfaces.GetUseSolver();
        if (!saveUseSolver)
        {
            _ptrOnmySvSurfaces.SetUseSolver(true);
        }
        int nbPntsToInsert = aNbPntsToInsert;
        if (nbPntsToInsert < (high - low))
        {
            nbPntsToInsert = (high - low);
        }
        int nbPnts = nbPntsToInsert + high - low + 1;
        int nbPntsmin = high - low;
        nbPntsmin += nbPntsmin;

        if (nbPnts < nbPntsmin)
        {
            nbPnts = nbPntsmin;
        }

        Vector3D t = new();
        Vector2D ts1 = new(), ts2 = new();
        Point3D p = new();

        //-----------------------l-------------------------------------------
        //--  Indice     :   Low       Low+1     I    I+1         High    --
        //--                                                              --
        //--  Abs.Curv.  :  S(Low)              S(I)  S(I+1)      S(High) --
        //--                                                              --
        //--                Sampled at constant curvilinear abscissa      --
        //--                (computed on the 3D points)                   --
        //------------------------------------------------------------------
        NCollection_Array1<double> U1 = new(low, high);
        NCollection_Array1<double> V1 = new(low, high);
        NCollection_Array1<double> U2 = new(low, high);
        NCollection_Array1<double> V2 = new(low, high);
        NCollection_Array1<double> ac = new(low, high);
        double s, ds;

        //------------------------------------------------------------
        //-- The arrays U1 .. V2 and AC
        //--
        double u1, v1, u2, v2;
        int i;
        _line!.Point(low).Parameters(out u1, out v1, out u2, out v2);
        U1[low] = u1;
        V1[low] = v1;
        U2[low] = u2;
        V2[low] = v2;
        ac[low] = 0.0;

        //-- Trial of 19 June 96 (parametrisation by curvilinear abscissa in XYZ)
        for (i = low + 1; i <= high; i++)
        {
            _line.Point(i).Parameters(out u1, out v1, out u2, out v2);
            U1[i] = u1;
            V1[i] = v1;
            U2[i] = u2;
            V2[i] = v2;
            ac[i] = ac[i - 1] + (_line.Point(i - 1).Value()).DistanceTo(_line.Point(i).Value());
        }

        //-------------------------------------------------------------
        //-- The structures holding the results

        LineOn2S resultPntOn2SLine = new();

        PntOn2S startPOn2S = new();

        ds = ac[high] / (nbPnts - 1);
        int indice = low;
        bool hasBeenInserted = false;
        double dsmin = ds * 0.3;
        double smax = ac[high];

        for (i = 2, s = ds; (s < smax && indice <= high - 1); i++, s += ds)
        {
            //----------------------------------------------------------
            //-- Search of the indices of the points                  --
            //-- Point           :    2       i       NbPnts-1        --
            //-- s                            s                       --
            //-- Current Indice such that AC(Indice)<= s < AC(Indice+1)--
            //----------------------------------------------------------
            while (ac[indice + 1] <= s)
            {
                if (!hasBeenInserted)
                {
                    resultPntOn2SLine.Add(_line.Point(indice));
                }

                hasBeenInserted = false;
                indice++;
                if (indice == high)
                {
                    break;
                }
            }

            if (indice == high)
            {
                break;
            }

            if (!hasBeenInserted && ac[indice] <= s)
            {
                resultPntOn2SLine.Add(_line.Point(indice));
                hasBeenInserted = true;
            }

            double a = s - ac[indice];
            double b = ac[indice + 1] - s;
            double nab = 1.0 / (a + b);

            //----------------------------------------------------------
            //-- Check: distance to the next point < dsmin            --
            //--        distance to the previous point < dsmin        --
            //----------------------------------------------------------
            if ((a > dsmin) && (b > dsmin))
            {
                u1 = (U1[indice] * b + U1[indice + 1] * a) * nab;
                v1 = (V1[indice] * b + V1[indice + 1] * a) * nab;
                u2 = (U2[indice] * b + U2[indice + 1] * a) * nab;
                v2 = (V2[indice] * b + V2[indice + 1] * a) * nab;

                if (_ptrOnmySvSurfaces.Compute(ref u1, ref v1, ref u2, ref v2, ref p, ref t, ref ts1, ref ts2))
                {
                    startPOn2S.SetValue(p, u1, v1, u2, v2);
                    resultPntOn2SLine.Add(startPOn2S);
                }
            }
            else
            {
                //-- Point not far enough from 2 existing points
                //-- with the point p[indice] already inserted

                if (b < 0.0)
                {
                    while (ac[indice + 1] <= s)
                    {
                        if (!hasBeenInserted)
                        {
                            resultPntOn2SLine.Add(_line.Point(indice));
                        }

                        hasBeenInserted = false;
                        indice++;

                        if (indice == high)
                        {
                            break;
                        }
                    }

                    if (indice == high)
                    {
                        break;
                    }

                    if (!hasBeenInserted && ac[indice] <= s)
                    {
                        resultPntOn2SLine.Add(_line.Point(indice));
                        hasBeenInserted = true;
                    }
                }
                else
                {
                    s += (dsmin - ds);
                }
            }
        }

        resultPntOn2SLine.Add(_line.Point(high)); //--  Point NbPnts
        TheLine temp = TheLine.Create(resultPntOn2SLine, false);

        //-- A posteriori check
        //-- There must be no too sharp turn in 2d and in 3d

        temp.Point(1).Parameters(out u1, out v1, out u2, out v2);
        Point2D p1A = new(u1, v1);
        Point2D p2A = new(u2, v2);

        temp.Point(2).Parameters(out u1, out v1, out u2, out v2);
        Point2D p1B = new(u1, v1);
        Point2D p2B = new(u2, v2);

        Point2D p1C = new(), p2C = new();

        int codeErreur = 0;

        for (i = 3, nbPnts = temp.NbPnts(); codeErreur == 0 && i <= nbPnts; i++)
        {
            double d, du, dv, duv2;
            temp.Point(i).Parameters(out u1, out v1, out u2, out v2);
            //-- Turn P1A P1B P1C
            p1C = p1C.WithCoord(u1, v1);
            du = p1B.X - p1A.X;
            dv = p1B.Y - p1A.Y;
            duv2 = 0.25 * (du * du + dv * dv);
            u1 = p1B.X + du;
            v1 = p1B.Y + dv;
            du = p1C.X - u1;
            dv = p1C.Y - v1;
            d = du * du + dv * dv;
            if (d > duv2)
            {
                codeErreur = 1;
                break;
            }

            //-- Turn P2A P2B P2C
            p2C = p2C.WithCoord(u2, v2);
            du = p2B.X - p2A.X;
            dv = p2B.Y - p2A.Y;
            duv2 = 0.25 * (du * du + dv * dv);
            u2 = p2B.X + du;
            v2 = p2B.Y + dv;
            du = p2C.X - u2;
            dv = p2C.Y - v2;
            d = du * du + dv * dv;
            if (d > duv2)
            {
                codeErreur = 2;
                break;
            }

            p1A = p1B;
            p2A = p2B;
            p1B = p1C;
            p2B = p2C;
        }

        if ((temp.NbPnts() >= nbPntsToInsert + high - low + 1) && (codeErreur == 0))
        {
            _ptrOnmySvSurfaces.SetUseSolver(saveUseSolver);
            return (new global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine>(temp, (high - low > 10) ? _ptrOnmySvSurfaces : null, _nbp3d, _nbp2d,
                                                     _approxU1V1, _approxU2V2, _xo, _yo, _zo, _u1o, _v1o, _u2o, _v2o,
                                                     _p2donfirst, 1, resultPntOn2SLine.PointCount));
        }
        else
        {
            _ptrOnmySvSurfaces.SetUseSolver(saveUseSolver);
            LineOn2S vide1 = new();
            TheLine vide = TheLine.Create(vide1, false);
            return (new global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine>(vide, null, _nbp3d, _nbp2d, _approxU1V1, _approxU2V2, _xo, _yo, _zo,
                                                     _u1o, _v1o, _u2o, _v2o, _p2donfirst, 1, 1));
        }
    }

    /// <summary>
    /// Tries to make a sub-line between the points <paramref name="low"/> and <paramref name="high"/> of this
    /// line by adding one more point between the points (<paramref name="indbad"/> - 1) and
    /// <paramref name="indbad"/>.
    /// </summary>
    /// <remarks>
    /// Where it returns false, OCCT leaves <paramref name="newMultiLine"/> unchanged (its callers pass a
    /// default-constructed line): it is an empty MultiLine (decision 202).
    /// </remarks>
    public bool MakeMLOneMorePoint(int low, int high, int indbad,
                                   out global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> newMultiLine)
    {
        newMultiLine = new global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine>();
        bool otherLineMade = false;
        if (_ptrOnmySvSurfaces is null)
        {
            return false;
        }

        bool saveUseSolver = _ptrOnmySvSurfaces.GetUseSolver();
        if (!saveUseSolver)
        {
            _ptrOnmySvSurfaces.SetUseSolver(true);
        }

        double sqTol3d = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = tolerance[2] = 1.0e-8;

        LineOn2S resultPntOn2SLine = new();
        for (int indice = low; indice <= high; indice++)
        {
            resultPntOn2SLine.Add(_line!.Point(indice));
        }

        // Insert new point between (theIndbad-1) and theIndbad
        // Using <thePtrSVSurf> for Rsnld: it may be ImpPrm or PrmPrm
        Point3D prevPnt = _line!.Point(indbad - 1).Value();
        Point3D curPnt = _line.Point(indbad).Value();
        _line.Point(indbad - 1).Parameters(out double uprev1, out double vprev1, out double uprev2, out double vprev2);
        _line.Point(indbad).Parameters(out double ucur1, out double vcur1, out double ucur2, out double vcur2);
        double umid1, vmid1, umid2, vmid2;
        umid1 = (uprev1 + ucur1) / 2;
        vmid1 = (vprev1 + vcur1) / 2;
        umid2 = (uprev2 + ucur2) / 2;
        vmid2 = (vprev2 + vcur2) / 2;
        PntOn2S midPoint = new();
        bool isNewPointInvalid = false;
        isNewPointInvalid = _approxU1V1 && Math.Abs(ucur1 - umid1) <= tolerance[1]
                            && Math.Abs(vcur1 - vmid1) <= tolerance[2];
        if (!isNewPointInvalid)
        {
            isNewPointInvalid = _approxU2V2 && Math.Abs(ucur2 - umid2) <= tolerance[1]
                                && Math.Abs(vcur2 - vmid2) <= tolerance[2];
            if (!isNewPointInvalid && _ptrOnmySvSurfaces.SeekPoint(umid1, vmid1, umid2, vmid2, ref midPoint))
            {
                Point3D newPnt = midPoint.Value();
                double sqDistNewPrev = newPnt.SquareDistanceTo(prevPnt);
                double sqDistNewCur = newPnt.SquareDistanceTo(curPnt);
                isNewPointInvalid = (sqDistNewPrev <= sqTol3d || sqDistNewCur <= sqTol3d);
                if (!isNewPointInvalid)
                {
                    midPoint.Parameters(out double unew1, out double vnew1, out double unew2, out double vnew2);
                    if (_approxU1V1)
                    {
                        double sqDistCurMid1 = (ucur1 - umid1) * (ucur1 - umid1) + (vcur1 - vmid1) * (vcur1 - vmid1);
                        double sqDistMidNew1 = (umid1 - unew1) * (umid1 - unew1) + (vmid1 - vnew1) * (vmid1 - vnew1);
                        isNewPointInvalid = (sqDistMidNew1 > sqDistCurMid1);
                    }
                    if (!isNewPointInvalid)
                    {
                        if (_approxU2V2)
                        {
                            double sqDistCurMid2 =
                                (ucur2 - umid2) * (ucur2 - umid2) + (vcur2 - vmid2) * (vcur2 - vmid2);
                            double sqDistMidNew2 =
                                (umid2 - unew2) * (umid2 - unew2) + (vmid2 - vnew2) * (vmid2 - vnew2);
                            isNewPointInvalid = (sqDistMidNew2 > sqDistCurMid2);
                        }
                        if (!isNewPointInvalid)
                        {
                            resultPntOn2SLine.InsertBefore(indbad - low + 1, midPoint);
                            otherLineMade = true;
                        }
                    }
                }
            }
        }

        if (!otherLineMade)
        {
            _ptrOnmySvSurfaces.SetUseSolver(saveUseSolver);
            return false;
        }

        TheLine temp = TheLine.Create(resultPntOn2SLine, false);
        _ptrOnmySvSurfaces.SetUseSolver(saveUseSolver);
        newMultiLine = new global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine>(temp, _ptrOnmySvSurfaces, _nbp3d, _nbp2d, _approxU1V1,
                                                           _approxU2V2, _xo, _yo, _zo, _u1o, _v1o, _u2o, _v2o, _p2donfirst,
                                                           1, resultPntOn2SLine.PointCount);
        return true;
    }

    /// <summary>Writes the points of the MultiLine, one per line, as OCCT prints them on the standard output.</summary>
    public void Dump(TextWriter theOut)
    {
        NCollection_Array1<Point3D> arr1 = new(1, 1);
        NCollection_Array1<Point2D> arr2 = new(1, 2);

        int indF = FirstPoint(), indL = LastPoint();

        for (int ind = indF; ind <= indL; ind++)
        {
            Value(ind, arr1, arr2);
            theOut.Write(string.Create(CultureInfo.InvariantCulture,
                                       $"{ind,4}  [{F(arr1[1].X)} {F(arr1[1].Y)} {F(arr1[1].Z)}]  "
                                       + $"[{F(arr2[1].X)} {F(arr2[1].Y)}]  [{F(arr2[2].X)} {F(arr2[2].Y)}]\n"));
        }
    }

    // printf's "%+10.20f": a sign always, twenty decimals.
    private static string F(double value)
    {
        string str = value.ToString("F20", CultureInfo.InvariantCulture);
        return str.StartsWith('-') ? str : "+" + str;
    }
}
