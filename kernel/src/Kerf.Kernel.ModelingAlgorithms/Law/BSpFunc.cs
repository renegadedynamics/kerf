// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSpFunc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSpFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Laws;

/// <summary>A law based on a 1D B-spline curve (<see cref="BSpline"/>) between two parameters.</summary>
/// <remarks>
/// A working object (decision 21): <see cref="SetCurve"/> is public and changes the law. The curve the law keeps
/// is frozen (decision 132's pattern): the constructor and <see cref="SetCurve"/> freeze the caller's curve,
/// which they keep as OCCT keeps the handle, and <see cref="Curve"/> returns that same curve. Laws made by
/// <see cref="Trim"/> share it.
/// </remarks>
internal class BSpFunc : LawsFunction
{
    private BSpline? _curv;
    private double _first;
    private double _last;

    private static double PosTol => global::Kerf.Engine.Foundation.Precision.PConfusion() / 2;

    public BSpFunc()
    {
        _first = 0.0;
        _last = 0.0;
    }

    /// <summary>The law of the curve C between First and Last; C is kept and frozen.</summary>
    public BSpFunc(BSpline c, double first, double last)
    {
        c.Freeze();
        _curv = c;
        _first = first;
        _last = last;
    }

    public override Continuity Continuity() => _curv!.Continuity();

    public override int NbIntervals(Continuity s)
    {
        int myNbIntervals = 1;
        if (s > Continuity())
        {
            int cont;
            switch (s)
            {
                case G1:
                case G2:
                    throw new DomainErrorException("Law_BSpFunc::NbIntervals");
                case C0:
                    myNbIntervals = 1;
                    break;
                case C1:
                case C2:
                case C3:
                case CN:
                {
                    if (s == C1)
                    {
                        cont = 1;
                    }
                    else if (s == C2)
                    {
                        cont = 2;
                    }
                    else if (s == C3)
                    {
                        cont = 3;
                    }
                    else
                    {
                        cont = _curv!.Degree();
                    }
                    BSplineKnotSplitting convector = new BSplineKnotSplitting(_curv!, cont);
                    int nbInt = convector.SplitCount - 1;
                    NCollection_Array1<int> inter = new NCollection_Array1<int>(1, nbInt + 1);
                    convector.Splitting(inter);

                    int nb = _curv!.KnotCount;
                    int index1;
                    int index2;
                    double newFirst, newLast;
                    NCollection_Array1<double> tk = new NCollection_Array1<double>(1, nb);
                    NCollection_Array1<int> tm = new NCollection_Array1<int>(1, nb);
                    _curv.Knots(tk);
                    _curv.Multiplicities(tm);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_curv.Degree(), tk, tm, _first, _curv.IsPeriodic, 1, nb, out index1,
                                             out newFirst);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_curv.Degree(), tk, tm, _last, _curv.IsPeriodic, 1, nb, out index2,
                                             out newLast);
                    if (Math.Abs(newFirst - tk[index1 + 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        index1++;
                    }
                    if (newLast - tk[index2] > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        index2++;
                    }

                    myNbIntervals = 1;
                    for (int i = 1; i <= nbInt; i++)
                    {
                        if (inter[i] > index1 && inter[i] < index2)
                        {
                            myNbIntervals++;
                        }
                    }
                }
                break;
            }
        }

        return myNbIntervals;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        int myNbIntervals = 1;
        if (s > Continuity())
        {
            int cont;
            switch (s)
            {
                case G1:
                case G2:
                    throw new DomainErrorException("Law_BSpFunc_Curve::Intervals");
                case C0:
                    myNbIntervals = 1;
                    break;
                case C1:
                case C2:
                case C3:
                case CN:
                {
                    if (s == C1)
                    {
                        cont = 1;
                    }
                    else if (s == C2)
                    {
                        cont = 2;
                    }
                    else if (s == C3)
                    {
                        cont = 3;
                    }
                    else
                    {
                        cont = _curv!.Degree();
                    }
                    BSplineKnotSplitting convector = new BSplineKnotSplitting(_curv!, cont);
                    int nbInt = convector.SplitCount - 1;
                    NCollection_Array1<int> inter = new NCollection_Array1<int>(1, nbInt + 1);
                    convector.Splitting(inter);

                    int nb = _curv!.KnotCount;
                    int index1;
                    int index2;
                    double newFirst, newLast;
                    NCollection_Array1<double> tk = new NCollection_Array1<double>(1, nb);
                    NCollection_Array1<int> tm = new NCollection_Array1<int>(1, nb);
                    _curv.Knots(tk);
                    _curv.Multiplicities(tm);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_curv.Degree(), tk, tm, _first, _curv.IsPeriodic, 1, nb, out index1,
                                             out newFirst);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_curv.Degree(), tk, tm, _last, _curv.IsPeriodic, 1, nb, out index2,
                                             out newLast);
                    if (Math.Abs(newFirst - tk[index1 + 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        index1++;
                    }
                    if (newLast - tk[index2] > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        index2++;
                    }

                    inter[1] = index1;
                    myNbIntervals = 1;
                    for (int i = 1; i <= nbInt; i++)
                    {
                        if (inter[i] > index1 && inter[i] < index2)
                        {
                            myNbIntervals++;
                            inter[myNbIntervals] = inter[i];
                        }
                    }
                    inter[myNbIntervals + 1] = index2;

                    for (int I = 1; I <= myNbIntervals + 1; I++)
                    {
                        t[I] = tk[inter[I]];
                    }
                }
                break;
            }
        }
        t[t.Lower()] = _first;
        t[t.Lower() + myNbIntervals] = _last;
    }

    /// <summary>The knot span to evaluate X on when X is a bound of the law (the span inside the bounds).</summary>
    private void LocateBound(double x, out int ideb, out int ifin)
    {
        ideb = 0;
        ifin = 0;
        if (x == _first)
        {
            _curv!.LocateU(_first, PosTol, out ideb, out ifin);
            if (ideb < 1)
            {
                ideb = 1;
            }
            if (ideb >= ifin)
            {
                ifin = ideb + 1;
            }
        }
        if (x == _last)
        {
            _curv!.LocateU(_last, PosTol, out ideb, out ifin);
            if (ifin > _curv.KnotCount)
            {
                ifin = _curv.KnotCount;
            }
            if (ideb >= ifin)
            {
                ideb = ifin - 1;
            }
        }
    }

    public override double Value(double x)
    {
        if ((x == _first) || (x == _last))
        {
            LocateBound(x, out int ideb, out int ifin);
            return _curv!.LocalValue(x, ideb, ifin);
        }
        else
        {
            return _curv!.Value(x);
        }
    }

    public override void D1(double x, out double f, out double d)
    {
        if ((x == _first) || (x == _last))
        {
            LocateBound(x, out int ideb, out int ifin);
            _curv!.LocalD1(x, ideb, ifin, out f, out d);
        }
        else
        {
            _curv!.D1(x, out f, out d);
        }
    }

    public override void D2(double x, out double f, out double d, out double d2)
    {
        if ((x == _first) || (x == _last))
        {
            LocateBound(x, out int ideb, out int ifin);
            _curv!.LocalD2(x, ideb, ifin, out f, out d, out d2);
        }
        else
        {
            _curv!.D2(x, out f, out d, out d2);
        }
    }

    /// <summary>A law of the same curve between PFirst and PLast.</summary>
    public override LawsFunction Trim(double pFirst, double pLast, double tol)
    {
        BSpFunc l = new BSpFunc(_curv!, pFirst, pLast);
        return l;
    }

    public override void Bounds(out double pFirst, out double pLast)
    {
        pFirst = _first;
        pLast = _last;
    }

    /// <summary>The curve of the law (frozen), or null if none was set.</summary>
    public BSpline? Curve() => _curv;

    /// <summary>Sets the curve of the law (kept and frozen) and its bounds as the law's.</summary>
    public void SetCurve(BSpline c)
    {
        c.Freeze();
        _curv = c;
        _first = c.FirstParameter();
        _last = c.LastParameter();
    }
}
