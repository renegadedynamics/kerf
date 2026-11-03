// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_ExactIntersectionPoint.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The exact intersection point of two 2D curves near the crossing of two segments of their polygons: a root of
/// <see cref="DistBetweenPCurvesGen{TheCurve, TheCurveTool}"/> found by math_FunctionSetRoot, with the
/// search bounds widened over the neighbouring segments when none is found.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_ExactIntersectionPoint.gxx, ported once as a generic class (decision 324). An
/// algorithm object (decision 117) keeping the curves, as OCCT keeps pointers to them.
/// </remarks>
public class ExactIntersectionPoint<TheCurve, TheCurveTool>
    where TheCurveTool : ICurveCurveTool<TheCurve>
{
    // OCCT sets done and never reads it.
#pragma warning disable CS0414
    private bool _done;
#pragma warning restore CS0414
    private int _nbroots;
    private readonly double _tol;
    private readonly DistBetweenPCurvesGen<TheCurve, TheCurveTool> _fctDist;
    private readonly math_Vector _toleranceVector;
    private readonly math_Vector _bInfVector;
    private readonly math_Vector _bSupVector;
    private readonly math_Vector _startingPoint;
    private readonly math_Vector _root;
    private bool _anErrorOccurred;

    public ExactIntersectionPoint(in TheCurve c1, in TheCurve c2, double tol)
    {
        _done = false;
        _nbroots = 0;
        _tol = tol * tol;
        _fctDist = new DistBetweenPCurvesGen<TheCurve, TheCurveTool>(c1, c2);
        _toleranceVector = new math_Vector(1, 2);
        _bInfVector = new math_Vector(1, 2);
        _bSupVector = new math_Vector(1, 2);
        _startingPoint = new math_Vector(1, 2);
        _root = new math_Vector(1, 2);
        _anErrorOccurred = false;
        _toleranceVector.Value(1) = TheCurveTool.EpsX(c1);
        _toleranceVector.Value(2) = TheCurveTool.EpsX(c2);
    }

    /// <summary>
    /// Searches the root near the point <paramref name="paramOnSeg1"/> of the segment <paramref name="numSegOn1"/> of
    /// <paramref name="poly1"/> and the point <paramref name="paramOnSeg2"/> of the segment
    /// <paramref name="numSegOn2"/> of <paramref name="poly2"/>; the segments and parameters are moved inside the
    /// polygons when they are at an end.
    /// </summary>
    public void Perform(Polygon2dGen<TheCurve, TheCurveTool> poly1,
                        Polygon2dGen<TheCurve, TheCurveTool> poly2,
                        ref int numSegOn1,
                        ref int numSegOn2,
                        ref double paramOnSeg1,
                        ref double paramOnSeg2)
    {
        //----------------------------------------------------------------------
        //-- On prend comme bornes de recherches  :
        //--
        //--   Segment      :      i-1        i           i+1        i+2
        //--
        //--                  |---------|-----X-------|---------|----------|
        //--                Inf                                Sup
        //--
        if (numSegOn1 >= poly1.SegmentCount && paramOnSeg1 == 0.0)
        {
            numSegOn1--;
            paramOnSeg1 = 1.0;
        }
        if (numSegOn2 >= poly2.SegmentCount && paramOnSeg2 == 0.0)
        {
            numSegOn2--;
            paramOnSeg2 = 1.0;
        }
        if (numSegOn1 <= 0)
        {
            numSegOn1 = 1;
            paramOnSeg1 = 0.0;
        }
        if (numSegOn2 <= 0)
        {
            numSegOn2 = 1;
            paramOnSeg2 = 0.0;
        }

        _startingPoint.Value(1) = poly1.ApproxParamOnCurve(numSegOn1, paramOnSeg1);
        if (numSegOn1 <= 2)
        {
            _bInfVector.Value(1) = poly1.InfParameter();
        }
        else
        {
            _bInfVector.Value(1) = poly1.ApproxParamOnCurve(numSegOn1 - 1, (double)0.0);
        }
        if (numSegOn1 >= (poly1.SegmentCount - 2))
        {
            _bSupVector.Value(1) = poly1.SupParameter();
        }
        else
        {
            _bSupVector.Value(1) = poly1.ApproxParamOnCurve(numSegOn1 + 2, (double)0.0);
        }

        _startingPoint.Value(2) = poly2.ApproxParamOnCurve(numSegOn2, paramOnSeg2);
        if (numSegOn2 <= 2)
        {
            _bInfVector.Value(2) = poly2.InfParameter();
        }
        else
        {
            _bInfVector.Value(2) = poly2.ApproxParamOnCurve(numSegOn2 - 1, (double)0.0);
        }
        if (numSegOn2 >= (poly2.SegmentCount - 2))
        {
            _bSupVector.Value(2) = poly2.SupParameter();
        }
        else
        {
            _bSupVector.Value(2) = poly2.ApproxParamOnCurve(numSegOn2 + 2, (double)0.0);
        }

        MathPerform();
        if (_nbroots == 0)
        {
            // OCCT reads both deflections and discards them.
            poly1.DeflectionOverEstimation();
            poly2.DeflectionOverEstimation();
            {
                //-- On risque de donner des bornes sur la courbe 1 trop etroites.
                int diff = 1;
                double anBinfVector = _bInfVector.Value(1);
                double anBsupVector = _bSupVector.Value(1);
                //---------------- On elargit les bornes par la gauche --------------------
                do
                {
                    diff++;
                    if ((numSegOn1 - diff) <= 1)
                    {
                        _bInfVector.Value(1) = poly1.InfParameter();
                        diff = 0;
                    }
                    else
                    {
                        _bInfVector.Value(1) = poly1.ApproxParamOnCurve(numSegOn1 - diff, (double)0.0);
                    }
                    MathPerform();
                    //-- le 18 nov 97
                    if (diff > 3)
                    {
                        diff += numSegOn1 / 2;
                    }
                } while (_nbroots == 0 && diff != 0);
                //---------------- On elargit les bornes par la droite --------------------
                if (_nbroots == 0)
                {
                    _bInfVector.Value(1) = anBinfVector;
                    diff = 1;
                    do
                    {
                        diff++;
                        if ((numSegOn1 + diff) >= (poly1.SegmentCount - 1))
                        {
                            _bSupVector.Value(1) = poly1.SupParameter();
                            diff = 0;
                        }
                        else
                        {
                            _bSupVector.Value(1) = poly1.ApproxParamOnCurve(numSegOn1 + 1 + diff, (double)0.0);
                        }
                        MathPerform();
                        //-- le 18 nov 97
                        if (diff > 3)
                        {
                            diff += 1 + (poly1.SegmentCount - numSegOn1) / 2;
                        }
                    } while (_nbroots == 0 && diff != 0);
                }
                _bSupVector.Value(1) = anBsupVector;
            }

            if (_nbroots == 0)
            {
                //-- On risque de donner des bornes sur la courbe 1 trop etroites.
                int diff = 1;
                double anBinfVector = _bInfVector.Value(2);
                double anBsupVector = _bSupVector.Value(2);
                //---------------- On elargit les bornes par la gauche --------------------
                do
                {
                    diff++;
                    if ((numSegOn2 - diff) <= 1)
                    {
                        _bInfVector.Value(2) = poly2.InfParameter();
                        diff = 0;
                    }
                    else
                    {
                        _bInfVector.Value(2) = poly2.ApproxParamOnCurve(numSegOn2 - diff, (double)0.0);
                    }
                    MathPerform();
                    //-- le 18 nov 97
                    if (diff > 3)
                    {
                        diff += numSegOn2 / 2;
                    }
                } while (_nbroots == 0 && diff != 0);
                //---------------- On elargit les bornes par la droite --------------------
                if (_nbroots == 0)
                {
                    _bInfVector.Value(2) = anBinfVector;
                    diff = 1;
                    do
                    {
                        diff++;
                        if ((numSegOn2 + diff) >= (poly2.SegmentCount - 1))
                        {
                            _bSupVector.Value(2) = poly2.SupParameter();
                            diff = 0;
                        }
                        else
                        {
                            _bSupVector.Value(2) = poly2.ApproxParamOnCurve(numSegOn2 + 1 + diff, (double)0.0);
                        }
                        MathPerform();
                        //-- le 18 nov 97
                        if (diff > 3)
                        {
                            diff += 1 + (poly2.SegmentCount - numSegOn2) / 2;
                        }
                    } while (_nbroots == 0 && diff != 0);
                }
                _bSupVector.Value(2) = anBsupVector;
            }
        }
    }

    /// <summary>Searches the root from (Uo, Vo) inside the given bounds.</summary>
    public void Perform(double uo, double vo, double uInf, double vInf, double uSup, double vSup)
    {
        _done = true;

        _bInfVector.Value(1) = uInf;
        _bInfVector.Value(2) = vInf;
        _bSupVector.Value(1) = uSup;
        _bSupVector.Value(2) = vSup;
        _startingPoint.Value(1) = uo;
        _startingPoint.Value(2) = vo;

        MathPerform();
    }

    /// <summary>The number of roots found (0 or 1).</summary>
    public int RootCount => (_nbroots);

    /// <summary>The parameters of the root on both curves.</summary>
    public void Roots(out double u, out double v)
    {
        u = _root.Value(1);
        v = _root.Value(2);
    }

    private void MathPerform()
    {
        FunctionSetRoot fct = new(_fctDist, _toleranceVector, 60);
        fct.Perform(_fctDist, _startingPoint, _bInfVector, _bSupVector);

        if (fct.IsDone)
        {
            fct.Root(_root);
            _nbroots = 1;
            math_Vector xy = new(1, 2);
            _fctDist.Value(_root, xy);
            double dist2 = ((xy[1] * xy[1] + xy[2] * xy[2]));

            if (dist2 > _tol)
            {
                _nbroots = 0;
            }
        }
        else
        {
            _anErrorOccurred = true;
            _nbroots = 0;
        }
    }

    /// <summary>True if the root search failed (not just found no root) at least once.</summary>
    public bool AnErrorOccurred() => _anErrorOccurred;
}
