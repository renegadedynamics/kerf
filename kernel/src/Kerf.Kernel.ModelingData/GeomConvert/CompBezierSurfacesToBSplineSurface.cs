// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompBezierSurfacesToBSplineSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompBezierSurfacesToBSplineSurface.lxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompBezierSurfacesToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a grid of adjacent non-rational Bezier surfaces into the data of one B-spline surface. Beziers(i, j)
/// is the patch of U index i (row) and V index j (column); consecutive patches must share their bounding
/// iso-curves with compatible parameterisations (not checked). The results are read with Poles, UKnots, VKnots,
/// UMultiplicities, VMultiplicities, UDegree and VDegree, and passed to a Geom_BSplineSurface constructor.
/// </summary>
/// <remarks>
/// The grid's patches are not modified: OCCT raises their degrees in place to the grid's, the port raises copies.
/// </remarks>
internal sealed class CompBezierSurfacesToBSplineSurface
{
    private int _uDegree;
    private int _vDegree;
    private NCollection_HArray1<int> _vMults = null!;
    private NCollection_HArray1<int> _uMults = null!;
    private NCollection_HArray1<double> _uKnots;
    private NCollection_HArray1<double> _vKnots;
    private NCollection_HArray2<Point3D> _poles = null!;
    private bool _isrational;
    private bool _done;

    /// <summary>
    /// The data of a C0 "piecewise Bezier" B-spline surface equivalent to the grid: knots 0, 1, 2, ... in each
    /// direction, the degrees the highest of the grid's, inner multiplicities equal to the degrees.
    /// </summary>
    /// <exception cref="MethodNotImplementedException">A patch is rational.</exception>
    public CompBezierSurfacesToBSplineSurface(NCollection_Array2<BezierSurface> beziers)
    {
        int ii;
        _done = true;
        // Choice of knots
        _uKnots = new NCollection_HArray1<double>(1, beziers.ColLength() + 1);
        for (ii = 0; ii < _uKnots.Length(); ii++)
        {
            _uKnots.SetValue(ii + 1, ii);
        }

        _vKnots = new NCollection_HArray1<double>(1, beziers.RowLength() + 1);
        for (ii = 0; ii < _vKnots.Length(); ii++)
        {
            _vKnots.SetValue(ii + 1, ii);
        }

        // Compute the Poles
        Perform(beziers);
    }

    /// <summary>
    /// The data of a B-spline surface equivalent to the grid, with knots spaced by the patches' pseudo-lengths
    /// along the iso-curves at 0.3, and as many inner knots removed as Tolerance allows (C0 is imposed without
    /// check). With RemoveKnots false, the knots are kept with multiplicity at least 1.
    /// </summary>
    /// <exception cref="MethodNotImplementedException">A patch is rational.</exception>
    /// <remarks>
    /// OCCT: the V knot removal loop tests the U multiplicity (occt-issues.md), so V knots are only removed when
    /// the last U removal failed, and then the loop may not end.
    /// </remarks>
    public CompBezierSurfacesToBSplineSurface(NCollection_Array2<BezierSurface> beziers,
                                                          double tolerance,
                                                          bool removeKnots = true)
    {
        int ii, jj, multU = 0, multV, minus;
        bool ok;
        Vector3D vec;
        double v1, v2, v3, ratio, l1, l2, tol, val;
        Point3D p1, p2, p3;
        Curve3D firstCurve, secondCurve;

        _done = true;

        // Choice of knots

        _uKnots = new NCollection_HArray1<double>(1, beziers.ColLength() + 1);
        _vKnots = new NCollection_HArray1<double>(1, beziers.RowLength() + 1);

        // --> in U
        _uKnots.SetValue(1, 0);
        jj = _vKnots.Length() / 2;
        firstCurve = beziers[1, jj].VIso(0.3);
        firstCurve.D0(0, out p1);
        firstCurve.D0(0.5, out p2);
        firstCurve.D1(1, out p3, out vec);

        l1 = p1.DistanceTo(p2) + p2.DistanceTo(p3);
        _uKnots.SetValue(2, l1);

        v1 = vec.Magnitude;
        // if the parameterization is too strange we keep the pseudo-length
        if ((v1 > 1000 * l1) || (v1 < l1 * 1.0e-3))
        {
            v1 = l1;
        }

        for (ii = 2; ii < _uKnots.Length(); ii++)
        {
            secondCurve = beziers[ii, jj].VIso(0.3);
            secondCurve.D1(0, out p1, out vec);
            v2 = vec.Magnitude;
            secondCurve.D0(0.5, out p2);
            secondCurve.D1(1, out p3, out vec);
            v3 = vec.Magnitude;
            l2 = p1.DistanceTo(p2) + p2.DistanceTo(p3);

            // Compute the ratio, avoiding degenerate cases...
            if ((v2 > 1000 * l2) || (v2 < l2 * 1.0e-3))
            {
                v2 = l2;
            }

            if ((v3 > 1000 * l2) || (v3 < l2 * 1.0e-3))
            {
                v3 = l2;
            }

            ratio = 1;
            if ((v1 > global::Kerf.Engine.Foundation.Precision.Confusion()) && (v2 > global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                ratio = v2 / v1;
            }

            if ((ratio < global::Kerf.Engine.Foundation.Precision.Confusion()) || (ratio > 1 / global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                ratio = 1;
            }

            // Deduce a new knot
            val = _uKnots.Value(ii);
            _uKnots.SetValue(ii + 1, val + ratio * (val - _uKnots.Value(ii - 1)));

            // And off we go again, for another round
            l1 = l2;
            v1 = v3;
            firstCurve = secondCurve;
        }

        // --> en V
        _vKnots.SetValue(1, 0);
        ii = _uKnots.Length() / 2;
        firstCurve = beziers[ii, 1].UIso(0.3);
        firstCurve.D0(0, out p1);
        firstCurve.D0(0.5, out p2);
        firstCurve.D1(1, out p3, out vec);

        l1 = p1.DistanceTo(p2) + p2.DistanceTo(p3);
        _vKnots.SetValue(2, l1);

        v1 = vec.Magnitude;
        // if the parameterization is too strange we keep the pseudo-length
        if ((v1 > 1000 * l1) || (v1 < l1 * 1.0e-3))
        {
            v1 = l1;
        }

        for (jj = 2; jj < _vKnots.Length(); jj++)
        {
            secondCurve = beziers[ii, jj].UIso(0.3);
            secondCurve.D1(0, out p1, out vec);
            v2 = vec.Magnitude;
            secondCurve.D0(0.5, out p2);
            secondCurve.D1(1, out p3, out vec);
            v3 = vec.Magnitude;
            l2 = p1.DistanceTo(p2) + p2.DistanceTo(p3);

            // Compute the ratio, avoiding degenerate cases...
            if ((v2 > 1000 * l2) || (v2 < l2 * 1.0e-3))
            {
                v2 = l2;
            }

            if ((v3 > 1000 * l2) || (v3 < l2 * 1.0e-3))
            {
                v3 = l2;
            }

            ratio = 1;
            if ((v1 > global::Kerf.Engine.Foundation.Precision.Confusion()) && (v2 > global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                ratio = v2 / v1;
            }

            if ((ratio < global::Kerf.Engine.Foundation.Precision.Confusion()) || (ratio > 1 / global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                ratio = 1;
            }

            // Deduce a new knot
            val = _vKnots.Value(jj);
            _vKnots.SetValue(jj + 1, val + ratio * (val - _vKnots.Value(jj - 1)));

            // And off we go again, for another round
            l1 = l2;
            v1 = v3;
            firstCurve = secondCurve;
        }

        // Compute the Poles
        Perform(beziers);

        // Reduction de la multiplicite
        var surface = new BSplineSurface(_poles,
                                              _uKnots,
                                              _vKnots,
                                              _uMults,
                                              _vMults,
                                              _uDegree,
                                              _vDegree);

        if (removeKnots)
        {
            minus = 0;
        }
        else
        {
            minus = 1;
        }

        for (ii = _uKnots.Length() - 1; ii > 1; ii--)
        {
            ok = true;
            tol = tolerance / 2;
            multU = _uMults.Value(ii) - 1;
            for (; ok && multU > minus; multU--, tol /= 2)
            {
                ok = surface.RemoveUKnot(ii, multU, tol);
            }
        }

        for (ii = _vKnots.Length() - 1; ii > 1; ii--)
        {
            ok = true;
            tol = tolerance / 2;
            multV = _vMults.Value(ii) - 1;
            for (; ok && multU > minus; multV--, tol /= 2) // OCCT: tests multU, not multV
            {
                ok = surface.RemoveVKnot(ii, multV, tol);
            }
        }

        // The new fields have been computed....
        _poles = new NCollection_HArray2<Point3D>(surface.InternalPoles());
        _uMults = new NCollection_HArray1<int>(surface.InternalUMultiplicities());
        _vMults = new NCollection_HArray1<int>(surface.InternalVMultiplicities());
        _uKnots = new NCollection_HArray1<double>(surface.InternalUKnots());
        _vKnots = new NCollection_HArray1<double>(surface.InternalVKnots());
    }

    /// <summary>
    /// The data of a B-spline surface equivalent to the grid, with the given knots (one more than the patches in
    /// each direction), made of continuity UContinuity and VContinuity (C0 to C3) by knot removal where the
    /// deformation stays below Tolerance; IsDone is false where it doesn't.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// A continuity above C3 or G1/G2, or a continuity not below the degree in a direction with several patches.
    /// </exception>
    /// <exception cref="MethodNotImplementedException">A patch is rational.</exception>
    public CompBezierSurfacesToBSplineSurface(NCollection_Array2<BezierSurface> beziers,
                                                          NCollection_Array1<double> uKnots,
                                                          NCollection_Array1<double> vKnots,
                                                          Continuity uContinuity = C0,
                                                          Continuity vContinuity = C0,
                                                          double tolerance = 1.0e-4)
    {
        int decu = 0, decv = 0;
        bool ok;

        _done = true;

        // Retrieve the knots
        _uKnots = new NCollection_HArray1<double>(1, beziers.ColLength() + 1);
        _uKnots.Assign(uKnots);

        _vKnots = new NCollection_HArray1<double>(1, beziers.RowLength() + 1);
        _vKnots.Assign(vKnots);

        // Compute the Poles
        Perform(beziers);

        // Obtaining the correct continuity

        switch (uContinuity)
        {
            case C0:
                decu = 0;
                break;
            case C1:
                decu = 1;
                break;
            case C2:
                decu = 2;
                break;
            case C3:
                decu = 3;
                break;
            default:
                throw new ConstructionErrorException("GeomConvert_CompBezierSurfacesToBSpl:: UContinuity error");
        }

        switch (vContinuity)
        {
            case C0:
                decv = 0;
                break;
            case C1:
                decv = 1;
                break;
            case C2:
                decv = 2;
                break;
            case C3:
                decv = 3;
                break;
            default:
                throw new ConstructionErrorException("GeomConvert_CompBezierSurfacesToBSpl:: VContinuity error");
        }

        if ((decu > 0) || (decv > 0))
        {
            int ii;
            int multU = _uDegree - decu;
            Standard_ConstructionError_Raise_if(
                ((multU <= 0) && (_uKnots.Length() > 2)),
                "GeomConvert_CompBezierSurfacesToBSpl:: UContinuity or Udeg error");

            int multV = _vDegree - decv;
            Standard_ConstructionError_Raise_if(
                ((multV <= 0) && (_vKnots.Length() > 2)),
                "GeomConvert_CompBezierSurfacesToBSpl:: VContinuity or Vdeg error");

            var surface = new BSplineSurface(_poles,
                                                  _uKnots,
                                                  _vKnots,
                                                  _uMults,
                                                  _vMults,
                                                  _uDegree,
                                                  _vDegree);

            if (decu > 0)
            {
                for (ii = 2; ii < _uKnots.Length(); ii++)
                {
                    ok = surface.RemoveUKnot(ii, multU, tolerance);
                    if (!ok)
                    {
                        _done = ok;
                    }
                }
            }

            if (decv > 0)
            {
                for (ii = 2; ii < _vKnots.Length(); ii++)
                {
                    ok = surface.RemoveVKnot(ii, multV, tolerance);
                    if (!ok)
                    {
                        _done = ok;
                    }
                }
            }

            // The new fields have been computed....
            _poles = new NCollection_HArray2<Point3D>(surface.InternalPoles());
            _uMults = new NCollection_HArray1<int>(surface.InternalUMultiplicities());
            _vMults = new NCollection_HArray1<int>(surface.InternalVMultiplicities());
        }
    }

    /// <summary>Used by the constructors: degrees, poles and multiplicities of the grid for the current knots.</summary>
    private void Perform(NCollection_Array2<BezierSurface> beziers)
    {
        int iu, iv;

        // (1) Determine the degrees and whether the result is rational.
        _isrational = false;
        _uDegree = 1;
        _vDegree = 1;

        for (iu = beziers.LowerRow(); iu <= beziers.UpperRow(); iu++)
        {
            for (iv = beziers.LowerCol(); iv <= beziers.UpperCol(); iv++)
            {
                if (beziers[iu, iv].IsURational || beziers[iu, iv].IsVRational)
                {
                    _isrational = true;
                }

                _uDegree = (beziers[iu, iv].UDegree > _uDegree) ? beziers[iu, iv].UDegree : _uDegree;

                _vDegree = (beziers[iu, iv].VDegree > _vDegree) ? beziers[iu, iv].VDegree : _vDegree;
            }
        }

        Standard_NotImplemented_Raise_if(_isrational, "CompBezierSurfacesToBSpl : rational !");

        // (2) Loop over the patches  -----------------------------

        BezierSurface patch;
        int uIndex, vIndex, uindex, vindex, udeb, vdeb;
        int upol, vpol, ii;

        _poles = new NCollection_HArray2<Point3D>(
            1,
            (_uDegree + 1) * beziers.ColLength() - _uKnots.Length() + 2,
            1,
            (_vDegree + 1) * beziers.RowLength() - _vKnots.Length() + 2);

        for (iu = beziers.LowerRow(); iu <= beziers.UpperRow(); iu++)
        {
            uIndex = (iu - 1) * _uDegree + 1;
            for (iv = beziers.LowerCol(); iv <= beziers.UpperCol(); iv++)
            {
                // OCCT raises the degree of the caller's patch in place; the port raises a copy
                patch = (BezierSurface)beziers[iu, iv].Copy();
                vIndex = (iv - 1) * _vDegree + 1;

                // (2.1) Increase the degree
                patch.Increase(_uDegree, _vDegree);

                // (2.2) Poles to copy
                if (iu == 1)
                {
                    udeb = 1;
                }
                else
                {
                    udeb = 2;
                }

                if (iv == 1)
                {
                    vdeb = 1;
                }
                else
                {
                    vdeb = 2;
                }

                uindex = uIndex + udeb - 1;

                for (upol = udeb; upol <= _uDegree + 1; upol++, uindex++)
                {
                    vindex = vIndex + vdeb - 1;
                    for (vpol = vdeb; vpol <= _vDegree + 1; vpol++, vindex++)
                    {
                        _poles[uindex, vindex] = patch.Pole(upol, vpol);
                    }
                }

                // (2.3) Poles a sommer
                if (udeb == 2)
                {
                    vindex = vIndex + vdeb - 1;
                    for (vpol = vdeb; vpol <= _vDegree + 1; vpol++, vindex++)
                    {
                        _poles[uIndex, vindex] = new Point3D(_poles[uIndex, vindex].XYZ() + patch.Pole(1, vpol).Coord());
                    }
                }

                if (vdeb == 2)
                {
                    uindex = uIndex + udeb - 1;
                    for (upol = udeb; upol <= _uDegree + 1; upol++, uindex++)
                    {
                        _poles[uindex, vIndex] = new Point3D(_poles[uindex, vIndex].XYZ() + patch.Pole(upol, 1).Coord());
                    }
                }

                if (udeb == 2 && vdeb == 2)
                {
                    _poles[uIndex, vIndex] = new Point3D(_poles[uIndex, vIndex].XYZ() + patch.Pole(1, 1).Coord());
                }
            }
        }

        // (3) Elimination of redundancies
        // because in the previous loop we count:
        // - 2 times the poles associated with simple knots
        // - 4 times the poles associated with double knots (in U and V)

        // (3.1) Elimination in U
        for (uIndex = _uDegree + 1, ii = 2; ii < _uKnots.Length(); ii++, uIndex += _uDegree)
        {
            for (vpol = 1; vpol <= _poles.UpperCol(); vpol++)
            {
                _poles[uIndex, vpol] = new Point3D(_poles[uIndex, vpol].XYZ() * 0.5);
            }
        }

        // (3.2) Elimination en V
        for (vIndex = _vDegree + 1, ii = 2; ii < _vKnots.Length(); ii++, vIndex += _vDegree)
        {
            for (upol = 1; upol <= _poles.UpperRow(); upol++)
            {
                _poles[upol, vIndex] = new Point3D(_poles[upol, vIndex].XYZ() * 0.5);
            }
        }

        // (4) Init of multiplicities
        _uMults = new NCollection_HArray1<int>(1, _uKnots.Length());
        _uMults.Init(_uDegree);
        _uMults.SetValue(1, _uDegree + 1);
        _uMults.SetValue(_uMults.Upper(), _uDegree + 1);

        _vMults = new NCollection_HArray1<int>(1, _vKnots.Length());
        _vMults.Init(_vDegree);
        _vMults.SetValue(1, _vDegree + 1);
        _vMults.SetValue(_vMults.Upper(), _vDegree + 1);
    }

    /// <summary>The number of knots in U of the computed surface.</summary>
    public int UKnotCount => _uKnots.Length();

    /// <summary>The number of poles in U of the computed surface.</summary>
    public int UPoleCount => _poles.ColLength();

    /// <summary>The number of knots in V of the computed surface.</summary>
    public int VKnotCount => _vKnots.Length();

    /// <summary>The number of poles in V of the computed surface.</summary>
    public int VPoleCount => _poles.RowLength();

    /// <summary>A copy of the poles of the computed surface.</summary>
    public NCollection_HArray2<Point3D> Poles() => new(_poles);

    /// <summary>A copy of the U knots of the computed surface.</summary>
    public NCollection_HArray1<double> UKnots() => new(_uKnots);

    /// <summary>The U degree of the computed surface.</summary>
    public int UDegree() => _uDegree;

    /// <summary>A copy of the V knots of the computed surface.</summary>
    public NCollection_HArray1<double> VKnots() => new(_vKnots);

    /// <summary>The V degree of the computed surface.</summary>
    public int VDegree() => _vDegree;

    /// <summary>A copy of the U multiplicities of the computed surface.</summary>
    public NCollection_HArray1<int> UMultiplicities() => new(_uMults);

    /// <summary>A copy of the V multiplicities of the computed surface.</summary>
    public NCollection_HArray1<int> VMultiplicities() => new(_vMults);

    /// <summary>
    /// False when a continuity requested at construction could not be reached within the tolerance (the data are
    /// then those of the surface as far as it could be smoothed).
    /// </summary>
    public bool IsDone => _done;
}
