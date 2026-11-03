// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_MakeApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_MakeApprox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// Approximates a plate surface by a B-spline surface (AdvApp2Var), with the G0 or G1 criterion of the plate's
/// constraints, a criterion given by the caller, or none.
/// </summary>
/// <remarks>
/// The approximation runs in the constructor. <see cref="Surface"/> freezes the surface it returns (the same object
/// each time); ported code that goes on changing it reads <see cref="InternalSurface"/>.
/// </remarks>
public class MakeApprox
{
    private readonly PlateSurface _plate;
    private readonly BSplineSurface? _surface;
    private readonly double _appError;
    private readonly double _critError;

    private sealed class GeomPlate_MakeApprox_Eval : EvaluatorFunc2Var
    {
        private readonly Surface _surf;

        public GeomPlate_MakeApprox_Eval(Surface surf)
        {
            _surf = surf;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> uStartEnd, ReadOnlySpan<double> vStartEnd,
                                      int favorIso, double constParam, int nbParams, ReadOnlySpan<double> parameters,
                                      int uOrder, int vOrder, Span<double> result, out int errorCode)
        {
            errorCode = 0;
            int idim, jpar;
            double upar, vpar;

            // Dimension incorrecte
            if (dimension != 3)
            {
                errorCode = 1;
            }

            // Parametres incorrects
            if (favorIso == 1)
            {
                upar = constParam;
                if ((upar < uStartEnd[0]) || (upar > uStartEnd[1]))
                {
                    errorCode = 2;
                }
                for (jpar = 1; jpar <= nbParams; jpar++)
                {
                    vpar = parameters[jpar - 1];
                    if ((vpar < vStartEnd[0]) || (vpar > vStartEnd[1]))
                    {
                        errorCode = 2;
                    }
                }
            }
            else
            {
                vpar = constParam;
                if ((vpar < vStartEnd[0]) || (vpar > vStartEnd[1]))
                {
                    errorCode = 2;
                }
                for (jpar = 1; jpar <= nbParams; jpar++)
                {
                    upar = parameters[jpar - 1];
                    if ((upar < uStartEnd[0]) || (upar > uStartEnd[1]))
                    {
                        errorCode = 2;
                    }
                }
            }

            // Initialisation
            for (idim = 1; idim <= dimension; idim++)
            {
                for (jpar = 1; jpar <= nbParams; jpar++)
                {
                    result[idim - 1 + (jpar - 1) * dimension] = 0.0;
                }
            }

            int order = uOrder + vOrder;
            Point3D pnt;
            Vector3D v1, v2, v3, v4, v5;

            if (favorIso == 1)
            {
                upar = constParam;
                switch (order)
                {
                    case 0:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            vpar = parameters[jpar - 1];
                            pnt = _surf.Value(upar, vpar);
                            result[(jpar - 1) * dimension] = pnt.X;
                            result[1 + (jpar - 1) * dimension] = pnt.Y;
                            result[2 + (jpar - 1) * dimension] = pnt.Z;
                        }
                        break;
                    case 1:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            vpar = parameters[jpar - 1];
                            _surf.D1(upar, vpar, out pnt, out v1, out v2);
                            if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v1.X;
                                result[1 + (jpar - 1) * dimension] = v1.Y;
                                result[2 + (jpar - 1) * dimension] = v1.Z;
                            }
                            else
                            {
                                result[(jpar - 1) * dimension] = v2.X;
                                result[1 + (jpar - 1) * dimension] = v2.Y;
                                result[2 + (jpar - 1) * dimension] = v2.Z;
                            }
                        }
                        break;
                    case 2:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            vpar = parameters[jpar - 1];
                            _surf.D2(upar, vpar, out pnt, out v1, out v2, out v3, out v4, out v5);
                            if (uOrder == 2)
                            {
                                result[(jpar - 1) * dimension] = v3.X;
                                result[1 + (jpar - 1) * dimension] = v3.Y;
                                result[2 + (jpar - 1) * dimension] = v3.Z;
                            }
                            else if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v5.X;
                                result[1 + (jpar - 1) * dimension] = v5.Y;
                                result[2 + (jpar - 1) * dimension] = v5.Z;
                            }
                            else if (uOrder == 0)
                            {
                                result[(jpar - 1) * dimension] = v4.X;
                                result[1 + (jpar - 1) * dimension] = v4.Y;
                                result[2 + (jpar - 1) * dimension] = v4.Z;
                            }
                        }
                        break;
                }
            }
            else
            {
                vpar = constParam;
                switch (order)
                {
                    case 0:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            upar = parameters[jpar - 1];
                            pnt = _surf.Value(upar, vpar);
                            result[(jpar - 1) * dimension] = pnt.X;
                            result[1 + (jpar - 1) * dimension] = pnt.Y;
                            result[2 + (jpar - 1) * dimension] = pnt.Z;
                        }
                        break;
                    case 1:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            upar = parameters[jpar - 1];
                            _surf.D1(upar, vpar, out pnt, out v1, out v2);
                            if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v1.X;
                                result[1 + (jpar - 1) * dimension] = v1.Y;
                                result[2 + (jpar - 1) * dimension] = v1.Z;
                            }
                            else
                            {
                                result[(jpar - 1) * dimension] = v2.X;
                                result[1 + (jpar - 1) * dimension] = v2.Y;
                                result[2 + (jpar - 1) * dimension] = v2.Z;
                            }
                        }
                        break;
                    case 2:
                        for (jpar = 1; jpar <= nbParams; jpar++)
                        {
                            upar = parameters[jpar - 1];
                            _surf.D2(upar, vpar, out pnt, out v1, out v2, out v3, out v4, out v5);
                            if (uOrder == 2)
                            {
                                result[(jpar - 1) * dimension] = v3.X;
                                result[1 + (jpar - 1) * dimension] = v3.Y;
                                result[2 + (jpar - 1) * dimension] = v3.Z;
                            }
                            else if (uOrder == 1)
                            {
                                result[(jpar - 1) * dimension] = v5.X;
                                result[1 + (jpar - 1) * dimension] = v5.Y;
                                result[2 + (jpar - 1) * dimension] = v5.Z;
                            }
                            else if (uOrder == 0)
                            {
                                result[(jpar - 1) * dimension] = v4.X;
                                result[1 + (jpar - 1) * dimension] = v4.Y;
                                result[2 + (jpar - 1) * dimension] = v4.Z;
                            }
                        }
                        break;
                }
            }
        }
    }

    /// <summary>Approximates <paramref name="surfPlate"/> under the criterion <paramref name="plateCrit"/>.</summary>
    /// <exception cref="ConstructionErrorException">The approximation fails.</exception>
    public MakeApprox(PlateSurface surfPlate, Criterion plateCrit, double tol3d, int nbmax,
                                int dgmax, Continuity continuity = C1, double enlargeCoeff = 1.1)
    {
        _plate = surfPlate;

        double u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
        _plate.RealBounds(out u0, out u1, out v0, out v1);
        u0 = enlargeCoeff * u0;
        u1 = enlargeCoeff * u1;
        v0 = enlargeCoeff * v0;
        v1 = enlargeCoeff * v1;

        int nb1 = 0, nb2 = 0, nb3 = 1;
        NCollection_HArray1<double> nul1 = new NCollection_HArray1<double>(1, 1);
        nul1.Init(0.0);
        NCollection_HArray2<double> nul2 = new NCollection_HArray2<double>(1, 1, 1, 4);
        nul2.Init(0.0);
        NCollection_HArray1<double> eps3D = new NCollection_HArray1<double>(1, 1);
        eps3D.Init(tol3d);
        NCollection_HArray2<double> epsfr = new NCollection_HArray2<double>(1, 1, 1, 4);
        epsfr.Init(tol3d);
        IsoType myType = IsoType.IsoV;
        int myPrec = 0;

        DichoCutting myDec = new DichoCutting();

        GeomPlate_MakeApprox_Eval ev = new GeomPlate_MakeApprox_Eval(_plate);
        ApproxAFunc2Var appPlate = new ApproxAFunc2Var(nb1, nb2, nb3, nul1, nul1, eps3D, nul2, nul2,
                                                                            epsfr, u0, u1, v0, v1, myType, continuity,
                                                                            continuity, myPrec, dgmax, dgmax, nbmax, ev,
                                                                            plateCrit, myDec, myDec);
        _surface = appPlate.Surface(1);
        _appError = appPlate.MaxError(3, 1);
        _critError = appPlate.CritError(3, 1);
    }

    /// <summary>
    /// Approximates <paramref name="surfPlate"/> under the G0 (<paramref name="critOrder"/> 0) or G1 (1) criterion of
    /// the plate's constraints, or none (-1). <paramref name="dmax"/> is the largest distance to the constraints:
    /// the criterion's threshold is at least ten times it.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The approximation fails.</exception>
    public MakeApprox(PlateSurface surfPlate, double tol3d, int nbmax, int dgmax, double dmax,
                                int critOrder = 0, Continuity continuity = C1, double enlargeCoeff = 1.1)
    {
        _plate = surfPlate;

        NCollection_Sequence<Xy> seq2d = new NCollection_Sequence<Xy>();
        NCollection_Sequence<Xyz> seq3d = new NCollection_Sequence<Xyz>();

        if (critOrder >= 0)
        {
            //    contraintes 2d d'ordre 0
            _plate.Constraints(seq2d);

            //    contraintes 3d correspondantes sur plate
            int i, nbp = seq2d.Length();
            for (i = 1; i <= nbp; i++)
            {
                Xy p2d = seq2d.Value(i);
                Point3D pp;
                Vector3D v1h, v2h, v3h;
                if (critOrder == 0)
                {
                    //    a l'ordre 0
                    _plate.D0(p2d.X(), p2d.Y(), out pp);
                    Xyz p3d = new Xyz(pp.X, pp.Y, pp.Z);
                    seq3d.Append(p3d);
                }
                else
                {
                    //    a l'ordre 1
                    _plate.D1(p2d.X(), p2d.Y(), out pp, out v1h, out v2h);
                    v3h = v1h ^ v2h;
                    Xyz p3d = new Xyz(v3h.X, v3h.Y, v3h.Z);
                    seq3d.Append(p3d);
                }
            }
        }

        double u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
        _plate.RealBounds(out u0, out u1, out v0, out v1);
        u0 = enlargeCoeff * u0;
        u1 = enlargeCoeff * u1;
        v0 = enlargeCoeff * v0;
        v1 = enlargeCoeff * v1;

        double seuil = tol3d;
        if (critOrder == 0 && tol3d < 10 * dmax)
        {
            seuil = 10 * dmax;
        }
        if (critOrder == 1 && tol3d < 10 * dmax)
        {
            seuil = 10 * dmax;
        }
        int nb1 = 0, nb2 = 0, nb3 = 1;
        NCollection_HArray1<double> nul1 = new NCollection_HArray1<double>(1, 1);
        nul1.Init(0.0);
        NCollection_HArray2<double> nul2 = new NCollection_HArray2<double>(1, 1, 1, 4);
        nul2.Init(0.0);
        NCollection_HArray1<double> eps3D = new NCollection_HArray1<double>(1, 1);
        eps3D.Init(tol3d);
        NCollection_HArray2<double> epsfr = new NCollection_HArray2<double>(1, 1, 1, 4);
        epsfr.Init(tol3d);

        IsoType myType = IsoType.IsoV;
        int myPrec = 0;

        DichoCutting myDec = new DichoCutting();

        if (critOrder == -1)
        {
            myPrec = 1;
            GeomPlate_MakeApprox_Eval ev = new GeomPlate_MakeApprox_Eval(_plate);
            ApproxAFunc2Var appPlate = new ApproxAFunc2Var(nb1, nb2, nb3, nul1, nul1, eps3D, nul2,
                                                                                nul2, epsfr, u0, u1, v0, v1, myType,
                                                                                continuity, continuity, myPrec, dgmax,
                                                                                dgmax, nbmax, ev, myDec, myDec);
            _surface = appPlate.Surface(1);
            _appError = appPlate.MaxError(3, 1);
            _critError = 0.0;
        }
        else if (critOrder == 0)
        {
            PlateG0Criterion crit0 = new PlateG0Criterion(seq2d, seq3d, seuil);
            GeomPlate_MakeApprox_Eval ev = new GeomPlate_MakeApprox_Eval(_plate);
            ApproxAFunc2Var appPlate = new ApproxAFunc2Var(nb1, nb2, nb3, nul1, nul1, eps3D, nul2,
                                                                                nul2, epsfr, u0, u1, v0, v1, myType,
                                                                                continuity, continuity, myPrec, dgmax,
                                                                                dgmax, nbmax, ev, crit0, myDec, myDec);
            _surface = appPlate.Surface(1);
            _appError = appPlate.MaxError(3, 1);
            _critError = appPlate.CritError(3, 1);
        }
        else if (critOrder == 1)
        {
            PlateG1Criterion crit1 = new PlateG1Criterion(seq2d, seq3d, seuil);
            GeomPlate_MakeApprox_Eval ev = new GeomPlate_MakeApprox_Eval(_plate);
            ApproxAFunc2Var appPlate = new ApproxAFunc2Var(nb1, nb2, nb3, nul1, nul1, eps3D, nul2,
                                                                                nul2, epsfr, u0, u1, v0, v1, myType,
                                                                                continuity, continuity, myPrec, dgmax,
                                                                                dgmax, nbmax, ev, crit1, myDec, myDec);
            _surface = appPlate.Surface(1);
            _appError = appPlate.MaxError(3, 1);
            _critError = appPlate.CritError(3, 1);
        }
        // OCCT: another CritOrder leaves the surface null and the errors uninitialised; the port's are 0
    }

    /// <summary>The approximating surface (frozen; null for a criterion order other than -1, 0 or 1).</summary>
    public BSplineSurface? Surface()
    {
        _surface?.Freeze();
        return _surface;
    }

    /// <summary>The approximating surface, not frozen, for ported code.</summary>
    internal BSplineSurface? InternalSurface() => _surface;

    public double ApproxError() => _appError;

    public double CriterionError() => _critError;
}
