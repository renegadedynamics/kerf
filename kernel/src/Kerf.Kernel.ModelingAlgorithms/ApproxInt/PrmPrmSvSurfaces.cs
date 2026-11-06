// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_PrmPrmSvSurfaces.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// The points and tangents of the intersection line of two parametric surfaces, refined by
/// <see cref="Int2S{ThePSurface, ThePSurfaceTool}"/>.
/// </summary>
/// <remarks>
/// OCCT's generic file ApproxInt_PrmPrmSvSurfaces.gxx, ported once as a generic class (decision 607). OCCT's macros
/// ApproxInt_TheInt2S and ApproxInt_TheFunctionOfTheInt2S are, in every instantiation (GeomInt's and BRepApprox's),
/// IntImp_Int2S and IntImp_ZerParFunc over the same surface and tool, so the body names them. TheLine is never used
/// by the body and is dropped. OCCT's instantiations are sealed subclasses.
/// </remarks>
internal class PrmPrmSvSurfaces<ThePSurface, ThePSurfaceTool> : SvSurfaces
    where ThePSurfaceTool : IZerCSParPSurfaceTool<ThePSurface>
{
    private const double Toltangency = 0.0000000001;

    private Point2D _myParOnS1;
    private Point2D _myParOnS2;
    private Point3D _myPnt;
    private Vector2D _myTguv1;
    private Vector2D _myTguv2;
    private Vector3D _myTg;
    private bool _myIsTangent;
    private bool _myHasBeenComputed;
    private Point2D _myParOnS1bis;
    private Point2D _myParOnS2bis;
    private Point3D _myPntbis;
    private Vector2D _myTguv1bis;
    private Vector2D _myTguv2bis;
    private Vector3D _myTgbis;
    private bool _myIsTangentbis;
    private bool _myHasBeenComputedbis;
    private readonly Int2S<ThePSurface, ThePSurfaceTool> _myIntersectionOn2S;

    private ThePSurface MySurf1 => _myIntersectionOn2S.Function().AuxillarSurface1();

    private ThePSurface MySurf2 => _myIntersectionOn2S.Function().AuxillarSurface2();

    public PrmPrmSvSurfaces(ThePSurface surf1, ThePSurface surf2)
    {
        _myParOnS1 = new Point2D();
        _myParOnS2 = new Point2D();
        _myPnt = new Point3D();
        _myTguv1 = new Vector2D();
        _myTguv2 = new Vector2D();
        _myTg = new Vector3D();
        _myIsTangent = false;
        _myHasBeenComputed = false;
        _myParOnS1bis = new Point2D();
        _myParOnS2bis = new Point2D();
        _myPntbis = new Point3D();
        _myTguv1bis = new Vector2D();
        _myTguv2bis = new Vector2D();
        _myTgbis = new Vector3D();
        _myIsTangentbis = false;
        _myHasBeenComputedbis = false;
        _myIntersectionOn2S = new Int2S<ThePSurface, ThePSurfaceTool>(surf1, surf2, Toltangency);
    }

    /// <summary>
    /// Computes the point on the curve, the 3D and 2D tangents of the curve and the parameters on the surfaces.
    /// </summary>
    public override bool Compute(ref double u1, ref double v1, ref double u2, ref double v2, ref Point3D p,
                                 ref Vector3D tg, ref Vector2D tguv1, ref Vector2D tguv2)
    {
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;

        if (_myHasBeenComputed)
        {
            if ((_myParOnS1.X == u1) && (_myParOnS1.Y == v1) && (_myParOnS2.X == u2)
                && (_myParOnS2.Y == v2))
            {
                return (_myIsTangent); // OCCT: a cached result leaves the outputs as given
            }
            else if (_myHasBeenComputedbis == false)
            {
                _myTgbis = _myTg;
                _myTguv1bis = _myTguv1;
                _myTguv2bis = _myTguv2;
                _myPntbis = _myPnt;
                _myParOnS1bis = _myParOnS1;
                _myParOnS2bis = _myParOnS2;
                _myIsTangentbis = _myIsTangent;
                _myHasBeenComputedbis = _myHasBeenComputed;
            }
        }
        if (_myHasBeenComputedbis)
        {
            if ((_myParOnS1bis.X == u1) && (_myParOnS1bis.Y == v1) && (_myParOnS2bis.X == u2)
                && (_myParOnS2bis.Y == v2))
            {
                // C# forbids a nested local named as a later outer one (TV): OCCT's TV is TV0 here
                Vector3D tv0 = _myTg;
                Vector2D TV1 = _myTguv1;
                Vector2D TV2 = _myTguv2;
                Point3D tp = _myPnt;
                Point2D tp1 = _myParOnS1;
                Point2D tp2 = _myParOnS2;
                bool tb = _myIsTangent;

                _myTg = _myTgbis;
                _myTguv1 = _myTguv1bis;
                _myTguv2 = _myTguv2bis;
                _myPnt = _myPntbis;
                _myParOnS1 = _myParOnS1bis;
                _myParOnS2 = _myParOnS2bis;
                _myIsTangent = _myIsTangentbis;

                _myTgbis = tv0;
                _myTguv1bis = TV1;
                _myTguv2bis = TV2;
                _myPntbis = tp;
                _myParOnS1bis = tp1;
                _myParOnS2bis = tp2;
                _myIsTangentbis = tb;

                return (_myIsTangent); // OCCT: a cached result leaves the outputs as given
            }
        }

        _myIsTangent = true;

        NCollection_Array1<double> param = new(1, 4);
        param[1] = u1;
        param[2] = v1;
        param[3] = u2;
        param[4] = v2;
        FunctionSetRoot rsnld = new(_myIntersectionOn2S.Function());
        _myIntersectionOn2S.Perform(param, rsnld);
        if (!_myIntersectionOn2S.IsDone)
        {
            _myHasBeenComputed = _myHasBeenComputedbis = false;
            return (false);
        }
        if (_myIntersectionOn2S.IsEmpty)
        {
            _myIsTangent = false;
            _myHasBeenComputed = _myHasBeenComputedbis = false;
            return (false);
        }
        _myHasBeenComputed = true;
        _myPnt = p = _myIntersectionOn2S.Point().Value();

        _myIntersectionOn2S.Point().Parameters(out u1, out v1, out u2, out v2);
        _myParOnS1 = _myParOnS1.WithCoord(tu1, tv1);
        _myParOnS2 = _myParOnS2.WithCoord(tu2, tv2);

        if (_myIntersectionOn2S.IsTangent)
        {
            _myIsTangent = false;
            _myHasBeenComputed = _myHasBeenComputedbis = false;
            return (false);
        }
        _myTg = tg = new Vector3D(_myIntersectionOn2S.Direction());
        _myTguv1 = tguv1 = new Vector2D(_myIntersectionOn2S.DirectionOnS1());
        _myTguv2 = tguv2 = new Vector2D(_myIntersectionOn2S.DirectionOnS2());

        //----------------------------------------------------------------------
        //-- If ( Tg ) TU and TV are normalised
        //--
        //-- Tg = DeltaU * TU + DeltaV * TV
        //--
        //-- so:   Tg.TU = DeltaU TU.TU + DeltaV TU.TV
        //--       Tg.TV = DeltaU TV.TU + DeltaV TV.TV
        //--
        //-- Hence:
        //--
        //--               Tg.TU TV.TV  - Tg.TV * TU.TV
        //--   DeltaU = -------------------------------
        //--               TU.TU TV.TV  - (TU.TV)**2
        //--
        //--               Tg.TV TU.TU  - Tg.TU * TU.TV
        //--   DeltaV = -------------------------------
        //--               TU.TU TV.TV  - (TU.TV)**2

        tg = tg.Normalized();
        _myTg = tg;

        double deltaU, deltaV;
        Vector3D tu, tv;
        double tutv, tgTU, tgTV, tutu, tvtv, dis;
        //------------------------------------------------------------
        //-- Tguv1
        //--
        ThePSurfaceTool.D1(MySurf1, u1, v1, out _, out tu, out tv);

        tutu = tu.Dot(tu);
        tvtv = tv.Dot(tv);
        tutv = tu.Dot(tv);
        tgTU = tg.Dot(tu);
        tgTV = tg.Dot(tv);
        dis = tutu * tvtv - tutv * tutv;
        if (Math.Abs(dis) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            _myIsTangent = false;
            _myHasBeenComputed = _myHasBeenComputedbis = false;
            return (false);
        }

        deltaU = (tgTU * tvtv - tgTV * tutv) / dis;
        deltaV = (tgTV * tutu - tgTU * tutv) / dis;

        tguv1 = tguv1.WithCoord(deltaU, deltaV);
        _myTguv1 = tguv1;

        //------------------------------------------------------------
        //-- Tguv2
        //--
        ThePSurfaceTool.D1(MySurf2, u2, v2, out _, out tu, out tv);

        tutu = tu.Dot(tu);
        tvtv = tv.Dot(tv);
        tutv = tu.Dot(tv);
        tgTU = tg.Dot(tu);
        tgTV = tg.Dot(tv);
        dis = tutu * tvtv - tutv * tutv;
        if (Math.Abs(dis) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            _myIsTangent = false;
            _myHasBeenComputed = _myHasBeenComputedbis = false;
            return (false);
        }

        deltaU = (tgTU * tvtv - tgTV * tutv) / dis;
        deltaV = (tgTV * tutu - tgTU * tutv) / dis;

        tguv2 = tguv2.WithCoord(deltaU, deltaV);
        _myTguv2 = tguv2;

        return (true);
    }

    public override void Pnt(double u1, double v1, double u2, double v2, ref Point3D p)
    {
        Point3D aP = new();
        Vector3D t = new();
        Vector2D ts1 = new(), ts2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref t, ref ts1, ref ts2);
        p = _myPnt;
    }

    /// <summary>Computes the point on the curve and the parameters on the surfaces.</summary>
    public override bool SeekPoint(double u1, double v1, double u2, double v2, ref PntOn2S point)
    {
        Point3D p = new();
        Vector3D t = new();
        Vector2D ts1 = new(), ts2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        if (!Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref p, ref t, ref ts1, ref ts2))
        {
            return false;
        }

        point.SetValue(p, tu1, tv1, tu2, tv2);
        return true;
    }

    public override bool Tangency(double u1, double v1, double u2, double v2, ref Vector3D T)
    {
        Point3D p = new();
        Vector3D aT = new();
        Vector2D ts1 = new(), ts2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref p, ref aT, ref ts1, ref ts2);
        T = _myTg;
        return (t);
    }

    public override bool TangencyOnSurf1(double u1, double v1, double u2, double v2, ref Vector2D T)
    {
        Point3D p = new();
        Vector3D aT = new();
        Vector2D ts1 = new(), ts2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref p, ref aT, ref ts1, ref ts2);
        T = _myTguv1;
        return (t);
    }

    public override bool TangencyOnSurf2(double u1, double v1, double u2, double v2, ref Vector2D T)
    {
        Point3D p = new();
        Vector3D aT = new();
        Vector2D ts1 = new(), ts2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref p, ref aT, ref ts1, ref ts2);
        T = _myTguv2;
        return (t);
    }
}
