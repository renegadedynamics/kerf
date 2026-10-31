// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_PrmPrmSvSurfaces.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The points and tangents of the intersection line of two parametric surfaces, refined by
/// <see cref="IntImp_Int2S{ThePSurface, ThePSurfaceTool}"/>.
/// </summary>
/// <remarks>
/// OCCT's generic file ApproxInt_PrmPrmSvSurfaces.gxx, ported once as a generic class (decision 607). OCCT's macros
/// ApproxInt_TheInt2S and ApproxInt_TheFunctionOfTheInt2S are, in every instantiation (GeomInt's and BRepApprox's),
/// IntImp_Int2S and IntImp_ZerParFunc over the same surface and tool, so the body names them. TheLine is never used
/// by the body and is dropped. OCCT's instantiations are sealed subclasses.
/// </remarks>
public class ApproxInt_PrmPrmSvSurfaces<ThePSurface, ThePSurfaceTool> : ApproxInt_SvSurfaces
    where ThePSurfaceTool : IIntImp_ZerCSParPSurfaceTool<ThePSurface>
{
    private const double TOLTANGENCY = 0.0000000001;

    private gp_Pnt2d MyParOnS1;
    private gp_Pnt2d MyParOnS2;
    private gp_Pnt MyPnt;
    private gp_Vec2d MyTguv1;
    private gp_Vec2d MyTguv2;
    private gp_Vec MyTg;
    private bool MyIsTangent;
    private bool MyHasBeenComputed;
    private gp_Pnt2d MyParOnS1bis;
    private gp_Pnt2d MyParOnS2bis;
    private gp_Pnt MyPntbis;
    private gp_Vec2d MyTguv1bis;
    private gp_Vec2d MyTguv2bis;
    private gp_Vec MyTgbis;
    private bool MyIsTangentbis;
    private bool MyHasBeenComputedbis;
    private readonly IntImp_Int2S<ThePSurface, ThePSurfaceTool> MyIntersectionOn2S;

    private ThePSurface MySurf1 => MyIntersectionOn2S.Function().AuxillarSurface1();

    private ThePSurface MySurf2 => MyIntersectionOn2S.Function().AuxillarSurface2();

    public ApproxInt_PrmPrmSvSurfaces(ThePSurface Surf1, ThePSurface Surf2)
    {
        MyParOnS1 = new gp_Pnt2d();
        MyParOnS2 = new gp_Pnt2d();
        MyPnt = new gp_Pnt();
        MyTguv1 = new gp_Vec2d();
        MyTguv2 = new gp_Vec2d();
        MyTg = new gp_Vec();
        MyIsTangent = false;
        MyHasBeenComputed = false;
        MyParOnS1bis = new gp_Pnt2d();
        MyParOnS2bis = new gp_Pnt2d();
        MyPntbis = new gp_Pnt();
        MyTguv1bis = new gp_Vec2d();
        MyTguv2bis = new gp_Vec2d();
        MyTgbis = new gp_Vec();
        MyIsTangentbis = false;
        MyHasBeenComputedbis = false;
        MyIntersectionOn2S = new IntImp_Int2S<ThePSurface, ThePSurfaceTool>(Surf1, Surf2, TOLTANGENCY);
    }

    /// <summary>
    /// Computes the point on the curve, the 3D and 2D tangents of the curve and the parameters on the surfaces.
    /// </summary>
    public override bool Compute(ref double u1, ref double v1, ref double u2, ref double v2, ref gp_Pnt P,
                                 ref gp_Vec Tg, ref gp_Vec2d Tguv1, ref gp_Vec2d Tguv2)
    {
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;

        if (MyHasBeenComputed)
        {
            if ((MyParOnS1.X() == u1) && (MyParOnS1.Y() == v1) && (MyParOnS2.X() == u2)
                && (MyParOnS2.Y() == v2))
            {
                return (MyIsTangent); // OCCT: a cached result leaves the outputs as given
            }
            else if (MyHasBeenComputedbis == false)
            {
                MyTgbis = MyTg;
                MyTguv1bis = MyTguv1;
                MyTguv2bis = MyTguv2;
                MyPntbis = MyPnt;
                MyParOnS1bis = MyParOnS1;
                MyParOnS2bis = MyParOnS2;
                MyIsTangentbis = MyIsTangent;
                MyHasBeenComputedbis = MyHasBeenComputed;
            }
        }
        if (MyHasBeenComputedbis)
        {
            if ((MyParOnS1bis.X() == u1) && (MyParOnS1bis.Y() == v1) && (MyParOnS2bis.X() == u2)
                && (MyParOnS2bis.Y() == v2))
            {
                // C# forbids a nested local named as a later outer one (TV): OCCT's TV is TV0 here
                gp_Vec TV0 = MyTg;
                gp_Vec2d TV1 = MyTguv1;
                gp_Vec2d TV2 = MyTguv2;
                gp_Pnt TP = MyPnt;
                gp_Pnt2d TP1 = MyParOnS1;
                gp_Pnt2d TP2 = MyParOnS2;
                bool TB = MyIsTangent;

                MyTg = MyTgbis;
                MyTguv1 = MyTguv1bis;
                MyTguv2 = MyTguv2bis;
                MyPnt = MyPntbis;
                MyParOnS1 = MyParOnS1bis;
                MyParOnS2 = MyParOnS2bis;
                MyIsTangent = MyIsTangentbis;

                MyTgbis = TV0;
                MyTguv1bis = TV1;
                MyTguv2bis = TV2;
                MyPntbis = TP;
                MyParOnS1bis = TP1;
                MyParOnS2bis = TP2;
                MyIsTangentbis = TB;

                return (MyIsTangent); // OCCT: a cached result leaves the outputs as given
            }
        }

        MyIsTangent = true;

        NCollection_Array1<double> Param = new(1, 4);
        Param[1] = u1;
        Param[2] = v1;
        Param[3] = u2;
        Param[4] = v2;
        math_FunctionSetRoot Rsnld = new(MyIntersectionOn2S.Function());
        MyIntersectionOn2S.Perform(Param, Rsnld);
        if (!MyIntersectionOn2S.IsDone())
        {
            MyHasBeenComputed = MyHasBeenComputedbis = false;
            return (false);
        }
        if (MyIntersectionOn2S.IsEmpty())
        {
            MyIsTangent = false;
            MyHasBeenComputed = MyHasBeenComputedbis = false;
            return (false);
        }
        MyHasBeenComputed = true;
        MyPnt = P = MyIntersectionOn2S.Point().Value();

        MyIntersectionOn2S.Point().Parameters(out u1, out v1, out u2, out v2);
        MyParOnS1.SetCoord(tu1, tv1);
        MyParOnS2.SetCoord(tu2, tv2);

        if (MyIntersectionOn2S.IsTangent())
        {
            MyIsTangent = false;
            MyHasBeenComputed = MyHasBeenComputedbis = false;
            return (false);
        }
        MyTg = Tg = new gp_Vec(MyIntersectionOn2S.Direction());
        MyTguv1 = Tguv1 = new gp_Vec2d(MyIntersectionOn2S.DirectionOnS1());
        MyTguv2 = Tguv2 = new gp_Vec2d(MyIntersectionOn2S.DirectionOnS2());

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

        Tg.Normalize();
        MyTg = Tg;

        double DeltaU, DeltaV;
        gp_Vec TU, TV;
        double TUTV, TgTU, TgTV, TUTU, TVTV, DIS;
        //------------------------------------------------------------
        //-- Tguv1
        //--
        ThePSurfaceTool.D1(MySurf1, u1, v1, out _, out TU, out TV);

        TUTU = TU.Dot(TU);
        TVTV = TV.Dot(TV);
        TUTV = TU.Dot(TV);
        TgTU = Tg.Dot(TU);
        TgTV = Tg.Dot(TV);
        DIS = TUTU * TVTV - TUTV * TUTV;
        if (Math.Abs(DIS) < Precision.Angular())
        {
            MyIsTangent = false;
            MyHasBeenComputed = MyHasBeenComputedbis = false;
            return (false);
        }

        DeltaU = (TgTU * TVTV - TgTV * TUTV) / DIS;
        DeltaV = (TgTV * TUTU - TgTU * TUTV) / DIS;

        Tguv1.SetCoord(DeltaU, DeltaV);
        MyTguv1 = Tguv1;

        //------------------------------------------------------------
        //-- Tguv2
        //--
        ThePSurfaceTool.D1(MySurf2, u2, v2, out _, out TU, out TV);

        TUTU = TU.Dot(TU);
        TVTV = TV.Dot(TV);
        TUTV = TU.Dot(TV);
        TgTU = Tg.Dot(TU);
        TgTV = Tg.Dot(TV);
        DIS = TUTU * TVTV - TUTV * TUTV;
        if (Math.Abs(DIS) < Precision.Angular())
        {
            MyIsTangent = false;
            MyHasBeenComputed = MyHasBeenComputedbis = false;
            return (false);
        }

        DeltaU = (TgTU * TVTV - TgTV * TUTV) / DIS;
        DeltaV = (TgTV * TUTU - TgTU * TUTV) / DIS;

        Tguv2.SetCoord(DeltaU, DeltaV);
        MyTguv2 = Tguv2;

        return (true);
    }

    public override void Pnt(double u1, double v1, double u2, double v2, ref gp_Pnt P)
    {
        gp_Pnt aP = new();
        gp_Vec aT = new();
        gp_Vec2d aTS1 = new(), aTS2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref aT, ref aTS1, ref aTS2);
        P = MyPnt;
    }

    /// <summary>Computes the point on the curve and the parameters on the surfaces.</summary>
    public override bool SeekPoint(double u1, double v1, double u2, double v2, ref IntSurf_PntOn2S Point)
    {
        gp_Pnt aP = new();
        gp_Vec aT = new();
        gp_Vec2d aTS1 = new(), aTS2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        if (!Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref aT, ref aTS1, ref aTS2))
        {
            return false;
        }

        Point.SetValue(aP, tu1, tv1, tu2, tv2);
        return true;
    }

    public override bool Tangency(double u1, double v1, double u2, double v2, ref gp_Vec T)
    {
        gp_Pnt aP = new();
        gp_Vec aT = new();
        gp_Vec2d aTS1 = new(), aTS2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref aT, ref aTS1, ref aTS2);
        T = MyTg;
        return (t);
    }

    public override bool TangencyOnSurf1(double u1, double v1, double u2, double v2, ref gp_Vec2d T)
    {
        gp_Pnt aP = new();
        gp_Vec aT = new();
        gp_Vec2d aTS1 = new(), aTS2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref aT, ref aTS1, ref aTS2);
        T = MyTguv1;
        return (t);
    }

    public override bool TangencyOnSurf2(double u1, double v1, double u2, double v2, ref gp_Vec2d T)
    {
        gp_Pnt aP = new();
        gp_Vec aT = new();
        gp_Vec2d aTS1 = new(), aTS2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref aT, ref aTS1, ref aTS2);
        T = MyTguv2;
        return (t);
    }
}
