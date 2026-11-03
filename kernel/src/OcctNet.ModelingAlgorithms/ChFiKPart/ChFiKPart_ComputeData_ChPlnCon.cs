// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnCon.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnCon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFiDS_ChamfMode;
using static Occt.ChFiKPart_ComputeData_ChAsymPlnCon;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>The chamfer between a plane and a cone along a circle, made as the distance-angle chamfer.</summary>
internal static class ChFiKPart_ComputeData_ChPlnCon
{
    //=======================================================================
    // function : MakeChamfer
    // purpose  : Compute the chamfer in the particular case Plane/Cone or
    //           Cylinder/Plane
    //           Compute the SurfData <Data> of the chamfer build on the <Spine>
    //           between the plane <Pln> and the cone <Con>, with the
    //           distances <Dis1> on <Pln> and <Dis2> on <Con>.
    //           <Or1> and <Or2> are the orientations of <Pln> and <Con>
    //           and <Ofpl> this of the face carried by <Pln>.
    //           <First> is the start point on the <Spine>
    //           <Plandab> is equal to True if the plane is the surface S1
    //           <fu> and <lu> are the first and last u parameters of the
    //           cone
    // out      : True if the chanfer has been computed
    //           False else
    //=======================================================================

    /// <summary>The chamfer at the distances <paramref name="theDis1"/> on the plane and <paramref name="theDis2"/>
    /// on the cone, computed by <see cref="ChFiKPart_ComputeData_ChAsymPlnCon"/>.</summary>
    /// <remarks>The older direct construction OCCT keeps commented out after the function is not ported.</remarks>
    internal static bool ChFiKPart_MakeChamfer(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data,
                                               ChFiDS_ChamfMode theMode, in Plane3D Pln, in Cone Con,
                                               double fu, double lu, Orientation Or1,
                                               Orientation Or2, double theDis1, double theDis2,
                                               in Circle3D Spine, double First, Orientation Ofpl,
                                               bool plandab)
    {
        double angcon = Con.SemiAngle();
        double Dis1 = theDis1, Dis2 = theDis2;
        double Alpha = Math.PI / 2 - angcon;
        double CosHalfAlpha = Math.Cos(Alpha / 2);
        if (theMode == ChFiDS_ConstThroatChamfer)
        {
            Dis1 = Dis2 = theDis1 / CosHalfAlpha;
        }
        else if (theMode == ChFiDS_ConstThroatWithPenetrationChamfer)
        {
            double aDis1 = Math.Min(theDis1, theDis2);
            double aDis2 = Math.Max(theDis1, theDis2);
            double dis1dis1 = aDis1 * aDis1, dis2dis2 = aDis2 * aDis2;
            double SinAlpha = Math.Sin(Alpha);
            double CosAlpha = Math.Cos(Alpha);
            double CotanAlpha = CosAlpha / SinAlpha;
            Dis1 = Math.Sqrt(dis2dis2 - dis1dis1) - aDis1 * CotanAlpha;
            double CosBeta = Math.Sqrt(1 - dis1dis1 / dis2dis2) * CosAlpha + aDis1 / aDis2 * SinAlpha;
            double FullDist1 = aDis2 / CosBeta;
            Dis2 = FullDist1 - aDis1 / SinAlpha;
        }
        double sincon = Math.Abs(Math.Sin(angcon));
        double angle;
        bool IsResol;
        CoordinateSystem PosPl = Pln.Position();
        Direction3D Dpl = PosPl.XDirection().Crossed(PosPl.YDirection());
        if (Or1 == Reversed)
        {
            Dpl.Reverse();
        }
        // compute the origin of the conical chamfer PtPl
        Point3D Or = Con.Location();
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(PosPl, Or, out u, out v);
        // (OCCT_DEBUG pt2dPln left out)
        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(u, v, PosPl, out Or);
        global::Kerf.Engine.Geometry.ElCLib.D1(First, Spine, out Point3D PtSp, out Vector3D DSp);
        // (OCCT_DEBUG Dx left out)
        // compute the normal to the cone in PtSp
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Con, PtSp, out u, out v);
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, Con, out Point3D PtCon, out Vector3D deru, out Vector3D derv);
        Direction3D Dcon = new(deru.Crossed(derv));
        if (Or2 == Reversed)
        {
            Dcon.Reverse();
        }
        bool ouvert = (Dpl.Dot(Dcon) >= 0.0);
        if (!ouvert)
        {
            if (Math.Abs(Dis1 - Dis2 * sincon) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                double abscos = Math.Abs(Dis2 - Dis1 * sincon);
                angle = Math.Atan((Dis1 * Math.Cos(angcon)) / abscos);
            }
            else
            {
                angle = angcon;
            }
        }
        else
        {
            angle = Math.Atan((Dis1 * Math.Cos(angcon)) / (Dis2 + Dis1 * sincon));
        }
        bool DisOnP = false;
        IsResol = ChFiKPart_MakeChAsym(DStr, Data, Pln, Con, fu, lu, Or1, Or2, Dis2, angle, Spine, First, Ofpl,
                                       plandab, DisOnP);
        return IsResol;
    }
}
