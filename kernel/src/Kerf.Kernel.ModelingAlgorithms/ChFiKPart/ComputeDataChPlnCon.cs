// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnCon.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnCon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Data;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Fillets.Data.ChamfMode;
using static Kerf.Engine.Fillets.Analytic.ComputeDataChAsymPlnCon;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Analytic;

/// <summary>The chamfer between a plane and a cone along a circle, made as the distance-angle chamfer.</summary>
internal static class ComputeDataChPlnCon
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

    /// <summary>The chamfer at the distances <paramref name="dis1"/> on the plane and <paramref name="dis2"/>
    /// on the cone, computed by <see cref="ComputeDataChAsymPlnCon"/>.</summary>
    /// <remarks>The older direct construction OCCT keeps commented out after the function is not ported.</remarks>
    internal static bool ChFiKPart_MakeChamfer(DsDataStructure dStr, SurfData data,
                                               ChamfMode mode, in Plane3D pln, in Cone3D con,
                                               double fu, double lu, Orientation or1,
                                               Orientation or2, double dis1, double dis2,
                                               in Circle3D spine, double first, Orientation ofpl,
                                               bool plandab)
    {
        double angcon = con.SemiAngle;
        double Dis1 = dis1, Dis2 = dis2;
        double alpha = Math.PI / 2 - angcon;
        double cosHalfAlpha = Math.Cos(alpha / 2);
        if (mode == ChamfMode.ConstThroatChamfer)
        {
            Dis1 = Dis2 = dis1 / cosHalfAlpha;
        }
        else if (mode == ChamfMode.ConstThroatWithPenetrationChamfer)
        {
            double aDis1 = Math.Min(dis1, dis2);
            double aDis2 = Math.Max(dis1, dis2);
            double dis1dis1 = aDis1 * aDis1, dis2dis2 = aDis2 * aDis2;
            double sinAlpha = Math.Sin(alpha);
            double cosAlpha = Math.Cos(alpha);
            double cotanAlpha = cosAlpha / sinAlpha;
            Dis1 = Math.Sqrt(dis2dis2 - dis1dis1) - aDis1 * cotanAlpha;
            double cosBeta = Math.Sqrt(1 - dis1dis1 / dis2dis2) * cosAlpha + aDis1 / aDis2 * sinAlpha;
            double fullDist1 = aDis2 / cosBeta;
            Dis2 = fullDist1 - aDis1 / sinAlpha;
        }
        double sincon = Math.Abs(Math.Sin(angcon));
        double angle;
        bool isResol;
        CoordinateSystem posPl = pln.Position;
        Direction3D dpl = posPl.XDirection.Crossed(posPl.YDirection);
        if (or1 == Reversed)
        {
            dpl = dpl.Reversed();
        }
        // compute the origin of the conical chamfer PtPl
        Point3D or = con.Location;
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, or, out u, out v);
        // (OCCT_DEBUG pt2dPln left out)
        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(u, v, posPl, out or);
        global::Kerf.Engine.Geometry.ElCLib.D1(first, spine, out Point3D ptSp, out Vector3D dSp);
        // (OCCT_DEBUG Dx left out)
        // compute the normal to the cone in PtSp
        global::Kerf.Engine.Geometry.ElSLib.Parameters(con, ptSp, out u, out v);
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, con, out Point3D ptCon, out Vector3D deru, out Vector3D derv);
        Direction3D dcon = new(deru.Crossed(derv));
        if (or2 == Reversed)
        {
            dcon = dcon.Reversed();
        }
        bool ouvert = (dpl.Dot(dcon) >= 0.0);
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
        bool disOnP = false;
        isResol = ChFiKPart_MakeChAsym(dStr, data, pln, con, fu, lu, or1, or2, Dis2, angle, spine, first, ofpl,
                                       plandab, disOnP);
        return isResol;
    }
}
