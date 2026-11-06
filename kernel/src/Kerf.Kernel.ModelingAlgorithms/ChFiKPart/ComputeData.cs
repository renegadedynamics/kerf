// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData.cxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_RstMap.hxx (a typedef, written out where used)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Fillets.Analytic.ComputeDataChAsymPlnCon;
using static Kerf.Engine.Fillets.Analytic.ComputeDataChAsymPlnCyl;
using static Kerf.Engine.Fillets.Analytic.ComputeDataChAsymPlnPln;
using static Kerf.Engine.Fillets.Analytic.ComputeDataChPlnCon;
using static Kerf.Engine.Fillets.Analytic.ComputeDataChPlnCyl;
using static Kerf.Engine.Fillets.Analytic.ComputeDataChPlnPln;
using static Kerf.Engine.Fillets.Analytic.ComputeDataCS;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFcts;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFilPlnCon;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFilPlnCyl;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFilPlnPln;
using static Kerf.Engine.Fillets.Analytic.ComputeDataRotule;
using static Kerf.Engine.Fillets.Analytic.ComputeDataSphere;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Analytic;

/// <summary>
/// The fillets and chamfers computed analytically ("cas particuliers"): a <see cref="SurfData"/> filled with
/// its surface, its curves on the faces and their pcurves, written into the fillet's data structure.
/// </summary>
/// <remarks>
/// <list type="bullet">
/// <item>a cylinder between two planes;</item>
/// <item>a torus or sphere between a plane and an orthogonal cylinder, or an orthogonal cone;</item>
/// <item>the chamfers (symmetric, two distances, distance and angle, constant throat) in the same cases;</item>
/// <item>a torus between a plane and an orthogonal line ("rotule"), and the toric and spherical corners.</item>
/// </list>
/// The data structure is the builder's own (decision 1063): nothing here reads or writes the input's shapes.
/// </remarks>
internal static class ComputeData
{
    //=================================================================================================

    /// <summary>Computes a simple fillet in several particular cases.</summary>
    /// <exception cref="MethodNotImplementedException">The pair of surfaces is not one of the particular cases.</exception>
    internal static bool Compute(DsDataStructure dStr, SurfData data, SurfaceAdaptorBase s1,
                                 SurfaceAdaptorBase s2, Orientation or1, Orientation or2,
                                 Spine sp, int iedge)
    {
        double wref = 0.0;

        FilSpine? spine = sp as FilSpine;
        ChamfSpine? cSpine = sp as ChamfSpine;
        bool surfok = false;
        SurfaceKind typ1 = s1.GetType();
        SurfaceKind typ2 = s2.GetType();
        CurveKind ctyp;

        if (spine is not null)
        {
            ctyp = spine.CurrentElementarySpine(iedge).GetType();
        }
        else
        {
            ctyp = cSpine!.CurrentElementarySpine(iedge).GetType();
        }

        // Return orientations.
        Orientation orFace1 = Forward, orFace2 = Forward;
        FaceSurfaceAdaptor? hs = s1 as FaceSurfaceAdaptor;
        if (hs is not null)
        {
            orFace1 = hs.Face().Orientation;
        }
        hs = s2 as FaceSurfaceAdaptor;
        if (hs is not null)
        {
            orFace2 = hs.Face().Orientation;
        }

        if (spine is not null)
        {
            double radius = spine.Radius(iedge);
            if (typ1 == Plane && typ2 == Plane)
            {
                surfok = ChFiKPart_MakeFillet(dStr, data, s1.Plane(), s2.Plane(), or1, or2, radius, spine.Line(), wref,
                                              orFace1);
            }
            else if (typ1 == Plane && typ2 == SurfaceKind.Cylinder)
            {
                if (ctyp == Line)
                {
                    surfok = ChFiKPart_MakeFillet(dStr, data, s1.Plane(), s2.Cylinder(), s2.FirstUParameter(),
                                                  s2.LastUParameter(), or1, or2, radius, spine.Line(), wref, orFace1,
                                                  true);
                }
                else
                {
                    surfok = ChFiKPart_MakeFillet(dStr, data, s1.Plane(), s2.Cylinder(), s2.FirstUParameter(),
                                                  s2.LastUParameter(), or1, or2, radius, spine.Circle(), wref, orFace1,
                                                  true);
                }
            }
            else if (typ1 == SurfaceKind.Cylinder && typ2 == Plane)
            {
                if (ctyp == Line)
                {
                    surfok = ChFiKPart_MakeFillet(dStr, data, s2.Plane(), s1.Cylinder(), s1.FirstUParameter(),
                                                  s1.LastUParameter(), or2, or1, radius, spine.Line(), wref, orFace2,
                                                  false);
                }
                else
                {
                    surfok = ChFiKPart_MakeFillet(dStr, data, s2.Plane(), s1.Cylinder(), s1.FirstUParameter(),
                                                  s1.LastUParameter(), or2, or1, radius, spine.Circle(), wref, orFace2,
                                                  false);
                }
            }
            else if (typ1 == Plane && typ2 == SurfaceKind.Cone)
            {
                surfok = ChFiKPart_MakeFillet(dStr, data, s1.Plane(), s2.Cone(), s2.FirstUParameter(),
                                              s2.LastUParameter(), or1, or2, radius, spine.Circle(), wref, orFace1,
                                              true);
            }
            else if (typ1 == SurfaceKind.Cone && typ2 == Plane)
            {
                surfok = ChFiKPart_MakeFillet(dStr, data, s2.Plane(), s1.Cone(), s1.FirstUParameter(),
                                              s1.LastUParameter(), or2, or1, radius, spine.Circle(), wref, orFace2,
                                              false);
            }
            else
            {
                throw new MethodNotImplementedException("particular case not written");
            }
        }
        else if (cSpine is not null)
        {

            ChamfMode mode = cSpine.Mode();

            if (cSpine.IsChamfer == ChamfMethod.Sym)
            {
                cSpine.GetDist(out double dis);

                if (typ1 == Plane && typ2 == Plane)
                {
                    surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s1.Plane(), s2.Plane(), or1, or2, dis, dis,
                                                   cSpine.Line(), wref, orFace1);
                }
                else if (typ1 == Plane && typ2 == SurfaceKind.Cylinder)
                {
                    if (ctyp == Circle)
                    {
                        surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s1.Plane(), s2.Cylinder(),
                                                       s2.FirstUParameter(), s2.LastUParameter(), or1, or2, dis, dis,
                                                       cSpine.Circle(), wref, orFace1, true);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s1.Plane(), s2.Cylinder(),
                                                       s2.FirstUParameter(), s2.LastUParameter(), or1, or2, dis, dis,
                                                       cSpine.Line(), wref, orFace1, true);
                    }
                }
                else if (typ1 == SurfaceKind.Cylinder && typ2 == Plane)
                {
                    if (ctyp == Circle)
                    {
                        surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s2.Plane(), s1.Cylinder(),
                                                       s1.FirstUParameter(), s1.LastUParameter(), or2, or1, dis, dis,
                                                       cSpine.Circle(), wref, orFace2, false);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s2.Plane(), s1.Cylinder(),
                                                       s1.FirstUParameter(), s1.LastUParameter(), or2, or1, dis, dis,
                                                       cSpine.Line(), wref, orFace2, false);
                    }
                }
                else if (typ1 == Plane && typ2 == SurfaceKind.Cone)
                {
                    surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s1.Plane(), s2.Cone(), s2.FirstUParameter(),
                                                   s2.LastUParameter(), or1, or2, dis, dis, cSpine.Circle(), wref,
                                                   orFace1, true);
                }
                else if (typ1 == SurfaceKind.Cone && typ2 == Plane)
                {
                    surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s2.Plane(), s1.Cone(), s1.FirstUParameter(),
                                                   s1.LastUParameter(), or2, or1, dis, dis, cSpine.Circle(), wref,
                                                   orFace2, false);
                }
                else
                {
                    throw new MethodNotImplementedException("particular case not written");
                }
            }
            else if (cSpine.IsChamfer == ChamfMethod.TwoDist)
            {
                cSpine.Dists(out double dis1, out double dis2);
                if (typ1 == Plane && typ2 == Plane)
                {
                    surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s1.Plane(), s2.Plane(), or1, or2, dis1, dis2,
                                                   cSpine.Line(), wref, orFace1);
                }
                else if (typ1 == Plane && typ2 == SurfaceKind.Cylinder)
                {
                    if (ctyp == Circle)
                    {
                        surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s1.Plane(), s2.Cylinder(),
                                                       s2.FirstUParameter(), s2.LastUParameter(), or1, or2, dis1, dis2,
                                                       cSpine.Circle(), wref, orFace1, true);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s1.Plane(), s2.Cylinder(),
                                                       s2.FirstUParameter(), s2.LastUParameter(), or1, or2, dis1, dis2,
                                                       cSpine.Line(), wref, orFace1, true);
                    }
                }
                else if (typ1 == SurfaceKind.Cylinder && typ2 == Plane)
                {
                    if (ctyp == Circle)
                    {
                        surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s2.Plane(), s1.Cylinder(),
                                                       s1.FirstUParameter(), s1.LastUParameter(), or2, or1, dis2, dis1,
                                                       cSpine.Circle(), wref, orFace2, false);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s2.Plane(), s1.Cylinder(),
                                                       s1.FirstUParameter(), s1.LastUParameter(), or2, or1, dis2, dis1,
                                                       cSpine.Line(), wref, orFace2, false);
                    }
                }
                else if (typ1 == Plane && typ2 == SurfaceKind.Cone)
                {
                    surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s1.Plane(), s2.Cone(), s2.FirstUParameter(),
                                                   s2.LastUParameter(), or1, or2, dis1, dis2, cSpine.Circle(), wref,
                                                   orFace1, true);
                }
                else if (typ1 == SurfaceKind.Cone && typ2 == Plane)
                {
                    surfok = ChFiKPart_MakeChamfer(dStr, data, mode, s2.Plane(), s1.Cone(), s1.FirstUParameter(),
                                                   s1.LastUParameter(), or2, or1, dis1, dis2, cSpine.Circle(), wref,
                                                   orFace2, false);
                }
                else
                {
                    throw new MethodNotImplementedException("particular case not written");
                }
            }
            else
            {
                bool disOnP = true;
                cSpine.GetDistAngle(out double dis, out double angle);
                if (typ1 == Plane && typ2 == Plane)
                {
                    surfok = ChFiKPart_MakeChAsym(dStr, data, s1.Plane(), s2.Plane(), or1, or2, dis, angle,
                                                  cSpine.Line(), wref, orFace1, disOnP);
                }
                else if (typ1 == Plane && typ2 == SurfaceKind.Cylinder)
                {
                    if (ctyp == Circle)
                    {
                        surfok = ChFiKPart_MakeChAsym(dStr, data, s1.Plane(), s2.Cylinder(), s2.FirstUParameter(),
                                                      s2.LastUParameter(), or1, or2, dis, angle, cSpine.Circle(), wref,
                                                      orFace1, true, disOnP);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChAsym(dStr, data, s1.Plane(), s2.Cylinder(), s2.FirstUParameter(),
                                                      s2.LastUParameter(), or1, or2, dis, angle, cSpine.Line(), wref,
                                                      orFace1, true, disOnP);
                    }
                }
                else if (typ1 == SurfaceKind.Cylinder && typ2 == Plane)
                {
                    if (ctyp == Circle)
                    {
                        surfok = ChFiKPart_MakeChAsym(dStr, data, s2.Plane(), s1.Cylinder(), s1.FirstUParameter(),
                                                      s1.LastUParameter(), or2, or1, dis, angle, cSpine.Circle(), wref,
                                                      orFace2, false, disOnP);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChAsym(dStr, data, s2.Plane(), s1.Cylinder(), s1.FirstUParameter(),
                                                      s1.LastUParameter(), or2, or1, dis, angle, cSpine.Line(), wref,
                                                      orFace2, false, disOnP);
                    }
                }
                else if (typ1 == Plane && typ2 == SurfaceKind.Cone)
                {
                    surfok = ChFiKPart_MakeChAsym(dStr, data, s1.Plane(), s2.Cone(), s2.FirstUParameter(),
                                                  s2.LastUParameter(), or1, or2, dis, angle, cSpine.Circle(), wref,
                                                  orFace1, true, disOnP);
                }
                else if (typ1 == SurfaceKind.Cone && typ2 == Plane)
                {
                    surfok = ChFiKPart_MakeChAsym(dStr, data, s2.Plane(), s1.Cone(), s1.FirstUParameter(),
                                                  s1.LastUParameter(), or2, or1, dis, angle, cSpine.Circle(), wref,
                                                  orFace2, false, disOnP);
                }
                else
                {
                    throw new MethodNotImplementedException("particular case not written");
                }
            }
        }
        return surfok;
    }

    //=================================================================================================

    /// <summary>Computes a toric or spheric corner fillet.</summary>
    /// <exception cref="ConstructionErrorException"><paramref name="s1"/> is not a plane.</exception>
    internal static bool ComputeCorner(DsDataStructure dStr, SurfData data, SurfaceAdaptorBase s1,
                                       SurfaceAdaptorBase s2, Orientation orFace1, Orientation orFace2,
                                       Orientation or1, Orientation or2, double minRad, double majRad,
                                       in Point2D p1S1, in Point2D p2S1, in Point2D p1S2, in Point2D p2S2)
    {
        bool surfok;
        SurfaceKind typ1 = s1.GetType();
        SurfaceKind typ2 = s2.GetType();
        if (typ1 != Plane)
        {
            throw new ConstructionErrorException("la face du conge torique doit etre plane");
        }
        // The guideline is the circle corresponding
        // to the section of S2, and other construction elements.

        Cylinder3D cyl = new();
        Circle3D circ = new();
        double first, last, fu, lu;
        ChFiKPart_CornerSpine(s1, s2, p1S1, p2S1, p1S2, p2S2, majRad, ref cyl, ref circ, out first, out last);
        if (typ2 == SurfaceKind.Cylinder)
        {
            cyl = s2.Cylinder();
            fu = p1S2.X;
            lu = p2S2.X;
        }
        else
        {
            fu = first;
            lu = last;
        }
        surfok = ChFiKPart_MakeFillet(dStr, data, s1.Plane(), cyl, fu, lu, or1, or2, minRad, circ, first, orFace1,
                                      true);
        if (surfok)
        {
            if (typ2 != SurfaceKind.Cylinder)
            {
                data.ChangeInterferenceOnS2().ChangePCurveOnFace() = ChFiKPart_PCurve(p1S2, p2S2, first, last);
            }
            data.ChangeVertexFirstOnS1().SetPoint(s1.Value(p1S1.X, p1S1.Y));
            data.ChangeVertexLastOnS1().SetPoint(s1.Value(p2S1.X, p2S1.Y));
            data.ChangeVertexFirstOnS2().SetPoint(s2.Value(p1S2.X, p1S2.Y));
            data.ChangeVertexLastOnS2().SetPoint(s2.Value(p2S2.X, p2S2.Y));
            data.ChangeInterferenceOnS1().SetFirstParameter(first);
            data.ChangeInterferenceOnS1().SetLastParameter(last);
            data.ChangeInterferenceOnS2().SetFirstParameter(first);
            data.ChangeInterferenceOnS2().SetLastParameter(last);
            return true;
        }
        return false;
    }

    //=================================================================================================

    /// <summary>Computes spheric corner fillet with non iso pcurve on S2.</summary>
    internal static bool ComputeCorner(DsDataStructure dStr, SurfData data, SurfaceAdaptorBase s1,
                                       SurfaceAdaptorBase s2, Orientation orFace1, Orientation orFace2,
                                       Orientation or1, Orientation or2, double rad, in Point2D ps1,
                                       in Point2D p1S2, in Point2D p2S2)
    {
        return ChFiKPart_Sphere(dStr, data, s1, s2, orFace1, orFace2, or1, or2, rad, ps1, p1S2, p2S2);
    }

    //=================================================================================================

    /// <summary>Computes a toric corner rotule.</summary>
    /// <exception cref="ConstructionErrorException">One of the surfaces is not a plane.</exception>
    internal static bool ComputeCorner(DsDataStructure dStr, SurfData data, SurfaceAdaptorBase s,
                                       SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, Orientation ofS,
                                       Orientation os, Orientation os1, Orientation os2,
                                       double radius)
    {
        SurfaceKind typ = s.GetType();
        SurfaceKind typ1 = s1.GetType();
        SurfaceKind typ2 = s2.GetType();
        if (typ != Plane || typ1 != Plane || typ2 != Plane)
        {
            throw new ConstructionErrorException("torus joint only between the planes");
        }
        return ChFiKPart_MakeRotule(dStr, data, s.Plane(), s1.Plane(), s2.Plane(), os, os1, os2, radius, ofS);
    }
}
