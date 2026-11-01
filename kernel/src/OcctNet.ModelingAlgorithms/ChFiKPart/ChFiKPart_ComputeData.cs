// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData.cxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_RstMap.hxx (a typedef, written out where used)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFiDS_ChamfMethod;
using static Occt.ChFiKPart_ComputeData_ChAsymPlnCon;
using static Occt.ChFiKPart_ComputeData_ChAsymPlnCyl;
using static Occt.ChFiKPart_ComputeData_ChAsymPlnPln;
using static Occt.ChFiKPart_ComputeData_ChPlnCon;
using static Occt.ChFiKPart_ComputeData_ChPlnCyl;
using static Occt.ChFiKPart_ComputeData_ChPlnPln;
using static Occt.ChFiKPart_ComputeData_CS;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.ChFiKPart_ComputeData_FilPlnCon;
using static Occt.ChFiKPart_ComputeData_FilPlnCyl;
using static Occt.ChFiKPart_ComputeData_FilPlnPln;
using static Occt.ChFiKPart_ComputeData_Rotule;
using static Occt.ChFiKPart_ComputeData_Sphere;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The fillets and chamfers computed analytically ("cas particuliers"): a <see cref="ChFiDS_SurfData"/> filled with
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
internal static class ChFiKPart_ComputeData
{
    //=================================================================================================

    /// <summary>Computes a simple fillet in several particular cases.</summary>
    /// <exception cref="Standard_NotImplemented">The pair of surfaces is not one of the particular cases.</exception>
    internal static bool Compute(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, Adaptor3d_Surface S1,
                                 Adaptor3d_Surface S2, TopAbs_Orientation Or1, TopAbs_Orientation Or2,
                                 ChFiDS_Spine Sp, int Iedge)
    {
        double Wref = 0.0;

        ChFiDS_FilSpine? Spine = Sp as ChFiDS_FilSpine;
        ChFiDS_ChamfSpine? CSpine = Sp as ChFiDS_ChamfSpine;
        bool surfok = false;
        GeomAbs_SurfaceType typ1 = S1.GetType();
        GeomAbs_SurfaceType typ2 = S2.GetType();
        GeomAbs_CurveType ctyp;

        if (Spine is not null)
        {
            ctyp = Spine.CurrentElementarySpine(Iedge).GetType();
        }
        else
        {
            ctyp = CSpine!.CurrentElementarySpine(Iedge).GetType();
        }

        // Return orientations.
        TopAbs_Orientation OrFace1 = TopAbs_FORWARD, OrFace2 = TopAbs_FORWARD;
        BRepAdaptor_Surface? HS = S1 as BRepAdaptor_Surface;
        if (HS is not null)
        {
            OrFace1 = HS.Face().Orientation();
        }
        HS = S2 as BRepAdaptor_Surface;
        if (HS is not null)
        {
            OrFace2 = HS.Face().Orientation();
        }

        if (Spine is not null)
        {
            double Radius = Spine.Radius(Iedge);
            if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Plane)
            {
                surfok = ChFiKPart_MakeFillet(DStr, Data, S1.Plane(), S2.Plane(), Or1, Or2, Radius, Spine.Line(), Wref,
                                              OrFace1);
            }
            else if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Cylinder)
            {
                if (ctyp == GeomAbs_Line)
                {
                    surfok = ChFiKPart_MakeFillet(DStr, Data, S1.Plane(), S2.Cylinder(), S2.FirstUParameter(),
                                                  S2.LastUParameter(), Or1, Or2, Radius, Spine.Line(), Wref, OrFace1,
                                                  true);
                }
                else
                {
                    surfok = ChFiKPart_MakeFillet(DStr, Data, S1.Plane(), S2.Cylinder(), S2.FirstUParameter(),
                                                  S2.LastUParameter(), Or1, Or2, Radius, Spine.Circle(), Wref, OrFace1,
                                                  true);
                }
            }
            else if (typ1 == GeomAbs_Cylinder && typ2 == GeomAbs_Plane)
            {
                if (ctyp == GeomAbs_Line)
                {
                    surfok = ChFiKPart_MakeFillet(DStr, Data, S2.Plane(), S1.Cylinder(), S1.FirstUParameter(),
                                                  S1.LastUParameter(), Or2, Or1, Radius, Spine.Line(), Wref, OrFace2,
                                                  false);
                }
                else
                {
                    surfok = ChFiKPart_MakeFillet(DStr, Data, S2.Plane(), S1.Cylinder(), S1.FirstUParameter(),
                                                  S1.LastUParameter(), Or2, Or1, Radius, Spine.Circle(), Wref, OrFace2,
                                                  false);
                }
            }
            else if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Cone)
            {
                surfok = ChFiKPart_MakeFillet(DStr, Data, S1.Plane(), S2.Cone(), S2.FirstUParameter(),
                                              S2.LastUParameter(), Or1, Or2, Radius, Spine.Circle(), Wref, OrFace1,
                                              true);
            }
            else if (typ1 == GeomAbs_Cone && typ2 == GeomAbs_Plane)
            {
                surfok = ChFiKPart_MakeFillet(DStr, Data, S2.Plane(), S1.Cone(), S1.FirstUParameter(),
                                              S1.LastUParameter(), Or2, Or1, Radius, Spine.Circle(), Wref, OrFace2,
                                              false);
            }
            else
            {
                throw new Standard_NotImplemented("particular case not written");
            }
        }
        else if (CSpine is not null)
        {

            ChFiDS_ChamfMode aMode = CSpine.Mode();

            if (CSpine.IsChamfer() == ChFiDS_Sym)
            {
                CSpine.GetDist(out double dis);

                if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Plane)
                {
                    surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S1.Plane(), S2.Plane(), Or1, Or2, dis, dis,
                                                   CSpine.Line(), Wref, OrFace1);
                }
                else if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Cylinder)
                {
                    if (ctyp == GeomAbs_Circle)
                    {
                        surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S1.Plane(), S2.Cylinder(),
                                                       S2.FirstUParameter(), S2.LastUParameter(), Or1, Or2, dis, dis,
                                                       CSpine.Circle(), Wref, OrFace1, true);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S1.Plane(), S2.Cylinder(),
                                                       S2.FirstUParameter(), S2.LastUParameter(), Or1, Or2, dis, dis,
                                                       CSpine.Line(), Wref, OrFace1, true);
                    }
                }
                else if (typ1 == GeomAbs_Cylinder && typ2 == GeomAbs_Plane)
                {
                    if (ctyp == GeomAbs_Circle)
                    {
                        surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S2.Plane(), S1.Cylinder(),
                                                       S1.FirstUParameter(), S1.LastUParameter(), Or2, Or1, dis, dis,
                                                       CSpine.Circle(), Wref, OrFace2, false);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S2.Plane(), S1.Cylinder(),
                                                       S1.FirstUParameter(), S1.LastUParameter(), Or2, Or1, dis, dis,
                                                       CSpine.Line(), Wref, OrFace2, false);
                    }
                }
                else if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Cone)
                {
                    surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S1.Plane(), S2.Cone(), S2.FirstUParameter(),
                                                   S2.LastUParameter(), Or1, Or2, dis, dis, CSpine.Circle(), Wref,
                                                   OrFace1, true);
                }
                else if (typ1 == GeomAbs_Cone && typ2 == GeomAbs_Plane)
                {
                    surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S2.Plane(), S1.Cone(), S1.FirstUParameter(),
                                                   S1.LastUParameter(), Or2, Or1, dis, dis, CSpine.Circle(), Wref,
                                                   OrFace2, false);
                }
                else
                {
                    throw new Standard_NotImplemented("particular case not written");
                }
            }
            else if (CSpine.IsChamfer() == ChFiDS_TwoDist)
            {
                CSpine.Dists(out double dis1, out double dis2);
                if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Plane)
                {
                    surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S1.Plane(), S2.Plane(), Or1, Or2, dis1, dis2,
                                                   CSpine.Line(), Wref, OrFace1);
                }
                else if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Cylinder)
                {
                    if (ctyp == GeomAbs_Circle)
                    {
                        surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S1.Plane(), S2.Cylinder(),
                                                       S2.FirstUParameter(), S2.LastUParameter(), Or1, Or2, dis1, dis2,
                                                       CSpine.Circle(), Wref, OrFace1, true);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S1.Plane(), S2.Cylinder(),
                                                       S2.FirstUParameter(), S2.LastUParameter(), Or1, Or2, dis1, dis2,
                                                       CSpine.Line(), Wref, OrFace1, true);
                    }
                }
                else if (typ1 == GeomAbs_Cylinder && typ2 == GeomAbs_Plane)
                {
                    if (ctyp == GeomAbs_Circle)
                    {
                        surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S2.Plane(), S1.Cylinder(),
                                                       S1.FirstUParameter(), S1.LastUParameter(), Or2, Or1, dis2, dis1,
                                                       CSpine.Circle(), Wref, OrFace2, false);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S2.Plane(), S1.Cylinder(),
                                                       S1.FirstUParameter(), S1.LastUParameter(), Or2, Or1, dis2, dis1,
                                                       CSpine.Line(), Wref, OrFace2, false);
                    }
                }
                else if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Cone)
                {
                    surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S1.Plane(), S2.Cone(), S2.FirstUParameter(),
                                                   S2.LastUParameter(), Or1, Or2, dis1, dis2, CSpine.Circle(), Wref,
                                                   OrFace1, true);
                }
                else if (typ1 == GeomAbs_Cone && typ2 == GeomAbs_Plane)
                {
                    surfok = ChFiKPart_MakeChamfer(DStr, Data, aMode, S2.Plane(), S1.Cone(), S1.FirstUParameter(),
                                                   S1.LastUParameter(), Or2, Or1, dis1, dis2, CSpine.Circle(), Wref,
                                                   OrFace2, false);
                }
                else
                {
                    throw new Standard_NotImplemented("particular case not written");
                }
            }
            else
            {
                bool DisOnP = true;
                CSpine.GetDistAngle(out double dis, out double Angle);
                if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Plane)
                {
                    surfok = ChFiKPart_MakeChAsym(DStr, Data, S1.Plane(), S2.Plane(), Or1, Or2, dis, Angle,
                                                  CSpine.Line(), Wref, OrFace1, DisOnP);
                }
                else if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Cylinder)
                {
                    if (ctyp == GeomAbs_Circle)
                    {
                        surfok = ChFiKPart_MakeChAsym(DStr, Data, S1.Plane(), S2.Cylinder(), S2.FirstUParameter(),
                                                      S2.LastUParameter(), Or1, Or2, dis, Angle, CSpine.Circle(), Wref,
                                                      OrFace1, true, DisOnP);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChAsym(DStr, Data, S1.Plane(), S2.Cylinder(), S2.FirstUParameter(),
                                                      S2.LastUParameter(), Or1, Or2, dis, Angle, CSpine.Line(), Wref,
                                                      OrFace1, true, DisOnP);
                    }
                }
                else if (typ1 == GeomAbs_Cylinder && typ2 == GeomAbs_Plane)
                {
                    if (ctyp == GeomAbs_Circle)
                    {
                        surfok = ChFiKPart_MakeChAsym(DStr, Data, S2.Plane(), S1.Cylinder(), S1.FirstUParameter(),
                                                      S1.LastUParameter(), Or2, Or1, dis, Angle, CSpine.Circle(), Wref,
                                                      OrFace2, false, DisOnP);
                    }
                    else
                    {
                        surfok = ChFiKPart_MakeChAsym(DStr, Data, S2.Plane(), S1.Cylinder(), S1.FirstUParameter(),
                                                      S1.LastUParameter(), Or2, Or1, dis, Angle, CSpine.Line(), Wref,
                                                      OrFace2, false, DisOnP);
                    }
                }
                else if (typ1 == GeomAbs_Plane && typ2 == GeomAbs_Cone)
                {
                    surfok = ChFiKPart_MakeChAsym(DStr, Data, S1.Plane(), S2.Cone(), S2.FirstUParameter(),
                                                  S2.LastUParameter(), Or1, Or2, dis, Angle, CSpine.Circle(), Wref,
                                                  OrFace1, true, DisOnP);
                }
                else if (typ1 == GeomAbs_Cone && typ2 == GeomAbs_Plane)
                {
                    surfok = ChFiKPart_MakeChAsym(DStr, Data, S2.Plane(), S1.Cone(), S1.FirstUParameter(),
                                                  S1.LastUParameter(), Or2, Or1, dis, Angle, CSpine.Circle(), Wref,
                                                  OrFace2, false, DisOnP);
                }
                else
                {
                    throw new Standard_NotImplemented("particular case not written");
                }
            }
        }
        return surfok;
    }

    //=================================================================================================

    /// <summary>Computes a toric or spheric corner fillet.</summary>
    /// <exception cref="Standard_ConstructionError"><paramref name="S1"/> is not a plane.</exception>
    internal static bool ComputeCorner(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, Adaptor3d_Surface S1,
                                       Adaptor3d_Surface S2, TopAbs_Orientation OrFace1, TopAbs_Orientation OrFace2,
                                       TopAbs_Orientation Or1, TopAbs_Orientation Or2, double minRad, double majRad,
                                       in gp_Pnt2d P1S1, in gp_Pnt2d P2S1, in gp_Pnt2d P1S2, in gp_Pnt2d P2S2)
    {
        bool surfok;
        GeomAbs_SurfaceType typ1 = S1.GetType();
        GeomAbs_SurfaceType typ2 = S2.GetType();
        if (typ1 != GeomAbs_Plane)
        {
            throw new Standard_ConstructionError("la face du conge torique doit etre plane");
        }
        // The guideline is the circle corresponding
        // to the section of S2, and other construction elements.

        gp_Cylinder cyl = new();
        gp_Circ circ = new();
        double First, Last, fu, lu;
        ChFiKPart_CornerSpine(S1, S2, P1S1, P2S1, P1S2, P2S2, majRad, ref cyl, ref circ, out First, out Last);
        if (typ2 == GeomAbs_Cylinder)
        {
            cyl = S2.Cylinder();
            fu = P1S2.X();
            lu = P2S2.X();
        }
        else
        {
            fu = First;
            lu = Last;
        }
        surfok = ChFiKPart_MakeFillet(DStr, Data, S1.Plane(), cyl, fu, lu, Or1, Or2, minRad, circ, First, OrFace1,
                                      true);
        if (surfok)
        {
            if (typ2 != GeomAbs_Cylinder)
            {
                Data.ChangeInterferenceOnS2().ChangePCurveOnFace() = ChFiKPart_PCurve(P1S2, P2S2, First, Last);
            }
            Data.ChangeVertexFirstOnS1().SetPoint(S1.Value(P1S1.X(), P1S1.Y()));
            Data.ChangeVertexLastOnS1().SetPoint(S1.Value(P2S1.X(), P2S1.Y()));
            Data.ChangeVertexFirstOnS2().SetPoint(S2.Value(P1S2.X(), P1S2.Y()));
            Data.ChangeVertexLastOnS2().SetPoint(S2.Value(P2S2.X(), P2S2.Y()));
            Data.ChangeInterferenceOnS1().SetFirstParameter(First);
            Data.ChangeInterferenceOnS1().SetLastParameter(Last);
            Data.ChangeInterferenceOnS2().SetFirstParameter(First);
            Data.ChangeInterferenceOnS2().SetLastParameter(Last);
            return true;
        }
        return false;
    }

    //=================================================================================================

    /// <summary>Computes spheric corner fillet with non iso pcurve on S2.</summary>
    internal static bool ComputeCorner(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, Adaptor3d_Surface S1,
                                       Adaptor3d_Surface S2, TopAbs_Orientation OrFace1, TopAbs_Orientation OrFace2,
                                       TopAbs_Orientation Or1, TopAbs_Orientation Or2, double Rad, in gp_Pnt2d PS1,
                                       in gp_Pnt2d P1S2, in gp_Pnt2d P2S2)
    {
        return ChFiKPart_Sphere(DStr, Data, S1, S2, OrFace1, OrFace2, Or1, Or2, Rad, PS1, P1S2, P2S2);
    }

    //=================================================================================================

    /// <summary>Computes a toric corner rotule.</summary>
    /// <exception cref="Standard_ConstructionError">One of the surfaces is not a plane.</exception>
    internal static bool ComputeCorner(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, Adaptor3d_Surface S,
                                       Adaptor3d_Surface S1, Adaptor3d_Surface S2, TopAbs_Orientation OfS,
                                       TopAbs_Orientation OS, TopAbs_Orientation OS1, TopAbs_Orientation OS2,
                                       double Radius)
    {
        GeomAbs_SurfaceType typ = S.GetType();
        GeomAbs_SurfaceType typ1 = S1.GetType();
        GeomAbs_SurfaceType typ2 = S2.GetType();
        if (typ != GeomAbs_Plane || typ1 != GeomAbs_Plane || typ2 != GeomAbs_Plane)
        {
            throw new Standard_ConstructionError("torus joint only between the planes");
        }
        return ChFiKPart_MakeRotule(DStr, Data, S.Plane(), S1.Plane(), S2.Plane(), OS, OS1, OS2, Radius, OfS);
    }
}
