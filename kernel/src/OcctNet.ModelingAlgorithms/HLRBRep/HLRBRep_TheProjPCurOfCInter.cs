// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheProjPCurOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheProjPCurOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The parameter of the point of a 2D curve nearest a given point.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersectors. Generated from
/// Geom2dInt_TheProjPCurOfGInter by substituting the curve and its tool (decision 1091).
/// </remarks>
public readonly struct HLRBRep_TheProjPCurOfCInter : IIntImpParGen_ProjectOnPCurveTool<HLRBRep_Curve>
{
    /// <summary>
    /// The parameter of the point of <paramref name="C"/> nearest <paramref name="P"/>, searched between the two
    /// parameters: the nearest sample, refined by a local extremum search.
    /// </summary>
    public static double FindParameter(in HLRBRep_Curve C, in Point2D P, double LowParameter,
                                       double HighParameter, double Tol)
    {
        double theparam, defaultparam;
        int NbPts = HLRBRep_CurveTool.NbSamples(C);
        double theEpsX = HLRBRep_CurveTool.EpsX(C);

        HLRBRep_TheCurveLocatorOfTheProjPCurOfCInter.Locate(P, C, NbPts, LowParameter, HighParameter,
                                                              out Extrema_POnCurv2d POnC);
        defaultparam = POnC.Parameter();
        HLRBRep_TheLocateExtPCOfTheProjPCurOfCInter Loc = new(P, C, defaultparam, theEpsX);

        if (!Loc.IsDone())
        {
            //-- cout<<"\n Erreur dans LocateExtPC "<<endl;
            theparam = defaultparam;
        }
        else
        {
            if (!Loc.IsMin())
            {
                //-- cout<<"\n Erreur dans LocateExtPC (Maximum trouve) "<<endl;
                theparam = defaultparam;
            }
            else
            {
                theparam = Loc.Point().Parameter();
            }
        }
        return theparam;
    }

    /// <summary>The parameter of the point of <paramref name="C"/> nearest <paramref name="P"/>.</summary>
    public static double FindParameter(in HLRBRep_Curve C, in Point2D P, double Tol)
    {
        double theParam;
        theParam = FindParameter(C, P, HLRBRep_CurveTool.FirstParameter(C),
                                 HLRBRep_CurveTool.LastParameter(C), Tol);
        return theParam;
    }
}
