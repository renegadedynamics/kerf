// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheProjPCurOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheProjPCurOfGInter_0.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheLocateExtPCOfTheProjPCurOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_PCLocFOfTheLocateExtPCOfTheProjPCurOfGInter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The parameter of the point of a 2D curve nearest a given point.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersectors. OCCT's
/// Geom2dInt_TheLocateExtPCOfTheProjPCurOfGInter and Geom2dInt_PCLocFOfTheLocateExtPCOfTheProjPCurOfGInter are
/// Extrema_LocEPCOfLocateExtPC2d and Extrema_PCFOfEPCOfExtPC2d (decision 323).
/// </remarks>
public readonly struct Geom2dInt_TheProjPCurOfGInter : IIntImpParGen_ProjectOnPCurveTool<Adaptor2d_Curve2d>
{
    /// <summary>
    /// The parameter of the point of <paramref name="C"/> nearest <paramref name="P"/>, searched between the two
    /// parameters: the nearest sample, refined by a local extremum search.
    /// </summary>
    public static double FindParameter(in Adaptor2d_Curve2d C, in Point2D P, double LowParameter,
                                       double HighParameter, double Tol)
    {
        double theparam, defaultparam;
        int NbPts = Geom2dInt_Geom2dCurveTool.NbSamples(C);
        double theEpsX = Geom2dInt_Geom2dCurveTool.EpsX(C);

        Geom2dInt_TheCurveLocatorOfTheProjPCurOfGInter.Locate(P, C, NbPts, LowParameter, HighParameter,
                                                              out Extrema_POnCurv2d POnC);
        defaultparam = POnC.Parameter();
        Extrema_LocEPCOfLocateExtPC2d Loc = new(P, C, defaultparam, theEpsX);

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
    public static double FindParameter(in Adaptor2d_Curve2d C, in Point2D P, double Tol)
    {
        double theParam;
        theParam = FindParameter(C, P, Geom2dInt_Geom2dCurveTool.FirstParameter(C),
                                 Geom2dInt_Geom2dCurveTool.LastParameter(C), Tol);
        return theParam;
    }
}
