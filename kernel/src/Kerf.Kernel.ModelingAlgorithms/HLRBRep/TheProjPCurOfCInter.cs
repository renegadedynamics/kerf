// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheProjPCurOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheProjPCurOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>The parameter of the point of a 2D curve nearest a given point.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersectors. Generated from
/// Geom2dInt_TheProjPCurOfGInter by substituting the curve and its tool (decision 1091).
/// </remarks>
internal readonly struct TheProjPCurOfCInter : IProjectOnPCurveTool<HiddenLineCurve>
{
    /// <summary>
    /// The parameter of the point of <paramref name="c"/> nearest <paramref name="p"/>, searched between the two
    /// parameters: the nearest sample, refined by a local extremum search.
    /// </summary>
    public static double FindParameter(in HiddenLineCurve c, in Point2D p, double lowParameter,
                                       double highParameter, double tol)
    {
        double theparam, defaultparam;
        int nbPts = HiddenLineCurveTool.NbSamples(c);
        double theEpsX = HiddenLineCurveTool.EpsX(c);

        TheCurveLocatorOfTheProjPCurOfCInter.Locate(p, c, nbPts, lowParameter, highParameter,
                                                              out POnCurv2d pOnC);
        defaultparam = pOnC.Parameter();
        TheLocateExtPCOfTheProjPCurOfCInter loc = new(p, c, defaultparam, theEpsX);

        if (!loc.IsDone)
        {
            //-- cout<<"\n Erreur dans LocateExtPC "<<endl;
            theparam = defaultparam;
        }
        else
        {
            if (!loc.IsMin)
            {
                //-- cout<<"\n Erreur dans LocateExtPC (Maximum trouve) "<<endl;
                theparam = defaultparam;
            }
            else
            {
                theparam = loc.Point().Parameter();
            }
        }
        return theparam;
    }

    /// <summary>The parameter of the point of <paramref name="c"/> nearest <paramref name="p"/>.</summary>
    public static double FindParameter(in HiddenLineCurve c, in Point2D p, double tol)
    {
        double theParam;
        theParam = FindParameter(c, p, HiddenLineCurveTool.FirstParameter(c),
                                 HiddenLineCurveTool.LastParameter(c), tol);
        return theParam;
    }
}
