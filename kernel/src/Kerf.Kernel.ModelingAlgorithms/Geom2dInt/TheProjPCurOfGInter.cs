// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheProjPCurOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheProjPCurOfGInter_0.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheLocateExtPCOfTheProjPCurOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_PCLocFOfTheLocateExtPCOfTheProjPCurOfGInter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>The parameter of the point of a 2D curve nearest a given point.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersectors. OCCT's
/// Geom2dInt_TheLocateExtPCOfTheProjPCurOfGInter and Geom2dInt_PCLocFOfTheLocateExtPCOfTheProjPCurOfGInter are
/// Extrema_LocEPCOfLocateExtPC2d and Extrema_PCFOfEPCOfExtPC2d (decision 323).
/// </remarks>
internal readonly struct TheProjPCurOfGInter : IProjectOnPCurveTool<Curve2DAdaptorBase>
{
    /// <summary>
    /// The parameter of the point of <paramref name="c"/> nearest <paramref name="p"/>, searched between the two
    /// parameters: the nearest sample, refined by a local extremum search.
    /// </summary>
    public static double FindParameter(in Curve2DAdaptorBase c, in Point2D p, double lowParameter,
                                       double highParameter, double tol)
    {
        double theparam, defaultparam;
        int nbPts = Geom2dCurveTool.NbSamples(c);
        double theEpsX = Geom2dCurveTool.EpsX(c);

        TheCurveLocatorOfTheProjPCurOfGInter.Locate(p, c, nbPts, lowParameter, highParameter,
                                                              out POnCurv2d pOnC);
        defaultparam = pOnC.Parameter();
        LocEpcOfLocateExtPC2d loc = new(p, c, defaultparam, theEpsX);

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
    public static double FindParameter(in Curve2DAdaptorBase c, in Point2D p, double tol)
    {
        double theParam;
        theParam = FindParameter(c, p, Geom2dCurveTool.FirstParameter(c),
                                 Geom2dCurveTool.LastParameter(c), tol);
        return theParam;
    }
}
