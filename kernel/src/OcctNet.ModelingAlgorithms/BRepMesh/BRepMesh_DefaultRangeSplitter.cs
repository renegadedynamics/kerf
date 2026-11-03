// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DefaultRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DefaultRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The default tool defining the range of a discrete face and the grid points distributed in it. The base of the
/// range splitters of each surface type.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. OCCT's <c>std::pair&lt;double, double&gt;</c> are value tuples with the fields
/// <c>first</c> and <c>second</c>. The face algorithms of unit 4D are generics over the splitter
/// (<c>where RangeSplitter : BRepMesh_DefaultRangeSplitter, new()</c>), so the methods the splitters redefine are
/// virtual, as in OCCT; <see cref="Scale"/> and <see cref="Point"/> aren't, in OCCT either.
/// </remarks>
public class BRepMesh_DefaultRangeSplitter
{
    protected IMeshData_Face? myDFace;
    protected (double first, double second) myRangeU;
    protected (double first, double second) myRangeV;
    protected (double first, double second) myDelta;
    protected (double first, double second) myTolerance;
    protected bool myIsValid;

    /// <summary>A splitter; <see cref="Reset"/> must be called before its first use.</summary>
    public BRepMesh_DefaultRangeSplitter()
    {
        myIsValid = true;
    }

    /// <summary>Resets the splitter for the face. Must be called before the first use.</summary>
    public virtual void Reset(IMeshData_Face theDFace, in IMeshTools_Parameters theParameters)
    {
        myDFace = theDFace;
        myRangeU.first = myRangeV.first = 1.0e100;
        myRangeU.second = myRangeV.second = -1.0e100;
        myDelta.first = myDelta.second = 1.0;
        myTolerance.first = myTolerance.second = global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    /// <summary>Registers a border point.</summary>
    public virtual void AddPoint(in Point2D thePoint)
    {
        myRangeU.first = Min(thePoint.X(), myRangeU.first);
        myRangeU.second = Max(thePoint.X(), myRangeU.second);
        myRangeV.first = Min(thePoint.Y(), myRangeV.first);
        myRangeV.second = Max(thePoint.Y(), myRangeV.second);
    }

    /// <summary>Updates the discrete range of the surface from its geometric range.</summary>
    public virtual void AdjustRange()
    {
        BRepAdaptor_Surface aSurface = GetSurface();
        updateRange(aSurface.FirstUParameter(),
                    aSurface.LastUParameter(),
                    aSurface.IsUPeriodic(),
                    ref myRangeU.first,
                    ref myRangeU.second);

        if (myRangeU.second < myRangeU.first)
        {
            myIsValid = false;
            return;
        }

        updateRange(aSurface.FirstVParameter(),
                    aSurface.LastVParameter(),
                    aSurface.IsVPeriodic(),
                    ref myRangeV.first,
                    ref myRangeV.second);

        if (myRangeV.second < myRangeV.first)
        {
            myIsValid = false;
            return;
        }

        double aLengthU = computeLengthU();
        double aLengthV = computeLengthV();
        myIsValid = aLengthU > global::Kerf.Engine.Foundation.Precision.PConfusion() && aLengthV > global::Kerf.Engine.Foundation.Precision.PConfusion();

        if (myIsValid)
        {
            computeTolerance(aLengthU, aLengthV);
            computeDelta(aLengthU, aLengthV);
        }
    }

    /// <summary>True if the computed range is valid.</summary>
    public virtual bool IsValid() => myIsValid;

    /// <summary>
    /// The point scaled from the real parametric space to the face's basis if <paramref name="isToFaceBasis"/>, or
    /// back.
    /// </summary>
    public Point2D Scale(in Point2D thePoint, bool isToFaceBasis)
    {
        return isToFaceBasis ? new Point2D((thePoint.X() - myRangeU.first) / myDelta.first,
                                            (thePoint.Y() - myRangeV.first) / myDelta.second)
                             : new Point2D(thePoint.X() * myDelta.first + myRangeU.first,
                                            thePoint.Y() * myDelta.second + myRangeV.first);
    }

    /// <summary>The nodes generated from the surface and the parameters; none (null) by default.</summary>
    public virtual NCollection_List<Point2D>? GenerateSurfaceNodes(in IMeshTools_Parameters theParameters)
    {
        return null;
    }

    /// <summary>The 3D point of the point of the surface's parametric space.</summary>
    public Point3D Point(in Point2D thePoint2d) => GetSurface().Value(thePoint2d.X(), thePoint2d.Y());

    /// <summary>Computes the parametric tolerance from the lengths along U and V.</summary>
    protected virtual void computeTolerance(double theLenU, double theLenV)
    {
        double aDiffU = myRangeU.second - myRangeU.first;
        double aDiffV = myRangeV.second - myRangeV.first;

        // Slightly increase exact resolution so to cover links with approximate
        // length equal to resolution itself on sub-resolution differences.
        double aTolerance = BRep_Tool.Tolerance(myDFace!.GetFace());
        Adaptor3d_Surface aSurface = GetSurface();
        double aResU = aSurface.UResolution(aTolerance) * 1.1;
        double aResV = aSurface.VResolution(aTolerance) * 1.1;

        double aDeflectionUV = 1.0e-05;
        myTolerance.first = Max(Min(aDeflectionUV, aResU), 1e-7 * aDiffU);
        myTolerance.second = Max(Min(aDeflectionUV, aResV), 1e-7 * aDiffV);
    }

    /// <summary>Computes the parametric delta from the lengths along U and V and the tolerance.</summary>
    protected virtual void computeDelta(double theLengthU, double theLengthV)
    {
        double aDiffU = myRangeU.second - myRangeU.first;
        double aDiffV = myRangeV.second - myRangeV.first;

        myDelta.first = aDiffU / (theLengthU < myTolerance.first ? 1.0 : theLengthU);
        myDelta.second = aDiffV / (theLengthV < myTolerance.second ? 1.0 : theLengthV);
    }

    /// <summary>The face model.</summary>
    public IMeshData_Face GetDFace() => myDFace!;

    /// <summary>The surface.</summary>
    public BRepAdaptor_Surface GetSurface() => myDFace!.GetSurface();

    /// <summary>The U range.</summary>
    public (double first, double second) GetRangeU() => myRangeU;

    /// <summary>The V range.</summary>
    public (double first, double second) GetRangeV() => myRangeV;

    /// <summary>The delta.</summary>
    public (double first, double second) GetDelta() => myDelta;

    /// <summary>The parametric tolerances along U and V.</summary>
    public (double first, double second) GetToleranceUV() => myTolerance;

    // Computes the length along U by evaluating a grid.
    private double computeLengthU()
    {
        double longu = 0.0;
        Point3D P11, P12, P21, P22, P31, P32;

        double du = 0.05 * (myRangeU.second - myRangeU.first);
        double dfvave = 0.5 * (myRangeV.second + myRangeV.first);
        double dfucur;
        int i1;

        BRepAdaptor_Surface gFace = GetSurface();
        gFace.D0(myRangeU.first, myRangeV.first, out P11);
        gFace.D0(myRangeU.first, dfvave, out P21);
        gFace.D0(myRangeU.first, myRangeV.second, out P31);
        for (i1 = 1, dfucur = myRangeU.first + du; i1 <= 20; i1++, dfucur += du)
        {
            gFace.D0(dfucur, myRangeV.first, out P12);
            gFace.D0(dfucur, dfvave, out P22);
            gFace.D0(dfucur, myRangeV.second, out P32);
            longu += (P11.Distance(P12) + P21.Distance(P22) + P31.Distance(P32));
            P11 = P12;
            P21 = P22;
            P31 = P32;
        }

        return longu / 3.0;
    }

    // Computes the length along V by evaluating a grid.
    private double computeLengthV()
    {
        double longv = 0.0;
        Point3D P11, P12, P21, P22, P31, P32;

        double dv = 0.05 * (myRangeV.second - myRangeV.first);
        double dfuave = 0.5 * (myRangeU.second + myRangeU.first);
        double dfvcur;
        int i1;

        BRepAdaptor_Surface gFace = GetSurface();
        gFace.D0(myRangeU.first, myRangeV.first, out P11);
        gFace.D0(dfuave, myRangeV.first, out P21);
        gFace.D0(myRangeU.second, myRangeV.first, out P31);
        for (i1 = 1, dfvcur = myRangeV.first + dv; i1 <= 20; i1++, dfvcur += dv)
        {
            gFace.D0(myRangeU.first, dfvcur, out P12);
            gFace.D0(dfuave, dfvcur, out P22);
            gFace.D0(myRangeU.second, dfvcur, out P32);
            longv += (P11.Distance(P12) + P21.Distance(P22) + P31.Distance(P32));
            P11 = P12;
            P21 = P22;
            P31 = P32;
        }

        return longv / 3.0;
    }

    // Updates the discrete range of the surface from its geometric range.
    private static void updateRange(double theGeomFirst,
                                    double theGeomLast,
                                    bool isPeriodic,
                                    ref double theDiscreteFirst,
                                    ref double theDiscreteLast)
    {
        if (theDiscreteFirst < theGeomFirst || theDiscreteLast > theGeomLast)
        {
            if (isPeriodic)
            {
                if ((theDiscreteLast - theDiscreteFirst) > (theGeomLast - theGeomFirst))
                {
                    theDiscreteLast = theDiscreteFirst + (theGeomLast - theGeomFirst);
                }
            }
            else
            {
                if ((theDiscreteFirst < theGeomLast) && (theDiscreteLast > theGeomFirst))
                {
                    // Protection against the faces whose pcurve is out of the surface's domain
                    //(see issue #23675 and test cases "bugs iges buc60820*")
                    if (theGeomFirst > theDiscreteFirst)
                    {
                        theDiscreteFirst = theGeomFirst;
                    }

                    if (theGeomLast < theDiscreteLast)
                    {
                        theDiscreteLast = theGeomLast;
                    }
                }
            }
        }
    }
}
