// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_TopolTool.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_TopolTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The topological tool of the surface/surface intersections of the Boolean component: Adaptor3d_TopolTool with its
/// own sample points (a regular grid sized by the surface type, strictly inside the parametric range).
/// </summary>
/// <remarks>A working object (decision 21): the sample grid is computed on first use and kept.</remarks>
public class IntTools_TopolTool : Adaptor3d_TopolTool
{
    private int myNbSmplU;
    private int myNbSmplV;
    private double myU0;
    private double myV0;
    private double myDU;
    private double myDV;

    /// <summary>An empty tool.</summary>
    public IntTools_TopolTool()
    {
        myNbSmplU = 0;
        myNbSmplV = 0;
        myDU = 1.0;
        myDV = 1.0;
    }

    /// <summary>The tool of the surface.</summary>
    public IntTools_TopolTool(Adaptor3d_Surface theSurface)
    {
        Initialize(theSurface);
        myNbSmplU = 0;
        myNbSmplV = 0;
        myDU = 1.0;
        myDV = 1.0;
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as in OCCT.</summary>
    public override void Initialize() => throw new MethodNotImplementedException("IntTools_TopolTool::Initialize ()");

    /// <summary>Initializes the tool with the surface.</summary>
    public override void Initialize(Adaptor3d_Surface theSurface)
    {
        base.Initialize(theSurface);
        // myS = theSurface;
        myNbSmplU = 0;
        myNbSmplV = 0;
        myDU = 1.0;
        myDV = 1.0;
    }

    /// <summary>Computes the sample grid: its size by the surface type, at most 50 per direction for most types.</summary>
    public override void ComputeSamplePoints()
    {
        double uinf, usup, vinf, vsup;
        uinf = myS!.FirstUParameter();
        usup = myS.LastUParameter();
        vinf = myS.FirstVParameter();
        vsup = myS.LastVParameter();
        const int aMaxNbSample = 50;

        if (usup < uinf)
        {
            double temp = uinf;
            uinf = usup;
            usup = temp;
        }
        if (vsup < vinf)
        {
            double temp = vinf;
            vinf = vsup;
            vsup = temp;
        }
        bool isbiguinf, isbigusup, isbigvinf, isbigvsup;
        isbiguinf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uinf);
        isbigusup = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(usup);
        isbigvinf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vinf);
        isbigvsup = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vsup);

        if (isbiguinf && isbigusup)
        {
            uinf = -1.0e5;
            usup = 1.0e5;
        }
        else if (isbiguinf)
        {
            uinf = usup - 2.0e5;
        }
        else if (isbigusup)
        {
            usup = uinf + 2.0e5;
        }

        if (isbigvinf && isbigvsup)
        {
            vinf = -1.0e5;
            vsup = 1.0e5;
        }
        else if (isbigvinf)
        {
            vinf = vsup - 2.0e5;
        }
        else if (isbigvsup)
        {
            vsup = vinf + 2.0e5;
        }
        myU0 = uinf;
        myV0 = vinf;

        int nbsu = 0, nbsv = 0;
        SurfaceKind typS = myS.GetType();

        switch (typS)
        {
            case Plane:
                {
                    nbsu = 10;
                    nbsv = 10;
                }
                break;
            case SurfaceKind.Cylinder:
                {
                    double aRadius = myS.Cylinder().Radius();
                    double aMaxAngle = M_PI * 0.5;
                    double aDeflection = 1.0e-02;

                    if (aRadius > aDeflection)
                    {
                        aMaxAngle = Math.Acos(1.0 - aDeflection / aRadius) * 2.0;
                    }
                    if (aMaxAngle > global::Kerf.Engine.Foundation.Precision.Angular())
                    {
                        nbsu = (int)((usup - uinf) / aMaxAngle);
                    }
                    nbsv = (int)(vsup - vinf);
                    nbsv /= 10;

                    if (nbsu < 2)
                    {
                        nbsu = 2;
                    }
                    if (nbsv < 2)
                    {
                        nbsv = 2;
                    }

                    //    if(nbsu < 10) nbsu = 10;
                    //    if(nbsv < 10) nbsv = 10;
                    if (nbsu > aMaxNbSample)
                    {
                        nbsu = aMaxNbSample;
                    }
                    if (nbsv > aMaxNbSample)
                    {
                        nbsv = aMaxNbSample;
                    }
                }
                break;
            case SurfaceKind.Cone:
                {
                    Cone aCone = myS.Cone();
                    Circle3D aCircle = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(aCone.Position(), aCone.RefRadius(), aCone.SemiAngle(), vinf);
                    double aRadius = aCircle.Radius();
                    aCircle = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(aCone.Position(), aCone.RefRadius(), aCone.SemiAngle(), vsup);

                    if (aRadius < aCircle.Radius())
                    {
                        aRadius = aCircle.Radius();
                    }
                    double aMaxAngle = M_PI * 0.5;
                    double aDeflection = 1.0e-02;

                    if (aRadius > aDeflection)
                    {
                        aMaxAngle = Math.Acos(1.0 - aDeflection / aRadius) * 2.0;
                    }

                    if (aMaxAngle > global::Kerf.Engine.Foundation.Precision.Angular())
                    {
                        nbsu = (int)((usup - uinf) / aMaxAngle);
                    }
                    nbsv = (int)(vsup - vinf);
                    nbsv /= 10;

                    //     if(nbsu < 2) nbsu = 2;
                    //     if(nbsv < 2) nbsv = 2;

                    if (nbsu < 10)
                    {
                        nbsu = 10;
                    }
                    if (nbsv < 10)
                    {
                        nbsv = 10;
                    }
                    if (nbsu > aMaxNbSample)
                    {
                        nbsu = aMaxNbSample;
                    }
                    if (nbsv > aMaxNbSample)
                    {
                        nbsv = aMaxNbSample;
                    }
                }
                break;
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                {
                    Circle3D aCircle;
                    double aRadius1, aRadius2;

                    if (typS == SurfaceKind.Torus)
                    {
                        Torus aTorus = myS.Torus();
                        aCircle = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(aTorus.Position(), aTorus.MajorRadius(), aTorus.MinorRadius(), uinf);
                        aRadius2 = aCircle.Radius();
                        aCircle = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(aTorus.Position(), aTorus.MajorRadius(), aTorus.MinorRadius(), usup);
                        aRadius2 = (aRadius2 < aCircle.Radius()) ? aCircle.Radius() : aRadius2;

                        aCircle = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(aTorus.Position(), aTorus.MajorRadius(), aTorus.MinorRadius(), vinf);
                        aRadius1 = aCircle.Radius();
                        aCircle = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(aTorus.Position(), aTorus.MajorRadius(), aTorus.MinorRadius(), vsup);
                        aRadius1 = (aRadius1 < aCircle.Radius()) ? aCircle.Radius() : aRadius1;
                    }
                    else
                    {
                        Sphere aSphere = myS.Sphere();
                        aRadius1 = aSphere.Radius();
                        aRadius2 = aSphere.Radius();
                    }
                    double aMaxAngle = M_PI * 0.5;
                    double aDeflection = 1.0e-02;

                    if (aRadius1 > aDeflection)
                    {
                        aMaxAngle = Math.Acos(1.0 - aDeflection / aRadius1) * 2.0;
                    }

                    if (aMaxAngle > global::Kerf.Engine.Foundation.Precision.Angular())
                    {
                        nbsu = (int)((usup - uinf) / aMaxAngle);
                    }
                    aMaxAngle = M_PI * 0.5;

                    if (aRadius2 > aDeflection)
                    {
                        aMaxAngle = Math.Acos(1.0 - aDeflection / aRadius2) * 2.0;
                    }

                    if (aMaxAngle > global::Kerf.Engine.Foundation.Precision.Angular())
                    {
                        nbsv = (int)((vsup - vinf) / aMaxAngle);
                    }
                    if (nbsu < 10)
                    {
                        nbsu = 10;
                    }
                    if (nbsv < 10)
                    {
                        nbsv = 10;
                    }
                    if (nbsu > aMaxNbSample)
                    {
                        nbsu = aMaxNbSample;
                    }
                    if (nbsv > aMaxNbSample)
                    {
                        nbsv = aMaxNbSample;
                    }
                }
                break;
            case BezierSurface:
                {
                    nbsv = 3 + myS.NbVPoles();
                    nbsu = 3 + myS.NbUPoles();

                    if (nbsu > 10 || nbsv > 10)
                    {
                        NCollection_Array2<Point3D> array2 = myS.Bezier().InternalPoles();
                        Analyse(array2, out nbsu, out nbsv);
                    }

                    if (nbsu < 10)
                    {
                        nbsu = 10;
                    }
                    if (nbsv < 10)
                    {
                        nbsv = 10;
                    }
                }
                break;
            case BSplineSurface:
                {
                    nbsv = myS.NbVKnots();
                    nbsv *= myS.VDegree();
                    if (nbsv < 4)
                    {
                        nbsv = 4;
                    }
                    nbsu = myS.NbUKnots();
                    nbsu *= myS.UDegree();
                    if (nbsu < 4)
                    {
                        nbsu = 4;
                    }

                    if (nbsu > 10 || nbsv > 10)
                    {
                        NCollection_Array2<Point3D> array2 = myS.BSpline().InternalPoles();
                        Analyse(array2, out nbsu, out nbsv);
                    }
                    if (nbsu < 10)
                    {
                        nbsu = 10;
                    }
                    if (nbsv < 10)
                    {
                        nbsv = 10;
                    }
                    // Check anisotropy
                    double anULen = (usup - uinf) / myS.UResolution(1.0);
                    double anVLen = (vsup - vinf) / myS.VResolution(1.0);
                    double aRatio = anULen / anVLen;
                    if (aRatio >= 10.0)
                    {
                        nbsu *= 2;
                        nbsu = Min(nbsu, aMaxNbSample);
                    }
                    else if (aRatio <= 0.1)
                    {
                        nbsv *= 2;
                        nbsv = Min(nbsv, aMaxNbSample);
                    }
                }
                break;
            case SurfaceOfExtrusion:
                {
                    nbsu = 15;
                    nbsv = (int)(vsup - vinf);
                    nbsv /= 10;
                    if (nbsv < 15)
                    {
                        nbsv = 15;
                    }
                    if (nbsv > aMaxNbSample)
                    {
                        nbsv = aMaxNbSample;
                    }
                }
                break;
            case SurfaceOfRevolution:
                {
                    nbsv = 15;
                    nbsu = 15;
                }
                break;
            default:
                {
                    nbsu = 10;
                    nbsv = 10;
                }
                break;
        }
        myNbSmplU = nbsu;
        myNbSmplV = nbsv;

        myNbSamplesU = myNbSmplU;
        myNbSamplesV = myNbSmplV;

        myDU = (usup - uinf) / (myNbSmplU + 1);
        myDV = (vsup - vinf) / (myNbSmplV + 1);
    }

    /// <summary>The number of samples in U (computing the grid on first use).</summary>
    public override int NbSamplesU()
    {
        if (myNbSmplU <= 0)
        {
            ComputeSamplePoints();
        }
        return myNbSmplU;
    }

    /// <summary>The number of samples in V (computing the grid on first use).</summary>
    public override int NbSamplesV()
    {
        if (myNbSmplV <= 0)
        {
            ComputeSamplePoints();
        }
        return myNbSmplV;
    }

    /// <summary>The number of sample points (computing the grid on first use).</summary>
    public override int NbSamples()
    {
        if (myNbSmplU <= 0)
        {
            ComputeSamplePoints();
        }
        return (myNbSmplU * myNbSmplV);
    }

    /// <summary>The sample point of the index (1 to NbSamples) in the parameter space and in 3D.</summary>
    public override void SamplePoint(int Index, out Point2D P2d, out Point3D P3d)
    {
        if (myUPars is null)
        {
            if (myNbSmplU <= 0)
            {
                ComputeSamplePoints();
            }
            int iv = 1 + Index / myNbSmplU;
            int iu = 1 + Index - (iv - 1) * myNbSmplU;
            double u = myU0 + iu * myDU;
            double v = myV0 + iv * myDV;
            P2d = new Point2D(u, v);
            P3d = myS!.Value(u, v);
        }
        else
        {
            base.SamplePoint(Index, out P2d, out P3d);
        }
    }

    // OCCT's file-static Analyse: the number of curvature sign changes along the pole rows and columns, plus 5.
    // As OCCT, the first three poles of each row are read at columns 1 to 3 whatever the array's lower column.
    private static void Analyse(NCollection_Array2<Point3D> array2, out int theNbSamplesU, out int theNbSamplesV)
    {
        Vector3D Vi = new(), Vip1 = new();
        int sh, nbch, i, j;
        int nbup = array2.UpperRow() - array2.LowerRow() + 1;
        int nbvp = array2.UpperCol() - array2.LowerCol() + 1;

        sh = 1;
        nbch = 0;
        if (nbvp > 2)
        {

            for (i = array2.LowerRow(); i <= array2.UpperRow(); i++)
            {
                Point3D A = array2.Value(i, 1);
                Point3D B = array2.Value(i, 2);
                Point3D C = array2.Value(i, 3);
                Vi.SetCoord(C.X() - B.X() - B.X() + A.X(),
                            C.Y() - B.Y() - B.Y() + A.Y(),
                            C.Z() - B.Z() - B.Z() + A.Z());
                int locnbch = 0;

                for (j = array2.LowerCol() + 2; j < array2.UpperCol(); j++)
                { //-- essai
                    Point3D Ax = array2.Value(i, j - 1);
                    Point3D Bx = array2.Value(i, j);
                    Point3D Cx = array2.Value(i, j + 1);
                    Vip1.SetCoord(Cx.X() - Bx.X() - Bx.X() + Ax.X(),
                                  Cx.Y() - Bx.Y() - Bx.Y() + Ax.Y(),
                                  Cx.Z() - Bx.Z() - Bx.Z() + Ax.Z());
                    double pd = Vi.Dot(Vip1);
                    Vi = Vip1;
                    if (pd > 1.0e-7 || pd < -1.0e-7)
                    {
                        if (pd > 0)
                        {
                            if (sh == -1)
                            {
                                sh = 1;
                                locnbch++;
                            }
                        }
                        else
                        {
                            if (sh == 1)
                            {
                                sh = -1;
                                locnbch++;
                            }
                        }
                    }
                }
                if (locnbch > nbch)
                {
                    nbch = locnbch;
                }
            }
        }
        theNbSamplesV = nbch + 5;

        nbch = 0;
        if (nbup > 2)
        {
            for (j = array2.LowerCol(); j <= array2.UpperCol(); j++)
            {
                Point3D A = array2.Value(array2.LowerRow(), j);
                Point3D B = array2.Value(array2.LowerRow() + 1, j);
                Point3D C = array2.Value(array2.LowerRow() + 2, j);
                Vi.SetCoord(C.X() - B.X() - B.X() + A.X(),
                            C.Y() - B.Y() - B.Y() + A.Y(),
                            C.Z() - B.Z() - B.Z() + A.Z());
                int locnbch = 0;

                for (i = array2.LowerRow() + 2; i < array2.UpperRow(); i++)
                { //-- essai
                    Point3D Ax = array2.Value(i - 1, j);
                    Point3D Bx = array2.Value(i, j);
                    Point3D Cx = array2.Value(i + 1, j);
                    Vip1.SetCoord(Cx.X() - Bx.X() - Bx.X() + Ax.X(),
                                  Cx.Y() - Bx.Y() - Bx.Y() + Ax.Y(),
                                  Cx.Z() - Bx.Z() - Bx.Z() + Ax.Z());
                    double pd = Vi.Dot(Vip1);
                    Vi = Vip1;
                    if (pd > 1.0e-7 || pd < -1.0e-7)
                    {
                        if (pd > 0)
                        {
                            if (sh == -1)
                            {
                                sh = 1;
                                locnbch++;
                            }
                        }
                        else
                        {
                            if (sh == 1)
                            {
                                sh = -1;
                                locnbch++;
                            }
                        }
                    }
                }
                if (locnbch > nbch)
                {
                    nbch = locnbch;
                }
            }
        }
        theNbSamplesU = nbch + 5;
    }

    // Modified IFV
    /// <summary>
    /// Computes the sample parameters adaptively (Adaptor3d_TopolTool.SamplePnts), then takes the grid's origin and
    /// steps from them.
    /// </summary>
    public override void SamplePnts(double theDefl, int theNUmin, int theNVmin)
    {
        base.SamplePnts(theDefl, theNUmin, theNVmin);

        myNbSmplU = base.NbSamplesU();
        myNbSmplV = base.NbSamplesV();

        myU0 = myUPars!.Value(1);
        myV0 = myVPars!.Value(1);

        myDU = (myUPars.Value(myNbSmplU) - myU0) / (myNbSmplU - 1);
        myDV = (myVPars.Value(myNbSmplV) - myV0) / (myNbSmplV - 1);
    }
}
