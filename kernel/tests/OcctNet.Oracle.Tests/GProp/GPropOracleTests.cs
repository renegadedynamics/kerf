// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GProp_GProps and GProp_PrincipalProps against native OCCT. Native global properties come from BRepGProp on
/// randomly placed primitives (box, cylinder, cone, sphere, torus, wedge; volume, surface and linear properties).
/// The port can't compute those (BRepGProp isn't ported), so the port's GProp_GProps is loaded with the native mass,
/// centre of mass and central matrix of inertia, and the queries built on them are compared: MatrixOfInertia,
/// StaticMoments, MomentOfInertia, RadiusOfGyration and PrincipalProperties (moments, axes, radii, symmetry), then
/// Add with densities and a third reference point.
/// </summary>
public class GPropOracleTests(ITestOutputHelper output)
{
    private const int Cases = 150;

    /// <summary>A GProp_GProps holding given properties, with the reference point at the centre of mass.</summary>
    private sealed class LoadedProps : GProp_GProps
    {
        public LoadedProps(double theMass, in gp_Pnt theCentre, in gp_Mat theInertia)
        {
            loc = theCentre;
            g = Occt.gp.Origin();
            dim = theMass;
            inertia = theInertia;
        }
    }

    /// <summary>
    /// An axis built on both sides from the same raw components, so that native OCCT and the port normalize the same
    /// numbers (normalizing the port's unit direction again can change its last bit).
    /// </summary>
    private static void Ax1(Oracle theOracle, out gp_Ax1 thePort, out N.gp_Ax1 theNative)
    {
        gp_Pnt aP = theOracle.Pnt();
        double x, y, z;
        do
        {
            x = theOracle.Real(-1, 1);
            y = theOracle.Real(-1, 1);
            z = theOracle.Real(-1, 1);
        } while (x * x + y * y + z * z < 0.01);
        thePort = new gp_Ax1(aP, new gp_Dir(x, y, z));
        using N.gp_Pnt aNP = Oracle.ToNative(aP);
        using var aND = new N.gp_Dir(x, y, z);
        theNative = new N.gp_Ax1(aNP, aND);
    }

    private static N.TopoDS_Shape MakeShape(Oracle theOracle, out string theName)
    {
        gp_Ax2 anAx = theOracle.Ax2();
        using N.gp_Ax2 aNAx = Oracle.ToNative(anAx);
        double a = theOracle.Real(0.5, 20.0), b = theOracle.Real(0.5, 20.0), c = theOracle.Real(0.5, 20.0);
        switch (theOracle.Int(0, 5))
        {
            case 0:
                {
                    theName = "box";
                    using var aMk = new N.BRepPrimAPI_MakeBox(aNAx, a, b, c);
                    return aMk.Shape();
                }
            case 1:
                {
                    theName = "cylinder";
                    using var aMk = new N.BRepPrimAPI_MakeCylinder(aNAx, a, b);
                    return aMk.Shape();
                }
            case 2:
                {
                    theName = "cone";
                    using var aMk = new N.BRepPrimAPI_MakeCone(aNAx, a, theOracle.Int(0, 1) == 0 ? 0.0 : b, c);
                    return aMk.Shape();
                }
            case 3:
                {
                    theName = "sphere";
                    using var aMk = new N.BRepPrimAPI_MakeSphere(aNAx, a);
                    return aMk.Shape();
                }
            case 4:
                {
                    theName = "torus";
                    using var aMk = new N.BRepPrimAPI_MakeTorus(aNAx, a + b, Math.Min(a, b) * 0.9);
                    return aMk.Shape();
                }
            default:
                {
                    theName = "wedge";
                    using var aMk = new N.BRepPrimAPI_MakeWedge(aNAx, a, b, c, theOracle.Real(0.0, a));
                    return aMk.Shape();
                }
        }
    }

    private static N.GProp_GProps NativeProps(N.TopoDS_Shape theShape, int theKind)
    {
        var aProps = new N.GProp_GProps();
        switch (theKind)
        {
            case 0: N.BRepGProp.VolumeProperties(theShape, aProps); break;
            case 1: N.BRepGProp.SurfaceProperties(theShape, aProps); break;
            default: N.BRepGProp.LinearProperties(theShape, aProps); break;
        }
        return aProps;
    }

    private static LoadedProps Load(N.GProp_GProps theNative)
    {
        using N.gp_Pnt aG = theNative.CentreOfMass();
        using N.gp_Mat aM = theNative.MatrixOfInertia();
        return new LoadedProps(theNative.Mass(), Oracle.FromNative(aG), Oracle.FromNative(aM));
    }

    /// <summary>Matrices compared element by element relative to their largest native element.</summary>
    private static void SameMat(Oracle theOracle, N.gp_Mat theNative, in gp_Mat thePort, string theWhat, double theTol)
    {
        double aScale = 1.0;
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aScale = Math.Max(aScale, Math.Abs(theNative.Value(i, j)));
            }
        }
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                theOracle.Same(theNative.Value(i, j) / aScale, thePort.Value(i, j) / aScale, $"{theWhat}({i},{j})", theTol);
            }
        }
    }

    [Fact]
    public void Queries()
    {
        var anOracle = new Oracle(output);
        for (int aCase = 0; aCase < Cases; aCase++)
        {
            using N.TopoDS_Shape aShape = MakeShape(anOracle, out string aName);
            for (int aKind = 0; aKind < 3; aKind++)
            {
                string aWhat = $"case {aCase} {aName} kind {aKind}";
                using N.GProp_GProps aNative = NativeProps(aShape, aKind);
                LoadedProps aPort = Load(aNative);

                anOracle.Same(aNative.Mass(), aPort.Mass(), aWhat + " Mass");
                using (N.gp_Pnt aNG = aNative.CentreOfMass())
                {
                    anOracle.Same(aNG, aPort.CentreOfMass(), aWhat + " CentreOfMass");
                }
                using (N.gp_Mat aNM = aNative.MatrixOfInertia())
                {
                    SameMat(anOracle, aNM, aPort.MatrixOfInertia(), aWhat + " MatrixOfInertia", Oracle.Tolerance);
                }

                double aNIx = 0, aNIy = 0, aNIz = 0;
                aNative.StaticMoments(ref aNIx, ref aNIy, ref aNIz);
                aPort.StaticMoments(out double aPIx, out double aPIy, out double aPIz);
                anOracle.Same(aNIx, aPIx, aWhat + " Ix");
                anOracle.Same(aNIy, aPIy, aWhat + " Iy");
                anOracle.Same(aNIz, aPIz, aWhat + " Iz");

                for (int k = 0; k < 3; k++)
                {
                    Ax1(anOracle, out gp_Ax1 anAxis, out N.gp_Ax1 aNAxis);
                    using N.gp_Ax1 aNAxisOwner = aNAxis; // disposes the native axis
                    anOracle.Same(aNative.MomentOfInertia(aNAxis), aPort.MomentOfInertia(anAxis), aWhat + " MomentOfInertia");
                    anOracle.Same(aNative.RadiusOfGyration(aNAxis), aPort.RadiusOfGyration(anAxis), aWhat + " RadiusOfGyration");
                }

                using N.GProp_PrincipalProps aNPP = aNative.PrincipalProperties();
                GProp_PrincipalProps aPPP = aPort.PrincipalProperties();
                double aN1 = 0, aN2 = 0, aN3 = 0;
                aNPP.Moments(ref aN1, ref aN2, ref aN3);
                aPPP.Moments(out double aP1, out double aP2, out double aP3);
                anOracle.Same(aN1, aP1, aWhat + " I1");
                anOracle.Same(aN2, aP2, aWhat + " I2");
                anOracle.Same(aN3, aP3, aWhat + " I3");
                aNPP.RadiusOfGyration(ref aN1, ref aN2, ref aN3);
                aPPP.RadiusOfGyration(out aP1, out aP2, out aP3);
                anOracle.Same(aN1, aP1, aWhat + " R1");
                anOracle.Same(aN2, aP2, aWhat + " R2");
                anOracle.Same(aN3, aP3, aWhat + " R3");
                using (N.gp_Vec aV = aNPP.FirstAxisOfInertia())
                {
                    anOracle.Same(aV, aPPP.FirstAxisOfInertia(), aWhat + " Axis1");
                }
                using (N.gp_Vec aV = aNPP.SecondAxisOfInertia())
                {
                    anOracle.Same(aV, aPPP.SecondAxisOfInertia(), aWhat + " Axis2");
                }
                using (N.gp_Vec aV = aNPP.ThirdAxisOfInertia())
                {
                    anOracle.Same(aV, aPPP.ThirdAxisOfInertia(), aWhat + " Axis3");
                }
                anOracle.Same(aNPP.HasSymmetryAxis(), aPPP.HasSymmetryAxis(), aWhat + " HasSymmetryAxis");
                anOracle.Same(aNPP.HasSymmetryPoint(), aPPP.HasSymmetryPoint(), aWhat + " HasSymmetryPoint");
                anOracle.Same(aNPP.HasSymmetryAxis(1e-6), aPPP.HasSymmetryAxis(1e-6), aWhat + " HasSymmetryAxis(tol)");
                anOracle.Same(aNPP.HasSymmetryPoint(1e-6), aPPP.HasSymmetryPoint(1e-6), aWhat + " HasSymmetryPoint(tol)");
            }
        }
        anOracle.Report("GProp_GProps queries and GProp_PrincipalProps");
    }

    /// <summary>
    /// Add with densities into a system with its own reference point. The native components keep BRepGProp's
    /// internal reference points while the port's are at the centres of mass, so Huygens' theorem could run on
    /// different numbers; the tolerances allow for that, although the results have so far been bit-identical.
    /// </summary>
    [Fact]
    public void Add()
    {
        var anOracle = new Oracle(output, 20261029);
        for (int aCase = 0; aCase < Cases; aCase++)
        {
            string aWhat = $"case {aCase}";
            gp_Pnt aLoc = anOracle.Pnt();
            using N.gp_Pnt aNLoc = Oracle.ToNative(aLoc);
            using var aNative = new N.GProp_GProps(aNLoc);
            var aPort = new GProp_GProps(aLoc);
            int aNb = anOracle.Int(1, 4);
            for (int i = 0; i < aNb; i++)
            {
                using N.TopoDS_Shape aShape = MakeShape(anOracle, out _);
                using N.GProp_GProps aNItem = NativeProps(aShape, anOracle.Int(0, 2));
                double aDensity = anOracle.Real(0.1, 10.0);
                aNative.Add(aNItem, aDensity);
                aPort.Add(Load(aNItem), aDensity);
            }

            anOracle.Same(aNative.Mass(), aPort.Mass(), aWhat + " Mass", 1e-10);
            using (N.gp_Pnt aNG = aNative.CentreOfMass())
            {
                anOracle.Same(aNG, aPort.CentreOfMass(), aWhat + " CentreOfMass", 1e-10);
            }
            using (N.gp_Mat aNM = aNative.MatrixOfInertia())
            {
                SameMat(anOracle, aNM, aPort.MatrixOfInertia(), aWhat + " MatrixOfInertia", 1e-9);
            }
            Ax1(anOracle, out gp_Ax1 anAxis, out N.gp_Ax1 aNAxis);
            using N.gp_Ax1 aNAxisOwner = aNAxis; // disposes the native axis
            anOracle.Same(aNative.MomentOfInertia(aNAxis), aPort.MomentOfInertia(anAxis), aWhat + " MomentOfInertia", 1e-9);

            // A copy behaves as the original.
            var aCopy = new GProp_GProps(aPort);
            anOracle.Same(aPort.Mass(), aCopy.Mass(), aWhat + " copy Mass");
        }
        anOracle.Report("GProp_GProps.Add");
    }
}
