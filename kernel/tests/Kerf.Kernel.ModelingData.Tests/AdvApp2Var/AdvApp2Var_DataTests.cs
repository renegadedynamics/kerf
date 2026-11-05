// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.ModelingData.Tests;

// The generated tables of AdvApp2Var_Data*.pxx: sizes, field boundaries and spot values copied from OCCT's source.
public class AdvApp2Var_DataTests
{
    [Fact]
    public void Mlgdrtl_FieldsAndValues()
    {
        global::Kerf.Engine.Approximation.Advanced.MlgdrtlCommon aTab = global::Kerf.Engine.Approximation.Advanced.Data.Getmlgdrtl();
        Assert.Equal(1891, aTab.rootab.AsSpan().Length);
        Assert.Equal(.577350269189625764509148780501958, aTab.rootab[0]);
        Assert.Equal(.999235597631363693039497410657766, aTab.rootab[929]);
        Assert.Equal(1.0, aTab.hiltab[0]);
        Assert.Equal(.00196145336166974360252310730973366, aTab.hiltab[929]);
        Assert.Equal(2.0, aTab.hi0tab[0]);
        Assert.Equal(.0510811194407819869756124378422272, aTab.hi0tab[30]);
        Assert.Equal(31, aTab.hi0tab.AsSpan().Length);
        // OCCT's struct is a cast of one C array: an index past a field reads the next one
        Assert.Equal(aTab.hiltab[0], aTab.rootab[930]);
        // the first Gauss root is 1/sqrt(3)
        Assert.Equal(1.0 / Math.Sqrt(3.0), aTab.rootab[0], 1e-15);
    }

    [Fact]
    public void Mmjcobi_FieldsAndValues()
    {
        global::Kerf.Engine.Approximation.Advanced.MmjcobiCommon aTab = global::Kerf.Engine.Approximation.Advanced.Data.Getmmjcobi();
        Assert.Equal(7936, aTab.plgcan.AsSpan().Length);
        Assert.Equal(3968, aTab.canjac.AsSpan().Length);
        Assert.Equal(.70710678118654752440084436210485, aTab.plgcan[0]);
        Assert.Equal(-972360558767407529.66171973925795, aTab.plgcan[3967]);
        Assert.Equal(1.4142135623730950488016887242097, aTab.canjac[0]);
        Assert.Equal(-1.0284250949746758839876395691897e-18, aTab.canjac[3967]);
    }

    [Fact]
    public void Mmcmcnp_IsTheSymmetricBinomialTable()
    {
        global::Kerf.Engine.Approximation.Advanced.MmcmcnpCommon aTab = global::Kerf.Engine.Approximation.Advanced.Data.Getmmcmcnp();
        Assert.Equal(3721, aTab.cnp.AsSpan().Length);
        // CNP(N, P) = N! / (P! (N - P)!), stored for N and P from 0 to 60, with CNP(I, J) = CNP(J, I)
        Assert.Equal(1.0, aTab.cnp[0]);
        Assert.Equal(2.0, aTab.cnp[2 + 1 * 61]);
        Assert.Equal(10.0, aTab.cnp[5 + 2 * 61]);
        Assert.Equal(10.0, aTab.cnp[2 + 5 * 61]);
        Assert.Equal(252.0, aTab.cnp[10 + 5 * 61]);
        Assert.Equal(1.0, aTab.cnp[3720]);
    }

    [Theory]
    [InlineData(0, 5017, 52, .256456162653232608756829902261057, 3.12208586703208853875278926933817e-4,
                .14324444858641334281356674080975, .0407561407383435779483091365130057)]
    [InlineData(1, 4761, 49, .339350899760562168358542462460598, .0064890083232969914090881960499836,
                .196145539320418982378623767812232, -.0407672808617546723713899270255496)]
    [InlineData(2, 4505, 46, .375693680937234041301644955593695, 9.32027794866696068097791608666759e-4,
                .224712945290939265035185233386492, .0408007928122239588413962075706306)]
    [InlineData(3, 4249, 43, .39632048021637914764176155847957, 4.69749156882345934031014312986152e-5,
                .245304484480172770561464997165472, -.0408569535997322120997291328426552)]
    public void Mmapgs_FieldsAndValues(int theTable, int theNbX, int theNb0, double theFirstX, double theLastX,
                                       double theFirst0, double theLast0)
    {
        GetGaussTable(theTable, out double aFirstX, out double aLastX, out double aFirst0, out double aLast0,
                      out int aTotal, out int aLength0);
        Assert.Equal(theNbX + theNb0, aTotal);
        Assert.Equal(theNb0, aLength0);
        Assert.Equal(theFirstX, aFirstX);
        Assert.Equal(theLastX, aLastX);
        Assert.Equal(theFirst0, aFirst0);
        Assert.Equal(theLast0, aLast0);
    }

    private static void GetGaussTable(int theTable, out double theFirstX, out double theLastX, out double theFirst0,
                                      out double theLast0, out int theTotal, out int theLength0)
    {
        global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<double> aX, a0;
        int aNbX;
        switch (theTable)
        {
            case 0:
                aX = global::Kerf.Engine.Approximation.Advanced.Data.Getmmapgss().gslxjs;
                a0 = global::Kerf.Engine.Approximation.Advanced.Data.Getmmapgss().gsl0js;
                aNbX = 5017;
                break;
            case 1:
                aX = global::Kerf.Engine.Approximation.Advanced.Data.Getmmapgs0().gslxj0;
                a0 = global::Kerf.Engine.Approximation.Advanced.Data.Getmmapgs0().gsl0j0;
                aNbX = 4761;
                break;
            case 2:
                aX = global::Kerf.Engine.Approximation.Advanced.Data.Getmmapgs1().gslxj1;
                a0 = global::Kerf.Engine.Approximation.Advanced.Data.Getmmapgs1().gsl0j1;
                aNbX = 4505;
                break;
            default:
                aX = global::Kerf.Engine.Approximation.Advanced.Data.Getmmapgs2().gslxj2;
                a0 = global::Kerf.Engine.Approximation.Advanced.Data.Getmmapgs2().gsl0j2;
                aNbX = 4249;
                break;
        }
        theFirstX = aX[0];
        theLastX = aX[aNbX - 1];
        theLength0 = a0.AsSpan().Length;
        theFirst0 = a0[0];
        theLast0 = a0[theLength0 - 1];
        theTotal = aX.AsSpan().Length;
    }

    [Fact]
    public void Main_Commons()
    {
        Assert.Equal(3.141592653589793, global::Kerf.Engine.Approximation.Advanced.Data.Getmdnombr().pi);
        Assert.Equal(90.0, global::Kerf.Engine.Approximation.Advanced.Data.Getmdnombr().a90);
        global::Kerf.Engine.Approximation.Advanced.MinombrCommon aNbr = global::Kerf.Engine.Approximation.Advanced.Data.Getminombr();
        Assert.Equal(1001, aNbr.nbr.AsSpan().Length);
        for (int i = 0; i <= 1000; i++)
        {
            Assert.Equal(i, aNbr.nbr[i]);
        }
        ref readonly global::Kerf.Engine.Approximation.Advanced.MaovparCommon aPar = ref global::Kerf.Engine.Approximation.Advanced.Data.Getmaovpar();
        Assert.Equal(1.797693134862315e307, aPar.r8ovr);
        Assert.Equal((float)3.40282347e38, aPar.r4ovr);
        Assert.Equal(17, aPar.r8ncs);
        Assert.Equal((short)-32768, aPar.i2ovn);
        Assert.Equal("SGI_IRIS        ", global::Kerf.Engine.Approximation.Advanced.Data.Getmaovpch().cnmmac);
        Assert.Equal("D_FLOAT ", global::Kerf.Engine.Approximation.Advanced.Data.Getmaovpch().cdcode);
    }
}
